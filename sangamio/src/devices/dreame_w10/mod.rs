//! Dreame Bot W10 (`r2104`) driver — the "over-ava" bridge.
//!
//! Unlike [`crl200s`](super::crl200s), this device does NOT open the serial
//! ports: on the W10 the proprietary navigation daemon `ava` owns `/dev/ttyS4`
//! (MCU) and `/dev/ttyS3` (LDS) exclusively. Instead we consume the read-only
//! serial tap relay (`avatap-relay`, part of `../dreame-w10`) over TCP and
//! decode the MCU stream with the shared [`dreame_w10_proto`] crate. `ava` keeps
//! full control of the robot (charging, cliff/bump safety, docking); this driver
//! only publishes sensors.
//!
//! # Sensor group `sensor_status`
//! - Pose/odometry (0x01): `left_vel`, `right_vel` (I16), `yaw_deg` (F32),
//!   `pose_x_mm`, `pose_y_mm` (F32)
//! - IMU + wheel travel (0x02): `gyro_x/y/z` (F32 deg/s), `accel_x/y/z` (F32 g),
//!   `wheel_dist_left`, `wheel_dist_right` (I8 mm/packet)
//! - Triggers (0x00): `bumper_left/right`, `wheel_float_left/right`,
//!   `is_dock_connected` (Bool)
//! - Battery (0x2b): `battery_voltage` (F32 V), `battery_level` (U8 %),
//!   `is_charging` (Bool)
//!
//! # Config
//! `gd32_port` is repurposed as the relay MCU endpoint `host:port` (defaults to
//! `127.0.0.1:7701` for SangamIO running on the robot). The `W10_MCU_ADDR`
//! environment variable overrides it — use that to point at a remote robot
//! without putting its address in any file. `lidar_port` (the `lds-rx`
//! endpoint) is reserved for when the LDS scan format is decoded.
//!
//! # Commands
//! Read-only: `ava` retains motor/actuator control, so drive/actuator commands
//! are logged and ignored. Teleop is done through Valetudo's manual control.

use crate::config::DeviceConfig;
use crate::core::driver::{DeviceDriver, DriverInitResult};
use crate::core::types::{
    Command, SensorGroupData, SensorValue, StreamSender, create_stream_channel,
};
use crate::error::{Error, Result};
use dreame_w10_proto::{FrameScanner, Msg, parse_body};
use std::collections::HashMap;
use std::io::Read;
use std::net::TcpStream;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};
use std::thread::{self, JoinHandle};
use std::time::Duration;

pub struct DreameW10Driver {
    config: DeviceConfig,
    shutdown: Arc<AtomicBool>,
    reader: Option<JoinHandle<()>>,
}

impl DreameW10Driver {
    pub fn new(config: DeviceConfig) -> Result<Self> {
        Ok(Self {
            config,
            shutdown: Arc::new(AtomicBool::new(false)),
            reader: None,
        })
    }
}

impl DeviceDriver for DreameW10Driver {
    fn initialize(&mut self) -> Result<DriverInitResult> {
        let hardware = self.config.hardware.as_ref().ok_or_else(|| {
            Error::Config("dreame_w10 requires a [device.hardware] section".to_string())
        })?;
        // Relay mcu-rx endpoint: W10_MCU_ADDR env wins (keeps device IPs out of
        // config files), else the repurposed gd32_port from [device.hardware].
        let mcu_addr = std::env::var("W10_MCU_ADDR").unwrap_or_else(|_| hardware.gd32_port.clone());
        log::info!(
            "Initializing Dreame W10 (over-ava): MCU relay at {}",
            mcu_addr
        );

        let mut sensor_data = HashMap::new();
        let mut stream_receivers = HashMap::new();

        let group = Arc::new(Mutex::new(SensorGroupData::new("sensor_status")));
        sensor_data.insert("sensor_status".to_string(), group.clone());
        let (tx, rx) = create_stream_channel();
        stream_receivers.insert("sensor_status".to_string(), rx);

        let shutdown = self.shutdown.clone();
        self.reader = Some(
            thread::Builder::new()
                .name("w10-mcu-reader".to_string())
                .spawn(move || reader_loop(mcu_addr, group, tx, shutdown))
                .map_err(|e| Error::Config(format!("failed to spawn reader: {e}")))?,
        );

        Ok(DriverInitResult {
            sensor_data,
            stream_receivers,
        })
    }

    fn send_command(&mut self, cmd: Command) -> Result<()> {
        if matches!(cmd, Command::Shutdown) {
            self.shutdown.store(true, Ordering::Relaxed);
            return Ok(());
        }
        // Read-only bridge: ava retains motor/actuator control. Motion is issued
        // through Valetudo's HighResolutionManualControl, not from here.
        log::warn!("dreame_w10 is a read-only over-ava bridge; ignoring command {cmd:?}");
        Ok(())
    }
}

impl Drop for DreameW10Driver {
    fn drop(&mut self) {
        self.shutdown.store(true, Ordering::Relaxed);
        if let Some(h) = self.reader.take() {
            let _ = h.join();
        }
    }
}

/// Connect to the relay's MCU stream, decode frames, and update `sensor_status`.
/// Reconnects on any error so it survives relay/ava restarts.
fn reader_loop(
    addr: String,
    group: Arc<Mutex<SensorGroupData>>,
    tx: StreamSender,
    shutdown: Arc<AtomicBool>,
) {
    let mut sc = FrameScanner::new();
    let mut buf = [0u8; 4096];
    while !shutdown.load(Ordering::Relaxed) {
        let mut stream = match TcpStream::connect(&addr) {
            Ok(s) => {
                let _ = s.set_read_timeout(Some(Duration::from_secs(2)));
                log::info!("dreame_w10: connected to MCU relay {addr}");
                s
            }
            Err(e) => {
                log::warn!("dreame_w10: connect {addr} failed: {e}; retrying");
                thread::sleep(Duration::from_millis(500));
                continue;
            }
        };

        loop {
            if shutdown.load(Ordering::Relaxed) {
                return;
            }
            let n = match stream.read(&mut buf) {
                Ok(0) => break, // relay closed
                Ok(n) => n,
                Err(e)
                    if e.kind() == std::io::ErrorKind::WouldBlock
                        || e.kind() == std::io::ErrorKind::TimedOut =>
                {
                    continue;
                }
                Err(e) => {
                    log::warn!("dreame_w10: read error: {e}; reconnecting");
                    break;
                }
            };

            let mut cloned = None;
            if let Ok(mut d) = group.lock() {
                let mut dirty = false;
                for &b in &buf[..n] {
                    if let Some(body) = sc.push(b) {
                        if let Ok((typ, payload)) = parse_body(body) {
                            dirty |= apply(&mut d, &Msg::decode(typ, payload));
                        }
                    }
                }
                if dirty {
                    d.touch();
                    cloned = Some(d.clone());
                }
            }
            // Stream outside the lock; drop if the publisher is behind.
            if let Some(c) = cloned {
                let _ = tx.try_send(c);
            }
        }
    }
}

/// Fold one decoded MCU message into the sensor group. Returns true if it
/// touched any field.
fn apply(d: &mut SensorGroupData, m: &Msg) -> bool {
    match m {
        Msg::Status20ms(s) => {
            d.set("left_vel", SensorValue::I16(s.left_vel));
            d.set("right_vel", SensorValue::I16(s.right_vel));
            d.set("yaw_deg", SensorValue::F32(s.yaw_deg()));
            d.set("pose_x_mm", SensorValue::F32(s.x_mm10 as f32 / 10.0));
            d.set("pose_y_mm", SensorValue::F32(s.y_mm10 as f32 / 10.0));
            true
        }
        Msg::Status10ms(s) => {
            let g = s.gyro_deg_s();
            let a = s.accel_g();
            d.set("gyro_x", SensorValue::F32(g[0]));
            d.set("gyro_y", SensorValue::F32(g[1]));
            d.set("gyro_z", SensorValue::F32(g[2]));
            d.set("accel_x", SensorValue::F32(a[0]));
            d.set("accel_y", SensorValue::F32(a[1]));
            d.set("accel_z", SensorValue::F32(a[2]));
            d.set("wheel_dist_left", SensorValue::I8(s.left_dis_mm));
            d.set("wheel_dist_right", SensorValue::I8(s.right_dis_mm));
            true
        }
        Msg::Triggers(t) => {
            d.set("bumper_left", SensorValue::Bool(t.left_bumper()));
            d.set("bumper_right", SensorValue::Bool(t.right_bumper()));
            d.set("wheel_float_left", SensorValue::Bool(t.left_wheel_floating()));
            d.set("wheel_float_right", SensorValue::Bool(t.right_wheel_floating()));
            d.set("is_dock_connected", SensorValue::Bool(t.dock_sta()));
            true
        }
        Msg::Battery(b) => {
            d.set("battery_voltage", SensorValue::F32(b.voltage_v()));
            let pct = b.soc_percent().clamp(0.0, 255.0) as u8;
            d.set("battery_level", SensorValue::U8(pct));
            d.set("is_charging", SensorValue::Bool(b.charge_voltage_mv > 1000));
            true
        }
        _ => false,
    }
}
