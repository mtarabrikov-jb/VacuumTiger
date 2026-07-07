//! Dreame Bot W10 (`r2104`) driver — two modes, chosen by the endpoint form:
//!
//! - **over-ava** (`gd32_port` = `host:port`): the W10's proprietary `ava` daemon
//!   owns `/dev/ttyS4` (MCU) + `/dev/ttyS3` (LDS), so we can't open them. Instead
//!   we read the tap relay (`avatap-relay`, `../dreame-w10`) over TCP — read-only
//!   sensors; `ava` keeps motor/actuator control (drive it via Valetudo).
//! - **direct** (`gd32_port` = `/dev/ttyS4`): a full replacement — with `ava`
//!   stopped, this driver opens the ports itself and drives the MCU (MotorCtrl at
//!   ~50 Hz, SetLED/SetCleaning heartbeats, `0x0f` ping→pong), so SangamIO
//!   commands (`drive`, `vacuum`, `main_brush`, `side_brush`, `water_pump`,
//!   `lidar`) actually actuate the robot. Own watchdog + speed clamp + cliff/bump
//!   hazard gate. See `../dreame-w10/docs/MCUD.md`.
//!
//! Both decode the MCU stream with the shared [`dreame_w10_proto`] crate.
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
//! # Sensor group `lidar`
//! - `scan`: `PointCloud2D` of `(angle_rad, distance_m, quality)`, robot-centered
//!   (each raw sensor angle goes through `frame_transforms.lidar` then
//!   `lidar_mounting.transform_to_robot_center`, same as the Revo/Delta drivers),
//!   one arc sweep per publish. **Caveats:** the ttyS3 feed is a fixed ~126 deg
//!   rear arc (~226-352 deg), not a full circle; and the transform values in
//!   `dreame_w10*.toml` are uncalibrated starting guesses (calibrate on-robot).
//!   See `../dreame-w10/docs/LDS_PROTOCOL.md`.
//! - `speed_raw`: `U16` turret speed (raw units, not RPM).
//!
//! # Config
//! `gd32_port` / `lidar_port` are the endpoints; `W10_MCU_ADDR` / `W10_LDS_ADDR`
//! env vars override them (keeps device paths/IPs out of committed config). A
//! `host:port` (e.g. `127.0.0.1:7701`) selects over-ava; a `/dev/...` path (e.g.
//! `/dev/ttyS4`, `/dev/ttyS3`) selects direct mode.
//!
//! # Commands
//! **Direct mode:** `drive` (`Configure{linear m/s, angular rad/s}`), the speed
//! components `vacuum`/`main_brush`/`side_brush`/`water_pump` (`Enable`/`Disable`/
//! `Configure{speed 0-100}`) and `lidar` (`Enable`/`Disable`) actuate the robot.
//! **Over-ava mode:** read-only — commands are logged and ignored (`ava` owns the
//! motors; teleop via Valetudo).

use crate::config::{AffineTransform1D, DeviceConfig, LidarMountingConfig};
use crate::core::driver::{DeviceDriver, DriverInitResult};
use crate::core::types::{
    Command, ComponentAction, SensorGroupData, SensorValue, StreamSender, create_stream_channel,
};
use crate::error::{Error, Result};
use dreame_w10_proto::lds::LdsScanner;
use dreame_w10_proto::{FrameScanner, Msg, encode_frame, encode_motor_ctrl, parse_body};
use serialport::SerialPort;
use std::collections::HashMap;
use std::io::{Read, Write};
use std::net::TcpStream;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};
use std::thread::{self, JoinHandle};
use std::time::{Duration, Instant};

type Port = Box<dyn SerialPort>;

/// Shared drive/actuator command state for **direct** mode.
struct CmdState {
    linear_mm_s: f32,
    rot_rad_s: f32,
    last_cmd: Instant,
    /// SetCleaning (0x01) bytes: [0]=side, [1]=main, [2]=fan, [3]=pump, [4]=mode.
    setcleaning: [u8; 6],
    lidar_on: bool,
}
impl CmdState {
    fn new() -> Self {
        Self {
            linear_mm_s: 0.0,
            rot_rad_s: 0.0,
            last_cmd: Instant::now(),
            setcleaning: [0, 1, 0, 0, 0, 0],
            lidar_on: false,
        }
    }
}

const MAX_LINEAR_MM_S: f32 = 150.0;
const MAX_ROT_RAD_S: f32 = 1.5;
const WATCHDOG: Duration = Duration::from_millis(500);

pub struct DreameW10Driver {
    config: DeviceConfig,
    shutdown: Arc<AtomicBool>,
    reader: Option<JoinHandle<()>>,
    lds_reader: Option<JoinHandle<()>>,
    tx_thread: Option<JoinHandle<()>>,
    /// Direct-mode command state (`None` in over-ava read-only mode).
    cmd: Option<Arc<Mutex<CmdState>>>,
    hazard: Arc<AtomicBool>,
}

impl DreameW10Driver {
    pub fn new(config: DeviceConfig) -> Result<Self> {
        Ok(Self {
            config,
            shutdown: Arc::new(AtomicBool::new(false)),
            reader: None,
            lds_reader: None,
            tx_thread: None,
            cmd: None,
            hazard: Arc::new(AtomicBool::new(false)),
        })
    }
}

impl DeviceDriver for DreameW10Driver {
    fn initialize(&mut self) -> Result<DriverInitResult> {
        let hardware = self.config.hardware.as_ref().ok_or_else(|| {
            Error::Config("dreame_w10 requires a [device.hardware] section".to_string())
        })?;
        // Endpoints: env wins (keeps device paths/IPs out of config). A `/dev/...`
        // path selects **direct** mode (this driver opens the port and drives the
        // MCU); a `host:port` selects **over-ava** mode (read from the tap relay).
        let mcu_addr = std::env::var("W10_MCU_ADDR").unwrap_or_else(|_| hardware.gd32_port.clone());
        let lds_addr = std::env::var("W10_LDS_ADDR").unwrap_or_else(|_| hardware.lidar_port.clone());
        let direct = mcu_addr.starts_with("/dev/");
        // Lidar frame -> robot frame, applied per point before publishing `scan`
        // (same as the Revo/Delta drivers, so dhruva-slam gets robot-centered
        // scans). Values come from the config; see dreame_w10*.toml.
        let lidar_tf = hardware.frame_transforms.lidar;
        let lidar_mount = hardware.lidar_mounting.clone();

        let mut sensor_data = HashMap::new();
        let mut stream_receivers = HashMap::new();

        let group = Arc::new(Mutex::new(SensorGroupData::new("sensor_status")));
        sensor_data.insert("sensor_status".to_string(), group.clone());
        let (tx, rx) = create_stream_channel();
        stream_receivers.insert("sensor_status".to_string(), rx);

        let lidar = Arc::new(Mutex::new(SensorGroupData::new("lidar")));
        sensor_data.insert("lidar".to_string(), lidar.clone());

        if direct {
            log::info!("Dreame W10 (direct): driving MCU {mcu_addr}, LDS {lds_addr}");
            let mcu = serialport::new(&mcu_addr, 115200)
                .timeout(Duration::from_millis(100))
                .open()
                .map_err(|e| Error::Config(format!("open {mcu_addr}: {e}")))?;
            let mcu_rd = mcu
                .try_clone()
                .map_err(|e| Error::Config(format!("clone {mcu_addr}: {e}")))?;
            let w = Arc::new(Mutex::new(mcu));
            let cmd = Arc::new(Mutex::new(CmdState::new()));
            self.cmd = Some(cmd.clone());

            let (rw, rg, rh, rs) = (w.clone(), group, self.hazard.clone(), self.shutdown.clone());
            self.reader = Some(
                thread::Builder::new()
                    .name("w10-mcu-rx".to_string())
                    .spawn(move || serial_rx_loop(mcu_rd, rw, rg, tx, rh, rs))
                    .map_err(|e| Error::Config(format!("spawn rx: {e}")))?,
            );
            let (th, ts) = (self.hazard.clone(), self.shutdown.clone());
            self.tx_thread = Some(
                thread::Builder::new()
                    .name("w10-mcu-tx".to_string())
                    .spawn(move || serial_tx_loop(w, cmd, th, ts))
                    .map_err(|e| Error::Config(format!("spawn tx: {e}")))?,
            );
            match serialport::new(&lds_addr, 230400)
                .timeout(Duration::from_millis(100))
                .open()
            {
                Ok(lds_rd) => {
                    let (lg, ls) = (lidar, self.shutdown.clone());
                    self.lds_reader = Some(
                        thread::Builder::new()
                            .name("w10-lds-rx".to_string())
                            .spawn(move || serial_lds_loop(lds_rd, lg, ls, lidar_tf, lidar_mount))
                            .map_err(|e| Error::Config(format!("spawn lds: {e}")))?,
                    );
                }
                Err(e) => log::warn!("Dreame W10: LDS {lds_addr} open failed ({e}); LDS disabled"),
            }
        } else {
            log::info!("Dreame W10 (over-ava): MCU relay {mcu_addr}, LDS relay {lds_addr}");
            let shutdown = self.shutdown.clone();
            self.reader = Some(
                thread::Builder::new()
                    .name("w10-mcu-reader".to_string())
                    .spawn(move || reader_loop(mcu_addr, group, tx, shutdown))
                    .map_err(|e| Error::Config(format!("failed to spawn reader: {e}")))?,
            );
            let lds_shutdown = self.shutdown.clone();
            self.lds_reader = Some(
                thread::Builder::new()
                    .name("w10-lds-reader".to_string())
                    .spawn(move || lds_reader_loop(lds_addr, lidar, lds_shutdown, lidar_tf, lidar_mount))
                    .map_err(|e| Error::Config(format!("failed to spawn lds reader: {e}")))?,
            );
        }

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
        // Only direct mode can drive; over-ava is a read-only mirror (ava owns the
        // motors — teleop it through Valetudo instead).
        let Some(state) = self.cmd.as_ref() else {
            log::warn!("dreame_w10 (over-ava) is read-only; ignoring {cmd:?}");
            return Ok(());
        };
        if let Command::ComponentControl { id, action } = cmd {
            apply_command(state, &id, &action);
        }
        Ok(())
    }
}

impl Drop for DreameW10Driver {
    fn drop(&mut self) {
        self.shutdown.store(true, Ordering::Relaxed);
        for h in [self.reader.take(), self.lds_reader.take(), self.tx_thread.take()]
            .into_iter()
            .flatten()
        {
            let _ = h.join();
        }
    }
}

// ---------------------------------------------------------------------------
// Direct mode: this driver opens /dev/ttyS4 + /dev/ttyS3 and drives the MCU
// itself (ava stopped). Reuses `apply()` / `publish_scan()` from over-ava mode.
// ---------------------------------------------------------------------------

/// Encode a frame and write it under the shared serial write lock.
fn send_frame(w: &Mutex<Port>, typ: u8, payload: &[u8]) {
    let mut buf = [0u8; 64];
    if let Some(n) = encode_frame(typ, payload, &mut buf) {
        if let Ok(mut p) = w.lock() {
            let _ = p.write_all(&buf[..n]);
        }
    }
}

/// Map a speed-component action to a level byte (Enable=100, Disable=0, Configure=speed).
fn speed_of(action: &ComponentAction) -> Option<u8> {
    match action {
        ComponentAction::Enable { .. } => Some(100),
        ComponentAction::Disable { .. } => Some(0),
        ComponentAction::Configure { config } => config.get("speed").and_then(|v| match v {
            SensorValue::U8(s) => Some(*s),
            SensorValue::U32(s) => Some(*s as u8),
            _ => None,
        }),
        _ => None,
    }
}

/// Fold a SangamIO `ComponentControl` into the direct-mode command state.
fn apply_command(state: &Arc<Mutex<CmdState>>, id: &str, action: &ComponentAction) {
    let Ok(mut c) = state.lock() else { return };
    match id {
        "drive" => match action {
            ComponentAction::Configure { config } => {
                if let (Some(SensorValue::F32(lin)), Some(SensorValue::F32(ang))) =
                    (config.get("linear"), config.get("angular"))
                {
                    c.linear_mm_s = lin * 1000.0; // m/s -> mm/s
                    c.rot_rad_s = *ang;
                    c.last_cmd = Instant::now();
                }
            }
            ComponentAction::Disable { .. } | ComponentAction::Reset { .. } => {
                c.linear_mm_s = 0.0;
                c.rot_rad_s = 0.0;
                c.last_cmd = Instant::now();
            }
            _ => {}
        },
        "side_brush" => {
            if let Some(s) = speed_of(action) {
                c.setcleaning[0] = s;
            }
        }
        "main_brush" => {
            if let Some(s) = speed_of(action) {
                c.setcleaning[1] = s;
            }
        }
        "vacuum" => {
            if let Some(s) = speed_of(action) {
                c.setcleaning[2] = s;
            }
        }
        "water_pump" => {
            if let Some(s) = speed_of(action) {
                c.setcleaning[3] = s;
            }
        }
        "lidar" => match action {
            ComponentAction::Enable { .. } => c.lidar_on = true,
            ComponentAction::Disable { .. } => c.lidar_on = false,
            _ => {}
        },
        other => log::debug!("dreame_w10: unhandled component '{other}'"),
    }
}

/// Read the MCU serial stream: decode telemetry into `sensor_status`, answer
/// `0x0f` pings with a pong, and track the cliff/bumper hazard.
fn serial_rx_loop(
    mut rd: Port,
    w: Arc<Mutex<Port>>,
    group: Arc<Mutex<SensorGroupData>>,
    tx: StreamSender,
    hazard: Arc<AtomicBool>,
    shutdown: Arc<AtomicBool>,
) {
    let mut sc = FrameScanner::new();
    let mut buf = [0u8; 4096];
    log::info!("dreame_w10: driving MCU (MotorCtrl 50Hz + pong + heartbeats)");
    while !shutdown.load(Ordering::Relaxed) {
        let n = match rd.read(&mut buf) {
            Ok(0) => continue,
            Ok(n) => n,
            Err(e)
                if e.kind() == std::io::ErrorKind::TimedOut
                    || e.kind() == std::io::ErrorKind::WouldBlock =>
            {
                continue
            }
            Err(_) => {
                thread::sleep(Duration::from_millis(5));
                continue;
            }
        };
        let mut cloned = None;
        if let Ok(mut d) = group.lock() {
            let mut dirty = false;
            for &b in &buf[..n] {
                if let Some(body) = sc.push(b) {
                    if let Ok((typ, payload)) = parse_body(body) {
                        if typ == 0x0f && payload.len() >= 4 {
                            send_frame(&w, 0x0f, &payload[..4]); // pong echoes the ping ts
                        }
                        if typ == 0x00 && payload.len() >= 2 {
                            hazard.store(
                                payload[0] & 0xF0 != 0 || payload[1] != 0,
                                Ordering::Relaxed,
                            );
                        }
                        dirty |= apply(&mut d, &Msg::decode(typ, payload));
                    }
                }
            }
            if dirty {
                d.touch();
                cloned = Some(d.clone());
            }
        }
        if let Some(c) = cloned {
            let _ = tx.try_send(c);
        }
    }
}

/// Drive the MCU: MotorCtrl at ~50 Hz (from the command state, gated by watchdog /
/// clamp / hazard) plus the periodic SetLED / SetCleaning / lidar frames.
fn serial_tx_loop(
    w: Arc<Mutex<Port>>,
    cmd: Arc<Mutex<CmdState>>,
    hazard: Arc<AtomicBool>,
    shutdown: Arc<AtomicBool>,
) {
    let mut tick: u64 = 0;
    let mut mbuf = [0u8; 64];
    while !shutdown.load(Ordering::Relaxed) {
        let (mut lin, rot, mut sc, lidar) = {
            let c = cmd.lock().unwrap_or_else(|e| e.into_inner());
            let (l, r) = if c.last_cmd.elapsed() < WATCHDOG {
                (c.linear_mm_s, c.rot_rad_s)
            } else {
                (0.0, 0.0)
            };
            (l, r, c.setcleaning, c.lidar_on)
        };
        lin = lin.clamp(-MAX_LINEAR_MM_S, MAX_LINEAR_MM_S);
        let rot = rot.clamp(-MAX_ROT_RAD_S, MAX_ROT_RAD_S);
        if hazard.load(Ordering::Relaxed) && lin != 0.0 {
            lin = 0.0; // never translate into a detected cliff/bump
        }
        if let Some(n) = encode_motor_ctrl(1, lin, rot, &mut mbuf) {
            if let Ok(mut p) = w.lock() {
                let _ = p.write_all(&mbuf[..n]);
            }
        }
        sc[4] = if sc[2] > 0 { 0x03 } else { 0x00 }; // vacuum mode when the fan runs
        if tick % 25 == 5 {
            send_frame(&w, 0x02, &[0x21]); // SetLED heartbeat
        }
        if tick % 25 == 12 {
            send_frame(&w, 0x01, &sc); // SetCleaning (actuator levels)
        }
        if tick % 50 == 20 {
            send_frame(&w, 0x14, if lidar { &[0x04, 0x01] } else { &[0x04, 0x00] });
        }
        if tick % 50 == 30 {
            let p: &[u8] = if lidar {
                &[0x14, 0, 0, 0, 0, 0, 0, 0x04]
            } else {
                &[0x64, 0, 0, 0, 0, 0, 0, 0x04]
            };
            send_frame(&w, 0x26, p);
        }
        if lidar && tick % 200 == 40 {
            send_frame(&w, 0x1d, &[0x05, 0x01]); // lidar enable re-pulse
        }
        tick = tick.wrapping_add(1);
        thread::sleep(Duration::from_millis(20));
    }
    if let Some(n) = encode_motor_ctrl(1, 0.0, 0.0, &mut mbuf) {
        if let Ok(mut p) = w.lock() {
            let _ = p.write_all(&mbuf[..n]);
        }
    }
}

/// Read the LDS serial stream and publish arc sweeps to the `lidar` group
/// (same accumulate/publish logic as the over-ava reader).
fn serial_lds_loop(
    mut rd: Port,
    group: Arc<Mutex<SensorGroupData>>,
    shutdown: Arc<AtomicBool>,
    tf: AffineTransform1D,
    mount: LidarMountingConfig,
) {
    use std::f32::consts::TAU;
    const MIN_POINTS: usize = 16;
    const SWEEP_RESET: u16 = 4000;
    const MAX_SWEEP: Duration = Duration::from_secs(1);
    let mut sc = LdsScanner::new();
    let mut buf = [0u8; 4096];
    let mut points: Vec<(f32, f32, u8)> = Vec::with_capacity(512);
    let mut last_fsa: Option<u16> = None;
    let mut sweep_start = Instant::now();
    while !shutdown.load(Ordering::Relaxed) {
        let n = match rd.read(&mut buf) {
            Ok(0) => continue,
            Ok(n) => n,
            Err(e)
                if e.kind() == std::io::ErrorKind::TimedOut
                    || e.kind() == std::io::ErrorKind::WouldBlock =>
            {
                continue
            }
            Err(_) => {
                thread::sleep(Duration::from_millis(5));
                continue;
            }
        };
        for &b in &buf[..n] {
            let Some(f) = sc.push(b) else { continue };
            if let Some(prev) = last_fsa {
                if f.fsa < prev && prev - f.fsa > SWEEP_RESET {
                    if points.len() >= MIN_POINTS {
                        publish_scan(&group, &points, f.speed);
                    }
                    points.clear();
                    sweep_start = Instant::now();
                }
            }
            for k in 0..dreame_w10_proto::lds::LDS_SAMPLES {
                let s = f.samples[k];
                if !s.valid {
                    continue;
                }
                let raw = f.sample_angle(k) as f32 / 65536.0 * TAU;
                let (angle, dist_m) =
                    mount.transform_to_robot_center(tf.apply(raw), s.dist_mm as f32 / 1000.0);
                points.push((angle, dist_m, s.quality.max(1)));
            }
            last_fsa = Some(f.fsa);
            if sweep_start.elapsed() > MAX_SWEEP && points.len() >= MIN_POINTS {
                publish_scan(&group, &points, f.speed);
                points.clear();
                sweep_start = Instant::now();
            }
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

/// LDS reader: connect to the relay's `lds-rx` stream, decode packets, accumulate
/// one arc sweep of points, and publish it to the `lidar` group. The W10's tapped
/// feed is a fixed ~126 deg rear arc, so a "sweep" is one pass of that arc: `fsa`
/// climbs, then jumps back down to the arc start — that reset is the publish
/// boundary. Reconnects on any error so it survives relay/ava restarts.
fn lds_reader_loop(
    addr: String,
    group: Arc<Mutex<SensorGroupData>>,
    shutdown: Arc<AtomicBool>,
    tf: AffineTransform1D,
    mount: LidarMountingConfig,
) {
    use std::f32::consts::TAU;
    /// Minimum valid points before a sweep is worth publishing.
    const MIN_POINTS: usize = 16;
    /// A `fsa` drop larger than this (u16 units, ~22 deg) marks a new sweep.
    const SWEEP_RESET: u16 = 4000;
    /// Publish anyway after this long if no clean reset was seen.
    const MAX_SWEEP: Duration = Duration::from_secs(1);

    let mut sc = LdsScanner::new();
    let mut buf = [0u8; 4096];
    let mut points: Vec<(f32, f32, u8)> = Vec::with_capacity(512);

    while !shutdown.load(Ordering::Relaxed) {
        let mut stream = match TcpStream::connect(&addr) {
            Ok(s) => {
                let _ = s.set_read_timeout(Some(Duration::from_secs(2)));
                log::info!("dreame_w10: connected to LDS relay {addr}");
                s
            }
            Err(e) => {
                log::warn!("dreame_w10: connect {addr} failed: {e}; retrying");
                thread::sleep(Duration::from_millis(500));
                continue;
            }
        };
        // Fresh per-connection sweep state; reuse the points allocation.
        points.clear();
        let mut last_fsa: Option<u16> = None;
        let mut sweep_start = Instant::now();

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
                    log::warn!("dreame_w10: LDS read error: {e}; reconnecting");
                    break;
                }
            };
            for &b in &buf[..n] {
                let Some(f) = sc.push(b) else { continue };
                // Sweep boundary: fsa jumped back to the arc start.
                if let Some(prev) = last_fsa {
                    if f.fsa < prev && prev - f.fsa > SWEEP_RESET {
                        if points.len() >= MIN_POINTS {
                            publish_scan(&group, &points, f.speed);
                        }
                        points.clear();
                        sweep_start = Instant::now();
                    }
                }
                for k in 0..dreame_w10_proto::lds::LDS_SAMPLES {
                    let s = f.samples[k];
                    if !s.valid {
                        continue;
                    }
                    let raw = f.sample_angle(k) as f32 / 65536.0 * TAU;
                    let (angle, dist_m) =
                        mount.transform_to_robot_center(tf.apply(raw), s.dist_mm as f32 / 1000.0);
                    points.push((angle, dist_m, s.quality.max(1)));
                }
                last_fsa = Some(f.fsa);
                // Safety valve: never let a sweep grow unbounded if the reset is missed.
                if sweep_start.elapsed() > MAX_SWEEP && points.len() >= MIN_POINTS {
                    publish_scan(&group, &points, f.speed);
                    points.clear();
                    sweep_start = Instant::now();
                }
            }
        }
    }
}

/// Publish one accumulated arc sweep to the `lidar` group as a `PointCloud2D`.
fn publish_scan(group: &Arc<Mutex<SensorGroupData>>, points: &[(f32, f32, u8)], speed: u16) {
    if let Ok(mut d) = group.lock() {
        d.touch();
        d.set("scan", SensorValue::PointCloud2D(points.to_vec()));
        d.set("speed_raw", SensorValue::U16(speed));
    }
}
