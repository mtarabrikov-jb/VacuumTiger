//! `w10-decode` — connect to `avatap-relay` and decode (or hexdump) a channel.
//!
//! Usage:
//!   w10-decode <host> [--mcu | --mcu-tx | --lds | --lds-tx] [--port N] [--raw]
//!
//! Defaults to `--mcu` (port 7701). `--mcu`/`--mcu-tx` decode the framed MCU
//! protocol and print a live summary (odometry / IMU / battery / triggers plus a
//! per-type frame histogram, so you can confirm the W10 field layouts against
//! ground truth). `--lds`/`--lds-tx` decode the LDS scan stream (turret speed,
//! covered angular sector, valid/invalid points, distance range). Add `--raw` to
//! hexdump any channel instead.

use dreame_w10_proto::lds::LdsScanner;
use dreame_w10_proto::{parse_body, FrameError, FrameScanner, Msg};
use std::collections::BTreeMap;
use std::io::Read;
use std::net::TcpStream;
use std::time::{Duration, Instant};

fn main() {
    let args: Vec<String> = std::env::args().skip(1).collect();
    // Host: positional arg, else the W10_ROBOT env var, else loopback. Keeps
    // device addresses out of the source.
    let host = args
        .iter()
        .find(|a| !a.starts_with("--"))
        .cloned()
        .or_else(|| std::env::var("W10_ROBOT").ok())
        .unwrap_or_else(|| "127.0.0.1".to_string());
    let has = |f: &str| args.iter().any(|a| a == f);
    let (mut port, decode) = if has("--mcu-tx") {
        (7703u16, true)
    } else if has("--lds") {
        (7702, false)
    } else if has("--lds-tx") {
        (7704, false)
    } else {
        (7701, true) // --mcu default
    };
    if let Some(i) = args.iter().position(|a| a == "--port") {
        if let Some(p) = args.get(i + 1).and_then(|s| s.parse().ok()) {
            port = p;
        }
    }
    let raw = has("--raw");
    let watch = has("--watch");
    let is_lds = has("--lds") || has("--lds-tx");

    let addr = format!("{host}:{port}");
    let kind = if watch {
        "watch"
    } else if raw {
        "hexdump"
    } else if is_lds {
        "lds decode"
    } else if decode {
        "decode"
    } else {
        "hexdump"
    };
    eprintln!("w10-decode: connecting to {addr} ({kind})");
    let mut stream = match TcpStream::connect(&addr) {
        Ok(s) => s,
        Err(e) => {
            eprintln!("w10-decode: connect {addr} failed: {e}");
            std::process::exit(1);
        }
    };

    if let Some(i) = args.iter().position(|a| a == "--log") {
        let typ = args
            .get(i + 1)
            .and_then(|s| u8::from_str_radix(s.trim_start_matches("0x"), 16).ok())
            .unwrap_or(0x01);
        log_frames(&mut stream, typ);
    } else if watch {
        watch_offsets(&mut stream);
    } else if raw {
        hexdump(&mut stream);
    } else if is_lds {
        decode_lds(&mut stream);
    } else if decode {
        decode_mcu(&mut stream);
    } else {
        hexdump(&mut stream);
    }
}

/// Decode the LDS scan stream and print a live summary: packet rate, turret
/// speed, the angular sector actually covered (under manual control `ava` only
/// sweeps a ~126 deg rear arc), valid/invalid point counts, and the distance
/// range. Use `--raw` to hexdump the same stream instead.
fn decode_lds(stream: &mut TcpStream) {
    let mut sc = LdsScanner::new();
    let mut buf = [0u8; 4096];
    let mut last_print = Instant::now();
    // Accumulated over the print interval.
    let (mut frames, mut valid, mut invalid) = (0u64, 0u64, 0u64);
    let (mut ang_min, mut ang_max) = (f32::MAX, f32::MIN);
    let (mut dmin, mut dmax) = (u16::MAX, 0u16);
    let mut dsum: u64 = 0;
    let mut speed = 0u16;
    loop {
        let n = match stream.read(&mut buf) {
            Ok(0) => {
                eprintln!("w10-decode: relay closed");
                return;
            }
            Ok(n) => n,
            Err(e) => {
                eprintln!("w10-decode: read error: {e}");
                return;
            }
        };
        for &b in &buf[..n] {
            if let Some(f) = sc.push(b) {
                frames += 1;
                speed = f.speed;
                for k in 0..dreame_w10_proto::lds::LDS_SAMPLES {
                    let s = f.samples[k];
                    if s.valid {
                        valid += 1;
                        dmin = dmin.min(s.dist_mm);
                        dmax = dmax.max(s.dist_mm);
                        dsum += s.dist_mm as u64;
                        let a = f.sample_angle_deg(k);
                        ang_min = ang_min.min(a);
                        ang_max = ang_max.max(a);
                    } else {
                        invalid += 1;
                    }
                }
            }
        }
        if last_print.elapsed() >= Duration::from_millis(500) {
            last_print = Instant::now();
            if frames == 0 {
                println!("---- lds: no frames (turret idle? start navigation / manual control)");
            } else {
                let mean = if valid > 0 { dsum / valid } else { 0 };
                println!(
                    "---- lds: {frames} pkt  speed={speed}  sector={ang_min:.0}..{ang_max:.0}deg  pts valid={valid} invalid={invalid}  dist {dmin}..{dmax}mm mean={mean}mm",
                );
            }
            frames = 0;
            valid = 0;
            invalid = 0;
            ang_min = f32::MAX;
            ang_max = f32::MIN;
            dmin = u16::MAX;
            dmax = 0;
            dsum = 0;
        }
    }
}

/// Dump every frame of one type at full rate: `<ms_since_start> <payload hex>`.
/// Drive a single clean motion (e.g. slow forward) and watch which i16 ramps
/// 0 -> v -> 0 to pin down leftVel/rightVel vs. position/current fields.
fn log_frames(stream: &mut TcpStream, typ: u8) {
    let start = Instant::now();
    let mut sc = FrameScanner::new();
    let mut buf = [0u8; 4096];
    eprintln!("w10-decode: logging every 0x{typ:02x} frame (ms  payload-hex)");
    loop {
        let n = match stream.read(&mut buf) {
            Ok(0) => return,
            Ok(n) => n,
            Err(_) => return,
        };
        for &b in &buf[..n] {
            if let Some(body) = sc.push(b) {
                if let Ok((t, p)) = parse_body(body) {
                    if t == typ {
                        let ms = start.elapsed().as_millis();
                        let hex: Vec<String> = p.iter().map(|b| format!("{b:02x}")).collect();
                        println!("{ms:6}  {}", hex.join(" "));
                    }
                }
            }
        }
    }
}

/// Per-byte volatility of each MCU message type: which payload offsets change
/// (the moving fields — wheel velocity / odometry) vs. stay constant. Spin a
/// wheel by hand and watch the `span` row light up under the relevant bytes.
fn watch_offsets(stream: &mut TcpStream) {
    struct PerType {
        n: u64,
        min: Vec<u8>,
        max: Vec<u8>,
        last: Vec<u8>,
    }
    let mut sc = FrameScanner::new();
    let mut buf = [0u8; 4096];
    let mut map: BTreeMap<u8, PerType> = BTreeMap::new();
    let mut last_print = Instant::now();
    // Types worth showing a per-offset breakdown for.
    let show: [u8; 4] = [0x01, 0x02, 0x03, 0x00];

    loop {
        let n = match stream.read(&mut buf) {
            Ok(0) => break,
            Ok(n) => n,
            Err(e) => {
                eprintln!("w10-decode: read error: {e}");
                break;
            }
        };
        for &b in &buf[..n] {
            if let Some(body) = sc.push(b) {
                if let Ok((typ, p)) = parse_body(body) {
                    let e = map.entry(typ).or_insert_with(|| PerType {
                        n: 0,
                        min: vec![0xff; p.len()],
                        max: vec![0x00; p.len()],
                        last: vec![0x00; p.len()],
                    });
                    if p.len() > e.min.len() {
                        e.min.resize(p.len(), 0xff);
                        e.max.resize(p.len(), 0x00);
                        e.last.resize(p.len(), 0x00);
                    }
                    for (i, &x) in p.iter().enumerate() {
                        if x < e.min[i] { e.min[i] = x; }
                        if x > e.max[i] { e.max[i] = x; }
                        e.last[i] = x;
                    }
                    e.n += 1;
                }
            }
        }
        if last_print.elapsed() >= Duration::from_millis(1000) {
            last_print = Instant::now();
            println!("======== per-offset volatility (span = max-min; ^ = changed) ========");
            for typ in show {
                if let Some(e) = map.get(&typ) {
                    let len = e.last.len();
                    let ruler: String = (0..len).map(|i| format!("{:02} ", i)).collect();
                    let last: String = e.last.iter().map(|b| format!("{:02x} ", b)).collect();
                    let span: String = (0..len)
                        .map(|i| {
                            let s = e.max[i].wrapping_sub(e.min[i]);
                            if s == 0 { "   ".to_string() } else { format!("{:02x} ", s) }
                        })
                        .collect();
                    println!("0x{typ:02x} len={len} n={}", e.n);
                    println!("  off : {ruler}");
                    println!("  last: {last}");
                    println!("  span: {span}");
                    // i16 LE candidates at offsets that changed
                    let mut cand = String::new();
                    let mut i = 0;
                    while i + 1 < len {
                        if e.max[i].wrapping_sub(e.min[i]) != 0 || e.max[i + 1].wrapping_sub(e.min[i + 1]) != 0 {
                            let v = i16::from_le_bytes([e.last[i], e.last[i + 1]]);
                            cand.push_str(&format!("[{i}..{}]i16={v} ", i + 1));
                        }
                        i += 2;
                    }
                    if !cand.is_empty() {
                        println!("  i16 : {cand}");
                    }
                }
            }
        }
    }
}

fn decode_mcu(stream: &mut TcpStream) {
    let mut sc = FrameScanner::new();
    let mut buf = [0u8; 4096];
    let mut counts: BTreeMap<u8, u64> = BTreeMap::new();
    let mut crc_errors: u64 = 0;
    let mut last = Latest::default();
    let mut last_print = Instant::now();

    loop {
        let n = match stream.read(&mut buf) {
            Ok(0) => {
                eprintln!("w10-decode: relay closed");
                return;
            }
            Ok(n) => n,
            Err(e) => {
                eprintln!("w10-decode: read error: {e}");
                return;
            }
        };
        for &b in &buf[..n] {
            if let Some(body) = sc.push(b) {
                match parse_body(body) {
                    Ok((typ, payload)) => {
                        *counts.entry(typ).or_default() += 1;
                        last.update(&Msg::decode(typ, payload));
                    }
                    Err(FrameError::Crc { .. }) => crc_errors += 1,
                    Err(FrameError::TooShort) => {}
                }
            }
        }
        if last_print.elapsed() >= Duration::from_millis(400) {
            last_print = Instant::now();
            print_summary(&counts, crc_errors, &last);
        }
    }
}

#[derive(Default)]
struct Latest {
    s20: Option<dreame_w10_proto::Status20ms>,
    s10: Option<dreame_w10_proto::Status10ms>,
    s100: Option<dreame_w10_proto::Status100ms>,
    batt: Option<dreame_w10_proto::Battery>,
    trig: Option<dreame_w10_proto::Triggers>,
}
impl Latest {
    fn update(&mut self, m: &Msg) {
        match m {
            Msg::Status20ms(s) => self.s20 = Some(*s),
            Msg::Status10ms(s) => self.s10 = Some(*s),
            Msg::Status100ms(s) => self.s100 = Some(*s),
            Msg::Battery(b) => self.batt = Some(*b),
            Msg::Triggers(t) => self.trig = Some(*t),
            _ => {}
        }
    }
}

fn print_summary(counts: &BTreeMap<u8, u64>, crc_errors: u64, l: &Latest) {
    let mut hist = String::new();
    for (typ, c) in counts {
        hist.push_str(&format!("0x{typ:02x}:{c} "));
    }
    println!("---- frames: {hist} crc_err:{crc_errors}");
    if let Some(s) = l.s20 {
        println!(
            "  odom : yaw={:+.2}deg  Lvel={:+} Rvel={:+}  x={:.1}mm y={:.1}mm  roller_i={} side_i={}",
            s.yaw_deg(),
            s.left_vel,
            s.right_vel,
            s.x_mm10 as f32 / 10.0,
            s.y_mm10 as f32 / 10.0,
            s.roller_current,
            s.sidebrush_current,
        );
    }
    if let Some(s) = l.s10 {
        let g = s.gyro_deg_s();
        let a = s.accel_g();
        println!(
            "  imu  : gyro=[{:+.1},{:+.1},{:+.1}]deg/s accel=[{:+.2},{:+.2},{:+.2}]g  dL={} dR={}",
            g[0], g[1], g[2], a[0], a[1], a[2], s.left_dis_mm, s.right_dis_mm
        );
    }
    if let Some(s) = l.s100 {
        println!(
            "  tilt : pitch={:.1} roll={:.1}  Icur L={} R={}  dust_missing={} water={} carpet={}",
            s.pitch_ddeg as f32 / 10.0,
            s.roll_ddeg as f32 / 10.0,
            s.left_current,
            s.right_current,
            s.dust_container_missing(),
            s.water_tank_installed(),
            s.carpet_state(),
        );
    }
    if let Some(b) = l.batt {
        println!(
            "  batt : {:.2}V {}mA {:.1}C soc={:.1}%  chg={:.2}V",
            b.voltage_v(),
            b.current_ma,
            b.temperature_ddeg as f32 / 10.0,
            b.soc_percent(),
            b.charge_voltage_mv as f32 / 1000.0,
        );
    }
    if let Some(t) = l.trig {
        println!(
            "  trig : dock={} bumpL={} bumpR={} floatL={} floatR={} irDock=[{},{},{},{}]  err[vel L{} R{} imu{} charge{} lidar{}]",
            t.dock_sta(),
            t.left_bumper(),
            t.right_bumper(),
            t.left_wheel_floating(),
            t.right_wheel_floating(),
            t.ir_dock_lf(), t.ir_dock_lmf(), t.ir_dock_rmf(), t.ir_dock_rf(),
            t.left_vel_error() as u8, t.right_vel_error() as u8, t.imu_error() as u8, t.charge_error() as u8, t.lidar_error() as u8,
        );
    }
}

fn hexdump(stream: &mut TcpStream) {
    let mut buf = [0u8; 4096];
    let mut off: u64 = 0;
    loop {
        let n = match stream.read(&mut buf) {
            Ok(0) => return,
            Ok(n) => n,
            Err(e) => {
                eprintln!("w10-decode: read error: {e}");
                return;
            }
        };
        for chunk in buf[..n].chunks(16) {
            let mut hex = String::new();
            let mut asc = String::new();
            for &b in chunk {
                hex.push_str(&format!("{b:02x} "));
                asc.push(if (0x20..0x7f).contains(&b) { b as char } else { '.' });
            }
            println!("{off:08x}  {hex:<48} |{asc}|");
            off += chunk.len() as u64;
        }
    }
}
