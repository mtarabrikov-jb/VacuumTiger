//! `w10-mcud` — path 3: a standalone driver that replaces `ava` at the MCU.
//!
//! With `ava` stopped (it exclusively owns `/dev/ttyS4`), this opens the port
//! directly and sustains the MCU protocol so the board stays healthy: it streams
//! `MotorCtrl` (0x00) at ~50 Hz, replays the periodic `SetLED`/`SetCleaning`/
//! `0x14`/`0x26` frames `ava` sends, and answers every `0x0f` ping with a pong
//! (echoing the ping's first 4 bytes — the com-fault handshake). Motion comes
//! from a control client (text over TCP 7705, same protocol as `avatap-relay`)
//! and is gated by a command watchdog, a speed clamp and a live cliff/bumper
//! hazard read from the MCU's own Triggers stream. Raw telemetry is re-served on
//! 7701 (MCU, ttyS4) and 7702 (LDS, ttyS3) so `w10-decode` still works. The LDS
//! turret is silent until enabled via the ttyS4 lidar command.
//!
//! v1 scope: teleop + actuator probing — no SLAM, docking or charging logic (the dock
//! hardware handles charging). Start/stop it with the `mcud.sh` wrapper, which
//! stops `ava` (and its respawn) first and restores it afterward.

use dreame_w10_proto::{encode_frame, encode_motor_ctrl, parse_body, FrameScanner};
use std::fs::{File, OpenOptions};
use std::io::{BufRead, BufReader, Read, Write};
use std::net::{TcpListener, TcpStream};
use std::os::unix::fs::OpenOptionsExt;
use std::os::unix::io::AsRawFd;
use std::sync::atomic::{AtomicBool, AtomicU32, AtomicU64, Ordering};
use std::sync::{Arc, Mutex};
use std::thread;
use std::time::{Duration, Instant};

const TTY_MCU: &str = "/dev/ttyS4";
const TTY_LDS: &str = "/dev/ttyS3";
const CONTROL_PORT: u16 = 7705;
const TELEM_MCU_PORT: u16 = 7701;
const TELEM_LDS_PORT: u16 = 7702;

const MAX_LINEAR_MM_S: f32 = 150.0;
const MAX_ROT_RAD_S: f32 = 1.5;
const WATCHDOG_MS: u64 = 500; // no fresh command for this long -> stop

// MCU message type ids (subset we act on / emit).
const T_TRIGGERS: u8 = 0x00;
const T_PING: u8 = 0x0f;

struct Shared {
    start: Instant,
    // control command
    enabled: AtomicBool,
    linear_bits: AtomicU32,
    rot_bits: AtomicU32,
    last_cmd_ms: AtomicU64,
    // safety
    hazard: AtomicBool,
    shutdown: AtomicBool,
    // lidar: when on, the periodic frames carry ava's nav values (keeps the
    // turret spinning) instead of the idle ones.
    lidar_on: AtomicBool,
    // diagnostics
    overrides: AtomicU64,
    pongs: AtomicU64,
    // raw-telemetry mirror clients (MCU ttyS4, LDS ttyS3)
    telem: Mutex<Vec<TcpStream>>,
    telem_lds: Mutex<Vec<TcpStream>>,
}

impl Shared {
    fn now_ms(&self) -> u64 {
        self.start.elapsed().as_millis() as u64
    }
}

fn clampf(v: f32, lo: f32, hi: f32) -> f32 {
    v.max(lo).min(hi)
}

/// Open a serial port in raw mode at `baud` 8N1.
fn open_serial(path: &str, baud: libc::speed_t) -> std::io::Result<File> {
    let f = OpenOptions::new()
        .read(true)
        .write(true)
        .custom_flags(libc::O_NOCTTY)
        .open(path)?;
    let fd = f.as_raw_fd();
    unsafe {
        let mut t: libc::termios = std::mem::zeroed();
        if libc::tcgetattr(fd, &mut t) != 0 {
            return Err(std::io::Error::last_os_error());
        }
        libc::cfmakeraw(&mut t);
        libc::cfsetispeed(&mut t, baud);
        libc::cfsetospeed(&mut t, baud);
        t.c_cflag |= libc::CLOCAL | libc::CREAD;
        t.c_cc[libc::VMIN] = 0;
        t.c_cc[libc::VTIME] = 1; // 0.1 s read timeout so the reader can exit
        if libc::tcsetattr(fd, libc::TCSANOW, &t) != 0 {
            return Err(std::io::Error::last_os_error());
        }
    }
    Ok(f)
}

/// Encode a frame and write it under the shared write lock.
fn send(w: &Mutex<File>, typ: u8, payload: &[u8]) {
    let mut buf = [0u8; 64];
    if let Some(n) = encode_frame(typ, payload, &mut buf) {
        if let Ok(mut f) = w.lock() {
            let _ = f.write_all(&buf[..n]);
        }
    }
}

/// RX: parse the MCU stream — answer pings, track the cliff/bumper hazard, and
/// mirror raw bytes to telemetry clients.
fn rx_loop(mut rd: File, w: Arc<Mutex<File>>, sh: Arc<Shared>) {
    let mut sc = FrameScanner::new();
    let mut buf = [0u8; 4096];
    while !sh.shutdown.load(Ordering::Relaxed) {
        let n = match rd.read(&mut buf) {
            Ok(0) => continue,
            Ok(n) => n,
            Err(_) => {
                thread::sleep(Duration::from_millis(5));
                continue;
            }
        };
        // mirror to telemetry clients (best effort)
        if let Ok(mut cs) = sh.telem.lock() {
            cs.retain_mut(|c| c.write_all(&buf[..n]).is_ok());
        }
        for &b in &buf[..n] {
            if let Some(body) = sc.push(b) {
                if let Ok((typ, payload)) = parse_body(body) {
                    match typ {
                        T_PING if payload.len() >= 4 => {
                            send(&w, T_PING, &payload[..4]); // echo the timestamp
                            sh.pongs.fetch_add(1, Ordering::Relaxed);
                        }
                        T_TRIGGERS if payload.len() >= 2 => {
                            let hz = (payload[0] & 0xF0) != 0 || payload[1] != 0;
                            sh.hazard.store(hz, Ordering::Relaxed);
                        }
                        _ => {}
                    }
                }
            }
        }
    }
}

/// TX: MotorCtrl at 50 Hz plus the periodic frames `ava` emits.
fn tx_loop(w: Arc<Mutex<File>>, sh: Arc<Shared>) {
    let mut tick: u64 = 0;
    let mut mbuf = [0u8; 64];
    while !sh.shutdown.load(Ordering::Relaxed) {
        // Commanded velocity, gated by enable + watchdog + clamp + hazard.
        let fresh = sh.now_ms().wrapping_sub(sh.last_cmd_ms.load(Ordering::Relaxed)) < WATCHDOG_MS;
        let (mut lin, rot) = if sh.enabled.load(Ordering::Relaxed) && fresh {
            (
                clampf(f32::from_bits(sh.linear_bits.load(Ordering::Relaxed)), -MAX_LINEAR_MM_S, MAX_LINEAR_MM_S),
                clampf(f32::from_bits(sh.rot_bits.load(Ordering::Relaxed)), -MAX_ROT_RAD_S, MAX_ROT_RAD_S),
            )
        } else {
            (0.0, 0.0)
        };
        if sh.hazard.load(Ordering::Relaxed) && lin != 0.0 {
            lin = 0.0; // never translate into a detected cliff/bump
        }
        if let Some(n) = encode_motor_ctrl(1, lin, rot, &mut mbuf) {
            if let Ok(mut f) = w.lock() {
                let _ = f.write_all(&mbuf[..n]);
            }
            sh.overrides.fetch_add(1, Ordering::Relaxed);
        }
        // Periodic frames replicating ava's steady-state mix (staggered). With
        // the lidar on, 0x14/0x26 carry ava's nav values (and 0x1d re-pulses) so
        // the turret keeps spinning; otherwise the idle values.
        let lidar = sh.lidar_on.load(Ordering::Relaxed);
        if tick % 25 == 5 {
            send(&w, 0x02, &[0x21]); // SetLED / heartbeat, ~2 Hz
        }
        if tick % 50 == 10 {
            send(&w, 0x01, &[0x00, 0x01, 0x00, 0x00, 0x00, 0x00]); // SetCleaning idle
        }
        if tick % 50 == 20 {
            send(&w, 0x14, if lidar { &[0x04, 0x01] } else { &[0x04, 0x00] });
        }
        if tick % 50 == 30 {
            let p: &[u8] = if lidar {
                &[0x14, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x04]
            } else {
                &[0x64, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x04]
            };
            send(&w, 0x26, p);
        }
        if lidar && tick % 200 == 40 {
            send(&w, 0x1d, &[0x05, 0x01]); // laser enable re-pulse, ~every 4 s
        }
        tick = tick.wrapping_add(1);
        thread::sleep(Duration::from_millis(20)); // ~50 Hz
    }
    // On exit, leave the motors stopped.
    if let Some(n) = encode_motor_ctrl(1, 0.0, 0.0, &mut mbuf) {
        if let Ok(mut f) = w.lock() {
            let _ = f.write_all(&mbuf[..n]);
        }
    }
}

/// Parse `"aa bb cc"` / `"aabbcc"` hex into bytes.
fn parse_hex(it: std::str::SplitWhitespace) -> Option<Vec<u8>> {
    let mut out = Vec::new();
    for tok in it {
        let t = tok.trim_start_matches("0x");
        if t.len() == 2 {
            out.push(u8::from_str_radix(t, 16).ok()?);
        } else if t.len() % 2 == 0 {
            for i in (0..t.len()).step_by(2) {
                out.push(u8::from_str_radix(&t[i..i + 2], 16).ok()?);
            }
        } else {
            return None;
        }
    }
    Some(out)
}

/// Control server: one drive client at a time (text lines, same as avatap-relay).
/// Also accepts `"frame <type_hex> <payload_hex...>"` to send one arbitrary frame
/// (actuator probing / control — e.g. SetCleaning or the lidar enable).
fn control_loop(sh: Arc<Shared>, w: Arc<Mutex<File>>) {
    let l = match TcpListener::bind(("0.0.0.0", CONTROL_PORT)) {
        Ok(l) => l,
        Err(e) => {
            eprintln!("mcud: cannot bind control {}: {}", CONTROL_PORT, e);
            return;
        }
    };
    eprintln!("mcud: control on 0.0.0.0:{}", CONTROL_PORT);
    for stream in l.incoming() {
        let Ok(s) = stream else { continue };
        let _ = s.set_nodelay(true);
        let peer = s.peer_addr().map(|a| a.to_string()).unwrap_or_default();
        eprintln!("mcud: control client {}", peer);
        let reader = BufReader::new(s);
        for line in reader.lines() {
            let Ok(line) = line else { break };
            let t = line.trim();
            if t.is_empty() {
                continue;
            }
            if t == "stop" || t == "disable" {
                sh.enabled.store(false, Ordering::Relaxed);
                continue;
            }
            let mut it = t.split_whitespace();
            let first = it.next().unwrap_or("");
            if first == "frame" {
                let typ = it
                    .next()
                    .and_then(|s| u8::from_str_radix(s.trim_start_matches("0x"), 16).ok());
                match (typ, parse_hex(it)) {
                    (Some(typ), Some(p)) => {
                        send(&w, typ, &p);
                        eprintln!("mcud: sent frame 0x{:02x} ({} bytes)", typ, p.len());
                    }
                    _ => eprintln!("mcud: bad frame line {:?}", t),
                }
                continue;
            }
            if first == "lidar" {
                let on = it.next().map(|s| s != "0").unwrap_or(false);
                sh.lidar_on.store(on, Ordering::Relaxed);
                eprintln!("mcud: lidar {}", if on { "on" } else { "off" });
                continue;
            }
            // drive: "<linear> <rot>"
            let lin = first.parse::<f32>().ok();
            let rot = it.next().and_then(|s| s.parse::<f32>().ok());
            if let (Some(l), Some(r)) = (lin, rot) {
                if l.is_finite() && r.is_finite() {
                    sh.linear_bits.store(l.to_bits(), Ordering::Relaxed);
                    sh.rot_bits.store(r.to_bits(), Ordering::Relaxed);
                    sh.last_cmd_ms.store(sh.now_ms(), Ordering::Relaxed);
                    sh.enabled.store(true, Ordering::Relaxed);
                }
            }
        }
        sh.enabled.store(false, Ordering::Relaxed); // client gone -> stop
        eprintln!("mcud: control released ({})", peer);
    }
}

/// LDS read: forward the raw ttyS3 scan stream to lds-rx clients. The LDS is
/// read-only (the turret spins only once enabled via the ttyS4 lidar command).
fn lds_rx_loop(mut rd: File, sh: Arc<Shared>) {
    let mut buf = [0u8; 4096];
    while !sh.shutdown.load(Ordering::Relaxed) {
        let n = match rd.read(&mut buf) {
            Ok(0) => continue,
            Ok(n) => n,
            Err(_) => {
                thread::sleep(Duration::from_millis(5));
                continue;
            }
        };
        if let Ok(mut cs) = sh.telem_lds.lock() {
            cs.retain_mut(|c| c.write_all(&buf[..n]).is_ok());
        }
    }
}

/// Telemetry server: hand each client the raw byte stream of one channel.
fn telem_loop(sh: Arc<Shared>, port: u16, lds: bool) {
    let l = match TcpListener::bind(("0.0.0.0", port)) {
        Ok(l) => l,
        Err(e) => {
            eprintln!("mcud: cannot bind telem {}: {}", port, e);
            return;
        }
    };
    eprintln!("mcud: {} on 0.0.0.0:{}", if lds { "lds-rx" } else { "mcu-rx" }, port);
    for stream in l.incoming() {
        let Ok(s) = stream else { continue };
        let _ = s.set_nodelay(true);
        let v = if lds { &sh.telem_lds } else { &sh.telem };
        if let Ok(mut cs) = v.lock() {
            cs.push(s);
        }
    }
}

fn main() {
    let sh = Arc::new(Shared {
        start: Instant::now(),
        enabled: AtomicBool::new(false),
        linear_bits: AtomicU32::new(0),
        rot_bits: AtomicU32::new(0),
        last_cmd_ms: AtomicU64::new(0),
        hazard: AtomicBool::new(false),
        shutdown: AtomicBool::new(false),
        lidar_on: AtomicBool::new(false),
        overrides: AtomicU64::new(0),
        pongs: AtomicU64::new(0),
        telem: Mutex::new(Vec::new()),
        telem_lds: Mutex::new(Vec::new()),
    });

    let rd = match open_serial(TTY_MCU, libc::B115200) {
        Ok(f) => f,
        Err(e) => {
            eprintln!("mcud: cannot open {} ({}) — is ava stopped?", TTY_MCU, e);
            std::process::exit(1);
        }
    };
    let wr = match rd.try_clone() {
        Ok(f) => f,
        Err(e) => {
            eprintln!("mcud: try_clone failed: {}", e);
            std::process::exit(1);
        }
    };
    let w = Arc::new(Mutex::new(wr));
    eprintln!("mcud: driving MCU on {} (MotorCtrl 50Hz + pong + heartbeats)", TTY_MCU);

    // LDS is optional: open it read-only-ish and forward its scan stream. The
    // turret is silent until enabled via the ttyS4 lidar command.
    let lds_rd = match open_serial(TTY_LDS, libc::B230400) {
        Ok(f) => {
            eprintln!("mcud: LDS on {} (forwarding to lds-rx)", TTY_LDS);
            Some(f)
        }
        Err(e) => {
            eprintln!("mcud: WARN cannot open {} ({}) — LDS disabled", TTY_LDS, e);
            None
        }
    };

    let mut hs = Vec::new();
    {
        let (w, sh) = (w.clone(), sh.clone());
        hs.push(thread::spawn(move || rx_loop(rd, w, sh)));
    }
    {
        let (w, sh) = (w.clone(), sh.clone());
        hs.push(thread::spawn(move || tx_loop(w, sh)));
    }
    {
        let (sh, w) = (sh.clone(), w.clone());
        hs.push(thread::spawn(move || control_loop(sh, w)));
    }
    if let Some(lds_rd) = lds_rd {
        let sh = sh.clone();
        hs.push(thread::spawn(move || lds_rx_loop(lds_rd, sh)));
    }
    {
        let sh = sh.clone();
        hs.push(thread::spawn(move || telem_loop(sh, TELEM_MCU_PORT, false)));
    }
    {
        let sh = sh.clone();
        hs.push(thread::spawn(move || telem_loop(sh, TELEM_LDS_PORT, true)));
    }
    for h in hs {
        let _ = h.join();
    }
}
