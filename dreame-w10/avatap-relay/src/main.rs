//! `avatap-relay` — out-of-`ava` process that maps the `avatap_shm` rings the
//! tap fills and re-serves each channel as a raw byte stream over TCP.
//!
//! One port per channel (a client just connects and reads bytes; a decoder or
//! the SangamIO `dreame_w10` driver parses them):
//!   7701 mcu-rx   telemetry from the MCU (/dev/ttyS4 reads)
//!   7702 lds-rx   raw LDS/LIDAR scan bytes (/dev/ttyS3 reads)
//!   7703 mcu-tx   commands ava sent to the MCU (MotorCtrl/SetCleaning/LED)
//!   7704 lds-tx   bytes ava sent to the LDS
//!
//! Thread-per-client with blocking writes: a slow/stalled client only lags
//! itself; when it resumes, the ring resyncs and reports the dropped bytes. The
//! relay never writes to any device — it is a read-only mirror.

use avatap_shm::{Shm, CH_LDS_RX, CH_LDS_TX, CH_MCU_RX, CH_MCU_TX};
use std::ffi::c_void;
use std::io::Write;
use std::net::{TcpListener, TcpStream};
use std::thread;
use std::time::Duration;

struct Channel {
    port: u16,
    ring: usize,
    name: &'static str,
}

const CHANNELS: [Channel; 4] = [
    Channel { port: 7701, ring: CH_MCU_RX, name: "mcu-rx" },
    Channel { port: 7702, ring: CH_LDS_RX, name: "lds-rx" },
    Channel { port: 7703, ring: CH_MCU_TX, name: "mcu-tx" },
    Channel { port: 7704, ring: CH_LDS_TX, name: "lds-tx" },
];

fn map_shm() -> &'static Shm {
    let path = std::ffi::CString::new(avatap_shm::SHM_PATH).unwrap();
    loop {
        unsafe {
            let fd = libc::open(path.as_ptr(), libc::O_RDWR);
            if fd >= 0 {
                let p = libc::mmap(
                    std::ptr::null_mut(),
                    Shm::SIZE,
                    libc::PROT_READ | libc::PROT_WRITE,
                    libc::MAP_SHARED,
                    fd,
                    0,
                );
                libc::close(fd);
                if p != libc::MAP_FAILED {
                    if let Some(s) = Shm::view(p as *const Shm) {
                        return s;
                    }
                    libc::munmap(p as *mut c_void, Shm::SIZE);
                }
            }
        }
        eprintln!("avatap-relay: waiting for {} (is the tap active in ava?)", avatap_shm::SHM_PATH);
        thread::sleep(Duration::from_millis(500));
    }
}

fn serve_client(shm: &'static Shm, ring_idx: usize, name: &'static str, mut stream: TcpStream) {
    let peer = stream.peer_addr().map(|a| a.to_string()).unwrap_or_default();
    eprintln!("avatap-relay: {} client connected: {}", name, peer);
    let ring = &shm.ring[ring_idx];
    let mut tail = ring.head(); // start live (only new bytes)
    let mut buf = [0u8; 65536];
    let mut total_lost: u64 = 0;
    loop {
        let (n, lost) = ring.read(&mut tail, &mut buf);
        if lost > 0 {
            total_lost += lost;
            eprintln!("avatap-relay: {} client {} lagged, dropped {} bytes (total {})", name, peer, lost, total_lost);
        }
        if n == 0 {
            thread::sleep(Duration::from_millis(2));
            continue;
        }
        if stream.write_all(&buf[..n]).is_err() {
            break;
        }
    }
    eprintln!("avatap-relay: {} client disconnected: {}", name, peer);
}

fn main() {
    let bind_host = std::env::args().nth(1).unwrap_or_else(|| "0.0.0.0".to_string());

    let shm = map_shm();
    eprintln!("avatap-relay: mapped {} ({} bytes)", avatap_shm::SHM_PATH, Shm::SIZE);

    let mut handles = Vec::new();
    for ch in CHANNELS.iter() {
        let addr = format!("{}:{}", bind_host, ch.port);
        let listener = match TcpListener::bind(&addr) {
            Ok(l) => l,
            Err(e) => {
                eprintln!("avatap-relay: cannot bind {} ({}): {}", ch.name, addr, e);
                continue;
            }
        };
        eprintln!("avatap-relay: serving {} on {}", ch.name, addr);
        let ring = ch.ring;
        let name = ch.name;
        handles.push(thread::spawn(move || {
            for stream in listener.incoming() {
                match stream {
                    Ok(s) => {
                        let _ = s.set_nodelay(true);
                        thread::spawn(move || serve_client(shm, ring, name, s));
                    }
                    Err(e) => eprintln!("avatap-relay: {} accept error: {}", name, e),
                }
            }
        }));
    }

    if handles.is_empty() {
        eprintln!("avatap-relay: no channels bound, exiting");
        return;
    }
    for h in handles {
        let _ = h.join();
    }
}
