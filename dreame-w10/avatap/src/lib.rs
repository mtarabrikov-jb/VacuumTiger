//! `avatap.so` — an `LD_PRELOAD` shim injected into `ava` (the W10 navigation
//! daemon) that mirrors its serial traffic to shared memory. Read-only by
//! default; it also has an **opt-in control override** that lets an out-of-`ava`
//! client replace the `MotorCtrl` (0x00) frame `ava` sends to the MCU with a
//! commanded velocity (via the `Control` block in shm). The override is gated by
//! a watchdog (stale command -> `ava` resumes), a speed clamp and a live
//! cliff/bumper hazard gate; with no active command the tap is a pure mirror.
//!
//! `ava` exclusively owns `/dev/ttyS4` (the motor/IMU/dock MCU) and `/dev/ttyS3`
//! (the LDS/LIDAR). We can't open those ports a second time without stealing
//! bytes, so instead we interpose libc `open`/`read`/`write`/`close`: when `ava`
//! opens one of those paths we remember the fd, and every subsequent `read`
//! (telemetry / scan bytes) and `write` (MotorCtrl / SetCleaning / LED) is copied
//! into an `avatap_shm` ring. The out-of-process `avatap-relay` drains the rings
//! and serves them over TCP. Nothing here blocks or allocates on the hot path,
//! so `ava`'s real-time behavior is unaffected — safe to keep active while the
//! robot drives.
//!
//! `no_std` on purpose: the only libc symbols referenced are ones present on the
//! robot's glibc 2.23, so the `.so` loads there.

#![no_std]

use avatap_shm::{Shm, CH_LDS_RX, CH_LDS_TX, CH_MCU_RX, CH_MCU_TX, NCHAN};
use core::cell::UnsafeCell;
use core::ffi::c_void;
use core::sync::atomic::{AtomicBool, AtomicPtr, AtomicU8, AtomicU32, Ordering};
use dreame_w10_proto::{encode_motor_ctrl, parse_body, FrameScanner, T_TRIGGERS};
use libc::{c_char, c_int, c_long, off_t, size_t, ssize_t};

#[panic_handler]
fn panic(_: &core::panic::PanicInfo) -> ! {
    // Never unwind across the C ABI into ava; just abort this process.
    unsafe { libc::abort() }
}

// ---------------------------------------------------------------------------
// fd -> channel table. 0 = untracked, 1 = MCU (ttyS4), 2 = LDS (ttyS3).
// ---------------------------------------------------------------------------
const MAXFD: usize = 4096;
const K_MCU: u8 = 1;
const K_LDS: u8 = 2;
static CHAN: [AtomicU8; MAXFD] = [const { AtomicU8::new(0) }; MAXFD];
static LOCK: [AtomicBool; NCHAN] = [const { AtomicBool::new(false) }; NCHAN];

// ---------------------------------------------------------------------------
// Real libc symbols, resolved lazily via dlsym(RTLD_NEXT). Resolution is guarded
// so that if dlsym itself performs I/O on this thread we fall back to a raw
// syscall instead of recursing.
// ---------------------------------------------------------------------------
static REAL_READ: AtomicPtr<c_void> = AtomicPtr::new(core::ptr::null_mut());
static REAL_WRITE: AtomicPtr<c_void> = AtomicPtr::new(core::ptr::null_mut());
static REAL_OPEN: AtomicPtr<c_void> = AtomicPtr::new(core::ptr::null_mut());
static REAL_OPEN64: AtomicPtr<c_void> = AtomicPtr::new(core::ptr::null_mut());
static REAL_OPENAT: AtomicPtr<c_void> = AtomicPtr::new(core::ptr::null_mut());
static REAL_CLOSE: AtomicPtr<c_void> = AtomicPtr::new(core::ptr::null_mut());
static RESOLVING: AtomicBool = AtomicBool::new(false);

unsafe fn resolve(slot: &AtomicPtr<c_void>, name: *const c_char) -> *mut c_void {
    let cached = slot.load(Ordering::Acquire);
    if !cached.is_null() {
        return cached;
    }
    if RESOLVING.swap(true, Ordering::AcqRel) {
        return core::ptr::null_mut(); // re-entered during resolution -> caller uses raw path
    }
    let s = libc::dlsym(libc::RTLD_NEXT, name);
    if !s.is_null() {
        slot.store(s, Ordering::Release);
    }
    RESOLVING.store(false, Ordering::Release);
    s
}

type ReadFn = unsafe extern "C" fn(c_int, *mut c_void, size_t) -> ssize_t;
type WriteFn = unsafe extern "C" fn(c_int, *const c_void, size_t) -> ssize_t;
type OpenFn = unsafe extern "C" fn(*const c_char, c_int, c_int) -> c_int;
type OpenatFn = unsafe extern "C" fn(c_int, *const c_char, c_int, c_int) -> c_int;
type CloseFn = unsafe extern "C" fn(c_int) -> c_int;

// ---------------------------------------------------------------------------
// Shared memory (mapped once, lazily, from the first tracked I/O).
// ---------------------------------------------------------------------------
static SHM: AtomicPtr<Shm> = AtomicPtr::new(core::ptr::null_mut());
static SHM_STATE: AtomicU32 = AtomicU32::new(0); // 0 uninit, 1 busy, 2 ready, 3 failed

unsafe fn now_ns() -> u64 {
    let mut ts = core::mem::zeroed::<libc::timespec>();
    if libc::clock_gettime(libc::CLOCK_MONOTONIC, &mut ts) == 0 {
        (ts.tv_sec as u64) * 1_000_000_000 + ts.tv_nsec as u64
    } else {
        0
    }
}

unsafe fn map_shm() -> *mut Shm {
    let openp = resolve(&REAL_OPEN, c"open".as_ptr());
    if openp.is_null() {
        return core::ptr::null_mut();
    }
    let open: OpenFn = core::mem::transmute(openp);
    let fd = open(
        avatap_shm::SHM_PATH_C.as_ptr() as *const c_char,
        libc::O_RDWR | libc::O_CREAT,
        0o600,
    );
    if fd < 0 {
        return core::ptr::null_mut();
    }
    let ok = libc::ftruncate(fd, Shm::SIZE as off_t) == 0;
    let p = if ok {
        libc::mmap(
            core::ptr::null_mut(),
            Shm::SIZE,
            libc::PROT_READ | libc::PROT_WRITE,
            libc::MAP_SHARED,
            fd,
            0,
        )
    } else {
        libc::MAP_FAILED
    };
    if let Some(close) = real_close() {
        close(fd);
    }
    if p == libc::MAP_FAILED {
        return core::ptr::null_mut();
    }
    Shm::init(p as *mut Shm, now_ns());
    p as *mut Shm
}

unsafe fn shm() -> Option<&'static Shm> {
    let p = SHM.load(Ordering::Acquire);
    if !p.is_null() {
        return Some(&*p);
    }
    if SHM_STATE
        .compare_exchange(0, 1, Ordering::AcqRel, Ordering::Relaxed)
        .is_ok()
    {
        let mapped = map_shm();
        if mapped.is_null() {
            SHM_STATE.store(3, Ordering::Release);
        } else {
            SHM.store(mapped, Ordering::Release);
            SHM_STATE.store(2, Ordering::Release);
        }
    }
    let p = SHM.load(Ordering::Acquire);
    if p.is_null() {
        None
    } else {
        Some(&*p)
    }
}

unsafe fn publish(ch: usize, ptr: *const u8, len: usize) {
    if len == 0 {
        return;
    }
    let shm = match shm() {
        Some(s) => s,
        None => return,
    };
    let slice = core::slice::from_raw_parts(ptr, len);
    while LOCK[ch].swap(true, Ordering::Acquire) {
        core::hint::spin_loop();
    }
    shm.ring[ch].write(slice);
    LOCK[ch].store(false, Ordering::Release);
}

// ---------------------------------------------------------------------------
// Control override (opt-in). Replace ava's MotorCtrl (0x00) with a commanded
// velocity, gated by a watchdog, a speed clamp and a live cliff/bumper hazard.
// Default off: with no active command every write is byte-identical to ava's, so
// the tap stays a pure read-only mirror until a client explicitly takes control.
// ---------------------------------------------------------------------------

/// Speed ceilings the tap enforces regardless of the commanded value.
const MAX_LINEAR_MM_S: f32 = 150.0;
const MAX_ROT_RAD_S: f32 = 1.5;
/// After this many MotorCtrl writes with an unchanged control `seq`, revert to
/// passthrough (~0.5 s at the 50 Hz MotorCtrl cadence — ava is the clock).
const WATCHDOG_WRITES: u32 = 25;

static LAST_SEQ: AtomicU32 = AtomicU32::new(0);
static STALE_WRITES: AtomicU32 = AtomicU32::new(u32::MAX); // start stale
/// Latest cliff / bumper / wheel-float hazard, tracked from the MCU read stream.
static HAZARD: AtomicBool = AtomicBool::new(false);

/// A `Sync` cell for the single-threaded MCU-read frame scanner.
struct SyncCell<T>(UnsafeCell<T>);
unsafe impl<T> Sync for SyncCell<T> {}
static RX_SCAN: SyncCell<FrameScanner> = SyncCell(UnsafeCell::new(FrameScanner::new()));
static RX_LOCK: AtomicBool = AtomicBool::new(false);

/// Feed MCU->SoC bytes through a frame scanner and update [`HAZARD`] from the
/// latest Triggers (0x00): bumper (bits 4/5) or wheel-float (6/7) in `payload[0]`,
/// or any cliff / floor sensor (`payload[1]`).
unsafe fn track_hazard(bytes: &[u8]) {
    if RX_LOCK.swap(true, Ordering::Acquire) {
        return; // contended (shouldn't happen: single reader thread) -> skip
    }
    let sc = &mut *RX_SCAN.0.get();
    for &b in bytes {
        if let Some(body) = sc.push(b) {
            if let Ok((typ, payload)) = parse_body(body) {
                if typ == T_TRIGGERS && payload.len() >= 2 {
                    let hz = (payload[0] & 0xF0) != 0 || payload[1] != 0;
                    HAZARD.store(hz, Ordering::Relaxed);
                }
            }
        }
    }
    RX_LOCK.store(false, Ordering::Release);
}

#[inline]
fn clampf(v: f32, lo: f32, hi: f32) -> f32 {
    if v < lo {
        lo
    } else if v > hi {
        hi
    } else {
        v
    }
}

/// If control is enabled and fresh, build the override MotorCtrl frame into `out`
/// and return its length; otherwise `None` (caller passes ava's frame through).
unsafe fn control_frame(out: &mut [u8]) -> Option<usize> {
    let shm = shm()?;
    let (seq, enabled, linear, rot) = shm.control.snapshot();
    // Watchdog on the control seq, clocked by ava's own MotorCtrl cadence.
    let fresh = if seq != LAST_SEQ.load(Ordering::Relaxed) {
        LAST_SEQ.store(seq, Ordering::Relaxed);
        STALE_WRITES.store(0, Ordering::Relaxed);
        true
    } else {
        STALE_WRITES.fetch_add(1, Ordering::Relaxed) + 1 <= WATCHDOG_WRITES
    };
    if !enabled || !fresh {
        return None;
    }
    let mut lin = clampf(linear, -MAX_LINEAR_MM_S, MAX_LINEAR_MM_S);
    let rotc = clampf(rot, -MAX_ROT_RAD_S, MAX_ROT_RAD_S);
    // Hazard gate: never translate into a detected cliff/bump (rotation stays ok).
    if HAZARD.load(Ordering::Relaxed) && lin != 0.0 {
        lin = 0.0;
        shm.control.hazard_clamps.fetch_add(1, Ordering::Relaxed);
    }
    let n = encode_motor_ctrl(1, lin, rotc, out)?;
    shm.control.overrides.fetch_add(1, Ordering::Relaxed);
    Some(n)
}

/// True if `buf` starts a MotorCtrl frame: `<`(0x3c) `len=09` `type=00`. Both
/// `09`/`00` are escape-free, and only MotorCtrl carries a 9-byte payload.
#[inline]
unsafe fn is_motorctrl(buf: *const u8, count: usize) -> bool {
    count >= 3 && *buf == b'<' && *buf.add(1) == 9 && *buf.add(2) == 0
}

// ---------------------------------------------------------------------------
// Real-symbol accessors (cached fn pointers)
// ---------------------------------------------------------------------------
unsafe fn real_read() -> Option<ReadFn> {
    let p = resolve(&REAL_READ, c"read".as_ptr());
    if p.is_null() {
        None
    } else {
        Some(core::mem::transmute::<*mut c_void, ReadFn>(p))
    }
}
unsafe fn real_write() -> Option<WriteFn> {
    let p = resolve(&REAL_WRITE, c"write".as_ptr());
    if p.is_null() {
        None
    } else {
        Some(core::mem::transmute::<*mut c_void, WriteFn>(p))
    }
}
unsafe fn real_close() -> Option<CloseFn> {
    let p = resolve(&REAL_CLOSE, c"close".as_ptr());
    if p.is_null() {
        None
    } else {
        Some(core::mem::transmute::<*mut c_void, CloseFn>(p))
    }
}

// ---------------------------------------------------------------------------
// Path classification
// ---------------------------------------------------------------------------
unsafe fn classify(path: *const c_char) -> u8 {
    if path.is_null() {
        return 0;
    }
    let mut len = 0usize;
    while len < 512 && *path.add(len) != 0 {
        len += 1;
    }
    let s = core::slice::from_raw_parts(path as *const u8, len);
    if ends_with(s, b"ttyS4") {
        K_MCU
    } else if ends_with(s, b"ttyS3") {
        K_LDS
    } else {
        0
    }
}
fn ends_with(s: &[u8], suffix: &[u8]) -> bool {
    s.len() >= suffix.len() && &s[s.len() - suffix.len()..] == suffix
}

unsafe fn track(fd: c_int, path: *const c_char) {
    if fd < 0 || (fd as usize) >= MAXFD {
        return;
    }
    let c = classify(path);
    if c != 0 {
        CHAN[fd as usize].store(c, Ordering::Relaxed);
        // bump the RX ring's open counter (diagnostic)
        if let Some(s) = shm() {
            let ring = if c == K_MCU { CH_MCU_RX } else { CH_LDS_RX };
            s.ring[ring].opens.fetch_add(1, Ordering::Relaxed);
        }
    }
}

// ---------------------------------------------------------------------------
// Raw syscall fallbacks (used only before real symbols resolve)
// ---------------------------------------------------------------------------
unsafe fn raw_read(fd: c_int, buf: *mut c_void, n: size_t) -> ssize_t {
    libc::syscall(libc::SYS_read, fd as c_long, buf, n) as ssize_t
}
unsafe fn raw_write(fd: c_int, buf: *const c_void, n: size_t) -> ssize_t {
    libc::syscall(libc::SYS_write, fd as c_long, buf, n) as ssize_t
}
unsafe fn raw_close(fd: c_int) -> c_int {
    libc::syscall(libc::SYS_close, fd as c_long) as c_int
}

// ===========================================================================
// Interposed symbols
// ===========================================================================

#[no_mangle]
pub unsafe extern "C" fn read(fd: c_int, buf: *mut c_void, count: size_t) -> ssize_t {
    let r = match real_read() {
        Some(f) => f(fd, buf, count),
        None => raw_read(fd, buf, count),
    };
    if r > 0 && fd >= 0 && (fd as usize) < MAXFD {
        match CHAN[fd as usize].load(Ordering::Relaxed) {
            K_MCU => {
                publish(CH_MCU_RX, buf as *const u8, r as usize);
                track_hazard(core::slice::from_raw_parts(buf as *const u8, r as usize));
            }
            K_LDS => publish(CH_LDS_RX, buf as *const u8, r as usize),
            _ => {}
        }
    }
    r
}

#[no_mangle]
pub unsafe extern "C" fn write(fd: c_int, buf: *const c_void, count: size_t) -> ssize_t {
    if fd >= 0 && (fd as usize) < MAXFD && count > 0 {
        let kind = CHAN[fd as usize].load(Ordering::Relaxed);
        // Mirror ava's intended bytes (what it *wanted* to send).
        match kind {
            K_MCU => publish(CH_MCU_TX, buf as *const u8, count),
            K_LDS => publish(CH_LDS_TX, buf as *const u8, count),
            _ => {}
        }
        // Control override: swap ava's MotorCtrl for the commanded velocity. Only
        // when a client is actively driving; otherwise this branch is inert.
        if kind == K_MCU && is_motorctrl(buf as *const u8, count) {
            let mut frame = [0u8; 64];
            if let Some(n) = control_frame(&mut frame) {
                let p = frame.as_ptr() as *const c_void;
                match real_write() {
                    Some(f) => {
                        f(fd, p, n);
                    }
                    None => {
                        raw_write(fd, p, n);
                    }
                }
                return count as ssize_t; // report ava's whole frame as written
            }
        }
    }
    match real_write() {
        Some(f) => f(fd, buf, count),
        None => raw_write(fd, buf, count),
    }
}

#[no_mangle]
pub unsafe extern "C" fn open(path: *const c_char, flags: c_int, mode: c_int) -> c_int {
    let p = resolve(&REAL_OPEN, c"open".as_ptr());
    let fd = if !p.is_null() {
        core::mem::transmute::<*mut c_void, OpenFn>(p)(path, flags, mode)
    } else {
        libc::syscall(libc::SYS_openat, libc::AT_FDCWD as c_long, path, flags, mode) as c_int
    };
    track(fd, path);
    fd
}

#[no_mangle]
pub unsafe extern "C" fn open64(path: *const c_char, flags: c_int, mode: c_int) -> c_int {
    let p = resolve(&REAL_OPEN64, c"open64".as_ptr());
    let fd = if !p.is_null() {
        core::mem::transmute::<*mut c_void, OpenFn>(p)(path, flags, mode)
    } else {
        open(path, flags, mode)
    };
    track(fd, path);
    fd
}

#[no_mangle]
pub unsafe extern "C" fn openat(dirfd: c_int, path: *const c_char, flags: c_int, mode: c_int) -> c_int {
    let p = resolve(&REAL_OPENAT, c"openat".as_ptr());
    let fd = if !p.is_null() {
        core::mem::transmute::<*mut c_void, OpenatFn>(p)(dirfd, path, flags, mode)
    } else {
        libc::syscall(libc::SYS_openat, dirfd as c_long, path, flags, mode) as c_int
    };
    track(fd, path);
    fd
}

#[no_mangle]
pub unsafe extern "C" fn close(fd: c_int) -> c_int {
    if fd >= 0 && (fd as usize) < MAXFD {
        CHAN[fd as usize].store(0, Ordering::Relaxed);
    }
    match real_close() {
        Some(f) => f(fd),
        None => raw_close(fd),
    }
}
