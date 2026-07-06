//! Shared byte-ring buffers between the in-`ava` serial tap (`avatap.so`) and the
//! out-of-`ava` relay (`avatap-relay`), backed by a tmpfs file.
//!
//! The tap runs inside the robot's navigation process and must never stall it,
//! so writing is a lock-free `head`-advancing copy with no syscalls and no wait
//! on the reader. If the relay falls behind, the writer laps it and the reader
//! resyncs, counting the overrun. One ring per direction/port; single writer per
//! ring (the `ava` thread that owns that fd), one or more readers.

#![no_std]

use core::cell::UnsafeCell;
use core::sync::atomic::{AtomicU64, Ordering};

pub const SHM_PATH: &str = "/tmp/avatap.shm";
/// NUL-terminated form for C `open()`.
pub const SHM_PATH_C: &[u8] = b"/tmp/avatap.shm\0";
pub const MAGIC: u32 = 0x5041_5441; // "ATAP"
pub const VERSION: u32 = 1;

pub const CH_MCU_RX: usize = 0; // /dev/ttyS4 read  (telemetry from MCU)
pub const CH_MCU_TX: usize = 1; // /dev/ttyS4 write (commands to MCU)
pub const CH_LDS_RX: usize = 2; // /dev/ttyS3 read  (raw LDS scan bytes)
pub const CH_LDS_TX: usize = 3; // /dev/ttyS3 write (LDS start/config)
pub const NCHAN: usize = 4;

pub const RING_BITS: u32 = 18;
pub const RING_SIZE: usize = 1 << RING_BITS; // 256 KiB
pub const RING_MASK: u64 = (RING_SIZE as u64) - 1;

#[repr(C)]
pub struct Ring {
    /// Total bytes ever written (monotonic). Publishes the data before it.
    pub head: AtomicU64,
    /// Times the owning fd was (re)opened — diagnostic.
    pub opens: AtomicU64,
    _pad: [u64; 6],
    data: UnsafeCell<[u8; RING_SIZE]>,
}

// The buffer is a shared mapping guarded by the single-writer discipline and the
// `head` release/acquire fence; concurrent access across processes is intended.
unsafe impl Sync for Ring {}

impl Ring {
    /// Writer side: append `buf` (single writer). Never blocks, never allocates.
    pub fn write(&self, buf: &[u8]) {
        let mut n = buf.len();
        if n == 0 {
            return;
        }
        let mut start = 0usize;
        if n > RING_SIZE {
            start = n - RING_SIZE; // keep only the freshest RING_SIZE bytes
            n = RING_SIZE;
        }
        let head = self.head.load(Ordering::Relaxed);
        let off = (head & RING_MASK) as usize;
        let first = core::cmp::min(RING_SIZE - off, n);
        let dp = self.data.get() as *mut u8;
        unsafe {
            core::ptr::copy_nonoverlapping(buf.as_ptr().add(start), dp.add(off), first);
            if n > first {
                core::ptr::copy_nonoverlapping(buf.as_ptr().add(start + first), dp, n - first);
            }
        }
        // Release so a reader that observes the new head also sees the bytes.
        self.head.store(head + n as u64, Ordering::Release);
    }

    #[inline]
    pub fn head(&self) -> u64 {
        self.head.load(Ordering::Acquire)
    }

    /// Reader side: copy new bytes from `*tail` up to `head` into `out`.
    /// Returns `(copied, lost)` where `lost` counts bytes the reader missed
    /// because it lagged more than one buffer behind. Advances `*tail`.
    pub fn read(&self, tail: &mut u64, out: &mut [u8]) -> (usize, u64) {
        let head = self.head();
        let mut lost = 0u64;
        // If we fell more than a full buffer behind, resync to the oldest still
        // present and report the gap.
        if head.wrapping_sub(*tail) > RING_SIZE as u64 {
            lost = head.wrapping_sub(*tail) - RING_SIZE as u64;
            *tail = head - RING_SIZE as u64;
        }
        let avail = (head - *tail) as usize;
        let n = core::cmp::min(avail, out.len());
        if n == 0 {
            return (0, lost);
        }
        let off = (*tail & RING_MASK) as usize;
        let first = core::cmp::min(RING_SIZE - off, n);
        let sp = self.data.get() as *const u8;
        unsafe {
            core::ptr::copy_nonoverlapping(sp.add(off), out.as_mut_ptr(), first);
            if n > first {
                core::ptr::copy_nonoverlapping(sp, out.as_mut_ptr().add(first), n - first);
            }
        }
        *tail += n as u64;
        (n, lost)
    }
}

#[repr(C)]
pub struct Shm {
    pub magic: u32,
    pub version: u32,
    pub started_ns: u64,
    pub ring: [Ring; NCHAN],
}

unsafe impl Sync for Shm {}

impl Shm {
    pub const SIZE: usize = core::mem::size_of::<Shm>();

    /// Initialize a freshly mapped/zeroed region (writer/tap side).
    ///
    /// # Safety
    /// `ptr` must point to at least [`Shm::SIZE`] writable, zeroed bytes.
    pub unsafe fn init(ptr: *mut Shm, started_ns: u64) -> &'static Shm {
        let s = &mut *ptr;
        s.started_ns = started_ns;
        s.version = VERSION;
        // magic last: a reader keys off it to know the region is ready.
        core::sync::atomic::fence(Ordering::Release);
        core::ptr::write_volatile(&mut s.magic as *mut u32, MAGIC);
        &*ptr
    }

    /// View an existing mapping (reader/relay side); checks magic + version.
    ///
    /// # Safety
    /// `ptr` must point to at least [`Shm::SIZE`] bytes of a mapping produced by
    /// the tap.
    pub unsafe fn view(ptr: *const Shm) -> Option<&'static Shm> {
        let s = &*ptr;
        if core::ptr::read_volatile(&s.magic as *const u32) == MAGIC && s.version == VERSION {
            Some(&*ptr)
        } else {
            None
        }
    }
}
