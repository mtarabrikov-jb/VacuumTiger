//! Dreame W10 LDS/LIDAR protocol (the framed link `ava` speaks on `/dev/ttyS3`
//! @ 230400, config node `AvaNodeLDS`).
//!
//! Reverse-engineered live on the `r2104` by tapping `ava`'s serial I/O while the
//! turret spun (it only spins during navigation; enabling Valetudo manual control
//! is enough to start it). Fixed-length 40-byte packets, little-endian:
//!
//! ```text
//! off  size  field
//!  0    2    55 aa            sync (constant)
//!  2    1    03               packet type (constant)
//!  3    1    08               sample count LSN = 8 (constant in all captures)
//!  4    2    u16  speed       turret rotation speed, raw units. Traced a clean
//!                             spin-up -> plateau (~25000) -> spin-down curve over
//!                             a capture, hence "speed". [partial: units unknown]
//!  6    2    u16  fsa         start angle of this packet's 8 samples
//!  8   24    8x {u16 dist, u8 quality}  samples; see below
//! 32    2    u16  lsa         end angle of this packet's 8 samples
//! 34    2    u16  checksum    noisy per packet; no standard CRC16/sum matched
//!                             (Modbus/CCITT/sum over every tried range). [unknown]
//! 36    2    u16  counter     monotonic +~3836/packet, wraps ~every 17 packets;
//!                             likely a timestamp/tick. [partial]
//! 38    2    u16  aux         high byte 0x4b/0x4c, low byte varies; magnitude near
//!                             `speed`. Not a fixed footer (verified). [unknown]
//! ```
//!
//! Angles are stored as a u16 fraction of a full circle (`65536 == 360 deg`,
//! the working assumption). Within a packet the 8 samples are spread linearly
//! from `fsa` to `lsa`; consecutive packets are angle-continuous (packet N's
//! `lsa` ~ packet N+1's `fsa`), which is how the fsa/lsa pair was confirmed.
//!
//! Each sample is a little-endian `u16` distance in mm plus a `u8` quality. When
//! the distance's top bit (`0x8000`) is set the point is invalid / no-return (the
//! low 15 bits are 0 in that case); otherwise the raw value is the distance in mm
//! (observed 0.34-8.4 m).
//!
//! IMPORTANT coverage caveat: the tapped ttyS3 stream is a FIXED ~126 deg rear
//! arc (`fsa` stays in ~226-352 deg), NOT a full circle. Verified identical under
//! both manual control and active SLAM navigation (a return-to-dock run, robot
//! driving and turning): the arc does not rotate with heading or widen, and the
//! whole stream is type-0x03 packets in this arc. Active navigation does not help
//! -- this serial link simply carries no full-360 feed. The framing/scaling below
//! are unaffected; only the angular coverage is partial.
//!
//! `no_std` and allocation-free, so the in-`ava` tap and the SangamIO driver can
//! share it. The scanner keys on the 4-byte header + fixed length (the trailing
//! bytes are NOT a reliable end marker), and resynchronises on the next header.

// Wire constants.
pub const LDS_SYNC0: u8 = 0x55;
pub const LDS_SYNC1: u8 = 0xAA;
pub const LDS_TYPE: u8 = 0x03;
/// Samples per packet (the constant `0x08` at offset 3; also `LSN`).
pub const LDS_SAMPLES: usize = 8;
/// Total packet length in bytes.
pub const LDS_FRAME_LEN: usize = 40;
/// Full-circle value of the u16 angle fields (working assumption: 65536 == 360deg).
pub const LDS_ANGLE_FULL: u32 = 0x1_0000;
/// Distance bit that marks an invalid / no-return sample.
pub const LDS_DIST_INVALID: u16 = 0x8000;

use super::u16le;

/// One range measurement.
#[derive(Debug, Clone, Copy, Default)]
pub struct LdsSample {
    /// Distance in mm (0 when `!valid`).
    pub dist_mm: u16,
    /// Signal quality/intensity (0..~56 observed).
    pub quality: u8,
    /// True when the sensor reported a real echo (top distance bit clear).
    pub valid: bool,
}

/// A decoded 40-byte LDS packet: 8 samples over the arc `[fsa, lsa]`.
#[derive(Debug, Clone, Copy)]
pub struct LdsFrame {
    /// Turret rotation speed, raw units. [partial]
    pub speed: u16,
    /// Start angle (u16 fraction of a circle).
    pub fsa: u16,
    /// End angle (u16 fraction of a circle).
    pub lsa: u16,
    pub samples: [LdsSample; LDS_SAMPLES],
    /// Per-packet checksum-like field; scheme unknown. [unknown]
    pub checksum: u16,
    /// Monotonic counter / likely timestamp tick. [partial]
    pub counter: u16,
    /// Trailing field, purpose unknown (high byte 0x4b/0x4c). [unknown]
    pub aux: u16,
}

impl LdsFrame {
    /// Parse a full 40-byte packet (header already matched). Returns `None` if
    /// the length or the `55 aa 03 08` header is wrong.
    pub fn parse(f: &[u8]) -> Option<Self> {
        if f.len() < LDS_FRAME_LEN
            || f[0] != LDS_SYNC0
            || f[1] != LDS_SYNC1
            || f[2] != LDS_TYPE
            || f[3] != LDS_SAMPLES as u8
        {
            return None;
        }
        let mut samples = [LdsSample::default(); LDS_SAMPLES];
        for (k, s) in samples.iter_mut().enumerate() {
            let o = 8 + 3 * k;
            let raw = u16le(f, o);
            let valid = raw & LDS_DIST_INVALID == 0;
            *s = LdsSample {
                dist_mm: if valid { raw } else { 0 },
                quality: f[o + 2],
                valid,
            };
        }
        Some(Self {
            speed: u16le(f, 4),
            fsa: u16le(f, 6),
            lsa: u16le(f, 32),
            samples,
            checksum: u16le(f, 34),
            counter: u16le(f, 36),
            aux: u16le(f, 38),
        })
    }

    /// Angular span of this packet's samples, in u16 units (handles wrap).
    #[inline]
    pub fn arc(&self) -> u16 {
        self.lsa.wrapping_sub(self.fsa)
    }

    /// Angle of sample `k` (0..LDS_SAMPLES) as a u16 fraction of a circle,
    /// linearly interpolated between `fsa` and `lsa`.
    #[inline]
    pub fn sample_angle(&self, k: usize) -> u16 {
        let step = (self.arc() as u32 * k as u32) / LDS_SAMPLES as u32;
        self.fsa.wrapping_add(step as u16)
    }

    /// Angle of sample `k` in degrees (`65536 == 360`).
    #[inline]
    pub fn sample_angle_deg(&self, k: usize) -> f32 {
        self.sample_angle(k) as f32 * (360.0 / LDS_ANGLE_FULL as f32)
    }
}

/// Byte-fed de-framer for the LDS stream. Push raw bytes; get an [`LdsFrame`]
/// each time a `55 aa 03 08` header plus 40 total bytes have arrived. Keys only
/// on the header + fixed length (the trailer is not a reliable marker) and
/// resyncs on the next header byte after any mismatch.
pub struct LdsScanner {
    buf: [u8; LDS_FRAME_LEN],
    idx: usize,
}

impl Default for LdsScanner {
    fn default() -> Self {
        Self::new()
    }
}

impl LdsScanner {
    pub const fn new() -> Self {
        Self {
            buf: [0u8; LDS_FRAME_LEN],
            idx: 0,
        }
    }

    /// Feed one byte; returns a decoded frame once 40 bytes have accumulated
    /// behind a valid header.
    pub fn push(&mut self, b: u8) -> Option<LdsFrame> {
        // Validate the 4-byte header as it streams in; anything else is free.
        let header_ok = match self.idx {
            0 => b == LDS_SYNC0,
            1 => b == LDS_SYNC1,
            2 => b == LDS_TYPE,
            3 => b == LDS_SAMPLES as u8,
            _ => true,
        };
        if !header_ok {
            // Resync: this byte might itself be a fresh sync0.
            self.idx = if b == LDS_SYNC0 {
                self.buf[0] = b;
                1
            } else {
                0
            };
            return None;
        }
        self.buf[self.idx] = b;
        self.idx += 1;
        if self.idx == LDS_FRAME_LEN {
            self.idx = 0;
            return LdsFrame::parse(&self.buf);
        }
        None
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    // A real packet captured from the robot's ttyS3 tap (rear-sector sweep).
    const PKT0: [u8; 40] = [
        0x55, 0xaa, 0x03, 0x08, 0x25, 0x4c, 0x8f, 0xa7, 0x5f, 0x07, 0x0c, 0xd6, 0x04, 0x0f, 0xdd,
        0x04, 0x0c, 0x3b, 0x07, 0x09, 0x70, 0x06, 0x07, 0x71, 0x06, 0x08, 0x56, 0x07, 0x0b, 0x59,
        0x07, 0x0c, 0x45, 0xa9, 0x0e, 0x23, 0x28, 0x57, 0xc4, 0x4b,
    ];

    #[test]
    fn parses_known_packet() {
        let f = LdsFrame::parse(&PKT0).unwrap();
        assert_eq!(f.speed, 19493);
        assert_eq!(f.fsa, 42895);
        assert_eq!(f.lsa, 43333);
        assert_eq!(f.counter, 22312);
        // First and last of the 8 samples (dist mm, quality).
        assert_eq!(f.samples[0].dist_mm, 1887);
        assert_eq!(f.samples[0].quality, 12);
        assert!(f.samples[0].valid);
        assert_eq!(f.samples[7].dist_mm, 1881);
        assert_eq!(f.samples[7].quality, 12);
        // Arc is positive and small (~438 units ~ 2.4 deg over 8 samples).
        assert_eq!(f.arc(), 438);
        // Sample 0 sits at fsa; sample angles increase toward lsa.
        assert_eq!(f.sample_angle(0), f.fsa);
        assert!(f.sample_angle(7) > f.fsa && f.sample_angle(7) < f.lsa);
    }

    #[test]
    fn invalid_sample_bit() {
        let mut p = PKT0;
        // Force sample 3's distance to the invalid marker 0x8000.
        let o = 8 + 3 * 3;
        p[o] = 0x00;
        p[o + 1] = 0x80;
        let f = LdsFrame::parse(&p).unwrap();
        assert!(!f.samples[3].valid);
        assert_eq!(f.samples[3].dist_mm, 0);
    }

    #[test]
    fn scanner_finds_frame_amid_noise() {
        let mut sc = LdsScanner::new();
        let mut got = None;
        // Leading garbage, a false 0x55, then the real packet.
        for &b in &[0x00u8, 0xff, 0x55, 0x11] {
            assert!(sc.push(b).is_none());
        }
        for &b in &PKT0 {
            if let Some(f) = sc.push(b) {
                got = Some(f);
            }
        }
        let f = got.expect("frame should be decoded");
        assert_eq!(f.fsa, 42895);
        assert_eq!(f.samples[0].dist_mm, 1887);
    }
}
