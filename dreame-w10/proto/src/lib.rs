//! Dreame W10 MCU protocol (the framed binary link `ava` speaks on `/dev/ttyS4`).
//!
//! Framing (from `alufers/dreame_mcu_protocol`, confirmed on the W10 `r2104`):
//! a packet is delimited by `<` (0x3c) .. `>` (0x3e); inside, `?` (0x3f) escapes
//! the next byte verbatim (so a literal `<`, `>` or `?` in the body is preceded
//! by `?`). The unescaped body is `[len][type][payload len bytes][crc16]`, where
//! `len` is the payload length, `type` selects the message, and `crc16` is
//! CRC-16/Modbus over `[len][type][payload]`, stored big-endian (hi, lo).
//!
//! This crate is `no_std` and allocation-free so it can be shared by the in-`ava`
//! tap (a `no_std` cdylib) and the std SangamIO `dreame_w10` driver.
//!
//! NOTE on model differences: the field layouts below match the reverse-
//! engineered Z10/W10 stack. `Status20ms` is 24-26 B on the Z10 but our own RE
//! notes record ~30 B on the W10 — parsers here read the known prefix and ignore
//! trailing bytes, and the exact W10 offsets are validated against a live tap
//! capture (that is milestone 1 of the bridge).

#![no_std]
#![forbid(unsafe_code)]

/// LDS/LIDAR protocol (`/dev/ttyS3`), a separate framing from the MCU link below.
pub mod lds;

// ===========================================================================
// CRC-16/Modbus
// ===========================================================================

/// CRC-16/Modbus (poly 0xA001, init 0xFFFF) over `data`.
pub fn crc16_modbus(data: &[u8]) -> u16 {
    let mut crc: u16 = 0xFFFF;
    for &b in data {
        crc ^= b as u16;
        for _ in 0..8 {
            if crc & 1 != 0 {
                crc = (crc >> 1) ^ 0xA001;
            } else {
                crc >>= 1;
            }
        }
    }
    crc
}

// ===========================================================================
// Wire constants
// ===========================================================================

pub const START: u8 = b'<';
pub const END: u8 = b'>';
pub const ESCAPE: u8 = b'?';

/// Largest unescaped body we accept (`len` is a u8, so payload <= 255, plus
/// len+type+crc = 4 overhead).
pub const MAX_BODY: usize = 260;

// Message type ids (from MCU).
pub const T_TRIGGERS: u8 = 0x00;
pub const T_STATUS_20MS: u8 = 0x01;
pub const T_STATUS_10MS: u8 = 0x02; // IMU
pub const T_STATUS_100MS: u8 = 0x03;
pub const T_STATUS_500MS: u8 = 0x05;
pub const T_PING: u8 = 0x0F;
pub const T_BATTERY: u8 = 0x2B;

// Message type ids (to MCU).
pub const TX_MOTOR_CTRL: u8 = 0x00; // <B f f>  flag, linear, rotational
pub const TX_SET_CLEANING: u8 = 0x01; // <B B B B B>  fan/brush/pump levels
pub const TX_SET_LED: u8 = 0x02; // <B>  button LED state (also a heartbeat)

// ===========================================================================
// Little-endian slice readers (bounds-checked; return 0 past the end)
// ===========================================================================

#[inline]
fn u16le(d: &[u8], o: usize) -> u16 {
    if o + 2 <= d.len() {
        u16::from_le_bytes([d[o], d[o + 1]])
    } else {
        0
    }
}
#[inline]
fn i16le(d: &[u8], o: usize) -> i16 {
    u16le(d, o) as i16
}
#[inline]
fn u32le(d: &[u8], o: usize) -> u32 {
    if o + 4 <= d.len() {
        u32::from_le_bytes([d[o], d[o + 1], d[o + 2], d[o + 3]])
    } else {
        0
    }
}
#[inline]
fn i32le(d: &[u8], o: usize) -> i32 {
    u32le(d, o) as i32
}

// ===========================================================================
// Frame scanner: feed raw serial bytes, get unescaped bodies
// ===========================================================================

/// Stateful de-framer. Push bytes off the wire one at a time; when a full
/// `<..>` packet has been seen, `push` returns its unescaped body
/// (`[len][type][payload][crc16]`). No allocation; the returned slice borrows an
/// internal buffer and is valid until the next `push`.
pub struct FrameScanner {
    buf: [u8; MAX_BODY],
    idx: usize,
    in_frame: bool,
    escape: bool,
    /// Bodies dropped because they exceeded `MAX_BODY` (diagnostic).
    pub overflows: u32,
}

impl Default for FrameScanner {
    fn default() -> Self {
        Self::new()
    }
}

impl FrameScanner {
    pub const fn new() -> Self {
        Self {
            buf: [0u8; MAX_BODY],
            idx: 0,
            in_frame: false,
            escape: false,
            overflows: 0,
        }
    }

    /// Feed one byte. Returns the unescaped body once a packet completes.
    pub fn push(&mut self, b: u8) -> Option<&[u8]> {
        if !self.in_frame {
            if b == START {
                self.in_frame = true;
                self.escape = false;
                self.idx = 0;
            }
            return None;
        }
        if self.escape {
            self.escape = false;
            return self.append(b);
        }
        match b {
            ESCAPE => {
                self.escape = true;
                None
            }
            START => {
                // resync: a new start inside a frame restarts it
                self.idx = 0;
                self.escape = false;
                None
            }
            END => {
                self.in_frame = false;
                Some(&self.buf[..self.idx])
            }
            _ => self.append(b),
        }
    }

    fn append(&mut self, b: u8) -> Option<&[u8]> {
        if self.idx >= MAX_BODY {
            self.overflows = self.overflows.wrapping_add(1);
            self.in_frame = false;
            return None;
        }
        self.buf[self.idx] = b;
        self.idx += 1;
        None
    }
}

// ===========================================================================
// Body -> (type, payload) with CRC check
// ===========================================================================

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FrameError {
    TooShort,
    Crc { received: u16, computed: u16 },
}

/// Split an unescaped body into `(type, payload)`, verifying the CRC.
pub fn parse_body(body: &[u8]) -> Result<(u8, &[u8]), FrameError> {
    if body.len() < 4 {
        return Err(FrameError::TooShort);
    }
    let len = body[0] as usize;
    let typ = body[1];
    let received = ((body[body.len() - 2] as u16) << 8) | body[body.len() - 1] as u16;
    let computed = crc16_modbus(&body[..body.len() - 2]);
    if received != computed {
        return Err(FrameError::Crc { received, computed });
    }
    // payload is between type and crc, clamped to the declared length
    let mut payload = &body[2..body.len() - 2];
    if len <= payload.len() {
        payload = &payload[..len];
    }
    Ok((typ, payload))
}

// ===========================================================================
// Decoded messages (from MCU)
// ===========================================================================

/// `0x01` @20ms — pose/velocity. **W10 layout (30 bytes), verified live by
/// driving the robot:** it is the Z10 layout with 4 reserved bytes inserted at
/// offset 16 (they read `0xA5A5A5A5` at rest), shifting the velocity/current
/// fields by +4. Confirmed: yaw@12 swept with rotation; x@4/y@8 stayed constant
/// during in-place rotation; leftVel@20 / rightVel@22 are 0 at rest and take
/// opposite signs when rotating. **`roller_current@26` and `sidebrush_current@28`
/// verified during a cleaning run** (0..~4 at rest -> ~478 / ~106 with the brushes
/// spinning). `edge_dis_mm@24` stayed 0 even while cleaning (unconfirmed).
#[derive(Debug, Clone, Copy, Default)]
pub struct Status20ms {
    pub timestamp_us: u32,
    pub x_mm10: i32,   // x, tenths of a mm
    pub y_mm10: i32,   // y, tenths of a mm
    pub yaw_cdeg: i16, // centidegrees
    pub yaw_integral: i16,
    // offset 16..19: reserved on the W10 (0xA5A5A5A5 at rest)
    pub left_vel: i16,
    pub right_vel: i16,
    pub edge_dis_mm: i16,
    pub roller_current: i16,
    pub sidebrush_current: i16,
}

impl Status20ms {
    pub const MIN_LEN: usize = 30;
    pub fn parse(p: &[u8]) -> Option<Self> {
        if p.len() < Self::MIN_LEN {
            return None;
        }
        Some(Self {
            timestamp_us: u32le(p, 0),
            x_mm10: i32le(p, 4),
            y_mm10: i32le(p, 8),
            yaw_cdeg: i16le(p, 12),
            yaw_integral: i16le(p, 14),
            left_vel: i16le(p, 20),
            right_vel: i16le(p, 22),
            edge_dis_mm: i16le(p, 24),
            roller_current: i16le(p, 26),
            sidebrush_current: i16le(p, 28),
        })
    }
    #[inline]
    pub fn yaw_deg(&self) -> f32 {
        self.yaw_cdeg as f32 / 100.0
    }
}

/// `0x02` @10ms — IMU + wheel odometry increments. `<I h h h h h h b b>`.
/// `0x02` @10ms — `<I h h h h h h b b>` (18 B). W10 verified live: gyro is
/// centi-deg/s (`/100`); accel is raw LSB at +/-2g full scale (`/16384` LSB/g,
/// gravity read 0.99g on Z at rest); gyro_z is axis index 2 (offset 8).
#[derive(Debug, Clone, Copy, Default)]
pub struct Status10ms {
    pub timestamp_us: u32,
    pub gyro_cdeg_s: [i16; 3], // centi-deg/s (index 2 = yaw rate)
    pub accel_raw: [i16; 3],   // raw LSB, +/-2g full scale (16384 LSB/g)
    pub left_dis_mm: i8,       // signed wheel distance increment since last packet
    pub right_dis_mm: i8,
}

impl Status10ms {
    pub const MIN_LEN: usize = 18;
    /// Accelerometer full-scale: 16384 LSB per g (+/-2g range), verified from
    /// the resting gravity vector on the W10.
    pub const ACCEL_LSB_PER_G: f32 = 16384.0;

    pub fn parse(p: &[u8]) -> Option<Self> {
        if p.len() < Self::MIN_LEN {
            return None;
        }
        Some(Self {
            timestamp_us: u32le(p, 0),
            gyro_cdeg_s: [i16le(p, 4), i16le(p, 6), i16le(p, 8)],
            accel_raw: [i16le(p, 10), i16le(p, 12), i16le(p, 14)],
            left_dis_mm: p[16] as i8,
            right_dis_mm: p[17] as i8,
        })
    }
    #[inline]
    pub fn gyro_deg_s(&self) -> [f32; 3] {
        [
            self.gyro_cdeg_s[0] as f32 / 100.0,
            self.gyro_cdeg_s[1] as f32 / 100.0,
            self.gyro_cdeg_s[2] as f32 / 100.0,
        ]
    }
    #[inline]
    pub fn accel_g(&self) -> [f32; 3] {
        [
            self.accel_raw[0] as f32 / Self::ACCEL_LSB_PER_G,
            self.accel_raw[1] as f32 / Self::ACCEL_LSB_PER_G,
            self.accel_raw[2] as f32 / Self::ACCEL_LSB_PER_G,
        ]
    }
}

/// `0x03` @100ms — tilt, wheel currents, load, consumable flags. **W10: 11 bytes.**
/// Wheel currents **[verified]** by rotating in place: `left_current@4` and
/// `right_current@6` sit near 0 at rest and jump to ~300-430 while the wheels spin.
/// `pitch@0`/`roll@2` are small signed values (deci-degrees assumed; ~-80/+14 = a
/// slight dock-ramp tilt) that shift under motion — offsets confirmed, units not.
/// `load@8` is a signed load/current (near 0 at rest, ~1-23 vacuuming, ~27-343
/// mopping — a pump/mop-load candidate). **`flags@10` is the consumables/attachment
/// bitfield: bit 0 = dustbin missing [verified] (1 when the bin is pulled, 0 when
/// present, isolated by a bin in/out A-B test); the other bits track mop/tank (the
/// byte read `0x12` with no mop and `0x00` with the mop attached) — those exact bit
/// assignments are the Z10 analogy, unverified.**
#[derive(Debug, Clone, Copy, Default)]
pub struct Status100ms {
    pub pitch_ddeg: i16, // [0] deci-degrees (offset verified; unit assumed)
    pub roll_ddeg: i16,  // [2]
    pub left_current: i16,  // [4] [verified] ~0 at rest, ~300-430 spinning
    pub right_current: i16, // [6] [verified]
    pub load: i16,          // [8] pump/mop-load current candidate
    pub flags: u8,          // [10] consumables/attachment bitfield
}

impl Status100ms {
    pub const MIN_LEN: usize = 11;
    pub fn parse(p: &[u8]) -> Option<Self> {
        if p.len() < Self::MIN_LEN {
            return None;
        }
        Some(Self {
            pitch_ddeg: i16le(p, 0),
            roll_ddeg: i16le(p, 2),
            left_current: i16le(p, 4),
            right_current: i16le(p, 6),
            load: i16le(p, 8),
            flags: p[10],
        })
    }
    /// **[verified]** bit 0 of `flags@10`: true when the dust bin is removed.
    #[inline]
    pub fn dust_container_missing(&self) -> bool {
        self.flags & 1 != 0
    }
    #[inline]
    pub fn water_tank_installed(&self) -> bool {
        self.flags & 2 != 0
    }
    #[inline]
    pub fn hepa_state(&self) -> bool {
        self.flags & 4 != 0
    }
    #[inline]
    pub fn carpet_state(&self) -> bool {
        self.flags & 8 != 0
    }
}

/// `0x00` — bumpers, wheel-float, cliff/floor sensors, dock and fault flags.
/// 7-byte bitfield, sent ~10 Hz (so bumper/cliff/lift are all polled together at
/// 10 Hz; the immediate safety reaction is on the MCU itself).
///
/// **W10 live-verified this session:** bit 4 = left bumper, bit 5 = right bumper
/// (pressed each); bit 6 = left / bit 7 = right wheel float (both set when the
/// robot is lifted); **`raw[1]` = six cliff / floor sensors at bits 8-13** (fire
/// when lifted; front L/R = bits 8/11, rear L/R = bits 12/13, bits 9/10 = two more
/// floor sensors, unmapped — see `cliff_flags`); bit 32 = `dock_sta`
/// (clears off-dock). NOTE: bits 16-18 read `0b111` both docked AND lifted, so the
/// Z10 `ir_dock*` decode below is **suspect on the W10** (not dock presence).
///
/// The MCU transmits each byte bit-reversed; equivalently, global bit `k` is
/// `(raw[k/8] >> (k%8)) & 1` (LSB-first), which is what `bit()` returns.
#[derive(Debug, Clone, Copy, Default)]
pub struct Triggers {
    pub raw: [u8; 7],
}

impl Triggers {
    pub const LEN: usize = 7;
    pub fn parse(p: &[u8]) -> Option<Self> {
        if p.len() < Self::LEN {
            return None;
        }
        let mut raw = [0u8; 7];
        raw.copy_from_slice(&p[..7]);
        Some(Self { raw })
    }
    #[inline]
    pub fn bit(&self, k: usize) -> bool {
        (self.raw[k / 8] >> (k % 8)) & 1 != 0
    }
    /// 3-bit dock-IR field starting at global bit `k` (MSB-first as the MCU packs it).
    #[inline]
    fn ir3(&self, k: usize) -> u8 {
        ((self.bit(k) as u8) << 2) | ((self.bit(k + 1) as u8) << 1) | (self.bit(k + 2) as u8)
    }

    pub fn left_bumper(&self) -> bool {
        self.bit(4)
    }
    pub fn right_bumper(&self) -> bool {
        self.bit(5)
    }
    /// Left drive wheel dropped/floating (no ground). **[verified]** by pushing
    /// the left wheel into the body — bit 6 clears.
    pub fn left_wheel_floating(&self) -> bool {
        self.bit(6)
    }
    /// Right drive wheel dropped/floating. **[verified]** — bit 7 clears when the
    /// right wheel is pushed in.
    pub fn right_wheel_floating(&self) -> bool {
        self.bit(7)
    }
    /// Cliff / floor-sensor bits (`raw[1]`). The W10 has **six** downward sensors
    /// at bits 8-13 (baseline `0x3f` when lifted); a bit is set when its sensor
    /// sees no floor (a fall edge, or a lift). **[verified]** by covering each in
    /// turn: **bit 8 = front-left, bit 11 = front-right, bit 12 = rear-left, bit 13
    /// = rear-right**. Bits 9 and 10 are two more floor sensors, positions not
    /// isolated (they are NOT the wheels — the wheel drop sensors are `raw[0]` bits
    /// 6/7, confirmed by pressing each wheel).
    pub fn cliff_flags(&self) -> u8 {
        self.raw[1]
    }
    /// True if any cliff / floor sensor reports no floor (a fall edge or a lift).
    pub fn any_cliff(&self) -> bool {
        self.raw[1] != 0
    }
    pub fn cliff_front_left(&self) -> bool {
        self.bit(8)
    }
    pub fn cliff_front_right(&self) -> bool {
        self.bit(11)
    }
    pub fn cliff_rear_left(&self) -> bool {
        self.bit(12)
    }
    pub fn cliff_rear_right(&self) -> bool {
        self.bit(13)
    }
    pub fn ir_dock_lf(&self) -> u8 {
        self.ir3(16)
    }
    pub fn ir_dock_lmf(&self) -> u8 {
        self.ir3(20)
    }
    pub fn ir_dock_rmf(&self) -> u8 {
        self.ir3(24)
    }
    pub fn ir_dock_rf(&self) -> u8 {
        self.ir3(28)
    }
    pub fn dock_sta(&self) -> bool {
        self.bit(32)
    }
    pub fn lds_button1(&self) -> bool {
        self.bit(33)
    }
    pub fn lds_button2(&self) -> bool {
        self.bit(34)
    }
    pub fn side_overcurrent(&self) -> bool {
        self.bit(40)
    }
    pub fn roll_overcurrent(&self) -> bool {
        self.bit(41)
    }
    pub fn fan_overcurrent(&self) -> bool {
        self.bit(42)
    }
    pub fn pump_overcurrent(&self) -> bool {
        self.bit(43)
    }
    pub fn left_wheel_overcurrent(&self) -> bool {
        self.bit(44)
    }
    pub fn right_wheel_overcurrent(&self) -> bool {
        self.bit(45)
    }
    pub fn lidar_error(&self) -> bool {
        self.bit(48)
    }
    pub fn fan_error(&self) -> bool {
        self.bit(49)
    }
    pub fn left_vel_error(&self) -> bool {
        self.bit(50)
    }
    pub fn right_vel_error(&self) -> bool {
        self.bit(51)
    }
    pub fn imu_error(&self) -> bool {
        self.bit(54)
    }
    pub fn charge_error(&self) -> bool {
        self.bit(55)
    }
}

/// `0x2B` — battery/charge. `<H H h H h H>`.
/// `0x2b` — battery/charge. `<H H h H h H>` (12 B on the W10). Verified live on
/// the dock (16.3 V / 25 °C / charge 19.9 V) and off-dock (283 mA discharge).
/// **SOC@8 is a direct percent (0..100), not centi-percent** — reads 88% at
/// 16.03 V and 100% full on the dock.
#[derive(Debug, Clone, Copy, Default)]
pub struct Battery {
    pub voltage_mv: u16,
    pub current_ma: u16,
    pub temperature_ddeg: i16, // deci-degrees C
    pub charge_voltage_mv: u16,
    pub soc_pct: i16, // state of charge, percent (0..100)
}

impl Battery {
    pub const MIN_LEN: usize = 10;
    pub fn parse(p: &[u8]) -> Option<Self> {
        if p.len() < Self::MIN_LEN {
            return None;
        }
        Some(Self {
            voltage_mv: u16le(p, 0),
            current_ma: u16le(p, 2),
            temperature_ddeg: i16le(p, 4),
            charge_voltage_mv: u16le(p, 6),
            soc_pct: i16le(p, 8),
        })
    }
    #[inline]
    pub fn voltage_v(&self) -> f32 {
        self.voltage_mv as f32 / 1000.0
    }
    #[inline]
    pub fn soc_percent(&self) -> f32 {
        self.soc_pct as f32
    }
}

/// `0x0F` — MCU->SoC latency ping. The SoC must reply with a 0x0F pong; a
/// full-replacement driver has to answer these or the MCU flags a com fault.
#[derive(Debug, Clone, Copy, Default)]
pub struct Ping {
    pub timestamp: u32,
    pub com_delay: u32,
}

impl Ping {
    pub fn parse(p: &[u8]) -> Option<Self> {
        if p.len() < 8 {
            return None;
        }
        Some(Self {
            timestamp: u32le(p, 0),
            com_delay: u32le(p, 4),
        })
    }
}

/// Decoded message, or `Raw` for a type we don't model yet.
#[derive(Debug, Clone)]
pub enum Msg {
    Triggers(Triggers),
    Status20ms(Status20ms),
    Status10ms(Status10ms),
    Status100ms(Status100ms),
    Battery(Battery),
    Ping(Ping),
    Raw { typ: u8, len: usize },
}

impl Msg {
    /// Decode a `(type, payload)` pair (as returned by [`parse_body`]).
    pub fn decode(typ: u8, payload: &[u8]) -> Msg {
        match typ {
            T_TRIGGERS => Triggers::parse(payload).map(Msg::Triggers),
            T_STATUS_20MS => Status20ms::parse(payload).map(Msg::Status20ms),
            T_STATUS_10MS => Status10ms::parse(payload).map(Msg::Status10ms),
            T_STATUS_100MS => Status100ms::parse(payload).map(Msg::Status100ms),
            T_BATTERY => Battery::parse(payload).map(Msg::Battery),
            T_PING => Ping::parse(payload).map(Msg::Ping),
            _ => None,
        }
        .unwrap_or(Msg::Raw {
            typ,
            len: payload.len(),
        })
    }
}

// ===========================================================================
// Encoding (to MCU) — used later by the driver's write-back path
// ===========================================================================

/// Encode `[type][payload]` into a full escaped `<..>` wire frame in `out`.
/// Returns the number of bytes written, or `None` if `out` is too small.
pub fn encode_frame(typ: u8, payload: &[u8], out: &mut [u8]) -> Option<usize> {
    if payload.len() > 255 {
        return None;
    }
    // inner = [len][type][payload][crc_hi][crc_lo]
    let mut inner = [0u8; MAX_BODY];
    let mut n = 0usize;
    inner[n] = payload.len() as u8;
    n += 1;
    inner[n] = typ;
    n += 1;
    for &b in payload {
        inner[n] = b;
        n += 1;
    }
    let crc = crc16_modbus(&inner[..n]);
    inner[n] = (crc >> 8) as u8;
    n += 1;
    inner[n] = (crc & 0xff) as u8;
    n += 1;

    // wrap + escape
    let mut w = 0usize;
    let put = |byte: u8, out: &mut [u8], w: &mut usize| -> bool {
        if *w >= out.len() {
            return false;
        }
        out[*w] = byte;
        *w += 1;
        true
    };
    if !put(START, out, &mut w) {
        return None;
    }
    for &b in &inner[..n] {
        if b == START || b == END || b == ESCAPE {
            if !put(ESCAPE, out, &mut w) {
                return None;
            }
        }
        if !put(b, out, &mut w) {
            return None;
        }
    }
    if !put(END, out, &mut w) {
        return None;
    }
    Some(w)
}

/// `0x00` MotorCtrl (to MCU): `<B f f>` = flag, linear, rotational. **Verified
/// live on the W10 by watching what `ava` writes:** flag=1, `linear` in **mm/s**
/// (settles to the commanded speed), `rotational` in **rad/s** (negative =
/// clockwise; e.g. a 45 °/s right turn = −0.7854 rad/s).
pub fn encode_motor_ctrl(flag: u8, linear: f32, rotational: f32, out: &mut [u8]) -> Option<usize> {
    let mut p = [0u8; 9];
    p[0] = flag;
    p[1..5].copy_from_slice(&linear.to_le_bytes());
    p[5..9].copy_from_slice(&rotational.to_le_bytes());
    encode_frame(TX_MOTOR_CTRL, &p, out)
}

/// `0x01` SetCleaning: **6** actuator-level bytes. **Fully mapped by driving each
/// actuator directly (path 3, `mcud`) and watching the decoded currents / fan
/// sound:** `[0]` = side-brush, `[1]` = main-brush (roller), `[2]` = fan, `[3]` =
/// water pump, `[4]` = mode (`03` vacuum, `00` mop, `01` nav), `[5]` = 0. Evidence:
/// byte 0 raised `sidebrush_current@28`, byte 1 raised `roller_current@26`, byte 2
/// spun the fan (audible), byte 3 the pump (needs mop pads). `ava`'s vacuuming
/// frame is `55 6e 96 00 03 00` = side 85, main 110, fan 150, mode 3. Not emitted
/// while idle.
pub fn encode_set_cleaning(f: [u8; 6], out: &mut [u8]) -> Option<usize> {
    encode_frame(TX_SET_CLEANING, &f, out)
}

/// `0x02` SetButtonLED: LED state; also serves as the MCU heartbeat.
pub fn encode_set_led(state: u8, out: &mut [u8]) -> Option<usize> {
    encode_frame(TX_SET_LED, &[state], out)
}

// ===========================================================================
// Tests
// ===========================================================================

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn crc_roundtrip_and_frame() {
        // Encode a MotorCtrl, then de-frame + parse it back (alloc-free).
        let mut wire = [0u8; 64];
        let n = encode_motor_ctrl(0, -100.0, -0.9, &mut wire).unwrap();
        let mut sc = FrameScanner::new();
        let mut seen = false;
        for &b in &wire[..n] {
            if let Some(body) = sc.push(b) {
                let (typ, payload) = parse_body(body).unwrap();
                assert_eq!(typ, TX_MOTOR_CTRL);
                assert_eq!(payload.len(), 9);
                assert_eq!(payload[0], 0);
                let lin = f32::from_le_bytes([payload[1], payload[2], payload[3], payload[4]]);
                assert_eq!(lin, -100.0);
                seen = true;
            }
        }
        assert!(seen, "expected one decoded frame");
    }

    #[test]
    fn escaping_roundtrips_delimiter_bytes() {
        // Payload full of bytes that must be escaped on the wire.
        let mut wire = [0u8; 64];
        let n = encode_frame(0x7f, &[START, END, ESCAPE, 0x00], &mut wire).unwrap();
        let mut sc = FrameScanner::new();
        let mut seen = false;
        for &b in &wire[..n] {
            if let Some(body) = sc.push(b) {
                let (typ, payload) = parse_body(body).unwrap();
                assert_eq!(typ, 0x7f);
                assert_eq!(payload, &[START, END, ESCAPE, 0x00][..]);
                seen = true;
            }
        }
        assert!(seen, "expected one decoded frame");
    }

    #[test]
    fn known_status20ms_w10_offsets() {
        let mut p = [0u8; 30];
        p[20..22].copy_from_slice(&123i16.to_le_bytes()); // left_vel @ 20 (W10)
        p[22..24].copy_from_slice(&(-45i16).to_le_bytes()); // right_vel @ 22 (W10)
        let s = Status20ms::parse(&p).unwrap();
        assert_eq!(s.left_vel, 123);
        assert_eq!(s.right_vel, -45);
    }
}
