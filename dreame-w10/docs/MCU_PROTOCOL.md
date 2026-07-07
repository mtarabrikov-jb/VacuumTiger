# Dreame W10 (`r2104`) MCU protocol — reverse-engineering findings

The MCU is the microcontroller that drives the wheels/brushes/fan, reads the
IMU, dock IR, bumpers and battery, and talks to the SoC (`ava`) over a framed
binary link on **`/dev/ttyS4 @ 115200`**. This document records what has been
decoded on the W10, how it was verified, and what is still open.

Base reference: [`alufers/dreame_mcu_protocol`](https://github.com/alufers/dreame_mcu_protocol)
(reverse-engineered on the Z10 Pro). The W10 shares the framing and most message
types but several payload layouts differ — those differences are the point of
this file. The parser that implements everything below is
[`proto/src/lib.rs`](../proto/src/lib.rs).

Legend: **[verified]** confirmed live on this robot · **[partial]** decodes but
some fields/scales are wrong · **[unknown]** seen on the wire, not decoded.

## Hardware map (verified live)

- **MCU**: `/dev/ttyS4 @ 115200` — held by `ava` (fd 25). Framed binary protocol below.
- **LDS/LIDAR**: `/dev/ttyS3 @ 230400` — held by `ava` (fd 30), config node `AvaNodeLDS`
  in `/ava/conf/r2104.conf`. **Scan format decoded** — see [`LDS_PROTOCOL.md`](LDS_PROTOCOL.md).
  **The turret only spins during active navigation** — `lds-rx` is silent while the
  robot is idle/docked; enabling Valetudo manual control is enough to start it.
- **Actuators**: fan/brush/pump via MCU `SetCleaning` (0x01) and/or SoC `pwmchip0`
  (16 channels); exact mapping TBD.
- `ava` opens both ports from process start; a second opener steals bytes, hence
  the read-only tap (`avatap.so`) rather than a direct open.

## Framing [verified]

A packet is delimited by `<` (0x3c) .. `>` (0x3e). Inside, `?` (0x3f) escapes the
next byte verbatim (a literal `<`, `>`, `?` in the body is preceded by `?`). The
unescaped body is:

```
[len:u8] [type:u8] [payload: len bytes] [crc16: 2 bytes, big-endian hi,lo]
```

`crc16` is **CRC-16/Modbus** (poly 0xA001, init 0xFFFF) over `[len][type][payload]`.
Verified: **0 CRC errors over many thousands of frames** through the live tap.

## Messages from the MCU

Observed types and rates (measured via the tap; ratios cross-checked, e.g.
`n(0x02)/n(0x01) ≈ 2.0`, `n(0x01)/n(0x03) ≈ 5.0`):

| type | name | len | period | status |
|------|------|-----|--------|--------|
| 0x00 | Triggers | 7 | ~100 ms | [verified] |
| 0x01 | Status20ms (pose/velocity) | 30 | 20 ms | [verified] |
| 0x02 | Status10ms (IMU/odometry) | 18 | 10 ms | [verified] |
| 0x03 | Status100ms (tilt/currents) | 11 | 100 ms | **[verified]** currents; tilt [partial] |
| 0x05 | slow timer / RTC-like counter | 6 | ~500 ms | [partial] — monotonic, opaque |
| 0x0f | Ping (SoC pongs 0x0f) | 8 | ~500 ms | **[verified]** — `ava` replies with a 4 B pong |
| 0x12 | timestamped status | 7 | ~100 ms | [partial] — `u32` ts + opaque bytes (`1d 01` const tail) |
| 0x23 | dock/station status | 6 | ~100 ms | **[partial]** — `byte[2]` = dock tank flags: **bit 0 = clean-water tank, bit 2 = waste-water tank missing** [both verified by A-B, all-present = 0]; bit 1 unknown/unused (this W10 has no auto-empty dust bag). Docked-all-present = `10 00 00 00 00 42` |
| 0x24 | flag byte | 1 | ~500 ms | [partial] — `00` at idle |
| 0x2b | BatteryStatus | 12 | ~1 s | [verified] |
| 0x2c | slow cumulative counter | 10 | ~0.5 s | [partial] — `[1:3]` +2 over 12 s (uptime/stat?) |

### 0x01 Status20ms — **W10 layout, 30 bytes [verified]**

The W10 payload is the Z10 layout (26 B) with **4 reserved bytes inserted at
offset 16** (they read `0xA5A5A5A5` at rest), shifting the velocity/current
fields by +4:

| offset | field | type | notes / verification |
|--------|-------|------|----------------------|
| 0..3   | timestamp_us | u32 | monotonic counter |
| 4..7   | x | i32 (/10 mm) | **constant during in-place rotation** ✓ |
| 8..11  | y | i32 (/10 mm) | **constant during in-place rotation** ✓ |
| 12..13 | yaw | i16 (/100 °) | **swept −100°→−156° while rotating CW; decreases = CW** ✓ |
| 14..15 | yaw_integral | i16 | small at rest |
| 16..19 | **reserved** | — | `0xA5A5A5A5` at rest (the +4 vs Z10) |
| 20..21 | **leftVel** | i16 | **0 at rest; +35..+47 during CW rotation** ✓ |
| 22..23 | **rightVel** | i16 | **0 at rest; −35..−45 during CW rotation** ✓ |
| 24..25 | edgeDis | i16 | **0 even while cleaning** — unconfirmed (cliff/edge sensor?) |
| 26..27 | **roller_current** | i16 | **[verified]** ~0..4 at rest → ~478 with the main brush spinning |
| 28..29 | **sidebrush_current** | i16 | **[verified]** ~1 at rest → ~106 with the side brush spinning |

Verification: driving the robot in place via Valetudo (see Methodology) makes
leftVel/rightVel take **opposite signs** (left forward + / right back − = CW),
while x/y stay put and yaw sweeps monotonically — the unambiguous differential-
drive signature. Before the fix the old Z10 offsets read the reserved bytes and
reported `leftVel=rightVel=−23131` (=`0xA5A5`) at rest.

### 0x02 Status10ms — IMU + wheel odometry [verified]

Layout `<I h h h h h h b b>` (18 B): timestamp, gyro[3]@4/6/8, accel[3]@10/12/14,
leftDis@16, rightDis@17 (both `i8`). Verified live:

- **accel = raw LSB, ±2 g full scale → `/16384` LSB·g⁻¹** (NOT `/1000`). At rest
  the raw vector is `[90, −318, 16242]`, |v| ≈ 16246 ≈ 1 g, i.e. `[0.01, −0.02,
  0.99] g` — level robot, gravity on Z. Live decode now reads `accel_z ≈ 0.99 g`.
- **gyro = centi-deg/s (`/100`)** — rest ≈ 0; the yaw-rate axis is **index 2
  (offset 8)**: in-place rotation drove it to −2937 raw = −29 °/s while gyro_x/y
  stayed small.
- **leftDis@16 / rightDis@17 = signed per-packet wheel travel (mm/10 ms)** —
  range 0..1 at ~40 mm/s; over a forward-then-CW-rotate run summed to
  `+501 / −47` (forward adds to both; CW rotation adds to left, subtracts right).

### 0x03 Status100ms — tilt / wheel currents [verified currents]

W10 payload is **11 bytes** (Z10 was 9). Resting frame:
`b0 ff 25 00 02 00 04 00 01 00 12`.

| offset | field | i16 at rest | i16 rotating in place | verdict |
|--------|-------|-------------|-----------------------|---------|
| 0..1   | pitch | ~-80 | ~-110 | [partial] deci-deg assumed; ~-8° = dock-ramp tilt |
| 2..3   | roll  | ~+14 | ~-13 | [partial] shifts under motion |
| 4..5   | **left_current**  | ~0 | **310..430** | **[verified]** |
| 6..7   | **right_current** | ~0 | **297..370** | **[verified]** |
| 8..9   | load/current | ~0 (±1) | vac 1..23; **mop 27..343** | [partial] pump/mop-load candidate; **not** the Z10 flag bitfield |
| 10     | **flags** | `0x00` (bin in) | `0x01` (bin out) | **[verified]** consumables/attachment bitfield — **bit 0 = dustbin missing** (bin in/out A-B test); other bits track mop/tank (`0x12` no-mop → `0x00` with mop) |

Verification: an in-place rotation via Valetudo manual control (wheels only, no
pump) makes `left_current@4` and `right_current@6` jump from ~0 to ~300-430 while
every other field barely moves — the unambiguous "both wheels drawing current"
signature. This corrects the earlier `[partial]`: the currents are the Z10
offsets (4/6) after all. Byte 8 is a small signed load/current value (near 0 at
rest, ~1-23 vacuuming, **~27-343 while mopping** — a pump/mop-load candidate), so
it is **not** the Z10 consumable-flag bitfield. The consumable/attachment flags
are at **byte 10**: an A-B test (pull the bin, reinsert it, mop unchanged) flipped
`byte[10]` bit 0 (`0x00` bin-in → `0x01` bin-out) with nothing else stable
changing, so **`byte[10]` bit 0 = dustbin missing [verified]**. The byte also read
`0x12` before the mop was attached and `0x00` after, so its other bits carry
mop/tank state (exact bits still to be split — pull the water tank to map them).
Pitch/roll are also available from the 0x02 accelerometer gravity
vector.

### 0x00 Triggers — bit flags [verified live]

7-byte bitfield, **~10 Hz**; global bit `k` = `(raw[k/8] >> (k%8)) & 1`. Bumper,
cliff and lift are therefore all polled together at 10 Hz (the immediate safety
reaction is on the MCU itself). Live A-B on the W10 this session:

- **bit 4 = left bumper, bit 5 = right bumper** — pressing each set `raw[0]` to
  `0x10` / `0x20`.
- **bit 6 = left wheel, bit 7 = right wheel** (drop/float) — both set
  (`raw[0]=0xc0`) when lifted; **[verified L/R]** by pushing each drive wheel into
  the body: bit 6 cleared for the left wheel, bit 7 for the right.
- **`raw[1]` = six downward floor sensors (bits 8-13)** — `0x00` on the floor,
  `0x3f` when lifted (no floor under any). Covering each cliff sensor in turn
  mapped the four cliffs: **bit 8 = front-left, bit 11 = front-right, bit 12 =
  rear-left, bit 13 = rear-right**. Bits 9 and 10 are two more floor sensors,
  positions not isolated (they are **not** the wheels — those are bits 6/7 above).
- **bit 32 = `dock_sta`** — 1 docked, clears when lifted off.
- **Caveat:** bits 16-18 read `0b111` both docked AND lifted, so they are **not**
  dock-presence IR — the Z10 `ir_dock*` decode in `proto` is suspect on the W10.

Rest on dock = `00 00 07 00 01 00 00`. Fault flags (overcurrents, lidar/vel/imu/
charge errors) are per [`proto`](../proto/src/lib.rs), from the Z10 analogy
(not individually W10-verified).

### 0x2b BatteryStatus [verified]

Layout `<H H h H h H>` (12 B): voltage_mv@0, current_ma@2, temp@4 (/10 °C),
charge_voltage_mv@6, **soc@8 = direct percent (0..100)**, then 2 bytes. Verified:
on the dock **16.33 V / 25.0 °C / charge 19.86 V / 100 %**; off-dock **16.08 V,
356 mA discharge, charge 0.22 V, 86 %** — SOC tracks a 4S Li-ion curve. The W10
SOC is a plain percent, not centi-percent (the Z10 `/100` was wrong here).

## Messages to the MCU (from `ava`)

From `alufers/dreame_mcu_protocol` + our RE. **Several TX types are now captured
and verified live** on the W10 via the `mcu-tx` tap (marked below); the rest are
Z10 reference, not yet observed. Encoders are in [`proto`](../proto/src/lib.rs):

| type | name | payload | notes |
|------|------|---------|-------|
| 0x00 | MotorCtrl | `<B f f>` = flag, linear, rotational | **[verified]** flag=1; linear **mm/s**, rotational **rad/s** (neg = CW); ~50 Hz keepalive at 0 when idle |
| 0x01 | SetCleaning | **6 bytes** | **[verified live — fully mapped]** by driving each actuator under `mcud` (path 3) and watching currents / fan sound: **`[0]`=side-brush, `[1]`=main-brush (roller), `[2]`=fan, `[3]`=water pump, `[4]`=mode** (`03` vacuum / `00` mop / `01` nav). byte 0→`sidebrush_current@28`, byte 1→`roller_current@26`, byte 2→fan (audible), byte 3→pump. Vacuuming `55 6e 96 00 03 00` = side 85 / main 110 / fan 150 / mode 3 |
| 0x02 | SetButtonLED | `<B>` | **[verified live]** LED-state enum: `0x21` idle, `0x02` after Locate, `0x04` during mop-dock clean; also the MCU heartbeat |
| 0x0f | Pong | 4 bytes | **[verified live]** `ava`'s reply to the MCU `0x0f` ping (echoes the ping payload) |
| 0x14 | nav/lidar flag | `<B B>` | **[verified]** `[1]`=1 keeps the LDS turret spinning; **`04 00` (idle) halts it**. Sent continuously in nav |
| 0x1d | Laser/ToF enable | `<B B>` | **[verified]** `05 01` re-pulse (~every 4 s in nav) — part of keeping the lidar on |
| 0x26 | nav/lidar status | 8 bytes | **[verified]** `[0]`=`0x64` idle → `0x14` in nav (lidar on); `[7]`=`0x04` const |
| — | **lidar on** | — | **[verified under mcud]** stream `0x14 04 01` + `0x26 14 ..` continuously + `0x1d 05 01` re-pulse → turret spins (~220 pkt/s on ttyS3); revert to idle `0x14 04 00` → stops |
| 0x04 | SetOdometer | `<B I I I b>` | Z10 ref, not observed |
| 0x11 | SetLDSCalibration | `<f f f>` | Z10 ref, not observed |
| 0x1f | CalibrateIMU | `<B>` | Z10 ref, not observed |

The MCU `0x0f` ping / `ava` `0x0f` pong exchange is **confirmed live** (the SoC
answers every ping). A full-replacement driver must reproduce that pong and
sustain the `0x02` LED/heartbeat, or the MCU flags a com fault.

## Methodology (how to reproduce / extend)

**Drive the robot** via Valetudo `HighResolutionManualControlCapability`
(`PUT /api/v2/robot/capabilities/HighResolutionManualControlCapability`):

```
{"action":"enable"}
{"action":"move","vector":{"velocity":<v>,"angle":<a>}}   # repeat < 700 ms (keepalive)
{"action":"disable"}
```

Dreame maps `spdv = round(velocity*300)` (mm/s) and `spdw = round(-angle)`
(deg/s). A **700 ms watchdog** auto-stops if you stop sending. **Rotation in
place** (`velocity:0, angle:±15`) is the cleanest calibration input: it splits
left/right wheels by sign and never translates off the dock.

**Capture** with `w10-decode` against the relay:
- `--mcu` — live decoded summary (odometry/IMU/battery/triggers).
- `--watch` — per-offset volatility (min/max/`span` per byte) across all types;
  good for a first look, but **1 Hz sampling aliases** the 50–100 Hz messages.
- `--log 0x01` — every frame of one type at full rate (`ms  payload-hex`); this
  is what pinned down the 0x01 offsets. Drive one clean single-axis motion and
  find the i16 that behaves right (0 at rest, opposite signs when rotating).

## Open items

Done this pass: **0x03 wheel currents** (in-place rotation) and **0x01 main/side
brush currents** (a cleaning run), **SetCleaning/SetButtonLED/Pong** TX (captured
live), the **0x12** type (new), the **dustbin flag** (`0x03[10]` bit 0) and the
**dock water-tank flags** (`0x23[2]` bit 0 = clean, bit 2 = waste) — all by A-B
removal tests — and the **LDS** scan format (see
[`LDS_PROTOCOL.md`](LDS_PROTOCOL.md)). Remaining:

1. **0x03** flags@10 — bit 0 = dustbin [verified]; the remaining bits are the
   **mop** attachment (the byte went `0x12`→`0x00` when the mop was attached; the
   dock water tanks live in `0x23`, not here). Pitch/roll units are low priority
   (derivable from the 0x02 accel gravity vector).
2. **0x01** `edgeDis@24` reads 0 even while cleaning (cliff/edge sensor?) —
   `roller_current@26` / `sidebrush_current@28` are now [verified].
3. **0x23** `byte[2]` dock tank flags — clean-water = bit 0, waste-water = bit 2
   (both [verified]); bit 1 unknown/unused (this W10 has no auto-empty dust bag).
   Other opaque types: **0x05** (slow timer/RTC), **0x12** (timestamped status),
   **0x24** (flag byte), **0x2c** (slow counter), and TX **0x14 / 0x26**.
4. **SetCleaning per-level scaling** — the byte roles are mapped (`[3]`=water/pump,
   `[0..3]`=fan/brush, `[4]`=mode), but the low/med/high value per level is not,
   because `ava` re-sends this frame only at clean start, not on a mid-clean preset
   change.
