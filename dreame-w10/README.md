# dreame-w10 — a VacuumTiger bridge for the Dreame Bot W10 (`r2104`)

VacuumTiger's model is to **replace** the robot's firmware: SangamIO opens the
motor-controller MCU and the LIDAR directly and runs its own SLAM. On the Dreame
W10 that firmware is `ava`, a proprietary navigation daemon that **exclusively
owns** both serial ports and also implements charging, cliff/bump safety and
docking. Ripping it out is possible but loses all of that.

This bridge takes the low-risk path first: **run on top of a working `ava`**.
We tap `ava`'s serial I/O read-only and re-expose it so a VacuumTiger
`dreame_w10` driver can consume real odometry / IMU / LDS while `ava` keeps the
robot safe. Everything here is Rust, matching the rest of VacuumTiger.

## Hardware map (verified on the robot)

- **MCU** — `/dev/ttyS4 @ 115200`. Framed binary protocol (`<len type payload
  crc16>`); gives wheel velocities, IMU, wheel/dock IR, currents, battery;
  accepts `MotorCtrl` (linear+rotational velocity), `SetCleaning`
  (fan/brushes/mop-pads) and `SetButtonLED` (also the heartbeat). Decoded in
  [`proto/`](proto/src/lib.rs); full findings in
  [`docs/MCU_PROTOCOL.md`](docs/MCU_PROTOCOL.md).
- **LDS/LIDAR** — `/dev/ttyS3 @ 230400` (config node `AvaNodeLDS`). Scan format
  reverse-engineered (40-byte packets, 8 range samples each); decoded in
  [`proto/src/lds.rs`](proto/src/lds.rs), findings in
  [`docs/LDS_PROTOCOL.md`](docs/LDS_PROTOCOL.md).
- **Actuators** — fan/brush/mop-pads run off the MCU `SetCleaning` command and/or the
  SoC `pwmchip0` (16 channels); exact mapping TBD (the robot has no water pump — the
  dock does; see [`docs/MCU_PROTOCOL.md`](docs/MCU_PROTOCOL.md)).
- **Base station (dock)** — a standalone GD32 MCU with its own firmware, LCD,
  wash/dry pumps + heater + fan, sub-GHz radio and ymodem bootloader; the robot talks
  to it over an `AA 55 …` RF frame. Reverse-engineered in
  [`docs/DOCK_PROTOCOL.md`](docs/DOCK_PROTOCOL.md).
- `ava` holds ttyS4 (fd 25) and ttyS3 (fd 30); a second opener would steal bytes,
  hence the tap rather than a direct open.

## Architecture

```
 ava (nav brain, LD_PRELOAD=…:avatap.so)          host / robot
 ├─ read(ttyS4) ─┐                          ┌── TCP 7701 mcu-rx ─┐
 ├─ write(ttyS4)─┤  avatap.so    /tmp/       │   TCP 7703 mcu-tx  │  w10-decode
 ├─ read(ttyS3) ─┼─► mirror ──► avatap.shm ─►│   TCP 7702 lds-rx  ├─►  or the
 └─ write(ttyS3)─┘  (ring bufs)   (tmpfs)    │   TCP 7704 lds-tx  │  SangamIO
                                  avatap-relay└────────────────────┘  dreame_w10
```

- **`avatap`** (`no_std` cdylib) — `LD_PRELOAD`ed into `ava`; interposes libc
  `open`/`read`/`write`/`close`, and when `ava` opens ttyS4/ttyS3 mirrors every
  byte (both directions) into `avatap_shm` rings. Lock-free, no syscalls on the
  hot path → `ava`'s real-time behavior is unaffected, safe during motion. Built
  against old glibc so it loads on the robot's glibc 2.23.
- **`avatap-relay`** (static-musl bin) — drains the rings and serves each channel
  over TCP. Runs outside `ava`, so a bug here can't crash navigation. Read-only:
  it never writes to a device.
- **`proto`** (`no_std` lib) — the MCU protocol (framing, CRC16-Modbus, message
  parsers, command encoders). Shared by the tap tooling and, later, the driver.
- **`w10-decode`** — host CLI that decodes the live streams to validate parsing.
- **`shm`** — the `repr(C)` ring layout shared by tap and relay.

## Milestones

1. **Tap + validate (this stage).** Mirror ttyS4/ttyS3, decode MCU telemetry
   live, confirm the W10 field offsets, capture raw LDS bytes. No `ava` changes
   beyond the preload. *Done so far:* framing/CRC verified (0 errors); the
   30-byte `Status20ms` fully decoded and **verified by driving** (leftVel/
   rightVel/yaw/x/y); Triggers + Battery largely validated. Remaining: 0x02
   accel scale, 0x03, battery SOC — see [`docs/MCU_PROTOCOL.md`](docs/MCU_PROTOCOL.md).
2. **LDS scan format.** *Done:* the `lds-rx` stream is decoded — fixed 40-byte
   packets (`55 aa 03 08` sync), 8 samples each of `u16` distance (mm) + `u8`
   quality, with per-packet start/end angles interpolated linearly; turret speed
   and validity flags too. Verified live end-to-end via `w10-decode --lds`, and
   the decode re-checked against an active-navigation capture. Coverage is a
   **fixed ~126 deg rear arc** (~226-352 deg) — identical under manual control and
   active SLAM (a return-to-dock run), so this ttyS3 link carries no full circle.
   Why only a sub-arc, the angle scale, and the checksum scheme are the open
   items. See [`docs/LDS_PROTOCOL.md`](docs/LDS_PROTOCOL.md).
3. **`dreame_w10` DeviceDriver** in
   [`sangamio/src/devices/dreame_w10`](../sangamio/src/devices/dreame_w10/mod.rs).
   *Done:* consumes the relay's `mcu-rx` channel and publishes a `sensor_status`
   group (odometry, IMU, wheel travel, bumpers, dock, battery) — **verified live
   streaming through SangamIO over UDP**. Read-only for now (`ava` keeps motor
   control; teleop is via Valetudo). Config: `sangamio/config/dreame_w10.toml`;
   point it at a remote robot with `W10_MCU_ADDR=<ip>:7701` (no device address in
   the repo). LDS group is reserved until the scan format is decoded.
4. **Optional full replacement** once the MCU heartbeat/enable and LDS are fully
   understood.

## Build & run

```sh
make check     # host: build all crates + proto tests
make docker    # cross-compile out/avatap.so + out/avatap-relay (aarch64)
make upload    # push to the robot (ROBOT=root@<ip>, tar-over-ssh)
make inject    # LD_PRELOAD the tap + start the relay  (RESTARTS ava — idle on dock)
make decode    # decode live MCU telemetry;  make lds = decode the LIDAR scan
make remove    # tap off, relay stopped, camera/stock ava restored
```

The tap is read-only and coexists with the [camera livestream](../../dreame-vacuum-livestream)
tap (it composes into the same `ava` wrapper). Injecting restarts `ava` once; it
is a dev/RE tool and is intentionally not made boot-persistent.
