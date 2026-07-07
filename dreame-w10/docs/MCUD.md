# Path 3 — `w10-mcud`: driving the W10 with `ava` fully replaced

Path 2 rides `ava`'s MotorCtrl (see [`CONTROL.md`](CONTROL.md)). **Path 3** removes
`ava` entirely: `w10-mcud` (`../mcud`) stops `ava`, opens `/dev/ttyS4` (MCU) and
`/dev/ttyS3` (LDS) itself, and becomes the sole controller.

## What it does

- **Sustains the MCU** so the board never faults: streams `MotorCtrl` (0x00) at
  ~50 Hz, replays the periodic `SetLED`/`SetCleaning`/`0x14`/`0x26` frames, and
  answers every `0x0f` **ping with a pong echoing the ping's first 4 bytes** (the
  com-fault handshake).
- **Drives** from a control client (text over TCP 7705, same protocol as the
  relay) with a command watchdog, a speed clamp and a live cliff/bumper hazard
  gate read from the MCU's Triggers.
- **Re-serves telemetry**: MCU on 7701, raw LDS on 7702 (so `w10-decode` works).

`mcud.sh start` enters this mode (stops the `ava` respawner, kills `ava`, runs
`w10-mcud`); `mcud.sh restore` brings `ava` + the tap relay back. Not
boot-persistent.

## Control protocol (TCP 7705, newline text)

| command | effect |
|---------|--------|
| `<linear_mm_s> <rot_rad_s>` | drive (watchdog-fed; send at >=2 Hz), e.g. `30 0` |
| `stop` | stop driving |
| `lidar 0` / `lidar 1` | spin the LDS turret off / on |
| `frame <type_hex> <payload_hex...>` | send one arbitrary MCU frame (actuator RE/control) |

## Actuators — SetCleaning (0x01), mapped live under mcud

Probed by sending each byte via `frame 01 ...` and watching the decoded currents
(main/side-brush) or listening (fan):

| byte | actuator | evidence |
|------|----------|----------|
| 0 | **side brush** level | `sidebrush_current@28` (0x01) rose 4 -> 128 |
| 1 | **main brush** (roller) level | `roller_current@26` rose 4 -> 472 |
| 2 | **fan** (suction) level | audible spin-up (no current telemetry) |
| 3 | **water pump** level | `load@8` (0x03); needs mop pads installed |
| 4 | mode | `03` vacuum, `00` mop, `01` navigation |
| 5 | (0) | — |

Cross-check: `ava`'s vacuuming frame `55 6e 96 00 03 00` = side 85, main 110, fan
150, mode 3. So e.g. `frame 01 00 6e 00 00 00 00` runs only the main brush.

## Lidar

`ava` writes nothing to ttyS3 — the turret is driven by ttyS4 frames. It spins
while the **nav values** are sent continuously: `0x14 = 04 01`, `0x26 = 14 ..`,
plus a `0x1d 05 01` re-pulse every few seconds. The **idle** `0x14 = 04 00` stops
it (a single idle frame halts the turret, which is why one-shot enables only
nudged it). `mcud` handles this: `lidar 1` makes its periodic loop emit the nav
values; `lidar 0` returns to idle. Verified: `lidar 1` -> ~220 LDS packets/s on
7702; `lidar 0` -> silent.

## Validated live (2026-07-07)

- MCU stays healthy with no `ava` (full telemetry, 0 CRC errors, no Triggers error
  flags — pong + heartbeats accepted).
- Motors respond: `30 mm/s x 1.5 s` moved the robot ~48 mm.
- Actuators: main/side brush and fan run on demand (currents / audible).
- Lidar: `lidar 1` streams a continuous scan (887 packets in 4 s).

## Remaining for a full replacement

SLAM (turning LDS scans into a map + localization), docking/undocking, and
charging orchestration — all previously `ava`'s job. `mcud` v1 is teleop + actuator
control only.

## Usage

```sh
make docker && make upload
make mcud-start        # KILLS ava (camera/Valetudo down); mcud drives the MCU
python3 teleop.py <robot-ip>          # keyboard teleop
# actuators / lidar, via the control port:
printf 'lidar 1\n'            | nc <robot-ip> 7705   # spin the lidar
printf 'frame 01 00 6e 00 00 00 00\n' | nc <robot-ip> 7705   # main brush on
make mcud-restore      # stop mcud, restore ava + relay
```
