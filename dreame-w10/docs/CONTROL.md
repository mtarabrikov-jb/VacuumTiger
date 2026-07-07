# Control mode (path 2) — driving the W10 by injecting into the tap

The read-only bridge observes `ava`; **control mode** lets an out-of-`ava` client
*drive* the robot without replacing `ava`, by having the `avatap` tap **substitute
`ava`'s `MotorCtrl` (0x00) frame** on `/dev/ttyS4` with a commanded velocity.

This is path 2 of three (see [`../README.md`](../README.md)):
1. drive via Valetudo -> `ava` (safe, but `ava` mediates);
2. **inject/filter in the tap (this doc)** — real control, `ava` still handles
   charging/LDS/docking;
3. full `ava` replacement (later).

## Why filter, not inject a second writer

`ava` exclusively owns `ttyS4` and already writes `MotorCtrl` at **~50 Hz**. A
second writer would race it for the motors. Instead the tap **rewrites `ava`'s own
MotorCtrl in flight**: we ride `ava`'s 50 Hz cadence (the MCU watchdog stays fed),
and every other frame (SetLED heartbeat, `0x0f` pong, SetCleaning) passes through
untouched. With no active command the tap is a byte-identical mirror.

## Data path

```
control client --TCP 7705--> avatap-relay --writes--> shm.Control
                                                         |
ava write(ttyS4, MotorCtrl) --> avatap intercept --reads shm.Control-->
   if enabled & fresh & safe: write our re-encoded MotorCtrl instead
   else: pass ava's frame through
```

- **shm `Control`** (`shm/src/lib.rs`): `enabled`, `seq`, `linear` (f32 mm/s),
  `rot` (f32 rad/s), plus `overrides` / `hazard_clamps` diagnostics.
- **relay control port `7705`** (`avatap-relay`): newline text —
  `"<linear_mm_s> <rot_rad_s>"` sets & keeps alive (send at >=2 Hz), `"stop"`
  disables. Disconnect releases control. One controller at a time.
- **tap override** (`avatap`): on a MotorCtrl write (`< 09 00 ...`), if control is
  enabled and the watchdog is fresh, re-encode via `encode_motor_ctrl` and write
  that; the MCU drives the motors from it.

## Safety (all enforced in the tap)

- **Watchdog** — the client must keep bumping `seq`; after `WATCHDOG_WRITES` (25 ~
  0.5 s at 50 Hz) MotorCtrl writes with an unchanged `seq`, the tap reverts to
  passthrough and `ava` resumes. No clock needed — `ava`'s cadence is the clock.
- **Enable gate** — `enabled == 0` -> passthrough (default).
- **Speed clamp** — `MAX_LINEAR_MM_S = 150`, `MAX_ROT_RAD_S = 1.5`.
- **Hazard gate** — the tap tracks the latest Triggers (0x00) from the read path;
  if any bumper (bits 4/5), wheel-float (6/7) or cliff/floor sensor (`raw[1]`) is
  active, forward motion is clamped to 0.
- **Auto-release** — the relay sets `enabled = 0` when the control client
  disconnects.

## Validated live (2026-07-07)

- **Injection:** zero-velocity command -> `overrides = 151` over 3 s (~50 Hz,
  every MotorCtrl frame overridden), robot did not move.
- **Motion:** `30 mm/s x 1.5 s` on the floor -> both wheels spun forward
  (leftVel/rightVel 0->~65), odometry moved ~49 mm.
- **Hazard gate:** `40 mm/s` while held aloft -> `hazard_clamps = 150` (all
  overrides clamped to 0), wheels did not drive forward.
- **Watchdog:** froze the command stream mid-drive (socket left open) -> robot
  stopped ~0.6 s later as the tap reverted to `ava`.

## Usage

```sh
make docker && make upload && make inject   # deploy tap+relay (restarts ava)
# drive: linear mm/s, rot rad/s, seconds (10 Hz keepalive, then stop)
python3 drive.py <robot-ip> 30 0 1.5
```

Test only on open floor away from stairs — the hazard gate covers detected
cliffs/bumps, not everything.
