# Dreame Bot W10 (r2104) driver

SangamIO device driver for the Dreame Bot W10 (Allwinner MR813, aarch64, Linux 4.9, glibc 2.23). The W10 ships a proprietary navigation daemon, `ava`, that owns the MCU (`/dev/ttyS4`) and the LDS lidar (`/dev/ttyS3`). The driver (`src/devices/dreame_w10/`) runs in one of two modes, selected by the `gd32_port` value in the config:

- **over-ava** (`gd32_port` = `host:port`, e.g. `127.0.0.1:7701`): read-only. SangamIO does not open the serial ports; it connects to the read-only tap relay (`avatap-relay`, from `../dreame-w10`) and decodes the mirrored MCU stream while `ava` keeps driving the robot. Config: `config/dreame_w10.toml`.
- **direct** (`gd32_port` = `/dev/ttyS4`): full replacement. With `ava` stopped, SangamIO opens the serial ports and drives the MCU itself (MotorCtrl at 50Hz, `0x0f` ping/pong, SetLED/SetCleaning heartbeats, lidar enable frames), translating client commands into robot actions. Config: `config/dreame_w10_direct.toml`.

This doc covers direct mode: build, deploy, test, restore.

## Build (aarch64 static musl)

The robot's glibc is old (2.23), so the daemon is built as a fully static musl binary (no glibc dependency). Requires Docker.

```
sangamio/build/build-aarch64.sh
```

Output: `sangamio/build/out/sangamio` (ELF aarch64, statically linked, stripped, ~925 KB). The script assembles a minimal Docker context (the daemon crate plus its `dreame-w10-proto` path dependency, excluding `target/`) and runs `build/Dockerfile`. See the Dockerfile header for why musl (static, glibc-independent), why a host `cc` is needed (build scripts / proc-macros target the build host), and why `protoc` is installed (prost-build compiles `proto/sangamio.proto`).

## Deploy and test

Direct mode replaces `ava`: the camera stream, Valetudo, and ava's own cliff/bump safety and docking go down while SangamIO drives the MCU. SangamIO runs its own watchdog (500ms), speed clamp (150mm/s, 1.5rad/s), and cliff/bump hazard gate. Fully reversible. Keep the robot on the dock with clear floor ahead.

1. Upload the binary and config to the robot (`$ROBOT` = `root@<robot-ip>`, kept out of the repo). tar-over-ssh, no scp:

   ```
   tar -C sangamio/build/out -cf - sangamio | ssh "$ROBOT" 'tar -C /data/w10bridge -xf - && chmod +x /data/w10bridge/sangamio'
   tar -C sangamio/config -cf - dreame_w10_direct.toml | ssh "$ROBOT" 'tar -C /data/w10bridge -xf -'
   ```

2. Enter direct mode (stops `ava`, starts SangamIO):

   ```
   ssh "$ROBOT" 'REMOTE_DIR=/data/w10bridge sh /data/w10bridge/w10-direct.sh start'
   ```

   (`build/w10-direct.sh` performs the ava stop/restore dance; upload it to `/data/w10bridge` too.) The log at `/data/log/sangamio.log` should show `Dreame W10 (direct): driving MCU /dev/ttyS4, LDS /dev/ttyS3`, `MotorCtrl 50Hz + pong + heartbeats`, and `TCP server listening on 0.0.0.0:5555`.

3. Send commands from any host that can reach the robot, using the test client (`tools/w10cli.py`, hand-encoded protobuf, no protoc/stubs needed):

   ```
   tools/w10cli.py <robot-ip> lidar on            # spin the LDS turret
   tools/w10cli.py <robot-ip> on side_brush       # ENABLE -> 100%
   tools/w10cli.py <robot-ip> speed vacuum 60     # CONFIGURE speed 0-100
   tools/w10cli.py <robot-ip> off main_brush
   tools/w10cli.py <robot-ip> drive 0.05 0 --hold 1.0   # creep 50mm/s for 1s, then stop
   tools/w10cli.py <robot-ip> stop
   ```

   Actuator state latches in the daemon and persists after the client disconnects (the tx loop keeps re-sending it). Drive is watchdog-gated: without a fresh command every 500ms the motors stop, so `drive` resends at 10Hz for `--hold` seconds then sends an explicit stop.

4. Restore `ava` (camera / Valetudo back):

   ```
   ssh "$ROBOT" 'REMOTE_DIR=/data/w10bridge sh /data/w10bridge/w10-direct.sh restore'
   ```

## Command reference

Client sends `Command.ComponentControl { id, action }` (see `proto/sangamio.proto`). Actions: `Enable` / `Disable` / `Configure` / `Reset`.

- `drive` - `Configure { linear: f32 (m/s), angular: f32 (rad/s) }`. `Disable` / `Reset` stop. Negative angular = clockwise.
- `side_brush`, `main_brush`, `vacuum`, `water_pump` - `Enable` (100%), `Disable` (0%), or `Configure { speed: 0-100 }`. Mapped to the MCU SetCleaning frame (`0x01`), bytes `[side, main, fan, pump, mode, ...]`; the driver sets `mode=vacuum` when the fan runs.
- `lidar` - `Enable` / `Disable`. Enable sustains the turret via the nav frames `0x14 04 01` + `0x26` + periodic `0x1d 05 01`; disable sends `0x14 04 00`, which stops it. LDS scans then stream to the `lidar` sensor group.

Note: `led` is sent as a fixed heartbeat only (`0x02 21`); it is not a client-controllable component in direct mode.

## Live validation

Direct mode was validated on the physical robot on 2026-07-07: with `ava` stopped, SangamIO held the MCU healthy (no com-fault), and every command actuated correctly - lidar turret, side brush, main brush (roller), vacuum fan, and a forward drive pulse (~5cm, auto-stopped by the 500ms watchdog). No protobuf parse or serial errors. Behavior is identical to the `w10-mcud` standalone driver (`../dreame-w10`), whose MCU/LDS protocol this driver reuses via `dreame-w10-proto`.

## Lidar and SLAM (dhruva-slam)

The `lidar` sensor group publishes `scan` as a `PointCloud2D` of `(angle_rad, distance_m, quality)`, robot-centered: each raw LDS angle goes through `frame_transforms.lidar` (`AffineTransform1D`) then `lidar_mounting.transform_to_robot_center`, the same pipeline as the Revo LDS driver that dhruva-slam already consumes. **Coverage is a fixed ~126 deg rear arc** (raw 225-352 deg), not a full circle - dhruva-slam accepts partial scans (min 50 points), but scan-matching is weaker than a 360 deg lidar.

Inspect the raw scan with the receiver tool (registers over TCP, decodes the UDP `scan`, prints a 30-degree coverage histogram):

```
tools/lidar_rx.py <robot-ip> [seconds]
```

Run the SLAM consumer (dhruva-slam) against the robot. The SangamIO address is set in the config file's `[source] sangam_address` (there is no `--sangam` flag); copy `dhruva-slam.toml`, point it at the robot, and give it a writable `[map_storage] path`:

```
sed -e 's#sangam_address = "localhost:5555"#sangam_address = "<robot-ip>:5555"#' \
    dhruva-slam.toml > dhruva-robot.toml
dhruva-slam -c dhruva-robot.toml
```

Validated end-to-end 2026-07-07: with SangamIO in direct mode and `lidar on`, dhruva-slam connects, streams UDP, and logs `Lidar scan received: ~270 points` at ~5 Hz (plus wheel odometry from `sensor_status`); no crashes.

### Frame calibration

The driver publishes each point as `robot_angle = frame_transforms.lidar.apply(raw)` then `lidar_mounting.transform_to_robot_center(...)`, where `apply(x) = scale*x + offset`. `scale` fixes the rotation direction (the LDS spins CW, so `scale=-1` gives ROS CCW); `offset` rotates the raw-sensor zero to robot forward. Target ROS frame (REP-103): **forward=0, left=90, back=180, right=270 deg, CCW positive**. Calibration = find `scale` and `offset` so a known physical direction lands at its ROS angle. TOML-only, no rebuild (the config is read at sangamio start).

Current calibrated values (this robot, `r2104`): `scale=-1`, `offset=0.506`, plus `angle_offset=0.4204` (theta 24.09 from `/mnt/misc/lds_config.json`) and `offset_x=-0.087` (LDS behind center). Redo this if the LDS is remounted.

**Why it works with one wall:** the closest point of a flat wall is the perpendicular foot, so the *nearest* returns point straight at the wall. `tools/cal_lidar.py` reports the median angle of the closest ~15% of points = the wall's direction in the current published frame. Compare that to where the wall physically is, and rotate `offset` by the difference.

**Procedure (with sangamio running in direct mode, `lidar on`):**

1. Take the robot off the dock into open space. Put a flat wall/box ~0.5 m from **one** side and note which side (front/back/left/right = target 0/180/90/270 deg).
2. Measure where the wall reads now:
   ```
   tools/cal_lidar.py <robot-ip> 6
   ```
   Note the reported "wall angle ... median M deg". (`tools/lidar_rx.py` shows the full 30-deg coverage histogram if you want to see the ~126-deg arc.)
3. Set `frame_transforms.lidar.offset += (target - M)` **in radians** (deg * pi/180). Redeploy the config and restart sangamio, then re-run step 2. The wall should now sit at the target; iterate once or twice to tighten it.
4. **Handedness check:** if step 3 moves the wall the *wrong* way (further from target), flip `scale` (e.g. `-1` <-> `1`) and redo. A correct `scale` makes the wall move toward the target as you increase `offset`.

**Worked example (this robot):** wall on the **left** (target 90). `cal_lidar.py` reported median **61 deg**; `90 - 61 = +29 deg = +0.506 rad`, so `offset 0.0 -> 0.506`. Re-measured: **~91 deg**. Done.

**Caveats:** coverage is the fixed ~126 deg arc, so the wall must fall inside it (rotate the robot if the chosen side isn't seen). The position offset (`offset_x/y`) perturbs the angle slightly for near points, so expect ~1-2 deg residual; a single wall pins rotation + handedness but not a full mounting solve.

## Protocol references

- MCU framing, SetCleaning actuator map, ping/pong, triggers: `../dreame-w10/docs/MCU_PROTOCOL.md`
- LDS packet format and the fixed rear-arc sector: `../dreame-w10/docs/LDS_PROTOCOL.md`
- Shared parser/encoder crate: `../dreame-w10/proto` (`dreame-w10-proto`)
