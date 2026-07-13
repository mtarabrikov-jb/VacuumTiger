# Dreame W10 (`r2104`) base-station (dock) — reverse-engineering findings

The base station (the mop-wash + drying dock) is **not** a dumb accessory: it is a
standalone microcontroller device with its own firmware, its own LCD, its own
wash/dry hardware, a sub-GHz radio and a serial bootloader. This document records
what was decoded about the dock itself and the robot<->dock link, mostly by
disassembling the dock's own firmware (`UIMA.bin`/`UIMB.bin`) and the robot MCU
firmware (`mcu.bin`) in Ghidra.

Companion docs: the robot-side view (what `ava` sends the dock over `ttyS4`) is in
[`MCU_PROTOCOL.md`](MCU_PROTOCOL.md) (`0x23` dock status, `0x26` dock control).
This file is the **dock-side** view.

Legend: **[verified]** confirmed from decompilation, cross-checked from both TX
and RX sides · **[partial]** structure clear, some fields/offsets not nailed ·
**[inferred]** strong evidence, not directly observed on the wire · **[open]** not
yet decoded.

## Two-layer picture

```
 ava (SoC)  --ttyS4 3c..3e frames-->  robot MCU (mcu.bin)  --RF AA55 frames-->  dock MCU (UIMA/UIMB.bin)
   0x25/0x26/0x28 dock frames            re-encodes                              drives pumps/heater/fan/LCD
   0x23/0x30 status back                 <--RF status--                          reports tanks/wash state
```

- **Layer 1 (`ava` <-> robot MCU, `ttyS4`)** — the framed `<len type payload crc16>`
  protocol in [`MCU_PROTOCOL.md`](MCU_PROTOCOL.md). Dock-related types: `0x23`
  dock/tank status (in), `0x26` dock control (out, wash/dry), `0x25` station-set,
  `0x28` station LED, `0x30` wifi-station wash status. Built by `node_signal.so`
  (`AvaCleanDockProcess -> CastComMsg(0x26,..)`, `StationLedSetProcess -> 0x28`,
  etc.). **[verified]**
- **Layer 2 (robot MCU <-> dock MCU, RF)** — a separate packet protocol between the
  two microcontrollers, documented below. The robot MCU is the bridge: it takes the
  Layer-1 dock frames and re-emits Layer-2 RF frames to the dock. **[verified]** frame
  format; **[inferred]** that the physical carrier is the radio (see Radio).

## Dock hardware (from `UIMA.bin` strings + code)

- **MCU**: **GD32** Cortex-M (firmware supports `GD32F303CCT6 / E103CBT6 / F103ZET6 /
  F303ZET6`). Vector table loads at **`0x08004000`** (a ~16 KB bootloader sits at
  `0x08000000` below it). **[verified]**
- **LCD**: a small **~240 px-wide RGB565 SPI TFT** (ST7789/ILI9341-class). Drawn in
  horizontal bands of 240x5 px; framebuffer band at RAM `0x200042e2` (0x960 B =
  1200 px). **[verified]** width/format; exact height [partial].
- **UI assets**: a **FAT32** filesystem on the dock's own flash holding the screen
  images as `/UI<set>/U<n>.bin`, `/UI<set>/U<n>_1.bin`, `/UI<set>/U14_<n>.bin`,
  digit glyphs `/Num/<n>.bin`, glyphs `/CH/C<n>.bin`, plus `version.txt`.
  Versioned independently as **"file system version"** (separate from `mc version`
  = the code and `ui version`). **[verified]**
- **Wash/dry actuators** (all physically in the dock): water pumps
  `pumpadd`/`pumpclean`/`pumpdirty`/`pumpdetergent`, motors `Motoradd`/
  `Motordetergent`, water valve `Solenoid`, dry heater `PTC`, fan `fandry`/`coolfan`.
  Sensors `chargercur`, `cleancur`, `dirtycur` (charge current, clean/dirty water).
  **[verified]** they exist and are toggleable (there is a UART debug CLI:
  `Motoradd_open/off`, `PTC_open/off`, `Solenoid_open/off`).
- **Radio**: a sub-GHz RF transceiver, regionally tuned — **433.92 MHz (China/EU),
  429.40 (Japan), 447.90 (Korea), 915 (USA)** — with `Rx mode`/`TX start`/`rftest`/
  `rf_tx_retry`/`RF_ErrorCode` in both firmwares. **[verified]** present.
- **IR**: `irML/irMR/irR-dock` beacons are for **docking alignment only**, not a data
  channel. **[verified]**

## Firmware image (`UIMA.bin` / `UIMB.bin`)

- Two variants for two dock hardware revisions: **`UIMA.bin`** (61776 B) and
  **`UIMB.bin`** (62120 B). Both ship **baked into the robot rootfs at `/UIMA.bin`
  and `/UIMB.bin`** (alongside the robot's own `/mcu.bin`); they are pure ARM
  Cortex-M code (no image assets inside). **[verified]**
- Update flow: `/ava/script/ota_watch_mcu_update.sh` picks the variant by dock type
  (`$2==1 -> UIMB.bin`, else `UIMA.bin`), renames it to `/tmp/update/UI.bin`, and
  runs `/ava/script/ota_base_station.sh`, which:
  1. sends the enter-bootloader MCU frame `uart_hex /dev/ttyS4 "3C010A81C0E63E"`,
  2. waits for the dock bootloader's ymodem `C` handshake,
  3. flashes with `/usr/bin/ymodem_bs -u <baud>-8-1-0 -b /tmp/update/UI.bin -d ttyS4`.
  `ota.conf` tracks the dock version separately as `ota_station_ver` ->
  `/tmp/update/basestation_ver.txt`. **[verified]**
- The dock code **also** implements OTA over the RF link (`ReceiveOTACmd:%02X` /
  `SendAckOTACmd` in both MCUs), i.e. a firmware/FS update can be pushed over the
  radio, not only over the wired ymodem path. **[verified]** present.

## RF frame format (Layer 2, robot MCU <-> dock MCU) [verified]

Confirmed from both sides: the builder `FUN_0800e7dc` in `mcu.bin` and the parser
`FUN_08005250` in `UIMA.bin` are exact mirrors.

```
AA 55 | LEN | CMD | PAYLOAD[LEN] | CRC16(hi,lo) | 0D 0A
```

- `AA 55` — fixed preamble.
- `LEN` — payload length, **1..0x18** (max 24 payload bytes).
- `CMD` — command code (1 byte).
- `PAYLOAD` — `LEN` bytes.
- `CRC16` — 2 bytes, **big-endian (hi, lo)**, computed over `[LEN][CMD][PAYLOAD]`
  (builder `FUN_0800577c` / parser `FUN_080053d4`).
- `0D 0A` — trailer (CR LF).
- Total on-wire length = `LEN + 8`.

The dock replies to the robot with the **same** framing (status / `AckCmd:%04X`).

### Dispatch (dock side, `FUN_08005250`)

After validating preamble + CRC + `\r\n`, the dock dispatches on `CMD`:

- **`CMD == 0x01` + payload signature `"BT"` (`0x42 0x54`)** — enter bootloader:
  the dock jumps to `0x08000000` (`FUN_0800ec1c -> FUN_0800ebec(0x8000000)`). This
  is the **OTA-over-RF** entry, the radio equivalent of `ota_base_station.sh`.
  **[verified]**
- **`CMD >= 0x80`** — indirect call through a handler table at `0x08012068`
  (`handler = *(0x08012068 + CMD*4)`). In this firmware only `0x80` is populated.
  **[verified]**
- **`CMD < 0x80`** — a stub in this parser (`FUN_08008624` = `return 0`); this parser
  handles the control/OTA class, not every command. **[verified]**

## Command `0x80` — the dock control / display frame [verified]

This is the one runtime command the robot sends (robot side: `mcu.bin`
`FUN_08009238 -> FUN_08008c28(0x80, &state9, 9)`, sent periodically). Payload is
**9 bytes**. The dock handler `FUN_08008634` copies them verbatim into a control
block at RAM `0x200000f0..0x200000f8`, consumed by the LCD state machine
`FUN_080090c8`:

| payload byte | meaning | verification |
|---|---|---|
| `[0]` | **screen/status code** (debounced via `FUN_0800eb68`) -> selects the screen file | [verified] |
| `[1] [2] [3]` | **actuator parameters** (pump rate / water amount) -> relayed to `0x200002bf/be/c0` by `FUN_08006f70` | [partial] |
| `[4] [5]` | not used by the renderer | [open] |
| `[6]` | **actuator bitmask** -> `0x200002c2` (see below) | [verified] |
| `[7]` | **progress percent (0..100)** -> `/CH/C<pct/10+1>.bin` (11 images C1..C11) | [verified] |
| `[8]` | **UI set / theme number** -> `/UI<[8]>/...` folder | [verified] |

So `0x80` is a combined **display + actuator** frame: the robot folds its Layer-1
`ttyS4` dock commands (`0x26` wash/dry, `0x28` LED, status) into these 9 bytes and
sends them to the dock periodically.

### Actuator control — payload `[6]` bitmask [verified]

`payload[6]` lands in `0x200002c2`, an actuator bitmask. The dock's own debug CLI
sets/clears exactly these bits (`FUN_08005d44`/`df8`/`fdc`/`08006170`), which pins
each bit down:

| bit | mask | actuator |
|---|---|---|
| 1 | `0x02` | **Solenoid** — water valve |
| 2 | `0x04` | **PTC** — mop-drying heater |
| 3 | `0x08` | **Motoradd** — clean-water pump motor |
| 4 | `0x10` | **Motordetergent** — detergent pump motor |
| 0, 5..7 | — | other pump(s) / drying fan — driven, no CLI toggle [partial] |

So the ava-side `0x26` **wash** (pump + water) and **dry** (heater + fan) map onto
`0x80` `payload[6]` bits plus the `payload[1..3]` rate/amount parameters — the `0x26`
payload reaches the dock actuators via `0x80`, not a separate command.

### Screen selection (`FUN_080090c8`) [verified]

Given screen code `c = payload[0]` and set `s = payload[8]`:

- `c == 0x14` or `0x16` -> progress screen using `/CH/C<pct/10+1>.bin` (pct = payload[7]).
- `c == 0x65 / 0x66 / 0x67` -> `/UI<s>/U14_1.bin` / `U14_2.bin` / `U14_3.bin`.
- `c in {0x02, 0x0d, 0x11, 0x12, 0x13}` -> animated: alternates `/UI<s>/U<c>.bin` and
  `/UI<s>/U<c>_1.bin` (table at `0x080128f1`).
- otherwise -> `/UI<s>/U<c>.bin`.

Format strings (in `UIMA.bin`): `/UI%d/U%d.bin`, `/UI%d/U%d_1.bin`,
`/UI%d/U14_%d.bin`, `/CH/C%d.bin`, `/Num/%d.bin`.

### Where the host sets the screen [verified]

`payload[0]` (screen code) originates from the **ttyS4 `0x26` frame's byte0** (the
dock "mode"). Traced in `mcu.bin`: the `0x26` payload feeds `FUN_08015866`, which
fills a dock-state struct at `0x2000d210` (`dst[0]=byte0` mode, `dst[1..3]=byte1..3`,
`dst[6]=byte6*4+1` actuator mask, percent from `byte7`); the periodic task
`FUN_0801679c` derives the screen code from that mode and packs the 9-byte `0x80`
payload (`FUN_08009238` sends it). So the robot picks the dock screen with `0x26`
byte0. `ros2dreame` exposes this as the `/set_dock_screen` topic (an idle-shaped
`0x26` with byte0 = the chosen code) and `/set_dock_frame` (raw 8-byte payload). The
full byte0 -> screen table below was **captured live** on an r2104 W10 dock by
sweeping `/set_dock_screen` and reading the panel.

The ava-side status -> screen-code decision lives in `node_aether.so`
`ancp::AvaNodeRoute::SendCleanDockMsg` (publishes `ava_clean_dock_msg`; strings
`screen_show_robot_tatus`, `station_show_clean_complete`) - a large function, not
extracted byte-exact. Building the full byte0 -> visible-screen table is easier live:
ros2dreame's `make dock-sweep` (`host/dock-sweep.sh`) sweeps `/set_dock_screen` while
you watch the dock panel (the LCD is on no camera, so it is a human-in-the-loop read,
same as how the `0x26` wash/dry table was captured).

### byte0 -> dock LCD screen [verified live, r2104 W10 dock]

Captured by publishing `/set_dock_screen` and reading the panel (ava OFF, ros2dreame
driving, robot docked):

| byte0 | dock LCD screen |
|---|---|
| `0x01` | "Dreame" (logo / splash) |
| `0x02` | Wi-Fi connecting (animation) |
| `0x03` | Cleaning |
| `0x04`, `0x05` | Mopping |
| `0x06` | Spot Clean Mode |
| `0x07`, `0x08`, `0x0b` | Standby |
| `0x09`, `0x0a` | Paused |
| `0x0c` | 100% / charging (the docked-idle default) |
| `0x0d` | Mop pad cleaning (animation) |
| `0x0e` | Mop pad dehydrating 1/4 (animation) |
| `0x0f` | Positioning |
| `0x10` | Resume Cleaning Mode |
| `0x11` | Returning to the base for self washing |
| `0x12`, `0x13` | Returning to the base for recharging |
| `0x14`, `0x15`, `0x16` | 100% / charging (`0x14` = the `0x26` idle mode) |
| `0x17` | Clean-water tank not installed |
| `0x18` | Dirty-water tank full or not installed |
| `0x19` | Mop-pad holder not installed |
| `0x1a` | Mop-pad water level abnormal |
| `0x1b` | Child lock |
| `0x1c` | Child-lock unlock hint (hold home + play) |
| `0x1d` | Exception occurs |
| `0x1e` | Upgrading |
| `0x1f` | Low water level in the clean-water tank |
| `0x20` | Cleaning paused |
| `0x21` | Mapping |
| `0x22`+ | "Dreame" / no distinct image (end of the set) |
| `0x65`, `0x66`, `0x67` | Mop pad dehydrating 2/4, 3/4, 4/4 |

Notes: a code with no dedicated image leaves the previous screen up (the dock only
repaints on a code that has an asset), so several codes read as "same as the last
one". Dehydrating progress is four discrete codes (`0x0e`=1/4, `0x65/0x66/0x67`=2/4..4/4),
not a percent byte. The firmware `0x14/0x16` "progress" path (`/CH/C<pct>.bin`)
rendered as plain 100% here because `payload[7]` carried no percent.

## Dock -> robot status / telemetry frame [verified]

The dock also transmits back to the robot, periodically (every ~`0x1d` ticks in
`FUN_080100b4`), using the **same framing but a different preamble** to mark the
direction:

```
AA AA | 05 | 00 | [actuator_mask] [4 status bytes] | CRC16 | 0D 0A
```

- preamble **`AA AA`** (dock->robot) vs `AA 55` (robot->dock);
- `LEN = 5`, `CMD = 0`;
- payload = the actuator mask (`0x200002c2`) + the 4-byte block at `0x200002be`
  (echoed command params / sensor readings — clean/dirty water level, current; exact
  fields [partial]).

## Radio / physical carrier [inferred]

Both MCUs carry a full sub-GHz RF stack (regional frequencies, `SysID`, TX/Rx
modes). On the dock, every outbound frame goes through `FUN_0801004c`, which kicks a
**DMA transfer** (GD32 DMA controller at `0x40020000`, channel config +
count/address regs `0x40020048`/`0x40020050`) feeding a serial peripheral — i.e. the
frame is DMA'd out to the radio module, not bit-banged onto a charging pin. Combined
with the RF stack this makes the robot<->dock link the **radio**, not a
charging-contact serial line. Not yet confirmed by an over-air capture, but the
earlier "charging contacts" assumption is not supported by the firmware.

## LCD image format `Ux.bin` (palettized RLE) [partial]

The draw routine `FUN_080076c8` decodes a `Ux.bin` file and blits it to the TFT in
240x5 bands (`FUN_08007004` sets the window, `FUN_08007414` flushes a band,
`FUN_080073b8` = RGB888->RGB565). The file is **not** a raw bitmap; it is a
palettized run-length structure:

- `[0] = 0xAA` magic; `[1] = H` = palette size; `[2..3] = N` = run count (u16 BE);
  `[4..5]` = reserved/unknown.
- `[6 .. 6+2N-1]` — **position[]**: N x u16, the start pixel of each run.
- then three parallel N-length arrays (per run): **run-length[]** (u8) and
  **palette-index[]** (u8), plus the **RGB palette** as three planes `R[H] G[H] B[H]`.
  Run colour = `RGB565(R[idx], G[idx], B[idx])` via `FUN_080073b8`.

The exact start offsets of the run-length / index / palette arrays have off-by-one
ambiguity in the decompiled index math (`FUN_080076c8`); pinning them byte-exact
needs validation against a real `Ux.bin` dumped from the dock flash. [partial]

## Consequence: custom text/images on the dock LCD

- The dock has a real graphical LCD, but the robot only ever selects **which stored
  image** to show (screen code + set + percent). It never sends pixels. So arbitrary
  content is **not** possible through the runtime protocol.
- To display arbitrary images you must go after the dock's own storage/firmware:
  1. author a `Ux.bin` in the palettized-RLE format above, drop it into `/UI<set>/`
     on the dock FAT32, and trigger its screen code via `0x80`; or replace the glyph
     files `/CH/C*.bin` + `/Num/*.bin` for custom text; or
  2. modify `UIMA.bin`/`UIMB.bin` (the renderer) and reflash via `ota_base_station.sh`
     (wired ymodem) or the RF OTA path (`CMD 0x01` + `"BT"`).
- Getting the assets requires dumping the dock's flash (the `Ux.bin` files are not in
  the robot rootfs — only the code `UIMA/UIMB.bin` is).

## Method / tooling

- Ghidra **12.1.2** headless, both images imported as `ARM:LE:32:Cortex` at base
  **`0x08004000`** (the +0x4000 offset is essential — at `0x08000000` the absolute
  data/string pointers and the `0x08012068` handler table fall outside the image and
  nothing resolves).
- Key addresses (`UIMA.bin`): parser `FUN_08005250`, CRC `FUN_080053d4`, OTA/boot
  `FUN_0800ec1c`, cmd-0x80 store `FUN_08008634`, LCD state machine `FUN_080090c8`,
  screen debounce `FUN_0800eb68`, LCD draw `FUN_080076c8`, handler table `0x08012068`,
  special-screen table `0x080128f1`. Actuators: mask `0x200002c2`, param relay
  `FUN_08006f70`, CLI handlers PTC `FUN_08005d44` / Motoradd `FUN_08005df8` /
  Motordetergent `FUN_08005fdc` / Solenoid `FUN_08006170`. Dock TX: status-frame
  builder `FUN_080100b4`, DMA transmit `FUN_0801004c` (`0x40020000`), RF-freq CLI
  `FUN_080062d4`.
- Key addresses (`mcu.bin`, base `0x08004000`): frame builder `FUN_0800e7dc`, CRC
  `FUN_0800577c`, send wrapper `FUN_08008c28`, 0x80 sender `FUN_08009238`.
- Strings in these firmwares are addressed via an index table, so both Ghidra's and
  capstone's string-xref come up empty; functions were located by constant/address
  targeting instead.

## Open items

- **[resolved]** `0x26` -> RF: byte0 -> `0x80` `payload[0]` screen code; byte6 ->
  `payload[6]` actuator bitmask (Solenoid/PTC/Motoradd/Motordetergent, `*4+1`);
  byte1..3 -> `payload[1..3]` rate/amount params (via `FUN_08015866`/`FUN_0801679c`).
- Verify the `0x26` byte0 -> visible dock screen mapping beyond `0x14`/`0x0d`/`0x0e`
  (drive `/set_dock_screen` and watch the LCD).
- **[resolved]** dock->robot status frame decoded (`AA AA | 05 | 00 | mask + 4B |
  crc16 | 0D 0A`); the 4 status/sensor bytes remain [partial].
- **[strengthened]** carrier is the radio (dock TX = DMA `0x40020000` -> serial -> RF
  module); still not confirmed by an over-air capture.
- Nail the exact `Ux.bin` array offsets — needs a real file dumped from dock flash.
- Actuator-mask bits `0`/`5..7` (dirty-water pump, drying fan) and the 4
  status-frame sensor bytes.
- Identify the exact sub-GHz transceiver part (SPI register map / `SysID` read at
  regs `0x3f..0x44` in `FUN_0800fc0c`).
