# Dreame W10 (`r2104`) LDS/LIDAR protocol — reverse-engineering findings

The LDS is the spinning laser range scanner `ava` uses for SLAM. It talks to the
SoC (`ava`) over a framed binary link on **`/dev/ttyS3 @ 230400`** (config node
`AvaNodeLDS` in `/ava/conf/r2104.conf`). This document records the scan format
decoded on the W10 and how it was verified. The Rust parser is
[`proto/src/lds.rs`](../proto/src/lds.rs).

Legend: **[verified]** confirmed live on this robot · **[partial]** decodes but
some fields/scales are unconfirmed · **[unknown]** seen on the wire, not decoded.

## Capturing scans

The turret **only spins during navigation** — `lds-rx` (relay port 7702) is
silent while the robot is idle or docked. Enabling Valetudo's
`HighResolutionManualControlCapability` is enough to start it (no full cleaning
needed): the turret spins up, and a keepalive of small in-place rotations keeps
`ava` in navigation mode. Any other navigation (a cleaning, a return-to-dock)
also spins it.

**Coverage [verified]: the tapped stream is a FIXED ~126 deg rear arc, not a full
circle.** `fsa` stays within raw 40986-63995 (~**226-352 deg**) and never leaves
it. This was checked two independent ways and is identical in both:

- **Manual control**, robot roughly stationary: sector 226-352 deg.
- **Active SLAM navigation** (captured during a real return-to-dock, ~3125
  packets while the robot drove and turned through many headings): sector still
  226-352 deg — it does **not** rotate with the robot's heading (so it is fixed in
  the robot frame) and does **not** widen. The entire capture was type-0x03
  packets in this arc; **no full-360 feed is present on ttyS3 at all**.

So active navigation is *not* the way to get a full circle — this serial link
simply does not carry one. Why only a sub-arc is exposed (physical sector
scanner vs. `ava` reading the rest of the circle over a different interface) is an
open question. The framing/scaling below are unaffected; only coverage is partial.

## Framing [verified]

Fixed-length **40-byte** packets, no escaping. Little-endian throughout. Sync is
the 4 bytes `55 aa 03 08`, constant across every packet observed (1435/1435 in
the reference capture); the trailing bytes are **not** a reliable end marker, so
the de-framer keys on the header + fixed length and resyncs on the next header.

```
off  size  field                                                     status
 0    2    55 aa            sync                                      [verified]
 2    1    03               packet type (constant)                    [verified]
 3    1    08               sample count LSN = 8 (constant)           [verified]
 4    2    u16  speed       turret rotation speed, raw units          [partial]
 6    2    u16  fsa         start angle of this packet's 8 samples    [verified]
 8   24    8 x {u16 dist, u8 quality}   samples                       [verified]
32    2    u16  lsa         end angle of this packet's 8 samples      [verified]
34    2    u16  checksum    noisy per packet; no standard CRC matched [unknown]
36    2    u16  counter     monotonic +~3836/pkt, wraps ~17 pkts      [partial]
38    2    u16  aux         high byte 0x4b/0x4c, low byte varies      [unknown]
```

Packet rate ~239/s in the reference capture (~9.5 KB/s, ~40% of the 230400 line).

### Angles [verified]

`fsa`/`lsa` are a u16 fraction of a full circle — the working scale is
**`65536 == 360 deg`** (`[partial]`: the exact scale is unconfirmed because a full
circle was never captured; it is consistent with the observed spin rate). The
8 samples of a packet are spread **linearly** from `fsa` to `lsa`:

```
angle(k) = fsa + (lsa - fsa) * k / 8     k = 0..7   (u16, wrapping)
```

How the pair was confirmed: within a packet `lsa - fsa` is a small positive arc
(~438 units ~ 2.4 deg for 8 samples), and consecutive packets are
angle-continuous — packet N's `lsa` (e.g. 43333) sits just before packet N+1's
`fsa` (e.g. 43396), a steady ~63-unit inter-packet gap. Decoding one revolution's
worth of `(angle, distance)` produced a coherent room outline (near walls
0.4-1.1 m, a far opening ~2.9 m), which is the end-to-end proof the scaling and
sample layout are right.

### Samples [verified]

Each sample is a little-endian **`u16` distance in mm** followed by a **`u8`
quality/intensity** (0..~56 observed). Distances observed 0.34-8.4 m. The
distance's top bit (`0x8000`) marks an **invalid / no-return** point — when set,
the low 15 bits are 0 (all 2125 flagged samples in the reference capture read
exactly `0x8000`). So:

```
valid = (raw & 0x8000) == 0      dist_mm = valid ? raw : 0
```

~18.5% of samples were invalid (no echo) in the reference capture.

### Speed [partial]

The u16 at offset 4 traced a clean **spin-up -> plateau (~25000) -> spin-down**
curve over a capture as the turret started and stopped, hence "rotation speed".
It updates about every 3 packets. Raw units are unknown (not converted to RPM).

### Trailer fields

- **`checksum` @34 [unknown]** — noisy per packet (looks hash-like). No standard
  scheme matched: CRC-16/Modbus, CRC-16/CCITT (both inits), and byte/word sums
  over every tried byte range all scored 0/400. May be a non-standard CRC or
  cover the timestamp too. Not needed for a read-only consumer — frames are
  validated by the header + fixed length.
- **`counter` @36 [partial]** — monotonic, +~3836/packet regardless of speed,
  wraps ~every 17 packets. Likely a timestamp/tick (constant per-packet delta ~
  constant packet period).
- **`aux` @38 [unknown]** — high byte is 0x4b or 0x4c, low byte varies; magnitude
  is near `speed`. Purpose unclear; explicitly **not** a fixed footer.

## Decoding tools

- `make lds` — live LDS summary via `w10-decode --lds`: packet rate, turret
  speed, covered angular sector, valid/invalid point counts, distance range.
- `w10-decode --lds --raw` — hexdump the raw stream instead.

## Open items

- Understand **why only a ~126 deg arc** is exposed on ttyS3 and whether the full
  360 deg circle is available elsewhere (a second serial device, a different
  `ava` node, or an internal-only path). Ruled out: active navigation — it does
  not widen the arc (see Coverage above).
- Confirm the `65536 == 360 deg` angle scale. It could not be pinned down because
  a full circle was never observed on this link; it is consistent with the
  observed spin rate but remains an assumption.
- Identify the `checksum` scheme (offset 34) and the `aux` field (offset 38).
- Convert `speed` (offset 4) to RPM.
- Feed decoded scans into the SangamIO `dreame_w10` driver's reserved LDS group
  (relay `lds-rx`, port 7702).
