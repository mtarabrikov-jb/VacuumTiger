#!/usr/bin/env python3
"""Receive sangamio's UDP sensor stream and decode the `lidar` PointCloud2D `scan`.

Registers by opening a TCP connection to sangamio:5555 (kept open), then listens
on UDP :5555 for datagrams ([4-byte BE len][protobuf Message]) and decodes the
`lidar` group's `scan` value. Reports per-scan point count, distance range, and a
30-degree histogram of point angles (post-transform, robot frame) for coverage
and calibration.

Usage: lidar_rx.py <robot-ip> [seconds]
"""
import socket, struct, sys, time, math

PORT = 5555


def rv(b, i):                       # read protobuf varint
    r = 0; s = 0
    while True:
        x = b[i]; i += 1
        r |= (x & 0x7f) << s
        if not (x & 0x80):
            return r, i
        s += 7


def fields(b):                      # walk a protobuf message -> [(field, wiretype, value)]
    i = 0; out = []
    while i < len(b):
        tag, i = rv(b, i); fn = tag >> 3; wt = tag & 7
        if wt == 0:
            v, i = rv(b, i)
        elif wt == 5:
            v = b[i:i + 4]; i += 4
        elif wt == 1:
            v = b[i:i + 8]; i += 8
        elif wt == 2:
            ln, i = rv(b, i); v = b[i:i + ln]; i += ln
        else:
            return out
        out.append((fn, wt, v))
    return out


def f32(b):
    return struct.unpack('<f', b)[0]


def decode_lidar(msg):
    """Return list[(angle_rad, dist_m, quality)] if this Message is the lidar scan, else None."""
    sg = None
    for fn, wt, v in fields(msg):           # Message: 1=topic, 2=sensor_group
        if fn == 2 and wt == 2:
            sg = v
    if sg is None:
        return None
    gid = None; entries = []
    for fn, wt, v in fields(sg):            # SensorGroup: 1=group_id, 2=ts, 3=map entries
        if fn == 1 and wt == 2:
            gid = v.decode('latin1')
        elif fn == 3 and wt == 2:
            entries.append(v)
    if gid != 'lidar':
        return None
    for e in entries:                       # map entry: 1=key, 2=SensorValue
        key = None; val = None
        for fn, wt, v in fields(e):
            if fn == 1 and wt == 2:
                key = v.decode('latin1')
            elif fn == 2 and wt == 2:
                val = v
        if key != 'scan' or val is None:
            continue
        for fn, wt, v in fields(val):       # SensorValue: 11=PointcloudVal
            if fn == 11 and wt == 2:
                pts = []
                for fn2, wt2, v2 in fields(v):   # PointCloud2D: 1=repeated LidarPoint
                    if fn2 == 1 and wt2 == 2:
                        a = d = 0.0; q = 0
                        for fn3, wt3, v3 in fields(v2):   # LidarPoint: 1=angle,2=dist,3=qual
                            if fn3 == 1 and wt3 == 5:
                                a = f32(v3)
                            elif fn3 == 2 and wt3 == 5:
                                d = f32(v3)
                            elif fn3 == 3 and wt3 == 0:
                                q = v3
                        pts.append((a, d, q))
                return pts
    return []


def main():
    if len(sys.argv) < 2:
        raise SystemExit(__doc__)
    robot = sys.argv[1]
    dur = float(sys.argv[2]) if len(sys.argv) > 2 else 10.0

    u = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    u.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    u.bind(("0.0.0.0", PORT))
    u.settimeout(2.0)
    t = socket.create_connection((robot, PORT), timeout=5)   # register for UDP streaming
    print(f"registered TCP {robot}:{PORT}, UDP :{PORT}, capturing {dur}s")

    t0 = time.time(); nscan = 0; nother = 0
    while time.time() - t0 < dur:
        try:
            data, _ = u.recvfrom(65535)
        except socket.timeout:
            continue
        pts = decode_lidar(data[4:])         # strip 4-byte length prefix
        if pts is None:
            nother += 1; continue
        nscan += 1
        valid = [(a, d, q) for (a, d, q) in pts if d > 0]
        if not valid:
            print(f"scan #{nscan}: {len(pts)} pts, all invalid/zero")
            continue
        bins = [0] * 12                      # 30-degree bins at 0,30,...,330
        for a, d, q in valid:
            bins[int(math.degrees(a) % 360 // 30)] = bins[int(math.degrees(a) % 360 // 30)] + 1
        ds = [d for a, d, q in valid]
        if nscan <= 3 or nscan % 5 == 0:
            hist = " ".join(f"{b:>3}" for b in bins)
            print(f"scan #{nscan}: {len(valid):>3} pts  dist {min(ds):.2f}..{max(ds):.2f}m  "
                  f"30deg-bins[0 30 60 .. 330]: {hist}")
    print(f"done: {nscan} lidar scans, {nother} other msgs in {dur:.0f}s")
    t.close(); u.close()


if __name__ == "__main__":
    main()
