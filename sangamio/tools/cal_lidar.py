#!/usr/bin/env python3
"""Lidar calibration helper: capture scans, find the wall (closest cluster) angle.

Reuses the sangamio UDP `scan` decode (see lidar_rx.py). Aggregates valid points
over a few seconds and reports: a 10-degree histogram of all point angles, and
the median angle of the closest ~15% of points (= the perpendicular to a flat
wall, i.e. the wall's direction in the current published frame).

Usage: cal_lidar.py <robot-ip> [seconds]
"""
import socket, struct, sys, time, math

PORT = 5555


def rv(b, i):
    r = 0; s = 0
    while True:
        x = b[i]; i += 1
        r |= (x & 0x7f) << s
        if not (x & 0x80):
            return r, i
        s += 7


def fields(b):
    i = 0; out = []
    while i < len(b):
        tag, i = rv(b, i); fn = tag >> 3; wt = tag & 7
        if wt == 0: v, i = rv(b, i)
        elif wt == 5: v = b[i:i+4]; i += 4
        elif wt == 1: v = b[i:i+8]; i += 8
        elif wt == 2:
            ln, i = rv(b, i); v = b[i:i+ln]; i += ln
        else: return out
        out.append((fn, wt, v))
    return out


def f32(b): return struct.unpack('<f', b)[0]


def decode_lidar(msg):
    sg = None
    for fn, wt, v in fields(msg):
        if fn == 2 and wt == 2: sg = v
    if sg is None: return None
    gid = None; entries = []
    for fn, wt, v in fields(sg):
        if fn == 1 and wt == 2: gid = v.decode('latin1')
        elif fn == 3 and wt == 2: entries.append(v)
    if gid != 'lidar': return None
    for e in entries:
        key = None; val = None
        for fn, wt, v in fields(e):
            if fn == 1 and wt == 2: key = v.decode('latin1')
            elif fn == 2 and wt == 2: val = v
        if key != 'scan' or val is None: continue
        for fn, wt, v in fields(val):
            if fn == 11 and wt == 2:
                pts = []
                for fn2, wt2, v2 in fields(v):
                    if fn2 == 1 and wt2 == 2:
                        a = d = 0.0
                        for fn3, wt3, v3 in fields(v2):
                            if fn3 == 1 and wt3 == 5: a = f32(v3)
                            elif fn3 == 2 and wt3 == 5: d = f32(v3)
                        pts.append((a, d))
                return pts
    return []


def main():
    robot = sys.argv[1]
    dur = float(sys.argv[2]) if len(sys.argv) > 2 else 6.0
    u = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    u.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    u.bind(("0.0.0.0", PORT)); u.settimeout(2.0)
    t = socket.create_connection((robot, PORT), timeout=5)
    pts = []
    t0 = time.time()
    while time.time() - t0 < dur:
        try: data, _ = u.recvfrom(65535)
        except socket.timeout: continue
        p = decode_lidar(data[4:])
        if p: pts += [(math.degrees(a) % 360, d) for a, d in p if d > 0]
    t.close(); u.close()
    if not pts:
        print("no points"); return
    # 10-degree histogram
    bins = [0] * 36
    for a, d in pts: bins[int(a // 10)] += 1
    print("10deg histogram (angle: count), nonzero:")
    for i, c in enumerate(bins):
        if c: print(f"  {i*10:3d}-{i*10+10:3d}: {c}")
    # closest ~15% = the wall
    pts.sort(key=lambda x: x[1])
    wall = pts[:max(10, len(pts) * 15 // 100)]
    wangs = sorted(a for a, d in wall)
    wdist = [d for a, d in wall]
    med = wangs[len(wangs) // 2]
    print(f"\nclosest {len(wall)} pts (the wall): dist {min(wdist):.2f}..{max(wdist):.2f}m")
    print(f"  wall angle: {wangs[0]:.0f}..{wangs[-1]:.0f} deg, median {med:.0f} deg")
    print(f"  -> the wall currently appears at ~{med:.0f} deg in the published frame")


if __name__ == "__main__":
    main()
