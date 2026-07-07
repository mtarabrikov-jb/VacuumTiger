#!/usr/bin/env python3
"""Minimal SangamIO TCP command client for the Dreame W10 direct-mode driver.

Wire: [4-byte BE length][protobuf Message]. Message{topic="command", command=Command}.
Protobuf is hand-encoded (no generated stubs / protoc needed).

Usage:
  w10cli.py <host> drive <lin_mps> <ang_radps> [--hold SEC]   # creep; --hold resends @10Hz
  w10cli.py <host> stop
  w10cli.py <host> on   <component>          # ENABLE (brush/vacuum -> 100%, lidar -> on)
  w10cli.py <host> off  <component>          # DISABLE
  w10cli.py <host> speed <component> <0-100> # CONFIGURE speed
  w10cli.py <host> lidar on|off
Components: drive vacuum main_brush side_brush water_pump lidar led
"""
import socket, struct, sys, time

PORT = 5555
ENABLE, DISABLE, RESET, CONFIGURE = 0, 1, 2, 3


def varint(n):
    out = bytearray()
    while True:
        b = n & 0x7F
        n >>= 7
        out.append(b | 0x80 if n else b)
        if not n:
            return bytes(out)


def tag(f, w):      return varint((f << 3) | w)
def ld(f, p):       return tag(f, 2) + varint(len(p)) + p         # length-delimited
def sfield(f, s):   return ld(f, s.encode())                      # string
def f32(f, v):      return tag(f, 5) + struct.pack('<f', v)       # float
def vint(f, n):     return tag(f, 0) + varint(n)                  # varint


def sv_f32(x):      return f32(6, x)                              # SensorValue{f32_val=6}
def sv_u32(x):      return vint(2, x)                             # SensorValue{u32_val=2}
def map_entry(k, sv):  return sfield(1, k) + ld(2, sv)           # {key=1, value=2}


def action(atype, entries=()):
    b = vint(1, atype)                                            # ComponentAction.type=1
    for e in entries:
        b += ld(2, e)                                            # .config=2 (repeated)
    return b


def cc(cid, atype, entries=()):
    return sfield(1, cid) + ld(2, action(atype, entries))       # ComponentControl{id=1,action=2}


def message(cc_bytes):
    cmd = ld(1, cc_bytes)                                        # Command.component_control=1
    return sfield(1, "command") + ld(3, cmd)                    # Message{topic=1,command=3}


def frame(cc_bytes):
    m = message(cc_bytes)
    return struct.pack('>I', len(m)) + m


def build(argv):
    """Return (list_of_cc_bytes, hold_sec). Multiple frames only for 'drive --hold'."""
    op = argv[0]
    if op == "drive":
        lin, ang = float(argv[1]), float(argv[2])
        hold = 0.0
        if "--hold" in argv:
            hold = float(argv[argv.index("--hold") + 1])
        return cc("drive", CONFIGURE, [map_entry("linear", sv_f32(lin)),
                                       map_entry("angular", sv_f32(ang))]), hold
    if op == "stop":
        return cc("drive", DISABLE), 0.0
    if op == "on":
        return cc(argv[1], ENABLE), 0.0
    if op == "off":
        return cc(argv[1], DISABLE), 0.0
    if op == "speed":
        return cc(argv[1], CONFIGURE, [map_entry("speed", sv_u32(int(argv[2])))]), 0.0
    if op == "lidar":
        return cc("lidar", ENABLE if argv[1] == "on" else DISABLE), 0.0
    raise SystemExit(f"unknown op: {op}")


def main():
    if len(sys.argv) < 3:
        raise SystemExit(__doc__)
    host = sys.argv[1]
    cc_bytes, hold = build(sys.argv[2:])
    with socket.create_connection((host, PORT), timeout=5) as s:
        if hold > 0:
            t0 = time.time()
            while time.time() - t0 < hold:
                s.sendall(frame(cc_bytes))
                time.sleep(0.1)
            s.sendall(frame(cc("drive", DISABLE)))   # explicit stop at the end
            print(f"drive held {hold}s, then stop")
        else:
            s.sendall(frame(cc_bytes))
            time.sleep(0.2)
            print("sent:", " ".join(sys.argv[2:]))


if __name__ == "__main__":
    main()
