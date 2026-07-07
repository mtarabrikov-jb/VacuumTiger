#!/usr/bin/env python3
"""Keyboard teleop for the W10 over the avatap-relay control port (see CONTROL.md).

    teleop.py <host>

Keys (hold to move — auto-repeat keeps it alive; release and it coasts to a stop):
    w / Up      forward           a / Left    spin left (CCW)
    s / Down    backward          d / Right   spin right (CW)
    space       stop now          - / =       slower / faster
    q or Ctrl-C quit (releases control back to ava)

"Dead-man": if no key arrives for ~0.35 s the command decays to 0, so letting go
stops the robot. The tap adds its own watchdog, speed clamp and cliff/bump gate.
Drive on open floor away from stairs.
"""
import os
import select
import socket
import sys
import termios
import time
import tty

CONTROL_PORT = 7705
SEND_HZ = 20.0
DEADMAN_S = 0.35          # no key for this long -> stop
STEP_MM_S = 40.0         # initial linear speed magnitude
ROT_RAD_S = 0.6          # rotation magnitude (scales with speed)
STEP_MIN, STEP_MAX = 10.0, 150.0


def read_keys():
    """Drain all pending keypresses; return a list of normalized tokens."""
    out = []
    while select.select([sys.stdin], [], [], 0)[0]:
        ch = os.read(sys.stdin.fileno(), 3)  # up to an arrow escape seq
        if ch in (b"\x1b[A",):
            out.append("w")
        elif ch in (b"\x1b[B",):
            out.append("s")
        elif ch in (b"\x1b[D",):
            out.append("a")
        elif ch in (b"\x1b[C",):
            out.append("d")
        else:
            for b in ch:
                out.append(chr(b))
    return out


def main() -> int:
    if len(sys.argv) != 2:
        print(__doc__)
        return 2
    host = sys.argv[1]
    sock = socket.create_connection((host, CONTROL_PORT), timeout=5)

    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    tty.setcbreak(fd)
    step = STEP_MM_S
    lin = rot = 0.0
    last_key = 0.0
    print("teleop: WASD/arrows to drive, space=stop, -/= speed, q=quit\r")
    try:
        while True:
            now = time.time()
            for k in read_keys():
                if k in ("q", "\x03"):
                    raise KeyboardInterrupt
                elif k == "w":
                    lin, rot, last_key = step, 0.0, now
                elif k == "s":
                    lin, rot, last_key = -step, 0.0, now
                elif k == "a":
                    lin, rot, last_key = 0.0, ROT_RAD_S * step / STEP_MM_S, now
                elif k == "d":
                    lin, rot, last_key = 0.0, -ROT_RAD_S * step / STEP_MM_S, now
                elif k == " ":
                    lin = rot = 0.0
                elif k in ("=", "+"):
                    step = min(STEP_MAX, step + 10)
                elif k in ("-", "_"):
                    step = max(STEP_MIN, step - 10)
            # dead-man: no key recently -> decay to a stop
            if now - last_key > DEADMAN_S:
                lin = rot = 0.0
            sock.sendall(f"{lin:.1f} {rot:.3f}\n".encode())
            sys.stdout.write(f"\rlin={lin:+6.1f} mm/s  rot={rot:+.2f} rad/s  step={step:.0f}   ")
            sys.stdout.flush()
            time.sleep(1.0 / SEND_HZ)
    except (KeyboardInterrupt, BrokenPipeError, ConnectionResetError):
        pass
    finally:
        try:
            sock.sendall(b"stop\n")
            time.sleep(0.15)
            sock.close()
        except OSError:
            pass
        termios.tcsetattr(fd, termios.TCSADRAIN, old)
        print("\r\nteleop: stopped, control released")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
