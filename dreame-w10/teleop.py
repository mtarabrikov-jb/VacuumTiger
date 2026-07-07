#!/usr/bin/env python3
"""Keyboard teleop for the W10 over the avatap-relay control port (see CONTROL.md).

    teleop.py <host>

Keys (hold to move — auto-repeat keeps it alive; release and it ramps to a stop):
    w / Up      forward           q   forward + left      e   forward + right
    s / Down    backward          z   back + left         c   back + right
    a / Left    spin left (CCW)    d / Right   spin right (CW)
    space       stop now          - / =       slower / faster
    x or Ctrl-C quit (releases control back to ava)

Velocity ramps smoothly toward the target (accel-limited), so starts/stops and
direction changes are gentle. "Dead-man": if no key arrives for ~0.35 s the target
goes to 0 and the robot coasts to a stop. The tap adds its own watchdog, speed
clamp and cliff/bump gate. Drive on open floor away from stairs.
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
DEADMAN_S = 0.35          # no key for this long -> target 0
STEP_MM_S = 40.0         # initial linear speed magnitude
ROT_AT_FULL = 0.6        # rotation magnitude at STEP_MM_S (scales with step)
STEP_MIN, STEP_MAX = 10.0, 150.0
LIN_ACCEL = 150.0        # mm/s^2  ramp rate
ROT_ACCEL = 3.0          # rad/s^2 ramp rate

ARROWS = {b"\x1b[A": "w", b"\x1b[B": "s", b"\x1b[D": "a", b"\x1b[C": "d"}


def read_keys():
    """Drain all pending keypresses; return normalized tokens."""
    out = []
    while select.select([sys.stdin], [], [], 0)[0]:
        ch = os.read(sys.stdin.fileno(), 3)  # up to an arrow escape seq
        if ch in ARROWS:
            out.append(ARROWS[ch])
        else:
            out.extend(chr(b) for b in ch)
    return out


def approach(cur, tgt, rate, dt):
    """Move `cur` toward `tgt` by at most `rate*dt`."""
    step = rate * dt
    if tgt - cur > step:
        return cur + step
    if cur - tgt > step:
        return cur - step
    return tgt


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
    lin = rot = 0.0            # current (sent) velocity
    lin_t = rot_t = 0.0        # target velocity
    last_key = 0.0
    dt = 1.0 / SEND_HZ
    print("teleop: WASD/arrows + QEZC diagonals, space=stop, -/= speed, x=quit\r")
    try:
        while True:
            now = time.time()
            r = ROT_AT_FULL * step / STEP_MM_S
            for k in read_keys():
                if k in ("x", "\x03"):
                    raise KeyboardInterrupt
                elif k == "w":
                    lin_t, rot_t, last_key = step, 0.0, now
                elif k == "s":
                    lin_t, rot_t, last_key = -step, 0.0, now
                elif k == "a":
                    lin_t, rot_t, last_key = 0.0, r, now
                elif k == "d":
                    lin_t, rot_t, last_key = 0.0, -r, now
                elif k == "q":
                    lin_t, rot_t, last_key = step, r, now
                elif k == "e":
                    lin_t, rot_t, last_key = step, -r, now
                elif k == "z":
                    lin_t, rot_t, last_key = -step, r, now
                elif k == "c":
                    lin_t, rot_t, last_key = -step, -r, now
                elif k == " ":
                    lin_t = rot_t = lin = rot = 0.0  # hard stop, no ramp
                elif k in ("=", "+"):
                    step = min(STEP_MAX, step + 10)
                elif k in ("-", "_"):
                    step = max(STEP_MIN, step - 10)
            # dead-man: no key recently -> ramp the target to 0
            if now - last_key > DEADMAN_S:
                lin_t = rot_t = 0.0
            # accel-limited ramp toward the target
            lin = approach(lin, lin_t, LIN_ACCEL, dt)
            rot = approach(rot, rot_t, ROT_ACCEL, dt)
            sock.sendall(f"{lin:.1f} {rot:.3f}\n".encode())
            sys.stdout.write(
                f"\rlin={lin:+6.1f} mm/s  rot={rot:+.2f} rad/s  step={step:.0f}   "
            )
            sys.stdout.flush()
            time.sleep(dt)
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
