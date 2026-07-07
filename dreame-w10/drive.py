#!/usr/bin/env python3
"""Drive the W10 through the avatap-relay control port (path 2, see docs/CONTROL.md).

Streams a velocity command to port 7705 at 10 Hz for a fixed duration, then sends
"stop" and disconnects (releasing control back to ava). The 10 Hz cadence keeps
the tap's watchdog fresh; stopping the stream makes the robot halt on its own.

    drive.py <host> <linear_mm_s> <rot_rad_s> <seconds>

Drive only on open floor away from stairs. The tap clamps speed and blocks
forward motion into a detected cliff/bumper, but that is not a full safety net.
"""
import socket
import sys
import time

CONTROL_PORT = 7705


def main() -> int:
    if len(sys.argv) != 5:
        print(__doc__)
        return 2
    host = sys.argv[1]
    linear = float(sys.argv[2])
    rot = float(sys.argv[3])
    secs = float(sys.argv[4])

    s = socket.create_connection((host, CONTROL_PORT), timeout=5)
    print(f"control: {linear} mm/s, {rot} rad/s for {secs}s (10 Hz)")
    t0 = time.time()
    try:
        while time.time() - t0 < secs:
            s.sendall(f"{linear} {rot}\n".encode())
            time.sleep(0.1)
    finally:
        s.sendall(b"stop\n")
        time.sleep(0.2)
        s.close()
        print("stopped, control released")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
