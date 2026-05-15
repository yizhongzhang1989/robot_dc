#!/usr/bin/env python3
"""
print_joints.py — live joint-angle monitor for any UR robot (real or URSim).

The easiest possible "is the robot moving?" check: opens one URScript socket
and prints joint angles + TCP pose at a configurable rate. No ROS, no driver,
no RViz. Re-uses the proven URRobot client from this repo.

Usage (after sourcing the workspace):

    cd /home/robot/Documents/robot_dc/colcon_ws && source install/setup.bash
    python3 ../scripts/print_joints.py 192.168.1.16
    python3 ../scripts/print_joints.py 192.168.1.16 --rate 20 --scroll

Press Ctrl-C to stop. Columns: joint angles (deg), TCP position (mm),
TCP rotation vector (deg).
"""

import argparse
import math
import os
import sys
import time

# Make stdout line-buffered so output streams instead of getting stuck in a
# block buffer when redirected to a pipe/file. Equivalent to running
# `python3 -u`, but works even when invoked without -u.
try:
    sys.stdout.reconfigure(line_buffering=True)  # py3.7+
except AttributeError:
    pass
os.environ.setdefault("PYTHONUNBUFFERED", "1")

from ur_robot_arm.ur_robot import URRobot


def parse_args():
    p = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    p.add_argument("ip", nargs="?", default="192.168.1.16",
                   help="UR robot IP (default: 192.168.1.16)")
    p.add_argument("--port", type=int, default=30002,
                   help="URScript primary port (default: 30002)")
    p.add_argument("--rate", type=float, default=5.0,
                   help="print rate in Hz (default: 5)")
    p.add_argument("--scroll", action="store_true",
                   help="scroll instead of overwriting a single line")
    return p.parse_args()


HEADER = (
    "  " + "  ".join(f"j{i}(deg)".rjust(8) for i in range(6))
    + "  |  " + "  ".join(f"{a}(mm)".rjust(7) for a in ("x", "y", "z"))
    + "  " + "  ".join(f"r{a}(deg)".rjust(7) for a in ("x", "y", "z"))
)


def fmt_row(joints_rad, tcp_pose):
    joints_deg = [math.degrees(v) for v in joints_rad]
    xyz_mm = [v * 1000.0 for v in tcp_pose[:3]]
    rpy_deg = [math.degrees(v) for v in tcp_pose[3:]]
    return ("  " + "  ".join(f"{v:+8.2f}" for v in joints_deg)
            + "  |  " + "  ".join(f"{v:+7.1f}" for v in xyz_mm)
            + "  " + "  ".join(f"{v:+7.2f}" for v in rpy_deg))


def main():
    args = parse_args()
    print(f"connecting to {args.ip}:{args.port}  ({args.rate} Hz, Ctrl-C to quit)\n")
    r = URRobot(args.ip, args.port)
    r.open()
    print(HEADER)
    period = 1.0 / max(args.rate, 0.1)
    try:
        while True:
            t0 = time.time()
            try:
                q = r.get_actual_joint_positions()
                p = r.get_actual_tcp_pose()
            except Exception as e:
                sys.stdout.write(f"\nread error: {e}; retrying …\n")
                time.sleep(0.5)
                continue
            line = fmt_row(q, p)
            if args.scroll or not sys.stdout.isatty():
                print(line)
            else:
                sys.stdout.write("\r" + line)
                sys.stdout.flush()
            dt = time.time() - t0
            if dt < period:
                time.sleep(period - dt)
    except KeyboardInterrupt:
        print("\nbye.")
    finally:
        try:
            r.close()
        except Exception:
            pass


if __name__ == "__main__":
    main()
