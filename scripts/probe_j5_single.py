#!/usr/bin/env python3
"""Isolate the J5 high-speed swing: send ONE pose, log the joint trajectory.

A1 follow-up. The top-down probe showed J5 swinging fast after the arm reached a
down-ish pose. To tell whether that comes from (a) chained send_angles / weak
settle / fresh-mode, or (b) reaching the single down pose itself (e.g. wrist
singularity at J5 ~ -90), this script sends exactly ONE motion command and then
just WATCHES get_angles over time. No settle loop, no second command.

Two modes:
  --pose reach   : home -> [0,-40,-40,0,0,0]      (J5 stays 0; baseline, safe)
  --pose down    : home -> [0,-40,-40,0,-90,0]    (drives J5 to -90 in one shot)
  --pose j5only  : assumes already at 'reach', then moves ONLY J5 0 -> -90
                   (J1-J4,J6 unchanged; tests J5-in-isolation toward singularity)

It logs (t, J1..J6) every ~0.2 s for --watch seconds, then reports J5's peak
step-to-step change (deg/sample) so a "swing" shows up as a large spike.

This MOVES the arm; clear the area, e-stop in reach. Single low-speed command.

Usage (robot host, project venv, server stopped):
    ./mycobot_server_ctl.sh stop
    python3 scripts/probe_j5_single.py --pose reach  --speed 15
    python3 scripts/probe_j5_single.py --pose down   --speed 15
    python3 scripts/probe_j5_single.py --pose j5only --speed 15   # run after 'reach'
    ./mycobot_server_ctl.sh start
"""

import argparse
import sys
import time

POSES = {
    "reach":  [0, -40, -40, 0, 0, 0],
    "down":   [0, -40, -40, 0, -90, 0],
    # j5only is resolved at runtime from the current pose (J5 -> -90)
}


def fmt(a):
    return "[" + ", ".join(f"{x:6.1f}" for x in a) + "]"


def read_angles(mc, retries=2):
    for _ in range(retries):
        try:
            a = mc.get_angles()
        except Exception:
            a = None
        if isinstance(a, (list, tuple)) and len(a) == 6:
            return list(a)
        time.sleep(0.05)
    return None


def main():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--port", default="/dev/ttyACM0")
    p.add_argument("--baudrate", type=int, default=115200)
    p.add_argument("--pose", choices=["reach", "down", "j5only"], required=True)
    p.add_argument("--speed", type=int, default=15)
    p.add_argument("--watch", type=float, default=10.0,
                   help="seconds to log the trajectory after the single command")
    args = p.parse_args()

    from pymycobot import MyCobot
    mc = MyCobot(args.port, args.baudrate)
    time.sleep(0.5)

    start = read_angles(mc)
    print(f"start angles: {fmt(start) if start else start}")
    if start is None:
        print("could not read start angles; aborting")
        return 1

    if args.pose == "j5only":
        target = list(start)
        target[4] = -90.0  # J5 only
    else:
        target = list(POSES[args.pose])

    print(f"sending ONE command -> {fmt(target)} @ speed {args.speed}")
    print("logging get_angles; watch J5 (index 5) for a fast swing\n")
    print(f"  {'t(s)':>5}  {'J1':>6} {'J2':>6} {'J3':>6} {'J4':>6} {'J5':>6} {'J6':>6}  dJ5/step")

    mc.send_angles(target, args.speed)

    t0 = time.time()
    prev_j5 = start[4]
    max_dj5 = 0.0
    max_dj5_t = 0.0
    while time.time() - t0 < args.watch:
        a = read_angles(mc, retries=1)
        t = time.time() - t0
        if a is None:
            print(f"  {t:5.1f}  <read failed>")
            time.sleep(0.2)
            continue
        dj5 = a[4] - prev_j5
        prev_j5 = a[4]
        if abs(dj5) > abs(max_dj5):
            max_dj5, max_dj5_t = dj5, t
        flag = "  <-- big J5 step" if abs(dj5) > 15 else ""
        print(f"  {t:5.1f}  {a[0]:6.1f} {a[1]:6.1f} {a[2]:6.1f} {a[3]:6.1f} {a[4]:6.1f} {a[5]:6.1f}  {dj5:+6.1f}{flag}")
        time.sleep(0.2)

    print(f"\n  peak J5 step: {max_dj5:+.1f} deg/sample at t={max_dj5_t:.1f}s")
    print("  (a large isolated spike = the 'swing'; steady ramp = normal motion)")
    print("\n  NOT auto-returning home (so you can inspect the final pose).")
    print("  To return: python3 -c \"from pymycobot import MyCobot;import time;"
          "mc=MyCobot('%s',%d);time.sleep(0.5);mc.send_angles([0,0,0,0,0,0],%d)\""
          % (args.port, args.baudrate, args.speed))
    return 0


if __name__ == "__main__":
    sys.exit(main())
