#!/usr/bin/env python3
"""Find the deepest J5 angle the arm can actually HOLD (toward gripper-down).

Finding so far: commanding J5 = -90 (gripper straight down) does NOT hold -- the
servo strains to ~-83, never reaches -90, then springs back past 0, and the
firmware retries forever (visible oscillation). So full vertical grasp is not
achievable on this unit. This script finds the deepest J5 that holds STABLY.

Method: from a fixed forward-reach pose (J2=-40, J3=-40, others ~0), command J5
to a sequence of increasingly negative targets (-50, -60, -70, -75, -80, -83),
one at a time. For each:
  - send the single target,
  - watch get_angles,
  - PASS if J5 settles within `hold_tol` of the target and stays there for
    `hold_secs` without springing back,
  - FAIL (spring-back) if, after first approaching, J5 reverses by more than
    `runaway_deg` away from the target -> immediately send home to kill the
    firmware retry, and STOP the sweep (deepest stable angle is the previous one).
Always returns home at the end.

SAFER than the earlier probes: it actively detects spring-back and aborts to
home instead of leaving the firmware oscillating.

This MOVES the arm; clear the area, e-stop in reach.

Usage (robot host, project venv, server stopped):
    ./mycobot_server_ctl.sh stop
    python3 scripts/probe_j5_depth.py --port /dev/ttyACM0 --speed 12
    ./mycobot_server_ctl.sh start
"""

import argparse
import sys
import time

REACH_BASE = [0.0, -40.0, -40.0, 0.0, 0.0, 0.0]   # J5 filled in per target
J5_TARGETS = [-50.0, -60.0, -70.0, -75.0, -80.0, -83.0]
HOME = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]


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


def go_home(mc, speed):
    print("  -> sending HOME to stop any retry/oscillation")
    mc.send_angles(list(HOME), speed)
    time.sleep(2.5)


def test_target(mc, j5_target, speed, hold_tol, hold_secs, runaway_deg, watch_cap):
    """Return 'hold' | 'spring_back' | 'no_reach'. Aborts to home on spring_back."""
    target = list(REACH_BASE)
    target[4] = j5_target
    print(f"\n== J5 target {j5_target:+.0f} (full pose {target}) @ speed {speed} ==")
    mc.send_angles(target, speed)

    t0 = time.time()
    best = 999.0          # closest (smallest |J5 - target|) seen so far
    best_j5 = None
    reached = False       # True once J5 has actually arrived near the target
    hold_start = None
    print(f"  {'t':>5} {'J5':>7} {'err':>6}  note")
    while time.time() - t0 < watch_cap:
        a = read_angles(mc, retries=1)
        t = time.time() - t0
        if a is None:
            time.sleep(0.2); continue
        j5 = a[4]
        err = abs(j5 - j5_target)
        if err < best:
            best, best_j5 = err, j5

        # Arm spring-back detection ONLY after J5 has genuinely reached the
        # target zone. (Earlier bug: it fired while J5 was still descending past
        # the target, mistaking normal approach for a reversal.)
        if err <= hold_tol:
            reached = True
        if reached and err > runaway_deg:
            print(f"  {t:5.1f} {j5:7.1f} {err:6.1f}  SPRING-BACK (best was {best_j5:.1f})")
            go_home(mc, speed)
            return "spring_back"

        if err <= hold_tol:
            if hold_start is None:
                hold_start = t
                print(f"  {t:5.1f} {j5:7.1f} {err:6.1f}  reached, holding...")
            elif t - hold_start >= hold_secs:
                print(f"  {t:5.1f} {j5:7.1f} {err:6.1f}  HELD {hold_secs:.0f}s OK")
                return "hold"
        else:
            if hold_start is not None:
                print(f"  {t:5.1f} {j5:7.1f} {err:6.1f}  drifted out of tolerance")
            hold_start = None
        time.sleep(0.2)

    print(f"  watch window elapsed; best |err| was {best:.1f} at J5={best_j5}")
    return "no_reach"


def main():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--port", default="/dev/ttyACM0")
    p.add_argument("--baudrate", type=int, default=115200)
    p.add_argument("--speed", type=int, default=12)
    p.add_argument("--hold-tol", type=float, default=2.0, help="deg within target to count as reached")
    p.add_argument("--hold-secs", type=float, default=3.0, help="seconds to hold before PASS")
    p.add_argument("--runaway-deg", type=float, default=8.0,
                   help="reverse beyond this (deg, toward 0) after approaching = spring-back")
    p.add_argument("--watch-cap", type=float, default=9.0, help="max seconds to watch per target")
    args = p.parse_args()

    from pymycobot import MyCobot
    mc = MyCobot(args.port, args.baudrate)
    time.sleep(0.5)

    print("Establishing the forward-reach base pose first (J5=0)...")
    mc.send_angles(list(REACH_BASE), args.speed)
    time.sleep(3.0)

    deepest_hold = None
    try:
        for j5 in J5_TARGETS:
            result = test_target(mc, j5, args.speed,
                                  args.hold_tol, args.hold_secs,
                                  args.runaway_deg, args.watch_cap)
            if result == "hold":
                deepest_hold = j5
                # re-establish reach base before next deeper target
                mc.send_angles(list(REACH_BASE), args.speed)
                time.sleep(2.5)
            else:
                print(f"\n  STOP: J5={j5:+.0f} did not hold ({result}).")
                break
    finally:
        go_home(mc, args.speed)

    print("\n================ RESULT ================")
    if deepest_hold is not None:
        print(f"  deepest J5 that HELD stably: {deepest_hold:+.0f} deg")
        print(f"  (gripper tilt from vertical ~= {90 + deepest_hold:.0f} deg off straight-down)")
    else:
        print("  no tested J5 target held; even -50 failed. Revisit approach.")
    print("  Arm returned to home.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
