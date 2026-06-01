#!/usr/bin/env python3
"""Verify solve_topdown_ik: straight-down grasp with controllable gripper yaw.

A1 final check. solve_topdown_ik(x,y,z,yaw) constrains only the approach axis
(tool-Z) to straight down (ikpy orientation_mode="Z") and sets the gripper yaw by
overriding J6. We confirm, BEFORE driving the arm, that across the reachable
table region and a range of yaws:
  - a solution is found,
  - the gripper points straight down (downness >= 0.98),
  - J5 stays near 0 (away from the servo-strain region near +/-90),
  - the achieved gripper yaw (azimuth of tool-X) matches the request (or its
    180 deg flip, since parallel jaws are symmetric).

Offline checks need ikpy + URDF only. --move drives the arm (clear area, e-stop).

Usage (robot host, project venv):
    python3 scripts/verify_topdown.py --no-connect
    python3 scripts/verify_topdown.py --port /dev/ttyACM0 --move --speed 12
"""

import argparse
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

TEST_POINTS = [(180.0, 0.0, 150.0), (180.0, 60.0, 150.0),
               (150.0, -60.0, 120.0), (200.0, 0.0, 100.0)]
TEST_YAWS = [0.0, 45.0, 90.0, -45.0]


def downness(R):
    tz = [R[i, 2] for i in range(3)]
    n = math.sqrt(sum(c * c for c in tz)) or 1e-9
    return -tz[2] / n


def yaw_match(achieved, requested):
    """Smallest error allowing a 180 deg flip (symmetric parallel jaws)."""
    e1 = abs((achieved - requested + 180) % 360 - 180)
    e2 = abs((achieved - requested + 360) % 360 - 180)  # +180 flip
    return min(e1, e2)


def make_ctrl_offline():
    from mycobot_joint_controller import MyCobotJointController
    c = MyCobotJointController.__new__(MyCobotJointController)
    c.joint_limits = {1: (-168, 168), 2: (-135, 135), 3: (-150, 150),
                      4: (-145, 145), 5: (-155, 160), 6: (-180, 180)}
    c.coords_min = [-350.0, -350.0, -70.0, -180.0, -180.0, -180.0]
    c.coords_max = [350.0, 350.0, 523.9, 180.0, 180.0, 180.0]
    c.ik_chain = None
    c.ik_active_idx = None
    c.ik_error = None
    c._init_ik_chain()
    return c


def offline_checks(ctrl):
    print("== solve_topdown_ik: down + yaw across the table region ==")
    print("   want: downness>=0.98, J5 near 0, yaw_err small (180-flip allowed)")
    ok = True
    for (x, y, z) in TEST_POINTS:
        for yaw in TEST_YAWS:
            ang = ctrl.solve_topdown_ik(x, y, z, yaw_deg=yaw,
                                        current_angles=[0, -40, -40, 0, 0, 0])
            if ang is None:
                print(f"  pt({x:.0f},{y:.0f},{z:.0f}) yaw {yaw:+6.1f} -> None")
                ok = False
                continue
            frame = ctrl.ik_chain.forward_kinematics(ctrl._ikpy_full_vector(ang))
            R = frame[:3, :3]
            d = downness(R)
            ach_yaw = math.degrees(math.atan2(R[1, 0], R[0, 0]))
            yerr = yaw_match(ach_yaw, yaw)
            bad = d < 0.98 or yerr > 5.0 or abs(ang[4]) > 30
            ok = ok and not bad
            flag = "" if not bad else "  <-- PROBLEM"
            print(f"  pt({x:.0f},{y:.0f},{z:.0f}) yaw {yaw:+6.1f}  J5 {ang[4]:+6.1f} "
                  f"J6 {ang[5]:+7.1f}  downness {d:+.3f}  yaw_err {yerr:4.1f}{flag}")
    print(f"\n  overall: {'OK — straight down with correct yaw' if ok else 'PROBLEM'}")
    return ok


def main():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--port", default="/dev/ttyACM0")
    p.add_argument("--baudrate", type=int, default=115200)
    p.add_argument("--no-connect", action="store_true")
    p.add_argument("--move", action="store_true", help="drive the arm (MOVES)")
    p.add_argument("--speed", type=int, default=12)
    args = p.parse_args()

    if args.no_connect:
        offline_checks(make_ctrl_offline())
        return 0

    from mycobot_joint_controller import MyCobotJointController
    ctrl = MyCobotJointController(args.port, args.baudrate)
    if ctrl.ik_chain is None:
        print(f"IK chain unavailable: {ctrl.ik_error}")
        return 1
    offline_checks(ctrl)

    if not args.move:
        print("\n(no --move; not driving the arm)")
        return 0

    import time
    ip = getattr(ctrl.mc, "is_power_on", None)
    if ip is not None:
        try:
            if ip() == 0:
                print("Servos not powered; skipping --move.")
                return 0
        except Exception:
            pass

    print("\n== LIVE: drive to a few top-down poses (ROBOT MOVES) ==")
    for (x, y, z, yaw) in [(180.0, 0.0, 180.0, 0.0), (180.0, 0.0, 180.0, 45.0)]:
        ang = ctrl.solve_topdown_ik(x, y, z, yaw_deg=yaw)
        print(f"\n  topdown({x:.0f},{y:.0f},{z:.0f}, yaw={yaw:+.0f}) -> "
              f"{'IK ok' if ang else 'no solution'}")
        if ang is None:
            continue
        # joint-limit guard (controller does this in send_coords; here direct)
        bad = any(not (ctrl.joint_limits[i][0] <= a <= ctrl.joint_limits[i][1])
                  for i, a in enumerate(ang, 1))
        if bad:
            print("    a joint exceeds its limit; skipping")
            continue
        ctrl.mc.send_angles(ang, args.speed)
        time.sleep(4.0)
        real = ctrl._read_coords(retries=3)
        cur = ctrl.get_all_joint_angles()
        frame = ctrl.ik_chain.forward_kinematics(ctrl._ikpy_full_vector(cur))
        print(f"    real coords {real}")
        print(f"    real J5 {cur[4]:+.1f} J6 {cur[5]:+.1f}  "
              f"downness {downness(frame[:3, :3]):+.3f} (eyeball the gripper too)")

    print("\n  returning home")
    ctrl.mc.send_angles([0, 0, 0, 0, 0, 0], args.speed)
    time.sleep(2.0)
    return 0


if __name__ == "__main__":
    sys.exit(main())
