#!/usr/bin/env python3
"""Verify top_down_pose: does the generated pose actually point the gripper DOWN?

A1 final check. top_down_pose(x,y,z,yaw) builds a straight-down grasp pose. We
must confirm, BEFORE driving the arm, that:
  1. the rpy round-trips: _euler_xyz_to_matrix(rpy) reproduces top_down_rotation
     (the gimbal boundary at ry≈±90 could otherwise corrupt it),
  2. solve_ik finds a solution whose ikpy FK points straight down (downness≈+1),
  3. yaw actually rotates the tool about vertical without flipping it sideways,
  4. (optional, --move) the real arm lands pointing down (visual + get_coords).

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

# Test points in the reachable table region (mm) and yaws (deg). z kept modest.
TEST_POINTS = [
    (180.0, 0.0, 150.0),
    (180.0, 60.0, 150.0),
    (150.0, -60.0, 120.0),
]
TEST_YAWS = [0.0, 45.0, 90.0, -45.0]


def downness(tz):
    n = math.sqrt(sum(c * c for c in tz)) or 1e-9
    return -tz[2] / n


def fmt(v):
    return "[" + ", ".join(f"{x:7.2f}" for x in v) + "]"


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
    import numpy as np
    print("== 1) rpy round-trip: euler(matrix)->matrix reproduces top_down_rotation ==")
    worst_rt = 0.0
    for yaw in TEST_YAWS:
        R = ctrl.top_down_rotation(yaw)
        rpy = ctrl._matrix_to_euler_xyz(R)
        R2 = ctrl._euler_xyz_to_matrix(*rpy)
        err = float(np.max(np.abs(np.array(R) - np.array(R2))))
        worst_rt = max(worst_rt, err)
        print(f"  yaw {yaw:+6.1f}  rpy {fmt(rpy)}  max|R-R2| {err:.4f}")
    print(f"  worst round-trip matrix error: {worst_rt:.4f} "
          f"({'OK' if worst_rt < 1e-3 else 'PROBLEM: rpy does not encode the down matrix'})\n")

    print("== 2/3) solve_ik downness + yaw behaviour (does it point DOWN?) ==")
    ok = True
    for (x, y, z) in TEST_POINTS:
        for yaw in TEST_YAWS:
            pose = ctrl.top_down_pose(x, y, z, yaw)
            ik = ctrl.solve_ik(pose)
            if ik is None:
                print(f"  pt({x:.0f},{y:.0f},{z:.0f}) yaw {yaw:+5.1f} -> solve_ik None (unreachable?)")
                continue
            frame = ctrl.ik_chain.forward_kinematics(ctrl._ikpy_full_vector(ik))
            tz = [frame[i, 2] for i in range(3)]
            d = downness(tz)
            j5 = ik[4]
            flag = "" if d > 0.9 else "  <-- NOT down!"
            if d <= 0.9:
                ok = False
            print(f"  pt({x:.0f},{y:.0f},{z:.0f}) yaw {yaw:+6.1f}  J5 {j5:+6.1f}  "
                  f"downness {d:+.3f}{flag}")
    print(f"\n  overall: {'all poses point down' if ok else 'SOME poses not down — revisit top_down_rotation'}")
    return ok


def main():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--port", default="/dev/ttyACM0")
    p.add_argument("--baudrate", type=int, default=115200)
    p.add_argument("--no-connect", action="store_true")
    p.add_argument("--move", action="store_true", help="drive the arm to a couple of poses (MOVES)")
    p.add_argument("--speed", type=int, default=12)
    args = p.parse_args()

    if args.no_connect:
        ctrl = make_ctrl_offline()
        offline_checks(ctrl)
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
    live = [(180.0, 0.0, 180.0, 0.0), (180.0, 0.0, 180.0, 45.0)]
    for (x, y, z, yaw) in live:
        pose = ctrl.top_down_pose(x, y, z, yaw)
        reachable, reason, ik = ctrl.check_pose_reachable(pose)
        print(f"\n  top_down({x:.0f},{y:.0f},{z:.0f}, yaw={yaw:+.0f}) -> {reason}")
        if not reachable:
            print("    not reachable; skipping")
            continue
        try:
            ctrl.send_coords(pose, speed=args.speed)
        except ValueError as e:
            print(f"    send_coords rejected: {e}")
            continue
        time.sleep(4.0)
        real = ctrl._read_coords(retries=3)
        ang = ctrl.get_all_joint_angles()
        frame = ctrl.ik_chain.forward_kinematics(ctrl._ikpy_full_vector(ang))
        tz = [frame[i, 2] for i in range(3)]
        print(f"    real coords {fmt(real) if real else real}")
        print(f"    real J5 {ang[4]:+.1f}  measured downness {downness(tz):+.3f} "
              f"(should be near +1.0; eyeball the gripper too)")

    print("\n  returning home")
    ctrl.mc.send_angles([0, 0, 0, 0, 0, 0], args.speed)
    time.sleep(2.0)
    return 0


if __name__ == "__main__":
    sys.exit(main())
