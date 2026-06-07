#!/usr/bin/env python3
"""Verify self-computed inverse kinematics before letting it drive the arm.

WHY: this unit's firmware IK is broken, so we compute IK ourselves (ikpy) and
send joint angles. Before trusting that for real motion, we must confirm the IK
is self-consistent and matches the real robot. Two checks, escalating in trust:

1. OFFLINE round-trip (no robot, but needs ikpy + URDF):
       angles -> ikpy FK -> coords -> solve_ik -> angles'
   angles' should reproduce the original end-effector pose (FK(angles') ~= the
   coords we asked for). We compare in POSE space (mm/deg), not joint space,
   because a 6-DOF arm can have multiple valid joint solutions for one pose.

2. LIVE (robot moves): for each pose, send_angles, read the true get_coords,
   feed that real pose into solve_ik, then send the IK angles and read get_coords
   again -- the second pose should match the first within a few mm. This proves
   the whole self-IK -> send_angles path lands where intended on real hardware.

This mirrors scripts/verify_fk.py. Run OFFLINE round-trip first; only run --move
once the offline pose error is small and the area is clear (e-stop in reach).

Usage (on the robot host, in the project venv):
    python3 scripts/verify_ik.py --no-connect      # offline FK->IK->FK round-trip
    python3 scripts/verify_ik.py --port /dev/ttyACM0 --move --speed 20
"""

import argparse
import math
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

# Joint poses (deg, J1..J6) spanning part of the workspace. Home first; modest
# so --move stays safe.
TEST_POSES_DEG = [
    [0, 0, 0, 0, 0, 0],
    [20, 0, 0, 0, 0, 0],
    [0, -20, 0, 0, 0, 0],
    [0, 0, 20, 0, 0, 0],
    [20, -20, 20, 0, 0, 0],
    [-20, -20, 30, -15, 20, 0],
]


def pos_err(a, b):
    return math.sqrt(sum((x - y) ** 2 for x, y in zip(a[:3], b[:3])))


def ori_err(ctrl, a, b):
    return max(ctrl._angle_delta(x, y) for x, y in zip(a[3:], b[3:]))


def fmt(c):
    return "[" + ", ".join(f"{v:7.2f}" for v in c) + "]"


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--port", default="/dev/ttyACM0")
    parser.add_argument("--baudrate", type=int, default=115200)
    parser.add_argument("--no-connect", action="store_true",
                        help="Offline FK->IK->FK round-trip only (needs ikpy+URDF, no robot)")
    parser.add_argument("--move", action="store_true",
                        help="Also drive the arm to validate the live path (ROBOT MOVES)")
    parser.add_argument("--speed", type=int, default=20)
    args = parser.parse_args()

    # Build a controller WITHOUT opening the serial port for the offline check.
    # We instantiate just enough to reuse its ikpy chain + IK methods. To avoid
    # connecting, we construct the object lazily: for --no-connect we only need
    # the kinematic helpers, which do not touch the serial port.
    from mycobot_joint_controller import MyCobotJointController

    if args.no_connect:
        # Build a bare instance without __init__ touching hardware: make a shell
        # object and initialise only the IK chain + limits we need.
        ctrl = MyCobotJointController.__new__(MyCobotJointController)
        ctrl.joint_limits = {
            1: (-168, 168), 2: (-135, 135), 3: (-150, 150),
            4: (-145, 145), 5: (-155, 160), 6: (-180, 180),
        }
        ctrl.coords_min = [-350.0, -350.0, -70.0, -180.0, -180.0, -180.0]
        ctrl.coords_max = [350.0, 350.0, 523.9, 180.0, 180.0, 180.0]
        ctrl.ik_chain = None
        ctrl.ik_active_idx = None
        ctrl.ik_error = None
        try:
            ctrl._init_ik_chain()
        except Exception as exc:
            print(f"IK chain init failed: {exc}")
            return 1
        print(f"ikpy chain ready: {len(ctrl.ik_chain.links)} links, "
              f"active idx {ctrl.ik_active_idx}\n")

        print("== OFFLINE round-trip: angles -> FK -> coords -> IK -> FK ==")
        print("   (pose_err is FK(IK(FK(angles))) vs FK(angles); small = consistent)")
        worst = 0.0
        for p in TEST_POSES_DEG:
            coords = ctrl.ikpy_fk(p)                 # FK of the original angles
            ik = ctrl.solve_ik(coords, current_angles=p)
            if ik is None:
                print(f"  angles {fmt(p)} -> IK returned None (no solution!)")
                continue
            coords2 = ctrl.ikpy_fk(ik)               # FK of the IK solution
            pe = pos_err(coords, coords2)
            oe = ori_err(ctrl, coords, coords2)
            worst = max(worst, pe)
            print(f"  angles {fmt(p)}")
            print(f"      target coords {fmt(coords)}")
            print(f"      IK angles     {fmt(ik)}")
            print(f"      pose_err      {pe:6.2f} mm / {oe:5.2f} deg")
        print(f"\n  worst position round-trip error: {worst:.2f} mm")
        return 0

    # ---- connected paths ----
    ctrl = MyCobotJointController(args.port, args.baudrate)
    if ctrl.ik_chain is None:
        print(f"IK chain unavailable: {ctrl.ik_error}")
        return 1

    if not args.move:
        print("Connected, but no --move given. Running offline round-trip on the "
              "loaded chain (no motion).")
        for p in TEST_POSES_DEG:
            coords = ctrl.ikpy_fk(p)
            ik = ctrl.solve_ik(coords, current_angles=p)
            tag = "None" if ik is None else fmt(ik)
            print(f"  {fmt(p)} -> coords {fmt(coords)} -> IK {tag}")
        return 0

    ip = getattr(ctrl.mc, "is_power_on", None)
    if ip is not None:
        try:
            if ip() == 0:
                print("Servos not powered (is_power_on()==0); skipping --move.")
                return 0
        except Exception:
            pass

    print("== LIVE: send_angles -> read real coords -> IK -> send_angles -> recheck ==")
    print("   (land_err is the gap between the first real pose and where IK landed)")
    for p in TEST_POSES_DEG:
        print(f"\n  -> moving to seed pose {fmt(p)} @ {args.speed}")
        ctrl.mc.send_angles(list(p), args.speed)
        _settle(ctrl, p)
        real = ctrl._read_coords(retries=3)
        if real is None:
            print("      could not read coords; skipping")
            continue
        print(f"      real pose      {fmt(real)}")

        ik = ctrl.solve_ik(real)
        if ik is None:
            print("      solve_ik(real pose) -> None (UNEXPECTED for a real pose!)")
            continue
        print(f"      IK angles      {fmt(ik)}")

        # Drive to the IK solution and see where we actually land.
        ctrl.mc.send_angles(ik, args.speed)
        _settle(ctrl, ik)
        landed = ctrl._read_coords(retries=3)
        if landed is None:
            print("      could not read landed coords; skipping")
            continue
        print(f"      landed pose    {fmt(landed)}")
        print(f"      land_err       {pos_err(real, landed):6.2f} mm / "
              f"{ori_err(ctrl, real, landed):5.2f} deg")

    print("\n  returning to home [0,0,0,0,0,0]")
    ctrl.mc.send_angles([0, 0, 0, 0, 0, 0], args.speed)
    time.sleep(2.0)
    return 0


def _settle(ctrl, target_angles, cap=12.0):
    t0 = time.time()
    while time.time() - t0 < cap:
        try:
            ang = ctrl.mc.get_angles()
        except Exception:
            ang = None
        if isinstance(ang, (list, tuple)) and len(ang) == 6:
            if max(abs(c - t) for c, t in zip(ang, target_angles)) <= 2.0:
                break
        time.sleep(0.3)
    time.sleep(0.5)


if __name__ == "__main__":
    sys.exit(main())
