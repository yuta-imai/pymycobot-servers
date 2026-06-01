#!/usr/bin/env python3
"""Probe the wrist orientation for top-down (vertical) grasps.

GOAL (A1): determine top_down_rpy(yaw) -- the [rx, ry, rz] pose that points the
gripper straight DOWN (tool -Z aligned with base -Z, i.e. pointing at the table)
while rotating the gripper about the vertical by `yaw`. This must be measured on
the real robot because the Euler convention/offset is hardware-specific.

APPROACH: drive to a set of known JOINT poses (safe, slow) that progressively
tip the wrist toward straight-down, and read the resulting get_coords [rx,ry,rz]
plus the ikpy tool-Z axis in base frame. From this we read off:
  1. which rpy corresponds to "gripper down" (tool -Z ~ base -Z), and
  2. how rz tracks a yaw change introduced by J1 and by J6 (which joint is yaw).

This MOVES the arm. Clear the area; keep e-stop in reach. Joint poses are chosen
to stay well inside limits and near the table-reach region. Read-only otherwise.

Usage (robot host, project venv, server stopped to free the serial port):
    ./mycobot_server_ctl.sh stop
    python3 scripts/probe_topdown.py --port /dev/ttyACM0 --speed 20
    ./mycobot_server_ctl.sh start
"""

import argparse
import math
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

# Candidate joint poses (deg, J1..J6). Each is meant to bring the gripper toward
# pointing down. We sweep J5 (wrist pitch) to find "straight down", then vary J1
# and J6 to see which drives yaw. All conservative, inside limits.
#   base reach: J2/J3 tip the arm forward over the table; J5 aims the tool down.
PROBE_POSES = [
    # label,                 J1,  J2,  J3,  J4,  J5,  J6
    ("home",                  0,   0,   0,   0,   0,   0),
    ("reach_fwd",             0, -40, -40,   0,   0,   0),
    ("down_J5_-80",           0, -40, -40,   0, -80,   0),
    ("down_J5_-90",           0, -40, -40,   0, -90,   0),
    ("down_J5_-100",          0, -40, -40,   0,-100,   0),
    # vary J6 (expected wrist/yaw spin) at a down-ish pose
    ("down_J6_+30",           0, -40, -40,   0, -90,  30),
    ("down_J6_-30",           0, -40, -40,   0, -90, -30),
    # vary J1 (base rotation) at a down-ish pose
    ("down_J1_+30",          30, -40, -40,   0, -90,   0),
    ("down_J1_-30",         -30, -40, -40,   0, -90,   0),
]


def tool_axes_base(ctrl, angles_deg):
    """Return (tool_x, tool_y, tool_z) unit axes expressed in the base frame.

    Columns of the FK rotation matrix are the tool axes in base coords. tool_z
    is the gripper approach axis; "straight down" means tool_z ~ [0,0,-1].
    """
    frame = ctrl.ik_chain.forward_kinematics(ctrl._ikpy_full_vector(angles_deg))
    R = frame[:3, :3]
    return ([R[i, 0] for i in range(3)],
            [R[i, 1] for i in range(3)],
            [R[i, 2] for i in range(3)])


def downness(tool_z):
    """Cosine between tool_z and base -Z; 1.0 == perfectly straight down."""
    return -tool_z[2] / max(1e-9, math.sqrt(sum(c * c for c in tool_z)))


def fmt(v):
    return "[" + ", ".join(f"{x:7.2f}" for x in v) + "]"


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--port", default="/dev/ttyACM0")
    parser.add_argument("--baudrate", type=int, default=115200)
    parser.add_argument("--speed", type=int, default=20)
    parser.add_argument("--no-connect", action="store_true",
                        help="ikpy-only: predict rpy / tool-Z per pose, no robot")
    args = parser.parse_args()

    from mycobot_joint_controller import MyCobotJointController

    if args.no_connect:
        ctrl = MyCobotJointController.__new__(MyCobotJointController)
        ctrl.ik_chain = None
        ctrl.ik_active_idx = None
        ctrl.ik_error = None
        ctrl._init_ik_chain()
        print("== ikpy prediction (no robot) ==")
        for label, *p in PROBE_POSES:
            coords = ctrl.ikpy_fk(p)
            _, _, tz = tool_axes_base(ctrl, p)
            print(f"  {label:16} angles {fmt(p)}")
            print(f"       ikpy rpy {fmt(coords[3:])}  toolZ {fmt(tz)}  downness {downness(tz):+.3f}")
        return 0

    ctrl = MyCobotJointController(args.port, args.baudrate)
    if ctrl.ik_chain is None:
        print(f"IK chain unavailable: {ctrl.ik_error}")
        return 1

    ip = getattr(ctrl.mc, "is_power_on", None)
    if ip is not None:
        try:
            if ip() == 0:
                print("Servos not powered (is_power_on()==0); aborting.")
                return 0
        except Exception:
            pass

    print("== LIVE top-down probe (ROBOT MOVES) ==")
    print("   downness: +1.0 = gripper straight down; rpy is the real get_coords pose")
    for label, *p in PROBE_POSES:
        print(f"\n  -> {label}: angles {fmt(p)} @ {args.speed}")
        ctrl.mc.send_angles(list(p), args.speed)
        _settle(ctrl, p)
        real = ctrl._read_coords(retries=3)
        _, _, tz_pred = tool_axes_base(ctrl, p)
        if real is None:
            print("       could not read coords")
            continue
        print(f"       real rpy   {fmt(real[3:])}  (xyz {fmt(real[:3])})")
        print(f"       ikpy toolZ {fmt(tz_pred)}  downness {downness(tz_pred):+.3f}")

    print("\n  returning to home")
    ctrl.mc.send_angles([0, 0, 0, 0, 0, 0], args.speed)
    time.sleep(2.0)
    return 0


def _settle(ctrl, target, cap=12.0):
    t0 = time.time()
    while time.time() - t0 < cap:
        try:
            a = ctrl.mc.get_angles()
        except Exception:
            a = None
        if isinstance(a, (list, tuple)) and len(a) == 6:
            if max(abs(c - t) for c, t in zip(a, target)) <= 2.0:
                break
        time.sleep(0.3)
    time.sleep(0.5)


if __name__ == "__main__":
    sys.exit(main())
