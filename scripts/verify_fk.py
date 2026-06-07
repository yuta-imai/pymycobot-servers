#!/usr/bin/env python3
"""Verify that our ikpy forward kinematics agrees with the robot's firmware.

WHY: this unit's firmware inverse kinematics is broken (solve_inv_kinematics
returns -1 for every pose), so we are switching to computing IK ourselves in
Python (ikpy + the MyCobot 280 URDF) and commanding joint angles via send_angles.
Before trusting any self-computed IK, we MUST confirm that our kinematic model
agrees with the real robot's coordinate frame, units, and conventions. The
firmware's FORWARD kinematics works fine, so we use it as ground truth:

    for several joint poses:
        coords_fw  = firmware angles_to_coords / get_coords   (ground truth)
        coords_ik  = ikpy FK of the same joint angles         (our model)
        compare    -> position (mm) and orientation (deg) error

If the position error is small (a few mm), the frames/units/offsets match and we
can build IK on top with confidence. A large or structured error means the URDF
frame differs from the firmware's base/TCP frame and must be reconciled FIRST
(an offset transform) before any IK-driven motion -- otherwise we repeat the
send_coords runaway.

This script is mostly OFFLINE: by default it only reads. With --move it will
command a few small, safe joint poses via send_angles to gather live get_coords
samples (the robot moves -- clear the area, keep e-stop in reach).

Usage (on the robot host, in the project venv):
    # offline: compare ikpy FK against firmware angles_to_coords only (no motion)
    python3 scripts/verify_fk.py --port /dev/ttyACM0

    # also move to a few safe poses and compare against live get_coords
    python3 scripts/verify_fk.py --port /dev/ttyACM0 --move

    # pure model check, no robot connection at all
    python3 scripts/verify_fk.py --no-connect
"""

import argparse
import math
import os
import sys
import time

# A handful of joint poses (degrees, J1..J6) to test the FK mapping over a range
# of the workspace. Kept modest so --move stays safe. Home first.
TEST_POSES_DEG = [
    [0, 0, 0, 0, 0, 0],
    [20, 0, 0, 0, 0, 0],
    [0, -30, 0, 0, 0, 0],
    [0, 0, 30, 0, 0, 0],
    [30, -20, 20, 0, 0, 0],
    [-30, -30, 40, -20, 30, 0],
]

URDF_DEFAULT = os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
    "urdf", "mycobot_280_m5.urdf",
)


# The 6 actuated revolute joints, in order J1..J6. ikpy creates one "link" per
# URDF joint (plus a prepended OriginLink and the fixed base joint), so we mark
# active strictly by matching these names rather than trusting auto-inference.
ACTIVE_JOINT_NAMES = [
    "joint2_to_joint1",
    "joint3_to_joint2",
    "joint4_to_joint3",
    "joint5_to_joint4",
    "joint6_to_joint5",
    "joint6output_to_joint6",
]


def build_chain(urdf_path):
    """Build an ikpy Chain from the URDF with a deterministic active-link mask.

    The URDF base joint (g_base_to_joint1) is fixed and ikpy also prepends an
    OriginLink, so the link list is longer than 6. We do NOT trust ikpy's
    auto-inferred active mask (it is fragile for this arm and may drop a joint);
    instead we force exactly the 6 revolute joints active by name and hard-fail
    if that does not yield 6 -- a wrong mask would make the whole FK comparison
    meaningless.
    """
    from ikpy.chain import Chain

    try:
        chain = Chain.from_urdf_file(urdf_path, base_elements=["g_base"])
    except Exception as exc:
        print(f"Chain.from_urdf_file failed: {exc}")
        raise

    names = [link.name for link in chain.links]
    mask = [name in ACTIVE_JOINT_NAMES for name in names]
    n_active = sum(mask)
    if n_active != 6:
        raise RuntimeError(
            f"Expected 6 active joints, got {n_active}.\n"
            f"  link names: {names}\n"
            f"  looked for: {ACTIVE_JOINT_NAMES}"
        )
    chain.active_links_mask = mask
    return chain


def ikpy_fk_coords(chain, angles_deg):
    """Return [x, y, z, rx, ry, rz] (mm / deg) for the given 6 joint angles.

    ikpy works in radians and metres; we convert. The chain has inactive links
    (the fixed base) which need 0 placeholders, so we build the full vector from
    the active-link mask.
    """
    import numpy as np

    mask = chain.active_links_mask
    full = np.zeros(len(chain.links))
    active_idx = [i for i, m in enumerate(mask) if m]
    # build_chain guarantees exactly 6 active links matched by name; no fallback.
    assert len(active_idx) == 6, f"active mask not 6: {mask}"
    for slot, a in zip(active_idx, angles_deg):
        full[slot] = math.radians(a)

    frame = chain.forward_kinematics(full)
    pos_m = frame[:3, 3]
    pos_mm = [float(v) * 1000.0 for v in pos_m]

    # Orientation: convert rotation matrix to XYZ Euler degrees (informational;
    # the firmware's Euler convention may differ, so position is the primary check).
    R = frame[:3, :3]
    sy = math.sqrt(R[0, 0] ** 2 + R[1, 0] ** 2)
    if sy > 1e-6:
        rx = math.atan2(R[2, 1], R[2, 2])
        ry = math.atan2(-R[2, 0], sy)
        rz = math.atan2(R[1, 0], R[0, 0])
    else:
        rx = math.atan2(-R[1, 2], R[1, 1])
        ry = math.atan2(-R[2, 0], sy)
        rz = 0.0
    ori_deg = [math.degrees(v) for v in (rx, ry, rz)]
    return pos_mm + ori_deg


def pos_err(a, b):
    return math.sqrt(sum((x - y) ** 2 for x, y in zip(a[:3], b[:3])))


def fmt(coords):
    return "[" + ", ".join(f"{v:7.2f}" for v in coords) + "]"


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--port", default="/dev/ttyACM0")
    parser.add_argument("--baudrate", type=int, default=115200)
    parser.add_argument("--urdf", default=URDF_DEFAULT)
    parser.add_argument("--no-connect", action="store_true",
                        help="Model-only: do not open the serial port")
    parser.add_argument("--move", action="store_true",
                        help="Command each pose via send_angles and compare live get_coords (ROBOT MOVES)")
    parser.add_argument("--speed", type=int, default=20, help="Motion speed for --move")
    args = parser.parse_args()

    print(f"URDF: {args.urdf}")
    if not os.path.exists(args.urdf):
        print("URDF not found"); return 1

    try:
        chain = build_chain(args.urdf)
    except Exception:
        return 1
    print(f"ikpy chain links ({len(chain.links)}): {[l.name for l in chain.links]}")
    print(f"active mask: {chain.active_links_mask}  (True = one of J1..J6)")
    print("NOTE: ikpy orientation columns are euler-xyz, informational only "
          "(firmware Euler convention differs); position error is the signal.")
    print()

    # ikpy FK for each pose (always available, no hardware needed).
    ik_coords = {tuple(p): ikpy_fk_coords(chain, p) for p in TEST_POSES_DEG}

    if args.no_connect:
        print("== ikpy FK only (no robot) ==")
        for p in TEST_POSES_DEG:
            print(f"  angles {fmt(p)} -> ikpy {fmt(ik_coords[tuple(p)])}")
        return 0

    from pymycobot import MyCobot
    mc = MyCobot(args.port, args.baudrate)
    time.sleep(0.5)

    # --- Ground truth via firmware FORWARD kinematics (no motion) ---
    a2c = getattr(mc, "angles_to_coords", None)
    print("== ikpy FK vs firmware angles_to_coords (offline, no motion) ==")
    print("   (position error in mm is the primary signal)")
    if a2c is None:
        print("  angles_to_coords not available on this firmware/library")
    else:
        for p in TEST_POSES_DEG:
            try:
                fw = a2c(list(p))
            except Exception as e:
                print(f"  angles {fmt(p)} -> angles_to_coords EXC: {e}")
                continue
            ik = ik_coords[tuple(p)]
            if isinstance(fw, (list, tuple)) and len(fw) == 6:
                print(f"  angles {fmt(p)}")
                print(f"      firmware {fmt(fw)}")
                print(f"      ikpy     {fmt(ik)}")
                print(f"      pos_err  {pos_err(ik, fw):6.2f} mm")
            else:
                print(f"  angles {fmt(p)} -> firmware returned {fw!r}")
    print()

    # --- Optional: live motion + get_coords comparison ---
    if args.move:
        ip = getattr(mc, "is_power_on", None)
        if ip is not None:
            try:
                powered = ip()
            except Exception:
                powered = None
            if powered == 0:
                print("Servos are not powered (is_power_on()==0); skipping --move.")
                return 0
        print("== LIVE: send_angles then compare get_coords (ROBOT MOVES) ==")
        for p in TEST_POSES_DEG:
            print(f"  -> moving to {fmt(p)} @ speed {args.speed}")
            mc.send_angles(list(p), args.speed)
            # settle: wait until angles are close or 12s cap
            t0 = time.time()
            settled = False
            while time.time() - t0 < 12:
                a = mc.get_angles()
                if isinstance(a, (list, tuple)) and len(a) == 6:
                    if max(abs(c - t) for c, t in zip(a, p)) <= 2.0:
                        settled = True
                        break
                time.sleep(0.3)
            if not settled:
                print("      WARNING: did not settle within 12s; reading mid-motion "
                      "(pos_err may be unreliable for this pose)")
            time.sleep(0.5)
            live = mc.get_coords()
            ik = ik_coords[tuple(p)]
            if isinstance(live, (list, tuple)) and len(live) == 6:
                print(f"      get_coords {fmt(live)}")
                print(f"      ikpy       {fmt(ik)}")
                print(f"      pos_err    {pos_err(ik, live):6.2f} mm")
            else:
                print(f"      get_coords returned {live!r}")
        print("\n  returning to home [0,0,0,0,0,0]")
        mc.send_angles([0, 0, 0, 0, 0, 0], args.speed)
        time.sleep(2.0)

    return 0


if __name__ == "__main__":
    sys.exit(main())
