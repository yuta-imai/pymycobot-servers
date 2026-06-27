#!/usr/bin/env python3
"""board->base refinement by command-and-observe (camera ground truth), brain-side.

Combines both methods: the existing relax+FK board->base (calib/board_to_base.json)
is the BOOTSTRAP that lets us command the gripper roughly onto the board; this
tool then COMMANDS each desired board point (POST /robot/topdown via the Pi API),
you read where the gripper tip ACTUALLY landed on the board grid (camera truth,
not FK), and it refits board->base from (observed_board -> commanded_base). That
inverts out the corrected_fk non-linear distortion that caps the FK-only fit.

  python calibrate_camera_observe.py --host 192.168.0.136

Per point: the arm moves to the bootstrap command for a target; you read the
tip's true board (x,y) in mm off the 28 mm grid and type it. RANSAC drops bad
points. Saves the refined calib/board_to_base.json (bootstrap is loaded fresh
each run, so re-running keeps improving from the last result).

Needs the Pi API running and an existing calib/board_to_base.json bootstrap.
"""

from __future__ import annotations

import argparse
import json
import urllib.request
from typing import List

import numpy as np

import config
from geometry import apply_homography, apply_plane, fit_plane

NCOLS = config.BOARD.squares_x - 1


def corner_xy(cid):
    sq = config.BOARD.square_length_mm
    return ((cid % NCOLS + 1) * sq, (cid // NCOLS + 1) * sq)


def default_targets() -> List[tuple]:
    """Spread board targets (mm) to command for the refinement."""
    bx = config.BOARD
    xs = [bx.square_length_mm * c for c in (1, (bx.squares_x - 1) // 2, bx.squares_x - 1)]
    ys = [bx.square_length_mm * r for r in (1, (bx.squares_y - 1) // 2, bx.squares_y - 1)]
    return [(x, y) for y in ys for x in xs]   # 3x3 grid over the board


def api_post(base, path, body=None):
    data = json.dumps(body).encode() if body is not None else None
    req = urllib.request.Request(base + path, data=data, method="POST",
                                 headers={"Content-Type": "application/json"})
    try:
        with urllib.request.urlopen(req, timeout=40) as r:
            return r.status, json.loads(r.read() or "{}")
    except urllib.error.HTTPError as e:
        return e.code, json.loads(e.read() or "{}")


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--host", default="192.168.0.136")
    ap.add_argument("--port", type=int, default=8080)
    ap.add_argument("--hover-mm", type=float, default=15.0,
                    help="command z = board plane + this (near surface, small parallax)")
    ap.add_argument("--ransac-mm", type=float, default=8.0)
    ap.add_argument("--targets", default=None,
                    help="board points to command via bootstrap, 'x1,y1;...' (mm)")
    ap.add_argument("--base-targets", default=None,
                    help="command BASE poses DIRECTLY (no bootstrap), 'x,y,z;...' mm; "
                         "use reachable-region poses so every command lands.")
    args = ap.parse_args()
    import cv2
    base_url = f"http://{args.host}:{args.port}"

    b2b = json.load(open(config.artifact_path(config.HANDEYE_JSON)))
    H0 = np.array(b2b["H_board2base"]); zpl0 = np.array(b2b["z_plane"])
    if args.base_targets:
        cmds = [tuple(float(v) for v in p.split(",")) for p in args.base_targets.split(";")]
    elif args.targets:
        tg = [tuple(float(v) for v in p.split(",")) for p in args.targets.split(";")]
        cmds = [(*apply_homography(H0, np.array([D], float))[0],
                 float(apply_plane(zpl0, np.array(D, float))) + args.hover_mm) for D in tg]
    else:
        cmds = [(*apply_homography(H0, np.array([D], float))[0],
                 float(apply_plane(zpl0, np.array(D, float))) + args.hover_mm)
                for D in default_targets()]
    print(f"[refine] commanding {len(cmds)} poses; read each landing off the grid")

    obs_board, cmd_base = [], []
    for k, (cx, cy, cz) in enumerate(cmds):
        print(f"\n[{k+1}/{len(cmds)}] command base=({cx:.1f},{cy:.1f},{cz:.1f})")
        code, resp = api_post(base_url, "/robot/topdown",
                              {"x": float(cx), "y": float(cy), "z": float(cz), "speed": 20})
        if code != 200:
            print(f"   unreachable/skip ({code}: {resp.get('detail','')})"); continue
        ans = input("   read the gripper-tip ACTUAL board x,y (mm) off the grid "
                    "[e.g. 84,56], or s=skip > ").strip().lower()
        if ans == "s" or "," not in ans:
            print("   skipped"); continue
        ox, oy = (float(v) for v in ans.split(","))
        obs_board.append([ox, oy]); cmd_base.append([cx, cy, cz])
        print(f"   recorded observed board=({ox:.0f},{oy:.0f})")

    if len(obs_board) < 4:
        raise SystemExit(f"need >= 4 observations, got {len(obs_board)}")
    O = np.array(obs_board, float); B = np.array(cmd_base, float)
    # G: observed_board -> commanded_base (so for desired T, command G(T))
    G, mask = cv2.findHomography(O, B[:, :2], cv2.RANSAC,
                                 ransacReprojThreshold=args.ransac_mm)
    inl = mask.ravel().astype(bool)
    if inl.sum() < 4:
        inl = np.ones(len(O), bool)
    zpl = fit_plane(O[inl], B[inl, 2]).copy()
    # bootstrap modes added hover to the commanded z; remove it so the saved plane
    # is the board-surface command-z. --base-targets z is used as-is.
    if not args.base_targets:
        zpl[2] -= args.hover_mm
    res = np.linalg.norm(apply_homography(G, O) - B[:, :2], axis=1)
    print(f"\n[fit] camera-observed board->base: {int(inl.sum())}/{len(O)} inliers  "
          f"xy rms={np.sqrt(np.mean(res[inl]**2)):.2f} max={res[inl].max():.2f} mm")

    out = config.artifact_path(config.HANDEYE_JSON)
    json.dump({"model": "homography+plane", "method": "camera_observe",
               "H_board2base": G.tolist(), "z_plane": zpl.tolist(),
               "xy_rms_mm": float(np.sqrt(np.mean(res[inl]**2))),
               "n_points": len(O), "n_inliers": int(inl.sum()),
               "observed_board_mm": O.tolist(), "commanded_base_mm": B.tolist()},
              open(out, "w"), indent=2)
    print(f"wrote {out}")


if __name__ == "__main__":
    main()
