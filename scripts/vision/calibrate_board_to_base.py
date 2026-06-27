#!/usr/bin/env python3
"""board->base calibration, brain-side, driven over the REST API (relax mode).

Architecture: the Pi is a thin executor (REST API); the brain issues the commands
and does all the computation. This tool:
  - puts the arm in CALIBRATION/RELAX mode via the API (POST /robot/release),
  - for each known board corner you hand-guide the closed, VERTICAL gripper onto
    it (use the coordinate-grid image), press Enter,
  - reads raw joint angles (GET /joints/angles) and computes the gripper-tip base
    position HERE with kinematics.corrected_fk_pos (no robot code on the brain
    beyond pure-numpy FK),
  - fits board(X,Y)->base(x,y) as a homography + base z as a plane (RANSAC drops
    mis-touches), and saves calib/board_to_base.json.

The Pi must be running mycobot_api_server.py. Robot primitives used (all already
in the API/MCP): /gripper/close, /robot/release, /joints/angles, /robot/power_on.

    python calibrate_board_to_base.py --host 192.168.0.136 --corner-ids 0,8,53,45,4,49,22,18,31
"""

from __future__ import annotations

import argparse
import json
import urllib.request
from typing import List

import numpy as np

import config
from geometry import apply_homography, apply_plane, fit_plane
from kinematics import corrected_fk_pos, downness

NCOLS = config.BOARD.squares_x - 1   # interior corner columns (9 for a 10-wide board)


def corner_xy(cid: int):
    """ChArUco interior-corner id -> board (x,y) mm (col-major within a row)."""
    sq = config.BOARD.square_length_mm
    col, row = cid % NCOLS, cid // NCOLS
    return ((col + 1) * sq, (row + 1) * sq)


def recommend_ids(n: int) -> List[int]:
    """A spread set: 4 corners, center, 4 edge midpoints (then fill)."""
    cmax = NCOLS - 1
    rmax = (config.BOARD.squares_y - 1) - 1
    picks = [(0, 0), (cmax, 0), (cmax, rmax), (0, rmax), (cmax // 2, rmax // 2),
             (cmax // 2, 0), (cmax, rmax // 2), (cmax // 2, rmax), (0, rmax // 2)]
    ids = [r * NCOLS + c for c, r in picks]
    return ids[:n]


class Api:
    def __init__(self, base): self.base = base.rstrip("/")

    def _req(self, path, method="GET", body=None):
        data = json.dumps(body).encode() if body is not None else None
        req = urllib.request.Request(self.base + path, data=data, method=method,
                                     headers={"Content-Type": "application/json"})
        with urllib.request.urlopen(req, timeout=30) as r:
            return json.loads(r.read() or "{}")

    def release(self): return self._req("/robot/release", "POST")
    def power_on(self): return self._req("/robot/power_on", "POST")
    def close_gripper(self): return self._req("/gripper/close", "POST", {"gripper_type": 3})
    def angles(self):
        d = self._req("/joints/angles")
        return d.get("joint_angles") or d.get("angles")


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--host", default="192.168.0.136")
    ap.add_argument("--port", type=int, default=8080)
    ap.add_argument("--corner-ids", default=None, help="comma-separated, in order")
    ap.add_argument("--n", type=int, default=9)
    ap.add_argument("--ransac-mm", type=float, default=8.0)
    args = ap.parse_args()

    import cv2
    api = Api(f"http://{args.host}:{args.port}")
    ids = [int(x) for x in args.corner_ids.split(",")] if args.corner_ids \
        else recommend_ids(args.n)
    print(f"[calib] board->base over {len(ids)} corners (ids={ids})")
    print("[calib] closing gripper for a repeatable contact point...")
    try:
        api.close_gripper()
    except Exception as e:
        print(f"  WARN close_gripper: {e}")

    src, dst, used = [], [], []
    try:
        for k, cid in enumerate(ids):
            bx, by = corner_xy(cid)
            print(f"\n[{k+1}/{len(ids)}] corner id={cid}  board=({bx:.0f},{by:.0f}) mm")
            print("  RELAX: hand-guide the closed gripper VERTICAL onto that exact "
                  "grid intersection; hold steady.")
            api.release()
            ans = input("  Enter=sample / s=skip / q=quit > ").strip().lower()
            if ans == "q":
                break
            if ans == "s":
                print("  skipped"); continue
            ang = api.angles()
            tip = corrected_fk_pos(ang)
            dn = downness(ang)
            flag = "" if dn >= 0.9 else "  <-- NOT vertical, redo more upright"
            print(f"    angles={[round(a,1) for a in ang]}")
            print(f"    tip(base)={tip.round(1).tolist()} downness={dn:+.2f}{flag}")
            src.append([bx, by]); dst.append(tip.tolist()); used.append(cid)
    finally:
        try:
            api.power_on()
        except Exception:
            pass

    if len(src) < 4:
        raise SystemExit(f"need >= 4 samples, got {len(src)}")
    src = np.array(src, float); dst = np.array(dst, float)
    H, mask = cv2.findHomography(src, dst[:, :2], cv2.RANSAC,
                                 ransacReprojThreshold=args.ransac_mm)
    inl = mask.ravel().astype(bool)
    if inl.sum() < 4:
        inl = np.ones(len(src), bool)
    zpl = fit_plane(src[inl], dst[inl, 2])
    xy_res = np.linalg.norm(apply_homography(H, src) - dst[:, :2], axis=1)
    z_res = np.abs(apply_plane(zpl, src) - dst[:, 2])
    print(f"\n[fit] {int(inl.sum())}/{len(src)} inliers  "
          f"xy rms={np.sqrt(np.mean(xy_res[inl]**2)):.2f} max={xy_res[inl].max():.2f} mm  "
          f"z rms={np.sqrt(np.mean(z_res[inl]**2)):.2f} mm")
    for cid, e, ok in zip(used, xy_res, inl):
        print(f"   corner {cid}: xy {e:.2f} mm" + ("" if ok else "  <-- OUTLIER"))

    config.ensure_artifact_dir()
    out = config.artifact_path(config.HANDEYE_JSON)
    with open(out, "w") as f:
        json.dump({"model": "homography+plane", "H_board2base": H.tolist(),
                   "z_plane": zpl.tolist(),
                   "xy_rms_mm": float(np.sqrt(np.mean(xy_res[inl]**2))),
                   "xy_max_mm": float(xy_res[inl].max()),
                   "z_rms_mm": float(np.sqrt(np.mean(z_res[inl]**2))),
                   "n_points": len(src), "n_inliers": int(inl.sum()),
                   "inliers": inl.tolist(), "corner_ids": used,
                   "src_board_mm": src.tolist(), "dst_base_mm": dst.tolist()},
                  f, indent=2)
    print(f"wrote {out}")


if __name__ == "__main__":
    main()
