#!/usr/bin/env python3
"""One-command eye-to-hand setup sequence — re-run whenever the ARM or CAMERA moves.

Chains every calibration this stack needs, brain-side, all robot interaction over
the Pi REST API (no serial access needed here). Each stage is idempotent, records
its metrics into calib/setup_state.json, and can be skipped/re-run alone, so the
whole thing is a repeatable tool rather than a one-off ritual:

  [1] preflight  : API /health + /robot/status, camera frame, artifact dir
  [2] wrist      : accelerometer wrist calibration via POST /robot/calibrate_wrist
                   (arm relocated -> gravity model changes), then SYNC the fitted
                   corrected_model.json down to this checkout so brain-side FK/IK
                   agree with the Pi (GET /robot/calibrate_wrist/model)
  [3] park       : send the arm home so the board is unobstructed
  [4] homography : grab an arm-clear frame -> pixel->board homography + empty-board
                   reference for locate + numbered touch-target image
  [5] touch      : INTERACTIVE board->base bootstrap (needed for grasp z): per
                   numbered corner, servos release over REST, you place the closed
                   VERTICAL gripper tip on it, Enter samples joints -> corrected FK
  [6] observe    : AUTOMATED board->base refine: command a grid of board points at
                   hover z, read where the GRIPPER MARKER actually landed (camera
                   truth via the fresh homography), refit board->base
  [7] verify     : command held-out board targets, measure marker-vs-target error
  [8] summary    : all metrics + READY / NOT-READY verdict for the gripper challenge

    python setup_sequence.py --host 192.168.0.136 --url "$SORACAM_STREAM_URL"
    python setup_sequence.py --host ... --only homography     # redo one stage
    python setup_sequence.py --host ... --skip wrist --skip touch   # partial re-run

Frames: --url (RTSP/DASH stream; enables stability-gated marker reads) or the
SORACOM still API (SORACOM_AUTH_KEY_ID/_KEY/_DEVICE_ID env or .soracom.env).

IMPORTANT import order: anything that loads the corrected wrist model (kinematics,
repeatability, topdown_ik) is imported LAZILY inside stages, always AFTER the
wrist stage may have refreshed scripts/wrist_calib/corrected_model.json.
"""

from __future__ import annotations

import argparse
import datetime as _dt
import json
import math
import os
import sys
import time
import urllib.error
import urllib.request
from types import SimpleNamespace
from typing import Any, Dict, List, Optional, Sequence, Tuple

import numpy as np

import config

STAGES = ["preflight", "wrist", "park", "homography", "touch", "observe",
          "verify", "summary"]
STATE_PATH = config.artifact_path("setup_state.json")
MODEL_PATH = os.path.join(config.ROOT, "scripts", "wrist_calib", "corrected_model.json")


# --------------------------------------------------------------------------- #
# state + logging
# --------------------------------------------------------------------------- #
def _now() -> str:
    return _dt.datetime.now().astimezone().isoformat(timespec="seconds")


def load_state() -> Dict[str, Any]:
    if os.path.exists(STATE_PATH):
        with open(STATE_PATH) as f:
            return json.load(f)
    return {"stages": {}}


def save_stage(state: Dict[str, Any], stage: str, result: Dict[str, Any]) -> None:
    state["stages"][stage] = {"at": _now(), **result}
    config.ensure_artifact_dir()
    with open(STATE_PATH, "w") as f:
        json.dump(state, f, indent=2)


def banner(title: str) -> None:
    print(f"\n{'=' * 66}\n=== {title}\n{'=' * 66}")


# --------------------------------------------------------------------------- #
# REST client (thin Pi API)
# --------------------------------------------------------------------------- #
class Api:
    def __init__(self, host: str, port: int, timeout_s: float = 20.0):
        self.base = f"http://{host}:{port}"
        self.timeout_s = timeout_s

    def req(self, method: str, path: str, body: Optional[dict] = None,
            timeout: Optional[float] = None) -> dict:
        data = json.dumps(body).encode() if body is not None else None
        r = urllib.request.Request(self.base + path, data=data, method=method,
                                   headers={"Content-Type": "application/json"})
        try:
            with urllib.request.urlopen(r, timeout=timeout or self.timeout_s) as resp:
                return json.loads(resp.read().decode() or "{}")
        except urllib.error.HTTPError as e:
            try:
                detail = json.loads(e.read().decode() or "{}")
            except Exception:
                detail = {"detail": str(e)}
            raise RuntimeError(f"{method} {path} -> HTTP {e.code}: {detail}") from e

    # movement
    def home(self, speed: int = 30) -> dict:
        return self.req("POST", "/robot/home", {"speed": speed}, timeout=60)

    def topdown(self, x: float, y: float, z: float, speed: int) -> dict:
        return self.req("POST", "/robot/topdown",
                        {"x": float(x), "y": float(y), "z": float(z),
                         "speed": int(speed)}, timeout=60)

    def wait(self, timeout_s: float = 20.0, tol_deg: float = 1.0) -> dict:
        return self.req("POST", "/robot/wait",
                        {"timeout": timeout_s, "tolerance": tol_deg},
                        timeout=timeout_s + 15)

    # state
    def health(self) -> dict:
        return self.req("GET", "/health")

    def status(self) -> dict:
        return self.req("GET", "/robot/status")

    def angles(self) -> List[float]:
        return [float(a) for a in self.req("GET", "/joints/angles")["angles"]]

    # servo/gripper for the touch stage
    def release_servos(self) -> dict:
        return self.req("POST", "/robot/release")

    def power_on(self) -> dict:
        return self.req("POST", "/robot/power_on")

    def close_gripper(self) -> dict:
        return self.req("POST", "/gripper/close", {"speed": 50, "gripper_type": 3})

    # wrist calibration job
    def wrist_start(self, body: dict) -> dict:
        return self.req("POST", "/robot/calibrate_wrist", body)

    def wrist_status(self) -> dict:
        return self.req("GET", "/robot/calibrate_wrist/status")

    def wrist_model(self) -> dict:
        return self.req("GET", "/robot/calibrate_wrist/model")


# --------------------------------------------------------------------------- #
# camera
# --------------------------------------------------------------------------- #
def grab_frame(args) -> "np.ndarray":
    from frame_source import grab
    return grab(backend=args.backend, url=args.url)


def resolve_stream_url(args, force: bool = False) -> Optional[str]:
    """Ensure args.url holds a live stream URL when possible.

    Explicit --url wins; otherwise, with SORACOM credentials in the environment
    (.soracom.env is auto-loaded by config), fetch a live-view URL from the API.
    Live URLs are short-lived, so this is also called to REFRESH after a failure.
    """
    if args.url and not force:
        return args.url
    if os.environ.get("SORACOM_AUTH_KEY_ID") and os.environ.get("SORACOM_DEVICE_ID"):
        from frame_source import get_soracom_live_url
        try:
            args.url = get_soracom_live_url()
            print(f"  [stream] live URL fetched from SORACOM API")
            return args.url
        except RuntimeError as e:
            print(f"  [stream] auto-fetch failed ({e}); falling back to stills")
            args.url = None
    return args.url


def marker_read(args, H: np.ndarray, pre_xy: Optional[Sequence[float]]) -> Dict[str, Any]:
    """Trustworthy gripper-marker board-XY after a move.

    With a stream URL: reuse repeatability.measure_stable (movement + distinct-frame
    + stability gates — defeats stale/duplicate frames); on a dead/expired stream
    the URL is re-fetched once from credentials. With the still backend: settle
    sleep + single still (each still is freshly captured server-side).
    """
    if args.url:
        import repeatability as rp  # lazy: loads kinematics AFTER wrist sync
        ns = SimpleNamespace(stream_drain=2, stable_frames=4, stable_tol_mm=1.0,
                             stable_min_window_s=0.8, stable_timeout_s=10.0,
                             read_interval_s=0.15, move_min_mm=3.0)
        try:
            return rp.measure_stable(args.url, H, ns, pre_xy=pre_xy)
        except RuntimeError:
            if resolve_stream_url(args, force=True):
                return rp.measure_stable(args.url, H, ns, pre_xy=pre_xy)
    if args.backend == "rtsp" and not args.url:
        return {"ok": False, "stable": False, "reason": "no_stream_url"}
    from gripper_marker import marker_board_xy
    time.sleep(args.settle_s)
    frame = grab_frame(args)
    m = marker_board_xy(frame, H)
    if m is None:
        return {"ok": False, "stable": False, "reason": "marker_not_detected"}
    return {"ok": True, "stable": True, "xy_mm": [float(m[0]), float(m[1])]}


def load_H(path_key: str = config.HOMOGRAPHY_JSON) -> np.ndarray:
    with open(config.artifact_path(path_key)) as f:
        return np.asarray(json.load(f)["H_img2board"], float)


# --------------------------------------------------------------------------- #
# stages
# --------------------------------------------------------------------------- #
def stage_preflight(api: Api, args, state) -> Dict[str, Any]:
    banner("[1] PREFLIGHT")
    h = api.health()
    st = api.status()
    angles = st.get("joint_angles")
    print(f"  API health: {h.get('status', h)}   joints: {angles}")
    if not angles or len(angles) != 6:
        raise SystemExit("robot status did not return 6 joint angles")
    frame = grab_frame(args)
    print(f"  camera frame: {frame.shape[1]}x{frame.shape[0]}")
    config.ensure_artifact_dir()
    return {"ok": True, "frame": [frame.shape[1], frame.shape[0]],
            "backend": args.backend or config.FRAME_SOURCE.backend}


def stage_wrist(api: Api, args, state) -> Dict[str, Any]:
    banner("[2] WRIST CALIBRATION (accelerometer, via API)")
    print("  The arm will sweep a pose set; keep the area clear. This takes minutes.")
    body = {"pose_set": args.wrist_pose_set, "speed": args.speed}
    api.wrist_start(body)
    t0 = time.monotonic()
    while True:
        s = api.wrist_status()
        prog = s.get("progress") or {}
        print(f"\r  {s.get('status')}/{s.get('phase')}  "
              f"{prog.get('done', 0)}/{prog.get('total', 0)}   ", end="", flush=True)
        if s.get("status") in ("done", "error"):
            print()
            break
        if time.monotonic() - t0 > args.wrist_timeout_s:
            raise SystemExit("wrist calibration timed out")
        time.sleep(3.0)
    if s.get("status") == "error":
        raise SystemExit(f"wrist calibration failed: {s.get('error')}")
    result = s.get("result") or {}
    print(f"  orient_err_deg={result.get('orient_err_deg')}  "
          f"clean poses={result.get('n_clean')}/{result.get('poses_recorded')}")

    # sync the fitted model down so brain-side FK/IK match the Pi
    try:
        model = api.wrist_model()
        if os.path.exists(MODEL_PATH):
            os.replace(MODEL_PATH, MODEL_PATH + ".prev")
        with open(MODEL_PATH, "w") as f:
            json.dump(model, f, indent=2)
        print(f"  synced model -> {MODEL_PATH}")
        synced = True
    except RuntimeError as e:
        print(f"  WARN: could not fetch model over API ({e}).\n"
              f"        Pi needs this branch + restart; or scp the Pi's "
              f"scripts/wrist_calib/corrected_model.json here manually.")
        synced = False
    err = result.get("orient_err_deg")
    if err is not None and err > 3.0:
        print(f"  WARN: orientation error {err:.2f} deg is high — consider re-running")
    return {"ok": True, "orient_err_deg": err, "model_synced": synced, **result}


def stage_park(api: Api, args, state) -> Dict[str, Any]:
    banner("[3] PARK (clear the board)")
    api.home(speed=args.speed)
    w = api.wait(timeout_s=30.0)
    print(f"  home: completed={w.get('completed')} reason={w.get('reason')}")
    if not w.get("completed"):
        raise SystemExit("could not park the arm (home did not converge) — fix before "
                         "grabbing the board frame")
    return {"ok": True, "wait": w}


def stage_homography(api: Api, args, state) -> Dict[str, Any]:
    banner("[4] HOMOGRAPHY + empty-board reference")
    import cv2
    import homography_calibrate as hc
    from charuco_board import build_board, make_detector, chessboard_corners_mm
    from setup_calibration import _select_spread

    print("  board must be FULLY visible and the arm clear (parked).")
    if not args.yes:
        input("  press Enter to grab the board frame...")
    frame = grab_frame(args)
    res = hc.fit_from_frame(frame)
    config.ensure_artifact_dir()
    with open(config.artifact_path(config.HOMOGRAPHY_JSON), "w") as f:
        json.dump(res, f, indent=2)
    cv2.imwrite(config.artifact_path("empty_board_ref.png"), frame)
    r = res["loo_residual_mm"]
    print(f"  corners={res['n_corners']}/{config.BOARD.n_corners}  "
          f"LOO mean={r['mean']:.2f} p95={r['p95']:.2f} max={r['max']:.2f} mm")
    if r["mean"] > 2.0:
        print("  WARN: residual > 2mm — recenter board / improve light and re-run")

    # numbered touch targets for stage 5
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    _, board = build_board()
    cc, ci = make_detector(board)(gray)
    ids = ci.flatten()
    sel = _select_spread(chessboard_corners_mm(board), ids, args.n_touch)
    px = cc.reshape(-1, 2)
    idpos = {int(v): i for i, v in enumerate(ids)}
    vis = frame.copy()
    for k, cid in enumerate(sel, 1):
        p = tuple(np.round(px[idpos[cid]]).astype(int))
        cv2.circle(vis, p, 20, (0, 0, 255), 4)
        cv2.putText(vis, str(k), (p[0] + 18, p[1] - 12),
                    cv2.FONT_HERSHEY_SIMPLEX, 1.4, (0, 0, 255), 4)
    cv2.imwrite(config.artifact_path("touch_targets.png"), vis)
    print(f"  touch targets ids={sel} -> touch_targets.png")
    return {"ok": True, "n_corners": res["n_corners"], "loo_mean_mm": r["mean"],
            "loo_p95_mm": r["p95"], "touch_ids": sel}


def stage_touch(api: Api, args, state) -> Dict[str, Any]:
    banner("[5] BOARD->BASE touch bootstrap (interactive, over REST)")
    import cv2
    from charuco_board import build_board, chessboard_corners_mm
    from geometry import apply_homography, apply_plane, fit_plane
    import kinematics  # lazy: AFTER wrist sync

    touch_ids = (state["stages"].get("homography", {}) or {}).get("touch_ids")
    if args.corner_ids:
        touch_ids = [int(x) for x in args.corner_ids.split(",")]
    if not touch_ids:
        raise SystemExit("no touch ids — run the homography stage first")
    _, board = build_board()
    corners = chessboard_corners_mm(board)

    print(f"  {len(touch_ids)} numbered corners (see calib/touch_targets.png).")
    print("  gripper will CLOSE; keep it VERTICAL (straight down) at every touch.")
    try:
        api.close_gripper()
    except RuntimeError as e:
        print(f"  WARN close_gripper: {e} — close it by hand")

    src_board, dst_base, used = [], [], []
    try:
        for k, cid in enumerate(touch_ids):
            bx, by, _ = corners[cid]
            print(f"\n  [{k + 1}/{len(touch_ids)}] corner id={cid} board=({bx:.0f},{by:.0f})mm")
            api.release_servos()
            ans = input("    place tip on the corner; Enter=sample s=skip q=abort > ").strip().lower()
            if ans == "q":
                break
            if ans == "s":
                continue
            angles = api.angles()
            tip = kinematics.corrected_fk_pos(angles)
            down = kinematics.downness(angles)
            flag = "" if down >= 0.9 else "  <-- NOT vertical, redo!"
            print(f"    tip={np.round(tip, 1).tolist()}  downness={down:+.2f}{flag}")
            src_board.append([bx, by])
            dst_base.append(tip.tolist())
            used.append(cid)
    finally:
        try:
            api.power_on()
        except Exception:
            pass

    if len(src_board) < 4:
        raise SystemExit(f"need >= 4 touch points, got {len(src_board)}")
    S = np.asarray(src_board, float)
    D = np.asarray(dst_base, float)
    H, mask = cv2.findHomography(S, D[:, :2], cv2.RANSAC,
                                 ransacReprojThreshold=args.ransac_mm)
    if H is None:
        raise SystemExit("board->base homography fit failed")
    inl = mask.ravel().astype(bool)
    if inl.sum() < 4:
        inl = np.ones(len(S), bool)
    zpl = fit_plane(S[inl], D[inl, 2])
    xy_res = np.linalg.norm(apply_homography(H, S) - D[:, :2], axis=1)
    z_res = np.abs(apply_plane(zpl, S) - D[:, 2])
    xy_rms = float(np.sqrt(np.mean(xy_res[inl] ** 2)))
    z_rms = float(np.sqrt(np.mean(z_res[inl] ** 2)))
    print(f"\n  fit: {int(inl.sum())}/{len(S)} inliers  xy rms={xy_rms:.2f}mm  "
          f"z rms={z_rms:.2f}mm")
    with open(config.artifact_path(config.HANDEYE_JSON), "w") as f:
        json.dump({"model": "homography+plane", "method": "touch_rest",
                   "H_board2base": H.tolist(), "z_plane": zpl.tolist(),
                   "xy_rms_mm": xy_rms, "xy_max_mm": float(xy_res[inl].max()),
                   "z_rms_mm": z_rms, "n_points": len(S),
                   "n_inliers": int(inl.sum()), "corner_ids": used,
                   "src_board_mm": S.tolist(), "dst_base_mm": D.tolist()}, f, indent=2)
    print(f"  wrote {config.HANDEYE_JSON}")
    return {"ok": True, "xy_rms_mm": xy_rms, "z_rms_mm": z_rms,
            "n_inliers": int(inl.sum()), "n_points": len(S)}


def _grid_targets(nx: int = 3, ny: int = 3) -> List[Tuple[float, float]]:
    b = config.BOARD
    xs = np.linspace(b.square_length_mm, b.width_mm - b.square_length_mm, nx)
    ys = np.linspace(b.square_length_mm, b.height_mm - b.square_length_mm, ny)
    return [(float(x), float(y)) for y in ys for x in xs]


def _observe_points(api: Api, args, H_img2board: np.ndarray, b2b: dict,
                    targets: Sequence[Tuple[float, float]]) -> Tuple[List, List, List]:
    """Command each board target at hover z via the given map; read the marker."""
    from geometry import apply_homography, apply_plane
    H0 = np.asarray(b2b["H_board2base"], float)
    zpl = np.asarray(b2b["z_plane"], float)
    obs, cmd, errs = [], [], []
    last_xy = None
    for k, T in enumerate(targets):
        bx = apply_homography(H0, np.asarray([T], float))[0]
        bz = float(apply_plane(zpl, np.asarray(T, float))) + args.hover_mm
        print(f"  [{k + 1}/{len(targets)}] board({T[0]:.0f},{T[1]:.0f}) -> "
              f"base({bx[0]:.0f},{bx[1]:.0f},{bz:.0f})", end="  ")
        try:
            api.topdown(bx[0], bx[1], bz, args.speed)
            w = api.wait(timeout_s=25.0)
        except RuntimeError as e:
            print(f"unreachable/skip ({e})")
            continue
        if not w.get("completed"):
            print(f"move did not converge ({w.get('reason')}) — skip")
            continue
        m = marker_read(args, H_img2board, pre_xy=last_xy)
        if not m.get("ok") or not m.get("stable"):
            print(f"marker read failed ({m.get('reason')}) — skip")
            continue
        last_xy = m["xy_mm"]
        e = float(np.hypot(m["xy_mm"][0] - T[0], m["xy_mm"][1] - T[1]))
        print(f"marker=({m['xy_mm'][0]:.1f},{m['xy_mm'][1]:.1f})  err={e:.1f}mm")
        obs.append(m["xy_mm"])
        cmd.append([float(bx[0]), float(bx[1]), bz])
        errs.append(e)
    return obs, cmd, errs


def stage_observe(api: Api, args, state) -> Dict[str, Any]:
    banner("[6] BOARD->BASE camera-observe refine (automated, marker truth)")
    import cv2
    from geometry import fit_plane, apply_homography
    from touch_calibrate import load_board_to_base

    H_img2board = load_H()
    b2b = load_board_to_base(config.artifact_path(config.HANDEYE_JSON))
    targets = ([tuple(float(v) for v in p.split(",")) for p in args.targets.split(";")]
               if args.targets else _grid_targets())
    obs, cmd, _ = _observe_points(api, args, H_img2board, b2b, targets)
    if len(obs) < 4:
        raise SystemExit(f"need >= 4 marker observations, got {len(obs)}")

    O = np.asarray(obs, float)
    B = np.asarray(cmd, float)
    G, mask = cv2.findHomography(O, B[:, :2], cv2.RANSAC,
                                 ransacReprojThreshold=args.ransac_mm)
    inl = mask.ravel().astype(bool)
    if inl.sum() < 4:
        inl = np.ones(len(O), bool)
    zpl = fit_plane(O[inl], B[inl, 2]).copy()
    zpl[2] -= args.hover_mm       # commanded z included hover; store surface z
    res = np.linalg.norm(apply_homography(G, O) - B[:, :2], axis=1)
    xy_rms = float(np.sqrt(np.mean(res[inl] ** 2)))
    print(f"\n  refine: {int(inl.sum())}/{len(O)} inliers  xy rms={xy_rms:.2f} "
          f"max={float(res[inl].max()):.2f} mm")
    with open(config.artifact_path(config.HANDEYE_JSON), "w") as f:
        json.dump({"model": "homography+plane", "method": "camera_observe_marker",
                   "H_board2base": G.tolist(), "z_plane": zpl.tolist(),
                   "xy_rms_mm": xy_rms, "xy_max_mm": float(res[inl].max()),
                   "n_points": len(O), "n_inliers": int(inl.sum()),
                   "observed_board_mm": O.tolist(),
                   "commanded_base_mm": B.tolist()}, f, indent=2)
    print(f"  wrote {config.HANDEYE_JSON} (method=camera_observe_marker)")
    api.home(speed=args.speed)
    return {"ok": True, "xy_rms_mm": xy_rms, "n_inliers": int(inl.sum()),
            "n_points": len(O)}


def stage_verify(api: Api, args, state) -> Dict[str, Any]:
    banner("[7] VERIFY (held-out board targets, closed map)")
    from touch_calibrate import load_board_to_base
    H_img2board = load_H()
    b2b = load_board_to_base(config.artifact_path(config.HANDEYE_JSON))
    b = config.BOARD
    targets = ([tuple(float(v) for v in p.split(",")) for p in args.verify_targets.split(";")]
               if args.verify_targets else
               [(b.width_mm * 0.35, b.height_mm * 0.65),
                (b.width_mm * 0.65, b.height_mm * 0.35)])
    _, _, errs = _observe_points(api, args, H_img2board, b2b, targets)
    api.home(speed=args.speed)
    if not errs:
        raise SystemExit("verification produced no measurements")
    mean_e, max_e = float(np.mean(errs)), float(np.max(errs))
    verdict = "PASS" if max_e <= args.verify_tol_mm else "FAIL"
    print(f"\n  open-loop marker error: mean={mean_e:.1f} max={max_e:.1f} mm "
          f"(gate {args.verify_tol_mm}mm) -> {verdict}")
    if verdict == "FAIL":
        print("  NOTE: visual servo can still null the residual in-loop; "
              "this gates OPEN-LOOP quality only.")
    return {"ok": True, "mean_err_mm": mean_e, "max_err_mm": max_e,
            "n": len(errs), "verdict": verdict}


def stage_summary(api: Api, args, state) -> Dict[str, Any]:
    banner("[8] SUMMARY")
    st = state["stages"]
    rows = [
        ("wrist   ", st.get("wrist", {}), "orient_err_deg", 3.0, "deg", False),
        ("homogr. ", st.get("homography", {}), "loo_mean_mm", 2.0, "mm", False),
        ("touch   ", st.get("touch", {}), "xy_rms_mm", 4.0, "mm", False),
        ("observe ", st.get("observe", {}), "xy_rms_mm", 5.0, "mm", False),
        ("verify  ", st.get("verify", {}), "max_err_mm", args.verify_tol_mm, "mm", True),
    ]
    ready = True
    for name, s, key, gate, unit, required in rows:
        v = s.get(key)
        if v is None:
            print(f"  {name}: -- not run --" + ("  (REQUIRED)" if required else ""))
            if required:
                ready = False
            continue
        ok = v <= gate
        ready = ready and (ok or not required)
        print(f"  {name}: {key}={v:.2f}{unit}  gate<={gate}{unit}  "
              f"{'OK' if ok else 'OVER'}  ({s.get('at', '')})")
    print(f"\n  {'READY for the gripper challenge' if ready else 'NOT READY — redo the failing stage'}")
    print("  next: python visual_servo.py init <board_x> <board_y>   (then step ...)")
    return {"ok": True, "ready": ready}


STAGE_FN = {"preflight": stage_preflight, "wrist": stage_wrist, "park": stage_park,
            "homography": stage_homography, "touch": stage_touch,
            "observe": stage_observe, "verify": stage_verify, "summary": stage_summary}


# --------------------------------------------------------------------------- #
def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--host", required=True, help="Pi REST API host")
    ap.add_argument("--port", type=int, default=8080)
    ap.add_argument("--backend", default=None, choices=[None, "soracom", "rtsp", "mcp"])
    ap.add_argument("--url", default=None, help="RTSP/DASH stream URL (enables "
                    "stability-gated marker reads; else soracom stills)")
    ap.add_argument("--only", choices=STAGES, default=None)
    ap.add_argument("--from", dest="from_stage", choices=STAGES, default=None)
    ap.add_argument("--skip", action="append", choices=STAGES, default=[])
    ap.add_argument("--yes", action="store_true", help="no confirmation prompts")
    ap.add_argument("--speed", type=int, default=20)
    ap.add_argument("--settle-s", type=float, default=2.0,
                    help="still-backend settle before a marker read")
    # wrist
    ap.add_argument("--wrist-pose-set", default="default",
                    choices=["default", "quick"])
    ap.add_argument("--wrist-timeout-s", type=float, default=1800.0)
    # homography/touch
    ap.add_argument("--n-touch", type=int, default=6)
    ap.add_argument("--corner-ids", default=None)
    ap.add_argument("--ransac-mm", type=float, default=8.0)
    # observe/verify
    ap.add_argument("--hover-mm", type=float, default=40.0,
                    help="marker-observe height above the touched board plane")
    ap.add_argument("--targets", default=None, help="observe board targets 'x,y;...'")
    ap.add_argument("--verify-targets", default=None)
    ap.add_argument("--verify-tol-mm", type=float, default=8.0,
                    help="open-loop gate; the servo loop tightens this in-loop")
    args = ap.parse_args()

    if args.only:
        run = [args.only]
    else:
        run = STAGES[STAGES.index(args.from_stage):] if args.from_stage else list(STAGES)
        run = [s for s in run if s not in args.skip]

    api = Api(args.host, args.port)
    state = load_state()
    resolve_stream_url(args)
    if not args.url and not args.backend and os.environ.get("SORACOM_AUTH_KEY_ID"):
        args.backend = "soracom"          # stills fallback from credentials
    print(f"setup sequence: {' -> '.join(run)}   "
          f"frames={'stream' if args.url else (args.backend or config.FRAME_SOURCE.backend)}")
    for stage in run:
        try:
            result = STAGE_FN[stage](api, args, state)
        except KeyboardInterrupt:
            print(f"\naborted during '{stage}' — re-run with --from {stage}")
            raise SystemExit(130)
        save_stage(state, stage, result)


if __name__ == "__main__":
    main()
