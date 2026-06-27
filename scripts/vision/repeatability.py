#!/usr/bin/env python3
"""Measure COMMAND REPEATABILITY of the gripper marker in board coordinates.

This is the prerequisite measurement for the fixed-height marker-convergence
experiment. Before building a visual-servo loop, it answers the load-bearing
question the convergence harness cannot:

    If the robot is commanded to the SAME pose many times, how much does the
    camera-observed gripper-marker land-point scatter, in board millimetres?

That scatter is the hard physical floor: no servo law can converge below it. If
the repeatability p95 is well under the convergence tolerance, closed-loop
servoing is worth building; if it is comparable to or larger than the tolerance,
the bottleneck is the robot/camera, not the controller, and that must be fixed
first (unidirectional approach / backlash preload / eye-in-hand) before sinking
effort into the convergence harness.

It deliberately does NOT close the gripper, descend in Z, or attempt any
correction. It holds a fixed top-down ``servo_z`` and only commands XY through the
brain-side reachable-branch IK, exactly like the convergence experiment will.

Two things are measured:

  * static jitter  -- the arm does not move; distinct fresh frames are taken and
    the marker scatter reported. This is the SENSOR/detection floor.
  * repeatability  -- per target, N cycles of: depart to an ``away`` pose, return
    to the target pose, wait for joint convergence, then poll the live stream
    until the observed marker board XY is STABLE. Stability is only accepted AFTER
    the marker is seen to MOVE away from the previous pose (so a stale buffered
    frame of the departure pose can never be mistaken for the settled target), and
    only across DISTINCT captured frames spanning a real wall-clock window (so a
    frozen/duplicated stream frame can never read as 'stable'). Only stable
    measurements feed the statistic; timeouts are excluded and surfaced.

By default each cycle approaches the target from the SAME ``away`` direction, so
the result is a UNIDIRECTIONAL repeatability (ISO-9283 style). Pass
``--alternate-away`` to flip the approach each cycle for a multidirectional /
backlash-inclusive figure (closer to what an arbitrary servo law will face).

Example:

    cd scripts/vision
    python repeatability.py \
        --host 192.168.0.136 --port 8080 \
        --url "$SORACAM_STREAM_URL" \
        --servo-z 150 --away-mm 30,0 --cycles 12 --speed 15 \
        --tol-mm 3 --log calib/repeatability.jsonl
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
from dataclasses import dataclass
from typing import Any, Dict, List, Optional, Sequence, Tuple

import numpy as np

import config
from frame_source import StreamReader
from gripper_marker import marker_board_xy
from homography_calibrate import load_homography
from kinematics import corrected_fk_pos
from topdown_ik import solve_reachable

Array2 = Tuple[float, float]


# --------------------------------------------------------------------------- #
# small helpers
# --------------------------------------------------------------------------- #
def _now_id() -> str:
    return _dt.datetime.utcnow().strftime("%Y%m%dT%H%M%SZ")


def parse_xy(text: str) -> Array2:
    vals = [float(v.strip()) for v in text.split(",")]
    if len(vals) != 2:
        raise argparse.ArgumentTypeError("expected X,Y")
    return vals[0], vals[1]


def parse_points(text: Optional[str]) -> List[Array2]:
    if not text:
        return []
    return [parse_xy(part) for part in text.split(";") if part.strip()]


def _json_safe(v: Any) -> Any:
    if isinstance(v, np.ndarray):
        return _json_safe(v.tolist())
    if isinstance(v, (np.floating, float)):
        x = float(v)
        return x if math.isfinite(x) else None
    if isinstance(v, (np.integer, int)):
        return int(v)
    if isinstance(v, dict):
        return {str(k): _json_safe(val) for k, val in v.items()}
    if isinstance(v, (list, tuple)):
        return [_json_safe(x) for x in v]
    return v


class JsonlLogger:
    def __init__(self, path: Optional[str]):
        self.path = path
        self.fh = None
        if path:
            parent = os.path.dirname(path)
            if parent:
                os.makedirs(parent, exist_ok=True)
            self.fh = open(path, "a", encoding="utf-8")

    def write(self, event: Dict[str, Any]) -> None:
        event = dict(event)
        event.setdefault("ts", _dt.datetime.utcnow().isoformat() + "Z")
        line = json.dumps(_json_safe(event), sort_keys=True)
        print(line)
        if self.fh:
            self.fh.write(line + "\n")
            self.fh.flush()

    def close(self) -> None:
        if self.fh:
            self.fh.close()
            self.fh = None

    def __enter__(self) -> "JsonlLogger":
        return self

    def __exit__(self, *_exc: object) -> None:
        self.close()


# Minimum sample size below which a p95 tail estimate is meaningless.
MIN_N_FOR_P95 = 5


def spread_stats(points: Sequence[Sequence[float]]) -> Dict[str, Any]:
    """Robust scatter of 2D points about their median (mm).

    p95 is reported as None below MIN_N_FOR_P95 samples (a tail percentile from a
    handful of points is not trustworthy and an n=1 'p95=0' would falsely look
    perfect).
    """
    pts = [p for p in points if p is not None]
    if not pts:
        return {"n": 0, "center_xy_mm": None, "rms_mm": None, "p95_mm": None, "max_mm": None}
    P = np.asarray(pts, float)
    center = np.median(P, axis=0)
    d = np.linalg.norm(P - center, axis=1)
    n = int(len(P))
    return {
        "n": n,
        "center_xy_mm": [float(center[0]), float(center[1])],
        "rms_mm": float(np.sqrt(np.mean(d ** 2))),
        "p95_mm": float(np.percentile(d, 95)) if n >= MIN_N_FOR_P95 else None,
        "max_mm": float(d.max()),
    }


# --------------------------------------------------------------------------- #
# robot REST client (thin Pi API; brain owns geometry/IK)
# --------------------------------------------------------------------------- #
@dataclass
class RobotApi:
    host: str
    port: int = 8080
    timeout_s: float = 15.0

    @property
    def base_url(self) -> str:
        return f"http://{self.host}:{self.port}"

    def _request(self, method: str, path: str, body: Optional[Dict[str, Any]] = None,
                 socket_timeout: Optional[float] = None) -> Dict[str, Any]:
        data = json.dumps(body).encode("utf-8") if body is not None else None
        req = urllib.request.Request(
            self.base_url + path, data=data, method=method,
            headers={"Content-Type": "application/json"})
        try:
            with urllib.request.urlopen(req, timeout=socket_timeout or self.timeout_s) as resp:
                return json.loads(resp.read().decode("utf-8") or "{}")
        except urllib.error.HTTPError as e:
            try:
                detail = json.loads(e.read().decode("utf-8") or "{}")
            except Exception:
                detail = {"detail": str(e)}
            raise RuntimeError(f"{method} {path} failed with HTTP {e.code}: {detail}") from e

    def get_status(self) -> Dict[str, Any]:
        return self._request("GET", "/robot/status")

    def move_angles(self, angles: Sequence[float], speed: int) -> Dict[str, Any]:
        return self._request("PUT", "/joints/angles",
                             {"angles": [float(a) for a in angles], "speed": int(speed)})

    def wait(self, target: Sequence[float], tolerance: float, timeout: float) -> Dict[str, Any]:
        # the server blocks up to `timeout` seconds, so the socket must outlive it
        return self._request("POST", "/robot/wait",
                            {"target": [float(a) for a in target],
                             "tolerance": float(tolerance), "timeout": float(timeout)},
                            socket_timeout=float(timeout) + 15.0)

    def stop(self) -> Dict[str, Any]:
        return self._request("POST", "/robot/stop")


def _safe_stop(api: Optional[RobotApi]) -> None:
    if api is None:
        return
    try:
        api.stop()
    except Exception:
        pass


# --------------------------------------------------------------------------- #
# motion + measurement
# --------------------------------------------------------------------------- #
def workspace_ok(xy: Sequence[float], args: argparse.Namespace) -> bool:
    x, y = float(xy[0]), float(xy[1])
    if args.workspace_x_min is not None and x < args.workspace_x_min:
        return False
    if args.workspace_x_max is not None and x > args.workspace_x_max:
        return False
    if args.workspace_y_min is not None and y < args.workspace_y_min:
        return False
    if args.workspace_y_max is not None and y > args.workspace_y_max:
        return False
    return True


def move_xy(api: Optional[RobotApi], xy: Sequence[float], args: argparse.Namespace,
            logger: JsonlLogger, run_id: str, event: str) -> Tuple[List[float], Dict[str, Any]]:
    x, y = float(xy[0]), float(xy[1])
    if not workspace_ok((x, y), args):
        raise RuntimeError(f"command xy outside workspace guard: {(x, y)}")
    q = solve_reachable(x, y, float(args.servo_z), pos_tol_mm=args.ik_pos_tol_mm)
    if q is None:
        raise RuntimeError(f"top-down IK unreachable at ({x:.1f}, {y:.1f}, {args.servo_z:.1f})")
    logger.write({"event": event, "run_id": run_id,
                  "cmd_xyz_mm": [x, y, float(args.servo_z)],
                  "joint_angles_deg": [round(float(a), 3) for a in q],
                  "dry_run": bool(args.dry_run)})
    if args.dry_run or api is None:
        return q, {"completed": True, "reason": "dry_run"}
    api.move_angles(q, args.speed)
    wait = api.wait(q, args.wait_tolerance_deg, args.wait_timeout_s)
    return q, wait


def _frame_id(frame: np.ndarray) -> bytes:
    """Cheap content fingerprint of a decoded frame (downsampled bytes).

    Used to skip frames whose content is identical to the previous one, so a
    frozen/duplicated stream frame cannot fill the stability window.
    """
    return np.ascontiguousarray(frame[::20, ::20]).tobytes()


def measure_stable(url: str, H: np.ndarray, args: argparse.Namespace,
                   pre_xy: Optional[Sequence[float]]) -> Dict[str, Any]:
    """Poll the live stream until the observed marker board XY is stable.

    Two guards make the reading trustworthy on a high-latency buffered stream:
      1. MOVEMENT gate -- if pre_xy is given, stability is only considered after a
         detection has moved >= --move-min-mm from pre_xy, proving we are looking
         at the post-move pose and not a stale frame of the previous pose.
      2. DISTINCT-frame gate -- byte-identical frames are skipped, and the window
         must consist of distinct captured frames spanning >= --stable-min-window-s
         with median scatter <= --stable-tol-mm. A frozen stream cannot pass.

    Returns ok+stable only on a genuine settle; a timeout returns ok=False.
    """
    moved = pre_xy is None
    pre = np.asarray(pre_xy, float) if pre_xy is not None else None
    readings: List[Tuple[float, np.ndarray]] = []   # only collected after movement
    prev_id: Optional[bytes] = None
    n_frames = 0
    n_detect = 0
    rt_ms = int(args.stable_timeout_s * 1000) + 2000
    with StreamReader(url, drain=args.stream_drain, read_timeout_ms=rt_ms) as sr:
        t0 = time.monotonic()
        while time.monotonic() - t0 < args.stable_timeout_s:
            frame = sr.latest()
            fid = _frame_id(frame)
            if prev_id is not None and fid == prev_id:
                if args.read_interval_s > 0:
                    time.sleep(args.read_interval_s)
                continue
            prev_id = fid
            n_frames += 1
            m = marker_board_xy(frame, H)
            if m is not None:
                n_detect += 1
                xy = np.asarray(m, float)
                if not moved and pre is not None and \
                        float(np.linalg.norm(xy - pre)) >= args.move_min_mm:
                    moved = True
                if moved:
                    readings.append((time.monotonic(), xy))
                    if len(readings) >= args.stable_frames:
                        window = readings[-args.stable_frames:]
                        span = window[-1][0] - window[0][0]
                        pts = np.asarray([p for _, p in window])
                        center = np.median(pts, axis=0)
                        spread = float(np.max(np.linalg.norm(pts - center, axis=1)))
                        if span >= args.stable_min_window_s and spread <= args.stable_tol_mm:
                            return {"ok": True, "stable": True,
                                    "xy_mm": [float(center[0]), float(center[1])],
                                    "spread_mm": spread, "n_frames": n_frames,
                                    "n_detect": n_detect,
                                    "elapsed_s": round(time.monotonic() - t0, 2)}
            if args.read_interval_s > 0:
                time.sleep(args.read_interval_s)

    if pre is not None and not moved:
        reason = "no_motion_observed"            # stream likely stale / not advancing
    elif n_detect == 0:
        reason = "marker_not_detected"
    else:
        reason = "did_not_settle"
    return {"ok": False, "stable": False, "reason": reason,
            "n_frames": n_frames, "n_detect": n_detect,
            "elapsed_s": round(time.monotonic() - t0, 2)}


def collect_jitter(url: str, H: np.ndarray, args: argparse.Namespace) -> Tuple[List[List[float]], int]:
    """Static (no-motion) detection scatter over DISTINCT frames.

    Returns (marker points, frames_attempted) so detect_rate reflects real misses
    rather than being pinned to 1.0 by only counting successful detections.
    """
    pts: List[List[float]] = []
    prev_id: Optional[bytes] = None
    attempts = 0
    with StreamReader(url, drain=args.stream_drain) as sr:
        t0 = time.monotonic()
        while attempts < args.jitter_frames and time.monotonic() - t0 < args.stable_timeout_s * 3:
            frame = sr.latest()
            fid = _frame_id(frame)
            if prev_id is not None and fid == prev_id:
                if args.read_interval_s > 0:
                    time.sleep(args.read_interval_s)
                continue
            prev_id = fid
            attempts += 1
            m = marker_board_xy(frame, H)
            if m is not None:
                pts.append([float(m[0]), float(m[1])])
            if args.read_interval_s > 0:
                time.sleep(args.read_interval_s)
    return pts, attempts


# --------------------------------------------------------------------------- #
# main routine
# --------------------------------------------------------------------------- #
def resolve_url(args: argparse.Namespace) -> str:
    if args.dry_run:
        return ""                                # geometry-only dry-run needs no stream
    url = args.url or config.FRAME_SOURCE.stream_url
    if not url:
        raise SystemExit("repeatability needs a streaming URL (rtsp/DASH): "
                         "pass --url or set $SORACAM_STREAM_URL")
    return url


def resolve_anchor(api: Optional[RobotApi], args: argparse.Namespace) -> np.ndarray:
    if args.anchor_base_mm is not None:
        return np.asarray(args.anchor_base_mm, float)
    if api is None:
        raise SystemExit("--anchor-base-mm is required with --dry-run (no robot read)")
    status = api.get_status()
    angles = status.get("joint_angles")
    if not angles or len(angles) != 6:
        raise RuntimeError(f"/robot/status did not return 6 joint angles: {status}")
    return corrected_fk_pos([float(a) for a in angles])[:2]


def recommend(worst_p95: Optional[float], jitter_p95: Optional[float],
              tol: float, enough_cycles: bool) -> str:
    if not enough_cycles or worst_p95 is None:
        return "insufficient_stable_cycles_inconclusive"
    if jitter_p95 is not None and jitter_p95 > tol:
        return "fix_marker_detection_first_static_jitter_exceeds_tol"
    if worst_p95 <= tol:
        return "repeatability_ok_proceed_to_convergence_harness"
    if worst_p95 <= 2.0 * tol:
        return "marginal_widen_tol_or_improve_approach_then_proceed"
    return "repeatability_floor_too_high_fix_hardware_before_servoing"


def run(args: argparse.Namespace) -> None:
    H = np.asarray(load_homography()["H_img2board"], float)
    url = resolve_url(args)
    api = None if args.dry_run else RobotApi(args.host, args.port)
    run_id = args.run_id or _now_id()
    args.log = args.log or config.artifact_path(f"repeatability_{run_id}.jsonl")

    no_guard = all(getattr(args, f"workspace_{b}") is None
                   for b in ("x_min", "x_max", "y_min", "y_max"))
    if no_guard and not args.dry_run:
        print("WARNING: no --workspace-* bounds set; commanded motion is bounded only "
              "by IK reachability (much larger than the board). Set bounds for safety.",
              file=sys.stderr)

    with JsonlLogger(args.log) as logger:
        anchor = resolve_anchor(api, args)
        targets = parse_points(args.targets) or [(float(anchor[0]), float(anchor[1]))]
        away_dx, away_dy = args.away_mm
        logger.write({"event": "config", "run_id": run_id,
                      "anchor_base_mm": [float(anchor[0]), float(anchor[1])],
                      "servo_z_mm": float(args.servo_z),
                      "targets": [list(t) for t in targets],
                      "away_mm": [away_dx, away_dy], "alternate_away": bool(args.alternate_away),
                      "cycles": args.cycles, "tol_mm": args.tol_mm,
                      "move_min_mm": args.move_min_mm, "workspace_guard": not no_guard,
                      "dry_run": bool(args.dry_run)})

        # 1) static jitter (sensor/detection floor) -- no motion
        jitter: Dict[str, Any] = {"n": 0}
        last_xy: Optional[List[float]] = None
        if not args.dry_run and args.jitter_frames > 0:
            jpts, jattempts = collect_jitter(url, H, args)
            jitter = spread_stats(jpts)
            jitter["frames_attempted"] = jattempts
            jitter["detect_rate"] = (len(jpts) / jattempts) if jattempts else None
            logger.write({"event": "jitter_summary", "run_id": run_id, **jitter})
            last_xy = jitter.get("center_xy_mm")   # seed the movement gate

        # 2) command repeatability per target
        aborted = False
        target_summaries: List[Dict[str, Any]] = []
        for tidx, target in enumerate(targets):
            landed: List[List[float]] = []
            away_landed: List[List[float]] = []
            n_stable = 0
            n_unstable = 0
            stop_reason: Optional[str] = None
            for c in range(args.cycles):
                sign = -1.0 if (args.alternate_away and c % 2 == 1) else 1.0
                away = (target[0] + sign * away_dx, target[1] + sign * away_dy)
                cycle_log: Dict[str, Any] = {"event": "cycle", "run_id": run_id,
                                             "target_index": tidx, "cycle": c,
                                             "target_cmd_mm": list(target), "away_cmd_mm": list(away)}
                try:
                    _, dep_wait = move_xy(api, away, args, logger, run_id, "depart")
                    cycle_log["depart_wait_max_error_deg"] = dep_wait.get("max_error")
                    if not (args.dry_run or dep_wait.get("completed")):
                        stop_reason = f"depart_wait_{dep_wait.get('reason', 'failed')}"
                        _safe_stop(api)
                        aborted = True
                        logger.write({"event": "error", "run_id": run_id, "target_index": tidx,
                                      "cycle": c, "reason": stop_reason, "wait": dep_wait})
                        break
                    if args.settle_pre_s > 0 and not args.dry_run:
                        time.sleep(args.settle_pre_s)
                    away_meas = ({"ok": False, "reason": "dry_run"} if args.dry_run
                                 else measure_stable(url, H, args, pre_xy=last_xy))
                    cycle_log["away_measure"] = away_meas
                    if away_meas.get("stable"):
                        last_xy = away_meas["xy_mm"]
                        away_landed.append(away_meas["xy_mm"])

                    _, app_wait = move_xy(api, target, args, logger, run_id, "approach")
                    cycle_log["approach_wait_max_error_deg"] = app_wait.get("max_error")
                    if not (args.dry_run or app_wait.get("completed")):
                        stop_reason = f"approach_wait_{app_wait.get('reason', 'failed')}"
                        _safe_stop(api)
                        aborted = True
                        logger.write({"event": "error", "run_id": run_id, "target_index": tidx,
                                      "cycle": c, "reason": stop_reason, "wait": app_wait})
                        break
                    if args.settle_pre_s > 0 and not args.dry_run:
                        time.sleep(args.settle_pre_s)
                    tgt_meas = ({"ok": False, "reason": "dry_run"} if args.dry_run
                                else measure_stable(url, H, args, pre_xy=last_xy))
                    cycle_log["target_measure"] = tgt_meas
                    if tgt_meas.get("stable"):
                        last_xy = tgt_meas["xy_mm"]
                        landed.append(tgt_meas["xy_mm"])
                        n_stable += 1
                    elif not args.dry_run:
                        n_unstable += 1
                except Exception as exc:
                    stop_reason = "move_or_measure_failed"
                    _safe_stop(api)
                    aborted = True
                    logger.write({"event": "error", "run_id": run_id, "target_index": tidx,
                                  "cycle": c, "reason": stop_reason, "detail": str(exc)})
                    break
                finally:
                    logger.write(cycle_log)

            stats = spread_stats(landed)            # STABLE measurements only
            summary = {
                "event": "target_summary", "run_id": run_id, "target_index": tidx,
                "target_cmd_mm": list(target), "away_mm": [away_dx, away_dy],
                "cycles_requested": args.cycles,
                "cycles_stable": n_stable, "cycles_unstable": n_unstable,
                "repeatability": stats,
                "away_repeatability": spread_stats(away_landed),
                "attainable_at_tol": (stats["p95_mm"] is not None and stats["p95_mm"] <= args.tol_mm),
                "stop_reason": stop_reason,
            }
            logger.write(summary)
            target_summaries.append(summary)
            if aborted:
                break

        stable_counts = [s["cycles_stable"] for s in target_summaries]
        enough = bool(stable_counts) and min(stable_counts) >= args.min_stable_cycles
        worst_p95 = max(
            [s["repeatability"]["p95_mm"] for s in target_summaries
             if s["repeatability"]["p95_mm"] is not None],
            default=None)
        overall = {
            "event": "summary", "run_id": run_id, "tol_mm": args.tol_mm,
            "aborted": aborted, "static_jitter": jitter,
            "n_targets": len(target_summaries),
            "min_cycles_stable": (min(stable_counts) if stable_counts else 0),
            "min_stable_cycles_required": args.min_stable_cycles,
            "worst_repeatability_p95_mm": worst_p95,
            "recommendation": recommend(worst_p95, jitter.get("p95_mm"), args.tol_mm, enough),
        }
        logger.write(overall)
        print("\n=== repeatability summary ===")
        print(json.dumps(_json_safe(overall), indent=2, sort_keys=True))


def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    # robot
    p.add_argument("--host", default="127.0.0.1", help="Pi REST API host")
    p.add_argument("--port", type=int, default=8080)
    p.add_argument("--servo-z", type=float, required=True,
                   help="fixed top-down command Z (corrected-FK/base command space)")
    p.add_argument("--anchor-base-mm", type=parse_xy, default=None, metavar="X,Y",
                   help="command-space anchor XY; default = corrected FK of current joints")
    p.add_argument("--speed", type=int, default=15)
    p.add_argument("--wait-tolerance-deg", type=float, default=1.0)
    p.add_argument("--wait-timeout-s", type=float, default=20.0)
    p.add_argument("--ik-pos-tol-mm", type=float, default=3.0)
    p.add_argument("--dry-run", action="store_true",
                   help="resolve IK and log moves fully offline (no robot, no stream); "
                        "requires --anchor-base-mm")
    # workspace guard
    p.add_argument("--workspace-x-min", type=float, default=None)
    p.add_argument("--workspace-x-max", type=float, default=None)
    p.add_argument("--workspace-y-min", type=float, default=None)
    p.add_argument("--workspace-y-max", type=float, default=None)
    # frame source (only streaming sources are supported here)
    p.add_argument("--backend", default=None, choices=[None, "rtsp"],
                   help="informational; only rtsp/DASH streaming is supported")
    p.add_argument("--url", default=None, help="RTSP/DASH stream URL (or $SORACAM_STREAM_URL)")
    p.add_argument("--stream-drain", type=int, default=2,
                   help="frames to drain per read to stay near-live")
    # repeatability protocol
    p.add_argument("--targets", default=None,
                   help="command-space targets 'x1,y1;x2,y2;...'; default = anchor only")
    p.add_argument("--away-mm", type=parse_xy, default=(30.0, 0.0), metavar="DX,DY",
                   help="offset to the departure pose each cycle (forces a real re-approach)")
    p.add_argument("--alternate-away", action="store_true",
                   help="flip the away direction each cycle (multidirectional/backlash-inclusive)")
    p.add_argument("--cycles", type=int, default=12, help="repeats per target")
    p.add_argument("--min-stable-cycles", type=int, default=8,
                   help="min stable cycles per target before a verdict is trusted")
    p.add_argument("--tol-mm", type=float, default=3.0,
                   help="convergence tolerance to compare repeatability against")
    p.add_argument("--move-min-mm", type=float, default=3.0,
                   help="marker must move at least this far from the previous pose before "
                        "a measurement is accepted (defeats stale-frame readings)")
    p.add_argument("--settle-pre-s", type=float, default=0.3,
                   help="fixed pause after wait before stability polling begins")
    # marker-stability gate
    p.add_argument("--stable-frames", type=int, default=5)
    p.add_argument("--stable-tol-mm", type=float, default=0.5)
    p.add_argument("--stable-min-window-s", type=float, default=1.0)
    p.add_argument("--stable-timeout-s", type=float, default=8.0)
    p.add_argument("--read-interval-s", type=float, default=0.15)
    p.add_argument("--jitter-frames", type=int, default=20,
                   help="distinct frames for the static (no-motion) jitter baseline")
    # logging
    p.add_argument("--run-id", default=None)
    p.add_argument("--log", default=None, help="JSONL log path; default under scripts/vision/calib")
    return p


def main() -> None:
    args = build_parser().parse_args()
    run(args)


if __name__ == "__main__":
    main()
