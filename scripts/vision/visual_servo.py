#!/usr/bin/env python3
"""Marker visual servo: drive the gripper to a target BOARD (x,y) by closing the
loop on the camera-observed gripper marker — robust to corrected_fk position
distortion and to board->base calibration error.

Why: open-loop board->base (even the RBF, ~12 mm) leaves a residual because the
arm's absolute position FK is distorted. The fixed SORACAM sees the gripper marker
directly, so we can null the residual in BOARD space: command a base pose, read
where the marker actually landed (board mm via the pixel->board homography), and
correct using a constant interaction matrix A = d(board)/d(base) estimated once
from the board->base calibration pairs.

State machine (frames come from the soracam MCP still; the loop is stepped by an
external caller that grabs a still between steps):

    visual_servo.py init <board_x> <board_y>   # RBF open-loop seed -> move+wait
    <grab a soracam still>
    visual_servo.py step <board_x> <board_y>   # read latest still -> correct -> move
    <grab a still> ; repeat step until it prints CONVERGED or MAX_ITERS

Reads:  calib/board_to_base_rbf.json (+ obs) and homography.json.
Writes: scratchpad servo_state.json (current base, iter, history).
"""
import sys, os, glob, json, base64, re, urllib.request
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import numpy as np
import config
from homography_calibrate import load_homography
from touch_calibrate import load_board_to_base
from geometry import board_to_base_fn
import gripper_marker as gm
from topdown_ik import solve_reachable, _limit_margin

HOST = "http://192.168.0.136:8080"
SCRATCH = "/tmp/claude-1000/-home-factory-playground-pymycobot-servers/c64f2264-9a81-49d5-8071-e2d5b69c70fa/scratchpad"
TR = "/home/factory/.claude/projects/-home-factory-playground-pymycobot-servers/c64f2264-9a81-49d5-8071-e2d5b69c70fa/tool-results"
STATE = os.path.join(SCRATCH, "servo_state.json")

TOL_MM = 3.0          # convergence tolerance in board mm
GAIN = 0.6            # damped IBVS step
MAX_STEP_MM = 20.0    # cap per-iter base move: base->board is ill-conditioned in
                      # the thin top-down envelope (det A ~0.15), so an unclamped
                      # A^{-1} step can fling the arm; cap keeps it stable.
MAX_ITERS = 8
SERVO_Z = 90.0        # fixed top-down hover (command/corrected-fk space)
# reachable top-down envelope guard (base mm), from the calibration sweep
BASE_GUARD = {"x": (-100, 36), "y": (-142, -108)}


def req(m, p, b=None, to=45):
    d = json.dumps(b).encode() if b is not None else None
    r = urllib.request.Request(HOST + p, data=d, method=m, headers={"Content-Type": "application/json"})
    with urllib.request.urlopen(r, timeout=to) as x:
        return json.loads(x.read().decode() or "{}")


def base2board_jacobian():
    """A (2x2), c (2,) s.t. board ~= A @ base + c, from the calibration pairs.

    A local-linear surrogate of the (non-projective) base->board map, good enough
    for a damped Newton step. Returns (A, c)."""
    rbf_path = config.artifact_path(config.HANDEYE_RBF_JSON)
    d = json.load(open(rbf_path))
    B = np.asarray(d["commanded_base_mm"], float)   # base xy
    O = np.asarray(d["observed_board_mm"], float)    # board xy
    # board = [base 1] @ W  ->  W is (3x2); A = W[:2].T, c = W[2]
    M = np.hstack([B, np.ones((len(B), 1))])
    W, *_ = np.linalg.lstsq(M, O, rcond=None)
    return W[:2].T, W[2]


def clamp_base(base):
    return np.array([min(max(base[0], BASE_GUARD["x"][0]), BASE_GUARD["x"][1]),
                     min(max(base[1], BASE_GUARD["y"][0]), BASE_GUARD["y"][1])], float)


def move_base(base):
    q = solve_reachable(float(base[0]), float(base[1]), SERVO_Z, pos_tol_mm=3.0)
    if q is None:
        return None, "unreachable"
    req("PUT", "/joints/angles", {"angles": [float(a) for a in q], "speed": 20})
    w = req("POST", "/robot/wait", {"target": [float(a) for a in q],
                                    "tolerance": 2.0, "timeout": 25.0})
    return q, w.get("reason")


def newest_still():
    return max(glob.glob(os.path.join(TR, "mcp-soracam-get_live_still_image-*.txt")),
               key=os.path.getmtime)


def observe_board():
    """Decode the newest soracam still, return the gripper marker board xy (or None)."""
    import cv2
    raw = open(newest_still(), encoding="utf-8").read()
    j = json.loads(raw)

    def walk(o):
        if isinstance(o, str) and len(o) > 1000:
            return o
        if isinstance(o, dict):
            for v in o.values():
                r = walk(v)
                if r:
                    return r
        if isinstance(o, list):
            for v in o:
                r = walk(v)
                if r:
                    return r
    b64 = re.sub(r'^data:image/\w+;base64,', '', walk(j))
    img = cv2.imdecode(np.frombuffer(base64.b64decode(b64 + "=" * (-len(b64) % 4)), np.uint8),
                       cv2.IMREAD_COLOR)
    H = np.asarray(load_homography()["H_img2board"], float)
    xy = gm.marker_board_xy(img, H)
    return None if xy is None else np.array(xy, float)


def main():
    mode = sys.argv[1]
    target = np.array([float(sys.argv[2]), float(sys.argv[3])], float)
    A, c = base2board_jacobian()
    Ainv = np.linalg.inv(A)

    if mode == "init":
        b2b = load_board_to_base()
        base = clamp_base(board_to_base_fn(b2b["H_board2base"])(target))
        q, reason = move_base(base)
        st = {"target": target.tolist(), "base": base.tolist(), "iter": 0,
              "history": [], "move_reason": reason}
        json.dump(st, open(STATE, "w"))
        print(f"[init] target_board={target.tolist()} open-loop base={[round(v,1) for v in base]} "
              f"move={reason} (now grab a still, then `step`)")
        return

    st = json.load(open(STATE))
    base = np.array(st["base"], float)
    obs = observe_board()
    if obs is None:
        print(f"[step {st['iter']}] marker NOT detected — regrab / nudge; no move issued")
        return
    err = target - obs
    emag = float(np.linalg.norm(err))
    st["history"].append({"iter": st["iter"], "base": base.tolist(),
                          "observed": [round(float(obs[0]), 1), round(float(obs[1]), 1)],
                          "err_mm": round(emag, 2)})
    if emag <= TOL_MM:
        json.dump(st, open(STATE, "w"))
        print(f"[step {st['iter']}] CONVERGED: observed={[round(float(v),1) for v in obs]} "
              f"target={target.tolist()} err={emag:.2f} mm <= {TOL_MM}")
        return
    if st["iter"] >= MAX_ITERS:
        json.dump(st, open(STATE, "w"))
        print(f"[step {st['iter']}] MAX_ITERS reached; last err={emag:.2f} mm")
        return
    # damped Newton correction: Δbase = gain * A^{-1} (target - observed),
    # magnitude-capped (A is ill-conditioned in the thin envelope).
    step = GAIN * (Ainv @ err)
    smag = float(np.linalg.norm(step))
    if smag > MAX_STEP_MM:
        step = step * (MAX_STEP_MM / smag)
    base_new = clamp_base(base + step)
    q, reason = move_base(base_new)
    st["iter"] += 1
    st["base"] = base_new.tolist()
    st["move_reason"] = reason
    json.dump(st, open(STATE, "w"))
    print(f"[step {st['iter']}] observed={[round(float(v),1) for v in obs]} err={emag:.2f}mm "
          f"-> base {[round(v,1) for v in base]}->{[round(v,1) for v in base_new]} move={reason} "
          f"(grab a still, then `step` again)")


if __name__ == "__main__":
    main()
