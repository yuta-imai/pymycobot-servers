#!/usr/bin/env python3
"""In-process wrist-calibration service (RUNS ON THE ROBOT HOST).

Reuses the proven offline fit from ``calibrate.py`` and the pose list from
``poses.py``, but drives the poses through an ALREADY-OPEN controller (the one the
REST API server owns) instead of opening its own serial port. This lets the API
expose wrist calibration without the stop-server / free-serial dance: the same
controller that serves /joints/angles also runs the calibration sweep.

Flow (``run_calibration``):
  power check -> for each pose: send_angles, settle, read back angles + coords,
  read a STABLE gravity sample from the BLE sensor -> records -> offline fit
  (``calibrate.structure_search`` + ``refine_structure``) -> write
  ``corrected_model.json`` -> hot-reload it into the live controller -> park home.

The fit is identical to ``calibrate.py --search`` so results match the CLI path.
Everything here is importable; the CLI collectors/fitters are untouched.
"""

from __future__ import annotations

import json
import os
import sys
import time
from typing import Callable, Dict, List, Optional

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(os.path.dirname(HERE))
for _p in (HERE, ROOT):
    if _p not in sys.path:
        sys.path.insert(0, _p)

import poses as poselib                     # noqa: E402
from gravity_source import make_source      # noqa: E402

# Default model path the controller loads (mycobot_joint_controller._CORRECTED_MODEL_PATH).
DEFAULT_MODEL_PATH = os.path.join(HERE, "corrected_model.json")

LIMITS = poselib.LIMITS


def within_limits(p: List[float]) -> bool:
    return all(LIMITS[i][0] <= a <= LIMITS[i][1] for i, a in enumerate(p, 1))


def fit_model_dict(recs: List[dict], max_cmd_rb: float = 10.0) -> Dict:
    """Fit the corrected wrist model from collected records (same as CLI --search).

    Returns the dict written to corrected_model.json (dh/signs/mount_rotvec/...).
    """
    import numpy as np
    import calibrate as C

    clean = C.clean_filter(recs, max_cmd_rb)
    if len(clean) < 8:
        raise RuntimeError(
            f"only {len(clean)} clean poses (of {len(recs)}); too few to fit "
            f"(arm likely failed to reach commanded poses)")
    m, signs, doff, dalpha = C.structure_search(clean)
    signs, doff, dalpha, Rm, rv = C.refine_structure(clean, signs, doff, dalpha)
    e_clean, _ = C._orient_resid(clean, signs, doff, dalpha, Rm)
    e_full, _ = C._orient_resid(recs, signs, doff, dalpha, Rm)
    rows, full_signs = C.corrected_dh(signs, doff, dalpha)
    return {
        "dh": rows,
        "signs": full_signs,
        "mount_rotvec": list(rv),
        "approach_axis": "J6 rotation axis (z of frame 5)",
        "orient_err_deg": {
            "clean_mean": float(np.mean(e_clean)),
            "clean_max": float(np.max(e_clean)),
            "full_mean": float(np.mean(e_full)),
            "full_max": float(np.max(e_full)),
        },
        "n_poses": len(recs),
        "n_clean": len(clean),
        "structure_search_mean_deg": float(m),
    }


def collect_records(controller, source, pset: List[List[float]], *,
                    speed: int = 20, settle: float = 3.0, samples: int = 12,
                    still_dps: float = 2.5, retries: int = 2,
                    progress_cb: Optional[Callable[[int, int, bool], None]] = None,
                    should_abort: Optional[Callable[[], bool]] = None) -> List[dict]:
    """Drive each pose via the live controller and record a stable gravity sample.

    Mirrors collect.py's loop but uses the passed-in controller (no serial open).
    Never calls firmware send_coords/go_home.
    """
    recs: List[dict] = []
    for i, p in enumerate(pset):
        if should_abort is not None and should_abort():
            raise RuntimeError("calibration aborted")
        controller.mc.send_angles(list(p), speed)
        time.sleep(settle)
        try:
            angles = [round(a, 2) for a in controller.get_all_joint_angles()]
            coords = [round(v, 1) for v in controller._read_coords(retries=3)]
        except Exception:
            angles, coords = list(p), [None] * 6
        g, still, motion = source.read_stable(n=samples, gyro_still_dps=still_dps)
        tries = 0
        while not still and tries < retries:
            tries += 1
            time.sleep(0.8)
            g, still, motion = source.read_stable(n=samples, gyro_still_dps=still_dps)
        recs.append({"i": i, "commanded": [round(x, 2) for x in p], "angles": angles,
                     "coords": coords, "gravity": [round(v, 5) for v in g],
                     "still": bool(still), "motion": round(motion, 4)})
        if progress_cb is not None:
            progress_cb(i + 1, len(pset), bool(still))
    return recs


def run_calibration(controller, *, model_path: str = DEFAULT_MODEL_PATH,
                    pose_set: str = "default", step: int = 30, speed: int = 20,
                    settle: float = 3.0, samples: int = 12, still_dps: float = 2.5,
                    retries: int = 2, max_cmd_rb: float = 10.0, park: bool = True,
                    ble_address: Optional[str] = None, ble_name: Optional[str] = None,
                    ble_mode: str = "witmotion", ble_char: Optional[str] = None,
                    data_out: Optional[str] = None,
                    progress_cb: Optional[Callable[[int, int, bool], None]] = None,
                    should_abort: Optional[Callable[[], bool]] = None) -> Dict:
    """Full in-process calibration: collect -> fit -> write model -> hot-reload.

    Requires servos powered (raises otherwise). Returns a summary dict including
    the fitted orientation error and where the model was written.
    """
    pset = poselib.default_set(step) if pose_set == "default" else poselib.quick_set()
    pset = [p for p in pset if within_limits(p)]
    if not pset:
        raise RuntimeError("no in-limit poses to run")

    is_on = getattr(controller.mc, "is_power_on", lambda: 1)()
    if is_on != 1:
        raise RuntimeError("servos not powered; call /robot/power_on first")

    source = make_source("ble", name=ble_name, address=ble_address,
                         mode=ble_mode, char_uuid=ble_char)
    try:
        recs = collect_records(controller, source, pset, speed=speed, settle=settle,
                               samples=samples, still_dps=still_dps, retries=retries,
                               progress_cb=progress_cb, should_abort=should_abort)
    finally:
        source.close()

    if data_out:
        with open(data_out, "w") as f:
            for r in recs:
                f.write(json.dumps(r) + "\n")

    model = fit_model_dict(recs, max_cmd_rb=max_cmd_rb)

    # Back up the current model, then write + hot-reload the new one.
    if os.path.exists(model_path):
        try:
            os.replace(model_path, model_path + ".prev")
        except OSError:
            pass
    with open(model_path, "w") as f:
        json.dump(model, f, indent=2)
    controller._load_corrected_model()

    if park:
        try:
            controller.mc.send_angles([0, 0, 0, 0, 0, 0], speed)
            time.sleep(2.0)
        except Exception:
            pass

    return {
        "poses_total": len(pset),
        "poses_recorded": len(recs),
        "model_path": model_path,
        "data_out": data_out,
        "orient_err_deg": model["orient_err_deg"],
        "n_clean": model["n_clean"],
    }
