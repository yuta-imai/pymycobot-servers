#!/usr/bin/env python3
"""Camera placement guide for the eye-to-hand SORACAM (item 2).

Pure math (numpy only). Turns the error-propagation model from
docs/eye-to-hand-soracam.md into numbers so you can choose where to mount the
camera BEFORE touching hardware:

  - ground error per pixel of detection/IFOV error  ~ (R^2/h) * IFOV
  - ground error per mrad of ray/calibration error  ~ (R^2/h) * dtheta
  - "centroid vs contact" parallax for a tall object ~ (obj_h/2) * tan(alpha)

where h = camera height above board, d = horizontal offset to the work point,
R = sqrt(h^2+d^2) slant range, alpha = incidence angle from the board normal.

Run directly to print a placement table and an allowable-region check for a
target pick tolerance:
    python placement_calculator.py --fov 120 --width 1920 \
        --tol-mm 8 --obj-height 60 --work-radius 210
"""

from __future__ import annotations

import argparse
import math
from dataclasses import dataclass


def ifov_mrad(fov_deg: float, width_px: int) -> float:
    """Instantaneous FOV: milliradians of scene per pixel (horizontal)."""
    return math.radians(fov_deg) / width_px * 1000.0


def slant_sensitivity_mm_per_mrad(h_mm: float, d_mm: float) -> float:
    """Ground error (mm) per 1 mrad of ray error, elevation direction = R^2/h.

    This is the dominant, obliquity-amplified term: d(ground)/d(theta) = R^2/h.
    """
    R = math.hypot(h_mm, d_mm)
    return (R * R / h_mm) * 1e-3            # (mm) per mrad


def incidence_deg(h_mm: float, d_mm: float) -> float:
    """Angle between the ray and the board normal at the work point (deg)."""
    return math.degrees(math.atan2(d_mm, h_mm))


def parallax_offset_mm(obj_height_mm: float, h_mm: float, d_mm: float) -> float:
    """Footprint error if you use the visual centroid of a tall object instead
    of its board contact point: (obj_height/2) * tan(incidence)."""
    alpha = math.radians(incidence_deg(h_mm, d_mm))
    return (obj_height_mm / 2.0) * math.tan(alpha)


@dataclass
class ErrorBudget:
    h_mm: float
    d_mm: float
    incidence_deg: float
    mm_per_pixel: float          # detection / IFOV term (1 px)
    mm_per_mrad: float           # calibration / ray term (1 mrad)
    parallax_mm: float           # centroid-vs-contact (if not using contact)
    vision_rss_mm: float         # RSS of vision terms (excludes robot)
    total_rss_mm: float          # incl. robot execution term


def error_budget(h_mm: float, d_mm: float, fov_deg: float, width_px: int,
                 *, detect_px: float = 1.0, calib_mrad: float = 1.5,
                 obj_height_mm: float = 0.0, use_contact: bool = True,
                 robot_mm: float = 3.5) -> ErrorBudget:
    """Compose the per-source ground errors at one work point into an RSS budget.

    detect_px : object localization error in pixels (incl. residual distortion).
    calib_mrad: residual ray error from intrinsics+extrinsics+touch (mrad).
    obj_height_mm/use_contact: if use_contact=False, add the centroid parallax.
    robot_mm  : the arm's own execution error (top-down IK + send_angles).
    """
    s_mm_per_mrad = slant_sensitivity_mm_per_mrad(h_mm, d_mm)
    mm_px = detect_px * ifov_mrad(fov_deg, width_px) * s_mm_per_mrad
    mm_mrad = calib_mrad * s_mm_per_mrad
    parallax = 0.0 if use_contact else parallax_offset_mm(obj_height_mm, h_mm, d_mm)
    vision = math.sqrt(mm_px ** 2 + mm_mrad ** 2 + parallax ** 2)
    total = math.sqrt(vision ** 2 + robot_mm ** 2)
    return ErrorBudget(h_mm, d_mm, incidence_deg(h_mm, d_mm),
                       mm_px, mm_mrad, parallax, vision, total)


def _table(args):
    print(f"FOV={args.fov}deg  width={args.width}px  "
          f"IFOV={ifov_mrad(args.fov, args.width):.2f} mrad/px  "
          f"detect={args.detect_px}px  calib={args.calib_mrad}mrad  "
          f"robot={args.robot_mm}mm  contact={'yes' if args.use_contact else 'no'}")
    print(f"target pick tolerance = {args.tol_mm} mm; work radius = "
          f"{args.work_radius} mm (worst point d = work_radius)")
    print()
    hdr = (f"{'h(mm)':>7}{'d(mm)':>7}{'incid':>7}{'mm/px':>8}{'mm/mrad':>9}"
           f"{'parax':>7}{'vis_rss':>8}{'tot_rss':>8}{'  verdict':>10}")
    print(hdr)
    print("-" * len(hdr))
    heights = [float(x) for x in args.heights.split(",")]
    d = args.work_radius                      # evaluate the worst (edge) point
    for h in heights:
        b = error_budget(h, d, args.fov, args.width,
                         detect_px=args.detect_px, calib_mrad=args.calib_mrad,
                         obj_height_mm=args.obj_height, use_contact=args.use_contact,
                         robot_mm=args.robot_mm)
        ok = "OK" if b.total_rss_mm <= args.tol_mm else "TOO BIG"
        print(f"{h:7.0f}{d:7.0f}{b.incidence_deg:7.1f}{b.mm_per_pixel:8.2f}"
              f"{b.mm_per_mrad:9.2f}{b.parallax_mm:7.2f}{b.vision_rss_mm:8.2f}"
              f"{b.total_rss_mm:8.2f}{ok:>10}")
    print()
    print("Reading it: pick the smallest h whose tot_rss <= tolerance at the work")
    print("edge. Lower h and a more top-down view (small incidence) shrink every")
    print("term; using the object's CONTACT point (not centroid) zeroes parallax.")


def _selftest():
    # Monotonicity / sanity checks of the model.
    e_close = error_budget(800, 300, 120, 1920)
    e_far = error_budget(500, 1300, 120, 1920)
    assert e_far.vision_rss_mm > e_close.vision_rss_mm, "oblique/far must be worse"
    # parallax must vanish at nadir and grow with obliquity
    assert parallax_offset_mm(60, 800, 0) < 1e-9
    assert parallax_offset_mm(60, 500, 1300) > parallax_offset_mm(60, 800, 300)
    # IFOV halves when resolution doubles
    assert abs(ifov_mrad(120, 1920) * 2 - ifov_mrad(120, 960)) < 1e-9
    print(f"[close mount h800 d300] incid={e_close.incidence_deg:.1f}deg "
          f"vis={e_close.vision_rss_mm:.2f}mm tot={e_close.total_rss_mm:.2f}mm")
    print(f"[far oblique h500 d1300] incid={e_far.incidence_deg:.1f}deg "
          f"vis={e_far.vision_rss_mm:.2f}mm tot={e_far.total_rss_mm:.2f}mm")
    print("placement_calculator self-tests passed")


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--fov", type=float, default=120.0, help="horizontal FOV (deg)")
    ap.add_argument("--width", type=int, default=1920, help="image width (px)")
    ap.add_argument("--heights", default="400,500,600,700,800,1000",
                    help="comma list of camera heights above board (mm)")
    ap.add_argument("--work-radius", type=float, default=210.0,
                    help="horizontal distance to the farthest work point (mm)")
    ap.add_argument("--tol-mm", type=float, default=8.0, help="target pick tolerance")
    ap.add_argument("--detect-px", type=float, default=1.0)
    ap.add_argument("--calib-mrad", type=float, default=1.5)
    ap.add_argument("--robot-mm", type=float, default=3.5)
    ap.add_argument("--obj-height", type=float, default=60.0)
    ap.add_argument("--use-contact", action="store_true", default=True,
                    help="detect object board-contact (zeroes parallax; default)")
    ap.add_argument("--use-centroid", dest="use_contact", action="store_false",
                    help="use visual centroid instead (adds parallax error)")
    ap.add_argument("--selftest", action="store_true")
    args = ap.parse_args()
    if args.selftest:
        _selftest()
        return
    _table(args)


if __name__ == "__main__":
    main()
