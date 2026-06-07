#!/usr/bin/env python3
"""Frame acquisition for the SORACAM eye-to-hand camera.

Two backends (see config.FRAME_SOURCE):
  rtsp : cv2.VideoCapture on a stream URL. Best for calibration bursts; run on
         the robot host / Pi where the camera network is reachable. DASH URLs
         from the soracam MCP get_live_view_unlimited tool also work (ffmpeg).
  mcp  : decode a base64 JPEG saved from the soracam MCP get_live_still_image
         tool result (a JSON blob with "imageBase64"). One-shot glances only;
         ~15 s/frame, not for calibration bursts.

A frame is returned as a BGR uint8 numpy array (OpenCV convention). Requires
OpenCV; the mcp backend additionally just needs base64+json (stdlib).

CLI: capture frames for intrinsics calibration into a folder:
    python frame_source.py --backend rtsp --url rtsp://... --burst 25 \
        --interval 0.8 --out-dir captures/
"""

from __future__ import annotations

import argparse
import base64
import json
import os
import time
from typing import Optional

import numpy as np

from config import FRAME_SOURCE


def _decode_jpeg(buf: bytes) -> np.ndarray:
    import cv2
    arr = np.frombuffer(buf, dtype=np.uint8)
    img = cv2.imdecode(arr, cv2.IMREAD_COLOR)
    if img is None:
        raise RuntimeError("cv2.imdecode failed (corrupt/empty image bytes)")
    return img


def from_mcp_still(json_path: str) -> np.ndarray:
    """Decode a saved soracam MCP get_live_still_image result (JSON w/ imageBase64)."""
    with open(json_path, "r") as f:
        data = json.load(f)
    if "imageBase64" not in data:
        raise RuntimeError(f"{json_path} has no imageBase64 field")
    return _decode_jpeg(base64.b64decode(data["imageBase64"]))


def grab_rtsp(url: str, *, warmup: int = 5, timeout_s: float = 10.0) -> np.ndarray:
    """Open a stream URL, discard a few warmup frames, return one BGR frame."""
    import cv2
    cap = cv2.VideoCapture(url)
    if not cap.isOpened():
        raise RuntimeError(f"could not open stream: {url}")
    try:
        deadline = time.monotonic() + timeout_s
        frame = None
        for _ in range(max(1, warmup)):
            ok, f = cap.read()
            if ok:
                frame = f
            if time.monotonic() > deadline:
                break
        if frame is None:
            raise RuntimeError("stream opened but no frame read (timeout)")
        return frame
    finally:
        cap.release()


def grab(*, backend: Optional[str] = None, url: Optional[str] = None,
         mcp_json: Optional[str] = None) -> np.ndarray:
    """Grab a single BGR frame from the configured (or overridden) backend."""
    backend = backend or FRAME_SOURCE.backend
    if backend == "rtsp":
        url = url or FRAME_SOURCE.stream_url
        if not url:
            raise RuntimeError("rtsp backend needs a stream URL "
                               "(config / $SORACAM_STREAM_URL / --url)")
        return grab_rtsp(url)
    if backend == "mcp":
        if not mcp_json:
            raise RuntimeError("mcp backend needs --mcp-json (saved tool result)")
        return from_mcp_still(mcp_json)
    raise ValueError(f"unknown backend: {backend}")


def capture_burst(out_dir: str, *, url: str, count: int, interval_s: float,
                  warmup: int = 5) -> int:
    """Capture `count` frames from an RTSP/DASH stream into out_dir as PNGs.

    Keeps ONE VideoCapture open across the burst (re-opening per frame is slow and
    SORACAM live URLs are short-lived). Returns the number written.
    """
    import cv2
    os.makedirs(out_dir, exist_ok=True)
    cap = cv2.VideoCapture(url)
    if not cap.isOpened():
        raise RuntimeError(f"could not open stream: {url}")
    written = 0
    try:
        for _ in range(warmup):
            cap.read()
        for i in range(count):
            ok, frame = cap.read()
            if not ok:
                print(f"[capture] frame {i}: read failed, retrying once")
                ok, frame = cap.read()
                if not ok:
                    continue
            path = os.path.join(out_dir, f"frame_{i:03d}.png")
            cv2.imwrite(path, frame)
            written += 1
            print(f"[capture] {written}/{count} -> {path}  "
                  "(move/tilt the board to a new pose now)")
            time.sleep(interval_s)
    finally:
        cap.release()
    return written


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--backend", default=None, choices=[None, "rtsp", "mcp"])
    ap.add_argument("--url", default=None)
    ap.add_argument("--mcp-json", default=None)
    ap.add_argument("--burst", type=int, default=0,
                    help="capture N frames into --out-dir (rtsp only)")
    ap.add_argument("--interval", type=float, default=0.8)
    ap.add_argument("--out-dir", default="captures")
    ap.add_argument("--save", default=None, help="grab one frame and save here")
    args = ap.parse_args()

    if args.burst > 0:
        url = args.url or FRAME_SOURCE.stream_url
        n = capture_burst(args.out_dir, url=url, count=args.burst,
                          interval_s=args.interval)
        print(f"wrote {n} frames to {args.out_dir}")
        return

    img = grab(backend=args.backend, url=args.url, mcp_json=args.mcp_json)
    print(f"grabbed frame {img.shape[1]}x{img.shape[0]}")
    if args.save:
        import cv2
        cv2.imwrite(args.save, img)
        print(f"saved {args.save}")


if __name__ == "__main__":
    main()
