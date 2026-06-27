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


_COVERAGE_BASE = {"jp": "https://api.soracom.io", "g": "https://g.api.soracom.io"}


def grab_soracom_still(*, device_id: str = "", auth_key_id: str = "",
                       auth_key: str = "", coverage: str = "",
                       retries: int = 8, retry_wait_s: float = 3.0) -> np.ndarray:
    """Grab a live still straight from the SORACOM API (stdlib HTTP, no MCP).

    auth -> request a still_picture URL -> download the JPEG. Credentials come
    from args or env (SORACOM_AUTH_KEY_ID / SORACOM_AUTH_KEY / SORACOM_DEVICE_ID
    / SORACOM_COVERAGE). The still takes ~15 s to prepare, so the image URL is
    polled with retries. This lets the setup routine grab frames on the robot
    host without the agent/MCP in the loop.
    """
    import json
    import time
    import urllib.request
    import urllib.error

    device_id = device_id or os.environ.get("SORACOM_DEVICE_ID", "")
    auth_key_id = auth_key_id or os.environ.get("SORACOM_AUTH_KEY_ID", "")
    auth_key = auth_key or os.environ.get("SORACOM_AUTH_KEY", "")
    coverage = coverage or os.environ.get("SORACOM_COVERAGE", "jp")
    base = _COVERAGE_BASE.get(coverage)
    if not (device_id and auth_key_id and auth_key and base):
        raise RuntimeError("soracom backend needs SORACOM_AUTH_KEY_ID, "
                           "SORACOM_AUTH_KEY, SORACOM_DEVICE_ID (coverage jp/g)")

    def _post(path, body, headers=None):
        req = urllib.request.Request(
            base + path, data=json.dumps(body).encode(),
            headers={"Content-Type": "application/json", **(headers or {})},
            method="POST")
        with urllib.request.urlopen(req, timeout=30) as r:
            return json.loads(r.read())

    def _get(path, headers=None):
        req = urllib.request.Request(base + path, headers=headers or {},
                                     method="GET")
        with urllib.request.urlopen(req, timeout=30) as r:
            return json.loads(r.read())

    auth = _post("/v1/auth", {"authKeyId": auth_key_id, "authKey": auth_key,
                              "tokenTimeoutSeconds": 3600})
    hdr = {"X-Soracom-API-Key": auth["apiKey"], "X-Soracom-Token": auth["token"]}
    meta = _get(f"/v1/sora_cam/devices/{device_id}/atom_cam/still_picture", hdr)
    img_url = meta["url"]
    last = None
    for _ in range(retries):
        try:
            with urllib.request.urlopen(img_url, timeout=30) as r:
                buf = r.read()
            if buf:
                return _decode_jpeg(buf)
        except urllib.error.HTTPError as e:      # not ready yet -> wait
            last = e
        time.sleep(retry_wait_s)
    raise RuntimeError(f"still image not ready after {retries} tries ({last})")


def grab(*, backend: Optional[str] = None, url: Optional[str] = None,
         mcp_json: Optional[str] = None) -> np.ndarray:
    """Grab a single BGR frame from the configured (or overridden) backend."""
    backend = backend or FRAME_SOURCE.backend
    if backend == "soracom":
        return grab_soracom_still(device_id=FRAME_SOURCE.device_id)
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


class StreamReader:
    """Keep ONE cv2.VideoCapture open and hand back the freshest decoded frame.

    A plain ``cap.read()`` returns whatever sits at the head of the decoder buffer;
    over a buffered RTSP/DASH live view that frame can lag the real scene by up to
    seconds. Right after a robot move that means a measurement can be taken on a
    stale, mid-motion frame — the dominant validity threat for closed-loop / repeat-
    ability experiments. This reader requests a 1-frame buffer and *drains* a few
    frames on every ``latest()`` so each returned frame is close to live.

    Draining cannot remove end-to-end propagation latency on its own, so callers
    that need certainty a frame reflects a SETTLED pose should still poll
    ``latest()`` until the observed quantity (e.g. the marker board XY) stops
    changing across a real wall-clock window. Use as a context manager:

        with StreamReader(url) as sr:
            frame = sr.latest()
    """

    def __init__(self, url: str, *, buffersize: int = 1, warmup: int = 5,
                 drain: int = 2, timeout_s: float = 10.0,
                 read_timeout_ms: int = 8000, open_timeout_ms: int = 10000):
        import cv2
        self.url = url
        self.drain = max(0, int(drain))
        self.cap = self._open(cv2, url, open_timeout_ms, read_timeout_ms)
        if not self.cap.isOpened():
            raise RuntimeError(f"could not open stream: {url}")
        for prop, val in (("CAP_PROP_BUFFERSIZE", buffersize),
                          ("CAP_PROP_READ_TIMEOUT_MSEC", read_timeout_ms)):
            try:
                if hasattr(cv2, prop):
                    self.cap.set(getattr(cv2, prop), val)
            except Exception:
                pass
        deadline = time.monotonic() + float(timeout_s)
        got = 0
        while got < max(1, warmup) and time.monotonic() < deadline:
            ok, _ = self.cap.read()
            if ok:
                got += 1
        if got == 0:
            self.cap.release()
            raise RuntimeError(f"stream opened but no frame read (timeout): {url}")

    @staticmethod
    def _open(cv2, url: str, open_timeout_ms: int, read_timeout_ms: int):
        """Open with FFmpeg read/open timeouts so a stalled stream surfaces as a
        clean read error instead of blocking grab()/read() indefinitely."""
        params = []
        for prop, val in (("CAP_PROP_OPEN_TIMEOUT_MSEC", open_timeout_ms),
                          ("CAP_PROP_READ_TIMEOUT_MSEC", read_timeout_ms)):
            if hasattr(cv2, prop):
                params += [int(getattr(cv2, prop)), int(val)]
        if params:
            try:
                return cv2.VideoCapture(url, cv2.CAP_FFMPEG, params)
            except Exception:
                pass
        return cv2.VideoCapture(url)

    def latest(self) -> np.ndarray:
        """Drain the buffer and return the newest decoded BGR frame.

        ``grab()`` advances without decoding (cheap), so draining skips backlogged
        frames; the final ``read()`` decodes the freshest one. On a slow stream the
        grabs block on new frames, which still yields a recent frame.
        """
        for _ in range(self.drain):
            self.cap.grab()
        ok, frame = self.cap.read()
        if not ok or frame is None:
            ok, frame = self.cap.read()          # one retry
        if not ok or frame is None:
            raise RuntimeError("stream read failed (EOF or URL expired)")
        return frame

    def close(self) -> None:
        if getattr(self, "cap", None) is not None:
            self.cap.release()
            self.cap = None

    def __enter__(self) -> "StreamReader":
        return self

    def __exit__(self, *_exc: object) -> None:
        self.close()


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--backend", default=None, choices=[None, "soracom", "rtsp", "mcp"])
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
