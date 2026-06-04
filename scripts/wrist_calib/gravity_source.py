#!/usr/bin/env python3
"""Pluggable gravity/orientation source for wrist calibration.

The calibration only needs the GRAVITY VECTOR in the sensor body frame while the
arm is held static. That 2-DOF measurement (gravity direction; yaw about vertical
is unobservable from accel alone) is exactly what pins down the broken wrist TILT.

The concrete sensor is not chosen yet (plan: a battery BLE accelerometer the Pi
reads over BLE — either a beacon whose manufacturer-data carries accel, or a BLE
UART/NUS link streaming "gx,gy,gz"). So everything is written against the
GravitySource interface; only BLEGravitySource.read() needs filling in once the
device + payload format are known. ManualGravitySource and ReplayGravitySource let
the whole pipeline (collect -> fit -> validate) run and be tested TODAY without it.

Units: any consistent unit; vectors are normalized before use. Convention: the
returned vector is the specific force the accelerometer reports at rest, i.e. it
points UP (≈ +1 g along the axis opposing gravity) for a typical MEMS accel. The
fit is sign-agnostic per pose (it fits R_mount), so an "up" vs "down" convention is
absorbed — but stay consistent across a run.
"""

from __future__ import annotations

import statistics
import time
from typing import List, Optional, Tuple


class GravitySource:
    """Abstract source of the gravity vector in the sensor body frame."""

    def read(self) -> Tuple[float, float, float]:
        """Return one raw (gx, gy, gz) sample. Override."""
        raise NotImplementedError

    def close(self) -> None:
        """Release any hardware handle. Override if needed."""

    # ---- shared helpers -------------------------------------------------
    def read_stable(self, n: int = 12, period_s: float = 0.1,
                    max_std_ratio: float = 0.03, gyro_still_dps: float = 2.0
                    ) -> Tuple[Tuple[float, float, float], bool, float]:
        """Average n samples; report whether the arm was still.

        Returns (unit_mean_xyz, still, motion). When the source also exposes a
        gyro (gyro_magnitude()), stillness is gyro-based: `motion` is the max
        angular speed (deg/s) seen and `still = motion < gyro_still_dps` — far
        more reliable than accel variance at the ~10 Hz sensor rate (a slowly
        moving arm can show low accel variance yet clear gyro). Without a gyro it
        falls back to the accel-magnitude std ratio (`motion` = that ratio).

        Default pacing (n=12, period=0.1s ≈ 1.2 s window) matches the WT9011DCL's
        ~10 Hz stream so each loop sees fresh samples, not repeats.
        """
        gyro_fn = getattr(self, "gyro_magnitude", None)
        xs: List[Tuple[float, float, float]] = []
        gyros: List[float] = []
        for _ in range(max(1, n)):
            xs.append(tuple(float(v) for v in self.read()))
            if gyro_fn is not None:
                gm = gyro_fn()
                if gm is not None:
                    gyros.append(gm)
            time.sleep(period_s)
        mx = sum(x[0] for x in xs) / len(xs)
        my = sum(x[1] for x in xs) / len(xs)
        mz = sum(x[2] for x in xs) / len(xs)
        norm = (mx * mx + my * my + mz * mz) ** 0.5 or 1e-9
        unit = (mx / norm, my / norm, mz / norm)
        if gyros:
            motion = max(gyros)
            return unit, (motion < gyro_still_dps), motion
        mags = [(_x ** 2 + _y ** 2 + _z ** 2) ** 0.5 for _x, _y, _z in xs]
        mean_mag = sum(mags) / len(mags) or 1e-9
        std_ratio = (statistics.pstdev(mags) if len(mags) > 1 else 0.0) / mean_mag
        return unit, (std_ratio <= max_std_ratio), std_ratio


class ManualGravitySource(GravitySource):
    """Type the reading in by hand (e.g. from a phone inclinometer / app).

    Useful for a quick quantitative spot-check before the BLE sensor exists.
    Enter three numbers separated by space/comma, or 'skip'.
    """

    def read(self) -> Tuple[float, float, float]:
        raw = input("  gravity gx gy gz (sensor frame) > ").strip()
        if raw.lower() in ("skip", "s", ""):
            raise KeyboardInterrupt("manual reading skipped")
        parts = raw.replace(",", " ").split()
        if len(parts) != 3:
            raise ValueError("need exactly 3 numbers")
        return float(parts[0]), float(parts[1]), float(parts[2])

    def read_stable(self, n=1, period_s=0.0, max_std_ratio=1.0):
        x = self.read()
        norm = (x[0] ** 2 + x[1] ** 2 + x[2] ** 2) ** 0.5 or 1e-9
        return (x[0] / norm, x[1] / norm, x[2] / norm), True, 0.0


class ReplayGravitySource(GravitySource):
    """Replay a fixed list of samples — for unit-testing the fit pipeline."""

    def __init__(self, samples: List[Tuple[float, float, float]]):
        self._samples = list(samples)
        self._i = 0

    def read(self) -> Tuple[float, float, float]:
        s = self._samples[self._i % len(self._samples)]
        self._i += 1
        return s


class BLEGravitySource(GravitySource):
    """BLE accelerometer read by the Pi. FILL IN once the device is chosen.

    Two common integration shapes — pick whichever the chosen sensor uses:

    A) BLE UART / Nordic UART Service (NUS): the device streams ASCII lines like
       "gx,gy,gz\\n". Subscribe to the TX characteristic, keep the latest line.
    B) BLE beacon / advertisement: accel is packed in manufacturer-specific data.
       Scan, parse the bytes (scale per the datasheet), keep the latest.

    Both are easy with `bleak` (cross-platform, works on the Pi). This class keeps
    a background event loop that updates self._latest; read() returns it. The ONLY
    device-specific code is the payload parsing in _parse_* below.
    """

    # WitMotion WT9011DCL (BLE 5.0) UUIDs.
    WITMOTION_NOTIFY = "0000ffe4-0000-1000-8000-00805f9a34fb"  # data notifications
    WITMOTION_WRITE = "0000ffe9-0000-1000-8000-00805f9a34fb"   # command (optional)

    def __init__(self, address: Optional[str] = None, name: Optional[str] = None,
                 mode: str = "witmotion", char_uuid: Optional[str] = None,
                 timeout_s: float = 20.0):
        self.address = address      # MAC (Linux) or UUID (macOS); None -> match by name
        self.name = name
        self.mode = mode            # "witmotion" | "nus" | "beacon"
        # data/notify characteristic; default to WitMotion for the WT9011DCL
        self.char_uuid = char_uuid or (self.WITMOTION_NOTIFY
                                       if mode == "witmotion" else None)
        self.timeout_s = timeout_s
        self._latest: Optional[Tuple[float, float, float]] = None
        self._gyro: Optional[Tuple[float, float, float]] = None  # for stillness
        # The notify callback ONLY stores raw bytes (cannot throw -> can't kill the
        # BLE stream); decoding happens in read(), in the caller's thread. An
        # earlier design parsed inside the callback and a parse hiccup silently
        # froze the stream (stale _latest forever). Keep callbacks trivial.
        self._raw_latest: Optional[bytes] = None
        self._thread = None
        self._loop = None
        self._stop = False
        self._err = None
        self._start()

    # -- device-specific decoding (pure; called from read()) -------------
    @staticmethod
    def _decode_witmotion(data: bytes):
        """Decode ONE WT9011DCL packet. Returns (ax,ay,az,gx,gy,gz) or None.

        Combined packet (20 B): 0x55 0x61 + accel,gyro,angle (9x int16 LE):
          accel(g)=raw/32768*16, gyro(deg/s)=raw/32768*2000, angle(deg)=raw/32768*180
        Also accepts the 11 B accel frame 0x55 0x51 (gyro fields -> None).
        Each BLE notification carries exactly one packet, so no buffering needed.
        """
        import struct
        b = bytes(data)
        for i in range(len(b) - 1):
            if b[i] != 0x55:
                continue
            t = b[i + 1]
            if t == 0x61 and i + 20 <= len(b):
                v = struct.unpack_from("<hhhhhhhhh", b, i + 2)
                return (v[0] / 32768.0 * 16.0, v[1] / 32768.0 * 16.0,
                        v[2] / 32768.0 * 16.0, v[3] / 32768.0 * 2000.0,
                        v[4] / 32768.0 * 2000.0, v[5] / 32768.0 * 2000.0)
            if t == 0x51 and i + 11 <= len(b):
                v = struct.unpack_from("<hhh", b, i + 2)
                return (v[0] / 32768.0 * 16.0, v[1] / 32768.0 * 16.0,
                        v[2] / 32768.0 * 16.0, None, None, None)
        return None

    @staticmethod
    def _parse_nus_line(data: bytes) -> Optional[Tuple[float, float, float]]:
        try:
            parts = data.decode("ascii", "ignore").strip().replace(",", " ").split()
            if len(parts) >= 3:
                return float(parts[0]), float(parts[1]), float(parts[2])
        except Exception:
            return None
        return None

    @staticmethod
    def _parse_beacon_mfr(mfr_data: dict) -> Optional[Tuple[float, float, float]]:
        # TODO: per datasheet — e.g. int16 LE at a known offset, scale to g.
        return None

    # -- background BLE loop --------------------------------------------
    def _start(self):
        import threading
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

        def have_sample():
            return self._raw_latest is not None or self._latest is not None
        t0 = time.time()
        while not have_sample() and self._err is None and time.time() - t0 < self.timeout_s:
            time.sleep(0.05)
        if self._err is not None:
            raise RuntimeError(f"BLE connect failed: {self._err}")
        if not have_sample():
            raise TimeoutError(
                "No BLE sample within timeout. Is the sensor on and free (only one "
                "central can connect)? Try --ble-address, or power-cycle the sensor."
            )

    def _run(self):
        import asyncio
        self._loop = asyncio.new_event_loop()
        asyncio.set_event_loop(self._loop)
        try:
            self._loop.run_until_complete(self._main())
        except Exception as e:  # surface in read()
            self._err = e

    async def _main(self):
        import asyncio  # needed here: this method (not _run) awaits asyncio.sleep
        from bleak import BleakScanner, BleakClient

        if self.mode == "beacon":
            def cb(dev, adv):
                v = self._parse_beacon_mfr(adv.manufacturer_data or {})
                if v is not None and (self.address is None or dev.address == self.address) \
                        and (self.name is None or (dev.name or "") == self.name):
                    self._latest = v
            scanner = BleakScanner(detection_callback=cb)
            await scanner.start()
            while not self._stop:
                await asyncio.sleep(0.05)
            await scanner.stop()
            return

        # connect + subscribe (witmotion | nus)
        address = self.address
        if address is None:
            dev = await BleakScanner.find_device_by_name(self.name, timeout=self.timeout_s)
            if dev is None:
                raise RuntimeError(f"BLE device named {self.name!r} not found")
            address = dev.address
        async with BleakClient(address) as client:
            def on_notify(_handle, data: bytearray):
                # trivial + exception-free: just stash the raw bytes
                if self.mode == "witmotion":
                    self._raw_latest = bytes(data)
                else:  # nus
                    v = self._parse_nus_line(bytes(data))
                    if v is not None:
                        self._latest = v
            await client.start_notify(self.char_uuid, on_notify)
            while not self._stop:
                await asyncio.sleep(0.05)
            try:
                await client.stop_notify(self.char_uuid)
            except Exception:
                pass

    def gyro_magnitude(self) -> Optional[float]:
        """Latest angular-speed magnitude (deg/s) — for stillness gating."""
        if self._gyro is None:
            return None
        return (self._gyro[0] ** 2 + self._gyro[1] ** 2 + self._gyro[2] ** 2) ** 0.5

    def read(self) -> Tuple[float, float, float]:
        if self.mode == "witmotion":
            if self._raw_latest is None and self._latest is None:
                raise RuntimeError("no BLE sample yet")
            dec = self._decode_witmotion(self._raw_latest) if self._raw_latest else None
            if dec is not None:
                self._latest = dec[0:3]
                if dec[3] is not None:
                    self._gyro = dec[3:6]
            if self._latest is None:
                raise RuntimeError("no decodable WitMotion packet yet")
            return self._latest
        if self._latest is None:
            raise RuntimeError("no BLE sample yet")
        return self._latest

    def close(self) -> None:
        self._stop = True
        if self._thread is not None:
            self._thread.join(timeout=3.0)  # let stop_notify + disconnect complete


def make_source(kind: str, **kw) -> GravitySource:
    kind = kind.lower()
    if kind == "manual":
        return ManualGravitySource()
    if kind == "ble":
        return BLEGravitySource(**kw)
    if kind == "replay":
        return ReplayGravitySource(kw.get("samples", [(0, 0, 1)]))
    raise ValueError(f"unknown gravity source {kind!r}")
