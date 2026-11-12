"""Camera helpers shared by the rig publishers (no heavy imports; unit-testable).

The original scripts reused the previous frame after a failed grab and never released
failed grab results.  These helpers return a frame only for a *new* successful
acquisition and always release the grab result.
"""

import os
import time
from pathlib import Path

# pypylon.TimeoutHandling_Return; injected so tests do not need pypylon.
DEFAULT_TIMEOUT_MS = 1000


def acquire(camera, converter, timeout_handling, timeout_ms=DEFAULT_TIMEOUT_MS):
    """Return one new BGR frame, or None for a timeout/invalid/failed grab or convert error.

    A grab result that was retrieved is always released, whatever happens next.
    """
    grab = None
    try:
        grab = camera.RetrieveResult(timeout_ms, timeout_handling)
        if grab is None:
            return None
        is_valid = getattr(grab, "IsValid", None)
        if callable(is_valid) and not is_valid():
            return None
        if not grab.GrabSucceeded():
            return None
        return converter.Convert(grab).GetArray()
    except Exception:  # timeout exception, convert failure, camera error: no new frame
        return None
    finally:
        if grab is not None:
            try:
                grab.Release()
            except Exception:
                pass


def choose_device(devices, serial):
    """Pick one Basler device.

    With a serial it must match an attached camera.  Without one, the choice is only
    unambiguous when exactly one camera is attached; otherwise this refuses to guess,
    so two publishers can never silently open the same camera.
    """
    devices = list(devices)
    if not devices:
        raise RuntimeError("No Basler cameras detected.")
    serial = (serial or "").strip()
    if serial:
        for device in devices:
            if device.GetSerialNumber() == serial:
                return device
        attached = ", ".join(d.GetSerialNumber() for d in devices)
        raise RuntimeError(f"Basler camera with serial {serial} not found (attached: {attached}).")
    if len(devices) != 1:
        attached = ", ".join(d.GetSerialNumber() for d in devices)
        raise RuntimeError(
            f"CAMERA_SERIAL is not set and {len(devices)} cameras are attached ({attached}); "
            "set CAMERA_SERIAL so the publisher does not guess.")
    return devices[0]


def require_file(path, label):
    """Return an existing file path; reject the retired 'world' stand-in marker."""
    text = str(path or "").strip()
    if not text or text.lower() == "world":
        raise RuntimeError(
            f"{label}: a model file path is required (the YOLO-World stand-in is not supported "
            "here); copy the real model into rig/assets/.")
    if not Path(text).is_file():
        raise RuntimeError(f"{label}: file not found: {text}")
    return text


def env_int(name, default, minimum=1):
    value = os.environ.get(name, "").strip()
    if not value:
        return default
    try:
        number = int(value)
    except ValueError:
        raise RuntimeError(f"{name} must be an integer, got {value!r}") from None
    if number < minimum:
        raise RuntimeError(f"{name} must be >= {minimum}, got {number}")
    return number


class RateMonitor:
    """Counts events and reports their rate and worst gap every ``period`` seconds."""

    def __init__(self, label, period=5.0, min_hz=None, clock=time.monotonic, log=print):
        self.label, self.period, self.min_hz = label, period, min_hz
        self.clock, self.log = clock, log
        self.window_start = clock()
        self.count = 0
        self.last_event = None
        self.max_gap = 0.0

    def tick(self):
        now = self.clock()
        if self.last_event is not None:
            self.max_gap = max(self.max_gap, now - self.last_event)
        self.last_event = now
        self.count += 1

    def poll(self):
        """Report once per period.  Returns the rate when a report was made, else None."""
        now = self.clock()
        elapsed = now - self.window_start
        if elapsed < self.period:
            return None
        hz = self.count / elapsed
        gap = max(self.max_gap, now - self.last_event if self.last_event is not None else elapsed)
        message = f"[rate] {self.label}: {hz:.1f} Hz, longest gap {gap:.2f} s"
        if self.min_hz is not None and hz < self.min_hz:
            message += (f"  WARNING: below {self.min_hz:.1f} Hz; the bridge treats an input "
                        "older than 0.5 s as lost")
        self.log(message)
        self.window_start, self.count, self.max_gap = now, 0, 0.0
        return hz
