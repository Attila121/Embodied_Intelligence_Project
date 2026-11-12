"""Camera acquisition, device selection and rate monitoring; no camera or pypylon needed."""

import os
from pathlib import Path
import sys
import tempfile
import unittest
from unittest import mock

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))
from rig_capture import RateMonitor, acquire, choose_device, env_int, require_file


class FakeArray:
    def __init__(self, tag):
        self.tag = tag

    def GetArray(self):
        return self.tag


class FakeGrab:
    def __init__(self, succeeded=True, valid=True, tag="frame"):
        self.succeeded, self.valid, self.tag = succeeded, valid, tag
        self.released = 0

    def IsValid(self):
        return self.valid

    def GrabSucceeded(self):
        return self.succeeded

    def Release(self):
        self.released += 1


class FakeConverter:
    def __init__(self, fail=False):
        self.fail = fail

    def Convert(self, grab):
        if self.fail:
            raise RuntimeError("convert failed")
        return FakeArray(grab.tag)


class FakeCamera:
    """Yields scripted results; an Exception instance is raised instead of returned."""

    def __init__(self, results):
        self.results = list(results)
        self.grabs = [r for r in results if isinstance(r, FakeGrab)]

    def RetrieveResult(self, timeout_ms, handling):
        result = self.results.pop(0)
        if isinstance(result, Exception):
            raise result
        return result


class AcquireTests(unittest.TestCase):
    def test_success_returns_frame_and_releases(self):
        camera = FakeCamera([FakeGrab(tag="a")])
        self.assertEqual(acquire(camera, FakeConverter(), "return"), "a")
        self.assertEqual(camera.grabs[0].released, 1)

    def test_failed_grab_returns_none_and_releases(self):
        camera = FakeCamera([FakeGrab(succeeded=False)])
        self.assertIsNone(acquire(camera, FakeConverter(), "return"))
        self.assertEqual(camera.grabs[0].released, 1)

    def test_invalid_result_returns_none_and_releases(self):
        camera = FakeCamera([FakeGrab(valid=False)])
        self.assertIsNone(acquire(camera, FakeConverter(), "return"))
        self.assertEqual(camera.grabs[0].released, 1)

    def test_convert_error_returns_none_and_releases(self):
        camera = FakeCamera([FakeGrab()])
        self.assertIsNone(acquire(camera, FakeConverter(fail=True), "return"))
        self.assertEqual(camera.grabs[0].released, 1)

    def test_timeout_exception_and_none_result_give_no_frame(self):
        self.assertIsNone(acquire(FakeCamera([TimeoutError("t")]), FakeConverter(), "return"))
        self.assertIsNone(acquire(FakeCamera([None]), FakeConverter(), "return"))

    def test_failure_after_success_never_returns_the_old_frame(self):
        camera = FakeCamera([FakeGrab(tag="first"), FakeGrab(succeeded=False), TimeoutError("t")])
        frames = [acquire(camera, FakeConverter(), "return") for _ in range(3)]
        self.assertEqual(frames, ["first", None, None])
        self.assertTrue(all(g.released == 1 for g in camera.grabs))


class FakeDevice:
    def __init__(self, serial):
        self.serial = serial

    def GetSerialNumber(self):
        return self.serial


class ChooseDeviceTests(unittest.TestCase):
    def test_serial_selects_matching_camera(self):
        devices = [FakeDevice("A"), FakeDevice("B")]
        self.assertIs(choose_device(devices, "B"), devices[1])

    def test_unknown_serial_fails(self):
        with self.assertRaisesRegex(RuntimeError, "not found"):
            choose_device([FakeDevice("A")], "Z")

    def test_unset_serial_is_refused_with_several_cameras(self):
        for serial in (None, "", "  "):
            with self.assertRaisesRegex(RuntimeError, "CAMERA_SERIAL"):
                choose_device([FakeDevice("A"), FakeDevice("B")], serial)

    def test_unset_serial_allowed_for_a_single_camera(self):
        only = FakeDevice("A")
        self.assertIs(choose_device([only], None), only)

    def test_no_cameras_fails(self):
        with self.assertRaisesRegex(RuntimeError, "No Basler"):
            choose_device([], "A")


class RequireFileTests(unittest.TestCase):
    def test_world_marker_and_empty_are_rejected(self):
        for value in ("world", "WORLD", "", None):
            with self.assertRaisesRegex(RuntimeError, "model file path is required"):
                require_file(value, "CAP_MODEL")

    def test_missing_file_rejected_and_existing_accepted(self):
        with tempfile.TemporaryDirectory() as directory:
            existing = Path(directory) / "cap.pt"
            existing.write_bytes(b"x")
            self.assertEqual(require_file(existing, "CAP_MODEL"), str(existing))
            with self.assertRaisesRegex(RuntimeError, "file not found"):
                require_file(Path(directory) / "other.pt", "CAP_MODEL")


class EnvIntTests(unittest.TestCase):
    def test_default_value_and_minimum(self):
        with mock.patch.dict(os.environ, {}, clear=False):
            os.environ.pop("RIG_TEST_N", None)
            self.assertEqual(env_int("RIG_TEST_N", 3), 3)
        with mock.patch.dict(os.environ, {"RIG_TEST_N": "5"}):
            self.assertEqual(env_int("RIG_TEST_N", 3), 5)
        for bad in ("0", "abc", "-1"):
            with mock.patch.dict(os.environ, {"RIG_TEST_N": bad}):
                with self.assertRaises(RuntimeError):
                    env_int("RIG_TEST_N", 3)


class RateMonitorTests(unittest.TestCase):
    def setUp(self):
        self.now = 100.0
        self.lines = []
        self.make = lambda **kw: RateMonitor("x", clock=lambda: self.now, log=self.lines.append, **kw)

    def test_no_report_before_period(self):
        monitor = self.make(period=5.0)
        monitor.tick()
        self.now += 4.9
        self.assertIsNone(monitor.poll())
        self.assertEqual(self.lines, [])

    def test_reports_rate_and_resets_window(self):
        monitor = self.make(period=5.0, min_hz=4.0)
        for _ in range(25):
            monitor.tick()
            self.now += 0.2
        self.assertAlmostEqual(monitor.poll(), 5.0)
        self.assertNotIn("WARNING", self.lines[-1])
        self.assertIsNone(monitor.poll())  # window restarted

    def test_warns_below_minimum_rate(self):
        monitor = self.make(period=5.0, min_hz=4.0)
        for _ in range(5):
            monitor.tick()
            self.now += 1.0
        self.assertAlmostEqual(monitor.poll(), 1.0)
        self.assertIn("WARNING", self.lines[-1])

    def test_no_events_reports_zero(self):
        monitor = self.make(period=5.0, min_hz=4.0)
        self.now += 5.0
        self.assertEqual(monitor.poll(), 0.0)
        self.assertIn("WARNING", self.lines[-1])


if __name__ == "__main__":
    unittest.main()
