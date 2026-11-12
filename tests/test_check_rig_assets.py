"""Rig asset/serial checker: validates the exact paths it is given."""

import argparse
import contextlib
import io
from pathlib import Path
import sys
import tempfile
import unittest
import zipfile

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))
import check_rig_assets as check

try:
    import numpy as np
except ImportError:
    np = None


def make_pt(path, member="archive/data.pkl"):
    path.parent.mkdir(parents=True, exist_ok=True)
    with zipfile.ZipFile(path, "w") as archive:
        archive.writestr(member, b"x")
    return path


def make_calibration(directory, matrix=None, distortion=None):
    directory.mkdir(parents=True, exist_ok=True)
    matrix = np.array([[900, 0, 640], [0, 900, 480], [0, 0, 1]], float) if matrix is None else matrix
    distortion = np.zeros(5) if distortion is None else distortion
    np.save(directory / "camera_matrix.npy", matrix)
    np.save(directory / "distortion_coefficients.npy", distortion)
    return directory


class ModelFileTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.root = Path(self.tmp.name)

    def test_valid_checkpoint(self):
        self.assertTrue(check.check_model_file("m", make_pt(self.root / "a.pt")).ok)

    def test_missing_required_fails_missing_optional_is_skipped(self):
        self.assertFalse(check.check_model_file("m", self.root / "nope.pt").ok)
        optional = check.check_model_file("m", self.root / "nope.pt", required=False)
        self.assertTrue(optional.ok and optional.skipped)

    def test_incomplete_or_wrong_files_fail(self):
        bad = self.root / "bad.pt"
        bad.write_bytes(b"PK\x03\x04 truncated, no central directory")
        self.assertFalse(check.check_model_file("m", bad).ok)
        text = self.root / "text.pt"
        text.write_text("not a model")
        self.assertFalse(check.check_model_file("m", text).ok)
        self.assertFalse(check.check_model_file("m", make_pt(self.root / "x.pt", "archive/other")).ok)

    def test_present_but_invalid_optional_still_fails(self):
        bad = self.root / "toy.pt"
        bad.write_text("junk")
        result = check.check_model_file("toy", bad, required=False)
        self.assertFalse(result.ok)
        self.assertFalse(check.passed([result]))  # the publisher would crash loading it


class CalibrationTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.root = Path(self.tmp.name)

    def test_missing_files_reported_without_numpy(self):
        result = check.check_calibration("cal", self.root)
        self.assertFalse(result.ok)
        self.assertIn("missing", result.detail)

    @unittest.skipIf(np is None, "numpy required")
    def test_valid_and_invalid_arrays(self):
        self.assertTrue(check.check_calibration("cal", make_calibration(self.root / "ok")).ok)
        bad_shape = make_calibration(self.root / "shape", matrix=np.eye(2))
        nan = make_calibration(self.root / "nan", matrix=np.full((3, 3), np.nan))
        few = make_calibration(self.root / "few", distortion=np.zeros(2))
        zero_focal = make_calibration(self.root / "zero", matrix=np.eye(3) * 0)
        for directory in (bad_shape, nan, few, zero_focal):
            self.assertFalse(check.check_calibration("cal", directory).ok, directory.name)

    @unittest.skipIf(np is None, "numpy required")
    def test_nan_distortion_rejected(self):
        directory = make_calibration(self.root / "d", distortion=np.array([0, 0, np.nan, 0, 0.0]))
        self.assertFalse(check.check_calibration("cal", directory).ok)


class SerialTests(unittest.TestCase):
    def details(self, human, robot, attached):
        return check.check_serials(human, robot, attached)

    def test_unset_serials_fail(self):
        for human, robot in (("", "B"), ("A", ""), (None, None)):
            self.assertFalse(all(r.ok for r in self.details(human, robot, ["A", "B"])))

    def test_identical_serials_fail(self):
        results = self.details("A", "A", ["A", "B"])
        self.assertFalse(all(r.ok for r in results))
        self.assertIn("distinct", results[0].detail)

    def test_unattached_serial_fails(self):
        results = self.details("A", "Z", ["A", "B"])
        self.assertEqual([r.ok for r in results], [True, False])

    def test_two_distinct_attached_serials_pass(self):
        self.assertTrue(all(r.ok for r in self.details("A", "B", ["A", "B", "C"])))


class RunChecksTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.root = Path(self.tmp.name)

    def args(self, **overrides):
        values = dict(
            human_pose=str(self.root / "o/pose.pt"), human_cap=str(self.root / "o/cap.pt"),
            human_calib=str(self.root / "o/hcal"), robot_model=str(self.root / "o/robot.pt"),
            robot_toy="", robot_calib=str(self.root / "o/rcal"),
            human_serial="A", robot_serial="B", skip_hardware=True)
        values.update(overrides)
        return argparse.Namespace(**values)

    def test_empty_assets_fail_and_list_every_missing_required_item(self):
        results = check.run_checks(self.args())
        self.assertFalse(check.passed(results))
        failed = {r.label for r in results if not r.ok}
        self.assertEqual(failed, {"human pose model", "human cap model", "human calibration",
                                  "robot model", "robot calibration"})

    @unittest.skipIf(np is None, "numpy required")
    def test_override_paths_are_the_ones_checked(self):
        # Valid files live in a custom location; default rig/assets/ is irrelevant.
        for name in ("pose.pt", "cap.pt", "robot.pt"):
            make_pt(self.root / "o" / name)
        make_calibration(self.root / "o/hcal")
        make_calibration(self.root / "o/rcal")
        results = check.run_checks(self.args())
        self.assertTrue(check.passed(results), [r for r in results if not r.ok])
        self.assertTrue(next(r for r in results if r.label.startswith("toy")).skipped)

    @unittest.skipIf(np is None, "numpy required")
    def test_hardware_checks_use_serial_rules(self):
        for name in ("pose.pt", "cap.pt", "robot.pt"):
            make_pt(self.root / "o" / name)
        make_calibration(self.root / "o/hcal")
        make_calibration(self.root / "o/rcal")
        ok = lambda module: check.Result(f"python module {module}", True, True, "importable")
        good = check.run_checks(self.args(skip_hardware=False), lambda: ["A", "B"], ok)
        self.assertTrue(check.passed(good))
        same = check.run_checks(self.args(skip_hardware=False, robot_serial="A"), lambda: ["A", "B"], ok)
        self.assertFalse(check.passed(same))
        gone = check.run_checks(self.args(skip_hardware=False), lambda: ["A"], ok)
        self.assertFalse(check.passed(gone))

    def test_missing_pypylon_blocks_serial_check(self):
        def importer(module):
            return check.Result(f"python module {module}", module != "pypylon", True, "x")

        results = check.run_checks(self.args(skip_hardware=False), lambda: ["A", "B"], importer)
        self.assertFalse(check.passed(results))
        self.assertTrue(any(r.label == "camera serials" and not r.ok for r in results))

    def test_main_exit_codes(self):
        argv = ["--human-pose", str(self.root / "p.pt"), "--human-cap", str(self.root / "c.pt"),
                "--human-calib", str(self.root / "h"), "--robot-model", str(self.root / "r.pt"),
                "--robot-calib", str(self.root / "rc"), "--skip-hardware"]
        with contextlib.redirect_stdout(io.StringIO()), contextlib.redirect_stderr(io.StringIO()):
            self.assertEqual(check.main(argv), 1)


if __name__ == "__main__":
    unittest.main()
