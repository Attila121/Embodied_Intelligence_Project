"""scripts/run_rig.sh: configuration refusal, exact-path checking and the per-process
environment contract, using fake python/ros2 executables (Linux/WSL only; no ROS needed)."""

import os
from pathlib import Path
import shutil
import signal
import subprocess
import sys
import tempfile
import time
import unittest
import zipfile

ROOT = Path(__file__).resolve().parents[1]
SCRIPT = ROOT / "scripts/run_rig.sh"

FAKE_PYTHON = """#!/usr/bin/env bash
case "$(basename "$1")" in
  check_rig_assets.py) exit 0 ;;
  *) echo "CHILD $(basename "$1") CAMERA_SERIAL=$CAMERA_SERIAL CALIB_DIR=$CALIB_DIR" \\
          "POSE_MODEL=${POSE_MODEL:-} CAP_MODEL=${CAP_MODEL:-} ROBOT_MODEL=${ROBOT_MODEL:-}" \\
          "TOY_MODEL=${TOY_MODEL:-} DOMAIN=$ROS_DOMAIN_ID LOCAL=$ROS_LOCALHOST_ONLY"
     exec sleep 300 ;;
esac
"""
FAKE_ROS2 = """#!/usr/bin/env bash
echo "CHILD ros2 $* DOMAIN=$ROS_DOMAIN_ID LOCAL=$ROS_LOCALHOST_ONLY"
exec sleep 300
"""


@unittest.skipUnless(sys.platform.startswith("linux") and shutil.which("bash"), "needs Linux bash")
class RunRigTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.dir = Path(self.tmp.name)
        self.processes = []
        self.addCleanup(self.reap)
        (self.dir / "ros_setup.bash").write_text("", newline="\n")
        for name, body in (("fakepython", FAKE_PYTHON), ("ros2", FAKE_ROS2)):
            path = self.dir / name
            path.write_text(body, newline="\n")
            path.chmod(0o755)

    def reap(self):
        for process in self.processes:
            if process.poll() is None:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait(timeout=5)

    def env_file(self, **values):
        base = {"RIG_ROS_DOMAIN_ID": "42", "RIG_LOCALHOST_ONLY": "1",
                "HUMAN_CAMERA_SERIAL": "HUM1", "ROBOT_CAMERA_SERIAL": "ROB2"}
        base.update(values)
        path = self.dir / "rig.env"
        path.write_text("".join(f"{k}={v}\r\n" for k, v in base.items() if v is not None), newline="")
        return path

    def run_rig(self, *args, env_file=None, extra_env=None, timeout=30, start=False):
        env = {**os.environ, "RIG_ENV": str(env_file or self.env_file()),
               "RIG_ROS_SETUP": str(self.dir / "ros_setup.bash"),
               "PATH": f"{self.dir}:{os.environ['PATH']}", "DISPLAY": ":0",
               "ROS_DOMAIN_ID": "30"}  # the simulation's value must not leak into the rig
        env.update(extra_env or {})
        process = subprocess.Popen(["bash", str(SCRIPT), *args], stdout=subprocess.PIPE,
                                   stderr=subprocess.STDOUT, text=True, env=env,
                                   start_new_session=True)
        self.processes.append(process)
        if start:
            return process
        output, _ = process.communicate(timeout=timeout)
        return process.returncode, output

    def test_missing_env_file(self):
        code, output = self.run_rig(env_file=self.dir / "absent.env")
        self.assertEqual(code, 2)
        self.assertIn("rig.env.example", output)

    def test_refuses_bad_domain_and_serials(self):
        for overrides, text in (({"RIG_ROS_DOMAIN_ID": "30"}, "simulation"),
                                ({"RIG_ROS_DOMAIN_ID": None}, "RIG_ROS_DOMAIN_ID"),
                                ({"RIG_ROS_DOMAIN_ID": "abc"}, "RIG_ROS_DOMAIN_ID"),
                                ({"RIG_ROS_DOMAIN_ID": "300"}, "RIG_ROS_DOMAIN_ID"),
                                ({"RIG_LOCALHOST_ONLY": "2"}, "RIG_LOCALHOST_ONLY"),
                                ({"HUMAN_CAMERA_SERIAL": ""}, "both"),
                                ({"ROBOT_CAMERA_SERIAL": None}, "both"),
                                ({"ROBOT_CAMERA_SERIAL": "HUM1"}, "must differ")):
            code, output = self.run_rig(env_file=self.env_file(**overrides))
            self.assertEqual(code, 2, (overrides, output))
            self.assertIn(text, output, overrides)

    def test_missing_explicit_toy_override_is_an_error(self):
        code, output = self.run_rig(env_file=self.env_file(ROBOT_TOY_MODEL=self.dir / "typo.pt"))
        self.assertEqual(code, 2)
        self.assertIn("ROBOT_TOY_MODEL", output)

    def test_check_with_empty_assets_fails_in_the_real_checker_and_lists_items(self):
        code, output = self.run_rig("--check", extra_env={"RIG_PYTHON": sys.executable})
        self.assertEqual(code, 1, output)
        for label in ("human pose model", "human cap model", "robot model", "robot calibration"):
            self.assertIn(label, output)
        self.assertNotIn("Checks passed", output)

    def test_checker_receives_the_override_paths(self):
        pt = self.dir / "custom_cap.pt"
        with zipfile.ZipFile(pt, "w") as archive:
            archive.writestr("a/data.pkl", b"x")
        code, output = self.run_rig("--check", env_file=self.env_file(HUMAN_CAP_MODEL=pt),
                                    extra_env={"RIG_PYTHON": sys.executable})
        self.assertEqual(code, 1)  # other items are still missing
        self.assertRegex(output, rf"OK\s+human cap model: {pt}")

    def test_check_passes_with_a_passing_checker_and_starts_nothing(self):
        code, output = self.run_rig("--check", extra_env={"RIG_PYTHON": str(self.dir / "fakepython")})
        self.assertEqual(code, 0, output)
        self.assertIn("nothing started", output)
        self.assertNotIn("CHILD", output)

    def test_each_publisher_gets_its_own_serial_calibration_and_the_rig_domain(self):
        env_file = self.env_file(HUMAN_CALIB_DIR="/cal/human", ROBOT_CALIB_DIR="/cal/robot",
                                 HUMAN_POSE_MODEL="/m/pose.pt", HUMAN_CAP_MODEL="/m/cap.pt",
                                 ROBOT_MODEL="/m/robot.pt")
        process = self.run_rig(env_file=env_file, start=True,
                               extra_env={"RIG_PYTHON": str(self.dir / "fakepython")})
        lines, deadline = [], time.monotonic() + 20
        while time.monotonic() < deadline and sum("CHILD" in line for line in lines) < 3:
            lines.append(process.stdout.readline())
        children = [line for line in lines if line.startswith("CHILD")]
        self.assertEqual(len(children), 3, lines)
        human = next(line for line in children if "rig_human_publisher.py" in line)
        robot = next(line for line in children if "rig_robot_publisher.py" in line)
        launch = next(line for line in children if line.startswith("CHILD ros2"))
        self.assertIn("CAMERA_SERIAL=HUM1 CALIB_DIR=/cal/human POSE_MODEL=/m/pose.pt CAP_MODEL=/m/cap.pt", human)
        self.assertIn("CAMERA_SERIAL=ROB2 CALIB_DIR=/cal/robot", robot)
        self.assertIn("ROBOT_MODEL=/m/robot.pt", robot)
        self.assertIn("TOY_MODEL=" + str(ROOT / "rig/assets/robot/toy.pt"), robot)
        for line in children:  # rig domain, never the simulation's 30
            self.assertIn("DOMAIN=42", line)
            self.assertIn("LOCAL=1", line)
        self.assertIn("source:=rig", launch)
        self.assertIn("drive:=false", launch)
        os.kill(process.pid, signal.SIGTERM)
        output, _ = process.communicate(timeout=30)
        self.assertEqual(process.returncode, 143, output)


if __name__ == "__main__":
    unittest.main()
