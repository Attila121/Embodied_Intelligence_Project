"""scripts/supervise.sh with real dummy processes (Linux/WSL only)."""

import os
from pathlib import Path
import re
import shutil
import signal
import subprocess
import sys
import tempfile
import time
import unittest

ROOT = Path(__file__).resolve().parents[1]
SUPERVISE = ROOT / "scripts/supervise.sh"


@unittest.skipUnless(sys.platform.startswith("linux") and shutil.which("bash") and shutil.which("ps"),
                     "needs Linux bash and ps")
class SuperviseTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.processes = []
        self.addCleanup(self.reap)

    def reap(self):
        for process in self.processes:
            if process.poll() is None:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait(timeout=5)

    def start(self, body):
        script = Path(self.tmp.name) / "run.sh"
        script.write_text(
            "set -eo pipefail\n"
            f"source '{SUPERVISE}'\n"
            "SUP_INT_WAIT=2 SUP_TERM_WAIT=2\n"
            "supervise_init\n" + body, newline="\n")
        process = subprocess.Popen(["bash", str(script)], stdout=subprocess.PIPE,
                                   stderr=subprocess.STDOUT, text=True, start_new_session=True)
        self.processes.append(process)
        return process

    @staticmethod
    def read_until(process, marker, timeout=15):
        lines, deadline = [], time.monotonic() + timeout
        while time.monotonic() < deadline:
            line = process.stdout.readline()
            if not line:
                break
            lines.append(line)
            if marker in line:
                return lines
        raise AssertionError(f"{marker!r} not seen; output so far: {lines}")

    @staticmethod
    def pids(lines):
        return [int(m.group(1)) for line in lines for m in [re.search(r"\(pid (\d+)\)", line)] if m]

    def assert_gone(self, pids):
        deadline = time.monotonic() + 5
        for pid in pids:
            while time.monotonic() < deadline:
                state = subprocess.run(["ps", "-o", "stat=", "-p", str(pid)],
                                       capture_output=True, text=True).stdout.strip()
                if not state or state.startswith("Z"):
                    break
                time.sleep(0.1)
            else:
                self.fail(f"process {pid} is still running")

    def test_sigterm_escalates_through_int_term_kill_and_leaves_nothing(self):
        # Each stubborn child touches a file only AFTER installing its trap, so the signal
        # can never arrive before the trap exists.
        ready = Path(self.tmp.name) / "ready"
        ready.mkdir()
        process = self.start(
            "start_child polite sleep 300\n"
            f"start_child ignores_int bash -c 'trap \"\" INT; touch {ready}/int; while :; do sleep 1; done'\n"
            f"start_child ignores_all bash -c 'trap \"\" INT TERM; touch {ready}/all; while :; do sleep 1; done'\n"
            "echo READY\n"
            "watch_children\n")
        pids = self.pids(self.read_until(process, "READY"))
        self.assertEqual(len(pids), 3)
        deadline = time.monotonic() + 15
        while time.monotonic() < deadline and not ((ready / "int").exists() and (ready / "all").exists()):
            time.sleep(0.05)
        self.assertTrue((ready / "int").exists() and (ready / "all").exists(), "children never armed")
        os.kill(process.pid, signal.SIGTERM)
        output, _ = process.communicate(timeout=30)
        self.assertEqual(process.returncode, 143, output)
        self.assertIn("sending SIGTERM", output)
        self.assertIn("sending SIGKILL", output)
        self.assert_gone(pids)

    def test_sigint_stops_children_with_130(self):
        process = self.start("start_child a sleep 300\nstart_child b sleep 300\necho READY\nwatch_children\n")
        pids = self.pids(self.read_until(process, "READY"))
        os.kill(process.pid, signal.SIGINT)
        output, _ = process.communicate(timeout=30)
        self.assertEqual(process.returncode, 130, output)
        self.assertNotIn("SIGKILL", output)  # graceful: SIGINT was enough
        self.assert_gone(pids)

    def test_unexpected_child_exit_is_reported_and_everything_stops(self):
        process = self.start("start_child dies bash -c 'sleep 1; exit 3'\n"
                             "start_child other sleep 300\necho READY\nwatch_children\n")
        pids = self.pids(self.read_until(process, "READY"))
        output, _ = process.communicate(timeout=30)
        self.assertEqual(process.returncode, 1, output)
        self.assertIn("child 'dies' exited unexpectedly (status 3)", output)
        self.assert_gone(pids)

    def test_lost_required_process_inside_a_surviving_launcher_is_detected(self):
        # Like `ros2 launch` outliving gzserver: the group leader keeps running, one member dies.
        process = self.start("start_child launcher bash -c 'sleep 311 & exec sleep 312'\n"
                             "sleep 0.5\n"
                             "require_process launcher 'sleep 311'\n"
                             "require_process nobody 'x' || echo UNKNOWN_REJECTED\n"
                             "echo READY\nwatch_children\n")
        lines = self.read_until(process, "READY")
        self.assertTrue(any("UNKNOWN_REJECTED" in line for line in lines), lines)
        pids = self.pids(lines)
        time.sleep(1.5)
        self.assertIsNone(process.poll(), "must keep running while the required process lives")
        subprocess.run(["pkill", "-f", "sleep 311"], check=True)
        output, _ = process.communicate(timeout=30)
        self.assertEqual(process.returncode, 1, output)
        self.assertIn("child 'launcher' lost required process matching 'sleep 311'", output)
        self.assert_gone(pids)

    def test_child_death_aborts_a_running_step(self):
        process = self.start("start_child dies bash -c 'sleep 1; exit 2'\n"
                             "start_child other sleep 300\necho READY\n"
                             "run_guarded 'long step' sleep 60\necho UNREACHABLE\n")
        pids = self.pids(self.read_until(process, "READY"))
        started = time.monotonic()
        output, _ = process.communicate(timeout=30)
        self.assertLess(time.monotonic() - started, 20)
        self.assertEqual(process.returncode, 1, output)
        self.assertIn("aborting during step: long step", output)
        self.assertNotIn("UNREACHABLE", output)
        self.assert_gone(pids)

    def test_run_guarded_returns_step_status(self):
        process = self.start("start_child a sleep 300\n"
                             "run_guarded ok true && echo OK_STEP\n"
                             "run_guarded bad bash -c 'exit 7' || echo BAD_STATUS=$?\n"
                             "echo DONE\n")
        output, _ = process.communicate(timeout=30)
        self.assertEqual(process.returncode, 0, output)
        self.assertIn("OK_STEP", output)
        self.assertIn("BAD_STATUS=7", output)
        self.assertIn("step failed (7): bad", output)
        self.assert_gone(self.pids(output.splitlines()))  # cleaned up on normal exit too


if __name__ == "__main__":
    unittest.main()
