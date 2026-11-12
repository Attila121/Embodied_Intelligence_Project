"""Settle/stop decision logic of the simulation demo monitor (no ROS needed)."""

from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))
from demo_monitor import SettleTracker, StopTracker, parse


class SettleTrackerTests(unittest.TestCase):
    def test_requires_the_distance_to_hold_for_the_full_duration(self):
        tracker = SettleTracker(0.9, 1.2, hold=3.0)
        self.assertFalse(tracker.update(0.0, 1.05, 0.0))
        self.assertFalse(tracker.update(2.9, 1.05, 0.0))
        self.assertTrue(tracker.update(3.0, 1.05, 0.0))

    def test_leaving_the_window_restarts_the_hold(self):
        tracker = SettleTracker(0.9, 1.2, hold=3.0)
        tracker.update(0.0, 1.0, 0.0)
        self.assertFalse(tracker.update(2.0, 1.6, 0.0))  # drifted away
        self.assertFalse(tracker.update(4.0, 1.0, 0.0))  # restarted at t=4
        self.assertFalse(tracker.update(6.9, 1.0, 0.0))
        self.assertTrue(tracker.update(7.0, 1.0, 0.0))

    def test_too_close_is_not_settled(self):
        self.assertFalse(SettleTracker(0.9, 1.2, hold=0.0).update(0.0, 0.5, 0.0))

    def test_stale_or_missing_samples_never_settle(self):
        tracker = SettleTracker(0.9, 1.2, hold=1.0, max_age=0.5)
        tracker.update(0.0, 1.0, 0.0)
        self.assertFalse(tracker.update(5.0, 1.0, 2.0))  # old sample (robot lost the owner)
        self.assertFalse(tracker.update(6.0, None, 0.0))
        self.assertFalse(tracker.update(7.0, 1.0, 0.0))  # hold restarted


class StopTrackerTests(unittest.TestCase):
    def update(self, tracker, now, visible=False, vis_age=0.1, command=(0.0, 0.0), cmd_age=0.1):
        return tracker.update(now, visible, vis_age, command, cmd_age)

    def test_needs_invisible_and_zero_velocity_held(self):
        tracker = StopTracker(hold=1.0)
        self.assertFalse(self.update(tracker, 0.0))
        self.assertFalse(self.update(tracker, 0.9))
        self.assertTrue(self.update(tracker, 1.0))

    def test_visible_owner_is_never_stopped(self):
        self.assertFalse(self.update(StopTracker(hold=0.0), 0.0, visible=True))
        self.assertFalse(self.update(StopTracker(hold=0.0), 0.0, visible=None))

    def test_any_motion_restarts_the_hold(self):
        tracker = StopTracker(hold=1.0)
        self.update(tracker, 0.0)
        self.assertFalse(self.update(tracker, 0.8, command=(0.05, 0.0)))
        self.assertFalse(self.update(tracker, 1.5, command=(0.0, -0.2)))
        self.assertFalse(self.update(tracker, 2.0))
        self.assertTrue(self.update(tracker, 3.0))

    def test_stale_command_or_visibility_does_not_count(self):
        self.assertFalse(self.update(StopTracker(hold=0.0), 0.0, cmd_age=2.0))
        self.assertFalse(self.update(StopTracker(hold=0.0), 0.0, vis_age=5.0))
        self.assertFalse(self.update(StopTracker(hold=0.0), 0.0, command=None))


class ArgumentTests(unittest.TestCase):
    def test_defaults_and_validation(self):
        args = parse(["wait-settled"])
        self.assertEqual((args.low, args.high, args.hold), (0.90, 1.20, 3.0))
        for bad in (["wait-visible", "--timeout", "0"], ["wait-visible", "--timeout", "nan"]):
            with self.assertRaises(SystemExit):
                parse(bad)


if __name__ == "__main__":
    unittest.main()
