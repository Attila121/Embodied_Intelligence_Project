"""Behavior tests: run after sourcing ROS Humble, without a running ROS graph."""

import math
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))
from following_guard import CommandGate, scan_problem
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan


class FollowingGuardTests(unittest.TestCase):
    def setUp(self):
        self.now = 100.0
        self.gate = CommandGate(clock=lambda: self.now)

    def scan(self):
        message = LaserScan()
        message.header.frame_id = "base_scan"
        message.angle_increment = math.tau / 360
        message.range_min, message.range_max = 0.12, 3.5
        message.ranges = [3.0] * 360
        return message

    def command(self, linear=0.12, angular=0.2):
        message = Twist()
        message.linear.x, message.angular.z = linear, angular
        return message

    def ready(self):
        self.gate.receive_scan(self.scan())
        self.gate.receive_command(self.command())
        self.now += 0.1
        return self.gate.evaluate()

    def assert_stopped(self, expected):
        linear, angular, reason = self.gate.evaluate()
        self.assertEqual((linear, angular), (0.0, 0.0))
        self.assertIn(expected, reason)

    def test_no_input_stays_stopped(self):
        self.assert_stopped("waiting for controller")

    def test_command_without_scan_stays_stopped(self):
        self.gate.receive_command(self.command())
        self.assert_stopped("waiting for scan")

    def test_clear_scan_allows_acceleration_limited_motion(self):
        linear, angular, reason = self.ready()
        self.assertAlmostEqual(linear, 0.02)
        self.assertAlmostEqual(angular, 0.1)
        self.assertEqual(reason, "following")

    def test_controller_loss_stops_while_scan_is_still_fresh(self):
        self.ready()
        self.now += 0.31
        self.gate.receive_scan(self.scan())
        self.assert_stopped("controller timeout")

    def test_scan_loss_stops_while_commands_continue(self):
        self.ready()
        self.now += 0.61
        self.gate.receive_command(self.command())
        self.assert_stopped("scan timeout")

    def test_turning_stops_for_obstacle_behind_robot(self):
        self.ready()
        scan = self.scan()
        scan.ranges[180] = 0.4
        self.gate.receive_scan(scan)
        self.assert_stopped("obstacle")

    def test_straight_motion_can_depart_from_rear_wall(self):
        self.ready()
        scan = self.scan()
        scan.ranges[151] = 0.336  # Regression: measured rear-left return.
        self.gate.receive_scan(scan)
        self.gate.receive_command(self.command(0.12, 0.0))
        self.now += 0.05
        linear, angular, reason = self.gate.evaluate()
        self.assertGreater(linear, 0.0)
        self.assertEqual(angular, 0.0)
        self.assertEqual(reason, "following")

    def test_wall_beside_corridor_does_not_block_straight_motion(self):
        self.ready()
        scan = self.scan()
        scan.ranges[90] = 0.415  # Regression: measured left-side return.
        self.gate.receive_scan(scan)
        self.gate.receive_command(self.command(0.12, 0.0))
        self.now += 0.05
        self.assertGreater(self.gate.evaluate()[0], 0.0)

    def test_front_obstacle_blocks_straight_motion(self):
        self.ready()
        scan = self.scan()
        scan.ranges[0] = 0.4
        self.gate.receive_scan(scan)
        self.gate.receive_command(self.command(0.12, 0.0))
        self.assert_stopped("forward path")

    def test_obstacle_at_edge_of_forward_corridor_blocks_motion(self):
        self.ready()
        scan = self.scan()
        scan.ranges[27] = 0.6  # x=0.535, y=0.272: inside padded corridor.
        self.gate.receive_scan(scan)
        self.gate.receive_command(self.command(0.12, 0.0))
        self.assert_stopped("forward path")

    def test_straight_motion_still_stops_if_rear_obstacle_touches_margin(self):
        self.ready()
        scan = self.scan()
        scan.ranges[180] = 0.25
        self.gate.receive_scan(scan)
        self.gate.receive_command(self.command(0.12, 0.0))
        self.assert_stopped("footprint")

    def test_clearance_is_rechecked_when_motion_changes_without_new_scan(self):
        self.ready()
        scan = self.scan()
        scan.ranges[151] = 0.336
        self.gate.receive_scan(scan)
        self.gate.receive_command(self.command(0.12, 0.0))
        self.now += 0.05
        self.assertGreater(self.gate.evaluate()[0], 0.0)
        self.gate.receive_command(self.command(0.12, 0.2))
        self.assert_stopped("turn blocked")

    def test_front_path_uses_scan_angle_min_and_negative_increments(self):
        scan = self.scan()
        scan.angle_min = math.pi
        scan.angle_increment = -scan.angle_increment
        scan.ranges[180] = 0.4  # This index now points forward.
        self.assertIn("forward path", scan_problem(
            scan, 0.55, "base_scan", forward_only=True))

    def test_emergency_stop_bypasses_ramp(self):
        self.ready()
        self.gate.receive_command(self.command(0.0, 0.0))
        self.assert_stopped("controller stopped")

    def test_positive_infinity_is_clear_but_invalid_returns_block(self):
        scan = self.scan()
        scan.ranges = [math.inf] * 360
        self.assertIsNone(scan_problem(scan, 0.55, "base_scan"))
        for invalid in (math.nan, -math.inf, 0.0, 4.0):
            scan.ranges[0] = invalid
            self.assertIsNotNone(scan_problem(scan, 0.55, "base_scan"))

    def test_wrong_frame_and_partial_coverage_block_motion(self):
        scan = self.scan()
        scan.header.frame_id = "camera"
        self.assertIn("frame", scan_problem(scan, 0.55, "base_scan"))
        scan.header.frame_id = "base_scan"
        scan.ranges = scan.ranges[:180]
        self.assertIn("coverage", scan_problem(scan, 0.55, "base_scan"))

    def test_invalid_metadata_blocks_motion(self):
        for field, invalid in (("angle_increment", 0.0),
                               ("angle_increment", math.nan),
                               ("range_max", 0.4), ("range_min", -1.0)):
            scan = self.scan()
            setattr(scan, field, invalid)
            self.assertIsNotNone(scan_problem(scan, 0.55, "base_scan"))

    def test_reverse_and_nonfinite_commands_block_motion(self):
        self.ready()
        for value in (-0.1, math.inf, math.nan):
            self.gate.receive_command(self.command(value))
            self.assert_stopped("invalid velocity")

    def test_sideways_commands_are_rejected(self):
        self.ready()
        command = self.command()
        command.linear.y = 0.1
        self.gate.receive_command(command)
        self.assert_stopped("invalid velocity")

    def test_velocity_caps_hold_during_continuous_updates(self):
        for _ in range(30):
            self.gate.receive_scan(self.scan())
            self.gate.receive_command(self.command(10.0, -10.0))
            self.now += 0.05
            linear, angular, _ = self.gate.evaluate()
            self.assertTrue(0.0 <= linear <= 0.12)
            self.assertTrue(-0.5 <= angular <= 0.5)
        self.assertEqual((linear, angular), (0.12, -0.5))

    def test_fresh_clear_scan_can_resume_after_obstacle(self):
        self.ready()
        scan = self.scan()
        scan.ranges[0] = 0.3
        self.gate.receive_scan(scan)
        self.assert_stopped("obstacle")
        self.now += 0.05
        self.gate.receive_scan(self.scan())
        self.gate.receive_command(self.command())
        linear, angular, reason = self.gate.evaluate()
        self.assertGreater(linear, 0.0)
        self.assertEqual(reason, "following")


if __name__ == "__main__":
    unittest.main()
