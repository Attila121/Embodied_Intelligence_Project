"""The obstacle_avoider node with synthetic topics (ROS Humble; process-specific remaps)."""

import importlib.util
import math
import os
from pathlib import Path
import sys
import time
import unittest

if importlib.util.find_spec("rclpy") is None:
    raise unittest.SkipTest("source ROS Humble to run the avoider node tests")

import rclpy
from geometry_msgs.msg import Point, Twist
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))
from obstacle_avoider import ObstacleAvoider


def scan_message(obstacles=(), frame="base_scan", poison=None):
    """360 one-degree rays; ``obstacles`` are (x, y) points in the sensor frame, hit as thin posts."""
    ranges = [math.inf] * 360
    for x, y in obstacles:
        ranges[round(math.degrees(math.atan2(y, x))) % 360] = math.hypot(x, y)
    if poison is not None:
        ranges[10] = poison
    message = LaserScan()
    message.header.frame_id = frame
    message.angle_min, message.angle_max = 0.0, 2 * math.pi
    message.angle_increment = math.radians(1.0)
    message.range_min, message.range_max = 0.12, 3.5
    message.ranges = ranges
    return message


class ObstacleAvoiderNodeTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        prefix = f"/avoider_test_{os.getpid()}"
        arguments = ["--ros-args", "--params-file", str(ROOT / "config/obstacle_avoidance.yaml")]
        for topic in ("following/desired_cmd_vel", "following/cmd_vel", "owner/relative_position",
                      "owner/visible", "scan"):
            arguments.extend(["-r", f"/{topic}:={prefix}/{topic}"])
        arguments.extend(["-r", f"__ns:={prefix}"])
        rclpy.init(args=arguments)
        cls.prefix = prefix

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.avoider = ObstacleAvoider()
        self.probe = Node("avoider_probe")
        self.executor = SingleThreadedExecutor()
        for node in (self.avoider, self.probe):
            self.executor.add_node(node)
        self.addCleanup(self.cleanup)
        self.desired = self.probe.create_publisher(Twist, "/following/desired_cmd_vel", 1)
        self.owner = self.probe.create_publisher(Point, "/owner/relative_position", 1)
        self.visible = self.probe.create_publisher(Bool, "/owner/visible", 1)
        self.scan = self.probe.create_publisher(
            LaserScan, "/scan", QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.commands = []
        self.probe.create_subscription(
            Twist, "/following/cmd_vel",
            lambda m: self.commands.append((time.monotonic(), m.linear.x, m.angular.z)), 1)
        self.pump(0.6)  # discovery

    def cleanup(self):
        for node in (self.avoider, self.probe):
            self.executor.remove_node(node)
            node.destroy_node()

    def pump(self, seconds):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            self.executor.spin_once(timeout_sec=0.02)

    def drive(self, seconds, scan=None, owner=(2.5, 0.0), desired=(0.12, 0.0), send_desired=True):
        """Publish inputs at ~20 Hz for ``seconds``; return the commands received in that period."""
        start = len(self.commands)
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            if send_desired:
                message = Twist()
                message.linear.x, message.angular.z = desired
                self.desired.publish(message)
            if owner is not None:
                self.owner.publish(Point(x=owner[0], y=owner[1]))
                self.visible.publish(Bool(data=True))
            if scan is not None:
                self.scan.publish(scan)
            self.pump(0.05)
        return self.commands[start:]

    def last(self, commands):
        self.assertTrue(commands, "no command received")
        return commands[-1][1:]

    def test_without_a_scan_the_output_is_zero(self):
        self.assertEqual(self.last(self.drive(0.8, scan=None)), (0.0, 0.0))

    def test_free_space_passes_the_follower_command_through(self):
        linear, angular = self.last(self.drive(1.0, scan=scan_message()))
        self.assertAlmostEqual(linear, 0.12)
        self.assertEqual(angular, 0.0)

    def test_obstacle_on_the_path_turns_instead_of_driving(self):
        linear, angular = self.last(self.drive(1.0, scan=scan_message([(1.0, 0.0)])))
        self.assertEqual(linear, 0.0)
        self.assertNotEqual(angular, 0.0)
        self.assertLessEqual(abs(angular), 0.5)

    def test_bad_scans_stop_the_robot(self):
        self.drive(0.6, scan=scan_message())
        for label, scan in (("wrong frame", scan_message(frame="odom")),
                            ("NaN reading", scan_message(poison=math.nan)),
                            ("reading below range_min", scan_message(poison=0.05))):
            with self.subTest(label):
                self.assertEqual(self.last(self.drive(1.2, scan=scan)), (0.0, 0.0))
                self.drive(0.6, scan=scan_message())  # recover with a good scan
                self.assertGreater(self.last(self.drive(0.6, scan=scan_message()))[0], 0.0)

    def test_stale_follower_command_stops_the_robot(self):
        self.assertGreater(self.last(self.drive(0.8, scan=scan_message()))[0], 0.0)
        # Follower goes silent; scan and owner keep coming.
        self.assertEqual(self.last(self.drive(1.0, scan=scan_message(), send_desired=False)), (0.0, 0.0))

    def test_owner_loss_stops_the_robot(self):
        self.assertGreater(self.last(self.drive(0.8, scan=scan_message()))[0], 0.0)
        self.assertEqual(self.last(self.drive(1.2, scan=scan_message(), owner=None)), (0.0, 0.0))

    def test_stopped_follower_is_never_overridden(self):
        commands = self.drive(1.0, scan=scan_message([(1.0, 0.0)]), desired=(0.0, 0.0))
        self.assertTrue(all(c[1:] == (0.0, 0.0) for c in commands))

    def test_invalid_parameters_are_refused(self):
        from avoidance import AvoidParams, validate_params

        for override in ({"max_linear": 0.5}, {"clearance": 0.4}, {"rotate_exit": 0.9}, {"lookahead": math.nan}):
            params = AvoidParams(**{**AvoidParams().__dict__, **override})
            with self.assertRaises(ValueError, msg=str(override)):
                validate_params(params)


if __name__ == "__main__":
    unittest.main()
