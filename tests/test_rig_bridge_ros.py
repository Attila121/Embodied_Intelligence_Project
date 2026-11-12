"""Synthetic Pose2D replay through the real bridge and follower (ROS Humble).

Every topic is remapped to a process-specific namespace; no /cmd_vel is used.
"""

import importlib.util
import math
import os
from pathlib import Path
import sys
import time
import unittest

if importlib.util.find_spec("rclpy") is None:
    raise unittest.SkipTest("source ROS Humble to run the bridge/follower replay")

import rclpy
from geometry_msgs.msg import Point, Pose2D, Twist
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import Bool

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))
from owner_follower import OwnerFollower
from rig_bridge import RigBridge


class RigBridgeReplayTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        prefix = f"/rig_bridge_test_{os.getpid()}"
        arguments = ["--ros-args", "--params-file", str(ROOT / "config/rig_bridge.yaml")]
        for topic in ("owner_position", "robot_position", "owner/visible",
                      "owner/relative_position", "following/cmd_vel"):
            arguments.extend(["-r", f"/{topic}:={prefix}/{topic}"])
        arguments.extend(["-r", f"__ns:={prefix}"])
        rclpy.init(args=arguments)

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.bridge = RigBridge()
        self.follower = OwnerFollower()
        self.probe = Node("replay_probe")
        self.executor = SingleThreadedExecutor()
        self.addCleanup(self.cleanup_nodes)
        for node in (self.bridge, self.follower, self.probe):
            self.executor.add_node(node)
        self.owner_pub = self.probe.create_publisher(Pose2D, "/owner_position", 1)
        self.robot_pub = self.probe.create_publisher(Pose2D, "/robot_position", 1)
        self.positions, self.visibility, self.commands = [], [], []
        self.subscriptions = [
            self.probe.create_subscription(Point, "/owner/relative_position",
                lambda m: self.positions.append((time.monotonic(), m.x, m.y)), 1),
            self.probe.create_subscription(Bool, "/owner/visible",
                lambda m: self.visibility.append((time.monotonic(), m.data)), 1),
            self.probe.create_subscription(Twist, "/following/cmd_vel",
                lambda m: self.commands.append((time.monotonic(), m.linear.x, m.angular.z)), 1),
        ]
        deadline = time.monotonic() + 5
        while time.monotonic() < deadline:
            if (self.owner_pub.get_subscription_count() and self.robot_pub.get_subscription_count()
                    and all(self.probe.count_publishers(s.topic_name) for s in self.subscriptions)):
                break
            self.executor.spin_once(timeout_sec=0.02)
        else:
            self.fail("ROS endpoints did not discover each other")

    def cleanup_nodes(self):
        self.executor.shutdown()
        for node in (self.probe, self.follower, self.bridge):
            node.destroy_node()

    def replay(self, seconds, owner=None, robot=None):
        deadline, next_publish = time.monotonic() + seconds, 0.0
        while time.monotonic() < deadline:
            now = time.monotonic()
            if now >= next_publish:
                if owner is not None:
                    self.owner_pub.publish(Pose2D(x=owner[0], y=owner[1], theta=owner[2]))
                if robot is not None:
                    self.robot_pub.publish(Pose2D(x=robot[0], y=robot[1], theta=robot[2]))
                next_publish = now + 0.04
            self.executor.spin_once(timeout_sec=0.01)

    def assert_tracking(self):
        self.assertTrue(self.visibility and self.visibility[-1][1])
        self.assertTrue(self.positions)
        self.assertTrue(self.commands)

    def assert_stopped(self):
        self.assertTrue(self.visibility)
        self.assertFalse(self.visibility[-1][1])
        self.assertTrue(self.commands)
        self.assertEqual(self.commands[-1][1:], (0.0, 0.0))

    def test_scripted_track_produces_forward_and_left_turn_commands(self):
        # Robot advances 0.2 m while owner stays ahead, then turns +90 degrees.
        for x in (0.3, 0.5):
            self.replay(0.3, owner=(2.5, 1.05, 0.0), robot=(x, 1.125, 0.0))
            self.assert_tracking()
            self.assertAlmostEqual(self.positions[-1][1], 2.5-x)
            self.assertAlmostEqual(self.positions[-1][2], 0.0)
            self.assertGreater(self.commands[-1][1], 0.0)
        self.replay(0.3, owner=(2.5, 1.05, 0.0), robot=(0.5, 1.125, math.pi/2))
        self.assertLess(self.commands[-1][2], 0.0)  # Owner is now on the robot's right.
        self.replay(0.3, owner=(0.5, 0.0, 0.0), robot=(0.5, 0.0, 0.0))
        self.assert_tracking()
        self.assertGreater(self.positions[-1][2], 0.0)
        self.assertGreater(self.commands[-1][2], 0.0)
        self.assertEqual(self.commands[-1][1], 0.0)

    def test_loss_of_either_publisher_stops_and_fresh_data_recovers(self):
        for missing in ("owner", "robot"):
            with self.subTest(missing=missing):
                owner, robot = (2.5, 1.05, 0.0), (0.5, 1.125, 0.0)
                self.replay(0.3, owner, robot)
                self.assert_tracking()
                cutoff = time.monotonic()
                self.replay(0.85, None if missing == "owner" else owner,
                            None if missing == "robot" else robot)
                self.assert_stopped()
                self.assertFalse([p for p in self.positions if p[0] > cutoff + 0.7])
                late = [c for c in self.commands if c[0] > cutoff + 0.7]
                self.assertTrue(late)
                self.assertTrue(all(c[1:] == (0.0, 0.0) for c in late))
                self.replay(0.3, owner, robot)
                self.assert_tracking()
                self.assertGreater(self.commands[-1][1], 0.0)

    def test_invalid_samples_invalidate_previous_targets_without_waiting_for_timeout(self):
        owner, robot = (2.5, 1.05, 0.0), (0.5, 1.125, 0.0)
        for bad_owner, bad_robot in (((math.nan, 1.05, 0.0), robot),
                                     (owner, (0.5, 1.125, math.inf)),
                                     ((100.0, 1.05, 0.0), robot),
                                     (owner, (-100.0, 1.125, 0.0))):
            with self.subTest(owner=bad_owner, robot=bad_robot):
                self.replay(0.3, owner, robot)
                self.assert_tracking()
                cutoff = time.monotonic()
                self.replay(0.3, bad_owner, bad_robot)
                self.assert_stopped()
                self.assertFalse([p for p in self.positions if p[0] > cutoff + 0.2])


if __name__ == "__main__":
    unittest.main()
