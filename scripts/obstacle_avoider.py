#!/usr/bin/env python3
"""Steer around obstacles between the follower and the LiDAR guard.

  owner_follower --/following/desired_cmd_vel--> obstacle_avoider --/following/cmd_vel--> guard

Launch with ``room_follow.launch.py avoid:=true`` (the follower's output is remapped for you).
The guard stays the final, unchanged safety gate.  Any fault here (no fresh command, owner,
or valid scan; an exception) publishes zero velocity, and if this node dies the guard's
command timeout stops the robot.  Decision logic lives in avoidance.py.
"""

import dataclasses
import math
import signal
import time

import rclpy
from geometry_msgs.msg import Point, Twist
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool

from avoidance import AvoidParams, Avoider, scan_points, validate_params


class ObstacleAvoider(Node):
    def __init__(self):
        super().__init__("obstacle_avoider")
        defaults = AvoidParams()
        values = {f.name: float(self.declare_parameter(f.name, getattr(defaults, f.name)).value)
                  for f in dataclasses.fields(AvoidParams)}
        self.params = AvoidParams(**values)
        validate_params(self.params)
        self.command_timeout = float(self.declare_parameter("command_timeout", 0.3).value)
        self.owner_timeout = float(self.declare_parameter("owner_timeout", 0.5).value)
        self.scan_timeout = float(self.declare_parameter("scan_timeout", 0.6).value)
        self.scan_frame = str(self.declare_parameter("scan_frame", "base_scan").value)
        for name in ("command_timeout", "owner_timeout", "scan_timeout"):
            value = getattr(self, name)
            if not math.isfinite(value) or value <= 0.0:
                raise ValueError(f"{name} must be positive and finite")
        self.avoider = Avoider(self.params, time.monotonic)

        self.desired = None  # (linear, angular, time)
        self.owner = None    # (x, y, time)
        self.points = None   # (points, time)
        self.status = None
        self.publisher = self.create_publisher(Twist, "/following/cmd_vel", 1)
        self.create_subscription(Twist, "/following/desired_cmd_vel", self.on_command, 1)
        self.create_subscription(Point, "/owner/relative_position", self.on_owner, 1)
        self.create_subscription(Bool, "/owner/visible", self.on_visible, 1)
        self.create_subscription(
            LaserScan, "/scan", self.on_scan,
            QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.timer = self.create_timer(0.05, self.update,
                                       clock=Clock(clock_type=ClockType.STEADY_TIME))
        self.get_logger().info("Steering between /following/desired_cmd_vel and /following/cmd_vel")

    # -- inputs (same validity rules as the guard for commands)
    def on_command(self, message):
        values = (message.linear.x, message.linear.y, message.linear.z,
                  message.angular.x, message.angular.y, message.angular.z)
        if (not all(math.isfinite(v) for v in values) or any(v != 0.0 for v in values[1:5])
                or message.linear.x < 0.0):
            self.desired = None
        else:
            self.desired = (message.linear.x, message.angular.z, time.monotonic())

    def on_owner(self, message):
        if all(math.isfinite(v) for v in (message.x, message.y)):
            self.owner = (message.x, message.y, time.monotonic())
        else:
            self.owner = None

    def on_visible(self, message):
        if not message.data:
            self.owner = None

    def on_scan(self, message):
        if message.header.frame_id != self.scan_frame:
            self.points = None
            return
        points = scan_points(message, self.params.sensor_offset_x)
        self.points = None if points is None else (points, time.monotonic())

    # -- control loop
    def update(self):
        command = Twist()
        mode = "stop"
        try:
            now = time.monotonic()
            desired = owner = None
            if self.desired and now - self.desired[2] < self.command_timeout:
                desired = self.desired[:2]
            if self.owner and now - self.owner[2] < self.owner_timeout:
                owner = self.owner[:2]
            if self.points is None or now - self.points[1] >= self.scan_timeout:
                mode = "no valid scan"  # stop: nothing to steer by
            else:
                decision = self.avoider.update(desired, owner, self.points[0])
                mode = decision.mode
                command.linear.x, command.angular.z = decision.linear, decision.angular
        except Exception as error:  # fail safe: never leave a stale command running
            command = Twist()
            mode = f"error: {error}"
        self.publisher.publish(command)
        if mode != self.status:
            self.get_logger().info(f"avoidance: {mode}")
            self.status = mode

    def stop(self):
        for _ in range(3):
            if rclpy.ok():
                self.publisher.publish(Twist())
                time.sleep(0.05)


def main():
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)

    def interrupt(*_):
        raise KeyboardInterrupt

    signal.signal(signal.SIGINT, interrupt)
    signal.signal(signal.SIGTERM, interrupt)
    node = None
    try:
        node = ObstacleAvoider()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        signal.signal(signal.SIGTERM, signal.SIG_IGN)
        if node is not None:
            node.stop()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
