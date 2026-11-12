#!/usr/bin/env python3
"""Bridge Haichen's planar Pose2D streams to the owner's robot-relative position."""

from dataclasses import asdict
import math
import signal
import time

import rclpy
from geometry_msgs.msg import Point, Pose2D
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from std_msgs.msg import Bool

from rig_geometry import RigGeometry, RigState


class RigBridge(Node):
    def __init__(self):
        super().__init__("rig_bridge")
        geometry = RigGeometry(**{
            name: self.declare_parameter(name, value).value
            for name, value in asdict(RigGeometry()).items()
        })
        owner_timeout = self.declare_parameter("owner_timeout", 0.5).value
        robot_timeout = self.declare_parameter("robot_timeout", 0.5).value
        rate = self.declare_parameter("publish_rate", 10.0).value
        if not math.isfinite(rate) or rate <= 0:
            raise ValueError("publish_rate must be positive and finite")
        self.state = RigState(geometry, owner_timeout, robot_timeout)
        owner_topic = self.declare_parameter("owner_topic", "/owner_position").value
        robot_topic = self.declare_parameter("robot_topic", "/robot_position").value
        self.position_pub = self.create_publisher(Point, "/owner/relative_position", 1)
        self.visible_pub = self.create_publisher(Bool, "/owner/visible", 1)
        self.owner_sub = self.create_subscription(Pose2D, owner_topic, self.on_owner, 1)
        self.robot_sub = self.create_subscription(Pose2D, robot_topic, self.on_robot, 1)
        self.status = None
        self.timer = self.create_timer(
            1.0/rate, self.update, clock=Clock(clock_type=ClockType.STEADY_TIME))
        self.get_logger().info(
            f"Rig inputs: {owner_topic}, {robot_topic}; output x forward / y left, metres. "
            "Receive-time freshness cannot detect the robot publisher replaying cached poses.")

    def on_owner(self, message):
        if not self.state.receive_owner((message.x, message.y, message.theta)):
            self.invalidate()

    def on_robot(self, message):
        if not self.state.receive_robot((message.x, message.y, message.theta)):
            self.invalidate()

    def invalidate(self):
        # Clear the follower immediately; the next timer also reports invalid.
        self.visible_pub.publish(Bool(data=False))

    def update(self):
        point = self.state.relative_position()
        valid = point is not None
        self.visible_pub.publish(Bool(data=valid))
        if valid:
            self.position_pub.publish(Point(x=float(point[0]), y=float(point[1]), z=0.0))
        if valid != self.status:
            self.get_logger().info("tracking owner" if valid else "missing, invalid or stale rig input")
            self.status = valid


def main():
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)

    def interrupt(*_):
        raise KeyboardInterrupt

    signal.signal(signal.SIGINT, interrupt)
    signal.signal(signal.SIGTERM, interrupt)
    node = None
    try:
        node = RigBridge()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        signal.signal(signal.SIGTERM, signal.SIG_IGN)
        if node is not None:
            if rclpy.ok():
                node.invalidate()
                time.sleep(0.05)
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
