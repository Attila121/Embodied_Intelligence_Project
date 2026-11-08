#!/usr/bin/env python3
"""Follow measured owner positions, routing commands through the LiDAR guard."""

import math
import signal
import time

import rclpy
from geometry_msgs.msg import Point, Twist
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from std_msgs.msg import Bool


class OwnerFollower(Node):
    def __init__(self):
        super().__init__("owner_follower")

        self.declare_parameter("following_distance", 1.0)
        self.declare_parameter("max_linear_speed", 0.12)
        self.declare_parameter("max_angular_speed", 0.5)
        self.declare_parameter("target_timeout", 0.5)

        self.target = None
        self.last_target_time = None

        self.publisher = self.create_publisher(
            Twist, "/following/cmd_vel", 1
        )
        self.subscription = self.create_subscription(
            Point, "/owner/relative_position", self.on_target, 1
        )
        self.visibility_subscription = self.create_subscription(
            Bool, "/owner/visible", self.on_visibility, 1
        )
        self.timer = self.create_timer(0.05, self.update)

        self.get_logger().info(
            "Waiting for owner position; publishing to /following/cmd_vel"
        )

    def on_target(self, message):
        if not all(math.isfinite(v) for v in (message.x, message.y)):
            self.target = None
            return

        self.target = (message.x, message.y)
        self.last_target_time = time.monotonic()

    def on_visibility(self, message):
        if not message.data:
            self.target = None

    def update(self):
        command = Twist()

        timeout = self.get_parameter("target_timeout").value
        target_fresh = (
            self.target is not None
            and self.last_target_time is not None
            and time.monotonic() - self.last_target_time < timeout
        )

        if target_fresh:
            x, y = self.target
            distance = math.hypot(x, y)
            bearing = math.atan2(y, x)

            desired = self.get_parameter("following_distance").value
            max_linear = self.get_parameter("max_linear_speed").value
            max_angular = self.get_parameter("max_angular_speed").value

            # Stop within the following distance plus a small tolerance.
            if distance > desired + 0.1:
                if abs(bearing) > 0.05:
                    command.angular.z = max(
                        -max_angular, min(max_angular, 0.8 * bearing)
                    )

                # Turn toward the owner before driving forward.
                if abs(bearing) < 0.35:
                    command.linear.x = min(
                        max_linear, 0.3 * (distance - desired)
                    )

        # Missing or stale targets produce zero velocity every cycle.
        self.publisher.publish(command)


def main():
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)

    def interrupt(*_):
        raise KeyboardInterrupt

    signal.signal(signal.SIGINT, interrupt)
    signal.signal(signal.SIGTERM, interrupt)
    node = OwnerFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        signal.signal(signal.SIGTERM, signal.SIG_IGN)
        if rclpy.ok():
            node.publisher.publish(Twist())
            time.sleep(0.05)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
