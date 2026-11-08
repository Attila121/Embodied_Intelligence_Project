#!/usr/bin/env python3
"""Simulation command guard: fresh commands, fresh full-circle scans, low speeds."""

import math
import signal
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import LaserScan


def scan_problem(scan, stop_distance, expected_frame, forward_only=False,
                 corridor_half_width=0.28, body_clearance=0.30):
    """Check scan validity and the clearance needed by the requested motion.

    Straight motion uses a padded forward corridor, allowing departure from a
    rear wall. Turning keeps the original all-around clearance. The smaller
    body-clearance circle is enforced in every direction for either motion.
    Geometry is specific to the simulated Waffle Pi with its aligned base_scan.
    """
    if scan.header.frame_id != expected_frame:
        return "unexpected scan frame"
    if not all(math.isfinite(v) for v in (
        scan.angle_min, scan.angle_increment, scan.range_min, scan.range_max
    )):
        return "invalid scan metadata"
    if not 0.0 <= scan.range_min < stop_distance < scan.range_max:
        return "invalid scan range limits"
    # Require dense, full-circle coverage before allowing either driving or turning.
    step = abs(scan.angle_increment)
    if not 0.0 < step <= math.radians(5):
        return "invalid scan resolution"
    if len(scan.ranges) * step < 2.0 * math.pi - 2.0 * step:
        return "incomplete scan coverage"
    blocked = []
    for index, distance in enumerate(scan.ranges):
        # Gazebo uses +inf for a ray with no return within range_max.
        if distance == math.inf:
            continue
        if not math.isfinite(distance) or distance < scan.range_min:
            return "invalid scan reading"
        if distance > scan.range_max:
            return "invalid scan reading"
        angle = scan.angle_min + index * scan.angle_increment
        x, y = distance * math.cos(angle), distance * math.sin(angle)
        if distance <= body_clearance:
            blocked.append((distance, angle, "obstacle near robot footprint"))
        elif forward_only:
            if 0.0 < x <= stop_distance and abs(y) <= corridor_half_width:
                blocked.append((distance, angle, "obstacle in forward path"))
        elif distance <= stop_distance:
            blocked.append((distance, angle, "turn blocked by nearby obstacle"))
    if blocked:
        distance, angle, reason = min(blocked)
        bearing = math.degrees(math.atan2(math.sin(angle), math.cos(angle)))
        return f"{reason}: {distance:.2f} m at {bearing:+.0f} deg"
    return None


class CommandGate:
    """Pure state machine; time is injectable for deterministic timeout tests."""

    def __init__(self, command_timeout=0.3, scan_timeout=0.6,
                 stop_distance=0.55, scan_frame="base_scan",
                 max_linear=0.12, max_angular=0.5,
                 corridor_half_width=0.28, body_clearance=0.30,
                 clock=time.monotonic):
        self.clock = clock
        self.command_timeout = command_timeout
        self.scan_timeout = scan_timeout
        self.stop_distance = stop_distance
        self.scan_frame = scan_frame
        self.max_linear = max_linear
        self.max_angular = max_angular
        self.corridor_half_width = corridor_half_width
        self.body_clearance = body_clearance
        self.command_time = None
        self.scan_time = None
        self.requested = None
        self.scan = None
        self.previous = (0.0, 0.0)
        self.update_time = self.clock()

    def receive_command(self, message):
        self.command_time = self.clock()
        values = (message.linear.x, message.linear.y, message.linear.z,
                  message.angular.x, message.angular.y, message.angular.z)
        if (not all(math.isfinite(v) for v in values)
                or any(v != 0.0 for v in values[1:5])
                or message.linear.x < 0.0):
            self.requested = None
        else:
            self.requested = (min(message.linear.x, self.max_linear),
                              max(-self.max_angular,
                                  min(message.angular.z, self.max_angular)))

    def receive_scan(self, message):
        self.scan_time = self.clock()
        self.scan = message

    def evaluate(self):
        now = self.clock()
        reason = None
        if self.command_time is None:
            reason = "waiting for controller"
        elif now - self.command_time >= self.command_timeout:
            reason = "controller timeout"
        elif self.requested is None:
            reason = "invalid velocity command"
        elif self.scan_time is None:
            reason = "waiting for scan"
        elif now - self.scan_time >= self.scan_timeout:
            reason = "scan timeout"
        else:
            # Turning (including a curved forward command) needs clearance all
            # around. Straight driving can move away from rear/side obstacles.
            forward_only = self.requested[0] > 0.0 and self.requested[1] == 0.0
            reason = scan_problem(
                self.scan, self.stop_distance, self.scan_frame,
                forward_only, self.corridor_half_width, self.body_clearance)

        elapsed = max(0.0, min(now - self.update_time, 0.1))
        self.update_time = now
        if reason is not None or self.requested == (0.0, 0.0):
            # Stops bypass acceleration limits.
            self.previous = (0.0, 0.0)
            return 0.0, 0.0, reason or "controller stopped"

        # Limit acceleration; zero components take effect immediately.
        output = []
        for target, old, rate in zip(self.requested, self.previous, (0.2, 1.0)):
            if target == 0.0:
                output.append(0.0)
            else:
                change = rate * elapsed
                output.append(max(old - change, min(target, old + change)))
        self.previous = tuple(output)
        return *self.previous, "following"


class FollowingGuard(Node):
    def __init__(self):
        super().__init__("following_guard")
        defaults = {
            "command_timeout": 0.3, "scan_timeout": 0.6,
            "stop_distance": 0.55, "scan_frame": "base_scan",
            "max_linear": 0.12, "max_angular": 0.5,
            "corridor_half_width": 0.28, "body_clearance": 0.30,
        }
        values = {name: self.declare_parameter(name, default).value
                  for name, default in defaults.items()}
        for name, value in values.items():
            if name != "scan_frame" and (not math.isfinite(value) or value <= 0.0):
                raise ValueError(f"{name} must be positive and finite")
        if values["body_clearance"] >= values["stop_distance"]:
            raise ValueError("body_clearance must be smaller than stop_distance")
        self.gate = CommandGate(**values)
        self.publisher = self.create_publisher(Twist, "/following/safe_cmd_vel", 1)
        self.command_sub = self.create_subscription(
            Twist, "/following/cmd_vel", self.gate.receive_command, 1)
        self.scan_sub = self.create_subscription(
            LaserScan, "/scan", self.gate.receive_scan,
            QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.status = None
        self.timer = self.create_timer(
            0.05, self.update, clock=Clock(clock_type=ClockType.STEADY_TIME))
        self.get_logger().info(f"Guard output: {self.publisher.topic_name}")

    def update(self):
        linear, angular, reason = self.gate.evaluate()
        message = Twist()
        message.linear.x, message.angular.z = linear, angular
        self.publisher.publish(message)
        if reason != self.status:
            self.get_logger().info(reason)
            self.status = reason

    def stop(self):
        for _ in range(3):
            if rclpy.ok():
                self.publisher.publish(Twist())
                time.sleep(0.05)


def main():
    # Keep the ROS context alive long enough to send zeros on Ctrl+C/SIGTERM.
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)

    def interrupt(*_):
        raise KeyboardInterrupt

    signal.signal(signal.SIGINT, interrupt)
    signal.signal(signal.SIGTERM, interrupt)
    node = None
    try:
        node = FollowingGuard()
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
