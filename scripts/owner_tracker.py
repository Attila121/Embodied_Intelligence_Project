#!/usr/bin/env python3
"""Publish the selected ArUco owner's measured position in base_footprint."""

import math
import signal
import time

import cv2
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import rclpy
from geometry_msgs.msg import Point, PointStamped
from rclpy.clock import Clock, ClockType
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.signals import SignalHandlerOptions
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Bool
from tf2_geometry_msgs import do_transform_point
from tf2_ros import Buffer, TransformException, TransformListener

from owner_marker import DEFAULT_ID, DEFAULT_SIZE, MarkerDetector


def camera_calibration(info):
    if info.distortion_model not in ("plumb_bob", "rational_polynomial"):
        raise ValueError("unsupported camera distortion model")
    matrix = np.asarray(info.k, np.float64).reshape(3, 3)
    distortion = np.asarray(info.d, np.float64)
    if not len(distortion):
        distortion = np.zeros(5)
    if (not np.isfinite(matrix).all() or not np.isfinite(distortion).all()
            or matrix[0, 0] <= 0 or matrix[1, 1] <= 0
            or not np.allclose(matrix[2], [0, 0, 1])
            or len(distortion) not in (4, 5, 8, 12, 14)
            or not info.width or not info.height):
        raise ValueError("camera calibration missing or invalid")
    if (info.binning_x > 1 or info.binning_y > 1
            or info.roi.x_offset or info.roi.y_offset):
        raise ValueError("cropped or binned camera requires adjusted calibration")
    return matrix, distortion


class OwnerTracker(Node):
    def __init__(self):
        super().__init__("owner_tracker")
        defaults = {
            "marker_id": DEFAULT_ID, "marker_size": DEFAULT_SIZE,
            "image_topic": "/camera/image_raw", "camera_info_topic": "/camera/camera_info",
            "target_frame": "base_footprint", "max_rate": 10.0,
            "image_timeout": 0.35, "max_image_age": 0.3,
            "min_edge_pixels": 18.0, "max_reprojection_error": 2.5,
            "min_distance": 0.2, "max_distance": 4.0,
        }
        self.settings = {name: self.declare_parameter(name, value).value
                         for name, value in defaults.items()}
        for name, value in self.settings.items():
            if name == "marker_id" or isinstance(value, str):
                continue
            if not math.isfinite(value) or value <= 0:
                raise ValueError(f"{name} must be positive and finite")
        if self.settings["min_distance"] >= self.settings["max_distance"]:
            raise ValueError("min_distance must be below max_distance")
        keys = ("marker_id", "marker_size", "min_edge_pixels", "max_reprojection_error",
                "min_distance", "max_distance")
        self.detector = MarkerDetector(**{key: self.settings[key] for key in keys})
        self.bridge = CvBridge()
        self.info = None
        self.calibration = None
        self.last_process_time = -math.inf
        self.last_stamp_ns = -1
        self.last_clock_ns = None
        self.last_success_time = -math.inf
        self.status = None
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.position_pub = self.create_publisher(Point, "/owner/relative_position", 1)
        self.observation_pub = self.create_publisher(PointStamped, "/owner/observation", 1)
        self.visible_pub = self.create_publisher(Bool, "/owner/visible", 1)
        self.debug_pub = self.create_publisher(Image, "/owner/debug_image", 1)
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.info_sub = self.create_subscription(
            CameraInfo, self.settings["camera_info_topic"], self.on_info, qos)
        self.image_sub = self.create_subscription(
            Image, self.settings["image_topic"], self.on_image, qos)
        self.watchdog = self.create_timer(
            0.05, self.check_image_timeout,
            clock=Clock(clock_type=ClockType.STEADY_TIME))
        self.report(f"waiting for owner marker ID {self.settings['marker_id']}")

    def report(self, status):
        if status != self.status:
            self.get_logger().info(status)
            self.status = status

    def lost(self, reason):
        self.last_success_time = -math.inf
        self.visible_pub.publish(Bool(data=False))
        self.report(reason)

    def check_image_timeout(self):
        if time.monotonic() - self.last_success_time > self.settings["image_timeout"]:
            self.visible_pub.publish(Bool(data=False))
            if self.status == "tracking owner":
                self.report("owner image updates stopped")

    def on_info(self, message):
        try:
            calibration = camera_calibration(message)
        except ValueError as error:
            self.info, self.calibration = None, None
            self.lost(str(error))
            return
        self.info, self.calibration = message, calibration

    def on_image(self, message):
        now = time.monotonic()
        if now - self.last_process_time < 1 / self.settings["max_rate"]:
            return
        self.last_process_time = now
        stamp = Time.from_msg(message.header.stamp)
        clock_ns = self.get_clock().now().nanoseconds
        if self.last_clock_ns is not None and clock_ns < self.last_clock_ns:
            self.last_stamp_ns = -1
            self.lost("simulation clock reset; waiting for a fresh image")
        self.last_clock_ns = clock_ns
        age = (clock_ns - stamp.nanoseconds) / 1e9
        if stamp.nanoseconds <= self.last_stamp_ns:
            self.lost("duplicate or out-of-order camera image")
            return
        self.last_stamp_ns = stamp.nanoseconds
        # Gazebo can deliver a camera sample before the corresponding /clock
        # sample; allow a small bounded lead as well as bounded lag.
        if abs(age) > self.settings["max_image_age"]:
            self.lost("camera image timestamp is stale or clock does not match")
            return
        if self.info is None:
            self.lost("waiting for valid camera calibration")
            return
        if (message.header.frame_id != self.info.header.frame_id
                or message.width != self.info.width or message.height != self.info.height):
            self.lost("image frame or dimensions do not match calibration")
            return
        try:
            image = self.bridge.imgmsg_to_cv2(message, desired_encoding="bgr8")
            detection, reason = self.detector.detect(image, *self.calibration)
        except (CvBridgeError, cv2.error, ValueError) as error:
            self.lost(f"camera processing failed: {error}")
            return
        if detection is None:
            self.lost(reason)
            self.publish_debug(image, message, None)
            return
        try:
            transform = self.buffer.lookup_transform(
                self.settings["target_frame"], message.header.frame_id,
                stamp, timeout=Duration(seconds=0.02))
            optical_point = PointStamped(header=message.header)
            optical_point.point = Point(**dict(zip(
                ("x", "y", "z"), map(float, detection.translation))))
            observation = do_transform_point(optical_point, transform)
        except TransformException:
            self.lost("waiting for camera-to-robot transform")
            self.publish_debug(image, message, detection)
            return
        point = observation.point
        if not all(math.isfinite(value) for value in (point.x, point.y, point.z)) or point.x <= 0:
            self.lost("owner position is invalid or behind robot")
            return
        # Recheck age after vision and TF work. Never refresh an old observation.
        age = (self.get_clock().now() - stamp).nanoseconds / 1e9
        if abs(age) > self.settings["max_image_age"]:
            self.lost("owner image became stale during processing")
            return
        observation.header.stamp = message.header.stamp
        observation.header.frame_id = self.settings["target_frame"]
        self.last_success_time = time.monotonic()
        self.visible_pub.publish(Bool(data=True))
        self.observation_pub.publish(observation)
        self.position_pub.publish(point)
        self.report("tracking owner")
        self.publish_debug(image, message, detection)

    def publish_debug(self, image, source, detection):
        if not self.debug_pub.get_subscription_count():
            return
        image = image.copy()
        if detection is not None:
            cv2.polylines(image, [detection.corners.astype(np.int32)], True, (0, 255, 0), 2)
        cv2.putText(image, f"ID {self.settings['marker_id']}: {self.status}",
                    (10, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 180, 255), 1)
        output = self.bridge.cv2_to_imgmsg(image, encoding="bgr8")
        output.header = source.header
        self.debug_pub.publish(output)


def main():
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)

    def interrupt(*_):
        raise KeyboardInterrupt

    signal.signal(signal.SIGINT, interrupt)
    signal.signal(signal.SIGTERM, interrupt)
    node = None
    try:
        node = OwnerTracker()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        signal.signal(signal.SIGTERM, signal.SIG_IGN)
        if node is not None:
            if rclpy.ok():
                node.visible_pub.publish(Bool(data=False))
                time.sleep(0.05)
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
