#!/usr/bin/env python3
"""Track owner and robot markers through two fixed room cameras."""

import math
import signal
import time

import cv2
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import rclpy
from geometry_msgs.msg import Point, PointStamped
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.signals import SignalHandlerOptions
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Bool

from owner_marker import MarkerDetector
from room_tracking import CameraObservation, camera_calibration, fuse_observations, owner_in_base


class RoomOwnerTracker(Node):
    def __init__(self):
        super().__init__("room_owner_tracker")
        cv2.setNumThreads(1)
        defaults = {
            "owner_id": 23, "robot_id": 24, "owner_size": 0.30, "robot_size": 0.24,
            "robot_marker_translation": [-0.064, 0.0, 0.32],
            "robot_marker_rpy": [0.0, 0.0, 0.0],
            "max_rate": 6.0, "image_timeout": 0.6, "max_image_age": 0.4,
            "max_camera_skew": 0.2, "max_disagreement": 0.35,
            "min_edge_pixels": 10.0, "max_reprojection_error": 2.5,
            "left_image_topic": "/room_cameras/left/image_raw",
            "left_info_topic": "/room_cameras/left/camera_info",
            "right_image_topic": "/room_cameras/right/image_raw",
            "right_info_topic": "/room_cameras/right/camera_info",
        }
        self.settings = {name: self.declare_parameter(name, value).value
                         for name, value in defaults.items()}
        for name in ("max_rate", "image_timeout", "max_image_age", "max_camera_skew",
                     "max_disagreement", "min_edge_pixels", "max_reprojection_error"):
            value = self.settings[name]
            if not math.isfinite(value) or value <= 0:
                raise ValueError(f"{name} must be positive and finite")
        for name in ("robot_marker_translation", "robot_marker_rpy"):
            value = self.settings[name]
            if len(value) != 3 or not all(math.isfinite(v) for v in value):
                raise ValueError(f"{name} must contain three finite values")
        if self.settings["owner_id"] == self.settings["robot_id"]:
            raise ValueError("owner and robot need different marker IDs")
        shared = {"min_edge_pixels": self.settings["min_edge_pixels"],
                  "max_reprojection_error": self.settings["max_reprojection_error"],
                  "max_distance": 12.0}
        self.owner_detector = MarkerDetector(self.settings["owner_id"],
                                             self.settings["owner_size"], **shared)
        self.robot_detector = MarkerDetector(self.settings["robot_id"],
                                             self.settings["robot_size"], **shared)
        self.bridge = CvBridge()
        self.observations = {"left": None, "right": None}
        self.cameras = {name: {"info": None, "calibration": None, "last_process": -math.inf,
                               "stamp": -1, "ambiguous": -math.inf, "status": None}
                        for name in self.observations}
        self.last_clock = None
        self.last_token = None
        self.status = None
        self.position_pub = self.create_publisher(Point, "/owner/relative_position", 1)
        self.observation_pub = self.create_publisher(PointStamped, "/owner/observation", 1)
        self.visible_pub = self.create_publisher(Bool, "/owner/visible", 1)
        self.debug = {name: self.create_publisher(
            Image, f"/room_cameras/{name}/debug_image", 1) for name in self.cameras}
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.subscriptions_keepalive = []
        for name in self.cameras:
            self.subscriptions_keepalive.extend([
                self.create_subscription(CameraInfo, self.settings[f"{name}_info_topic"],
                    lambda message, camera=name: self.on_info(camera, message), qos),
                self.create_subscription(Image, self.settings[f"{name}_image_topic"],
                    lambda message, camera=name: self.on_image(camera, message), qos),
            ])
        self.timer = self.create_timer(0.05, self.update,
                                      clock=Clock(clock_type=ClockType.STEADY_TIME))

    def report_camera(self, name, status):
        camera = self.cameras[name]
        if camera["status"] != status:
            self.get_logger().info(f"{name}: {status}")
            camera["status"] = status

    def reject(self, name, reason, ambiguous=False):
        self.observations[name] = None
        self.cameras[name]["ambiguous"] = time.monotonic() if ambiguous else -math.inf
        self.report_camera(name, reason)

    def on_info(self, name, message):
        try:
            calibration = camera_calibration(message)
        except ValueError as error:
            self.cameras[name]["info"] = None
            self.reject(name, str(error))
            return
        self.cameras[name]["info"] = message
        self.cameras[name]["calibration"] = calibration

    def on_image(self, name, message):
        camera = self.cameras[name]
        now = time.monotonic()
        if now-camera["last_process"] < 1/self.settings["max_rate"]:
            return
        camera["last_process"] = now
        clock_ns = self.get_clock().now().nanoseconds
        if self.last_clock is not None and clock_ns < self.last_clock:
            for key, state in self.cameras.items():
                state["stamp"], state["ambiguous"] = -1, -math.inf
                self.observations[key] = None
            self.last_token = None
        self.last_clock = clock_ns
        stamp = Time.from_msg(message.header.stamp)
        if stamp.nanoseconds <= camera["stamp"]:
            self.reject(name, "duplicate or out-of-order image")
            return
        camera["stamp"] = stamp.nanoseconds
        if abs(clock_ns-stamp.nanoseconds)/1e9 > self.settings["max_image_age"]:
            self.reject(name, "stale image or mismatched clock")
            return
        info = camera["info"]
        if info is None:
            self.reject(name, "waiting for camera calibration")
            return
        if (message.header.frame_id != info.header.frame_id
                or (message.width, message.height) != (info.width, info.height)):
            self.reject(name, "image does not match camera calibration")
            return
        try:
            image = self.bridge.imgmsg_to_cv2(message, "bgr8")
            corners, ids = self.robot_detector.detect_corners(image)
            owner, owner_reason = self.owner_detector.estimate(corners, ids, *camera["calibration"])
            robot, robot_reason = self.robot_detector.estimate(corners, ids, *camera["calibration"])
            ambiguous = "ambiguous" in owner_reason or "ambiguous" in robot_reason
            if owner is None or robot is None:
                self.reject(name, f"owner: {owner_reason}; robot: {robot_reason}", ambiguous)
                self.publish_debug(name, image, message, owner, robot)
                return
            point = owner_in_base(robot, owner, self.settings["robot_marker_translation"],
                                  self.settings["robot_marker_rpy"])
        except (cv2.error, CvBridgeError, ValueError) as error:
            self.reject(name, f"camera processing failed: {error}")
            return
        age = abs(self.get_clock().now().nanoseconds-stamp.nanoseconds)/1e9
        if age > self.settings["max_image_age"]:
            self.reject(name, "image became stale during processing")
            return
        self.observations[name] = CameraObservation(point, stamp.nanoseconds/1e9, time.monotonic())
        camera["ambiguous"] = -math.inf
        self.report_camera(name, "sees owner and robot")
        self.publish_debug(name, image, message, owner, robot)

    def update(self):
        now = time.monotonic()
        timeout = self.settings["image_timeout"]
        if any(now-camera["ambiguous"] < timeout for camera in self.cameras.values()):
            point, token, reason = None, None, "duplicate marker IDs; identity ambiguous"
        else:
            point, token, reason = fuse_observations(
                self.observations, now, timeout, self.settings["max_camera_skew"],
                self.settings["max_disagreement"])
        valid = point is not None
        self.visible_pub.publish(Bool(data=valid))
        if valid and token != self.last_token:
            message = Point(x=float(point[0]), y=float(point[1]), z=float(point[2]))
            observation = PointStamped()
            observation.header.frame_id = "base_footprint"
            newest_stamp = max(stamp for _, stamp in token)
            observation.header.stamp = Time(nanoseconds=int(newest_stamp*1e9)).to_msg()
            observation.point = message
            self.observation_pub.publish(observation)
            self.position_pub.publish(message)
        self.last_token = token if valid else None
        if self.status != reason:
            self.get_logger().info(reason)
            self.status = reason

    def publish_debug(self, name, image, source, owner, robot):
        if not self.debug[name].get_subscription_count():
            return
        image = image.copy()
        for detection, color in ((owner, (0, 255, 0)), (robot, (255, 160, 0))):
            if detection is not None:
                cv2.polylines(image, [detection.corners.astype(np.int32)], True, color, 2)
        cv2.putText(image, f"{name}: {self.cameras[name]['status']}",
                    (12, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 180, 255), 1)
        output = self.bridge.cv2_to_imgmsg(image, "bgr8")
        output.header = source.header
        self.debug[name].publish(output)


def main():
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)

    def interrupt(*_):
        raise KeyboardInterrupt

    signal.signal(signal.SIGINT, interrupt)
    signal.signal(signal.SIGTERM, interrupt)
    node = None
    try:
        node = RoomOwnerTracker()
        rclpy.spin(node)
    except KeyboardInterrupt:
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
