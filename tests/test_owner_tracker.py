"""Calibration, optical-to-base transform and controller loss semantics."""

from pathlib import Path
import math
import sys
from types import SimpleNamespace
import unittest

import numpy as np
from geometry_msgs.msg import Point, PointStamped, TransformStamped
from sensor_msgs.msg import CameraInfo
from std_msgs.msg import Bool
from rclpy.time import Time
from sensor_msgs.msg import Image
from tf2_geometry_msgs import do_transform_point

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))
from owner_follower import OwnerFollower
from owner_tracker import OwnerTracker, camera_calibration


class OwnerTrackerTests(unittest.TestCase):
    def info(self):
        info = CameraInfo(width=640, height=480, distortion_model="plumb_bob")
        info.k = [530., 0., 320., 0., 530., 240., 0., 0., 1.]
        info.d = [0.] * 5
        return info

    def test_valid_calibration_is_accepted(self):
        matrix, distortion = camera_calibration(self.info())
        self.assertEqual(matrix.shape, (3, 3))
        self.assertEqual(distortion.shape, (5,))

    def test_uncalibrated_and_unsupported_cameras_are_rejected(self):
        info = self.info()
        info.k = [0.] * 9
        with self.assertRaises(ValueError):
            camera_calibration(info)
        info = self.info()
        info.distortion_model = "equidistant"
        with self.assertRaises(ValueError):
            camera_calibration(info)

    def test_nonfinite_and_cropped_calibration_are_rejected(self):
        info = self.info()
        info.d[0] = float("nan")
        with self.assertRaises(ValueError):
            camera_calibration(info)
        info = self.info()
        info.roi.x_offset = 10
        with self.assertRaises(ValueError):
            camera_calibration(info)

    def test_optical_axes_are_transformed_into_robot_axes(self):
        transform = TransformStamped()
        transform.header.frame_id = "base_footprint"
        q = transform.transform.rotation
        q.x, q.y, q.z, q.w = -0.5, 0.5, -0.5, 0.5
        transform.transform.translation.x = 0.076
        observation = PointStamped()
        observation.point = Point(x=0.2, y=0.1, z=1.5)
        point = do_transform_point(observation, transform).point
        np.testing.assert_allclose((point.x, point.y, point.z),
                                   (1.576, -0.2, -0.1), atol=1e-6)

    def test_owner_loss_clears_current_target_immediately(self):
        controller = SimpleNamespace(target=(2.0, 0.0))
        OwnerFollower.on_visibility(controller, Bool(data=False))
        self.assertIsNone(controller.target)
        OwnerFollower.on_visibility(controller, Bool(data=True))
        self.assertIsNone(controller.target)  # Visibility alone cannot command motion.

    def image_harness(self, clock_ns):
        clock = SimpleNamespace(now=lambda: Time(nanoseconds=clock_ns))
        reasons = []
        return SimpleNamespace(
            settings={"max_rate": 10., "max_image_age": 0.3},
            last_process_time=-math.inf, last_stamp_ns=-1, last_clock_ns=None,
            get_clock=lambda: clock, lost=reasons.append, info=None), reasons

    def image(self, seconds):
        image = Image()
        image.header.stamp = Time(nanoseconds=int(seconds * 1e9)).to_msg()
        return image

    def test_stale_camera_images_are_rejected(self):
        harness, reasons = self.image_harness(2_000_000_000)
        OwnerTracker.on_image(harness, self.image(1.0))
        self.assertIn("timestamp", reasons[-1])

    def test_small_camera_lead_over_gazebo_clock_is_accepted(self):
        harness, reasons = self.image_harness(1_000_000_000)
        OwnerTracker.on_image(harness, self.image(1.15))
        self.assertIn("calibration", reasons[-1])  # Passed timestamp validation.

    def test_duplicate_camera_image_is_rejected(self):
        harness, reasons = self.image_harness(1_000_000_000)
        harness.last_stamp_ns = 1_000_000_000
        OwnerTracker.on_image(harness, self.image(1.0))
        self.assertIn("duplicate", reasons[-1])

    def test_world_clock_reset_allows_fresh_images_to_resume(self):
        harness, reasons = self.image_harness(200_000_000)
        harness.last_clock_ns, harness.last_stamp_ns = 5_000_000_000, 5_000_000_000
        OwnerTracker.on_image(harness, self.image(0.2))
        self.assertIn("clock reset", reasons[0])
        self.assertIn("calibration", reasons[-1])
        self.assertEqual(harness.last_stamp_ns, 200_000_000)


if __name__ == "__main__":
    unittest.main()
