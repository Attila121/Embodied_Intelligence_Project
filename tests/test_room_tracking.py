"""Coordinate and fusion regressions for two fixed cameras."""

from pathlib import Path
import sys
from types import SimpleNamespace
import unittest

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/"scripts"))
from room_tracking import CameraObservation, fuse_observations, owner_in_base


class RoomTrackingTests(unittest.TestCase):
    def detection(self, translation, rotation):
        return SimpleNamespace(translation=np.array(translation, float),
                               rotation=np.array(rotation, float))

    def test_robot_rotation_is_removed_from_relative_position(self):
        # Camera coordinates are arbitrary; robot marker has yaw +90 degrees.
        robot = self.detection((1, 2, 5), (0, 0, np.pi/2))
        owner = self.detection((1, 4, 5.5), (0, 0, 0))
        result = owner_in_base(robot, owner, (-0.064, 0, 0.32), (0, 0, 0))
        np.testing.assert_allclose(result, (1.936, 0, 0.82), atol=1e-8)

    def test_camera_pose_cancels_for_two_distinct_viewpoints(self):
        marker_position = np.array((0.2, -0.3, 0.32))
        owner_position = np.array((1.8, 0.6, 0.75))
        expected = owner_position-marker_position+np.array((-0.064, 0, 0.32))
        for rotation, translation in (((0.7, 0.3, -0.4), (1, 3, 5)),
                                      ((-0.5, 0.1, 0.9), (-2, 1, 6))):
            matrix, _ = cv2.Rodrigues(np.array(rotation))
            robot = self.detection(matrix@marker_position+translation, rotation)
            owner = self.detection(matrix@owner_position+translation, (0, 0, 0))
            result = owner_in_base(robot, owner, (-0.064, 0, 0.32), (0, 0, 0))
            np.testing.assert_allclose(result, expected, atol=1e-8)

    def test_known_marker_mount_rotation_is_applied(self):
        robot = self.detection((0, 0, 4), (0, 0, 0))
        owner = self.detection((2, 0, 4), (0, 0, 0))
        np.testing.assert_allclose(owner_in_base(robot, owner, (0, 0, 0), (0, 0, np.pi/2)),
                                   (0, 2, 0), atol=1e-8)

    def obs(self, position=(2, 0, 0.75), stamp=10., received=100.):
        return CameraObservation(np.array(position, float), stamp, received)

    def test_either_camera_can_supply_a_fresh_position(self):
        for name in ("left", "right"):
            point, token, reason = fuse_observations({name: self.obs()}, 100.1)
            np.testing.assert_allclose(point, (2, 0, 0.75))
            self.assertIn(name, reason)

    def test_agreeing_views_are_averaged(self):
        observations = {"left": self.obs(), "right": self.obs((2.1, 0.1, 0.75), 10.05)}
        point, token, reason = fuse_observations(observations, 100.1)
        np.testing.assert_allclose(point, (2.05, 0.05, 0.75))
        self.assertIn("left+right", reason)
        self.assertEqual(len(token), 2)

    def test_disagreement_blocks_motion(self):
        point, token, reason = fuse_observations(
            {"left": self.obs(), "right": self.obs((3, 1, 0.75))}, 100.1)
        self.assertIsNone(point)
        self.assertIn("disagree", reason)

    def test_unsynchronised_views_use_newest_instead_of_averaging(self):
        point, token, _ = fuse_observations(
            {"left": self.obs(), "right": self.obs((3, 0, 0.75), 10.5)}, 100.1)
        np.testing.assert_allclose(point, (3, 0, 0.75))
        self.assertEqual(len(token), 1)

    def test_stale_camera_is_discarded_but_other_can_continue(self):
        point, token, reason = fuse_observations(
            {"left": self.obs(received=99.), "right": self.obs()}, 100.1)
        self.assertIsNotNone(point)
        self.assertNotIn("left", reason)

    def test_missing_stale_or_nonfinite_observations_stop(self):
        for values in ({}, {"left": None}, {"left": self.obs(received=99.)},
                       {"left": self.obs((np.nan, 0, 0))}):
            point, token, _ = fuse_observations(values, 100.1)
            self.assertIsNone(point)


if __name__ == "__main__":
    unittest.main()
