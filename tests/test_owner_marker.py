"""Exercise actual marker detection/pose estimation from projected camera images."""

import math
from pathlib import Path
import sys
import unittest

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))
from owner_marker import MarkerDetector, dictionary, marker_cells


K = np.array([[530., 0, 320.], [0, 530., 240.], [0, 0, 1.]])


def projected_marker(marker_id=23, translation=(0., 0., 1.5),
                     rotation=(math.pi, 0., 0.), size=0.30):
    half = size / 2
    points = np.array([[-half, half, 0], [half, half, 0],
                       [half, -half, 0], [-half, -half, 0]], np.float64)
    corners, _ = cv2.projectPoints(points, np.array(rotation, np.float64),
                                   np.array(translation, np.float64), K, np.zeros(5))
    image = cv2.aruco.drawMarker(dictionary(), marker_id, 600)
    homography = cv2.getPerspectiveTransform(
        np.array([[0, 0], [599, 0], [599, 599], [0, 599]], np.float32),
        corners.reshape(4, 2).astype(np.float32))
    return cv2.warpPerspective(image, homography, (640, 480),
                               flags=cv2.INTER_NEAREST, borderValue=255)


class OwnerMarkerTests(unittest.TestCase):
    def setUp(self):
        self.detector = MarkerDetector(23, 0.30)

    def detect(self, image):
        return self.detector.detect(image, K, np.zeros(5))

    def test_front_marker_recovers_metric_distance(self):
        detection, reason = self.detect(projected_marker())
        self.assertEqual(reason, "tracking owner")
        np.testing.assert_allclose(detection.translation, (0., 0., 1.5), atol=0.04)

    def test_off_centre_tilted_marker_recovers_direction_and_distance(self):
        expected = (0.25, -0.1, 1.8)
        detection, reason = self.detect(projected_marker(
            translation=expected, rotation=(3.0, 0.12, -0.06)))
        self.assertEqual(reason, "tracking owner")
        np.testing.assert_allclose(detection.translation, expected, atol=0.06)

    def test_wrong_id_is_not_adopted_as_owner(self):
        detection, reason = self.detect(projected_marker(marker_id=24))
        self.assertIsNone(detection)
        self.assertIn("do not match", reason)

    def test_duplicate_owner_id_is_ambiguous(self):
        image = np.minimum(projected_marker(translation=(-0.3, 0, 1.5)),
                           projected_marker(translation=(0.3, 0, 1.5)))
        detection, reason = self.detect(image)
        self.assertIsNone(detection)
        self.assertIn("ambiguous", reason)

    def test_distractor_marker_does_not_change_selected_owner(self):
        image = np.minimum(projected_marker(translation=(-0.3, 0, 1.5)),
                           projected_marker(marker_id=24, translation=(0.3, 0, 1.5)))
        detection, reason = self.detect(image)
        self.assertEqual(reason, "tracking owner")
        self.assertLess(detection.translation[0], 0)

    def test_blank_and_very_small_markers_do_not_produce_positions(self):
        detection, _ = self.detect(np.full((480, 640), 255, np.uint8))
        self.assertIsNone(detection)
        detection, _ = self.detect(projected_marker(translation=(0, 0, 6)))
        self.assertIsNone(detection)

    def test_range_limit_is_enforced_after_detection(self):
        detector = MarkerDetector(23, 0.30, max_distance=1.0)
        detection, reason = detector.detect(projected_marker(), K, np.zeros(5))
        self.assertIsNone(detection)
        self.assertIn("range", reason)

    def test_marker_cells_have_white_quiet_zone_and_black_border(self):
        cells = marker_cells(23)
        self.assertTrue(np.all(cells[0] == 255))
        self.assertTrue(np.all(cells[:, 0] == 255))
        self.assertTrue(np.all(cells[1, 1:7] == 0))

    def test_invalid_marker_configuration_fails(self):
        for marker_id in (-1, 50):
            with self.assertRaises(ValueError):
                MarkerDetector(marker_id=marker_id, marker_size=0.30)
        with self.assertRaises(ValueError):
            MarkerDetector(marker_id=23, marker_size=0)


if __name__ == "__main__":
    unittest.main()
