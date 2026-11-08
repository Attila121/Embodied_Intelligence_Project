#!/usr/bin/env python3
"""Marker cells and camera-only pose estimation for the room demo."""

from dataclasses import dataclass
import math

import cv2
import numpy as np

def dictionary():
    return cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)


def check_marker_id(marker_id):
    if not 0 <= marker_id < 50:
        raise ValueError("marker_id must be between 0 and 49 (DICT_4X4_50)")


def marker_cells(marker_id):
    check_marker_id(marker_id)
    if hasattr(cv2.aruco, "generateImageMarker"):
        marker = cv2.aruco.generateImageMarker(dictionary(), marker_id, 6)
    else:
        marker = cv2.aruco.drawMarker(dictionary(), marker_id, 6)
    return np.pad(marker, 1, constant_values=255)


@dataclass
class MarkerDetection:
    translation: np.ndarray  # Optical frame: x right, y down, z forward.
    rotation: np.ndarray
    corners: np.ndarray
    reprojection_error: float


class MarkerDetector:
    def __init__(self, marker_id, marker_size,
                 min_edge_pixels=18.0, max_reprojection_error=2.5,
                 min_distance=0.2, max_distance=4.0):
        check_marker_id(marker_id)
        if not math.isfinite(marker_size) or marker_size <= 0:
            raise ValueError("marker_size must be positive, in metres")
        self.marker_id, self.marker_size = marker_id, marker_size
        self.min_edge_pixels = min_edge_pixels
        self.max_reprojection_error = max_reprojection_error
        self.min_distance, self.max_distance = min_distance, max_distance
        parameters = (cv2.aruco.DetectorParameters()
                      if hasattr(cv2.aruco, "DetectorParameters")
                      else cv2.aruco.DetectorParameters_create())
        parameters.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
        self.parameters = parameters
        self.aruco = (cv2.aruco.ArucoDetector(dictionary(), parameters)
                      if hasattr(cv2.aruco, "ArucoDetector") else None)

    def detect(self, image, camera_matrix, distortion):
        corners, ids = self.detect_corners(image)
        return self.estimate(corners, ids, camera_matrix, distortion)

    def detect_corners(self, image):
        """Detect once so multiple selected marker IDs can share the same image."""
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY) if image.ndim == 3 else image
        if self.aruco is not None:
            corners, ids, _ = self.aruco.detectMarkers(gray)
        else:
            corners, ids, _ = cv2.aruco.detectMarkers(
                gray, dictionary(), parameters=self.parameters)
        return corners, ids

    def estimate(self, corners, ids, camera_matrix, distortion):
        if ids is None:
            return None, "owner marker not visible"
        matches = np.flatnonzero(ids.reshape(-1) == self.marker_id)
        if len(matches) == 0:
            return None, "visible markers do not match owner ID"
        if len(matches) > 1:
            return None, "duplicate owner markers; identity ambiguous"
        selected = np.asarray(corners[int(matches[0])], dtype=np.float32).reshape(4, 2)
        edges = np.linalg.norm(selected - np.roll(selected, -1, axis=0), axis=1)
        if not np.isfinite(selected).all() or min(edges) < self.min_edge_pixels:
            return None, "owner marker too small or malformed"
        half = self.marker_size / 2
        object_points = np.array([[-half, half, 0], [half, half, 0],
                                  [half, -half, 0], [-half, -half, 0]], np.float64)
        ok, rotation, translation = cv2.solvePnP(
            object_points, selected.astype(np.float64), camera_matrix, distortion,
            flags=cv2.SOLVEPNP_IPPE_SQUARE)
        if not ok or not np.isfinite(translation).all() or not np.isfinite(rotation).all():
            return None, "owner pose estimation failed"
        translation = translation.reshape(3)
        if translation[2] <= 0:
            return None, "owner marker is behind camera"
        distance = float(np.linalg.norm(translation))
        if not self.min_distance <= distance <= self.max_distance:
            return None, "owner marker outside tracking range"
        projected, _ = cv2.projectPoints(
            object_points, rotation, translation, camera_matrix, distortion)
        error = float(np.sqrt(np.mean(np.sum(
            (projected.reshape(4, 2) - selected) ** 2, axis=1))))
        if not math.isfinite(error) or error > self.max_reprojection_error:
            return None, "owner pose reprojection error too high"
        return MarkerDetection(translation, rotation, selected, error), "tracking owner"
