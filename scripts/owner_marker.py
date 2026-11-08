#!/usr/bin/env python3
"""Marker generation and camera-only pose estimation, without ROS dependencies."""

from dataclasses import dataclass
import math
from xml.sax.saxutils import quoteattr

import cv2
import numpy as np

DEFAULT_ID = 23
DEFAULT_SIZE = 0.15  # Outer black square side, excluding the white quiet zone.


def dictionary():
    return cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)


def check_marker(marker_id, marker_size):
    if not 0 <= marker_id < 50:
        raise ValueError("marker_id must be between 0 and 49 (DICT_4X4_50)")
    if not math.isfinite(marker_size) or marker_size <= 0:
        raise ValueError("marker_size must be positive, in metres")


def marker_cells(marker_id=DEFAULT_ID):
    check_marker(marker_id, DEFAULT_SIZE)
    if hasattr(cv2.aruco, "generateImageMarker"):
        marker = cv2.aruco.generateImageMarker(dictionary(), marker_id, 6)
    else:
        marker = cv2.aruco.drawMarker(dictionary(), marker_id, 6)
    return np.pad(marker, 1, constant_values=255)


def marker_image(marker_id=DEFAULT_ID, pixels_per_cell=100):
    cells = marker_cells(marker_id)
    return cv2.resize(cells, (8 * pixels_per_cell, 8 * pixels_per_cell),
                      interpolation=cv2.INTER_NEAREST)


def marker_svg(marker_id=DEFAULT_ID, marker_size=DEFAULT_SIZE):
    check_marker(marker_id, marker_size)
    side_mm = marker_size * 1000 * 8 / 6
    lines = [f'<svg xmlns="http://www.w3.org/2000/svg" width="{side_mm:g}mm" '
             f'height="{side_mm:g}mm" viewBox="0 0 8 8">',
             '<rect width="8" height="8" fill="white"/>']
    for row, values in enumerate(marker_cells(marker_id)):
        for col, value in enumerate(values):
            if value == 0:
                lines.append(f'<rect x="{col}" y="{row}" width="1" '
                             'height="1" fill="black"/>')
    return "\n".join(lines + ["</svg>", ""])


def marker_sdf(marker_id=DEFAULT_ID, marker_size=DEFAULT_SIZE,
               model_name="owner_marker"):
    """Pixel boxes avoid external textures, paths and mirrored UV coordinates.

    The marker faces local -x. Seen from there, image-right is local -y, and
    image-up is local +z. Its local centre is the model origin.
    """
    check_marker(marker_id, marker_size)
    cell = marker_size / 6
    board = cell * 8
    lines = [f'<sdf version="1.6"><model name={quoteattr(model_name)}>',
             '<static>true</static><link name="board">',
             '<collision name="board_collision"><geometry><box>',
             f'<size>0.015 {board:.9f} {board:.9f}</size>',
             '</box></geometry></collision>']
    for row, values in enumerate(marker_cells(marker_id)):
        for col, value in enumerate(values):
            color = "1 1 1 1" if value else "0 0 0 1"
            material = "Gazebo/White" if value else "Gazebo/Black"
            y, z = (3.5 - col) * cell, (3.5 - row) * cell
            lines += [f'<visual name="cell_{row}_{col}">',
                      f'<pose>0 {y:.9f} {z:.9f} 0 0 0</pose>',
                      f'<geometry><box><size>0.01 {cell:.9f} {cell:.9f}</size>'
                      '</box></geometry>',
                      '<cast_shadows>false</cast_shadows><material>',
                      '<script><uri>file://media/materials/scripts/gazebo.material</uri>'
                      f'<name>{material}</name></script>',
                      f'<ambient>{color}</ambient><diffuse>{color}</diffuse>'
                      f'<emissive>{color}</emissive>',
                      '</material></visual>']
    return "\n".join(lines + ['</link></model></sdf>'])


@dataclass
class MarkerDetection:
    translation: np.ndarray  # Optical frame: x right, y down, z forward.
    rotation: np.ndarray
    corners: np.ndarray
    reprojection_error: float


class MarkerDetector:
    def __init__(self, marker_id=DEFAULT_ID, marker_size=DEFAULT_SIZE,
                 min_edge_pixels=18.0, max_reprojection_error=2.5,
                 min_distance=0.2, max_distance=4.0):
        check_marker(marker_id, marker_size)
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
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY) if image.ndim == 3 else image
        if self.aruco is not None:
            corners, ids, _ = self.aruco.detectMarkers(gray)
        else:
            corners, ids, _ = cv2.aruco.detectMarkers(
                gray, dictionary(), parameters=self.parameters)
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
