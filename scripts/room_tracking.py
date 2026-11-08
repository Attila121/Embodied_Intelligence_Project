"""Camera-relative marker geometry and freshness/agreement checks (no ROS)."""

from dataclasses import dataclass
import math

import cv2
import numpy as np


def camera_calibration(info):
    """Validate raw camera intrinsics before estimating either marker's pose."""
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


def owner_in_base(robot, owner, marker_translation, marker_rpy):
    """T_base_owner = T_base_robot_marker * inv(T_camera_robot_marker) * T_camera_owner.

    Both detections must come from one image. The mounted robot marker's +x/+y
    axes and its pose relative to base_footprint are part of the calibration.
    Camera position in the room cancels out of this expression.
    """
    rotation, _ = cv2.Rodrigues(np.asarray(robot.rotation, np.float64))
    roll, pitch, yaw = marker_rpy
    cr, sr, cp, sp, cy, sy = (math.cos(roll), math.sin(roll), math.cos(pitch),
                             math.sin(pitch), math.cos(yaw), math.sin(yaw))
    mount_rotation = np.array([
        [cy*cp, cy*sp*sr-sy*cr, cy*sp*cr+sy*sr],
        [sy*cp, sy*sp*sr+cy*cr, sy*sp*cr-cy*sr],
        [-sp, cp*sr, cp*cr],
    ])
    delta = np.asarray(owner.translation) - np.asarray(robot.translation)
    result = mount_rotation @ rotation.T @ delta + np.asarray(marker_translation)
    if not np.isfinite(result).all():
        raise ValueError("nonfinite relative owner position")
    return result


@dataclass
class CameraObservation:
    position: np.ndarray
    stamp: float
    received: float


def fuse_observations(observations, now, timeout=0.6, max_skew=0.2,
                      max_disagreement=0.35):
    """Use a fresh camera or average agreeing, sufficiently simultaneous views."""
    fresh = [(name, obs) for name, obs in observations.items()
             if obs is not None and 0 <= now-obs.received < timeout
             and np.isfinite(obs.position).all() and math.isfinite(obs.stamp)]
    if not fresh:
        return None, None, "neither camera sees both markers"
    fresh.sort(key=lambda item: item[1].stamp, reverse=True)
    newest_stamp = fresh[0][1].stamp
    paired = [(name, obs) for name, obs in fresh if newest_stamp-obs.stamp <= max_skew]
    for _, a in paired:
        for _, b in paired:
            if np.linalg.norm(a.position[:2]-b.position[:2]) > max_disagreement:
                return None, None, "camera measurements disagree"
    names = "+".join(sorted(name for name, _ in paired))
    token = tuple(sorted((name, obs.stamp) for name, obs in paired))
    position = np.mean([obs.position for _, obs in paired], axis=0)
    return position, token, f"tracking owner ({names})"
