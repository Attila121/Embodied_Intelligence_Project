"""Planar rig conversion and receive-time validity, without ROS dependencies."""

from dataclasses import dataclass
import math
import time


def owner_in_robot(robot, owner, heading_offset=0.0):
    """Room coordinates -> robot x forward / y left, in metres."""
    dx, dy = owner[0] - robot[0], owner[1] - robot[1]
    heading = robot[2] + heading_offset
    c, s = math.cos(heading), math.sin(heading)
    return c*dx + s*dy, -s*dx + c*dy


@dataclass(frozen=True)
class RigGeometry:
    plane_width_m: float = 3.60
    plane_height_m: float = 2.25
    field_margin_m: float = 0.10
    robot_heading_offset_rad: float = 0.0
    robot_marker_height_m: float = 0.0
    robot_camera_height_m: float = 0.0
    robot_camera_foot_x_m: float = 0.0
    robot_camera_foot_y_m: float = 0.0

    def __post_init__(self):
        if not all(math.isfinite(value) for value in vars(self).values()):
            raise ValueError("geometry parameters must be finite")
        if self.plane_width_m <= 0 or self.plane_height_m <= 0:
            raise ValueError("plane dimensions must be positive")
        if min(self.field_margin_m, self.robot_marker_height_m, self.robot_camera_height_m) < 0:
            raise ValueError("field margin and heights must be nonnegative")
        if (self.robot_marker_height_m > 0
                and self.robot_camera_height_m <= self.robot_marker_height_m):
            raise ValueError("parallax correction requires camera height > marker height")

    def _in_field(self, pose):
        x, y, theta = pose
        margin = self.field_margin_m
        return (all(math.isfinite(v) for v in pose)
                and -margin <= x <= self.plane_width_m + margin
                and -margin <= y <= self.plane_height_m + margin)

    def robot_to_room(self, pose):
        if not all(math.isfinite(v) for v in pose):
            return None
        x, y, theta = pose
        sx, sy = self.plane_width_m / 3.60, self.plane_height_m / 2.25
        x, y = sx*x, sy*y
        theta = math.atan2(sy*math.sin(theta), sx*math.cos(theta))
        if self.robot_marker_height_m > 0:
            factor = 1.0 - self.robot_marker_height_m / self.robot_camera_height_m
            fx, fy = self.robot_camera_foot_x_m, self.robot_camera_foot_y_m
            x, y = fx + factor*(x-fx), fy + factor*(y-fy)
        result = (x, y, theta)
        return result if self._in_field(result) else None

    def human_to_room(self, pose):
        if not all(math.isfinite(v) for v in pose):
            return None
        x, y, theta = pose
        sx, sy = self.plane_width_m / 3.60, self.plane_height_m / 2.10
        result = (sx*x, self.plane_height_m - sy*y,
                  math.atan2(-sy*math.sin(theta), sx*math.cos(theta)))
        return result if self._in_field(result) else None


class RigState:
    """Latest valid samples. Timer reads never change their receive timestamps.

    The ROS wrapper uses a single-threaded executor. Equal consecutive poses
    count as new messages, since a stationary robot can produce equal poses.
    Receipt freshness therefore cannot detect upstream cached-pose replay.
    """

    def __init__(self, geometry=None, owner_timeout=0.5, robot_timeout=0.5,
                 clock=time.monotonic):
        if not all(math.isfinite(v) and v > 0 for v in (owner_timeout, robot_timeout)):
            raise ValueError("input timeouts must be positive and finite")
        self.geometry = geometry if geometry is not None else RigGeometry()
        self.owner_timeout, self.robot_timeout = owner_timeout, robot_timeout
        self.clock = clock
        self.owner = self.robot = None

    def receive_owner(self, pose):
        converted = self.geometry.human_to_room(pose)
        self.owner = (converted, self.clock()) if converted is not None else None
        return converted is not None

    def receive_robot(self, pose):
        converted = self.geometry.robot_to_room(pose)
        self.robot = (converted, self.clock()) if converted is not None else None
        return converted is not None

    def relative_position(self):
        now = self.clock()
        for sample, timeout in ((self.owner, self.owner_timeout),
                                (self.robot, self.robot_timeout)):
            if sample is None or not 0 <= now - sample[1] < timeout:
                return None
        result = owner_in_robot(self.robot[0], self.owner[0],
                                self.geometry.robot_heading_offset_rad)
        return result if all(math.isfinite(v) for v in result) else None
