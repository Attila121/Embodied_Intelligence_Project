"""Rig coordinate and input-validity regressions; no ROS or camera required."""

import math
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))
from rig_geometry import RigGeometry, RigState, owner_in_robot


class RigGeometryTests(unittest.TestCase):
    def test_human_corners_match_robot_room_corners(self):
        geometry = RigGeometry()
        # Physical A=BL, B=BR, C=TR, D=TL in the common room frame.
        for human, room in (((0, 2.10), (0, 0)), ((3.60, 2.10), (3.60, 0)),
                            ((3.60, 0), (3.60, 2.25)), ((0, 0), (0, 2.25))):
            self.assertEqual(geometry.human_to_room((*human, 0))[:2], room)

    def test_measured_room_size_rescales_both_publishers(self):
        geometry = RigGeometry(plane_width_m=7.2, plane_height_m=4.5)
        self.assertEqual(geometry.robot_to_room((1.8, 1.125, 0))[:2], (3.6, 2.25))
        self.assertEqual(geometry.human_to_room((1.8, 1.05, 0))[:2], (3.6, 2.25))

    def test_robot_diagonal_heading_changes_with_unequal_axis_scales(self):
        geometry = RigGeometry(plane_width_m=7.2)
        self.assertAlmostEqual(geometry.robot_to_room((1, 1, math.pi/4))[2],
                               math.atan2(1, 2))

    def test_human_heading_is_reflected_and_rescaled(self):
        geometry = RigGeometry()
        self.assertAlmostEqual(geometry.human_to_room((1, 1, math.pi/2))[2], -math.pi/2)
        self.assertAlmostEqual(geometry.human_to_room((1, 1, math.pi/4))[2],
                               math.atan2(-2.25/2.10, 1))

    def test_relative_coordinates_for_rotated_robot(self):
        for yaw, owner, expected in ((0, (3, 1), (2, 0)),
                                     (math.pi/2, (1, 3), (2, 0)),
                                     (math.pi, (-1, 1), (2, 0)),
                                     (0, (1, 3), (0, 2)),
                                     (0, (1, -1), (0, -2)),
                                     (0, (-1, 1), (-2, 0))):
            with self.subTest(yaw=yaw, owner=owner):
                result = owner_in_robot((1, 1, yaw), owner)
                for actual, wanted in zip(result, expected):
                    self.assertAlmostEqual(actual, wanted)

    def test_heading_offset_is_applied_after_plane_scaling(self):
        result = owner_in_robot((1, 1, 0), (3, 1), math.pi)
        self.assertAlmostEqual(result[0], -2)
        self.assertAlmostEqual(result[1], 0)

    def test_optional_parallax_correction_uses_camera_foot_and_height(self):
        geometry = RigGeometry(robot_marker_height_m=0.5, robot_camera_height_m=2.0,
                               robot_camera_foot_x_m=1.0, robot_camera_foot_y_m=1.0)
        self.assertEqual(geometry.robot_to_room((3, 2, 0)), (2.5, 1.75, 0))
        self.assertEqual(RigGeometry().robot_to_room((3, 2, 0)), (3, 2, 0))

    def test_field_margin_and_nonfinite_input(self):
        geometry = RigGeometry(field_margin_m=0.1)
        self.assertIsNotNone(geometry.robot_to_room((-0.05, 1, 0)))
        for pose in ((-0.11, 1, 0), (3.71, 1, 0), (1, 2.36, 0),
                     (1, -0.11, 0), (math.nan, 1, 0), (1, math.inf, 0),
                     (1, 1, -math.inf)):
            self.assertIsNone(geometry.robot_to_room(pose))
        for pose in ((0, -1, 0), (4, 0, 0), (1, math.nan, 0), (1, 1, math.nan)):
            self.assertIsNone(geometry.human_to_room(pose))

    def test_invalid_configuration_is_rejected(self):
        for settings in (dict(plane_width_m=0), dict(plane_height_m=math.nan),
                         dict(field_margin_m=-1), dict(robot_heading_offset_rad=math.inf),
                         dict(robot_marker_height_m=-0.1), dict(robot_marker_height_m=0.5),
                         dict(robot_marker_height_m=2, robot_camera_height_m=2),
                         dict(robot_camera_foot_x_m=math.nan)):
            with self.subTest(settings=settings), self.assertRaises(ValueError):
                RigGeometry(**settings)


class RigStateTests(unittest.TestCase):
    def setUp(self):
        self.now = 100.0
        self.state = RigState(clock=lambda: self.now)

    def ready(self):
        self.assertTrue(self.state.receive_robot((0.5, 1.125, 0)))
        self.assertTrue(self.state.receive_owner((2.5, 1.05, 0)))

    def test_both_inputs_are_required(self):
        self.assertIsNone(self.state.relative_position())
        self.state.receive_owner((2.5, 1.05, 0))
        self.assertIsNone(self.state.relative_position())
        self.state.receive_robot((0.5, 1.125, 0))
        self.assertEqual(self.state.relative_position(), (2.0, 0.0))

    def test_owner_timeout_while_robot_continues(self):
        self.ready()
        self.now += 0.5
        self.state.receive_robot((0.5, 1.125, 0))
        self.assertIsNone(self.state.relative_position())

    def test_robot_timeout_while_owner_continues(self):
        self.ready()
        self.now += 0.5
        self.state.receive_owner((2.5, 1.05, 0))
        self.assertIsNone(self.state.relative_position())

    def test_timer_reads_do_not_refresh_input_age(self):
        self.ready()
        for _ in range(4):
            self.now += 0.1
            self.assertEqual(self.state.relative_position(), (2.0, 0.0))
        self.now += 0.11
        self.assertIsNone(self.state.relative_position())

    def test_each_invalid_input_immediately_clears_its_previous_sample(self):
        for source in ("owner", "robot"):
            for pose in ((math.nan, 1, 0), (1, 1, math.inf), (100, 1, 0)):
                with self.subTest(source=source, pose=pose):
                    self.ready()
                    self.assertFalse(getattr(self.state, "receive_" + source)(pose))
                    self.assertIsNone(self.state.relative_position())

    def test_fresh_samples_recover_after_loss(self):
        self.ready()
        self.now += 1
        self.assertIsNone(self.state.relative_position())
        self.ready()
        self.assertEqual(self.state.relative_position(), (2.0, 0.0))

    def test_separate_timeouts(self):
        self.state = RigState(owner_timeout=0.3, robot_timeout=0.8, clock=lambda: self.now)
        self.ready()
        self.now += 0.4
        self.state.receive_owner((2.5, 1.05, 0))
        self.assertEqual(self.state.relative_position(), (2.0, 0.0))

    def test_invalid_timeouts_are_rejected(self):
        for setting in (dict(owner_timeout=0), dict(robot_timeout=math.inf)):
            with self.assertRaises(ValueError):
                RigState(**setting)


if __name__ == "__main__":
    unittest.main()
