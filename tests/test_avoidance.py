"""Obstacle-avoidance decision logic (pure Python, no ROS).

Conventions: robot frame, x forward, y left; headings/bearings are counter-clockwise
positive; a positive angular command turns left.
"""

import math
from pathlib import Path
from types import SimpleNamespace
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))
from avoidance import AvoidParams, Avoider, free_distance, scan_points

P = AvoidParams()


def circle(cx, cy, r=0.15, n=36):
    return [(cx + r * math.cos(2 * math.pi * k / n), cy + r * math.sin(2 * math.pi * k / n))
            for k in range(n)]


def follower_command(owner, params=P):
    """What owner_follower.py would command for this owner position."""
    x, y = owner
    distance, bearing = math.hypot(x, y), math.atan2(y, x)
    linear = angular = 0.0
    if distance > params.following_distance + 0.1:
        if abs(bearing) > 0.05:
            angular = max(-params.max_angular, min(params.max_angular, 0.8 * bearing))
        if abs(bearing) < 0.35:
            linear = min(params.max_linear, 0.3 * (distance - params.following_distance))
    return linear, angular


class Clock:
    def __init__(self):
        self.now = 100.0

    def __call__(self):
        return self.now


def decide(owner, points, desired=None, avoider=None):
    avoider = avoider or Avoider(P, Clock())
    return avoider.update(follower_command(owner) if desired is None else desired, owner, points)


class PassThroughTests(unittest.TestCase):
    def test_empty_scan_passes_the_follower_command_unchanged(self):
        d = decide((2.5, 0.4), [])
        self.assertEqual((d.mode, (d.linear, d.angular)), ("follow", follower_command((2.5, 0.4))))

    def test_obstacle_well_off_the_path_is_ignored(self):
        d = decide((2.5, 0.0), circle(1.4, 1.5))
        self.assertEqual(d.mode, "follow")

    def test_returns_from_the_owner_stand_are_not_obstacles(self):
        owner = (2.0, 0.0)
        stand = [(owner[0] - 0.08, owner[1] + dy / 100) for dy in range(-8, 9, 2)]
        self.assertEqual(decide(owner, stand).mode, "follow")


class NeverAddsMotionTests(unittest.TestCase):
    def test_stopped_follower_stays_stopped_with_obstacles_present(self):
        d = decide((2.5, 0.0), circle(1.0, 0.0), desired=(0.0, 0.0))
        self.assertEqual((d.mode, d.linear, d.angular), ("stop", 0.0, 0.0))

    def test_missing_command_or_owner_stops(self):
        for desired, owner in ((None, (2.5, 0.0)), ((0.1, 0.0), None)):
            d = Avoider(P, Clock()).update(desired, owner, [])
            self.assertEqual((d.mode, d.linear, d.angular), ("stop", 0.0, 0.0))

    def test_speed_never_exceeds_the_follower_limits(self):
        for owner in ((1.4, 0.0), (3.0, 0.2), (2.5, -0.5)):
            for points in ([], circle(1.0, 0.1), circle(0.4, 0.8)):
                d = decide(owner, points)
                self.assertLessEqual(d.linear, P.max_linear + 1e-9)
                self.assertLessEqual(d.linear, 0.3 * (math.hypot(*owner) - P.following_distance) + 1e-9)
                self.assertLessEqual(abs(d.angular), P.max_angular + 1e-9)
                self.assertGreaterEqual(d.linear, 0.0)


class SteeringTests(unittest.TestCase):
    def test_obstacle_on_the_line_gives_pure_rotation_not_a_curve(self):
        d = decide((2.5, 0.0), circle(1.0, 0.0))
        self.assertEqual(d.mode, "rotate")
        self.assertEqual(d.linear, 0.0)
        self.assertNotEqual(d.angular, 0.0)
        self.assertLessEqual(abs(d.angular), P.max_angular)

    def test_steers_to_the_side_with_the_shorter_detour(self):
        self.assertLess(decide((2.5, 0.0), circle(1.0, 0.1)).angular, 0.0)    # obstacle left -> turn right
        self.assertGreater(decide((2.5, 0.0), circle(1.0, -0.1)).angular, 0.0)  # obstacle right -> turn left

    def test_drives_straight_with_exactly_zero_turn_once_aligned(self):
        d = decide((2.5, 0.0), circle(0.0, 0.88))  # beside the path (0.73 m, outside the margin), inside the engage radius
        self.assertEqual(d.mode, "drive")
        self.assertGreater(d.linear, 0.0)
        self.assertEqual(d.angular, 0.0)

    def test_chosen_heading_leaves_the_clearance_the_guard_needs(self):
        points = circle(1.0, 0.1)
        d = decide((2.5, 0.0), points)
        heading = d.heading
        self.assertIsNotNone(heading)
        self.assertGreaterEqual(free_distance(points, heading, P, 1.2), 1.2)
        # Every obstacle point stays further than the guard's 0.55 m from the chosen line.
        for x, y in points:
            s = x * math.cos(heading) + y * math.sin(heading)
            lateral = abs(-x * math.sin(heading) + y * math.cos(heading))
            if s > 0:
                self.assertGreater(lateral, 0.55)

    def test_side_choice_is_held_against_small_perturbations(self):
        clock = Clock()
        avoider = Avoider(P, clock)
        first = avoider.update(follower_command((2.5, 0.0)), (2.5, 0.0), circle(1.0, 0.0))
        for offset in (0.02, -0.02, 0.01, -0.01):
            clock.now += 0.1
            again = avoider.update(follower_command((2.5, 0.0)), (2.5, 0.0), circle(1.0, offset))
            self.assertEqual(math.copysign(1, again.angular), math.copysign(1, first.angular))

    def test_small_heading_error_while_clear_is_a_smooth_curve(self):
        avoider = Avoider(P, Clock())
        owner, points = (2.5, 0.0), circle(0.0, 0.88)  # way ahead clear, nothing within the guard's turn limit
        self.assertEqual(avoider.update(follower_command(owner), owner, points).mode, "drive")
        d = avoider.decide_heading(0.3, travel=1.5)
        self.assertEqual(d.mode, "curve")
        self.assertGreater(d.linear, 0.0)
        self.assertGreater(d.angular, 0.0)                       # heading is to the left
        self.assertLess(avoider.decide_heading(-0.3, travel=1.5).angular, 0.0)

    def test_rotation_hysteresis_prevents_start_stop_chatter(self):
        avoider = Avoider(P, Clock())
        owner, points = (2.5, 0.0), circle(0.0, 0.88)
        avoider.update(follower_command(owner), owner, points)
        # Between rotate_exit (0.6) and rotate_enter (0.8) it keeps whatever it was doing.
        self.assertEqual(avoider.decide_heading(0.7, travel=1.5).mode, "curve")
        self.assertEqual(avoider.decide_heading(0.9, travel=1.5).mode, "rotate")
        self.assertEqual(avoider.decide_heading(0.7, travel=1.5).mode, "rotate")  # still rotating
        self.assertEqual(avoider.decide_heading(0.5, travel=1.5).mode, "curve")   # resumes below exit

    def test_never_drives_into_a_blocked_way_even_with_a_small_heading_error(self):
        avoider = Avoider(P, Clock())
        owner, points = (2.5, 0.0), circle(1.0, 0.0)
        avoider.update(follower_command(owner), owner, points)
        d = avoider.decide_heading(0.05, travel=1.5)  # nearly aligned, but an obstacle is ahead
        self.assertEqual(d.linear, 0.0)

    def test_pass_through_resumes_only_after_clear_margin(self):
        avoider = Avoider(P, Clock())
        owner = (2.5, 0.0)
        near = circle(0.3, 0.72)                      # nearest point ~0.62 m: engage
        self.assertNotEqual(avoider.update(follower_command(owner), owner, near).mode, "follow")
        borderline = circle(0.3, 0.97)                # nearest ~0.85 m: inside the release margin
        self.assertNotEqual(avoider.update(follower_command(owner), owner, borderline).mode, "follow")
        far = circle(0.3, 1.3)                        # clearly away
        self.assertEqual(avoider.update(follower_command(owner), owner, far).mode, "follow")


class GuardConstraintTests(unittest.TestCase):
    def test_boxed_in_stops_instead_of_pushing(self):
        wall = [(0.6 * math.cos(math.radians(a)), 0.6 * math.sin(math.radians(a)))
                for a in range(-150, 151, 3)]
        d = decide((2.5, 0.0), wall)
        self.assertEqual((d.mode, d.linear, d.angular), ("blocked", 0.0, 0.0))

    def test_never_commands_a_turn_the_guard_would_refuse(self):
        # Owner far to the left (needs a big turn) but an obstacle sits 0.5 m to the right:
        # the guard blocks all turning that close, so the avoider must not ask for one.
        owner = (0.5, 2.5)
        d = decide((0.5, 2.5), circle(0.0, -0.5, r=0.05))
        self.assertEqual(d.angular, 0.0)

    def test_scan_conversion_applies_the_sensor_offset_and_skips_no_return(self):
        scan = SimpleNamespace(ranges=[1.0, math.inf, 2.0, 1.5], angle_min=0.0,
                               angle_increment=math.pi / 2, range_min=0.12, range_max=3.5)
        pts = scan_points(scan, offset_x=-0.064, require_full_circle=False)
        self.assertEqual(len(pts), 3)
        self.assertAlmostEqual(pts[0][0], 1.0 - 0.064)
        self.assertAlmostEqual(pts[0][1], 0.0)
        self.assertAlmostEqual(pts[1][0], -0.064 + 2.0 * math.cos(math.pi), places=6)

    def test_invalid_scans_are_rejected(self):
        good = dict(angle_min=0.0, angle_increment=math.radians(1), range_min=0.12, range_max=3.5)
        full = [1.0] * 360
        self.assertIsNotNone(scan_points(SimpleNamespace(ranges=full, **good)))
        for bad in (full[:10] + [math.nan] + full[11:], full[:10] + [0.05] + full[11:],
                    full[:10] + [9.0] + full[11:], full[:90]):
            self.assertIsNone(scan_points(SimpleNamespace(ranges=bad, **good)))
        coarse = dict(good, angle_increment=math.radians(10))
        self.assertIsNone(scan_points(SimpleNamespace(ranges=[1.0] * 36, **coarse)))


if __name__ == "__main__":
    unittest.main()
