"""Demo obstacle layout sanity (pure geometry; no ROS or OpenCV)."""

import math
from pathlib import Path
import subprocess
import sys
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))
from room_obstacles import (OBSTACLE_HEIGHT, OBSTACLES, OWNER_START, OWNER_WALK, ROOM_HALF_SIZE,
                            obstacle_distance)

ROBOT_START = (-1.0, 0.0)
AVOID_CLEARANCE = 0.62       # avoidance.py keeps this far from every surface
OWNER_LIMIT = 2.4            # room_target.py refuses owner positions outside +/- this


def half_extent(spec):
    return spec["radius"] if spec["shape"] == "cylinder" else math.hypot(*spec["size"]) / 2


class LayoutTests(unittest.TestCase):
    def test_names_are_unique_and_shapes_valid(self):
        names = [o["name"] for o in OBSTACLES]
        self.assertEqual(len(names), len(set(names)))
        for spec in OBSTACLES:
            self.assertIn(spec["shape"], {"cylinder", "box"})

    def test_obstacles_are_low_enough_not_to_hide_either_marker(self):
        self.assertLess(OBSTACLE_HEIGHT, 0.32)   # robot marker height; owner marker is at 0.75
        self.assertGreater(OBSTACLE_HEIGHT, 0.20)  # but above the LiDAR plane (~0.17 m)

    def test_obstacles_leave_a_drivable_gap_between_each_other(self):
        for i, a in enumerate(OBSTACLES):
            for b in OBSTACLES[i + 1:]:
                gap = math.hypot(a["x"] - b["x"], a["y"] - b["y"]) - half_extent(a) - half_extent(b)
                # The avoider needs AVOID_CLEARANCE on both sides of a path between them.
                self.assertGreater(gap, 2 * AVOID_CLEARANCE + 0.3, (a["name"], b["name"]))

    def test_obstacles_are_kept_off_the_walls(self):
        # The robot never has to squeeze between an obstacle and a wall (the closed-loop test
        # proves the walk works); this only guards against obstacles pushed into a wall.
        for spec in OBSTACLES:
            reach = ROOM_HALF_SIZE - max(abs(spec["x"]), abs(spec["y"])) - half_extent(spec)
            self.assertGreater(reach, 0.5)

    def test_robot_start_is_clear_of_obstacles(self):
        self.assertGreater(obstacle_distance(*ROBOT_START), AVOID_CLEARANCE + 0.2)

    def test_owner_walk_stays_in_range_and_clear_of_obstacles(self):
        for waypoint in (OWNER_START, *OWNER_WALK):
            self.assertLessEqual(max(abs(waypoint[0]), abs(waypoint[1])), OWNER_LIMIT)
            self.assertGreater(obstacle_distance(*waypoint), 0.5)
        for a, b in zip((OWNER_START, *OWNER_WALK), OWNER_WALK):
            self.assertGreater(math.dist(a, b), 0.5)

    def test_first_approach_is_blocked_by_an_obstacle(self):
        """The point of the layout: the straight line from robot to owner hits an obstacle."""
        samples = [(ROBOT_START[0] + (OWNER_START[0] - ROBOT_START[0]) * k / 200,
                    ROBOT_START[1] + (OWNER_START[1] - ROBOT_START[1]) * k / 200) for k in range(201)]
        self.assertLess(min(obstacle_distance(*s) for s in samples), 0.2)


class DistanceTests(unittest.TestCase):
    def test_cylinder_and_box_distances(self):
        cylinder = {"shape": "cylinder", "x": 0.0, "y": 0.0, "radius": 0.2}
        box = {"shape": "box", "x": 0.0, "y": 0.0, "size": (0.4, 0.4)}
        self.assertAlmostEqual(obstacle_distance(1.0, 0.0, (cylinder,)), 0.8)
        self.assertAlmostEqual(obstacle_distance(0.0, 1.0, (box,)), 0.8)
        self.assertAlmostEqual(obstacle_distance(1.0, 1.0, (box,)), math.hypot(0.8, 0.8))
        self.assertLess(obstacle_distance(0.0, 0.0, (cylinder,)), 0)
        self.assertLess(obstacle_distance(0.05, 0.05, (box,)), 0)

    def test_command_line_outputs_for_the_shell_script(self):
        start = subprocess.run([sys.executable, str(ROOT / "scripts/room_obstacles.py"), "start"],
                               capture_output=True, text=True, check=True).stdout.split()
        walk = subprocess.run([sys.executable, str(ROOT / "scripts/room_obstacles.py"), "walk"],
                              capture_output=True, text=True, check=True).stdout.split()
        self.assertEqual(tuple(map(float, start)), OWNER_START)
        self.assertEqual(len(walk), 2 * len(OWNER_WALK))


if __name__ == "__main__":
    unittest.main()
