"""Floor obstacles for the room (follow-demo --obstacles); plain geometry, no ROS/OpenCV.

Shared by the scene generator, the clearance monitor and the avoidance simulation so the
Gazebo world and the checks cannot drift apart.  Low (0.30 m): the Waffle Pi LiDAR plane
(~0.17 m) sees them, but they stay below both tracking markers (0.32 m robot, 0.75 m
owner), so they cannot hide a marker from the fixed cameras.  Boxes are axis-aligned.
"""

import math

OBSTACLE_HEIGHT = 0.30
ROOM_HALF_SIZE = 3.0  # inner face of the walls

# Obstacle clearance matters here: the guard freezes the robot if a turn is attempted within
# 0.55 m of anything, so the avoider keeps ~0.7 m from every surface.  Obstacles therefore
# need roughly 2.6 m between centres (and ~0.7 m from the walls) to leave a drivable gap.
OBSTACLES = (
    {"name": "obstacle_cylinder_a", "shape": "cylinder", "x": 0.20, "y": 0.00, "radius": 0.20, "color": "Orange"},
    {"name": "obstacle_box_b", "shape": "box", "x": -1.60, "y": 2.00, "size": (0.45, 0.45), "color": "Red"},
    {"name": "obstacle_cylinder_c", "shape": "cylinder", "x": 2.00, "y": -2.00, "radius": 0.20, "color": "Orange"},
)

# Scripted owner walk for follow-demo --obstacles.  The stand is moved here before the
# follower starts so the first approach has to go around obstacle A.  Chosen and checked with
# tests/avoidance_sim.py: the robot passes within ~0.65-0.85 m of every obstacle.
OWNER_START = (2.2, 0.0)
OWNER_WALK = ((2.2, -1.2), (0.0, -2.2), (-1.8, -0.8), (-1.6, 1.0), (0.0, 2.2))


def obstacle_distance(x, y, obstacles=OBSTACLES):
    """Distance from a floor point to the nearest obstacle surface (negative inside)."""
    best = math.inf
    for spec in obstacles:
        dx, dy = x - spec["x"], y - spec["y"]
        if spec["shape"] == "cylinder":
            distance = math.hypot(dx, dy) - spec["radius"]
        else:
            half_x, half_y = spec["size"][0] / 2, spec["size"][1] / 2
            outside = math.hypot(max(abs(dx) - half_x, 0.0), max(abs(dy) - half_y, 0.0))
            distance = outside if outside > 0 else max(abs(dx) - half_x, abs(dy) - half_y)
        best = min(best, distance)
    return best


if __name__ == "__main__":  # used by scripts/follow_demo.sh
    import sys

    if sys.argv[1:] == ["start"]:
        print(*OWNER_START)
    elif sys.argv[1:] == ["walk"]:
        for waypoint in OWNER_WALK:
            print(*waypoint)
    else:
        sys.exit("usage: room_obstacles.py start|walk")
