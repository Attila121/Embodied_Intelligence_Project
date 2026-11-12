"""Closed-loop check of follower -> avoider -> the real guard logic around the demo obstacles.

Kinematic simulation (see avoidance_sim.py): it validates the decision logic against the
guard's real rules and a ray-cast scan, not Gazebo physics or camera tracking.
"""

from pathlib import Path
import random
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parent))
from avoidance_sim import Sim
from room_obstacles import OBSTACLES, OWNER_START, OWNER_WALK, obstacle_distance

GUARD_TURN_LIMIT = 0.55  # the guard blocks turning with anything closer than this


def run_demo_walk(avoid=True, jitter=0.0, seed=0):
    rng = random.Random(seed)

    def j(v):
        return v + rng.uniform(-jitter, jitter) if jitter else v

    obstacles = tuple(dict(o, x=j(o["x"]), y=j(o["y"])) for o in OBSTACLES)
    sim = Sim(avoid=avoid, obstacles=obstacles, owner=OWNER_START, dt=0.1, rays=180)
    stages = [sim.settle(100)]
    for waypoint in OWNER_WALK:
        if stages[-1] is None:
            break
        sim.move_owner((j(waypoint[0]), j(waypoint[1])), 0.06)
        sim.run(8.0)
        stages.append(sim.settle(100))
    per_obstacle = [min(obstacle_distance(x, y, (o,)) for x, y in sim.path) for o in obstacles]
    return sim, stages, per_obstacle


class ClosedLoopTests(unittest.TestCase):
    def test_without_avoidance_the_robot_is_stuck_at_the_first_obstacle(self):
        sim, stages, _ = run_demo_walk(avoid=False)
        self.assertIsNone(stages[0])                       # never reaches the following distance
        self.assertGreater(sim.blocked_seconds, 60)        # the guard just holds it still

    def test_demo_walk_completes_every_stage_with_clearance(self):
        sim, stages, per_obstacle = run_demo_walk()
        self.assertEqual(len(stages), 1 + len(OWNER_WALK))
        self.assertTrue(all(s is not None for s in stages), stages)
        self.assertGreater(sim.min_clearance, GUARD_TURN_LIMIT)
        self.assertLess(sim.blocked_seconds, 5.0)
        # Each obstacle is really passed near (so the walk exercises avoidance), never touched.
        for distance in per_obstacle:
            self.assertGreater(distance, GUARD_TURN_LIMIT)
            self.assertLess(distance, 1.0)

    def test_robust_to_obstacle_and_waypoint_position_error(self):
        for seed in (1, 2):
            sim, stages, _ = run_demo_walk(jitter=0.12, seed=seed)
            self.assertTrue(all(s is not None for s in stages), (seed, stages))
            self.assertGreater(sim.min_clearance, GUARD_TURN_LIMIT, seed)
            self.assertLess(sim.blocked_seconds, 8.0, seed)

    def test_avoidance_never_exceeds_the_guard_speed_limits(self):
        sim = Sim(avoid=True, owner=OWNER_START, dt=0.1, rays=180)
        for _ in range(900):
            sim.step()
            decision = sim.last_decision
            self.assertLessEqual(decision.linear, 0.12 + 1e-9)
            self.assertLessEqual(abs(decision.angular), 0.5 + 1e-9)
            self.assertGreaterEqual(decision.linear, 0.0)


if __name__ == "__main__":
    unittest.main()
