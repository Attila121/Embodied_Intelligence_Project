"""Closed-loop kinematic simulation of follower -> avoider -> guard in the obstacle room.

Uses the REAL guard logic (following_guard.CommandGate; ROS imports are stubbed when ROS is
not installed), the follower's exact command formula, and a ray-cast 360-degree scan of the
same obstacles, walls and owner stand as the Gazebo world.  Not a replacement for the Gazebo
run: it has no sensor noise, camera tracking or physics.  Run directly to print a walk summary:

    python tests/avoidance_sim.py            # with avoidance
    python tests/avoidance_sim.py --baseline # follower + guard only
"""

import math
from pathlib import Path
import sys
from types import ModuleType, SimpleNamespace

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))
from avoidance import AvoidParams, Avoider, scan_points
from room_obstacles import OBSTACLES, OWNER_START, OWNER_WALK, ROOM_HALF_SIZE, obstacle_distance

ROBOT_CONTACT_RADIUS = 0.22   # Waffle Pi circumscribed radius ~0.21 m
SENSOR_OFFSET_X = -0.064
OWNER_STAND_HALF = 0.08


def load_guard():
    try:
        import following_guard
        return following_guard
    except ImportError:
        class Stub(ModuleType):
            def __getattr__(self, name):
                if name.startswith("__"):
                    raise AttributeError(name)
                return type(name, (), {})

        for name in ("rclpy", "rclpy.clock", "rclpy.node", "rclpy.qos", "rclpy.signals",
                     "geometry_msgs", "geometry_msgs.msg", "sensor_msgs", "sensor_msgs.msg"):
            sys.modules[name] = Stub(name)
        import following_guard
        return following_guard


def follower_command(owner, following_distance=1.0, max_linear=0.12, max_angular=0.5):
    """owner_follower.py's update(), for a fresh visible owner at robot-frame (x, y)."""
    x, y = owner
    distance, bearing = math.hypot(x, y), math.atan2(y, x)
    linear = angular = 0.0
    if distance > following_distance + 0.1:
        if abs(bearing) > 0.05:
            angular = max(-max_angular, min(max_angular, 0.8 * bearing))
        if abs(bearing) < 0.35:
            linear = min(max_linear, 0.3 * (distance - following_distance))
    return linear, angular


def _ray_circle(ox, oy, dx, dy, cx, cy, r):
    fx, fy = ox - cx, oy - cy
    b = fx * dx + fy * dy
    c = fx * fx + fy * fy - r * r
    disc = b * b - c
    if disc < 0:
        return math.inf
    root = math.sqrt(disc)
    t = -b - root
    if t > 0:
        return t
    t = -b + root
    return t if t > 0 else math.inf


def _ray_box(ox, oy, dx, dy, cx, cy, hx, hy):
    t_min, t_max = -math.inf, math.inf
    for o, d, c, h in ((ox, dx, cx, hx), (oy, dy, cy, hy)):
        if abs(d) < 1e-12:
            if abs(o - c) > h:
                return math.inf
            continue
        t1, t2 = (c - h - o) / d, (c + h - o) / d
        t_min, t_max = max(t_min, min(t1, t2)), min(t_max, max(t1, t2))
    if t_max < max(t_min, 0.0):
        return math.inf
    return t_min if t_min > 0 else t_max


class Sim:
    def __init__(self, avoid=True, obstacles=OBSTACLES, params=None, dt=0.05,
                 start=(-1.0, 0.0, 0.0), owner=(1.0, 0.0), rays=360):
        guard = load_guard()
        self.t = 0.0
        self.dt = dt
        self.gate = guard.CommandGate(clock=lambda: self.t)
        self.avoider = Avoider(params or AvoidParams(), lambda: self.t) if avoid else None
        self.obstacles = obstacles
        self.x, self.y, self.yaw = start
        self.owner = owner
        self.min_clearance = math.inf
        self.blocked_seconds = 0.0
        self.reasons = {}
        self.last_decision = None
        self.rays = rays
        self.path = []

    # -- world
    def owner_relative(self):
        dx, dy = self.owner[0] - self.x, self.owner[1] - self.y
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        return c * dx + s * dy, -s * dx + c * dy

    def scan(self):
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        ox, oy = self.x + c * SENSOR_OFFSET_X, self.y + s * SENSOR_OFFSET_X
        ranges = []
        step = 360.0 / self.rays
        for i in range(self.rays):
            angle = self.yaw + math.radians(i * step)
            dx, dy = math.cos(angle), math.sin(angle)
            t = math.inf
            for spec in self.obstacles:
                if spec["shape"] == "cylinder":
                    t = min(t, _ray_circle(ox, oy, dx, dy, spec["x"], spec["y"], spec["radius"]))
                else:
                    t = min(t, _ray_box(ox, oy, dx, dy, spec["x"], spec["y"],
                                        spec["size"][0] / 2, spec["size"][1] / 2))
            t = min(t, _ray_box(ox, oy, dx, dy, *self.owner, OWNER_STAND_HALF, OWNER_STAND_HALF))
            for wall in (ROOM_HALF_SIZE, -ROOM_HALF_SIZE):
                if abs(dx) > 1e-12 and (wall - ox) / dx > 0:
                    t = min(t, (wall - ox) / dx)
                if abs(dy) > 1e-12 and (wall - oy) / dy > 0:
                    t = min(t, (wall - oy) / dy)
            ranges.append(t if 0.12 <= t <= 3.5 else math.inf)
        return SimpleNamespace(header=SimpleNamespace(frame_id="base_scan"), angle_min=0.0,
                               angle_increment=math.radians(step), range_min=0.12, range_max=3.5,
                               ranges=ranges)

    # -- one control cycle
    def step(self):
        scan = self.scan()
        relative = self.owner_relative()
        desired = follower_command(relative)
        command = desired
        if self.avoider is not None:
            points = scan_points(scan, SENSOR_OFFSET_X)
            if points is None:
                command = (0.0, 0.0)
            else:
                self.last_decision = self.avoider.update(desired, relative, points)
                command = (self.last_decision.linear, self.last_decision.angular)
        twist = SimpleNamespace(linear=SimpleNamespace(x=command[0], y=0.0, z=0.0),
                                angular=SimpleNamespace(x=0.0, y=0.0, z=command[1]))
        self.gate.receive_command(twist)
        self.gate.receive_scan(scan)
        linear, angular, reason = self.gate.evaluate()
        self.reasons[reason] = self.reasons.get(reason, 0) + 1
        if desired != (0.0, 0.0) and (linear, angular) == (0.0, 0.0):
            self.blocked_seconds += self.dt
        self.yaw += angular * self.dt
        self.x += linear * math.cos(self.yaw) * self.dt
        self.y += linear * math.sin(self.yaw) * self.dt
        self.t += self.dt
        self.min_clearance = min(self.min_clearance, obstacle_distance(self.x, self.y, self.obstacles))
        self.path.append((self.x, self.y))
        return reason

    def distance(self):
        return math.hypot(*self.owner_relative())

    def run(self, seconds):
        for _ in range(int(seconds / self.dt)):
            self.step()

    # -- owner motion and demo-style settling
    def move_owner(self, target, speed=0.06):
        sx, sy = self.owner
        length = math.hypot(target[0] - sx, target[1] - sy)
        steps = max(1, int(length / speed / self.dt))
        for k in range(1, steps + 1):
            f = k / steps
            self.owner = (sx + (target[0] - sx) * f, sy + (target[1] - sy) * f)
            self.step()

    def settle(self, timeout=120.0, low=0.9, high=1.2, hold=3.0):
        """Seconds until the owner distance stayed in [low, high] for ``hold``; None on timeout."""
        start, since = self.t, None
        while self.t - start < timeout:
            self.step()
            if low <= self.distance() <= high:
                since = self.t if since is None else since
                if self.t - since >= hold:
                    return self.t - start
            else:
                since = None
        return None

    def walk(self, waypoints, speed=0.06, pause=8.0, settle_timeout=120.0):
        results = []
        for waypoint in waypoints:
            self.move_owner(waypoint, speed)
            self.run(pause)
            results.append((waypoint, self.settle(settle_timeout), round(self.distance(), 2),
                            round(self.min_clearance, 2)))
        return results


def main(argv):
    avoid = "--baseline" not in argv
    sim = Sim(avoid=avoid, owner=OWNER_START, dt=0.1, rays=180)
    print("avoidance:", "on" if avoid else "off (baseline)")
    first = sim.settle(180)
    print(f"initial approach: settled in {first and round(first, 1)} s, clearance {sim.min_clearance:.2f} m")
    for waypoint, settled, distance, clearance in sim.walk(OWNER_WALK):
        print(f"-> {waypoint}: settled in {settled and round(settled, 1)} s, "
              f"distance {distance} m, min clearance so far {clearance} m")
    print(f"min clearance {sim.min_clearance:.2f} m (contact below {ROBOT_CONTACT_RADIUS}), "
          f"blocked {sim.blocked_seconds:.1f} s, sim time {sim.t:.0f} s")
    print("guard reasons:", {k: v for k, v in sorted(sim.reasons.items(), key=lambda kv: -kv[1])[:4]})


if __name__ == "__main__":
    main(sys.argv[1:])
