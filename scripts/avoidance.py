"""Reactive obstacle avoidance between the follower and the LiDAR guard (pure Python).

The follower asks for a velocity toward the owner; the guard (following_guard.py) is the
final safety gate.  The guard has two rules that shape everything here:
  * any command with a turn component, including a curved drive, needs NOTHING within
    ~0.55 m in ANY direction;
  * pure straight driving only needs a clear forward corridor.
So the avoider (a) plans paths that keep >= ~0.62 m from obstacles (an obstacle already
closer than that only has to not be approached further), so the guard never freezes the robot
beside one, and (b) uses curved driving only while nothing is closer than the guard's turning
limit, falling back to pure rotation or pure straight driving otherwise.  Distances are measured from the LiDAR, like the guard does, not from the robot centre.  It never adds motion the follower did not ask for, ignores the
owner's own stand, and leaves the guard unchanged and authoritative.

Frame: robot (base_footprint), x forward, y left, angles counter-clockwise positive.
"""

import math
import time
from dataclasses import dataclass, replace


@dataclass(frozen=True)
class AvoidParams:
    # Same meaning/defaults as the follower; used to size the approach.
    following_distance: float = 1.0
    max_linear: float = 0.12
    max_angular: float = 0.5
    turn_gain: float = 0.8
    # Planning geometry.
    lookahead: float = 1.6          # how far ahead a heading must be clear
    clearance: float = 0.62         # keep this far from obstacles (guard turns need > 0.55 m)
    close_floor: float = 0.32       # never plan closer than this to anything (guard drive: 0.30)
    close_shrink: float = 0.90      # an obstacle already inside `clearance` may be approached to 90% of its current distance
    plan_margin: float = 0.08       # headings are planned with this much extra clearance, so small heading error stays legal
    sensor_offset_x: float = -0.064  # LiDAR x offset from base_footprint (Waffle Pi)
    # When to take control from the follower.
    engage_radius: float = 0.80     # any obstacle this close -> steer (so turns stay legal)
    release_margin: float = 0.10    # hysteresis before handing control back
    turn_block_radius: float = 0.58  # guard refuses to turn with anything closer (stop 0.55)
    owner_exclusion_radius: float = 0.35  # LiDAR returns this close to the owner are the stand
    # Heading selection.
    candidate_step_deg: float = 5.0
    max_deflection_deg: float = 100.0
    commit_seconds: float = 3.0     # a chosen side is kept while avoidance stays active
    side_penalty_deg: float = 180.0  # switching sides only if the committed side has no clear heading
    min_progress: float = 0.70      # least clear distance worth driving into when nothing is fully clear
    rotate_enter: float = 0.80      # rad: turn in place (no driving) above this heading error
    rotate_exit: float = 0.60       # rad: resume driving below this
    straight_tolerance: float = 0.06  # rad: below this, drive exactly straight (no turn component)
    drive_free_min: float = 0.80    # drive straight only if the way ahead is clear this far
    escape_speed_fraction: float = 0.5


def validate_params(params):
    """Raise ValueError for settings that would make the planner unsafe or meaningless."""
    for name, value in vars(params).items():
        if not math.isfinite(value):
            raise ValueError(f"{name} must be finite")
    positive = ("following_distance", "max_linear", "max_angular", "turn_gain", "lookahead",
                "clearance", "close_floor", "close_shrink", "engage_radius", "turn_block_radius",
                "owner_exclusion_radius", "candidate_step_deg", "max_deflection_deg",
                "commit_seconds", "min_progress", "rotate_enter", "rotate_exit", "straight_tolerance",
                "drive_free_min",
                "escape_speed_fraction")
    for name in positive:
        if getattr(params, name) <= 0.0:
            raise ValueError(f"{name} must be positive")
    if params.plan_margin < 0.0 or params.release_margin < 0.0 or params.side_penalty_deg < 0.0:
        raise ValueError("margins and penalties must not be negative")
    if params.max_linear > 0.12 + 1e-9:
        raise ValueError("max_linear must not exceed the guard's 0.12 m/s limit")
    if params.max_angular > 0.5 + 1e-9:
        raise ValueError("max_angular must not exceed the guard's 0.5 rad/s limit")
    if params.close_floor >= params.clearance or params.close_shrink > 1.0:
        raise ValueError("need close_floor < clearance and close_shrink <= 1")
    if params.rotate_exit >= params.rotate_enter:
        raise ValueError("rotate_exit must be smaller than rotate_enter")
    if params.candidate_step_deg > 30.0 or params.max_deflection_deg > 175.0:
        raise ValueError("candidate step/deflection out of range")
    if params.clearance < params.turn_block_radius:
        raise ValueError("clearance must be at least turn_block_radius, or the guard will block turns")


@dataclass(frozen=True)
class Decision:
    mode: str  # stop | follow | rotate | curve | drive | escape | blocked
    linear: float = 0.0
    angular: float = 0.0
    heading: float = None


def _wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def scan_points(scan, offset_x=-0.064, require_full_circle=True):
    """LaserScan -> robot-frame points, or None if the scan is not trustworthy.

    +inf means "no return" and is skipped (as in the guard); NaN, readings below
    range_min and readings above range_max invalidate the whole scan.  ``offset_x`` is the
    sensor's x offset from base_footprint (Waffle Pi: -0.064 m).
    """
    meta = (scan.angle_min, scan.angle_increment, scan.range_min, scan.range_max)
    if not all(math.isfinite(v) for v in meta):
        return None
    step = abs(scan.angle_increment)
    if require_full_circle:
        if not 0.0 < step <= math.radians(5):
            return None
        if len(scan.ranges) * step < 2.0 * math.pi - 2.0 * step:
            return None
    points = []
    for index, distance in enumerate(scan.ranges):
        if distance == math.inf:
            continue
        if not math.isfinite(distance) or distance < scan.range_min or distance > scan.range_max:
            return None
        angle = scan.angle_min + index * scan.angle_increment
        points.append((distance * math.cos(angle) + offset_x, distance * math.sin(angle)))
    return points


def free_distance(points, heading, params, limit):
    """How far the robot can travel along ``heading`` (up to ``limit``) before breaking clearance.

    Every obstacle point needs ``clearance`` metres from the path.  A point already closer
    than that (the robot is beside something) may only be approached to ``close_shrink`` of
    its current distance and never below ``close_floor``, so leaving a tight spot is allowed
    but squeezing past things is not.
    """
    ux, uy = math.cos(heading), math.sin(heading)
    best = limit
    for x, y in points:
        s = x * ux + y * uy
        if s <= 0.0:
            continue
        lateral = abs(y * ux - x * uy)
        needed = max(params.close_floor,
                     min(params.clearance, params.close_shrink * math.hypot(x, y)))
        if lateral >= needed:
            continue
        entry = s - math.sqrt(needed * needed - lateral * lateral)
        if entry < best:
            best = max(entry, 0.0)
    return best


class Avoider:
    def __init__(self, params=None, clock=time.monotonic):
        self.params = params or AvoidParams()
        validate_params(self.params)
        self.clock = clock
        self._plan = replace(self.params, clearance=self.params.clearance + self.params.plan_margin)
        self.reset()

    def reset(self):
        self.avoiding = False
        self.rotating = False
        self.side = 0
        self.side_until = -math.inf
        self.min_dist = math.inf
        self._points = []

    # ------------------------------------------------------------------ main entry
    def update(self, desired, owner, points):
        """``desired`` = follower (linear, angular) or None; ``owner`` = (x, y) or None."""
        p = self.params
        if (desired is None or owner is None
                or not all(math.isfinite(v) for v in (*desired, *owner))
                or (desired[0] == 0.0 and desired[1] == 0.0)):
            self.reset()
            return Decision("stop")
        ox, oy = owner
        bearing = math.atan2(oy, ox)
        travel = max(0.0, math.hypot(ox, oy) - p.following_distance)
        points = [q for q in points if math.hypot(q[0] - ox, q[1] - oy) >= p.owner_exclusion_radius]
        self._points = points
        self.min_dist = min((math.hypot(q[0] - p.sensor_offset_x, q[1]) for q in points),
                            default=math.inf)

        need = min(p.lookahead, travel + 0.6)
        direct_free = free_distance(points, bearing, p, need) >= need
        threshold = p.engage_radius + (p.release_margin if self.avoiding else 0.0)
        if direct_free and self.min_dist >= threshold:
            self.avoiding = False
            self.rotating = False
            self.side = 0
            return Decision("follow", desired[0], desired[1])

        self.avoiding = True
        self.side_until = self.clock() + p.commit_seconds if self.side else -math.inf
        heading = self._choose_heading(points, bearing, need)
        if heading is None:
            self.rotating = False
            return Decision("blocked")
        return self.decide_heading(heading, travel)

    # ------------------------------------------------------------------ heading -> command
    def decide_heading(self, heading, travel):
        p = self.params
        can_turn = self.min_dist > p.turn_block_radius  # the guard refuses any turn component below this
        ahead_clear = free_distance(self._points, 0.0, p, p.drive_free_min) >= p.drive_free_min
        limit = p.rotate_exit if self.rotating else p.rotate_enter
        if abs(heading) > limit or not ahead_clear:
            self.rotating = True
            if can_turn and abs(heading) > 1e-3:
                return Decision("rotate", 0.0, self._turn_rate(heading), heading)
            return self._cannot_turn(heading)
        self.rotating = False
        linear = min(p.max_linear, 0.3 * travel)
        if linear <= 0.0:
            return Decision("stop")
        if abs(heading) <= p.straight_tolerance:
            return Decision("drive", linear, 0.0, heading)
        if can_turn:
            return Decision("curve", linear, self._turn_rate(heading), heading)
        return self._cannot_turn(heading)

    def _turn_rate(self, heading):
        p = self.params
        return max(-p.max_angular, min(p.max_angular, p.turn_gain * heading))

    def _cannot_turn(self, heading):
        """The guard would refuse a turn.  Creep straight ahead if that is clear, which moves the
        robot away from whatever is beside it; otherwise wait."""
        p = self.params
        self.rotating = False
        reach = p.turn_block_radius + 0.1
        if free_distance(self._points, 0.0, p, reach) >= reach:
            return Decision("escape", p.escape_speed_fraction * p.max_linear, 0.0, 0.0)
        return Decision("blocked", heading=heading)

    # ------------------------------------------------------------------ planning
    def _choose_heading(self, points, bearing, need):
        p = self._plan
        step = math.radians(p.candidate_step_deg)
        steps = int(math.radians(p.max_deflection_deg) / step)
        now = self.clock()
        side = self.side if now < self.side_until else 0

        if free_distance(points, bearing, p, need) >= need:
            return bearing  # straight at the owner is clear

        # Smallest clear deflection on each side; switching sides costs side_penalty_deg.
        options = []
        for sign in (1, -1):
            for k in range(1, steps + 1):
                heading = _wrap(bearing + sign * k * step)
                clear = free_distance(points, heading, p, need)
                if clear >= need:
                    penalty = math.radians(p.side_penalty_deg) if side and sign != side else 0.0
                    options.append((k * step + penalty, -clear, -sign, sign, heading))
                    break
        if options:
            _, _, _, sign, heading = min(options)
            self.side, self.side_until = sign, now + p.commit_seconds
            return heading

        # Nothing is fully clear: take the most open heading if it is worth driving into.
        best = None
        for k in range(steps + 1):
            for delta in ([0.0] if k == 0 else [k * step, -k * step]):
                heading = _wrap(bearing + delta)
                clear = free_distance(points, heading, p, need)
                if best is None or clear > best[0] + 1e-9:
                    best = (clear, heading)
        return best[1] if best and best[0] >= p.min_progress else None
