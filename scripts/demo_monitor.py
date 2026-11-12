#!/usr/bin/env python3
"""Bounded readiness/convergence checks for the simulation following demo.

Every subcommand waits for RECEIVED messages (not merely existing topics), exits 0 on
success and 1 on timeout, and prints what was missing.  Time is wall time.

  wait-images   one image from each room camera
  wait-visible  /owner/visible is true
  wait-settled  owner distance stays within [low, high] metres for --hold seconds
  wait-stopped  /owner/visible is false AND /cmd_vel is exactly zero for --hold seconds
  watch-clearance  run until stopped; exit 3 if the robot comes closer than --minimum to an
                obstacle (ground truth from Gazebo, a test check only: nothing else uses it)
"""

import argparse
import math
import sys
import time

SETTLED_LOW, SETTLED_HIGH = 0.90, 1.20  # follower holds ~1.0-1.1 m; allow measurement noise
ZERO_TOLERANCE = 1e-3


class SettleTracker:
    """True once the distance has stayed inside [low, high] with fresh samples for ``hold`` s."""

    def __init__(self, low=SETTLED_LOW, high=SETTLED_HIGH, hold=3.0, max_age=0.5):
        self.low, self.high, self.hold, self.max_age = low, high, hold, max_age
        self.since = None

    def update(self, now, distance, age):
        if distance is None or age > self.max_age or not self.low <= distance <= self.high:
            self.since = None
            return False
        if self.since is None:
            self.since = now
        return now - self.since >= self.hold


class StopTracker:
    """True once visibility is false and the commanded velocity has been zero for ``hold`` s."""

    def __init__(self, hold=1.0, max_visible_age=1.0, max_command_age=0.5):
        self.hold, self.max_visible_age, self.max_command_age = hold, max_visible_age, max_command_age
        self.since = None

    def update(self, now, visible, visible_age, command, command_age):
        """``command`` is (linear_x, angular_z) or None; ages are seconds since receipt."""
        stopped = (visible is False and visible_age <= self.max_visible_age
                   and command is not None and command_age <= self.max_command_age
                   and abs(command[0]) <= ZERO_TOLERANCE and abs(command[1]) <= ZERO_TOLERANCE)
        if not stopped:
            self.since = None
            return False
        if self.since is None:
            self.since = now
        return now - self.since >= self.hold


class Monitor:
    def __init__(self, args):
        import rclpy
        from geometry_msgs.msg import Point, Twist
        from rclpy.qos import QoSProfile, ReliabilityPolicy
        from sensor_msgs.msg import Image
        from std_msgs.msg import Bool

        self.rclpy = rclpy
        rclpy.init(args=[])
        self.node = rclpy.create_node("follow_demo_monitor")
        self.images = {}
        self.visible = None  # (value, receipt time)
        self.position = None  # (distance, receipt time)
        self.command = None  # ((linear, angular), receipt time)
        best_effort = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        for name, topic in (("left", args.left), ("right", args.right)):
            self.node.create_subscription(
                Image, topic, lambda _m, n=name: self.images.__setitem__(n, time.monotonic()), best_effort)
        self.node.create_subscription(
            Bool, "/owner/visible", lambda m: setattr(self, "visible", (bool(m.data), time.monotonic())), 1)
        self.node.create_subscription(
            Point, "/owner/relative_position",
            lambda m: setattr(self, "position", (math.hypot(m.x, m.y), time.monotonic())), 1)
        self.node.create_subscription(
            Twist, "/cmd_vel",
            lambda m: setattr(self, "command", ((m.linear.x, m.angular.z), time.monotonic())), 1)

    def wait(self, timeout, predicate):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            self.rclpy.spin_once(self.node, timeout_sec=0.05)
            if predicate(time.monotonic()):
                return True
        return False

    def close(self):
        self.node.destroy_node()
        if self.rclpy.ok():
            self.rclpy.shutdown()


def clearance_violation(distance, minimum):
    return distance < minimum


def run_clearance(args):
    """Exit 0 (stopped, prints the closest approach), 3 on contact risk, 4 if no robot pose arrives."""
    import signal

    import rclpy
    from gazebo_msgs.msg import ModelStates
    from rclpy.signals import SignalHandlerOptions

    from room_obstacles import OBSTACLES, obstacle_distance

    rclpy.init(args=[], signal_handler_options=SignalHandlerOptions.NO)
    node = rclpy.create_node("follow_demo_clearance")
    state = {"closest": math.inf, "seen": False, "violation": False, "stop": False}

    def write_status():
        if args.status_file:
            with open(args.status_file, "w") as handle:
                handle.write(f"closest approach {state['closest']:.2f} m from an obstacle surface "
                             f"(limit {args.minimum:g} m)")

    def on_states(message):
        if args.model not in message.name:
            return
        position = message.pose[message.name.index(args.model)].position
        distance = obstacle_distance(position.x, position.y, OBSTACLES)
        state["seen"] = True
        if distance < state["closest"]:
            state["closest"] = distance
            write_status()
        if clearance_violation(distance, args.minimum) and not state["violation"]:
            state["violation"] = True
            print(f"watch-clearance: VIOLATION - robot centre {distance:.2f} m from an obstacle surface "
                  f"(limit {args.minimum:g} m) at ({position.x:.2f}, {position.y:.2f})",
                  file=sys.stderr, flush=True)

    node.create_subscription(ModelStates, args.topic, on_states, 10)
    for number in (signal.SIGINT, signal.SIGTERM):
        signal.signal(number, lambda *_: state.update(stop=True))
    started = time.monotonic()
    status = 0
    while not state["stop"] and not state["violation"]:
        rclpy.spin_once(node, timeout_sec=0.1)
        if not state["seen"] and time.monotonic() - started > args.start_timeout:
            print(f"watch-clearance: no '{args.model}' pose on {args.topic} after {args.start_timeout:g} s",
                  file=sys.stderr, flush=True)
            status = 4
            break
    if state["violation"]:
        status = 3
    if state["seen"]:
        print(f"watch-clearance: closest approach {state['closest']:.2f} m from an obstacle surface "
              f"(limit {args.minimum:g} m)", flush=True)
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
    return status


def run(args):
    monitor = Monitor(args)
    try:
        if args.command == "wait-images":
            ok = monitor.wait(args.timeout, lambda now: set(monitor.images) == {"left", "right"})
            missing = sorted({"left", "right"} - set(monitor.images))
            detail = f"no image received from: {', '.join(missing)}"
        elif args.command == "wait-visible":
            ok = monitor.wait(args.timeout, lambda now: monitor.visible is not None
                              and monitor.visible[0] and now - monitor.visible[1] < 1.0)
            detail = f"/owner/visible never true (last: {monitor.visible})"
        elif args.command == "wait-settled":
            tracker = SettleTracker(args.low, args.high, args.hold)

            def settled(now):
                if monitor.position is None:
                    return tracker.update(now, None, 0.0)
                distance, stamp = monitor.position
                return tracker.update(now, distance, now - stamp)

            ok = monitor.wait(args.timeout, settled)
            last = f"{monitor.position[0]:.2f} m" if monitor.position else "none"
            detail = (f"owner distance did not hold within [{args.low}, {args.high}] m "
                      f"for {args.hold} s (last: {last})")
        else:  # wait-stopped
            tracker = StopTracker(args.hold)

            def stopped(now):
                visible, visible_age = (monitor.visible[0], now - monitor.visible[1]) \
                    if monitor.visible else (None, math.inf)
                command, command_age = (monitor.command[0], now - monitor.command[1]) \
                    if monitor.command else (None, math.inf)
                return tracker.update(now, visible, visible_age, command, command_age)

            ok = monitor.wait(args.timeout, stopped)
            detail = (f"robot did not stop: visible={monitor.visible}, cmd_vel={monitor.command}")
    finally:
        monitor.close()
    if ok:
        print(f"{args.command}: OK")
        return 0
    print(f"{args.command}: TIMEOUT after {args.timeout:g} s - {detail}", file=sys.stderr)
    return 1


def parse(argv):
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawTextHelpFormatter)
    parser.add_argument("--left", default="/room_cameras/left/image_raw")
    parser.add_argument("--right", default="/room_cameras/right/image_raw")
    commands = parser.add_subparsers(dest="command", required=True)
    for name, timeout in (("wait-images", 120.0), ("wait-visible", 30.0),
                          ("wait-settled", 90.0), ("wait-stopped", 10.0)):
        sub = commands.add_parser(name)
        sub.add_argument("--timeout", type=float, default=timeout)
        if name == "wait-settled":
            sub.add_argument("--low", type=float, default=SETTLED_LOW)
            sub.add_argument("--high", type=float, default=SETTLED_HIGH)
            sub.add_argument("--hold", type=float, default=3.0)
        if name == "wait-stopped":
            sub.add_argument("--hold", type=float, default=1.0)
    clearance = commands.add_parser("watch-clearance")
    clearance.add_argument("--minimum", type=float, default=0.25,
                           help="closest allowed distance from the robot centre to an obstacle surface (m)")
    clearance.add_argument("--topic", default="/gazebo/model_states")
    clearance.add_argument("--model", default="waffle_pi")
    clearance.add_argument("--start-timeout", type=float, default=30.0)
    clearance.add_argument("--status-file", default="", help="keep the closest approach in this file")
    args = parser.parse_args(argv)
    if args.command == "watch-clearance":
        if not math.isfinite(args.minimum) or args.minimum <= 0:
            parser.error("--minimum must be positive and finite")
    elif not math.isfinite(args.timeout) or args.timeout <= 0:
        parser.error("--timeout must be positive and finite")
    return args


if __name__ == "__main__":
    parsed = parse(sys.argv[1:])
    sys.exit(run_clearance(parsed) if parsed.command == "watch-clearance" else run(parsed))
