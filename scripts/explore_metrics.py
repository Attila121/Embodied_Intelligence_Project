#!/usr/bin/env python3
"""Record simple metrics for one autonomous exploration run (simulation only).

Start this BEFORE exploration starts, in its own terminal:

    bash scripts/run_ros_experiment.sh metrics --world turtlebot3_world

It waits until explore_lite reports that exploration is complete, then adds one
row to maps/exploration_runs.csv and exits. Press Ctrl+C to stop early; the row
is then saved with result "stopped_by_user".

Measured values:
  duration_s     simulation time from "exploration started" to "complete"
  distance_m     path length from /odom while exploring
  known_area_m2  known (free or occupied) area in the latest /map
  coverage_pct   known_area_m2 / known area of a reference map (--reference)
  pauses         how many times exploration was paused (explore-stop)
"""

import argparse
import csv
import datetime
import math
import os
import time
from pathlib import Path

import rclpy
from explore_lite_msgs.msg import ExploreStatus
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data

PROJECT_DIR = Path(os.environ.get("PROJECT_DIR", Path(__file__).resolve().parents[1]))
DEFAULT_OUT = PROJECT_DIR / "maps" / "exploration_runs.csv"
STEP_NOISE_M = 0.001  # ignore odometry steps smaller than this
OLD_STATUS_WINDOW_S = 1.0  # status messages arriving this soon are from an earlier run
MIN_RUN_S = 5.0  # a "complete" sooner than this means exploration could not begin
UNKNOWN_PIXEL = 205  # value map_saver uses for unknown cells in trinary maps
FIELDS = [
    "date_time",
    "world",
    "result",
    "duration_s",
    "distance_m",
    "known_area_m2",
    "reference_area_m2",
    "coverage_pct",
    "pauses",
]


def read_pgm(path):
    """Return (width, height, pixels 0-255) for a P5 or P2 PGM image."""
    data = Path(path).read_bytes()
    tokens, i, n = [], 0, len(data)
    while len(tokens) < 4:
        while i < n and data[i:i + 1].isspace():
            i += 1
        if i >= n:
            raise ValueError(f"{path}: incomplete PGM header")
        if data[i:i + 1] == b"#":
            while i < n and data[i:i + 1] not in (b"\n", b"\r"):
                i += 1
            continue
        start = i
        while i < n and not data[i:i + 1].isspace():
            i += 1
        tokens.append(data[start:i])
    magic, width, height, maxval = tokens[0], int(tokens[1]), int(tokens[2]), int(tokens[3])
    count = width * height
    if magic == b"P5":
        if maxval > 255:
            raise ValueError(f"{path}: 16-bit PGM is not supported")
        raw = data[i + 1:i + 1 + count]
        values = list(raw)
    elif magic == b"P2":
        values = [int(v) for v in data[i:].split()[:count]]
    else:
        raise ValueError(f"{path}: not a PGM image (save maps with map_saver_cli)")
    if len(values) != count:
        raise ValueError(f"{path}: image data is shorter than {width}x{height}")
    if maxval != 255:
        values = [v * 255 // maxval for v in values]
    return width, height, values


def reference_known_area(yaml_path):
    """Known area (m^2) of a saved map, using map_server's thresholds."""
    import yaml

    yaml_path = Path(yaml_path).expanduser().resolve()
    info = yaml.safe_load(yaml_path.read_text())
    image = Path(info["image"])
    if not image.is_absolute():
        image = yaml_path.parent / image
    resolution = float(info["resolution"])
    _, _, pixels = read_pgm(image)
    if str(info.get("mode", "trinary")) == "trinary" and not int(info.get("negate", 0)):
        # map_saver writes unknown cells as exactly 205 (free 254, occupied 0).
        known = sum(1 for v in pixels if v != UNKNOWN_PIXEL)
    else:
        negate = int(info.get("negate", 0))
        occupied = float(info.get("occupied_thresh", 0.65))
        free = float(info.get("free_thresh", 0.25))
        known = 0
        for v in pixels:
            p = v / 255.0 if negate else (255 - v) / 255.0
            if p > occupied or p < free:
                known += 1
    return known * resolution * resolution


class ExploreMetrics(Node):
    def __init__(self, args, reference_area):
        super().__init__(
            "explore_metrics",
            parameter_overrides=[Parameter("use_sim_time", Parameter.Type.BOOL, True)],
        )
        self.args = args
        self.reference_area = reference_area
        self.created_wall = time.monotonic()
        self.date_time = datetime.datetime.now().isoformat(timespec="seconds")
        self.start = None
        self.latest = None
        self.distance = 0.0
        self.last_xy = None
        self.known_area = 0.0
        self.pauses = 0
        self.paused = False
        self.done = False

        status_qos = QoSProfile(
            depth=10,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        map_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.create_subscription(ExploreStatus, "/explore/status", self.on_status, status_qos)
        self.create_subscription(Odometry, "/odom", self.on_odom, qos_profile_sensor_data)
        self.create_subscription(OccupancyGrid, "/map", self.on_map, map_qos)
        if args.timeout_s > 0:
            self.create_timer(1.0, self.check_timeout)
        self.get_logger().info("Waiting for explore_lite to start (start exploration now).")

    def now(self):
        self.latest = self.get_clock().now()
        return self.latest

    def elapsed_s(self):
        if self.start is None:
            return 0.0
        end = self.latest or self.start
        try:
            end = self.now()
        except Exception:  # clock unavailable after Ctrl+C: use the last known time
            pass
        return max(0.0, (end - self.start).nanoseconds / 1e9)

    def on_status(self, msg):
        if time.monotonic() - self.created_wall < OLD_STATUS_WINDOW_S:
            self.get_logger().warn(
                f"Ignoring old status '{msg.status}'. Start this script BEFORE exploration."
            )
            return
        status = msg.status
        if status == ExploreStatus.EXPLORATION_STARTED and self.start is None:
            self.start = self.now()
            self.get_logger().info("Exploration started. Recording...")
        elif status == ExploreStatus.EXPLORATION_PAUSED and not self.paused:
            self.paused = True
            self.pauses += 1
            self.get_logger().info("Exploration paused.")
        elif status == ExploreStatus.EXPLORATION_IN_PROGRESS:
            self.paused = False
        elif status == ExploreStatus.EXPLORATION_COMPLETE and self.start is not None:
            if self.elapsed_s() < MIN_RUN_S:
                self.get_logger().warn(
                    "Exploration stopped right after starting. Turn the robot in place with "
                    "teleop, close teleop, then run explore-resume. Still recording."
                )
                return
            self.finish("complete")

    def on_odom(self, msg):
        if self.start is None or self.done:
            return
        self.now()
        p = msg.pose.pose.position
        if self.last_xy is not None:
            step = math.hypot(p.x - self.last_xy[0], p.y - self.last_xy[1])
            if step >= STEP_NOISE_M:
                self.distance += step
                self.last_xy = (p.x, p.y)
        else:
            self.last_xy = (p.x, p.y)

    def on_map(self, msg):
        res = msg.info.resolution
        known = len(msg.data) - msg.data.count(-1)
        self.known_area = known * res * res

    def check_timeout(self):
        if self.start is not None and not self.done and self.elapsed_s() >= self.args.timeout_s:
            self.finish("timeout")

    def finish(self, result):
        if self.done:
            return
        self.done = True
        if self.start is None:
            print("Exploration never started; nothing was saved.")
            return
        coverage = ""
        if self.reference_area:
            coverage = f"{100.0 * self.known_area / self.reference_area:.1f}"
        row = {
            "date_time": self.date_time,
            "world": self.args.world,
            "result": result,
            "duration_s": f"{self.elapsed_s():.1f}",
            "distance_m": f"{self.distance:.2f}",
            "known_area_m2": f"{self.known_area:.2f}",
            "reference_area_m2": f"{self.reference_area:.2f}" if self.reference_area else "",
            "coverage_pct": coverage,
            "pauses": str(self.pauses),
        }
        out = Path(self.args.out).expanduser()
        out.parent.mkdir(parents=True, exist_ok=True)
        new_file = not out.exists() or out.stat().st_size == 0
        with out.open("a", newline="") as f:
            writer = csv.DictWriter(f, fieldnames=FIELDS)
            if new_file:
                writer.writeheader()
            writer.writerow(row)
        print("\nRun saved to", out)
        for key in FIELDS:
            print(f"  {key:18} {row[key]}")


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--world", default="turtlebot3_world", help="name written in the CSV")
    parser.add_argument("--out", default=str(DEFAULT_OUT), help="CSV file to append to")
    parser.add_argument(
        "--reference",
        help="map.yaml of a complete map of the same world (for coverage_pct), "
        "for example maps/first_room/map.yaml",
    )
    parser.add_argument(
        "--timeout-s", type=float, default=0.0, help="stop after this many simulation seconds (0 = no limit)"
    )
    return parser.parse_args()


def main():
    args = parse_args()
    reference_area = None
    if args.reference:
        try:
            reference_area = reference_known_area(args.reference)
        except Exception as error:  # show a short message instead of a traceback
            raise SystemExit(f"Could not read reference map: {error}")
        print(f"Reference map known area: {reference_area:.2f} m^2")

    rclpy.init()
    node = ExploreMetrics(args, reference_area)
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.5)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if not node.done:
            node.finish("stopped_by_user")
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
