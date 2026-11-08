#!/usr/bin/env python3
"""Create a printable owner marker or spawn/remove its Gazebo demonstration board."""

import argparse
import math
from pathlib import Path

from owner_marker import DEFAULT_ID, DEFAULT_SIZE, marker_sdf, marker_svg


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    actions = parser.add_subparsers(dest="action", required=True)
    for action in ("spawn", "remove", "print"):
        command = actions.add_parser(action)
        if action in ("spawn", "print"):
            command.add_argument("--marker-id", type=int, default=DEFAULT_ID)
            command.add_argument("--marker-size", type=float, default=DEFAULT_SIZE)
        if action in ("spawn", "remove"):
            command.add_argument("--name", default="owner_marker")
        if action == "spawn":
            command.add_argument("--distance", type=float, default=1.5)
            command.add_argument("--lateral", type=float, default=0.0)
            command.add_argument("--height", type=float, default=0.20)
            command.add_argument("--robot-entity", default="waffle_pi")
        if action == "print":
            command.add_argument("--output", type=Path, default=Path("/tmp/owner_marker_23.svg"))
    args = parser.parse_args()
    if args.action == "print":
        args.output.write_text(marker_svg(args.marker_id, args.marker_size))
        print(f"Wrote {args.output.resolve()}. Print at 100% scale; the outer black "
              f"square must measure {args.marker_size * 100:g} cm per side.")
        return
    if args.action == "spawn":
        if not all(math.isfinite(v) for v in (args.distance, args.lateral, args.height)):
            parser.error("target position must be finite")
        if args.distance < 0.8 or args.height < 0.1:
            parser.error("use distance >= 0.8 m and height >= 0.1 m")
        xml = marker_sdf(args.marker_id, args.marker_size, args.name)

    import rclpy
    from gazebo_msgs.srv import DeleteEntity, SpawnEntity
    from rclpy.node import Node

    rclpy.init(args=[])
    node = Node("owner_marker_target")
    try:
        if args.action == "spawn":
            client = node.create_client(SpawnEntity, "/spawn_entity")
            request = SpawnEntity.Request()
            request.name, request.xml = args.name, xml
            request.reference_frame = args.robot_entity
            request.initial_pose.position.x = args.distance
            request.initial_pose.position.y = args.lateral
            request.initial_pose.position.z = args.height
            request.initial_pose.orientation.w = 1.0
        else:
            client = node.create_client(DeleteEntity, "/delete_entity")
            request = DeleteEntity.Request()
            request.name = args.name
        if not client.wait_for_service(timeout_sec=10):
            raise RuntimeError("Gazebo service unavailable. Check world and ROS_DOMAIN_ID=30.")
        future = client.call_async(request)
        rclpy.spin_until_future_complete(node, future, timeout_sec=15)
        if not future.done() or future.result() is None:
            raise RuntimeError("Gazebo marker request timed out")
        response = future.result()
        if not response.success:
            raise RuntimeError(response.status_message)
        print(response.status_message)
        if args.action == "spawn":
            print(f"Owner marker {args.marker_id} is {args.distance:g} m ahead of "
                  f"{args.robot_entity}. Move the board in Gazebo to change the "
                  "camera-measured target; no synthetic position is published.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
