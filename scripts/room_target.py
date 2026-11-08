#!/usr/bin/env python3
"""Move the room's visible marker stand; tracking still uses camera images only."""

import argparse
import math
import time

from room_scene import owner_sdf


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    actions = parser.add_subparsers(dest="action", required=True)
    move = actions.add_parser("move")
    move.add_argument("--x", type=float, required=True)
    move.add_argument("--y", type=float, required=True)
    move.add_argument("--duration", type=float, default=5.)
    spawn = actions.add_parser("spawn")
    spawn.add_argument("--x", type=float, default=1.)
    spawn.add_argument("--y", type=float, default=0.)
    actions.add_parser("remove")
    args = parser.parse_args()
    if args.action != "remove":
        if not all(math.isfinite(v) and abs(v) <= 2.4 for v in (args.x, args.y)):
            parser.error("x and y must be finite and within [-2.4, 2.4] metres")
    if args.action == "move" and (not math.isfinite(args.duration) or args.duration <= 0):
        parser.error("duration must be positive and finite")

    import rclpy
    from rclpy.node import Node
    from gazebo_msgs.srv import DeleteEntity, GetEntityState, SetEntityState, SpawnEntity

    rclpy.init(args=[])
    node = Node("room_target_control")

    def request(kind, topic, message):
        client = node.create_client(kind, topic)
        try:
            if not client.wait_for_service(timeout_sec=5):
                raise RuntimeError(f"{topic} missing; start world-room in ROS domain 30")
            future = client.call_async(message)
            rclpy.spin_until_future_complete(node, future, timeout_sec=5)
            response = future.result() if future.done() else None
            if response is None or not response.success:
                raise RuntimeError(f"Gazebo request failed: {response}")
            return response
        finally:
            node.destroy_client(client)

    try:
        if args.action == "remove":
            request(DeleteEntity, "/delete_entity", DeleteEntity.Request(name="room_owner"))
            print("Removed owner marker stand; the robot should stop.")
        elif args.action == "spawn":
            message = SpawnEntity.Request(name="room_owner", xml=owner_sdf(), reference_frame="world")
            message.initial_pose.position.x, message.initial_pose.position.y = args.x, args.y
            message.initial_pose.orientation.w = 1.
            request(SpawnEntity, "/spawn_entity", message)
            print("Created owner marker stand.")
        else:
            state = request(GetEntityState, "/gazebo/get_entity_state",
                            GetEntityState.Request(name="room_owner", reference_frame="world")).state
            state.name = "room_owner"
            start_x, start_y = state.pose.position.x, state.pose.position.y
            distance = math.hypot(args.x-start_x, args.y-start_y)
            duration = max(args.duration, distance/0.25)
            state.reference_frame = "world"
            start = time.monotonic()
            while True:
                fraction = min(1., (time.monotonic()-start)/duration)
                state.pose.position.x = start_x+(args.x-start_x)*fraction
                state.pose.position.y = start_y+(args.y-start_y)*fraction
                request(SetEntityState, "/gazebo/set_entity_state", SetEntityState.Request(state=state))
                if fraction >= 1:
                    break
                time.sleep(0.1)
            print(f"Moved the visible owner stand to ({args.x:g}, {args.y:g}) m.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
