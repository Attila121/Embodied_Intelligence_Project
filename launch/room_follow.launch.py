"""Follow the owner using two fixed cameras; drive:=true enables Gazebo motion."""

from pathlib import Path
import sys

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration


def start_nodes(context):
    root = Path(__file__).resolve().parents[1]
    drive = LaunchConfiguration("drive").perform(context).lower()
    if drive not in {"true", "false"}:
        raise ValueError("drive must be true or false")
    output = "/cmd_vel" if drive == "true" else "/following/safe_cmd_vel"
    return [
        ExecuteProcess(cmd=[sys.executable, str(root/"scripts/owner_follower.py")], output="screen"),
        ExecuteProcess(cmd=[sys.executable, str(root/"scripts/following_guard.py"), "--ros-args",
                            "-r", f"/following/safe_cmd_vel:={output}"], output="screen"),
        ExecuteProcess(cmd=[sys.executable, str(root/"scripts/room_owner_tracker.py"), "--ros-args",
                            "--params-file", str(root/"config/room_following.yaml")], output="screen"),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("drive", default_value="false",
                              description="Publish guarded commands to /cmd_vel for simulation"),
        OpaqueFunction(function=start_nodes),
    ])
