"""Start following and its guard; tracker:=true enables camera marker tracking."""

import sys
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration


def start_nodes(context):
    drive = LaunchConfiguration("drive").perform(context).lower()
    if drive not in {"true", "false"}:
        raise ValueError("drive must be true or false")
    tracker = LaunchConfiguration("tracker").perform(context).lower()
    if tracker not in {"true", "false"}:
        raise ValueError("tracker must be true or false")
    output = "/cmd_vel" if drive == "true" else "/following/safe_cmd_vel"
    scripts = Path(__file__).resolve().parents[1] / "scripts"
    actions = [
        ExecuteProcess(
            cmd=[sys.executable, str(scripts / "owner_follower.py")],
            output="screen"),
        ExecuteProcess(
            cmd=[sys.executable, str(scripts / "following_guard.py"),
                 "--ros-args", "-r", f"/following/safe_cmd_vel:={output}"],
            output="screen"),
    ]
    if tracker == "true":
        actions.append(ExecuteProcess(
            cmd=[sys.executable, str(scripts / "owner_tracker.py"), "--ros-args",
                 "--params-file", str(scripts.parent / "config" / "owner_tracking.yaml")],
            output="screen"))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "drive", default_value="false",
            description="Publish guarded commands to /cmd_vel for simulation"),
        DeclareLaunchArgument(
            "tracker", default_value="false",
            description="Detect the owner marker in camera images"),
        OpaqueFunction(function=start_nodes),
    ])
