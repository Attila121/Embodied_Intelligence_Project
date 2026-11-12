"""Follow marker or rig observations; drive:=true enables Gazebo motion."""

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
    source = LaunchConfiguration("source").perform(context).lower()
    if source not in {"markers", "rig"}:
        raise ValueError("source must be markers or rig")
    tracker, config = (("room_owner_tracker.py", "room_following.yaml")
                       if source == "markers" else ("rig_bridge.py", "rig_bridge.yaml"))
    avoid = LaunchConfiguration("avoid").perform(context).lower()
    if avoid not in {"true", "false"}:
        raise ValueError("avoid must be true or false")
    output = "/cmd_vel" if drive == "true" else "/following/safe_cmd_vel"
    # With avoidance the chain is follower -> avoider -> guard; the guard still gates everything.
    follower = [sys.executable, str(root/"scripts/owner_follower.py")]
    if avoid == "true":
        follower += ["--ros-args", "-r", "/following/cmd_vel:=/following/desired_cmd_vel"]
    nodes = [
        ExecuteProcess(cmd=follower, output="screen"),
        ExecuteProcess(cmd=[sys.executable, str(root/"scripts/following_guard.py"), "--ros-args",
                            "-r", f"/following/safe_cmd_vel:={output}"], output="screen"),
        ExecuteProcess(cmd=[sys.executable, str(root/"scripts"/tracker), "--ros-args",
                            "--params-file", str(root/"config"/config)], output="screen"),
    ]
    if avoid == "true":
        nodes.append(ExecuteProcess(
            cmd=[sys.executable, str(root/"scripts/obstacle_avoider.py"), "--ros-args",
                 "--params-file", str(root/"config/obstacle_avoidance.yaml")], output="screen"))
    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("source", default_value="markers",
                              description="Owner position source: markers or rig Pose2D bridge"),
        DeclareLaunchArgument("drive", default_value="false",
                              description="Publish guarded commands to /cmd_vel for simulation"),
        DeclareLaunchArgument("avoid", default_value="false",
                              description="Insert the obstacle-avoidance node between follower and guard"),
        OpaqueFunction(function=start_nodes),
    ])
