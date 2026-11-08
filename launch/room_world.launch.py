"""Two fixed room cameras and a tagged robot; contains no following controller."""

import os
from pathlib import Path
import shutil
import sys
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            OpaqueFunction, RegisterEventHandler, SetEnvironmentVariable)
from launch.conditions import IfCondition
from launch.event_handlers import OnShutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/"scripts"))
from room_scene import build_room


def world_actions(context):
    gazebo = Path(get_package_share_directory("gazebo_ros"))
    tb3 = Path(get_package_share_directory("turtlebot3_gazebo"))
    directory = Path(tempfile.mkdtemp(prefix="two-camera-room-"))
    world = directory/"room.world"
    world.write_text(build_room(tb3/"models/turtlebot3_waffle_pi/model.sdf"))

    def cleanup(_context):
        shutil.rmtree(directory, ignore_errors=True)
        return []

    return [
        SetEnvironmentVariable("TURTLEBOT3_MODEL", "waffle_pi"),
        SetEnvironmentVariable("GAZEBO_MODEL_PATH", str(tb3/"models")+os.pathsep+
                               os.environ.get("GAZEBO_MODEL_PATH", "")),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(
            str(gazebo/"launch/gzserver.launch.py")), launch_arguments={"world": str(world)}.items()),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(
            str(gazebo/"launch/gzclient.launch.py")), condition=IfCondition(LaunchConfiguration("gui"))),
        Node(package="robot_state_publisher", executable="robot_state_publisher",
             name="robot_state_publisher", output="screen",
             parameters=[{"use_sim_time": True,
                          "robot_description": (tb3/"urdf/turtlebot3_waffle_pi.urdf").read_text()}]),
        RegisterEventHandler(OnShutdown(on_shutdown=[OpaqueFunction(function=cleanup)])),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("gui", default_value="true"),
        OpaqueFunction(function=world_actions),
    ])
