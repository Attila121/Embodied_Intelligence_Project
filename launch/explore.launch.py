"""Autonomous exploration in simulation: slam_toolbox + Nav2 + explore_lite + RViz.

Start Gazebo first, then run this file through the project launcher:

    bash scripts/run_ros_experiment.sh world      # terminal A
    bash scripts/run_ros_experiment.sh explore    # terminal B

slam_toolbox builds the map, Nav2 drives the robot, and explore_lite picks the
next unknown edge (frontier) to visit. Do not run Cartographer (`slam`) or the
keyboard controller at the same time.

Optional arguments (append to the explore command):
    use_rviz:=false                        run without RViz
    explore_params:=/path/to/file.yaml     other explore_lite settings
    nav2_params:=/path/to/file.yaml        other Nav2 settings
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

PROJECT_DIR = os.path.abspath(os.path.join(os.path.dirname(os.path.realpath(__file__)), ".."))

# Seconds to wait before explore_lite starts, so slam_toolbox can publish a
# first map and Nav2 can finish starting.
EXPLORE_DELAY_S = 15.0


def generate_launch_description():
    nav2_dir = get_package_share_directory("nav2_bringup")
    slam_dir = get_package_share_directory("slam_toolbox")

    use_sim_time = LaunchConfiguration("use_sim_time")
    nav2_params = LaunchConfiguration("nav2_params")
    slam_params = LaunchConfiguration("slam_params")
    explore_params = LaunchConfiguration("explore_params")
    use_rviz = LaunchConfiguration("use_rviz")

    arguments = [
        DeclareLaunchArgument(
            "use_sim_time", default_value="true", description="Use the Gazebo clock"
        ),
        DeclareLaunchArgument(
            "nav2_params",
            default_value=os.path.join(nav2_dir, "params", "nav2_params.yaml"),
            description="Nav2 parameters (default: Nav2's TurtleBot3 settings)",
        ),
        DeclareLaunchArgument(
            "slam_params",
            default_value=os.path.join(slam_dir, "config", "mapper_params_online_async.yaml"),
            description="slam_toolbox parameters",
        ),
        DeclareLaunchArgument(
            "explore_params",
            default_value=os.path.join(PROJECT_DIR, "config", "explore_params.yaml"),
            description="explore_lite parameters",
        ),
        DeclareLaunchArgument("use_rviz", default_value="true", description="Open RViz"),
    ]

    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(slam_dir, "launch", "online_async_launch.py")),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "slam_params_file": slam_params,
        }.items(),
    )

    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(nav2_dir, "launch", "navigation_launch.py")),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "params_file": nav2_params,
            "autostart": "true",
        }.items(),
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", os.path.join(nav2_dir, "rviz", "nav2_default_view.rviz")],
        parameters=[{"use_sim_time": use_sim_time}],
        condition=IfCondition(use_rviz),
        output="log",
    )

    explore = Node(
        package="explore_lite",
        executable="explore",
        name="explore_node",
        parameters=[explore_params, {"use_sim_time": use_sim_time}],
        output="screen",
    )

    start_explore = TimerAction(
        period=EXPLORE_DELAY_S,
        actions=[LogInfo(msg="Starting explore_lite: the robot will now move by itself."), explore],
    )

    return LaunchDescription(
        arguments
        + [
            LogInfo(msg=f"explore_lite starts in {EXPLORE_DELAY_S:.0f} s. Keep the keyboard controller closed."),
            slam,
            nav2,
            rviz,
            start_explore,
        ]
    )
