#!/usr/bin/env bash
# Simulation only: do not use these settings to control a physical robot.
set -eo pipefail
source /opt/ros/humble/setup.bash
workspace="${TB3_WORKSPACE:-$HOME/turtlebot3_ws}"
if [[ -f "$workspace/install/setup.bash" ]]; then
  source "$workspace/install/setup.bash"
fi
export TURTLEBOT3_MODEL=waffle_pi
export ROS_DOMAIN_ID=30
export ROS_LOCALHOST_ONLY=1
PROJECT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
export PROJECT_DIR

case "${1:-check}" in
  check)
    for pkg in turtlebot3_gazebo turtlebot3_teleop turtlebot3_cartographer nav2_map_server; do
      ros2 pkg prefix "$pkg"
    done
    printf 'DISPLAY=%s\nWAYLAND_DISPLAY=%s\n' "${DISPLAY:-}" "${WAYLAND_DISPLAY:-}"
    ;;
  world) exec ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py ;;
  rviz) exec rviz2 --ros-args -p use_sim_time:=true ;;
  teleop) exec python3 "$PROJECT_DIR/scripts/game_teleop.py" ;;
  teleop-classic) exec ros2 run turtlebot3_teleop teleop_keyboard ;;
  slam) exec ros2 launch turtlebot3_cartographer cartographer.launch.py use_sim_time:=True ;;
  topics) exec ros2 topic list -t ;;
  save)
    folder="$PROJECT_DIR/maps/first_room"
    if [[ -e "$folder/map.yaml" || -e "$folder/map.pgm" || -e "$folder/map.png" ]]; then
      printf 'A saved map already exists in %s; preserve or rename it before saving again.\n' "$folder" >&2
      exit 1
    fi
    mkdir -p "$folder"
    exec ros2 run nav2_map_server map_saver_cli -f "$folder/map" \
      --ros-args -p use_sim_time:=true -p save_map_timeout:=10.0
    ;;
  *) printf 'Usage: bash scripts/run_ros_experiment.sh {check|world|rviz|teleop|teleop-classic|slam|topics|save}\n' >&2; exit 2 ;;
esac
