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

# explore_lite version these files were written for (robo-friends/m-explore-ros2, June 2026).
# Set EXPLORE_REF=main to use the newest version instead.
EXPLORE_REPO="https://github.com/robo-friends/m-explore-ros2.git"
EXPLORE_REF="${EXPLORE_REF:-326cf8a0b487c34246bb8f3326afbcd69576dc60}"

# Print OK/MISSING for each package. Returns 1 if any package is missing.
check_pkgs() {
  local missing=0 pkg
  for pkg in "$@"; do
    if ros2 pkg prefix "$pkg" >/dev/null 2>&1; then
      printf 'OK       %s\n' "$pkg"
    else
      printf 'MISSING  %s\n' "$pkg"
      missing=1
    fi
  done
  return "$missing"
}

need_explore() {
  if ! ros2 pkg prefix explore_lite >/dev/null 2>&1; then
    printf 'explore_lite is missing. Run: bash scripts/run_ros_experiment.sh setup-explore\n' >&2
    exit 1
  fi
}

case "${1:-check}" in
  check)
    status=0
    printf 'Manual mapping:\n'
    check_pkgs turtlebot3_gazebo turtlebot3_teleop turtlebot3_cartographer nav2_map_server || status=1
    printf 'Autonomous exploration:\n'
    if ! check_pkgs nav2_bringup slam_toolbox explore_lite; then
      status=1
      printf 'Fix: sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup ros-humble-slam-toolbox\n'
      printf '     then: bash scripts/run_ros_experiment.sh setup-explore\n'
    fi
    printf 'DISPLAY=%s\nWAYLAND_DISPLAY=%s\n' "${DISPLAY:-}" "${WAYLAND_DISPLAY:-}"
    exit "$status"
    ;;
  setup-explore)
    if ! check_pkgs nav2_bringup slam_toolbox >/dev/null; then
      printf 'Install Nav2 and slam_toolbox first:\n' >&2
      printf '  sudo apt update\n  sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup ros-humble-slam-toolbox\n' >&2
      exit 1
    fi
    # explore_lite needs the newer tf2_ros headers (*.hpp) from an up-to-date Humble install.
    if [[ ! -f "$(ros2 pkg prefix tf2_ros)/include/tf2_ros/tf2_ros/buffer.hpp" ]]; then
      printf 'Your ROS 2 Humble packages are too old for explore_lite. Update them first:\n' >&2
      printf '  sudo apt update && sudo apt upgrade\n' >&2
      exit 1
    fi
    mkdir -p "$workspace/src"
    if [[ ! -d "$workspace/src/m-explore-ros2" ]]; then
      git clone "$EXPLORE_REPO" "$workspace/src/m-explore-ros2"
      git -C "$workspace/src/m-explore-ros2" checkout --quiet "$EXPLORE_REF"
    else
      printf 'Using existing %s\n' "$workspace/src/m-explore-ros2"
    fi
    cd "$workspace"
    rosdep install --from-paths src/m-explore-ros2/explore src/m-explore-ros2/explore_lite_msgs \
      --ignore-src --rosdistro humble -r -y
    export MAKEFLAGS=-j1 CMAKE_BUILD_PARALLEL_LEVEL=1
    colcon build --symlink-install --parallel-workers 1 --packages-up-to explore_lite \
      --cmake-args -DCMAKE_BUILD_TYPE=Release
    printf '\nDone. Open a NEW terminal, then run: bash scripts/run_ros_experiment.sh check\n'
    ;;
  world) exec ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py ;;
  world-empty) exec ros2 launch turtlebot3_gazebo empty_world.launch.py ;;
  world-house) exec ros2 launch turtlebot3_gazebo turtlebot3_house.launch.py ;;
  rviz) exec rviz2 --ros-args -p use_sim_time:=true ;;
  teleop) exec python3 "$PROJECT_DIR/scripts/game_teleop.py" ;;
  teleop-classic) exec ros2 run turtlebot3_teleop teleop_keyboard ;;
  slam) exec ros2 launch turtlebot3_cartographer cartographer.launch.py use_sim_time:=True ;;
  explore)
    need_explore
    exec ros2 launch "$PROJECT_DIR/launch/explore.launch.py" "${@:2}"
    ;;
  explore-stop)
    exec ros2 topic pub --once /explore/resume std_msgs/msg/Bool "{data: false}"
    ;;
  explore-resume)
    exec ros2 topic pub --once /explore/resume std_msgs/msg/Bool "{data: true}"
    ;;
  metrics)
    need_explore
    exec python3 "$PROJECT_DIR/scripts/explore_metrics.py" "${@:2}"
    ;;
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
  save-explore)
    folder="$PROJECT_DIR/maps/explore_$(date +%Y%m%d_%H%M%S)"
    mkdir -p "$folder"
    printf 'Saving to %s\n' "$folder"
    exec ros2 run nav2_map_server map_saver_cli -f "$folder/map" \
      --ros-args -p use_sim_time:=true -p save_map_timeout:=10.0
    ;;
  *)
    printf 'Usage: bash scripts/run_ros_experiment.sh {check|world|world-empty|world-house|rviz|teleop|teleop-classic|slam|topics|save}\n' >&2
    printf '       exploration: {setup-explore|explore|explore-stop|explore-resume|metrics|save-explore}\n' >&2
    exit 2
    ;;
esac
