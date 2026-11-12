#!/usr/bin/env bash
# Simulation only: one supervised command for the two-camera owner-following demo.
#
#   bash scripts/run_ros_experiment.sh follow-demo [--headless] [--no-walk] [--obstacles]
#                                                  [--owner-speed M/S]
#
# Starts the room world, the follower (drive:=true, simulation motion) and a scripted
# owner walk, checking each stage from RECEIVED messages.  Ctrl+C (or SIGTERM) stops
# everything.  After the walk the world and follower keep running until you stop them.
# With --obstacles the room gets floor obstacles and the follower runs with obstacle
# avoidance (follower -> obstacle_avoider -> guard); a ground-truth monitor aborts the
# demo if the robot ever gets within 0.25 m of an obstacle surface.
set -eo pipefail

PROJECT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
export PROJECT_DIR

usage() {
  cat <<'EOF'
Usage: bash scripts/run_ros_experiment.sh follow-demo [options]
  --headless          no Gazebo window (gui:=false)
  --no-walk           start the world and follower only; move the stand yourself with
                      scripts/room_target.py
  --obstacles         add floor obstacles and obstacle avoidance; uses a longer walk that
                      passes each obstacle (about 5 minutes)
  --owner-speed M/S   owner stand speed for the scripted walk (default 0.06, max 0.10;
                      the follower and guard cap the robot at 0.12 m/s)
EOF
}

headless=0 walk=1 obstacles=0 speed=0.06
while (($#)); do
  case "$1" in
    --headless) headless=1 ;;
    --no-walk) walk=0 ;;
    --obstacles) obstacles=1 ;;
    --owner-speed)
      speed="${2:-}"
      shift
      ;;
    -h | --help) usage; exit 0 ;;
    *) usage >&2; exit 2 ;;
  esac
  shift
done
if ! [[ "$speed" =~ ^[0-9]*\.?[0-9]+$ ]] || ! awk -v s="$speed" 'BEGIN { exit !(s > 0 && s <= 0.10) }'; then
  printf 'error: --owner-speed must be a number in (0, 0.10] m/s, got %q\n' "$speed" >&2
  exit 2
fi

if [[ -z "${ROS_DISTRO:-}" ]]; then # started directly rather than through run_ros_experiment.sh
  source /opt/ros/humble/setup.bash
  workspace="${TB3_WORKSPACE:-$HOME/turtlebot3_ws}"
  if [[ -f "$workspace/install/setup.bash" ]]; then
    source "$workspace/install/setup.bash"
  fi
fi
export TURTLEBOT3_MODEL=waffle_pi ROS_DOMAIN_ID=30 ROS_LOCALHOST_ONLY=1
export LIBGL_ALWAYS_SOFTWARE="${LIBGL_ALWAYS_SOFTWARE:-true}" GALLIUM_DRIVER="${GALLIUM_DRIVER:-llvmpipe}"

# shellcheck source=scripts/supervise.sh
source "$PROJECT_DIR/scripts/supervise.sh"
supervise_init

monitor=(python3 "$PROJECT_DIR/scripts/demo_monitor.py")
target=(python3 "$PROJECT_DIR/scripts/room_target.py")

world_args=() follower_args=(drive:=true)
if ((headless)); then world_args+=(gui:=false); fi
if ((obstacles)); then
  world_args+=(obstacles:=true)
  follower_args+=(avoid:=true)
  clearance_file="$(mktemp)"
  register_temp_file "$clearance_file"
fi

owner_move_seconds() { # distance from (ax, ay) to (bx, by) at the chosen speed, plus a margin
  awk -v ax="$1" -v ay="$2" -v bx="$3" -v by="$4" -v v="$speed" \
    'BEGIN { printf "%.0f", sqrt((bx - ax) ^ 2 + (by - ay) ^ 2) / v + 2 }'
}

printf '== Following demo (simulation, ROS domain 30%s) ==\n' "$( ((obstacles)) && printf ', with obstacles')"
start_child world bash "$PROJECT_DIR/scripts/run_ros_experiment.sh" world-room "${world_args[@]}"
run_guarded "both room cameras publishing images" "${monitor[@]}" wait-images --timeout 240
require_process world '^gzserver ' # ros2 launch keeps running if the simulator itself dies

if ((obstacles)); then
  # Ground truth, for this check only: abort if the robot ever nears an obstacle surface.
  start_child clearance "${monitor[@]}" watch-clearance --status-file "$clearance_file"
  # Move the stand to the far side first so the robot's first approach must go around an obstacle.
  read -r start_x start_y < <(python3 "$PROJECT_DIR/scripts/room_obstacles.py" start)
  duration="$(owner_move_seconds 1.0 0.0 "$start_x" "$start_y")"
  printf '== Moving the stand to (%s, %s) so the first approach has to go around an obstacle ==\n' "$start_x" "$start_y"
  run_guarded "move owner stand to the obstacle-course start" "${target[@]}" move --x "$start_x" --y "$start_y" --duration "$duration"
fi

start_child follower ros2 launch "$PROJECT_DIR/launch/room_follow.launch.py" "${follower_args[@]}"
run_guarded "owner marker visible to the cameras" "${monitor[@]}" wait-visible --timeout 60
for node in room_owner_tracker owner_follower following_guard; do
  require_process follower "scripts/$node\\.py"
done
if ((obstacles)); then require_process follower 'scripts/obstacle_avoider\.py'; fi
run_guarded "robot closes to the following distance" "${monitor[@]}" wait-settled --timeout 240
printf '== Robot is following the owner stand ==\n'

if ((walk)); then
  if ((obstacles)); then
    waypoints=()
    while read -r line; do waypoints+=("$line"); done < <(python3 "$PROJECT_DIR/scripts/room_obstacles.py" walk)
    prev_x=$start_x prev_y=$start_y
  else
    # The room spawns the owner stand at (1, 0).  Legs stay near the walls' inner area and
    # curve around the robot, which trails the stand by about 1 m.
    waypoints=("2.0 0.8" "2.0 -1.0" "0.9 -1.9")
    prev_x=1.0 prev_y=0.0
  fi
  for waypoint in "${waypoints[@]}"; do
    read -r x y <<<"$waypoint"
    duration="$(owner_move_seconds "$prev_x" "$prev_y" "$x" "$y")"
    printf '== Moving the stand to (%s, %s) over %s s ==\n' "$x" "$y" "$duration"
    run_guarded "move owner stand to ($x, $y)" "${target[@]}" move --x "$x" --y "$y" --duration "$duration"
    run_guarded "pause at waypoint" sleep 8
    run_guarded "robot settles behind the stand" "${monitor[@]}" wait-settled --timeout 240
    prev_x=$x prev_y=$y
  done

  printf '== Removing the stand: the robot must stop ==\n'
  run_guarded "remove owner stand" "${target[@]}" remove
  run_guarded "robot stops (owner invisible, cmd_vel zero)" "${monitor[@]}" wait-stopped --timeout 15
  printf '== Respawning the stand ==\n'
  run_guarded "respawn owner stand" "${target[@]}" spawn --x 1.5 --y 0.5
  run_guarded "owner visible again" "${monitor[@]}" wait-visible --timeout 60
  if ((obstacles)) && [[ -s "$clearance_file" ]]; then
    printf '== Obstacle check: %s ==\n' "$(cat "$clearance_file")"
  fi
  printf '== Demo walk complete. World and follower are still running; Ctrl+C to stop. ==\n'
else
  printf '== Ready. Move the stand with scripts/room_target.py (domain 30); Ctrl+C to stop. ==\n'
fi

watch_children
