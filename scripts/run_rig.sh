#!/usr/bin/env bash
# LOCAL DRY RUN of the real-camera rig: human publisher + robot publisher + rig bridge,
# follower and guard.  drive stays false, so guarded commands go only to
# /following/safe_cmd_vel; nothing here commands a physical robot (see docs/ROBOT.md).
#
#   bash scripts/run_rig.sh [--check]
#
# Settings come from rig/rig.env (copy rig/rig.env.example).  --check validates the
# configuration, models, calibration and camera serials, then exits without starting anything.
# This script does NOT use run_ros_experiment.sh, whose simulation settings force domain 30.
set -eo pipefail

ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
check_only=0
case "${1:-}" in
  "") ;;
  --check) check_only=1 ;;
  -h | --help) sed -n '2,10p' "${BASH_SOURCE[0]}" | sed 's/^# \{0,1\}//'; exit 0 ;;
  *) printf 'Usage: bash scripts/run_rig.sh [--check]\n' >&2; exit 2 ;;
esac

fail() {
  printf 'error: %s\n' "$*" >&2
  exit 2
}

env_file="${RIG_ENV:-$ROOT/rig/rig.env}"
[[ -f "$env_file" ]] || fail "missing $env_file; copy rig/rig.env.example to rig/rig.env and fill it in"

source "${RIG_ROS_SETUP:-/opt/ros/humble/setup.bash}" # RIG_ROS_SETUP: non-default ROS install
# Load rig.env AFTER ROS setup so its domain wins; tolerate Windows line endings.
set -a
# shellcheck disable=SC1090
source <(tr -d '\r' <"$env_file")
set +a

domain="${RIG_ROS_DOMAIN_ID:-}"
[[ "$domain" =~ ^[0-9]+$ ]] && ((10#$domain <= 232)) || fail "RIG_ROS_DOMAIN_ID must be an integer 0-232 (got '$domain')"
domain=$((10#$domain))
((domain != 30)) || fail "ROS domain 30 is the simulation's; choose another RIG_ROS_DOMAIN_ID"
localhost="${RIG_LOCALHOST_ONLY:-1}"
[[ "$localhost" == 0 || "$localhost" == 1 ]] || fail "RIG_LOCALHOST_ONLY must be 0 or 1"

human_serial="${HUMAN_CAMERA_SERIAL:-}"
robot_serial="${ROBOT_CAMERA_SERIAL:-}"
[[ -n "$human_serial" && -n "$robot_serial" ]] || fail "set both HUMAN_CAMERA_SERIAL and ROBOT_CAMERA_SERIAL"
[[ "$human_serial" != "$robot_serial" ]] || fail "HUMAN_CAMERA_SERIAL and ROBOT_CAMERA_SERIAL must differ"

# Resolve every path exactly once; the checker and the publishers receive the same values.
assets="$ROOT/rig/assets"
human_pose="${HUMAN_POSE_MODEL:-$assets/human/pose.pt}"
human_cap="${HUMAN_CAP_MODEL:-$assets/human/cap.pt}"
human_calib="${HUMAN_CALIB_DIR:-$assets/human}"
robot_model="${ROBOT_MODEL:-$assets/robot/robot.pt}"
robot_calib="${ROBOT_CALIB_DIR:-$assets/robot}"
robot_toy="${ROBOT_TOY_MODEL:-$assets/robot/toy.pt}"
if [[ -n "${ROBOT_TOY_MODEL:-}" && ! -f "$ROBOT_TOY_MODEL" ]]; then
  fail "ROBOT_TOY_MODEL is set but is not a file: $ROBOT_TOY_MODEL"
fi
python="${RIG_PYTHON:-python3}"

printf '== Rig configuration ==\n'
printf '  ROS domain %s, localhost-only %s, python %s\n' "$domain" "$localhost" "$python"
printf '  human camera %s | pose %s | cap %s | calibration %s\n' "$human_serial" "$human_pose" "$human_cap" "$human_calib"
printf '  robot camera %s | model %s | toy %s | calibration %s\n' "$robot_serial" "$robot_model" "$robot_toy" "$robot_calib"

"$python" "$ROOT/scripts/check_rig_assets.py" \
  --human-pose "$human_pose" --human-cap "$human_cap" --human-calib "$human_calib" \
  --robot-model "$robot_model" --robot-toy "$robot_toy" --robot-calib "$robot_calib" \
  --human-serial "$human_serial" --robot-serial "$robot_serial"

if ((check_only)); then
  printf 'Checks passed (--check): nothing started.\n'
  exit 0
fi
[[ -n "${DISPLAY:-}${WAYLAND_DISPLAY:-}" ]] || fail "no DISPLAY: both publishers open ROI-click windows"

export ROS_DOMAIN_ID="$domain" ROS_LOCALHOST_ONLY="$localhost"
# shellcheck source=scripts/supervise.sh
source "$ROOT/scripts/supervise.sh"
supervise_init

printf '== LOCAL DRY RUN: guarded commands go to /following/safe_cmd_vel only. ==\n'
printf '== Click each window'"'"'s floor corners (see docs/RIG_BRIDGE.md), then check rates with:\n'
printf '   ros2 topic hz /owner_position ; ros2 topic hz /robot_position   (aim for >= 4 Hz each)\n'

start_child human_publisher env \
  CAMERA_SERIAL="$human_serial" CALIB_DIR="$human_calib" \
  POSE_MODEL="$human_pose" CAP_MODEL="$human_cap" \
  "$python" "$ROOT/scripts/rig_human_publisher.py"
start_child robot_publisher env \
  CAMERA_SERIAL="$robot_serial" CALIB_DIR="$robot_calib" \
  ROBOT_MODEL="$robot_model" TOY_MODEL="$robot_toy" \
  "$python" "$ROOT/scripts/rig_robot_publisher.py"
start_child bridge_follower ros2 launch "$ROOT/launch/room_follow.launch.py" source:=rig drive:=false

watch_children
