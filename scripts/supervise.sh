# shellcheck shell=bash
# Process supervision for the demo/rig launchers.  Source it; do not execute it.
#
#   supervise_init                 enable job control and install EXIT/INT/TERM/HUP traps
#   start_child NAME CMD...        run CMD in its own process group (prefix with `env VAR=x` to set env)
#   register_temp_file PATH       delete PATH when the supervisor exits
#   require_process CHILD REGEX    a process matching REGEX must stay alive in CHILD's group
#   run_guarded DESC CMD...        run a foreground step; abort everything if a child dies meanwhile
#   watch_children                 block until a child exits (then shut everything down, exit 1)
#   shutdown_all                   SIGINT -> (10 s) SIGTERM -> (5 s) SIGKILL, per process group
#
# With job control (set -m) every background job gets its own process group (pgid == pid), so
# Ctrl+C only reaches this script, which then stops the groups gracefully, newest first.

SUP_NAMES=()
SUP_PIDS=()
SUP_REQ_INDEX=()
SUP_REQ_PATTERN=()
SUP_TEMP_FILES=()
SUP_DONE=0
SUP_INT_WAIT="${SUP_INT_WAIT:-10}"
SUP_TERM_WAIT="${SUP_TERM_WAIT:-5}"

supervise_init() {
  set -m
  trap 'supervise_on_exit' EXIT
  trap 'supervise_on_signal 130' INT
  trap 'supervise_on_signal 143' TERM
  trap 'supervise_on_signal 129' HUP
}

supervise_on_exit() {
  local status=$?
  shutdown_all
  if ((${#SUP_TEMP_FILES[@]})); then rm -f -- "${SUP_TEMP_FILES[@]}"; fi
  return "$status"
}

register_temp_file() {
  SUP_TEMP_FILES+=("$1")
}

supervise_on_signal() {
  printf '\n[supervise] signal received\n' >&2
  shutdown_all
  exit "$1"
}

_pid_alive() {
  local state
  state="$(ps -o stat= -p "$1" 2>/dev/null)" || return 1
  [[ -n "$state" && "$state" != Z* ]]
}

_group_alive() {
  kill -0 -- "-$1" 2>/dev/null
}

start_child() {
  local name="$1"
  shift
  "$@" </dev/null &
  local pid=$!
  SUP_NAMES+=("$name")
  SUP_PIDS+=("$pid")
  printf '[supervise] started %s (pid %s)\n' "$name" "$pid" >&2
}

# require_process CHILD PATTERN: from now on a process whose full command line matches the
# regex PATTERN must exist in CHILD's process group, otherwise CHILD counts as dead.  This
# catches launchers (ros2 launch) that keep running after one of their processes died.
# Call it only once the process is known to be up.
require_process() {
  local name="$1" pattern="$2" i
  for i in "${!SUP_NAMES[@]}"; do
    if [[ "${SUP_NAMES[$i]}" == "$name" ]]; then
      SUP_REQ_INDEX+=("$i")
      SUP_REQ_PATTERN+=("$pattern")
      return 0
    fi
  done
  printf "[supervise] require_process: unknown child '%s'\n" "$name" >&2
  return 1
}

# Print the first dead child's name/status; return 0 if one died, 1 if all are alive.
_dead_child() {
  local i r status
  for i in "${!SUP_PIDS[@]}"; do
    if ! _pid_alive "${SUP_PIDS[$i]}"; then
      wait "${SUP_PIDS[$i]}" 2>/dev/null && status=0 || status=$?
      printf "[supervise] child '%s' exited unexpectedly (status %s)\n" "${SUP_NAMES[$i]}" "$status" >&2
      return 0
    fi
  done
  for r in "${!SUP_REQ_INDEX[@]}"; do
    i="${SUP_REQ_INDEX[$r]}"
    if ! pgrep -g "${SUP_PIDS[$i]}" -f -- "${SUP_REQ_PATTERN[$r]}" >/dev/null 2>&1; then
      printf "[supervise] child '%s' lost required process matching '%s'\n" \
        "${SUP_NAMES[$i]}" "${SUP_REQ_PATTERN[$r]}" >&2
      return 0
    fi
  done
  return 1
}

_wait_groups() {
  local limit="$1" waited=0 i alive
  while ((waited < limit * 10)); do
    alive=0
    for i in "${!SUP_PIDS[@]}"; do
      if _group_alive "${SUP_PIDS[$i]}"; then alive=1; fi
    done
    ((alive)) || return 0
    sleep 0.1
    waited=$((waited + 1))
  done
  return 1
}

_signal_groups() {
  local sig="$1" i
  for ((i = ${#SUP_PIDS[@]} - 1; i >= 0; i--)); do
    kill "-$sig" -- "-${SUP_PIDS[$i]}" 2>/dev/null || true
  done
}

shutdown_all() {
  ((SUP_DONE)) && return 0
  SUP_DONE=1
  ((${#SUP_PIDS[@]})) || return 0
  printf '[supervise] stopping %s process group(s)\n' "${#SUP_PIDS[@]}" >&2
  _signal_groups INT
  if ! _wait_groups "$SUP_INT_WAIT"; then
    printf '[supervise] still running after SIGINT; sending SIGTERM\n' >&2
    _signal_groups TERM
    if ! _wait_groups "$SUP_TERM_WAIT"; then
      printf '[supervise] still running after SIGTERM; sending SIGKILL\n' >&2
      _signal_groups KILL
      _wait_groups 2 || true
    fi
  fi
  printf '[supervise] stopped\n' >&2
}

# Run a foreground step as its own job; fail fast if a supervised child dies meanwhile.
run_guarded() {
  local description="$1"
  shift
  "$@" </dev/null &
  local pid=$! status
  while _pid_alive "$pid"; do
    if _dead_child; then
      kill -TERM -- "-$pid" 2>/dev/null || true
      printf '[supervise] aborting during step: %s\n' "$description" >&2
      exit 1
    fi
    sleep 0.5
  done
  wait "$pid" && status=0 || status=$?
  if ((status != 0)); then
    printf '[supervise] step failed (%s): %s\n' "$status" "$description" >&2
  fi
  return "$status"
}

watch_children() {
  while true; do
    if _dead_child; then
      exit 1
    fi
    sleep 1
  done
}
