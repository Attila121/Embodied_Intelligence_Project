# Progress

Updated: 2026-11-12. Branch: `real-world-test` (from `following-human-bridge`), pushed to
`origin/real-world-test`.

## Done

### Real-world test checklist (`docs/CLASS_TEST.md`)

- Lists what the simulation does not cover and the in-class order: `run_rig.sh --check`,
  floor rectangle, cameras + chain, perception rates/accuracy, follower intent, real `/scan`
  format, avoidance dry run (`source:=rig avoid:=true`), then supervised driving. Includes a
  table of values to record.

### One-command simulation demo (`docs/DEMO.md`)

- `bash scripts/run_ros_experiment.sh follow-demo [--headless] [--no-walk] [--owner-speed M/S]`
  (`scripts/follow_demo.sh`) starts the room world and the follower (`drive:=true`,
  simulation motion), then walks the owner stand at 0.06 m/s through three waypoints
  with pauses and settle checks, removes the stand (requires visibility false and
  `/cmd_vel` exactly zero for 1 s), respawns it, and keeps running until Ctrl+C.
- Every stage waits on received messages via `scripts/demo_monitor.py`
  (images from both cameras, `/owner/visible`, distance within 0.9-1.2 m for 3 s,
  stopped). Timeouts fail loudly.
- `scripts/supervise.sh`: each child in its own process group; SIGINT -> 10 s -> SIGTERM
  -> 5 s -> SIGKILL; a child exiting, or a required process vanishing inside a
  surviving `ros2 launch` (e.g. `gzserver`), aborts the run and cleans up.
- Verified in WSL (headless, software GL): the full walk completed with every stage OK
  on two separate runs; SIGTERM and SIGINT stopped everything in 1-2 s with no leftover
  processes; SIGKILL of `gzserver`
  mid-run was detected in 2 s (exit 1, no leftovers). That kill test exposed that
  watching only the `ros2 launch` wrapper missed a dead simulator; fixed with
  `require_process`.

### Obstacles and avoidance (`docs/OBSTACLES.md`)

- `follow-demo --obstacles` adds three low obstacles (`scripts/room_obstacles.py`, one
  source of truth for the scene, the checks and the simulation) and runs the follower
  with `room_follow.launch.py avoid:=true`: follower -> `scripts/obstacle_avoider.py` ->
  guard. **The follower and guard are unchanged**; the guard stays the final gate. Default
  demo and defaults of both launch files (`obstacles:=false`, `avoid:=false`) are unchanged.
- Logic in `scripts/avoidance.py` (pure Python). Driven by the guard's rules: any turn
  component is refused with anything within 0.55 m, so it plans paths with ~0.7 m
  clearance, curves smoothly when nothing is that close, and falls back to pure rotation,
  straight driving or a slow straight "escape". It never adds motion the follower did not
  ask for, ignores the owner stand's LiDAR returns, and stops on any missing/invalid input.
- A ground-truth monitor (`demo_monitor.py watch-clearance`) aborts the demo if the robot
  comes within 0.25 m of an obstacle and reports the closest approach at the end.
- Design lessons (kept in the docs): obstacles need ~2.6 m between centres at this
  clearance; my first layout had a 0.25 m inflated gap and trapped the robot. Early
  versions that rotated in place then drove straight drifted into the inflated zone and
  looped; tracking the clear heading with a smooth curve fixed that.
- Verified: closed-loop simulation with the real guard logic (without avoidance the robot
  freezes at the first obstacle for the whole run; with it every stage settles, closest
  approach 0.64 m, also under +/-12 cm position errors); full headless Gazebo run of
  `follow-demo --obstacles` with every stage OK and a measured closest approach of 0.65 m;
  the default demo re-run afterwards and still passes; stop with SIGINT left no processes.

### Real-camera rig, local dry run (`rig/README.md`)

- Drop models into `rig/assets/` (contract in `rig/assets/README.md`; ignored by Git),
  copy `rig/rig.env.example` to `rig/rig.env`, then `bash scripts/run_rig.sh --check`
  and `bash scripts/run_rig.sh`. `drive` stays false.
- `scripts/rig_human_publisher.py` and `scripts/rig_robot_publisher.py` are rig copies of
  Haichen's detectors (originals untouched). The real cap model is required (no
  YOLO-World/CLIP fallback). Camera selection is by serial and never guesses. Failed grabs
  are skipped and always released. The robot publisher publishes only for a fresh frame with
  the robot and both stickers found, so tracking loss now stops following through the
  bridge's 0.5 s timeout. It processes every frame (original: every 4th) and logs its rate.
- `scripts/check_rig_assets.py` validates the exact resolved paths (checkpoint archives,
  calibration arrays, imports, two distinct attached serials). `run_rig.sh` exports the
  rig domain after sourcing ROS (refuses 30), and gives each publisher its own
  `CAMERA_SERIAL`/`CALIB_DIR`.
- `scripts/rig_capture.py` holds the shared acquire/device/rate helpers.

### Earlier on this branch

- `scripts/rig_geometry.py`, `scripts/rig_bridge.py`, `config/rig_bridge.yaml`, the
  `source:=markers|rig` launch option and `docs/RIG_BRIDGE.md` (planar Pose2D streams to
  the follower topics, heading and axis-scale corrections, per-stream freshness checks).

### Verification

- WSL Ubuntu-22.04 with ROS Humble: full suite 179 tests passed (69 existing + 110 new). New tests cover stale-frame/stale-pose behaviour through the
  real `acquire`, partial detections on synthetic images with a fake YOLO model, the
  asset/serial checker, the demo monitor logic, `run_rig.sh` environment contract using
  fake executables, and the supervisor with real dummy processes (escalation to SIGKILL,
  child death, mid-step abort, lost required process).
- Windows: the new tests pass or skip cleanly (no numpy/OpenCV there). Five older test
  modules fail to import without ROS/OpenCV, as before.
- Not run: any real Basler camera, Haichen's models, or the human/robot publishers
  end to end (their camera and YOLO calls are exercised only through fakes).

## Next

- Look at `follow-demo --obstacles` with the Gazebo window (only the headless mode was run; the
  window itself, its rendering speed and the obstacle colours have not been seen).

- Obtain models and calibration from Haichen, fill `rig/assets/` and `rig/rig.env`, and
  run `bash scripts/run_rig.sh --check` on the machine with the cameras.
- On the rig: measure the real publish rates (`ros2 topic hz /owner_position`,
  `/robot_position`; aim for >= 4 Hz each), verify tracking loss stops following, and
  tune inference cost (`PROCESS_EVERY_N`, `TOY_EVERY_N`, image size) if rates are low.
- Measure the common field rectangle, label A/B/C/D physical corners, and calibrate both
  publishers to the documented corner correspondence (`docs/RIG_BRIDGE.md`).
- Confirm camera serial assignments and green-to-red sticker direction; set the heading
  offset; compare relative positions at marked locations and headings; evaluate the real
  cap model's owner assignment.
- Before hardware following, validate the real robot's command interface/guard using
  `docs/ROBOT.md`.
- If exploration is needed in this WSL environment, resolve the existing `explore_lite`
  setup separately.

## Open decisions and limitations

- Obstacle avoidance is reactive (1.6 m look-ahead, no map): it can be boxed in by obstacles
  closer than ~2.6 m or U-shapes and then stops. Only 2-D LiDAR-height obstacles are seen;
  moving obstacles are treated as static. Simulation only; Waffle Pi geometry.

- Measured plane dimensions, heading offset and parallax measurements remain unknown.
  Parallax is disabled by default.
- Pose2D streams have no exposure timestamps; the bridge combines latest received samples
  without capture-time synchronization.
- Owner identity can switch between frames. Identity hysteresis and physical robot
  deployment remain separate work; the simulation LiDAR guard only checks obstacle
  clearance.
- The rig launcher is a local dry run only; it is not a procedure for driving the robot.
- Working-tree line endings: this Windows checkout writes CRLF, while `.gitattributes`
  stores LF. Shell scripts must stay LF in the working tree to run under WSL/Linux
  (`sed -i 's/\r$//' scripts/*.sh` if an editor converts them).
