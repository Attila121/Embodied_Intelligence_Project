# Real-world test session (class checklist)

`follow-demo --obstacles` proves the following and avoidance **logic** in Gazebo, with ideal
ArUco tracking, a simulated Waffle Pi and a perfect LiDAR. It does not prove the real cameras,
models, LiDAR or robot base. This checklist is the order for testing those in class. Steps 1-7
never command the robot: guarded output goes only to `/following/safe_cmd_vel`.

## What the simulation does not cover

- **Perception:** Basler cameras, Haichen's YOLO models and the rig bridge replace the markers.
  Field size, heading offset and publish rates are unmeasured; owner identity can switch
  between frames. Avoidance was only tested with +/-12 cm position error.
- **Owner vs. obstacle:** the avoider ignores LiDAR returns within 0.35 m of the reported owner
  position. A real person is wider than the sim stand, so with position error their legs may be
  treated as an obstacle.
- **LiDAR format:** the guard requires frame `base_scan`, a full 360 deg scan, and `+inf` for
  "no return". Many real LiDARs report `0.0` or `NaN`, which the guard rejects
  ("invalid scan reading"): safe, but the robot never moves.
- **Robot geometry:** LiDAR offset (-6.4 cm), body clearance (0.30 m) and corridor width are
  Waffle Pi values, not the mecanum robot's.
- **Base and network:** command topic, signs, units and the driver's command-loss timeout are
  unconfirmed. Wi-Fi delay must fit the guard's 0.3 s command timeout; WSL2 networking may
  block ROS discovery.
- **Real rooms:** the 2-D LiDAR sees only its own height (chair bases, table tops are invisible).
  Obstacles closer than ~2.6 m centre to centre leave no gap; the robot stops and waits.

## Before class

- [ ] From Haichen: `pose.pt`, `cap.pt`, `robot.pt` (and optional `toy.pt`), both cameras'
      `camera_matrix.npy` + `distortion_coefficients.npy`, and the two camera serials. Place
      them as named in [rig/assets/README.md](../rig/assets/README.md).
- [ ] `cp rig/rig.env.example rig/rig.env`; set `RIG_ROS_DOMAIN_ID` (not 30) and both serials.
      For step 6 onward the domain must match the robot's and `RIG_LOCALHOST_ONLY=0`.
- [ ] Prefer a native Linux machine with the cameras attached; WSL2 will likely not see the
      Basler cameras or the robot's topics.
- [ ] Bring a tape measure and floor tape.

## In class, in this order

### 1. Check the setup (nothing starts)

```bash
bash scripts/run_rig.sh --check
```

### 2. Measure the floor rectangle

Tape and label corners A, B, C, D as in
[RIG_BRIDGE.md](RIG_BRIDGE.md#calibrate-one-physical-rectangle) and enter the measured width and
height as `plane_width_m` / `plane_height_m` in `config/rig_bridge.yaml` (3.60 x 2.25 m are
placeholders).

### 3. Start cameras and the following chain (robot receives nothing)

```bash
bash scripts/run_rig.sh
```

Click the corners in each window in the order from [rig/README.md](../rig/README.md)
(human: D, C, A, B; robot: A, B, C, D).

### 4. Check perception

In a second terminal with the same `ROS_DOMAIN_ID`:

```bash
ros2 topic hz /owner_position      # need >= 4 Hz
ros2 topic hz /robot_position      # need >= 4 Hz
ros2 topic echo /owner/relative_position
```

- [ ] Stand on taped spots 1.5 m ahead, left and right of the robot; compare with the output
      (x forward, y left, metres).
- [ ] If left/right or forward/back are swapped, set `robot_heading_offset_rad` in
      `config/rig_bridge.yaml` (usually 0 or pi) and restart.
- [ ] Cover the robot's stickers: `/owner/visible` must go false within about a second.
- [ ] If a rate is below 4 Hz, raise `TOY_EVERY_N` or `PROCESS_EVERY_N` before touching
      timeouts.

### 5. Check the follower's intent

```bash
ros2 topic echo /following/cmd_vel
```

- [ ] Forward motion when you are more than 1.1 m ahead; turning toward your side; zero when you
      step out of view. (`/following/safe_cmd_vel` stays zero until the guard gets a real scan.)

### 6. Check the real LiDAR (robot on the same domain, see [ROBOT.md](ROBOT.md))

```bash
ros2 topic echo /scan --once --qos-reliability best_effort
```

- [ ] Note `header.frame_id`, the angle range (full 360 deg?), and what "no return" looks like
      (`inf`, `0.0` or `nan`).
- [ ] Watch the guard's log in the `run_rig.sh` terminal: it prints why it stops
      (`unexpected scan frame`, `invalid scan reading`, `incomplete scan coverage`, ...).

### 7. Obstacle avoidance dry run

`run_rig.sh` starts the chain without avoidance. Stop it (Ctrl+C) and start the parts manually,
each in its own terminal with ROS sourced and the rig domain exported
(`export ROS_DOMAIN_ID=<rig domain> ROS_LOCALHOST_ONLY=0`):

```bash
CAMERA_SERIAL=<human serial> CALIB_DIR=rig/assets/human \
  POSE_MODEL=rig/assets/human/pose.pt CAP_MODEL=rig/assets/human/cap.pt \
  python3 scripts/rig_human_publisher.py

CAMERA_SERIAL=<robot serial> CALIB_DIR=rig/assets/robot \
  ROBOT_MODEL=rig/assets/robot/robot.pt TOY_MODEL=rig/assets/robot/toy.pt \
  python3 scripts/rig_robot_publisher.py

ros2 launch ./launch/room_follow.launch.py source:=rig avoid:=true    # drive stays false
```

- [ ] Put a box between you and the robot; watch `ros2 topic echo /following/safe_cmd_vel`.
      It should steer around the box, not straight at it.
- [ ] Push the robot by hand past the box and check the commands change sensibly.

### 8. Real driving (only with the supervisor's agreement)

Only if steps 4-7 look right. `drive:=true` publishes to `/cmd_vel` but is tested only in
simulation, and the guard's geometry is the Waffle Pi's, not the mecanum robot's.

1. Wheels off the ground, connected to the robot's real command topic: check direction signs,
   and that killing the launch (and dropping Wi-Fi) stops the wheels.
2. On the floor, someone on the physical stop: no obstacles first, then one, then the
   three-obstacle layout at sim spacing (~2.6 m between centres).

## Write down

| Item | Value |
| --- | --- |
| `/owner_position` rate (Hz) | |
| `/robot_position` rate (Hz) | |
| Position error at taped spots (ahead / left / right) | |
| `robot_heading_offset_rad` | |
| Measured `plane_width_m` x `plane_height_m` | |
| `/scan` frame, angle range, "no return" value | |
| Robot command topic, message type, command-loss timeout | |
| Guard stop reasons seen | |

Those numbers decide what needs changing before real following (guard scan frame/format,
footprint, owner exclusion radius, timeouts). Steps 1-5 are a realistic goal for one class;
6 and 7 depend on getting the robot onto your ROS network.
