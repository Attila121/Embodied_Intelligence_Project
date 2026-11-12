# Real-camera rig (local dry run)

Runs Haichen's human/cap detector and robot detector on two Basler cameras and feeds
their `Pose2D` streams through the [rig bridge](../docs/RIG_BRIDGE.md) into the follower
and guard. **It is a local dry run, not a hardware driving procedure:** `drive` stays
false, so guarded commands go only to `/following/safe_cmd_vel` and nothing commands a
physical robot. Driving a real robot additionally needs [docs/ROBOT.md](../docs/ROBOT.md)
and a guard suited to it.

## 1. Drop in the files

Copy the models and calibration into [assets/](assets/README.md) under the names listed
there (`human/pose.pt`, `human/cap.pt`, `robot/robot.pt`, `robot/toy.pt` optional, and a
`camera_matrix.npy` + `distortion_coefficients.npy` pair for **each** camera). The real cap
model is required; there is no YOLO-World fallback and no `clip` dependency.

## 2. Configure

```bash
cp rig/rig.env.example rig/rig.env     # ignored by Git
```

Fill in `RIG_ROS_DOMAIN_ID` (anything but 30, which is the simulation's) and the two
camera serials, which must be different attached cameras. Set `RIG_PYTHON` if the
publishers' Python lives in a venv; it needs rclpy (source ROS or use
`--system-site-packages`), ultralytics, pypylon, opencv and numpy.

## 3. Check, then run

```bash
bash scripts/run_rig.sh --check   # validates everything, starts nothing
bash scripts/run_rig.sh           # human publisher + robot publisher + bridge/follower/guard
```

`--check` verifies: the model files are complete PyTorch archives, each calibration is a
finite 3x3 matrix plus 4-14 distortion values, `ultralytics` and `pypylon` import, and both
serials are set, different and attached. It checks the exact paths the publishers will
receive, including any overrides in `rig.env`. The launcher exports your `rig.env` domain
and localhost setting after sourcing ROS, so the rig never inherits the simulation's
domain 30, and gives each publisher its own `CAMERA_SERIAL` and `CALIB_DIR`.

Each publisher then opens a window and asks for four floor corners; follow the corner
table in [RIG_BRIDGE.md](../docs/RIG_BRIDGE.md#calibrate-one-physical-rectangle) (human:
TL, TR, BL, BR = D, C, A, B; robot: BL, BR, TR, TL = A, B, C, D). Ctrl+C stops everything
(SIGINT, then SIGTERM, then SIGKILL, per process group); if any process dies the launcher
reports which and stops the rest.

## 4. Acceptance checks on the real rig

```bash
ros2 topic hz /owner_position ; ros2 topic hz /robot_position   # aim for >= 4 Hz each while visible
ros2 topic echo /owner/relative_position ; ros2 topic echo /owner/visible
ros2 topic echo /following/cmd_vel     # what the follower wants (the guard output stays zero without a scan)
```

The bridge treats an input older than 0.5 s as lost, so a rate under about 4 Hz makes
following flicker. Both publishers log their measured rates every 5 s and the robot
publisher warns below 4 processed frames/s. Cover the robot's stickers: `/robot_position`
must stop and `/owner/visible` must go false within about a second.

## What differs from Haichen's originals

The detection, drawing, homographies and published values are unchanged. The originals
stay untouched in `EI_Human_Tracking26-27-1-main/` and the sibling worktree.

| Area | Original | Rig copy |
| --- | --- | --- |
| Paths | hardcoded `/home/haichen/...` | `rig/assets/`, overridable in `rig.env` |
| Camera | first enumerated camera | named by serial; refuses to guess; two distinct serials required |
| Robot tracking loss | republishes the last pose forever | publishes nothing |
| Failed grab | reuses the previous frame; result not released | skipped; every grab result released |
| Robot cadence | processes every 4th frame | every frame (`PROCESS_EVERY_N`), toy every 4th (`TOY_EVERY_N`) |
| Calibration | human: silently skipped if missing | required for both |
| Cap model | YOLO-World stand-in by default | real model file required (`CAP_CONF` default 0.3) |

## Not verified

No Basler camera or Haichen's models were available when this was written. The pipeline
logic, grab/release behaviour, asset and serial checks, launcher environment and process
supervision are covered by tests in `tests/` (the camera and YOLO calls are replaced by
fakes). Real-camera frame rates, detection accuracy and the corner calibration remain to be
measured on the rig.
