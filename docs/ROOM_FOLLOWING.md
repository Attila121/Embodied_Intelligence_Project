# Owner following with two fixed room cameras

This demo matches the planned external-camera arrangement: a robot in a room,
with one fixed camera at the back-left and another at the back-right. The cameras
observe upward-facing markers on the owner target and robot. The robot's onboard
camera sensor is omitted from this demo world. The third-party robot model
on disk remains unchanged.

The room is 6 × 6 m. The Waffle Pi starts at (-1, 0), facing +x, and the blue owner
stand starts at (1, 0). Both cameras are raised above the back corners and look
down into the room. The stand represents a movable owner marker, not a detected
human body.

| Object | ArUco dictionary / ID | Outer black square | Mount |
| --- | --- | --- | --- |
| Owner stand | DICT_4X4_50 / 23 | 0.30 m | Upward-facing, 0.75 m high |
| Robot | DICT_4X4_50 / 24 | 0.24 m | Upward-facing; pose in `base_footprint` is (-0.064, 0, 0.32), with aligned axes |

The markers are sized so the room cameras can detect them at a distance. Scene generation and
tracker configuration must use the same sizes and robot-marker mounting pose.

## How tracking works

Each camera independently detects both IDs **in the same image**, uses its own
lens calibration to estimate their poses, and calculates:

```text
owner in robot frame = robot-marker mount × inverse(camera-to-robot-marker) × camera-to-owner
```

The robot marker supplies the robot's heading as well as its position. A separate
room/map localization system is not needed for this relative calculation. Camera
placement cancels out of the transform when both markers are observed together.
No robot or owner coordinates from Gazebo's state services enter the tracker.
Those services are used only by the target-movement tool and integration test.

The two views are checked for freshness and timestamp proximity. Agreeing views
are averaged; a fresh single view can continue when the other loses a marker.
Conflicting simultaneous measurements, duplicate selected IDs, or loss of both
usable views stop following. Old observations are not republished as new target
measurements. Timeouts use wall time even if Gazebo is paused.

The existing following controller and LiDAR guard receive the resulting
`/owner/relative_position` in metres: **x forward, y left, z up**, relative to the
robot. The controller uses x/y and keeps about 1.0–1.1 m distance. The guard stops
for obstacles and stale commands/scans. This demo does not plan routes around
obstacles or search for a lost owner.

## Run step by step

Stop any old Gazebo world, following launch, exploration, and teleop first. Run only this room's tracker as the owner-position
publisher. All four terminals are inside the container.

### Terminal 1 — room, robot, owner stand and both cameras

```bash
cd ~/turtlebot3_ws/src/Embodied_Intelligence_Project
LIBGL_ALWAYS_SOFTWARE=true GALLIUM_DRIVER=llvmpipe \
  bash scripts/run_ros_experiment.sh world-room
```

Wait for the room and robot. Keep Gazebo unpaused. The marker stand and robot
marker are already included; no separate spawning step is required.
The blue camera housings sit above the back corners of the room.

### Terminal 2 — tracking, controller and guard (dry run)

```bash
cd ~/turtlebot3_ws/src/Embodied_Intelligence_Project
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=30 ROS_LOCALHOST_ONLY=1
ros2 launch ./launch/room_follow.launch.py
```

Expect camera logs such as `left: sees owner and robot` and
`tracking owner (left+right)`. A single fresh camera is also valid. In this mode,
guarded commands go to `/following/safe_cmd_vel`, so the robot stays stationary.

### Terminal 3 — measured target position

```bash
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=30 ROS_LOCALHOST_ONLY=1
ros2 topic echo /owner/relative_position
```

Initially x should be near **2 m** and y near **0 m**. Small camera measurement
errors are expected. To inspect a camera, stop the echo and run instead:

```bash
LIBGL_ALWAYS_SOFTWARE=true GALLIUM_DRIVER=llvmpipe \
  ros2 run rqt_image_view rqt_image_view /room_cameras/left/debug_image
```

For the other view, use `/room_cameras/right/debug_image`. Green outlines mark the
owner; blue/orange outlines mark the robot.

### Enable movement in Terminal 2

After confirming tracking, press Ctrl+C in Terminal 2 and run:

```bash
ros2 launch ./launch/room_follow.launch.py drive:=true
```

The robot should approach the owner stand and stop at the following distance.

### Terminal 4 — move the owner stand

```bash
cd ~/turtlebot3_ws/src/Embodied_Intelligence_Project
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=30 ROS_LOCALHOST_ONLY=1
python3 scripts/room_target.py move --x 1.5 --y 0.5 --duration 5
```

This moves the visible stand smoothly to **room coordinates** (1.5, 0.5). It does
not publish an owner-position topic. The cameras must measure its new position
and the robot should follow. You can also move the stand using Gazebo's move
tool; keep its top marker horizontal and its base on the floor.

Test owner loss:

```bash
python3 scripts/room_target.py remove
```

The robot should stop. Recreate the stand to resume tracking:

```bash
python3 scripts/room_target.py spawn --x 1.5 --y 0.5
```

To finish, Ctrl+C in Terminal 2 first, then Terminal 1. Stop the topic echo or
camera viewer in Terminal 3 as well.

## Settings and troubleshooting

Settings are in `config/room_following.yaml`. Both selected IDs must be distinct.
Every usable camera must see both markers in its image; this version does not
combine a robot-only view from one camera with an owner-only view from another.
The markers identify visible marker holders, not authenticated people.

| Message / symptom | Check |
| --- | --- |
| Neither camera sees both markers | Inspect both debug images; verify marker sizes, room world and camera topics |
| Camera measurements disagree | Verify robot-marker mount and both calibrations; keep target movement slow |
| Stale image or clock mismatch | Keep Gazebo unpaused and use `use_sim_time: true` for this demo |
| Robot does not move despite tracking | Use `drive:=true`; inspect the guard's obstacle/scan status |
| Camera images are slow | Close other Gazebo worlds and heavy processes; the watchdog can intentionally stop under excessive delay |

The camera topics are `/room_cameras/{left,right}/image_raw` and
`/room_cameras/{left,right}/camera_info`. The tracker also publishes
`/owner/observation`, `/owner/visible`, and the two debug images. Physical room
use still requires real camera calibration, a measured robot-marker mount,
sufficient marker visibility and the real robot's verified base interfaces.

## Verification

```bash
source /opt/ros/humble/setup.bash
python3 -m unittest discover -s tests -v
```

For the rendered room test, also source the simulation workspace:

```bash
source ~/turtlebot3_ws/install/setup.bash
python3 tests/rendered_room_following_smoke.py
```

The test uses a separate ROS domain and Gazebo port, writes temporary logs/images
under `/tmp`, and moves only its own simulated robot. It checks two-view tracking,
camera handover, a 90-degree robot turn, target movement, actual robot movement,
owner-loss stopping and paused-simulation stopping.
