# Camera-based owner tracking with an ArUco marker

The owner is the visible holder of ArUco **DICT_4X4_50, ID 23**. Its outer black
square is **0.15 m** on each side. This is a visible identifier, not face
recognition or a way to authenticate a person: anyone holding the same marker
can appear as the owner. Two matching markers in view are treated as ambiguous.

`owner_tracker.py` detects the selected ID in `/camera/image_raw`, uses calibration
from `/camera/camera_info` and the known marker size to estimate its metric pose,
then uses TF at the image timestamp to transform it into `base_footprint`. No
Gazebo target coordinates or synthetic owner positions enter this measurement.

The tracker publishes:

| Topic | Type | Meaning |
| --- | --- | --- |
| `/owner/relative_position` | `geometry_msgs/Point` | Measured position used by the controller: x forward, y left, z up |
| `/owner/observation` | `geometry_msgs/PointStamped` | The same observation with source timestamp and robot frame |
| `/owner/visible` | `std_msgs/Bool` | False on missing, ambiguous, invalid or stale owner detections |
| `/owner/debug_image` | `sensor_msgs/Image` | Camera image with detection outline and tracking status |

The follower clears its target when visibility becomes false. Independent target,
command and LiDAR timeouts continue to apply. Duplicate IDs, undersized detections,
bad calibration, missing TF, inconsistent image dimensions, large reprojection
errors and stale images produce no owner position. A wall-clock image watchdog
still stops tracking if camera images or the simulation clock stop updating.

The existing guard can stop for obstacles and block turns near walls. This stage
does not navigate around obstacles or search for an owner after losing sight.
Keep the marker visible and facing the camera; move slowly for the first demo.

## Four-terminal Gazebo demonstration

First stop the old following launch, all synthetic target publishers, exploration
and teleop. There must be only one owner-position publisher. Use an empty world
for this test so the earlier room's walls do not obscure the marker or block turns.

### Terminal 1: empty world and robot

Stop the old Gazebo launch with Ctrl+C before starting another world.

```bash
cd ~/turtlebot3_ws/src/Embodied_Intelligence_Project
LIBGL_ALWAYS_SOFTWARE=true GALLIUM_DRIVER=llvmpipe \
  bash scripts/run_ros_experiment.sh world-empty
```

Wait for the robot. Keep the simulation unpaused.

### Terminal 2: camera tracker, controller and guard

```bash
cd ~/turtlebot3_ws/src/Embodied_Intelligence_Project
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=30 ROS_LOCALHOST_ONLY=1
ros2 launch ./launch/follow.launch.py tracker:=true
```

This defaults to a dry run. The tracker should say the owner marker is not visible
until you create it in Terminal 4. Guarded commands go to `/following/safe_cmd_vel`.

After confirming detection and the measured position, stop this launch with Ctrl+C
and enable simulation movement:

```bash
ros2 launch ./launch/follow.launch.py tracker:=true drive:=true
```

The robot should approach the stationary marker and stop roughly 1.0–1.1 m from
its centre. This differs from the earlier synthetic constant offset: the measured
distance actually decreases as the robot approaches.

### Terminal 3: observe the measured owner position

```bash
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=30 ROS_LOCALHOST_ONLY=1
ros2 topic echo /owner/relative_position
```

With the initial stationary robot and marker, expect x near 1.5 m, y near zero,
and z near 0.2 m. The camera measurement has some noise. During movement x should
decrease. No output is expected while the marker is not visible.

To see what the robot sees, stop the echo and run instead:

```bash
LIBGL_ALWAYS_SOFTWARE=true GALLIUM_DRIVER=llvmpipe \
  ros2 run rqt_image_view rqt_image_view /owner/debug_image
```

### Terminal 4: create the visible owner marker

```bash
cd ~/turtlebot3_ws/src/Embodied_Intelligence_Project
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=30 ROS_LOCALHOST_ONLY=1
python3 scripts/owner_marker_target.py spawn --distance 1.5
```

This adds a small black-and-white board 1.5 m ahead of the robot model `waffle_pi`.
The board centre is 0.2 m above the robot origin so its low camera can see it. The
script exits; the board remains. It does not publish `/owner/relative_position`.

With movement enabled, select `owner_marker` in Gazebo and use its move tool to
move the board slowly away from the robot. Keep its black-and-white face oriented
toward the robot and its height near 0.2 m. The camera should measure the new
position and the robot should follow when there is clearance. If you move it
outside the camera view, the robot stops rather than guessing its location.

Remove the marker to test owner loss:

```bash
python3 scripts/owner_marker_target.py remove
```

The robot should stop and `/owner/visible` should become false. To recreate the
board, rerun `spawn`. If it already exists, remove it before spawning again.

To verify wrong-ID rejection, remove ID 23 and spawn another ID:

```bash
python3 scripts/owner_marker_target.py spawn --marker-id 24
```

The tracker should report that the visible marker does not match the owner. Do
not run the old synthetic publisher alongside camera tracking.

## Printed marker and configuration

The printable [SVG](../assets/owner_marker_23.svg) is 200 mm square including its
white quiet zone. Print at **100% scale**, without fit-to-page resizing. Measure
the outer black square with a ruler: it must be **150 mm**, not 200 mm. Mount it
flat, keep its white border, and hold it at a height visible to the camera.

Generate a different print with:

```bash
python3 scripts/owner_marker_target.py print --output /tmp/owner_marker.svg
```

Settings are in `config/owner_tracking.yaml`. If you change the selected ID or
black-square size, use exactly the same values for the printed/spawned marker.
Restart the tracker after changing settings. Physical camera use requires its
own correct calibration and TF, with `use_sim_time: false`; this guide validates
the Gazebo setup, not physical robot operation.

The implementation uses [OpenCV ArUco pose estimation](https://docs.opencv.org/4.5.3/d9/d6a/group__aruco.html).
The Gazebo board uses ordinary model geometry and built-in black/white materials,
without external texture files.

## Verification

```bash
source /opt/ros/humble/setup.bash
python3 -m unittest discover -s tests -v
```

Tests cover projected marker detection, metric pose, distractor/duplicate IDs,
calibration rejection, coordinate transforms, visibility loss and the existing
motion guard. Rendering and ROS pipeline tests additionally check a real Gazebo
camera in an isolated world with robot movement disabled. With a working display,
run the separate rendering check using:

```bash
python3 tests/rendered_owner_tracking_smoke.py
```

It starts and stops its own Gazebo server on a separate port and ROS domain,
writes temporary results under `/tmp`, and never publishes `/cmd_vel`.
