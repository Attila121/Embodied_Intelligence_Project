# Human/cap rig bridge

The bridge adapts Haichen's `/owner_position` and `/robot_position`
(`geometry_msgs/msg/Pose2D`) to the existing follower's
`/owner/relative_position` (`geometry_msgs/msg/Point`, metres, x forward,
y left, z zero) and `/owner/visible` (`std_msgs/msg/Bool`). The robot publisher
uses the direction from its green sticker to its red sticker. Stranger and toy
topics are ignored. The follower and simulation LiDAR guard are unchanged.

This is an integration/dry-run milestone. Physical robot driving requires the
separate [robot procedure](ROBOT.md), verified base interfaces and a suitable
guard. The current guard assumes the simulated Waffle Pi footprint, full-circle
LiDAR and aligned `base_scan`. It checks obstacles, not owner identity.

## Start a dry run

In the `following-human-bridge` worktree, with ROS 2 Humble sourced:

```bash
source /opt/ros/humble/setup.bash
ros2 launch ./launch/room_follow.launch.py source:=rig
```

The default `drive:=false` publishes guarded commands to
`/following/safe_cmd_vel`. It does not publish `/cmd_vel`. `drive:=true` retains
the existing simulation-motion behavior; it is not a hardware launch procedure.
`source:=markers` is still the default and starts the original ArUco tracker.
Only run one source publishing the follower's owner topics at a time. The rig
launch starts the bridge, follower and guard; it does not start the cameras.
Without a valid scan the guard remains stopped, even when the follower requests
motion on `/following/cmd_vel`.

For the bridge alone (no controller or guard):

```bash
python3 scripts/rig_bridge.py --ros-args --params-file config/rig_bridge.yaml
```

With real cameras, use the one-command local dry run described in
[rig/README.md](../rig/README.md): `bash scripts/run_rig.sh`. It validates the
models, calibration and two distinct camera serials, then starts
`scripts/rig_human_publisher.py`, `scripts/rig_robot_publisher.py` and the launch
above under one supervisor. The two publishers are rig copies of Haichen's
`new_with_pub_previous.py` and `send_pos_2.py` (originals untouched); each opens
only the camera named by its own serial and never guesses between cameras.

The bridge itself needs only the ROS messages. The rig needs the correct per-camera
intrinsics, distortion arrays and models: cap `best.pt`, robot `robot_test.pt`, toy
`best (4).pt` (optional) and both cameras' `*.npy` files. Those assets are not
provided by this change; copy them into `rig/assets/` as described in
[rig/assets/README.md](../rig/assets/README.md). There is no YOLO-World substitute in
the rig publisher: the real cap model is required, and it
must be evaluated on the actual rig.

For simulated observations, use the simulation's domain/network settings. For
real camera observations use the agreed hardware domain, network and wall-time
settings from [ROBOT.md](ROBOT.md); do not run the simulation wrapper to connect
to the robot. All publishers, bridge and follower must share the ROS domain and
compatible middleware/network settings.

## Calibrate one physical rectangle

Measure width W and height H and enter `plane_width_m` / `plane_height_m` in
`config/rig_bridge.yaml`. The defaults 3.60 x 2.25 m are placeholders until
measured. These are dimensions of the same floor rectangle, not separate fields
for each camera.

Name its physical corners using the common room frame:

```text
D = (0,H) -------- C = (W,H)
    |                  |
A = (0,0) -------- B = (W,0)
             room +x ->     room +y points from A to D
```

Record A/B/C/D with a room sketch or floor labels before clicking. Each camera
must assign these same physical corners to the same world axes, regardless of
where they appear in its image. Image top-left alone is not a corner identity.

| Publisher | Click physical corners | Existing coordinate system |
| --- | --- | --- |
| Human (`rig_human_publisher.py`) | D, C, A, B (its TL, TR, BL, BR prompts) | origin D, x right, y down; 3.60 x 2.10 |
| Robot (`rig_robot_publisher.py`) | A, B, C, D (its BL, BR, TR, TL prompts) | origin A, x right, y up; 3.60 x 2.25 |

Keep the publisher homography constants unchanged when using this bridge's
conversion. If those constants change, update the bridge's source dimensions
as well. A flipped/permuted set of corners cannot be repaired by changing W/H.

The bridge maps robot x/y by W/3.60 and H/2.25, and human x/y by
`x_room = x*W/3.60`, `y_room = H - y*H/2.10`. Direction vectors undergo the same
axis scaling before `atan2`; human y is also reflected. This matters for
diagonal headings when the two scale factors differ. Human heading is validated
and converted but does not affect following.

Drive the robot forward under the supervisor's established controller and
compare its displacement with the reported green-to-red direction. Set
`robot_heading_offset_rad` so that the corrected heading points forward
(typically 0 or pi). The parameter accepts any finite offset. Check multiple
headings and place the owner at known points ahead, left and right before
trusting relative outputs.

Optional parallax correction is disabled by `robot_marker_height_m: 0.0`.
To enable it, measure the tracked point's height h, camera height Z above the
same floor, and the camera's vertical floor projection F in room coordinates.
Set `robot_marker_height_m`, `robot_camera_height_m`, and
`robot_camera_foot_x_m` / `robot_camera_foot_y_m`. The correction is
`P_ground = F + (1-h/Z)*(P_plane-F)` and requires `0 <= h < Z`.
It assumes the reported point represents a physical feature at height h;
a bounding-box centre may not satisfy that assumption exactly. It does not
correct camera calibration errors or a sticker-to-base positional offset.

## Freshness and limitations

The bridge publishes at 10 Hz using a steady timer. Each input has its own
monotonic receive timestamp and 0.5 s timeout. Both must be valid and fresh;
otherwise visibility is false and no position is published. Non-finite x/y/theta
or a converted point outside the room plus `field_margin_m` (default 0.10 m)
immediately invalidates that input and sends visibility false. Recovery requires
a fresh valid message for the invalid input and a fresh valid other input.
Reading or republishing cached state never extends its receive-time lifetime.

For publisher loss, the normal stopping budget is the bridge timeout plus one
bridge period (0.10 s) and one follower period (0.05 s), plus transport/scheduling
delay. The follower's own 0.5 s timeout also stops commands if the bridge dies.
Invalid samples trigger visibility false from the callback without waiting for
the timer. Freshness parameters are startup settings; restart after edits.

**Robot tracking loss.** The original `send_pos_2.py` repeats its last valid pose on
every failed detection (and keeps using the previous frame after a failed grab).
Those messages pass a receive-time check, and an unchanged pose cannot distinguish a
stationary robot from lost tracking, so a bridge cannot detect it. Use
`scripts/rig_robot_publisher.py` instead: it publishes only for a newly acquired
frame with the robot detected and both stickers found, and publishes nothing
otherwise, so the 0.5 s timeout above does the stopping. This is covered by
`tests/test_rig_robot_publisher.py`. Any other robot producer must meet the same rule.

**Publish cadence.** The timeout leaves little margin for a slow producer. The rig
publishers log their measured rates every 5 s and the robot publisher warns when it
processes fewer than 4 frames/s. Confirm on the real rig with `ros2 topic hz
/owner_position` and `ros2 topic hz /robot_position` (aim for at least 4 Hz each while
visible) before relying on following; if lower, reduce inference cost before touching the
timeouts. Real-camera cadence has not been measured yet.

`Pose2D` has no capture timestamp or frame ID. Receipt times cannot measure
camera exposure skew, inference delay or queued old observations. The streams
are combined using the latest received samples, not synchronized captures.
Owner/stranger assignment is also per frame and can switch people. Neither the
bridge nor the LiDAR guard establishes identity continuity. Evaluate cap
assignment separately with `scripts/test_cap_detection.py` in the original
worktree. These limitations remain outside this dry-run implementation.

## Verification

Pure Python on Windows, no new dependencies:

```powershell
python -m unittest discover -s tests -p test_rig_geometry.py -v
```

In Linux/WSL with ROS Humble and the existing OpenCV/NumPy dependencies:

```bash
source /opt/ros/humble/setup.bash
python3 -m unittest discover -s tests -v
```

`tests/test_rig_bridge_ros.py` sends synthetic `Pose2D` streams through the real
bridge and unchanged follower, using process-specific topic remaps. It asserts
forward and turning commands for a scripted robot track, stopping when either
publisher disappears, immediate invalidation and recovery. It loads the real
bridge YAML and does not publish `/cmd_vel` or require Gazebo/cameras. It skips
when ROS Python bindings are absent. Run it alone with:

```bash
python3 -m unittest discover -s tests -p test_rig_bridge_ros.py -v
```

For manual observation during a dry run:

```bash
ros2 topic echo /owner/relative_position
ros2 topic echo /owner/visible
ros2 topic echo /following/cmd_vel
```

Use an owner more than 1.1 m away when checking turn direction: the existing
follower suppresses both driving and turning inside the following distance.
With hardware available, compare measured relative positions at several marked
floor locations and robot headings, then test tracking loss explicitly. The
synthetic replay validates integration, not camera/model accuracy. The rendered
Gazebo test in [ROOM_FOLLOWING.md](ROOM_FOLLOWING.md) remains optional.
