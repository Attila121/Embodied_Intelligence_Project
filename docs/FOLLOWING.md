# Owner following: simulation controller and command guard

For measured camera targets, see [owner marker tracking](OWNER_TRACKING.md).
The synthetic target instructions below remain useful for controller-only tests;
do not run the synthetic publisher while camera tracking is active.

This stage tests movement from a supplied relative target. It does not detect
people or identify the owner. `/owner/relative_position` is a `geometry_msgs/Point`
in metres relative to `base_footprint`: x forward, y left. Positions must be
updated continuously; these messages currently have no source timestamp or
identity field. A later perception node must transform detections into this
frame, reject old detections, and publish only the selected owner.

The controller publishes `/following/cmd_vel`. A separate guard accepts only
fresh forward/turn commands and fresh, dense, full-circle `/scan` data in
`base_scan`. Straight driving checks a forward corridor extending 0.55 m from the
scanner and 0.28 m to each side. This allows the robot to drive away from a rear
wall. Turning or curved driving still requires 0.55 m clearance in every direction.
An obstacle within 0.30 m of the scanner blocks either motion, and invalid readings
always stop movement. These margins are specific to the simulated Waffle Pi.
Positive infinity means no return within the scanner's maximum range. This version
stops for obstacles; it does not plan a path around them. Blocked logs include the
distance and bearing of the closest blocking return (zero forward, positive left).

The guard checks commands every 0.05 seconds, times out controller messages after
0.3 seconds and scans after 0.6 seconds, and caps speed at 0.12 m/s and 0.5 rad/s.
It ramps acceleration but sends stop commands immediately. Controller target
updates time out after 0.5 seconds. The guard uses wall/steady time so its watchdog
does not depend on the simulation clock advancing. Normal guard shutdown sends
zero commands. Forced termination or failure of the guard itself requires a
separate base-level velocity timeout; this guard alone cannot cover that failure.
This is a supervised Gazebo demonstration, not a physical-robot safety system.

## Numbered terminal setup

Stop previous following nodes, exploration, Nav2 and keyboard controllers first.
Only the guard should publish `/cmd_vel` during this experiment. Each terminal is
inside the container.

### Terminal 1: simulation

Leave the existing world running if already started. Otherwise:

```bash
cd ~/turtlebot3_ws/src/Embodied_Intelligence_Project
LIBGL_ALWAYS_SOFTWARE=true GALLIUM_DRIVER=llvmpipe bash scripts/run_ros_experiment.sh world
```

### Terminal 2: controller and guard

```bash
cd ~/turtlebot3_ws/src/Embodied_Intelligence_Project
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=30 ROS_LOCALHOST_ONLY=1
ros2 launch ./launch/follow.launch.py
```

The default is a dry run: guarded commands go to `/following/safe_cmd_vel` and do
not move the robot. The guard prints the current reason for allowing or blocking
movement, such as `following`, `controller timeout`, `scan timeout` or an obstacle.

To enable Gazebo movement, stop this launch with Ctrl+C and restart it with:

```bash
ros2 launch ./launch/follow.launch.py drive:=true
```

### Terminal 3: observe guarded commands

```bash
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=30 ROS_LOCALHOST_ONLY=1
ros2 topic echo /following/safe_cmd_vel
```

When `drive:=true`, observe `/cmd_vel` instead. Before driving,
`ros2 topic info /cmd_vel --verbose` should show exactly one publisher,
`following_guard`. Use `ros2 topic info /scan` to check that scans are available.

### Terminal 4: short synthetic target test

```bash
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=30 ROS_LOCALHOST_ONLY=1
ros2 topic pub --rate 10 /owner/relative_position geometry_msgs/msg/Point \
  "{x: 2.0, y: 0.0, z: 0.0}"
```

Stop the publisher with Ctrl+C after one or two seconds. Commands should become
zero within roughly 0.55 seconds. This synthetic target is a constant offset from
the moving robot, so it does NOT get closer as the robot drives: it is suitable
only for short movement/timeout tests, not for demonstrating distance keeping.
It does not create a person in Gazebo.

To check distance keeping, repeat with x=0.8 and y=0.0: commands should remain
zero. For forward obstacle checks, place an object in the corridor ahead of the
scanner before sending a target, and verify the guard blocks movement. To test
turning, send y=0.3 in an open area; a nearby rear wall should block the turn.
Do not repeatedly
approach obstacles to test the threshold. Verify clear-space scans and the guard
log before trying movement; it can intentionally refuse to move in narrow areas.

For the controller watchdog test, stop the synthetic target publisher first,
then run controller and guard in separate terminals instead of the combined
launch. Keep the guard running, stop the controller, and check the guarded output
stays zero. Keep this test on `/following/safe_cmd_vel` until verified.

## Automated checks

```bash
source /opt/ros/humble/setup.bash
python3 -m unittest discover -s tests -p 'test_following_guard.py' -v
```

These test timeout handling, obstacle rejection, malformed scans and commands,
velocity limits, acceleration and recovery. They do not require a running Gazebo.
