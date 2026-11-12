# Simulation following demo

One supervised command starts the room world, both fixed cameras, the follower and a
scripted owner walk. No models or hardware are needed: the cameras see ArUco markers on
the robot (24) and the owner stand (23), as described in
[ROOM_FOLLOWING.md](ROOM_FOLLOWING.md). Requires the simulation setup from
[SIMULATION.md](SIMULATION.md) (Ubuntu 22.04, ROS 2 Humble, Gazebo Classic, TurtleBot3).

```bash
bash scripts/run_ros_experiment.sh follow-demo            # with the Gazebo window
bash scripts/run_ros_experiment.sh follow-demo --headless # no window (CI, slow machines)
```

Options: `--no-walk` (start the world and follower, then move the stand yourself with
`scripts/room_target.py`), `--obstacles` (add floor obstacles and obstacle avoidance, with a
longer walk; see [OBSTACLES.md](OBSTACLES.md)) and `--owner-speed M/S` (default 0.06, maximum
0.10; the follower and LiDAR guard limit the robot to 0.12 m/s). It uses ROS domain 30 and
software rendering, like the other simulation commands.

## What you should see

Each stage waits for **received messages**, not for topics to exist, and the command fails
with a message if a stage times out:

1. **Cameras up**: an image has arrived from both room cameras (up to 4 min on software GL).
2. **Owner seen and approached**: `/owner/visible` turns true and the robot drives forward
   from 2 m until the measured distance has stayed within 0.9–1.2 m for 3 s. The follower
   settles at roughly 1.0–1.1 m; the band absorbs camera measurement noise.
3. **Scripted walk**: the stand moves to (2.0, 0.8), (2.0, -1.0) and (0.9, -1.9) at the
   chosen speed, with an 8 s pause and a settle check at every waypoint. The robot turns
   toward the stand and re-settles each time. If the machine renders slowly the robot
   lags in wall time, so each settle check allows up to 2 min.
4. **Loss test**: the stand is removed. The demo then requires `/owner/visible` to be
   false *and* `/cmd_vel` to be exactly zero for 1 s, so the robot is verified stopped.
5. **Recovery**: the stand is respawned at (1.5, 0.5) and the demo waits until it is
   visible again.

When it prints `Demo walk complete`, the world and follower **keep running** so you can
keep experimenting (`python3 scripts/room_target.py move --x 1.5 --y 0.5`, `remove`,
`spawn`). Press Ctrl+C to stop.

## Stopping and failures

Ctrl+C, `kill` (SIGTERM) or the terminal closing stops every process group: SIGINT first,
SIGTERM after 10 s, SIGKILL after a further 5 s. If Gazebo or the follower exits on its own
the command reports which one and shuts the rest down instead of hanging. Check for leftovers
with `pgrep -fa 'gzserver|owner_follower|room_owner_tracker|following_guard'`.

This demo is **simulation only**. It validates the follower, the guard and the
camera-based owner measurement in Gazebo, not the real robot's mecanum drive or the YOLO
detectors; for those see [RIG_BRIDGE.md](RIG_BRIDGE.md) and [../rig/README.md](../rig/README.md).
