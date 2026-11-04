# Autonomous exploration (simulation)

The robot explores an unknown room by itself and builds a map. Nobody drives it with the keyboard.

Three ready-made packages work together:

| Package | Job |
| --- | --- |
| `slam_toolbox` | Builds the map from LiDAR and finds where the robot is on it |
| Nav2 | Drives the robot to a goal without hitting walls |
| `explore_lite` | Finds the edge between known and unknown space (a *frontier*), sends it to Nav2 as the next goal, and stops when no frontiers are left |

Exploration uses `slam_toolbox`, not Cartographer. The manual mapping commands (`slam`, `teleop`, `save`) still work as before.

## 1. Install once

```bash
sudo apt update
sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup ros-humble-slam-toolbox
bash scripts/run_ros_experiment.sh setup-explore
```

`setup-explore` downloads `explore_lite` into your TurtleBot3 workspace (`~/turtlebot3_ws`, or `TB3_WORKSPACE`) and builds it. It takes a few minutes.

Open a **new** terminal and check:

```bash
bash scripts/run_ros_experiment.sh check
```

Every line must say `OK`.

## 2. Run one exploration

Open terminals in the repository folder. Start them in this order.

```bash
# Terminal A: Gazebo world. Wait until the robot appears.
bash scripts/run_ros_experiment.sh world
```

```bash
# Terminal B: metrics recorder. Start it BEFORE exploration.
bash scripts/run_ros_experiment.sh metrics --world turtlebot3_world
```

```bash
# Terminal C: slam_toolbox + Nav2 + RViz, then explore_lite after 15 seconds
bash scripts/run_ros_experiment.sh explore
```

What you should see:

1. RViz opens and the map starts to appear.
2. After 15 to 30 seconds the robot starts moving by itself.
3. In RViz, click **Add → By topic → /explore/frontiers → MarkerArray** to see the frontiers.
4. When the map is complete, terminal C prints `No frontiers found, stopping.` (or `All frontiers traversed/tried out, stopping.`). The robot drives back to its start point.
5. Terminal B prints the results and saves one row to `maps/exploration_runs.csv`.

In a test run in `turtlebot3_world`, exploration finished in about 58 simulated seconds, drove about 10.5 m and mapped about 21 m² (the whole hexagon with all nine pillars). Your numbers should be close.

Save the map while everything is still running:

```bash
# Terminal D
bash scripts/run_ros_experiment.sh save-explore
```

The map goes to a new folder, `maps/explore_<date>_<time>/`, so nothing is overwritten.

To finish, press `Ctrl+C` in terminal C, then in terminal A.

**Before every new run, restart terminals A, B and C.** If you keep `slam_toolbox` running, the next run starts with the old map.

## 3. Pause, help, resume

```bash
bash scripts/run_ros_experiment.sh explore-stop     # robot stops
bash scripts/run_ros_experiment.sh explore-resume   # exploration continues
```

If the robot is stuck: run `explore-stop`, open `teleop`, drive the robot free, **close teleop**, then run `explore-resume`. Never drive with the keyboard while exploration is moving the robot: both write to `/cmd_vel`.

Each pause is counted in the metrics as a manual help. The paused time is included in `duration_s`.

## 4. Try a harder room

Use the house world instead of the small world:

```bash
bash scripts/run_ros_experiment.sh world-house
bash scripts/run_ros_experiment.sh metrics --world turtlebot3_house
bash scripts/run_ros_experiment.sh explore
```

The house is bigger, so exploration takes longer. Exploration only finishes in a closed space.

In our test on a slow computer (2 CPU cores), the house run did **not** finish: Nav2 was too slow, the robot got stuck near the start, and only part of the first room was mapped. Try it on a normal PC. If it still stalls, use section 6 and the problems table, and write down what happened. This is the main open task for the evaluation phase.

## 5. Record three runs

Do at least three runs in the same world. Restart A, B and C each time.

To get a coverage percentage, give a complete map of the same world as a reference. Your manual Cartographer map works:

```bash
bash scripts/run_ros_experiment.sh metrics --world turtlebot3_world --reference maps/first_room/map.yaml
```

Columns in `maps/exploration_runs.csv`:

| Column | Meaning |
| --- | --- |
| `result` | `complete`, `timeout` or `stopped_by_user` |
| `duration_s` | Simulation seconds from start to complete |
| `distance_m` | Distance driven, from `/odom` |
| `known_area_m2` | Mapped area (free + walls) |
| `coverage_pct` | Mapped area ÷ mapped area of the reference map |
| `pauses` | How many times exploration was paused |

`--timeout-s 600` stops a run after 600 simulation seconds if it never finishes.

## 6. Change settings

Settings are in `config/explore_params.yaml`. Change one value, restart exploration, and write down what changed.

| Setting | When to change |
| --- | --- |
| `progress_timeout` | Robot stays on one goal too long → lower it (for example 15) |
| `min_frontier_size` | Robot chases tiny gaps or never finishes → raise it (for example 1.0) |
| `return_to_init` | Set `false` if the robot should stay where it finished |

## 7. Problems

| Problem | Fix |
| --- | --- |
| Exploration stops at once with `All frontiers traversed/tried out` | Not enough map around the robot yet. Run `teleop`, turn the robot in place two times, close teleop, then run `explore-resume`. |
| Exploration ends after a long time with `All frontiers traversed/tried out`, but the map is not complete | The robot could not reach the goals, so they were skipped. Look for `Planner loop missed its desired rate` (computer too slow: close other programs). Lower `progress_timeout`, raise `min_frontier_size`, or help with teleop and run `explore-resume`. |
| Robot never moves | Nav2 is not ready. Look for errors in terminal C. `ros2 action list` (with `ROS_DOMAIN_ID=30 ROS_LOCALHOST_ONLY=1`) must show `/navigate_to_pose`. |
| `Couldn't find transform` | Gazebo is not running or is paused. Check that `/clock` is moving. |
| Map stays grey | Same as above, or Cartographer (`slam`) is also running. Run only one SLAM. |
| Exploration never finishes | The room is not closed, or there are tiny gaps. Raise `min_frontier_size` or use `--timeout-s`. |
| Nav2 is slow or loses messages | Try Cyclone DDS: `sudo apt install ros-humble-rmw-cyclonedds-cpp`, then `export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp` in **every** terminal before the commands. |
| `MISSING explore_lite` | Run `setup-explore`, then open a new terminal. |
| `setup-explore` says ROS is too old, or the build fails with `tf2_ros/buffer.hpp: No such file` | Update ROS first: `sudo apt update && sudo apt upgrade`, then run `setup-explore` again. |

## What the simulation does not test

The simulated robot uses normal (differential) wheels. The real robot has mecanum wheels and can move sideways. Nav2 settings for sideways motion and the real robot's odometry must be checked on the hardware later.

## Files

| File | Purpose |
| --- | --- |
| `launch/explore.launch.py` | Starts slam_toolbox, Nav2, RViz and explore_lite |
| `config/explore_params.yaml` | explore_lite settings |
| `scripts/explore_metrics.py` | Records time, distance, area and coverage for one run |
| `scripts/run_ros_experiment.sh` | New commands: `setup-explore`, `world-house`, `explore`, `explore-stop`, `explore-resume`, `metrics`, `save-explore` |

Sources: [explore_lite (m-explore-ros2)](https://github.com/robo-friends/m-explore-ros2), [Nav2 with SLAM](https://docs.nav2.org/tutorials/docs/navigation2_with_slam.html), [Husarion exploration tutorial](https://husarion.com/tutorials/ros2-tutorials/10-exploration/).
