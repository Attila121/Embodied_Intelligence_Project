# Embodied Intelligence Project

A semester project integrating indoor mapping, navigation, person recognition, and LED state expression on a modified TurtleBot3 Waffle Pi with mecanum wheels.

The intended behavior is to scan an unfamiliar room with LiDAR, navigate within it, distinguish an owner from an unknown person, and communicate a simple behavioral state through LEDs. We reuse established ROS packages and focus on integrating the components into a working system.

## Current status

| Component | Status |
| --- | --- |
| TurtleBot3 simulation | Working with ROS 2 Humble and Gazebo Classic |
| LiDAR and odometry | Live data verified in simulation |
| Mapping | Cartographer publishes a map; RViz visualization verified |
| Keyboard control | Hold-to-drive controller implemented and exercised in simulation |
| Saved map and evaluation | Map export and exploration metrics provided; local results retained under `maps/` |
| Physical mecanum robot | Hardware, drivers, odometry, and interfaces awaiting inspection |
| Autonomous exploration | Simulation milestone complete: slam_toolbox + Nav2 + explore_lite, map export, and metrics |
| Owner following | Simulation milestone complete: camera-based ArUco owner tracking, distance keeping, obstacle guard, and target-loss stopping |
| Navigation | Nav2 used by exploration in simulation; physical robot pending |
| Appearance-based owner recognition and LEDs | Planned; current owner identification uses a unique ArUco marker |

The standard simulation uses differential drive. It does not validate the real robot's sideways motion or mecanum odometry. Completed exploration and following milestones apply to simulation; physical deployment and broader evaluation remain separate work.

## Directory layout

Keep this repository focused on project source, configuration, guides,
and tests. Third-party dependencies and active build outputs belong in the outer
ROS workspace:

```text
~/turtlebot3_ws/
  build/                         # Used by the symlink installation
  install/                       # Active ROS package environment
  src/
    turtlebot3_simulations/       # Official Humble simulation source
    m-explore-ros2/               # Exploration dependency (pinned by setup-explore)
    Embodied_Intelligence_Project/
      config/ docs/ launch/ scripts/ tests/
      maps/                      # Local results, ignored by Git
```

Do not build the third-party packages inside this repository or clone another
copy of them into it. Follow [simulation setup](docs/SIMULATION.md) and
[exploration setup](docs/EXPLORATION.md) from the outer workspace. The outer
`build/` and dependency source directories are required while using
`colcon build --symlink-install`; deleting them breaks the active installation.

## Quick start

Prerequisites: Ubuntu 22.04, ROS 2 Humble Desktop, TurtleBot3 packages, and the built Humble simulation workspace. See [Simulation setup and checklist](docs/SIMULATION.md) for installation and troubleshooting.

```bash
git clone https://github.com/Attila121/Embodied_Intelligence_Project.git
cd Embodied_Intelligence_Project
bash scripts/run_ros_experiment.sh check
```

Open three terminals in this repository and run one command in each:

```bash
# Terminal A: Gazebo world
bash scripts/run_ros_experiment.sh world
```

```bash
# Terminal B: Cartographer and RViz
bash scripts/run_ros_experiment.sh slam
```

```bash
# Terminal C: keyboard-control window
bash scripts/run_ros_experiment.sh teleop
```

Click the control window to give it focus:

| Key | Action while held |
| --- | --- |
| W / S | Forward / backward |
| A / D | Turn left / right |
| W + A or D | Move while steering |
| Space / Esc | Stop until movement keys are released |

Releasing the keys stops motion. Losing window focus or closing the controller also sends a stop command. Current speeds are 0.15 m/s and 0.7 rad/s. The controller publishes `geometry_msgs/msg/Twist` on `/cmd_vel` at approximately 20 Hz; it does not command sideways movement.

After exploration, stop the robot and save from a fourth terminal while Gazebo and Cartographer remain running:

```bash
bash scripts/run_ros_experiment.sh save
```

Outputs go to `maps/first_room/`. The launcher refuses to overwrite an existing map. Generated maps, screenshots, and local observations are ignored by Git by default.

### Autonomous exploration

The robot can also explore and map the room by itself. Install once with `bash scripts/run_ros_experiment.sh setup-explore` (see the [exploration guide](docs/EXPLORATION.md)), then:

```bash
# Terminal A: Gazebo world
bash scripts/run_ros_experiment.sh world
```

```bash
# Terminal B: metrics recorder (start before exploration)
bash scripts/run_ros_experiment.sh metrics --world turtlebot3_world
```

```bash
# Terminal C: slam_toolbox, Nav2, RViz and explore_lite
bash scripts/run_ros_experiment.sh explore
```

Save the map with `bash scripts/run_ros_experiment.sh save-explore`. Do not run Cartographer or the keyboard controller during exploration.

### Owner following with two fixed room cameras

The external-camera demo places a robot and movable owner stand in a room, with
fixed cameras at the back-left and back-right. Both cameras observe owner marker
**23** and robot marker **24**, and measure the owner's position relative to the
robot, including its heading. The robot's onboard camera is not used in this room.

Stop previous Gazebo, exploration, teleop, and following processes first. Use four container terminals:

```bash
# Terminal 1: room, both cameras, tagged robot and owner stand
cd ~/turtlebot3_ws/src/Embodied_Intelligence_Project
LIBGL_ALWAYS_SOFTWARE=true GALLIUM_DRIVER=llvmpipe \
  bash scripts/run_ros_experiment.sh world-room
```

```bash
# Terminal 2: external-camera tracking, controller and guard (dry run)
cd ~/turtlebot3_ws/src/Embodied_Intelligence_Project
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=30 ROS_LOCALHOST_ONLY=1
ros2 launch ./launch/room_follow.launch.py
```

```bash
# Terminal 3: measured relative owner position (initially x near 2, y near 0)
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=30 ROS_LOCALHOST_ONLY=1
ros2 topic echo /owner/relative_position
```

Confirm Terminal 2 reports tracking, then Ctrl+C there and restart it to enable
simulation movement:

```bash
ros2 launch ./launch/room_follow.launch.py drive:=true
```

```bash
# Terminal 4: move the visible owner stand through the room
cd ~/turtlebot3_ws/src/Embodied_Intelligence_Project
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=30 ROS_LOCALHOST_ONLY=1
python3 scripts/room_target.py move --x 1.5 --y 0.5 --duration 5
```

The cameras measure the moving stand; its position is not supplied directly to
the follower. The robot should follow and maintain roughly 1.0–1.1 m distance
when there is clearance. In Terminal 4, run `python3 scripts/room_target.py remove`
to check that loss of the owner stops the robot. Use
`python3 scripts/room_target.py spawn --x 1.5 --y 0.5` to recreate it.

The [full room-camera guide](docs/ROOM_FOLLOWING.md) covers both camera viewers,
marker sizes, settings, camera handover and tests.

## Guides and code

- [Simulation setup and checklist](docs/SIMULATION.md): installation, running, inspection, saving, shutdown, and common failures.
- [Autonomous exploration guide](docs/EXPLORATION.md): install, run, pause, metrics, settings, and common failures.
- [Two fixed room cameras](docs/ROOM_FOLLOWING.md): external-camera owner/robot tracking, room demo, and target movement.
- [Physical robot checklist](docs/ROBOT.md): hardware inventory, SSH, ROS connectivity, supervised movement, and optional mapping.
- [Hold-to-drive controller](scripts/game_teleop.py): keyboard press/release handling and stop commands.
- [Simulation launcher](scripts/run_ros_experiment.sh): shared environment and commands.
- [Exploration launch](launch/explore.launch.py), [settings](config/explore_params.yaml), and [metrics recorder](scripts/explore_metrics.py).

The simulation launcher uses ROS domain 30, localhost-only discovery, and simulation time. Use the physical robot guide for hardware; these simulation settings are not a robot connection procedure.

## Project direction

1. Verify physical base control, LiDAR, and odometry.
2. Transfer the completed exploration and marker-based following behaviors to the physical robot after verifying its interfaces.
3. Extend owner identification beyond a visible marker if required.
4. Integrate a small state machine and LED expression.

Autonomous exploration and marker-based owner following are complete as simulation milestones. Physical robot integration, LED state expression, and optional multi-person tracking remain future work.
