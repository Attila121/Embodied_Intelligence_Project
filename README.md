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

Keep this repository focused on project source, configuration, guides, marker assets,
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
      assets/ config/ docs/ launch/ scripts/ tests/
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

### Owner following with a camera marker

The robot identifies its owner using **ArUco DICT_4X4_50, marker ID 23**, with a
**15 cm outer black square**. The camera measures the marker's position using its
known size and camera calibration, and TF transforms that measurement into the
robot frame. The controller maintains roughly **1.0–1.1 m** distance. A separate
guard stops commands for obstacles, stale LiDAR, or a missing controller; losing
the owner marker also stops following. This identifies the visible marker holder,
not a face or a person's appearance.

Use **four separate container terminals**, in the order below. Stop any existing
Gazebo world, exploration, teleop, old following launch, and synthetic target
publisher first. The empty world gives the marker demo room to move and turn.

**Terminal 1 — start the empty world and robot:**

```bash
cd ~/turtlebot3_ws/src/Embodied_Intelligence_Project
LIBGL_ALWAYS_SOFTWARE=true GALLIUM_DRIVER=llvmpipe \
  bash scripts/run_ros_experiment.sh world-empty
```

Wait for the robot and leave Gazebo unpaused. The rendering variables retain the
software-rendering workaround used for the D3D12 GUI crash.

**Terminal 2 — start camera tracking, the controller, and the guard:**

```bash
cd ~/turtlebot3_ws/src/Embodied_Intelligence_Project
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=30 ROS_LOCALHOST_ONLY=1
ros2 launch ./launch/follow.launch.py tracker:=true
```

This starts a dry run: the robot stays stationary and guarded commands go to
`/following/safe_cmd_vel`. Tracking waits for the marker created in Terminal 4.

**Terminal 3 — watch the measured owner position:**

```bash
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=30 ROS_LOCALHOST_ONLY=1
ros2 topic echo /owner/relative_position
```

**Terminal 4 — create the visible marker board:**

```bash
cd ~/turtlebot3_ws/src/Embodied_Intelligence_Project
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=30 ROS_LOCALHOST_ONLY=1
python3 scripts/owner_marker_target.py spawn --distance 1.5
```

The command finishes, leaving a marker board 1.5 m ahead of the robot. Terminal 2
should report `tracking owner`; Terminal 3 should show approximately x=1.5, y=0,
z=0.2 metres. These values come from the camera, not a synthetic position publisher.

**Enable movement after confirming detection:** press Ctrl+C in **Terminal 2**,
then restart it with:

```bash
ros2 launch ./launch/follow.launch.py tracker:=true drive:=true
```

The robot should approach the board and stop at its following distance. Select
`owner_marker` in Gazebo and move it slowly with the move tool, keeping the marker
face toward the camera and its centre near 0.2 m high. The robot follows the
measured position when there is clearance. It stops for obstacles; it does not
plan a path around them or search after losing the owner.

The [full owner-tracking guide](docs/OWNER_TRACKING.md) includes the camera viewer,
wrong-ID tests, and troubleshooting. Settings are in
[`config/owner_tracking.yaml`](config/owner_tracking.yaml). The
[printable marker](assets/owner_marker_23.svg) is 200 mm square including its white
border: print at 100% scale and verify that the **black square measures 150 mm**.

## Guides and code

- [Simulation setup and checklist](docs/SIMULATION.md): installation, running, inspection, saving, shutdown, and common failures.
- [Autonomous exploration guide](docs/EXPLORATION.md): install, run, pause, metrics, settings, and common failures.
- [Owner marker tracking](docs/OWNER_TRACKING.md): camera-based owner position, printable marker, and guarded following in Gazebo.
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
