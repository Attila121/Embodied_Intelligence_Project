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
| Saved map and evaluation | Map saving provided; completed exploration and exported results pending |
| Physical mecanum robot | Hardware, drivers, odometry, and interfaces awaiting inspection |
| Navigation | Planned |
| Owner/stranger recognition and LEDs | Planned |

The standard simulation uses differential drive. It does not validate the real robot's sideways motion or mecanum odometry.

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

## Guides and code

- [Simulation setup and checklist](docs/SIMULATION.md): installation, running, inspection, saving, shutdown, and common failures.
- [Physical robot checklist](docs/ROBOT.md): hardware inventory, SSH, ROS connectivity, supervised movement, and optional mapping.
- [Hold-to-drive controller](scripts/game_teleop.py): keyboard press/release handling and stop commands.
- [Simulation launcher](scripts/run_ros_experiment.sh): shared environment and commands.

The simulation launcher uses ROS domain 30, localhost-only discovery, and simulation time. Use the physical robot guide for hardware; these simulation settings are not a robot connection procedure.

## Project direction

1. Verify physical base control, LiDAR, and odometry.
2. Establish reliable mapping and navigation.
3. Add manageable owner/stranger recognition.
4. Integrate a small state machine and LED expression.

Advanced exploration, person following, and multi-person tracking are stretch goals. The first milestone is a working integrated demonstration.
