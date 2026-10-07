# Physical robot connection checklist

The modified four-wheel mecanum robot has not yet been validated. Use its existing working software and the supervisor's startup procedure. The standard TurtleBot3 drivetrain configuration may not apply.

## Identify before changing anything

- [ ] Record onboard computer/OS, ROS version/distribution, workspace, actual startup command, and existing services.
- [ ] Identify motor controller, base driver, encoders, IMU, LiDAR, RGB camera, front sensor, and LED interface.
- [ ] Record command topic/message type, odometry topic, and base/sensor frames.
- [ ] Confirm whether `linear.y` sideways control and mecanum odometry already work.
- [ ] Obtain network details, robot address, SSH username, and authentication method. Keep credentials and private network details out of public commits.
- [ ] Confirm physical stop/motor disable and the driver's command-loss timeout.
- [ ] Stop simulation processes before using the same ROS topics on hardware.

## Connect and inspect

Connect the laptop to the approved robot Wi-Fi/Ethernet network. In a Linux terminal:

```bash
ROBOT_IP='REPLACE_WITH_ROBOT_IP'
ROBOT_USER='REPLACE_WITH_SSH_USERNAME'
ping -c 3 "$ROBOT_IP"
ssh "$ROBOT_USER@$ROBOT_IP"
```

Confirm the SSH host-key fingerprint with the supervisor. A blocked ping does not necessarily mean SSH is unavailable. Inside SSH, commands execute on the robot:

```bash
hostname
cat /etc/os-release
ls /opt/ros
printenv ROS_DISTRO ROS_DOMAIN_ID ROS_LOCALHOST_ONLY RMW_IMPLEMENTATION
```

Source the robot's actual ROS and workspace setup files, then inspect `ros2 node list` and `ros2 topic list -t` if ROS 2 is confirmed. Do not reinstall the OS or drivers to match the simulation. SSH and domain configuration are described in [ROBOTIS SBC setup](https://docs.robotis.com/docs/systems/turtlebot3/quick_start_guide/sbc_setup/).

- [ ] Start the supervisor-confirmed bringup only if it is not already running.
- [ ] Confirm an active LiDAR publisher and plausible ranges.
- [ ] Confirm odometry and continuous TF to the base and sensor frames.
- [ ] Identify camera image/camera-info topics and LED control examples.

For standard topic names, useful read-only checks are:

```bash
ros2 topic info /scan --verbose
ros2 topic echo /scan --once --qos-reliability best_effort
ros2 topic echo /odom --once --field pose
ros2 run tf2_ros tf2_echo odom base_footprint
```

Replace topics and frames when the robot uses different names. Standard `turtlebot3_bringup` is not verified for this modified base.

## Laptop ROS connectivity

In a local Linux terminal, source the matching ROS environment and set the supervisor-assigned domain:

```bash
source /opt/ros/humble/setup.bash  # Only if Humble is the confirmed distribution.
export ROS_DOMAIN_ID=REPLACE_WITH_CONFIRMED_NUMBER
export ROS_LOCALHOST_ONLY=0
# Apply the agreed middleware/DDS configuration if required.
ros2 daemon stop
ros2 topic list -t
ros2 topic info /scan --verbose
```

The robot processes need matching settings at startup too. A topic name alone does not prove a remote publisher or working sensor delivery. Receive a real message on the laptop before proceeding.

WSL2's default NAT networking may complicate DDS discovery even when SSH works. If local ROS discovery fails, continue read-only checks through SSH and use a working native Linux PC or onboard display. Mirrored networking is an option only on supported Windows 11 versions; see [Microsoft's WSL networking guide](https://learn.microsoft.com/en-us/windows/wsl/networking).

**Do not use the simulation launcher on hardware.** It forces localhost-only discovery, domain 30, and simulated mapping time.

## Supervised first movement

- [ ] Clear a floor area and assign a person to the physical stop.
- [ ] Use the supervisor's known working controller first, at an agreed low speed.
- [ ] Test forward/backward and stopping, then small left/right rotations.
- [ ] Check odometry direction, units, and pose changes.
- [ ] Test sideways motion only after confirming the mecanum driver interface.
- [ ] Verify command-loss stopping using the supervisor's procedure.

The provided game controller uses `/cmd_vel` with `geometry_msgs/msg/Twist`, `linear.x`, and `angular.z`, at fixed speeds of 0.15 m/s and 0.7 rad/s. It has no sideways command and is simulation-tested only. Zero-on-release/focus-loss cannot replace a base-driver timeout if the process crashes or connectivity is lost.

If the supervisor accepts the interface, speeds, and stop behavior, close other controllers and run `python3 scripts/game_teleop.py` directly from the repository root in a prepared real-robot environment. Do not use the simulation launcher.

## Optional mapping and handover

- [ ] Use the confirmed mapping configuration for the actual sensors, frames, and odometry, with `use_sim_time=false`.
- [ ] Visualize real `/map` and `/scan`; explore slowly, stop, and save with `nav2_map_server map_saver_cli` on the machine receiving the map.
- [ ] Keep YAML and image together; transfer onboard outputs with `scp` if needed.
- [ ] Record exact startup, control, environment, and shutdown commands.
- [ ] Stop motion and follow the supervisor's shutdown/power-off order.

The minimum useful first session is a confirmed hardware inventory, working SSH, an established startup procedure, and sensor evidence. Missing base control or odometry is a project dependency to resolve before navigation.
