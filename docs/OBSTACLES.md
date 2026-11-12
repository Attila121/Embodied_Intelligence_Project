# Following with obstacles (simulation)

```bash
bash scripts/run_ros_experiment.sh follow-demo --obstacles            # with the Gazebo window
bash scripts/run_ros_experiment.sh follow-demo --obstacles --headless
```

Adds three low obstacles to the room (two orange cylinders and a red box, 0.30 m high) and
turns on obstacle avoidance. The stand is first moved to the far side of the room, so the
robot's first approach has to go around the central cylinder. The scripted walk then leads it
past all three obstacles (about 4–5 simulated minutes, longer on a slow renderer). The default
demo ([DEMO.md](DEMO.md)) is unchanged and has no obstacles.

A ground-truth monitor watches the robot's real position in Gazebo and **aborts the demo** if
the robot centre ever comes within 0.25 m of an obstacle surface (the robot's body radius is
about 0.21 m). At the end it prints the closest approach. This check is for the demo only;
the tracker and avoider never see Gazebo's positions.

## How it works

```text
owner_follower --/following/desired_cmd_vel--> obstacle_avoider --/following/cmd_vel--> following_guard --> /cmd_vel
```

`room_follow.launch.py avoid:=true` inserts [obstacle_avoider.py](../scripts/obstacle_avoider.py)
between the follower and the guard. **The follower and the guard are unchanged**, and the guard
is still the final gate for every command. If the avoider dies or loses its inputs, its output
(or the guard's command timeout) stops the robot.

The decision logic is in [avoidance.py](../scripts/avoidance.py), without ROS. Each cycle it:

1. Takes the follower's command and the owner's position, and a 360° LiDAR scan converted to
   robot coordinates (including the LiDAR's 6.4 cm offset). It drops returns within 0.35 m of the
   owner, because that is the owner's own stand.
2. If the straight line to the owner is clear and nothing is within 0.8 m, it **passes the
   follower's command through unchanged**.
3. Otherwise it looks for the clear heading closest to the owner's direction: one that stays at
   least ~0.7 m from every obstacle surface for the next 1.6 m. It keeps the side it chose
   (left or right) until it is clear of the obstacle, so it does not dither.
4. It then turns toward that heading: a smooth **curve** for small heading errors, a **rotation in
   place** for large ones, and **straight driving** when already aligned. It never exceeds the
   follower's speed, and it never moves when the follower asks for a stop.

### Why 0.7 m of clearance

The guard refuses any command with a turn component (including a curved drive) when something
is within 0.55 m of the LiDAR in *any* direction; it only allows pure straight driving closer
than that. Without a margin, the robot would freeze beside every obstacle it passed. So the
avoider plans paths that keep ~0.7 m from every surface, which means the guard never has to
intervene. Where the robot already is closer than that (for example after being squeezed), it
only allows paths that do not approach further, and uses a slow straight "escape" drive, because
a turn would be refused.

This also means obstacles need room: with ~0.7 m of clearance on each side of the path, obstacles
must be roughly 2.6 m apart centre to centre, or there is no gap for the robot to drive through.
The layout in [room_obstacles.py](../scripts/room_obstacles.py) respects that and is checked by
tests. If you add obstacles, keep them apart and re-run `python tests/avoidance_sim.py`.

## Settings

[config/obstacle_avoidance.yaml](../config/obstacle_avoidance.yaml) holds the main settings;
the full list with defaults is `AvoidParams` in `avoidance.py`. Settings are validated at
startup: the speed limits cannot exceed the guard's, and `clearance` must stay above the
guard's 0.55 m turning limit. Keep `following_distance` equal to the follower's.

Manual use, without the demo script (domain 30, as in [ROOM_FOLLOWING.md](ROOM_FOLLOWING.md)):

```bash
bash scripts/run_ros_experiment.sh world-room obstacles:=true
ros2 launch ./launch/room_follow.launch.py drive:=true avoid:=true
```

## Tests

- `tests/test_avoidance.py`: decision logic (pass-through, which way it steers, never adds
  motion, owner stand ignored, side commitment, the guard's turning limit, scan validation).
- `tests/test_avoidance_sim.py`: closed loop with the **real guard logic** and a ray-cast scan.
  Without avoidance the robot sits frozen at the first obstacle; with it, the whole walk
  completes with at least 0.55 m of clearance, also under ±12 cm position errors. Run
  `python tests/avoidance_sim.py` (add `--baseline` to compare) for a printed summary.
- `tests/test_obstacle_avoider_ros.py`: the node with synthetic topics, including bad or missing
  scans, a stale command and owner loss.
- `tests/test_room_obstacles.py` and `tests/test_room_scene.py`: layout spacing, heights
  (below both markers so they cannot hide them from the cameras) and the world SDF.

## Limits

- **Reactive only.** It looks 1.6 m ahead and has no map, so it can be caught in a pocket
  (for example between two obstacles closer than ~2.6 m) or by a U-shaped obstacle. If it is
  boxed in, it stops and waits rather than pushing; the owner moving away usually frees it.
- It sees only what the 2-D LiDAR plane (~0.17 m high) hits. Anything lower, or overhanging
  above it, is invisible. Moving obstacles and people are treated as static each cycle.
- If an obstacle sits where the robot would stop (within about 1 m of the owner on the robot's
  side), the robot stops short of its usual following distance.
- Geometry is specific to the simulated Waffle Pi and its aligned LiDAR. This has not been
  tried on the physical mecanum robot; follow [ROBOT.md](ROBOT.md) before any hardware use.
