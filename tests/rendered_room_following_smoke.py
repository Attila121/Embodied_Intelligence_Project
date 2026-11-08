"""Isolated two-camera room test, including real simulated robot movement."""

import math
import os
from pathlib import Path
import random
import signal
import socket
import subprocess
import sys
import tempfile
import time

root = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(root/"scripts"))
from room_scene import build_room
from ament_index_python.packages import get_package_share_directory

domain = random.SystemRandom().randint(60, 90)
with socket.socket() as listener:
    listener.bind(("127.0.0.1", 0))
    port = listener.getsockname()[1]
directory = Path(tempfile.mkdtemp(prefix="two-camera-room-test-"))
models = Path(get_package_share_directory("turtlebot3_gazebo"))/"models"
os.environ.update(ROS_DOMAIN_ID=str(domain), ROS_LOCALHOST_ONLY="1",
                  GAZEBO_MASTER_URI=f"http://127.0.0.1:{port}",
                  GAZEBO_MODEL_PATH=str(models)+os.pathsep+os.environ.get("GAZEBO_MODEL_PATH", ""),
                  ROS_LOG_DIR=str(directory/"logs"),
                  LIBGL_ALWAYS_SOFTWARE="true", GALLIUM_DRIVER="llvmpipe")
world = directory/"room.world"
world.write_text(build_room(models/"turtlebot3_waffle_pi/model.sdf"))
print(f"Isolated room test: domain={domain}, port={port}, results={directory}", flush=True)

import cv2
from cv_bridge import CvBridge
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Point, Twist
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
from std_srvs.srv import Empty
from gazebo_msgs.srv import DeleteEntity, GetEntityState, SetEntityState

rclpy.init()
node = Node("room_following_test")
positions, visible, commands, raw, debug = [], [], [], {}, {}
subscriptions = [
    node.create_subscription(Point, "/owner/relative_position", lambda msg:
        positions.append((time.monotonic(), msg.x, msg.y, msg.z)), 1),
    node.create_subscription(Bool, "/owner/visible", lambda msg:
        visible.append((time.monotonic(), msg.data)), 1),
    node.create_subscription(Twist, "/following/safe_cmd_vel", lambda msg:
        commands.append((time.monotonic(), msg.linear.x, msg.angular.z)), 1),
    node.create_subscription(Twist, "/cmd_vel", lambda msg:
        commands.append((time.monotonic(), msg.linear.x, msg.angular.z)), 1),
]
for camera in ("left", "right"):
    subscriptions.extend([
        node.create_subscription(Image, f"/room_cameras/{camera}/image_raw",
            lambda msg, name=camera: raw.update({name: msg}), qos_profile_sensor_data),
        node.create_subscription(Image, f"/room_cameras/{camera}/debug_image",
            lambda msg, name=camera: debug.update({name: msg}), 1),
    ])
processes, files = [], []


def start(command, name):
    handle = open(directory/f"{name}.log", "w")
    files.append(handle)
    process = subprocess.Popen(command, stdout=handle, stderr=subprocess.STDOUT,
                               start_new_session=True)
    processes.append(process)
    return process


def stop(process):
    if process.poll() is None:
        os.killpg(process.pid, signal.SIGINT)
        try:
            process.wait(timeout=8)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGKILL)
            process.wait(timeout=3)
    processes.remove(process)


def pump(seconds):
    deadline = time.monotonic()+seconds
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.03)
        if any(p.poll() is not None for p in processes):
            raise RuntimeError("A room test process exited; inspect the log directory")


def request(kind, topic, message):
    client = node.create_client(kind, topic)
    try:
        assert client.wait_for_service(timeout_sec=15), topic
        future = client.call_async(message)
        rclpy.spin_until_future_complete(node, future, timeout_sec=5)
        response = future.result()
        assert response is not None
        if hasattr(response, "success"):
            assert response.success, response
        return response
    finally:
        node.destroy_client(client)


def get(name):
    state = request(GetEntityState, "/gazebo/get_entity_state",
                    GetEntityState.Request(name=name, reference_frame="world")).state
    state.name = name
    return state


def put(state):
    state.reference_frame = "world"
    request(SetEntityState, "/gazebo/set_entity_state", SetEntityState.Request(state=state))


def recent_position():
    recent = [v for v in positions if time.monotonic()-v[0] < 1.]
    assert recent, "No recent image-measured owner position"
    return recent[-1]


def assert_zero():
    recent = [v for v in commands if time.monotonic()-v[0] < 0.3]
    assert recent and all(v[1:] == (0., 0.) for v in recent), recent


try:
    start(["ros2", "launch", str(root/"launch/room_world.launch.py"), "gui:=false"], "gazebo")
    follower = start(["ros2", "launch", str(root/"launch/room_follow.launch.py")], "dry_follow")
    get("waffle_pi")
    pump(10)
    assert set(raw) == {"left", "right"}, f"Missing fixed camera topics: {raw.keys()}"
    point = recent_position()
    print("Measured initial relative position:", point[1:], flush=True)
    assert abs(point[1]-2.) < 0.25 and abs(point[2]) < 0.25, point
    assert "left+right" in (directory/"dry_follow.log").read_text()
    assert node.count_publishers("/cmd_vel") == 0
    assert node.count_publishers("/camera/image_raw") == 0
    print("PASS: both fixed cameras track owner and robot; no onboard camera or driving", flush=True)

    left = get("room_camera_left")
    saved_x, saved_y = left.pose.position.x, left.pose.position.y
    left.pose.position.x, left.pose.position.y = 50., 50.
    put(left)
    pump(2)
    point = recent_position()
    assert abs(point[1]-2.) < 0.25
    assert visible[-1][1]
    print("PASS: right camera continues when left view is lost", flush=True)
    left.pose.position.x, left.pose.position.y = saved_x, saved_y
    put(left)
    pump(2)

    robot = get("waffle_pi")
    robot.pose.orientation.z = math.sin(math.pi/4)
    robot.pose.orientation.w = math.cos(math.pi/4)
    robot.pose.orientation.x = robot.pose.orientation.y = 0.
    put(robot)
    pump(3)
    point = recent_position()
    print("Measured after robot turns +90 degrees:", point[1:], flush=True)
    assert abs(point[1]) < 0.3 and abs(point[2]+2.) < 0.3, point
    print("PASS: robot heading comes from its observed marker", flush=True)
    robot.pose.orientation.z, robot.pose.orientation.w = 0., 1.
    put(robot)
    pump(2)

    # Exercise the actual user-facing movement command, not a target-position
    # publisher. Only Gazebo moves the stand; perception must measure the change.
    with open(directory/"target_move.log", "w") as handle:
        target_command = subprocess.Popen(
            [sys.executable, str(root/"scripts/room_target.py"), "move",
             "--x", "1.5", "--y", "0.3", "--duration", "3"],
            stdout=handle, stderr=subprocess.STDOUT)
        deadline = time.monotonic()+15
        while target_command.poll() is None and time.monotonic() < deadline:
            pump(0.2)
        if target_command.poll() is None:
            target_command.kill()
            target_command.wait(timeout=3)
            raise AssertionError("Visible target movement command timed out")
        assert target_command.returncode == 0, (directory/"target_move.log").read_text()
    pump(1)
    point = recent_position()
    assert abs(point[1]-2.5) < 0.3 and abs(point[2]-0.3) < 0.25, point
    print("PASS: moving the visible stand changes the camera-measured target", flush=True)
    target_state = get("room_owner")
    target_state.pose.position.x, target_state.pose.position.y = 1., 0.
    put(target_state)
    pump(1)

    stop(follower)
    start(["ros2", "launch", str(root/"launch/room_follow.launch.py"), "drive:=true"], "drive_follow")
    origin = get("waffle_pi").pose.position.x
    deadline = time.monotonic()+35
    current = origin
    while time.monotonic() < deadline:
        pump(0.5)
        current = get("waffle_pi").pose.position.x
        if current-origin > 0.25:
            break
    assert current-origin > 0.25, f"Robot did not follow: travelled {current-origin:.3f} m"
    point = recent_position()
    assert point[1] < 1.85, point
    print(f"PASS: real simulated robot followed camera measurements for {current-origin:.2f} m", flush=True)

    request(DeleteEntity, "/delete_entity", DeleteEntity.Request(name="room_owner"))
    pump(1.5)
    assert visible and not visible[-1][1]
    assert_zero()
    print("PASS: losing owner in both views stops movement", flush=True)

    request(Empty, "/pause_physics", Empty.Request())
    pump(1)
    assert_zero()
    print("PASS: paused cameras/simulation remain stopped", flush=True)
finally:
    bridge = CvBridge()
    for name, image in {**raw, **{f"{name}_debug": msg for name, msg in debug.items()}}.items():
        cv2.imwrite(str(directory/f"{name}.png"), bridge.imgmsg_to_cv2(image, "bgr8"))
    for process in list(reversed(processes)):
        stop(process)
    node.destroy_node()
    rclpy.shutdown()
    for handle in files:
        handle.close()
