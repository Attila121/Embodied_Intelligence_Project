"""Isolated Gazebo camera test. Does not connect to the user's ROS domain/world."""

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
sys.path.insert(0, str(root / "scripts"))
from owner_marker import marker_sdf

domain = random.SystemRandom().randint(60, 90)
with socket.socket() as listener:
    listener.bind(("127.0.0.1", 0))
    port = listener.getsockname()[1]
run_dir = Path(tempfile.mkdtemp(prefix="owner-tracking-rendered-"))
os.environ.update(ROS_DOMAIN_ID=str(domain), ROS_LOCALHOST_ONLY="1",
                  ROS_LOG_DIR=str(run_dir / "logs"),
                  GAZEBO_MASTER_URI=f"http://127.0.0.1:{port}",
                  LIBGL_ALWAYS_SOFTWARE="true", GALLIUM_DRIVER="llvmpipe")

import rclpy
from gazebo_msgs.srv import SpawnEntity, DeleteEntity
from geometry_msgs.msg import Point, TransformStamped, Twist
from rclpy.node import Node
from sensor_msgs.msg import Image, LaserScan
from std_msgs.msg import Bool
from std_srvs.srv import Empty
from tf2_ros import StaticTransformBroadcaster

world = '''<sdf version="1.6"><world name="default">
<physics type="ode"><max_step_size>0.01</max_step_size>
<real_time_update_rate>100</real_time_update_rate></physics>
<scene><ambient>1 1 1 1</ambient><background>0.5 0.5 0.5 1</background></scene>
<model name="test_camera"><static>true</static><pose>0 0 0.2 0 0 0</pose>
<link name="camera"><sensor name="camera" type="camera"><always_on>true</always_on>
<update_rate>15</update_rate><camera><horizontal_fov>1.085595</horizontal_fov>
<image><width>640</width><height>480</height><format>R8G8B8</format></image>
<clip><near>0.03</near><far>10</far></clip></camera>
<plugin name="camera_driver" filename="libgazebo_ros_camera.so">
<camera_name>camera</camera_name><frame_name>camera_optical</frame_name></plugin>
</sensor></link></model></world></sdf>'''
(run_dir / "camera.world").write_text(world)
print(f"Isolated test domain={domain}, Gazebo port={port}, logs={run_dir}", flush=True)
rclpy.init()
node = Node("rendered_owner_test")
tf = StaticTransformBroadcaster(node)
transform = TransformStamped()
transform.header.frame_id = "base_footprint"
transform.child_frame_id = "camera_optical"
transform.transform.translation.z = 0.2
q = transform.transform.rotation
q.x, q.y, q.z, q.w = -0.5, 0.5, -0.5, 0.5
tf.sendTransform(transform)
positions, visibility, commands, images = [], [], [], []
subscriptions = [
    node.create_subscription(Point, "/owner/relative_position",
        lambda msg: positions.append((time.monotonic(), msg.x, msg.y, msg.z)), 1),
    node.create_subscription(Bool, "/owner/visible",
        lambda msg: visibility.append((time.monotonic(), msg.data)), 1),
    node.create_subscription(Twist, "/following/safe_cmd_vel",
        lambda msg: commands.append((time.monotonic(), msg.linear.x, msg.angular.z)), 1),
    node.create_subscription(Image, "/owner/debug_image", images.append, 1),
]
scan_pub = node.create_publisher(LaserScan, "/scan", 1)
scan = LaserScan()
scan.header.frame_id = "base_scan"
scan.angle_increment = math.tau / 360
scan.range_min, scan.range_max = 0.12, 3.5
scan.ranges = [3.] * 360
spawn = node.create_client(SpawnEntity, "/spawn_entity")
delete = node.create_client(DeleteEntity, "/delete_entity")
processes, logs = [], []


def start(command, name):
    log = open(run_dir / f"{name}.log", "w")
    logs.append(log)
    process = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT,
                               start_new_session=True)
    processes.append(process)
    return process


def pump(duration):
    deadline = time.monotonic() + duration
    while time.monotonic() < deadline:
        scan_pub.publish(scan)
        rclpy.spin_once(node, timeout_sec=0.02)
        time.sleep(0.01)
        assert all(p.poll() is None for p in processes), "Test process exited early"


def request(client, message):
    if not client.wait_for_service(timeout_sec=20):
        raise RuntimeError("Gazebo service unavailable")
    future = client.call_async(message)
    rclpy.spin_until_future_complete(node, future, timeout_sec=10)
    response = future.result()
    assert response is not None and response.success, response


def add(name="owner_marker", marker_id=23, x=1.5, y=0.):
    message = SpawnEntity.Request(name=name, xml=marker_sdf(marker_id, model_name=name))
    message.initial_pose.position.x, message.initial_pose.position.y = x, y
    message.initial_pose.position.z = 0.2
    message.initial_pose.orientation.w = 1.
    request(spawn, message)


def remove(name="owner_marker"):
    request(delete, DeleteEntity.Request(name=name))
    pump(0.4)  # Let Gazebo's rendering thread retire the old visual names.


def stopped():
    recent = [v for v in commands if time.monotonic() - v[0] < 0.2]
    assert recent and all(v[1:] == (0., 0.) for v in recent), recent


try:
    start(["gzserver", "--verbose", str(run_dir / "camera.world"),
           "-s", "libgazebo_ros_init.so", "-s", "libgazebo_ros_factory.so"], "gazebo")
    start(["ros2", "launch", str(root / "launch" / "follow.launch.py"),
           "tracker:=true"], "following")
    add(x=1.5)
    pump(6)
    if images:
        from cv_bridge import CvBridge
        import cv2
        cv2.imwrite(str(run_dir / "camera_debug.png"), CvBridge().imgmsg_to_cv2(images[-1], "bgr8"))
    recent = [v for v in positions if time.monotonic() - v[0] < 0.5]
    assert recent, "No marker position from rendered camera"
    print("Measured rendered marker position:", recent[-1][1:], flush=True)
    assert abs(recent[-1][1] - 1.5) < 0.08, recent[-1]
    assert abs(recent[-1][2]) < 0.03, recent[-1]
    assert any(v[1] > 0 for v in commands[-10:]), commands[-10:]
    assert node.count_publishers("/cmd_vel") == 0
    print("PASS: actual rendered marker -> calibrated position -> guarded dry-run motion", flush=True)

    remove()
    add(name="owner_close", x=0.8)
    pump(3)
    recent = [v for v in positions if time.monotonic() - v[0] < 0.5]
    if images:
        cv2.imwrite(str(run_dir / "camera_close.png"), CvBridge().imgmsg_to_cv2(images[-1], "bgr8"))
    print("Close target observations:", recent[-3:], flush=True)
    assert recent and abs(recent[-1][1] - 0.8) < 0.06
    stopped()
    print("PASS: closer marker changes measured distance and stops following", flush=True)

    remove("owner_close")
    add(name="wrong_owner", marker_id=24, x=1.5)
    pump(1.5)
    assert visibility and not visibility[-1][1]
    assert not [v for v in positions if time.monotonic() - v[0] < 0.8]
    stopped()
    print("PASS: wrong marker ID produces no owner position and zero motion", flush=True)

    remove("wrong_owner")
    add(x=1.5, y=-0.25)
    add(name="duplicate", x=1.5, y=0.25)
    pump(1.5)
    assert not visibility[-1][1]
    stopped()
    print("PASS: duplicate owner markers are rejected", flush=True)

    remove("duplicate")
    remove()
    pump(1)
    assert not visibility[-1][1]
    stopped()
    print("PASS: marker loss stops output", flush=True)

    add(name="pause_test", x=1.5)
    pump(2)
    assert visibility[-1][1]
    pause = node.create_client(Empty, "/pause_physics")
    assert pause.wait_for_service(timeout_sec=3)
    future = pause.call_async(Empty.Request())
    rclpy.spin_until_future_complete(node, future, timeout_sec=3)
    assert future.done()
    pump(1)
    assert not visibility[-1][1]
    stopped()
    print("PASS: paused simulation/camera triggers wall-clock stop", flush=True)
finally:
    for process in reversed(processes):
        if process.poll() is None:
            os.killpg(process.pid, signal.SIGINT)
            try:
                process.wait(timeout=8)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait(timeout=3)
    node.destroy_node()
    rclpy.shutdown()
    for log in logs:
        log.close()
