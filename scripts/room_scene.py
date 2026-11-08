#!/usr/bin/env python3
"""Generate a room with two fixed cameras, a tagged Waffle Pi and owner target."""

import math
from pathlib import Path
import xml.etree.ElementTree as ET

from owner_marker import marker_cells

OWNER_ID, ROBOT_ID = 23, 24
OWNER_SIZE, ROBOT_SIZE = 0.30, 0.24
ROBOT_MARKER_POSE = (-0.064, 0.0, 0.32)
OWNER_MARKER_HEIGHT = 0.75
CAMERAS = {"left": (-2.8, 2.7, 3.4), "right": (-2.8, -2.7, 3.4)}


def element(parent, tag, text=None, **attrs):
    child = ET.SubElement(parent, tag, attrs)
    if text is not None:
        child.text = str(text)
    return child


def box(link, name, size, pose, color, collision=False):
    visual = element(link, "visual", name=name)
    element(visual, "pose", " ".join(map(str, pose)))
    geometry = element(visual, "geometry")
    element(element(geometry, "box"), "size", " ".join(map(str, size)))
    material = element(visual, "material")
    script = element(material, "script")
    element(script, "uri", "file://media/materials/scripts/gazebo.material")
    element(script, "name", f"Gazebo/{color}")
    element(visual, "cast_shadows", "false")
    if collision:
        shape = element(link, "collision", name=name+"_collision")
        element(shape, "pose", " ".join(map(str, pose)))
        geometry = element(shape, "geometry")
        element(element(geometry, "box"), "size", " ".join(map(str, size)))


def top_marker(link, marker_id, size, origin):
    """Marker +x points robot-forward; marker +y points robot-left; +z up."""
    cell = size/6
    for row, values in enumerate(marker_cells(marker_id)):
        for col, value in enumerate(values):
            x = origin[0]+(col-3.5)*cell
            y = origin[1]+(3.5-row)*cell
            # The calibrated marker plane is the top surface of these boxes.
            box(link, f"tag_{row}_{col}", (cell, cell, 0.003),
                (x, y, origin[2]-0.0015, 0, 0, 0), "White" if value else "Black")


def owner_model(name="room_owner", marker_id=OWNER_ID):
    model = ET.Element("model", name=name)
    element(model, "static", "true")
    link = element(model, "link", name="owner")
    box(link, "stand", (0.16, 0.16, OWNER_MARKER_HEIGHT-0.02),
        (0, 0, (OWNER_MARKER_HEIGHT-0.02)/2, 0, 0, 0), "Blue", collision=True)
    box(link, "board", (OWNER_SIZE*8/6, OWNER_SIZE*8/6, 0.01),
        (0, 0, OWNER_MARKER_HEIGHT-0.01, 0, 0, 0), "White", collision=True)
    top_marker(link, marker_id, OWNER_SIZE, (0, 0, OWNER_MARKER_HEIGHT))
    return model


def owner_sdf(name="room_owner", marker_id=OWNER_ID):
    root = ET.Element("sdf", version="1.6")
    root.append(owner_model(name, marker_id))
    return ET.tostring(root, encoding="unicode")


def add_camera(world, name, position, width=1280, height=960):
    x, y, z = position
    dx, dy, dz = -x, -y, 0.2-z
    yaw = math.atan2(dy, dx)
    pitch = math.atan2(-dz, math.hypot(dx, dy))
    model = element(world, "model", name=f"room_camera_{name}")
    element(model, "static", "true")
    element(model, "pose", f"{x} {y} {z} 0 {pitch} {yaw}")
    link = element(model, "link", name="camera")
    box(link, "housing", (0.16, 0.12, 0.1), (-0.1, 0, 0, 0, 0, 0), "Blue")
    sensor = element(link, "sensor", name=name, type="camera")
    element(sensor, "always_on", "true")
    element(sensor, "update_rate", 10)
    camera = element(sensor, "camera")
    element(camera, "horizontal_fov", 1.6)
    image = element(camera, "image")
    element(image, "width", width)
    element(image, "height", height)
    element(image, "format", "R8G8B8")
    clip = element(camera, "clip")
    element(clip, "near", 0.05)
    element(clip, "far", 15)
    plugin = element(sensor, "plugin", name=f"room_camera_{name}_driver",
                     filename="libgazebo_ros_camera.so")
    element(element(plugin, "ros"), "namespace", "/room_cameras")
    element(plugin, "camera_name", name)
    element(plugin, "frame_name", f"room_camera_{name}_optical")


def build_room(model_path, include_owner=True):
    root = ET.Element("sdf", version="1.6")
    world = element(root, "world", name="room_following")
    physics = element(world, "physics", type="ode")
    element(physics, "max_step_size", 0.001)
    element(physics, "real_time_update_rate", 1000)
    scene = element(world, "scene")
    element(scene, "ambient", "0.8 0.8 0.8 1")
    element(scene, "background", "0.65 0.7 0.8 1")
    light = element(world, "light", name="sun", type="directional")
    element(light, "direction", "-0.3 0.1 -1")
    element(light, "diffuse", "0.8 0.8 0.8 1")
    room = element(world, "model", name="room")
    element(room, "static", "true")
    walls = element(room, "link", name="walls")
    box(walls, "floor", (6.2, 6.2, 0.1), (0, 0, -0.05, 0, 0, 0), "Grey", True)
    for name, size, pose in (
        ("back", (0.1, 6.2, 1.0), (-3.05, 0, 0.5, 0, 0, 0)),
        ("front", (0.1, 6.2, 1.0), (3.05, 0, 0.5, 0, 0, 0)),
        ("left", (6.2, 0.1, 1.0), (0, 3.05, 0.5, 0, 0, 0)),
        ("right", (6.2, 0.1, 1.0), (0, -3.05, 0.5, 0, 0, 0)),
    ):
        box(walls, name, size, pose, "Grey", True)
    for name, position in CAMERAS.items():
        add_camera(world, name, position)
    robot = ET.parse(Path(model_path)).getroot().find("model")
    robot.set("name", "waffle_pi")
    robot.find("pose").text = "-1 0 0.01 0 0 0"
    # This demo consumes only the fixed cameras. Keep the original robot model
    # on disk unchanged, but omit its onboard camera sensor from this world.
    for link in robot.findall("link"):
        for sensor in list(link.findall("sensor")):
            if sensor.attrib.get("type") == "camera":
                link.remove(sensor)
    tag = element(robot, "link", name="room_tracking_marker")
    inertia = element(tag, "inertial")
    element(inertia, "mass", 0.01)
    tensor = element(inertia, "inertia")
    for name in ("ixx", "iyy", "izz"):
        element(tensor, name, 0.00001)
    for name in ("ixy", "ixz", "iyz"):
        element(tensor, name, 0.)
    top_marker(tag, ROBOT_ID, ROBOT_SIZE, ROBOT_MARKER_POSE)
    joint = element(robot, "joint", name="room_tracking_marker_joint", type="fixed")
    element(joint, "parent", "base_footprint")
    element(joint, "child", "room_tracking_marker")
    world.append(robot)
    if include_owner:
        owner = owner_model()
        element(owner, "pose", "1 0 0 0 0 0")
        world.append(owner)
    state = element(world, "plugin", name="room_state", filename="libgazebo_ros_state.so")
    element(element(state, "ros"), "namespace", "/gazebo")
    element(state, "update_rate", 10)
    ET.indent(root)
    return ET.tostring(root, encoding="unicode")
