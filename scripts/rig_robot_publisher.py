#!/usr/bin/env python3
"""Stand-in copy of send_pos_2.py (Haichen's robot/toy detector + ROS publisher).

The detection itself is unchanged: OBB robot model, HSV green/red sticker masks, circular
centroid scoring, ROI-click homography (BL -> BR -> TR -> TL, 3.60 x 2.25 m) and the
green -> red heading.  Differences from the original:

  1. Paths come from the environment (ROBOT_MODEL, TOY_MODEL, CALIB_DIR), defaulting to
     rig/assets/robot/.  Nothing is hardcoded to /home/haichen.
  2. CAMERA_SERIAL selects the camera.  If it is unset the script only runs when exactly one
     camera is attached; it never guesses between several.
  3. A pose is published ONLY for a newly acquired frame in which the robot is detected AND
     both stickers are found.  The original republished its last pose on every failed
     detection and kept using the previous frame after a failed grab.  Here a lost robot
     publishes nothing, so the bridge's 0.5 s receive-time timeout stops following.
  4. Every retrieved grab result is released, including failed ones.
  5. Every acquired frame is processed (PROCESS_EVERY_N=1) instead of every 4th; the toy
     model, if present, runs every TOY_EVERY_N processed frames.  Publish and processing
     rates are logged every 5 s.
  6. Calibration is required (no silent no-undistortion), and the undistortion maps are
     precomputed once; the result is identical to cv2.undistort(frame, K, D).
  7. toy.pt is optional.
"""

import math
import os
import sys
import time
from pathlib import Path

try:
    import cv2
    import numpy as np
except ImportError:  # the pipeline logic is testable with fakes when these are absent
    cv2 = np = None

sys.path.insert(0, str(Path(__file__).resolve().parent))
from rig_capture import RateMonitor, acquire, choose_device, env_int, require_file

REPO = Path(__file__).resolve().parents[1]
ASSET_DIR = REPO / "rig" / "assets" / "robot"

DISPLAY_W, DISPLAY_H = 640, 480
MIN_MARKER_AREA = 5  # pixels
# Real-world ROI in centimetres, same corners/order as the original: BL, BR, TR, TL.
ROI_REAL_CM = [[0, 0], [360, 0], [360, 225], [0, 225]]
MIN_PROCESS_HZ = 4.0  # half of the bridge's 0.5 s timeout margin


def get_roi_points(frame):
    points = []
    window_name = "Select 4 ROI points (BL -> BR -> TR -> TL)"

    def click_event(event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            points.append((x, y))
            cv2.circle(frame, (x, y), 5, (0, 255, 0), -1)
            cv2.imshow(window_name, frame)

    while True:
        cv2.imshow(window_name, cv2.resize(frame, (DISPLAY_W, DISPLAY_H)))
        cv2.setMouseCallback(window_name, click_event)
        if len(points) == 4:
            break
        if cv2.waitKey(1) & 0xFF == ord("q"):
            break
    cv2.destroyAllWindows()
    return np.array(points, dtype=np.float32)


def make_color_masks(hsv):
    lower_red1, upper_red1 = np.array([0, 100, 50]), np.array([10, 255, 255])
    lower_red2, upper_red2 = np.array([170, 100, 50]), np.array([180, 255, 255])
    lower_green, upper_green = np.array([40, 60, 60]), np.array([90, 255, 255])
    red = cv2.bitwise_or(cv2.inRange(hsv, lower_red1, upper_red1),
                         cv2.inRange(hsv, lower_red2, upper_red2))
    green = cv2.inRange(hsv, lower_green, upper_green)
    kernel = np.ones((3, 3), np.uint8)
    red = cv2.morphologyEx(red, cv2.MORPH_CLOSE, kernel)
    green = cv2.morphologyEx(green, cv2.MORPH_CLOSE, kernel)
    return red, green


def find_largest_centroid(mask_img, min_area=MIN_MARKER_AREA):
    """Most circular, largest contour's centroid, or None."""
    contours, _ = cv2.findContours(mask_img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    best, best_score = None, -1
    for contour in contours:
        area = cv2.contourArea(contour)
        if area < min_area:
            continue
        perimeter = cv2.arcLength(contour, True)
        if perimeter == 0:
            continue
        score = 4 * math.pi * area / (perimeter ** 2) * area
        if score > best_score:
            best, best_score = contour, score
    if best is None:
        return None
    moments = cv2.moments(best)
    if moments["m00"] == 0:
        return None
    return int(moments["m10"] / moments["m00"]), int(moments["m01"] / moments["m00"])


class RobotDetector:
    """Robot OBB + green/red stickers -> (x_m, y_m, theta_rad) or None, never a cached value."""

    def __init__(self, robot_model, homography, camera_matrix=None, distortion=None):
        self.robot_model, self.H = robot_model, homography
        self.camera_matrix, self.distortion = camera_matrix, distortion
        self._maps = None

    def process_frame(self, frame):
        """Undistort (precomputed maps), resize to the display size, then detect."""
        if self.camera_matrix is not None:
            if self._maps is None or self._maps[0] != frame.shape[:2]:
                height, width = frame.shape[:2]
                maps = cv2.initUndistortRectifyMap(self.camera_matrix, self.distortion, None,
                                                   self.camera_matrix, (width, height), cv2.CV_16SC2)
                self._maps = (frame.shape[:2], *maps)
            frame = cv2.remap(frame, self._maps[1], self._maps[2], cv2.INTER_LINEAR)
        return self.process(cv2.resize(frame, (DISPLAY_W, DISPLAY_H)))

    def process(self, frame_display):
        hsv = cv2.cvtColor(frame_display, cv2.COLOR_BGR2HSV)
        results = self.robot_model.predict(
            frame_display, task="obb", imgsz=320, conf=0.60, iou=0.45,
            verbose=False, agnostic_nms=True, max_det=1)
        pose = None
        for result in results:
            if result.obb is None:
                continue
            for i in range(len(result.obb.cls)):
                if int(result.obb.cls[i].item()) != 0:  # robot class
                    continue
                cx, cy, w, h, angle = result.obb.xywhr[i].cpu().numpy()
                box = cv2.boxPoints(((cx, cy), (w, h), -np.degrees(angle))).astype(np.int32)
                cv2.drawContours(frame_display, [box], 0, (0, 255, 0), 2)
                real = cv2.perspectiveTransform(np.array([[[cx, cy]]], dtype=np.float32), self.H)
                robot_x, robot_y = real[0][0] / 100.0

                padding = 0.1
                x_min = max(0, int(min(box[:, 0]) - padding * w))
                y_min = max(0, int(min(box[:, 1]) - padding * h))
                x_max = min(DISPLAY_W, int(max(box[:, 0]) + padding * w))
                y_max = min(DISPLAY_H, int(max(box[:, 1]) + padding * h))
                mask = np.zeros((DISPLAY_H, DISPLAY_W), dtype=np.uint8)
                cv2.rectangle(mask, (x_min, y_min), (x_max, y_max), 255, -1)
                red_mask, green_mask = make_color_masks(cv2.bitwise_and(hsv, hsv, mask=mask))
                red, green = find_largest_centroid(red_mask), find_largest_centroid(green_mask)
                cv2.rectangle(frame_display, (x_min, y_min), (x_max, y_max), (255, 255, 0), 2)
                if not (red and green):
                    continue  # robot seen but heading unknown: not a valid observation

                points = np.array([[green, red]], dtype=np.float32)
                green_real, red_real = cv2.perspectiveTransform(points, self.H)[0]
                dx, dy = red_real[0] - green_real[0], red_real[1] - green_real[1]
                if math.hypot(dx, dy) < 1e-6:
                    continue
                theta = math.atan2(dy, dx)
                values = (float(robot_x), float(robot_y), theta)
                if not all(math.isfinite(v) for v in values):
                    continue
                pose = values
                cv2.line(frame_display, green, red, (255, 0, 0), 2)
                cv2.circle(frame_display, red, 8, (0, 0, 255), -1)
                cv2.circle(frame_display, green, 8, (0, 255, 0), -1)
                cv2.putText(frame_display,
                            f"Pos: ({robot_x:.2f},{robot_y:.2f})m  {math.degrees(theta):.0f}deg",
                            (green[0] + 10, green[1] + 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                            (255, 255, 255), 1)
                break
            if pose is not None:
                break
        return pose, frame_display


class RobotPipeline:
    """acquire -> detect -> publish.  Publishes only a fresh, complete observation."""

    def __init__(self, grab, detect, publish, process_every=1,
                 process_monitor=None, publish_monitor=None):
        self.grab, self.detect, self.publish = grab, detect, publish
        self.process_every = max(1, process_every)
        self.process_monitor, self.publish_monitor = process_monitor, publish_monitor
        self.acquired = 0
        self.processed = 0

    def step(self):
        """Return (status, overlay); status is no_frame, skipped, lost or published."""
        frame = self.grab()
        if frame is None:
            return "no_frame", None
        self.acquired += 1
        if self.acquired % self.process_every:
            return "skipped", None
        pose, overlay = self.detect(frame)
        self.processed += 1
        if self.process_monitor:
            self.process_monitor.tick()
        if pose is None:
            return "lost", overlay
        self.publish(*pose)
        if self.publish_monitor:
            self.publish_monitor.tick()
        return "published", overlay


def detect_toy(toy_model, frame_display, H):
    """Original toy logic: one OBB detection above 0.5 confidence -> (x_m, y_m) or None."""
    results = toy_model.predict(frame_display, task="obb", imgsz=640, conf=0.8,
                                verbose=False, max_det=1)
    for result in results:
        if result.obb is None:
            continue
        for i in range(len(result.obb.cls)):
            if result.obb.conf[i].item() > 0.5:
                cx, cy, w, h, angle = result.obb.xywhr[i].cpu().numpy()
                real = cv2.perspectiveTransform(np.array([[[cx, cy]]], dtype=np.float32), H)
                box = cv2.boxPoints(((cx, cy), (w, h), -np.degrees(angle))).astype(np.int32)
                cv2.drawContours(frame_display, [box], 0, (0, 0, 255), 2)
                return tuple(real[0][0] / 100.0)
    return None


def main():
    if cv2 is None:
        raise RuntimeError("OpenCV and numpy are required to run the robot publisher")
    import rclpy
    from geometry_msgs.msg import Pose2D
    from pypylon import pylon
    from ultralytics import YOLO
    from check_rig_assets import load_calibration

    model_path = require_file(os.environ.get("ROBOT_MODEL") or ASSET_DIR / "robot.pt", "ROBOT_MODEL")
    toy_path = os.environ.get("TOY_MODEL") or str(ASSET_DIR / "toy.pt")
    calib_dir = os.environ.get("CALIB_DIR") or str(ASSET_DIR)
    process_every = env_int("PROCESS_EVERY_N", 1)
    toy_every = env_int("TOY_EVERY_N", 4)
    camera_matrix, distortion = load_calibration(calib_dir)  # fatal if unusable
    robot_model = YOLO(model_path)
    toy_model = YOLO(toy_path) if Path(toy_path).is_file() else None
    print(f"Robot model: {model_path}\nToy model: {toy_path if toy_model else 'absent (toy skipped)'}"
          f"\nCalibration: {calib_dir}", flush=True)

    factory = pylon.TlFactory.GetInstance()
    device = choose_device(factory.EnumerateDevices(), os.environ.get("CAMERA_SERIAL"))
    print(f"Robot camera serial: {device.GetSerialNumber()}", flush=True)
    camera = pylon.InstantCamera(factory.CreateDevice(device))
    camera.Open()
    try:
        camera.AcquisitionFrameRateEnable.SetValue(True)
        camera.AcquisitionFrameRate.SetValue(10.0)
    except Exception:
        print("Could not set custom frame rate", flush=True)
    camera.StartGrabbing(pylon.GrabStrategy_LatestImageOnly)
    converter = pylon.ImageFormatConverter()
    converter.OutputPixelFormat = pylon.PixelType_BGR8packed
    converter.OutputBitAlignment = pylon.OutputBitAlignment_MsbAligned

    def grab():
        return acquire(camera, converter, pylon.TimeoutHandling_Return)

    rclpy.init()
    node = rclpy.create_node("rig_robot_publisher")
    robot_pub = node.create_publisher(Pose2D, "robot_position", 10)
    toy_pub = node.create_publisher(Pose2D, "toy_pose", 10)

    def publish(x, y, theta):
        robot_pub.publish(Pose2D(x=float(x), y=float(y), theta=float(theta)))
        node.get_logger().info(f"[Robot] x={x:.2f} m, y={y:.2f} m, theta={theta:.3f} rad",
                               throttle_duration_sec=0.5)

    try:
        first = None
        for _ in range(100):
            first = grab()
            if first is not None:
                break
        if first is None:
            raise RuntimeError("Robot camera delivered no frame")
        first = cv2.undistort(first, camera_matrix, distortion)
        roi_pixels = get_roi_points(cv2.resize(first, (DISPLAY_W, DISPLAY_H)))
        if len(roi_pixels) != 4:
            raise RuntimeError("ROI selection aborted (need 4 points: BL, BR, TR, TL)")
        H, _ = cv2.findHomography(roi_pixels, np.array(ROI_REAL_CM, dtype=np.float32))
        if H is None:
            raise RuntimeError("Could not compute the ROI homography")

        detector = RobotDetector(robot_model, H, camera_matrix, distortion)
        process_monitor = RateMonitor("robot frames processed", min_hz=MIN_PROCESS_HZ)
        publish_monitor = RateMonitor("robot_position published")
        pipeline = RobotPipeline(grab, detector.process_frame, publish, process_every,
                                 process_monitor, publish_monitor)
        print("Running. q = quit, p = pause (c/Esc resumes).", flush=True)
        while rclpy.ok() and camera.IsGrabbing():
            status, overlay = pipeline.step()
            process_monitor.poll()
            publish_monitor.poll()
            if status in ("lost", "published") and toy_model is not None \
                    and pipeline.processed % toy_every == 0:
                toy = detect_toy(toy_model, overlay, detector.H)
                # Original behaviour: 9999 marks "toy not found" (the bridge ignores this topic).
                toy_pub.publish(Pose2D(x=float(toy[0]) if toy else 9999.0,
                                       y=float(toy[1]) if toy else 9999.0, theta=0.0))
            if overlay is not None:
                if status == "lost":
                    cv2.putText(overlay, "Robot lost - not publishing", (20, 30),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
                cv2.imshow("Tracking", overlay)
            key = cv2.waitKey(1)
            if key == ord("q"):
                break
            if key == ord("p"):
                while cv2.waitKey(0) not in (ord("c"), 27):
                    pass
    except KeyboardInterrupt:
        pass
    finally:
        try:
            camera.StopGrabbing()
            camera.Close()
        except Exception:
            pass
        cv2.destroyAllWindows()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
