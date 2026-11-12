"""Robot publisher: stale-frame/stale-pose behaviour, tested through the real code paths.

Pipeline tests need no OpenCV.  Detector tests use a synthetic image and a fake YOLO model
and are skipped where OpenCV/numpy are missing.
"""

import math
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))
sys.path.insert(0, str(Path(__file__).resolve().parent))
import rig_robot_publisher as robot
from rig_capture import RateMonitor, acquire
from test_rig_capture import FakeCamera, FakeConverter, FakeGrab

cv2, np = robot.cv2, robot.np


class PipelineTests(unittest.TestCase):
    def build(self, frames, poses, process_every=1):
        frames, poses = list(frames), list(poses)
        self.detected, self.published = [], []

        def grab():
            return frames.pop(0)

        def detect(frame):
            self.detected.append(frame)
            return poses.pop(0), f"overlay-{frame}"

        return robot.RobotPipeline(grab, detect, lambda *p: self.published.append(p), process_every)

    def test_failed_acquisition_publishes_nothing_and_skips_detection(self):
        pipeline = self.build([None], [])
        self.assertEqual(pipeline.step(), ("no_frame", None))
        self.assertEqual((self.detected, self.published), ([], []))

    def test_robot_not_detected_publishes_nothing(self):
        pipeline = self.build(["f1"], [None])
        self.assertEqual(pipeline.step()[0], "lost")
        self.assertEqual(self.published, [])

    def test_complete_detection_publishes_exactly_once(self):
        pipeline = self.build(["f1"], [(1.0, 2.0, 0.5)])
        self.assertEqual(pipeline.step(), ("published", "overlay-f1"))
        self.assertEqual(self.published, [(1.0, 2.0, 0.5)])

    def test_loss_after_a_pose_never_republishes_the_cached_pose(self):
        pipeline = self.build(["f1", "f2", None, "f4"], [(1.0, 2.0, 0.5), None, None])
        statuses = [pipeline.step()[0] for _ in range(4)]
        self.assertEqual(statuses, ["published", "lost", "no_frame", "lost"])
        self.assertEqual(self.published, [(1.0, 2.0, 0.5)])

    def test_process_every_n_skips_frames_without_detecting_them(self):
        pipeline = self.build(["a", "b", "c", "d"], [(0, 0, 0), (1, 1, 1)], process_every=2)
        statuses = [pipeline.step()[0] for _ in range(4)]
        self.assertEqual(statuses, ["skipped", "published", "skipped", "published"])
        self.assertEqual(self.detected, ["b", "d"])

    def test_monitors_count_processed_and_published_frames(self):
        process, publish = RateMonitor("p", log=lambda *_: None), RateMonitor("q", log=lambda *_: None)
        frames, poses = ["a", "b"], [(0, 0, 0), None]
        pipeline = robot.RobotPipeline(lambda: frames.pop(0), lambda f: (poses.pop(0), f),
                                       lambda *p: None, 1, process, publish)
        pipeline.step()
        pipeline.step()
        self.assertEqual((process.count, publish.count), (2, 1))

    def test_real_acquire_with_failed_grabs_publishes_only_the_fresh_frame(self):
        """A failed GrabSucceeded() after a good frame must not re-run detection on old data."""
        camera = FakeCamera([FakeGrab(tag="good"), FakeGrab(succeeded=False), TimeoutError("t"),
                             FakeGrab(valid=False)])
        detections = []

        def detect(frame):
            detections.append(frame)
            return (1.0, 1.0, 0.0), frame

        published = []
        pipeline = robot.RobotPipeline(lambda: acquire(camera, FakeConverter(), "return"),
                                       detect, lambda *p: published.append(p))
        statuses = [pipeline.step()[0] for _ in range(4)]
        self.assertEqual(statuses, ["published", "no_frame", "no_frame", "no_frame"])
        self.assertEqual(detections, ["good"])
        self.assertEqual(len(published), 1)
        self.assertTrue(all(g.released == 1 for g in camera.grabs))


@unittest.skipIf(cv2 is None or np is None, "OpenCV/numpy required for synthetic detection")
class DetectorTests(unittest.TestCase):
    class Scalar:
        def __init__(self, value):
            self.value = np.array(value, dtype=float)

        def cpu(self):
            return self

        def numpy(self):
            return self.value

        def item(self):
            return float(self.value)

    class Model:
        def __init__(self, detections):
            self.detections = detections  # list of (class_id, [cx, cy, w, h, angle])

        def predict(self, *args, **kwargs):
            scalar = DetectorTests.Scalar
            obb = type("Obb", (), {
                "cls": [scalar(c) for c, _ in self.detections],
                "xywhr": [scalar(box) for _, box in self.detections],
            })()
            return [type("Result", (), {"obb": obb if self.detections else None})()]

    def setUp(self):
        pixels = np.array([[0, 480], [640, 480], [640, 0], [0, 0]], dtype=np.float32)  # BL BR TR TL
        self.H, _ = cv2.findHomography(pixels, np.array(robot.ROI_REAL_CM, dtype=np.float32))

    def frame(self, green=None, red=None):
        image = np.zeros((480, 640, 3), np.uint8)
        if green:
            cv2.circle(image, green, 8, (0, 200, 0), -1)
        if red:
            cv2.circle(image, red, 8, (0, 0, 255), -1)
        return image

    def detector(self, detections):
        return robot.RobotDetector(self.Model(detections), self.H)

    BOX = [320, 240, 120, 80, 0.0]

    def test_robot_and_both_stickers_give_position_and_heading(self):
        pose, _ = self.detector([(0, self.BOX)]).process(self.frame(green=(300, 240), red=(340, 240)))
        self.assertAlmostEqual(pose[0], 1.8, places=2)
        self.assertAlmostEqual(pose[1], 1.125, places=2)
        self.assertAlmostEqual(pose[2], 0.0, places=2)

    def test_heading_follows_green_to_red_direction(self):
        # Image-up maps to room +y (BL origin), so red above green is +90 degrees.
        pose, _ = self.detector([(0, self.BOX)]).process(self.frame(green=(300, 255), red=(300, 225)))
        self.assertAlmostEqual(pose[2], math.pi / 2, places=2)

    def test_missing_sticker_is_not_an_observation(self):
        det = self.detector([(0, self.BOX)])
        self.assertIsNone(det.process(self.frame(green=(300, 240)))[0])
        self.assertIsNone(det.process(self.frame(red=(340, 240)))[0])
        self.assertIsNone(det.process(self.frame())[0])

    def test_stickers_outside_the_robot_region_are_ignored(self):
        pose, _ = self.detector([(0, self.BOX)]).process(self.frame(green=(40, 40), red=(80, 40)))
        self.assertIsNone(pose)

    def test_no_robot_or_wrong_class_gives_none(self):
        both = dict(green=(300, 240), red=(340, 240))
        self.assertIsNone(self.detector([]).process(self.frame(**both))[0])
        self.assertIsNone(self.detector([(1, self.BOX)]).process(self.frame(**both))[0])

    def test_detector_inside_pipeline_does_not_republish_after_loss(self):
        det = self.detector([(0, self.BOX)])
        frames = [self.frame(green=(300, 240), red=(340, 240)), self.frame(green=(300, 240))]
        published = []
        pipeline = robot.RobotPipeline(lambda: frames.pop(0), det.process,
                                       lambda *p: published.append(p))
        self.assertEqual([pipeline.step()[0] for _ in range(2)], ["published", "lost"])
        self.assertEqual(len(published), 1)


if __name__ == "__main__":
    unittest.main()
