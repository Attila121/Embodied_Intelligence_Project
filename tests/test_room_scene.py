"""Scene structure and marker orientation match tracking calibration."""

from pathlib import Path
import sys
import unittest
import xml.etree.ElementTree as ET

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/"scripts"))
from room_obstacles import OBSTACLES, OBSTACLE_HEIGHT
from room_scene import OWNER_ID, ROBOT_ID, obstacle_model, owner_sdf, top_marker


class RoomSceneTests(unittest.TestCase):
    def test_top_marker_axes_match_object_corner_convention(self):
        link = ET.Element("link")
        top_marker(link, ROBOT_ID, 0.24, (-0.064, 0, 0.32))
        top_left = link.find("visual[@name='tag_0_0']")
        x, y, z, *_ = map(float, top_left.findtext("pose").split())
        self.assertLess(x, -0.064)
        self.assertGreater(y, 0)
        self.assertAlmostEqual(z+0.0015, 0.32)
        self.assertEqual(len(link.findall("visual")), 64)

    def test_owner_and_robot_use_distinct_ids(self):
        self.assertNotEqual(OWNER_ID, ROBOT_ID)
        root = ET.fromstring(owner_sdf())
        self.assertEqual(root.find("model").attrib["name"], "room_owner")
        self.assertEqual(root.findtext("model/static"), "true")
        self.assertEqual(len(root.findall("model/link/visual")), 66)

    def test_obstacle_models_are_static_collidable_and_low(self):
        for spec in OBSTACLES:
            model = obstacle_model(spec)
            self.assertEqual(model.attrib["name"], spec["name"])
            self.assertEqual(model.findtext("static"), "true")
            self.assertEqual(len(model.findall("link/collision")), 1)
            self.assertEqual(len(model.findall("link/visual")), 1)
            x, y, *_ = map(float, model.findtext("pose").split())
            self.assertEqual((x, y), (spec["x"], spec["y"]))
            _, _, z, *_ = map(float, model.findtext("link/visual/pose").split())
            self.assertAlmostEqual(z, OBSTACLE_HEIGHT / 2)       # sits on the floor
            if spec["shape"] == "cylinder":
                self.assertAlmostEqual(float(model.findtext("link/collision/geometry/cylinder/radius")),
                                       spec["radius"])
                self.assertAlmostEqual(float(model.findtext("link/collision/geometry/cylinder/length")),
                                       OBSTACLE_HEIGHT)
            else:
                size = list(map(float, model.findtext("link/collision/geometry/box/size").split()))
                self.assertEqual(size, [*spec["size"], OBSTACLE_HEIGHT])


if __name__ == "__main__":
    unittest.main()
