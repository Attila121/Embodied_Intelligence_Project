"""Scene structure and marker orientation match tracking calibration."""

from pathlib import Path
import sys
import unittest
import xml.etree.ElementTree as ET

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/"scripts"))
from room_scene import OWNER_ID, ROBOT_ID, owner_sdf, top_marker


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


if __name__ == "__main__":
    unittest.main()
