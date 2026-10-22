import sys
import tempfile
import unittest
from pathlib import Path

sys.path.insert(0, str(
    Path(__file__).resolve().parents[1] / "ros2_ws/src/slam_localization_3d/scripts"
))
from floor_maps import fleet_mask_topics, fleet_zone_namespace, floor_maps


class FloorMapsTest(unittest.TestCase):
    def test_repository_floors_have_complete_map_pairs(self):
        root = Path(__file__).resolve().parents[1] / "maps"
        for name in ("1F", "B1"):
            with self.subTest(floor=name):
                yaml, pcd = floor_maps(root, name)
                self.assertEqual(Path(yaml).parent, root / name)
                self.assertEqual(Path(pcd).parent, root / name)

    def test_rejects_path_traversal_unknown_floor_and_incomplete_pair(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            (root / "B1").mkdir()
            for name in ("../B1", "/B1", "", "missing", "B1"):
                with self.subTest(floor=name), self.assertRaises((ValueError, OSError)):
                    floor_maps(root, name)

    def test_rejects_missing_image(self):
        with tempfile.TemporaryDirectory() as directory:
            floor = Path(directory) / "B1"
            floor.mkdir()
            (floor / "map.pcd").touch()
            (floor / "map_2d.yaml").write_text("image: missing.pgm\n")
            with self.assertRaisesRegex(ValueError, "Missing map image"):
                floor_maps(directory, "B1")

    def test_fleet_zone_topics_are_valid_ros_names_or_rejected(self):
        self.assertEqual(fleet_mask_topics("1F"), [
            "/fleet/zones/floor_1F/costmap_filters/keepout_mask",
            "/fleet/zones/floor_1F/costmap_filters/speed_mask",
        ])
        for name in ("B-1", "", "../1F", "1F/x"):
            with self.subTest(floor=name), self.assertRaisesRegex(ValueError, "floor names"):
                fleet_zone_namespace(name)
