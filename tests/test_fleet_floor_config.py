import sys
import tempfile
import unittest
from pathlib import Path

sys.path.insert(0, str(
    Path(__file__).resolve().parents[1] / "ros2_ws/src/slam_localization_3d/launch"
))
from robot_fleet import fleet_floor_maps, fleet_rviz


class FleetFloorConfigTest(unittest.TestCase):
    def test_auto_initialization_hides_only_reference_display(self):
        config = fleet_rviz(["carter1"], simulation_reference=False)
        displays = config["Visualization Manager"]["Displays"]
        reference = next(display for display in displays
                         if display["Name"] == "Simulation reference (approximate)")
        self.assertFalse(reference["Enabled"])
        self.assertTrue(next(display for display in displays if display["Name"] == "carter1")["Enabled"])

    def test_maps_are_independent_of_robot_navigation_maps(self):
        config = fleet_rviz(["carter1", "carter2"], ["B1", "1F"])
        panel = config["Panels"][-1]
        self.assertEqual(panel["Floors"], ["B1", "1F"])
        self.assertEqual(panel["Selected Floor"], "B1")
        self.assertEqual(panel["Selected Robot"], "carter1")
        displays = config["Visualization Manager"]["Displays"]
        map_display = next(display for display in displays if display["Name"] == "Map")
        self.assertEqual(map_display["Topic"]["Value"], "/fleet/maps/floor_0/map")
        self.assertEqual(map_display["Topic"]["Durability Policy"], "Transient Local")
        reference = next(display for display in displays
                         if display["Name"] == "Simulation reference (approximate)")
        self.assertEqual(reference["Class"], "rviz_default_plugins/MarkerArray")
        self.assertTrue(reference["Enabled"])
        self.assertEqual(reference["Topic"]["Value"], "/fleet/maps/floor_0/simulation_reference")
        groups = [display for display in displays if display["Class"] == "rviz_common/Group"]
        self.assertEqual([group["Name"] for group in groups], ["carter1", "carter2"])
        for group in groups:
            pose = next(display for display in group["Displays"] if display["Name"] == "Pose")
            self.assertEqual(pose["Topic"]["Value"], f"/{group['Name']}/odometry/global")

    def test_discover_maps_with_initial_floor_first(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            for floor in ("1F", "B1", "B2", "unused"):
                (root / floor).mkdir()
                if floor != "unused":
                    (root / floor / "map_2d.yaml").touch()
            maps = fleet_floor_maps(root, "B1")
            self.assertEqual([name for name, _ in maps], ["B1", "1F", "B2"])
            self.assertEqual(maps[0][1], root / "B1/map_2d.yaml")

    def test_missing_map_directory_or_initial_floor_is_explicit(self):
        with tempfile.TemporaryDirectory() as directory:
            with self.assertRaisesRegex(ValueError, "not a directory"):
                fleet_floor_maps(Path(directory) / "missing", "1F")
            with self.assertRaisesRegex(ValueError, "has no map_2d.yaml"):
                fleet_floor_maps(directory, "1F")

    def test_explicit_monitoring_floors_exclude_other_maps(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            for floor in ("1F", "B1", "office"):
                (root / floor).mkdir()
                (root / floor / "map_2d.yaml").touch()
            maps = fleet_floor_maps(root, "1F", ["1F", "B1"])
            self.assertEqual([name for name, _ in maps], ["1F", "B1"])
            custom = fleet_floor_maps(root, "office", ["1F", "B1"])
            self.assertEqual([name for name, _ in custom], ["office", "1F", "B1"])
            with self.assertRaisesRegex(ValueError, "has no map_2d.yaml"):
                fleet_floor_maps(root, "1F", ["1F", "B2"])
            for floors in ([], [""], ["1F", "1F"]):
                with self.subTest(floors=floors), self.assertRaises(ValueError):
                    fleet_floor_maps(root, "1F", floors)

    def test_invalid_floor_lists_are_rejected(self):
        for floors in ([], ["1F", "1F"], [""]):
            with self.subTest(floors=floors), self.assertRaises(ValueError):
                fleet_rviz(["carter1"], floors)


if __name__ == "__main__":
    unittest.main()
