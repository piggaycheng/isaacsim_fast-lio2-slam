import copy
import json
import math
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import Mock, patch

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from rclpy.parameter import Parameter
from visualization_msgs.msg import Marker

sys.path.insert(0, str(
    Path(__file__).resolve().parents[1] / "ros2_ws/src/slam_localization_3d/scripts"
))
from fleet_ground_truth import (
    FleetGroundTruth, FloorReference, floor_at_height, inverse_pose, reference_markers,
)
from localization_3d_pose import compose_pose


def session(floor, height=0.0):
    pose = {"position": [10.0, 20.0, height], "orientation_xyzw": [0.0, 0.0, 0.0, 1.0],
            "frame_id": "isaac_world", "child_frame_id": "chassis_link"}
    return {
        "floor": floor,
        "reference": {
            "isaac_chassis_pose": pose,
            "lio_initial_pose": {
                **pose, "position": [0.0, 0.0, 0.0],
                "frame_id": "lidar", "child_frame_id": "body",
            },
            "sensor_frames": {"imu_link": {
                "x": 1.0, "y": 0.0, "z": 1.0, "roll": 0.0, "pitch": 0.0, "yaw": 0.0,
            }},
        },
    }


class FloorReferenceTest(unittest.TestCase):
    def load(self, data, floor="1F"):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "session.json"
            path.write_text(json.dumps(data))
            return FloorReference.load(floor, path)

    def test_reference_chassis_projects_to_lio_body_minus_imu_mount(self):
        reference = self.load(session("1F"))
        position, quaternion = reference.project(((10, 20, 0), (0, 0, 0, 1)))
        self.assertEqual(position, (-1.0, 0.0, -1.0))
        self.assertEqual(quaternion, (0.0, 0.0, 0.0, 1.0))

    def test_world_rotation_and_nonzero_lio_reference_are_composed(self):
        data = session("1F")
        half = math.sqrt(0.5)
        for pose in ("isaac_chassis_pose", "lio_initial_pose"):
            data["reference"][pose]["orientation_xyzw"] = [0, 0, half, half]
        data["reference"]["lio_initial_pose"]["position"] = [3, 4, 1]
        reference = self.load(data)
        position, quaternion = reference.project(((11, 20, 0), (0, 0, half, half)))
        for actual, expected in zip(position, (4, 3, 0)):
            self.assertAlmostEqual(actual, expected)
        self.assertAlmostEqual(quaternion[2], half)
        self.assertAlmostEqual(quaternion[3], half)

    def test_inverse_composes_to_identity(self):
        pose = ((3, -2, 5), (0.1, 0.2, 0.3, math.sqrt(0.86)))
        position, quaternion = compose_pose(pose, inverse_pose(pose))
        for actual, expected in zip((*position, *quaternion), (0, 0, 0, 0, 0, 0, 1)):
            self.assertAlmostEqual(actual, expected)

    def test_physical_height_not_localization_selects_floor(self):
        references = [self.load(session("1F")), self.load(session("B1", -3.3), "B1")]
        self.assertEqual(floor_at_height(references, 0.2, 0.75).name, "1F")
        self.assertEqual(floor_at_height(references, -3.1, 0.75).name, "B1")
        self.assertIsNone(floor_at_height(references, -1.6, 0.75))
        self.assertIsNone(floor_at_height(references, -1.6, 2.0))

    def test_invalid_references_are_rejected(self):
        data = session("1F")
        cases = []
        for key, value in (("frame_id", "map"), ("position", [0, 0, math.nan]),
                           ("orientation_xyzw", [0, 0, 0, 0])):
            bad = copy.deepcopy(data)
            bad["reference"]["isaac_chassis_pose"][key] = value
            cases.append(bad)
        bad = copy.deepcopy(data)
        bad["floor"] = "B1"
        cases.append(bad)
        bad = copy.deepcopy(data)
        bad["reference"] = None
        cases.append(bad)
        for bad in cases:
            with self.subTest(data=bad), self.assertRaises(ValueError):
                self.load(bad)

    def test_markers_are_planar_yellow_labeled_and_expire(self):
        half = math.sqrt(0.5)
        markers = reference_markers("carter1", ((1, 2, -0.526), (0, 0, half, half)),
                                    Odometry().header.stamp)
        arrow, label = markers
        self.assertEqual((arrow.type, label.type), (Marker.ARROW, Marker.TEXT_VIEW_FACING))
        self.assertEqual((arrow.pose.position.x, arrow.pose.position.y), (1, 2))
        self.assertEqual(arrow.pose.position.z, 0.1)
        self.assertAlmostEqual(arrow.pose.orientation.z, half)
        self.assertEqual((arrow.color.r, arrow.color.g, arrow.color.b), (1.0, 0.9, 0.1))
        self.assertIn("carter1", label.text)
        self.assertIn("SIM REF ~", label.text)
        self.assertTrue(all(marker.lifetime.sec == 1 for marker in markers))

    def test_actual_floor_metadata_can_be_loaded(self):
        root = Path(__file__).resolve().parents[1] / "maps"
        if not all((root / floor / "session.json").is_file() for floor in ("1F", "B1")):
            self.skipTest("Local mapping sessions are not present (maps/ is not versioned)")
        references = [FloorReference.load(floor, root / floor / "session.json")
                      for floor in ("1F", "B1")]
        self.assertGreater(references[0].world_height - references[1].world_height, 3.0)


class FleetGroundTruthTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.directory = tempfile.TemporaryDirectory()
        paths = []
        for floor, height in (("1F", 0.0), ("B1", -3.3)):
            path = Path(self.directory.name) / f"{floor}.json"
            path.write_text(json.dumps(session(floor, height)))
            paths.append(str(path))
        self.node = FleetGroundTruth(parameter_overrides=[
            Parameter("robots", value=["carter1", "carter2"]),
            Parameter("spawn_robots", value=["carter1", "carter2"]),
            Parameter("spawn_poses", value=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0]),
            Parameter("floors", value=["1F", "B1"]),
            Parameter("session_files", value=paths),
        ])
        self.outputs = {}
        self.node.marker_publishers = {
            floor: Mock(publish=lambda msg, floor=floor: self.outputs.__setitem__(floor, msg))
            for floor in ("1F", "B1")
        }

    def tearDown(self):
        self.node.destroy_node()
        self.directory.cleanup()

    def truth(self, z=0.0):
        message = Odometry()
        message.header.frame_id = "isaac_world"
        message.pose.pose.position.x, message.pose.pose.position.y = 12.0, 23.0
        message.pose.pose.position.z = z
        message.pose.pose.orientation.w = 1.0
        return message

    def added(self, floor):
        return [marker for marker in self.outputs[floor].markers if marker.action == Marker.ADD]

    def test_markers_exist_without_localization_or_floor_manager(self):
        self.node.on_truth("carter1", self.truth())
        self.node.publish_references()
        self.assertEqual(len(self.added("1F")), 2)
        self.assertEqual(len(self.added("B1")), 0)
        arrow = self.added("1F")[0]
        self.assertEqual((arrow.pose.position.x, arrow.pose.position.y), (1.0, 3.0))
        publishers = self.node.get_publisher_names_and_types_by_node(
            self.node.get_name(), self.node.get_namespace())
        self.assertTrue(all("simulation_reference" in topic or topic in
                            ("/rosout", "/parameter_events") for topic, _ in publishers))

    def test_two_robots_on_different_floors_and_elevator_transit(self):
        self.node.on_truth("carter1", self.truth())
        self.node.on_truth("carter2", self.truth(-3.3))
        self.node.publish_references()
        self.assertEqual(self.added("1F")[0].ns, "simulation_reference/carter1")
        self.assertEqual(self.added("B1")[0].ns, "simulation_reference/carter2")
        self.node.on_truth("carter1", self.truth(-1.6))
        self.node.publish_references()
        self.assertEqual(self.added("1F"), [])
        self.assertEqual(len(self.added("B1")), 2)

    def test_identical_local_truth_preserves_spawn_separation(self):
        self.node.world_from_spawn["carter2"] = ((3.5, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0))
        for robot in ("carter1", "carter2"):
            self.node.on_truth(robot, self.truth())
        self.node.publish_references()
        arrows = [marker for marker in self.added("1F") if marker.type == Marker.ARROW]
        self.assertEqual(len(arrows), 2)
        self.assertAlmostEqual(arrows[1].pose.position.x - arrows[0].pose.position.x, 3.5)
        self.assertAlmostEqual(arrows[1].pose.position.y - arrows[0].pose.position.y, 0.0)

    def test_spawn_rotation_transforms_position_and_heading(self):
        half = math.sqrt(0.5)
        self.node.world_from_spawn["carter2"] = ((10.0, 20.0, 0.0), (0.0, 0.0, half, half))
        message = self.truth()
        message.pose.pose.position.x, message.pose.pose.position.y = 2.0, 0.0
        self.node.on_truth("carter2", message)
        self.node.publish_references()
        arrow = self.added("1F")[0]
        self.assertAlmostEqual(arrow.pose.position.x, -1.0)
        self.assertAlmostEqual(arrow.pose.position.y, 2.0)
        self.assertAlmostEqual(arrow.pose.orientation.z, half)
        self.assertAlmostEqual(arrow.pose.orientation.w, half)

    def test_unknown_spawn_does_not_assume_origin(self):
        self.node.world_from_spawn.pop("carter2")
        self.node.on_truth("carter2", self.truth())
        self.node.publish_references()
        self.assertNotIn("carter2", self.node.truth)
        self.assertEqual(self.added("1F"), [])

    def test_manual_pose_clears_only_that_robot_and_does_not_reappear(self):
        for robot in self.node.robots:
            self.node.on_truth(robot, self.truth())
        message = PoseWithCovarianceStamped()
        message.header.frame_id = "map"
        message.pose.pose.orientation.w = 1.0
        self.node.on_initial_pose("carter1", message)
        self.assertEqual({marker.ns for marker in self.added("1F")},
                         {"simulation_reference/carter2"})
        self.node.on_truth("carter1", self.truth(-3.3))
        self.node.publish_references()
        self.assertEqual(self.added("B1"), [])
        deleted = [marker for marker in self.outputs["1F"].markers
                   if marker.ns == "simulation_reference/carter1"]
        self.assertTrue(all(marker.action == Marker.DELETE for marker in deleted))

    def test_invalid_manual_pose_does_not_clear_reference(self):
        self.node.on_truth("carter1", self.truth())
        message = PoseWithCovarianceStamped()
        message.header.frame_id = "map"
        message.pose.pose.orientation.w = 0.0
        self.node.on_initial_pose("carter1", message)
        self.node.publish_references()
        self.assertEqual(len(self.added("1F")), 2)

    def test_invalid_and_stale_truth_remove_markers(self):
        self.node.on_truth("carter1", self.truth())
        with patch("fleet_ground_truth.time.monotonic",
                   return_value=self.node.truth["carter1"][1] + 3.0):
            self.node.publish_references()
        self.assertEqual(self.added("1F"), [])
        invalid = self.truth()
        invalid.header.frame_id = "map"
        self.node.on_truth("carter1", invalid)
        self.assertNotIn("carter1", self.node.truth)
        invalid = self.truth()
        invalid.pose.pose.position.x = math.nan
        self.node.on_truth("carter1", invalid)
        self.assertNotIn("carter1", self.node.truth)

    def test_missing_reference_does_not_use_identity_transform(self):
        self.node.references.pop("1F")
        self.node.on_truth("carter1", self.truth())
        self.node.publish_references()
        self.assertEqual(self.added("1F"), [])
        self.assertEqual(self.added("B1"), [])

    def test_missing_session_is_reported_without_breaking_monitoring(self):
        node = FleetGroundTruth(namespace="missing_reference_test", parameter_overrides=[
            Parameter("robots", value=["missing_reference_probe"]),
            Parameter("floors", value=["1F"]),
            Parameter("session_files", value=[str(Path(self.directory.name) / "missing.json")]),
        ])
        try:
            self.assertEqual(node.references, {})
            node.publish_references()
        finally:
            node.destroy_node()

    def test_snapshot_contains_every_robot_on_the_floor(self):
        for robot in ("carter1", "carter2"):
            self.node.on_truth(robot, self.truth())
        self.node.publish_references()
        self.assertEqual(len(self.added("1F")), 4)
        self.assertEqual({marker.ns for marker in self.added("1F")},
                         {"simulation_reference/carter1", "simulation_reference/carter2"})


if __name__ == "__main__":
    unittest.main()
