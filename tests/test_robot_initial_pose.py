"""Map-session startup seeds and actual per-robot one-shot initialpose publication."""

import importlib.util
import json
import math
from pathlib import Path
import tempfile
import time
import unittest
from unittest.mock import patch

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from launch import LaunchContext
from nav_msgs.msg import Odometry
from rclpy.executors import SingleThreadedExecutor
from rclpy.parameter import Parameter

from tests.test_fleet_ground_truth import FloorReference, session
from global_pose_adapter import GlobalPoseAdapter


ROOT = Path(__file__).resolve().parents[1]
module_spec = importlib.util.spec_from_file_location(
    "robot_startup_launch", ROOT / "ros2_ws/src/slam_bringup/launch/robot.launch.py")
robot_launch = importlib.util.module_from_spec(module_spec)
module_spec.loader.exec_module(robot_launch)
from robot_fleet import body_pose, imu_mount, initial_pose, load_robot_profile, parse_robot_spec


class RobotInitialPoseTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.directory = tempfile.TemporaryDirectory()
        self.floor = Path(self.directory.name) / "1F"
        self.floor.mkdir()
        (self.floor / "session.json").write_text(json.dumps(session("1F")))

    def tearDown(self):
        self.directory.cleanup()

    def arguments(self, robot, auto=True, floor=None):
        floor = floor or self.floor
        context = LaunchContext()
        context.launch_configurations.update({
            "robot": robot, "auto_initial_pose": str(auto).lower(),
            "map_pgm": str(floor / "map_2d.yaml"), "map_pcd": str(floor / "map.pcd"),
            "mqtt_host": "",
        })
        with patch.object(robot_launch, "IncludeLaunchDescription") as include:
            robot_launch.robot_stack(context, robot_launch.LOCALIZATION)
        return dict(include.call_args_list[0].kwargs["launch_arguments"])

    def seed(self, arguments):
        return tuple(float(arguments[f"initial_{key}"]) for key in ("x", "y", "z", "yaw"))

    def test_session_transform_is_used_instead_of_office_seed(self):
        spec = parse_robot_spec("prior_test:carter_v1@12,23,1.2")
        profile = load_robot_profile(spec.robot_type)
        seed = self.seed(self.arguments("prior_test:carter_v1@12,23,1.2"))
        reference = FloorReference.load("1F", self.floor / "session.json")
        expected = reference.initial_body_pose(body_pose(spec, profile))
        for actual, target in zip(seed, expected):
            self.assertAlmostEqual(actual, target)
        self.assertNotEqual(seed, initial_pose(spec, profile))

    def test_missing_or_invalid_session_requires_explicit_manual_mode(self):
        path = self.floor / "session.json"
        for data in (None, "{}", "{bad json"):
            if data is None:
                path.unlink()
            else:
                path.write_text(data)
            with self.subTest(data=data):
                with self.assertRaisesRegex(ValueError, "auto_initial_pose:=false"):
                    self.arguments("prior_test@0,0")
                self.arguments("prior_test@0,0", auto=False)

    def test_legacy_office_without_session_preserves_seed(self):
        office = Path(self.directory.name) / "office"
        office.mkdir()
        spec = parse_robot_spec("prior_test@3.5,0,0.7")
        self.assertEqual(
            self.seed(self.arguments("prior_test@3.5,0,0.7", floor=office)),
            initial_pose(spec, load_robot_profile(spec.robot_type)),
        )

    def test_real_ros_routes_one_prior_per_robot_and_none_in_manual_mode(self):
        executor = SingleThreadedExecutor()
        nodes, adapters, publishers, received = [], [], [], {}
        try:
            for index, (robot_type, auto) in enumerate(
                    (("nova_carter", True), ("carter_v1", True), ("nova_carter", False))):
                name = f"prior_test_{index}"
                arguments = self.arguments(f"{name}:{robot_type}@{12 + 3.5 * index},23,0.7", auto)
                seed = self.seed(arguments)
                profile = load_robot_profile(robot_type)
                adapter = GlobalPoseAdapter(namespace=name, parameter_overrides=[
                    Parameter("auto_initial_pose", value=auto),
                    Parameter("imu_mount", value=imu_mount(profile)),
                    *(Parameter(f"initial_{key}", value=value)
                      for key, value in zip(("x", "y", "z", "yaw"), seed)),
                ])
                upstream = rclpy.create_node("global_localization", namespace=name)
                received[name] = []
                upstream.create_subscription(
                    PoseWithCovarianceStamped, "initialpose",
                    lambda message, name=name: received[name].append(message), 10)
                publishers.append(upstream.create_publisher(Odometry, "Odometry", 10))
                adapters.append(adapter)
                for node in (adapter, upstream):
                    nodes.append(node)
                    executor.add_node(node)

            deadline = time.monotonic() + 10.0
            extra_until = None
            while time.monotonic() < deadline:
                for adapter, publisher in zip(adapters, publishers):
                    message = Odometry()
                    message.header.frame_id, message.child_frame_id = "camera_init", "body"
                    message.header.stamp = adapter.get_clock().now().to_msg()
                    message.pose.pose.orientation.w = 1.0
                    publisher.publish(message)
                executor.spin_once(timeout_sec=0.01)
                if all(received[f"prior_test_{index}"] for index in (0, 1)):
                    if extra_until is None:
                        extra_until = time.monotonic() + 0.5
                    if time.monotonic() >= extra_until:
                        break
            for index in (0, 1):
                messages = received[f"prior_test_{index}"]
                self.assertEqual(len(messages), 1, received)
                message = messages[0]
                self.assertEqual(message.header.frame_id, "map")
                self.assertGreater(message.header.stamp.sec, 0)
                pose = message.pose.pose
                self.assertAlmostEqual(pose.position.x, 1.0 + 3.5 * index)
                self.assertAlmostEqual(pose.position.y, 3.0)
                self.assertAlmostEqual(pose.orientation.z, math.sin(0.7 / 2))
                self.assertAlmostEqual(pose.orientation.w, math.cos(0.7 / 2))
            self.assertEqual(received["prior_test_2"], [])
        finally:
            executor.shutdown()
            for node in nodes:
                node.destroy_node()


if __name__ == "__main__":
    unittest.main()
