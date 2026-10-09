import ast
import sys
import time
import unittest
from pathlib import Path
from unittest.mock import patch

import rclpy
import yaml
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, Header

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))
sys.path.insert(0, str(ROOT / "ros2_ws/src/slam_localization_3d/scripts"))
from cmd_vel_control import CmdVelReceiver
from cmd_vel_safety import CmdVelSafety


class TestCmdVelReceiver(unittest.TestCase):
    def setUp(self):
        self.receiver = CmdVelReceiver()

    def test_command_requires_fresh_ros_message(self):
        self.assertEqual(self.receiver.command(), (0.0, 0.0))
        self.receiver.accept_twist((0.15, 0, 0), (0, 0, 0.3))
        self.assertEqual(self.receiver.command(), (0.15, 0.3))
        with patch("cmd_vel_control.time.monotonic", return_value=self.receiver.last_received + 0.6):
            self.assertEqual(self.receiver.command(), (0.0, 0.0))

    def test_speed_limits_are_inclusive_in_both_directions(self):
        for sign in (1, -1):
            self.receiver.accept_twist((sign * 0.75, 0, 0), (0, 0, sign * 0.5))
            self.assertEqual(self.receiver.command(), (sign * 0.75, sign * 0.5))

    def test_carter_navigation_and_simulator_limits_match(self):
        config = ROOT / "ros2_ws/src/slam_localization_3d/config"
        overrides = yaml.safe_load((config / "robots/nova_carter.yaml").read_text())[
            "parameter_overrides"
        ]
        parameters = overrides["cmd_vel_safety"]["ros__parameters"]
        self.assertEqual(parameters["max_linear_speed"], self.receiver.max_linear)
        self.assertEqual(parameters["max_angular_speed"], self.receiver.max_angular)
        smoother = overrides["velocity_smoother"]["ros__parameters"]
        self.assertEqual(smoother["max_velocity"], [0.75, 0.0, 0.5])
        self.assertEqual(smoother["min_velocity"], [-0.75, 0.0, -0.5])
        controller = overrides["controller_server"]["ros__parameters"]["FollowPath"]
        self.assertEqual(controller["desired_linear_vel"], 0.75)
        self.assertEqual(controller["rotate_to_heading_angular_vel"], 0.5)
        assignments = {
            target.id: ast.literal_eval(node.value)
            for node in ast.parse((ROOT / "scripts/standalone.py").read_text()).body
            if isinstance(node, ast.Assign)
            for target in node.targets
            if isinstance(target, ast.Name)
            and target.id in ("LINEAR_JOG_SPEED", "ANGULAR_JOG_SPEED")
        }
        self.assertEqual(assignments, {"LINEAR_JOG_SPEED": 0.75, "ANGULAR_JOG_SPEED": 0.5})

    def test_both_robot_profiles_use_the_requested_limits(self):
        profiles = ROOT / "ros2_ws/src/slam_localization_3d/config/robots"
        for robot_type in ("nova_carter", "carter_v1"):
            with self.subTest(robot_type=robot_type):
                overrides = yaml.safe_load((profiles / f"{robot_type}.yaml").read_text())[
                    "parameter_overrides"]
                parameters = overrides["cmd_vel_safety"]["ros__parameters"]
                self.assertEqual(parameters["max_linear_speed"], 0.75)
                self.assertEqual(parameters["max_angular_speed"], 0.5)
                smoother = overrides["velocity_smoother"]["ros__parameters"]
                self.assertEqual(smoother["max_velocity"], [0.75, 0.0, 0.5])
                self.assertEqual(smoother["min_velocity"], [-0.75, 0.0, -0.5])
                controller = overrides["controller_server"]["ros__parameters"]["FollowPath"]
                self.assertEqual(controller["desired_linear_vel"], 0.75)
                self.assertEqual(controller["rotate_to_heading_angular_vel"], 0.5)

    def test_invalid_ros_message_stops_instead_of_reusing_velocity(self):
        for linear, angular in (
            ((0.751, 0, 0), (0, 0, 0)),
            ((-0.751, 0, 0), (0, 0, 0)),
            ((0, 0, 0), (0, 0, 0.501)),
            ((0, 0, 0), (0, 0, -0.501)),
            ((float("nan"), 0, 0), (0, 0, 0)),
            ((0.1, 0.1, 0), (0, 0, 0)),
        ):
            with self.subTest(linear=linear, angular=angular):
                self.receiver.accept_twist((0.1, 0, 0), (0, 0, 0))
                with self.assertRaises(ValueError):
                    self.receiver.accept_twist(linear, angular)
                self.assertEqual(self.receiver.command(), (0.0, 0.0))


class TestCmdVelSafety(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def test_ros_safety_publishes_bounded_twist_and_stops_on_bad_input(self):
        node = CmdVelSafety()
        listener = Node("cmd_vel_safety_test_listener")
        received = []
        listener.create_subscription(Twist, "/cmd_vel", received.append, 10)

        def check(command, expected, refresh_correction=True):
            deadline = time.monotonic() + 2
            while node.publisher.get_subscription_count() == 0 and time.monotonic() < deadline:
                time.sleep(0.02)
            self.assertGreater(node.publisher.get_subscription_count(), 0)
            received.clear()
            deadline = time.monotonic() + 2
            while time.monotonic() < deadline:
                stamp = node.get_clock().now().to_msg()
                scan = LaserScan()
                scan.header = Header(frame_id="base_link", stamp=stamp)
                node.on_sensor("scan", scan)
                if refresh_correction:
                    node.on_correction(Header(frame_id="map", stamp=stamp))
                node.on_command(command)
                rclpy.spin_once(listener, timeout_sec=0.02)
                if received and (received[-1].linear.x, received[-1].angular.z) == expected:
                    return
                time.sleep(0.02)
            self.fail(f"Did not reach bounded output {expected}: {received[-1:]}")

        try:
            node.last_correction = node.get_clock().now().nanoseconds * 1e-9
            valid = Twist()
            valid.linear.x = 0.1
            valid.angular.z = 0.2
            check(valid, (0.1, 0.2))
            fast = Twist()
            fast.linear.x = 1.0
            fast.angular.z = -1.2
            check(fast, (0.75, -0.5))
            fast.linear.x = -1.0
            check(fast, (-0.75, -0.5))
            invalid = Twist()
            invalid.linear.y = 1.0
            check(invalid, (0.0, 0.0))
            node.last_correction -= 5
            check(valid, (0.0, 0.0), refresh_correction=False)
            node.last_correction = node.get_clock().now().nanoseconds * 1e-9
            node.on_stop(Bool(data=True))
            check(valid, (0.0, 0.0))
        finally:
            listener.destroy_node()
            node.destroy_node()


if __name__ == "__main__":
    unittest.main()
