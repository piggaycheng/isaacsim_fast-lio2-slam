import sys
import time
import unittest
from pathlib import Path
from unittest.mock import patch

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, Header

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "ros2_ws/src/isaac_localization_3d/scripts"))
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

    def test_075_meter_per_second_limit_is_inclusive(self):
        self.receiver.accept_twist((0.75, 0, 0), (0, 0, 0))
        self.assertEqual(self.receiver.command(), (0.75, 0.0))
        self.receiver.accept_twist((-0.75, 0, 0), (0, 0, 0))
        self.assertEqual(self.receiver.command(), (-0.75, 0.0))

    def test_invalid_ros_message_stops_instead_of_reusing_velocity(self):
        for linear, angular in (
            ((0.751, 0, 0), (0, 0, 0)),
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
            check(fast, (0.75, -0.7))
            fast.linear.x = -1.0
            check(fast, (-0.75, -0.7))
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
