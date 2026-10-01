import sys
import time
import unittest
from pathlib import Path
from unittest.mock import Mock, patch

import rclpy
from geometry_msgs.msg import Twist
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import LaserScan, PointCloud2
from std_msgs.msg import Header

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))
from cmd_vel_safety import CmdVelSafety


class SafetyTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = CmdVelSafety()
        self.publisher = Mock()
        self.node.publisher = self.publisher
        self.now = 10.0
        self.clock = Mock()
        self.clock.now.side_effect = lambda: Time(seconds=self.now)
        self.clock_patch = patch.object(self.node, "get_clock", return_value=self.clock)
        self.clock_patch.start()
        self.node.on_correction(self.header(10))
        self.command = Twist()
        self.command.linear.x = 0.5
        self.command.angular.z = 0.2

    def tearDown(self):
        self.clock_patch.stop()
        self.node.destroy_node()

    @staticmethod
    def header(stamp):
        header = Header(frame_id="map")
        header.stamp = Time(seconds=stamp).to_msg()
        return header

    def sensor(self, source, stamp):
        message = LaserScan() if source == "scan" else PointCloud2()
        message.header = self.header(stamp)
        self.node.on_sensor(source, message)

    def assert_output(self, linear, angular=0.0):
        output = self.publisher.publish.call_args.args[0]
        self.assertEqual(output.linear.x, linear)
        self.assertEqual(output.angular.z, angular)

    def test_startup_blocks_commands_without_sensor_data(self):
        self.node.on_command(self.command)
        self.assert_output(0)

    def test_either_source_allows_commands(self):
        for source in ("scan", "obstacles"):
            with self.subTest(source=source):
                self.node.sensor_stamps = dict.fromkeys(self.node.sensor_stamps)
                self.sensor(source, 9.9)
                self.node.on_command(self.command)
                self.assert_output(0.5, 0.2)

    def test_expiry_boundary_and_delayed_messages(self):
        self.sensor("scan", 9.01)
        self.node.on_command(self.command)
        self.assert_output(0.5, 0.2)
        self.now = 10.01
        self.node.on_command(self.command)
        self.assert_output(0)
        self.sensor("obstacles", 9.0)
        self.assertIsNone(self.node.sensor_stamps["obstacles"])
        self.node.on_watchdog()
        self.assert_output(0)

    def test_one_stale_source_does_not_block_fresh_source(self):
        self.sensor("scan", 10)
        self.now = 11.1
        self.sensor("obstacles", 11.1)
        self.node.on_command(self.command)
        self.assert_output(0.5, 0.2)

    def test_timer_stops_without_new_command_and_does_not_replay(self):
        self.sensor("scan", 10)
        self.node.on_command(self.command)
        self.now = 11
        self.publisher.reset_mock()
        executor = SingleThreadedExecutor()
        executor.add_node(self.node)
        try:
            executor.spin_once(timeout_sec=0.3)
        finally:
            executor.remove_node(self.node)
            executor.shutdown()
        self.assert_output(0)
        self.publisher.reset_mock()
        self.sensor("scan", 11)
        self.node.on_watchdog()
        self.publisher.publish.assert_not_called()
        self.node.on_command(self.command)
        self.assert_output(0.5, 0.2)

    def test_future_and_invalid_headers_are_rejected(self):
        self.sensor("scan", 11)
        message = PointCloud2()
        message.header.stamp = Time(seconds=10).to_msg()
        self.node.on_sensor("obstacles", message)
        self.assertEqual(self.node.sensor_stamps, {"scan": None, "obstacles": None})
        self.node.on_command(self.command)
        self.assert_output(0)

    def test_clock_reset_clears_old_freshness_state(self):
        self.sensor("scan", 10)
        self.now = 5
        self.node.on_watchdog()
        self.assert_output(0)
        self.assertIsNone(self.node.last_correction)
        self.sensor("scan", 5)
        self.node.on_command(self.command)
        self.assert_output(0)
        self.node.on_correction(self.header(5))
        self.node.on_command(self.command)
        self.assert_output(0.5, 0.2)
        self.now = 10
        self.node.on_command(self.command)
        self.assert_output(0)

    def test_existing_safety_checks_remain_active(self):
        self.sensor("scan", 10)
        self.command.linear.x = 2.0
        self.command.angular.z = 2.0
        self.node.on_command(self.command)
        self.assert_output(0.75, 0.7)
        self.command.linear.y = 0.1
        self.node.on_command(self.command)
        self.assert_output(0)
        self.command.linear.y = 0.0
        self.node.last_correction = 6
        self.node.on_command(self.command)
        self.assert_output(0)
        self.node.last_correction = 10
        self.node.stop_latched = True
        self.node.on_command(self.command)
        self.assert_output(0)

    def test_best_effort_sensor_delivery(self):
        feeder = Node("safety_test_sensor")
        publisher = feeder.create_publisher(LaserScan, "/scan", qos_profile_sensor_data)
        executor = SingleThreadedExecutor()
        executor.add_node(feeder)
        executor.add_node(self.node)
        try:
            deadline = time.monotonic() + 3
            while self.node.sensor_stamps["scan"] is None and time.monotonic() < deadline:
                message = LaserScan()
                message.header = self.header(10)
                publisher.publish(message)
                executor.spin_once(timeout_sec=0.05)
            self.assertEqual(self.node.sensor_stamps["scan"], 10)
            self.node.on_command(self.command)
            self.assert_output(0.5, 0.2)
        finally:
            executor.remove_node(feeder)
            executor.remove_node(self.node)
            executor.shutdown()
            feeder.destroy_node()

    def test_invalid_timeout_is_rejected(self):
        for value in (0.0, -1.0, float("nan"), float("inf")):
            with self.subTest(value=value):
                with self.assertRaises(ValueError):
                    CmdVelSafety(
                        enable_rosout=False,
                        parameter_overrides=[Parameter("sensor_timeout", value=value)],
                    )


class SafetyTopicTest(unittest.TestCase):
    def test_output_stops_on_sensor_loss_and_recovers_only_with_new_command(self):
        rclpy.init()
        safety = CmdVelSafety(parameter_overrides=[Parameter("sensor_timeout", value=0.3)])
        probe = Node("safety_topic_test")
        outputs = []
        probe.create_subscription(Twist, "/cmd_vel", outputs.append, 10)
        scan_pub = probe.create_publisher(LaserScan, "/scan", qos_profile_sensor_data)
        correction_pub = probe.create_publisher(Header, "/localization_3d/accepted_correction", 10)
        command_pub = probe.create_publisher(Twist, "/nav2/cmd_vel", 10)
        executor = SingleThreadedExecutor()
        executor.add_node(safety)
        executor.add_node(probe)
        command = Twist()
        command.linear.x = 0.4

        def spin(duration):
            deadline = time.monotonic() + duration
            while time.monotonic() < deadline:
                executor.spin_once(timeout_sec=0.01)

        def send_sensors():
            correction = Header(frame_id="map", stamp=probe.get_clock().now().to_msg())
            scan = LaserScan()
            scan.header = Header(frame_id="base_link", stamp=correction.stamp)
            correction_pub.publish(correction)
            scan_pub.publish(scan)

        try:
            deadline = time.monotonic() + 3
            while time.monotonic() < deadline:
                send_sensors()
                spin(0.03)
                command_pub.publish(command)
                spin(0.03)
                if outputs and outputs[-1].linear.x == 0.4:
                    break
            self.assertTrue(outputs)
            self.assertEqual(outputs[-1].linear.x, 0.4)

            # The controller keeps requesting motion while sensor updates stop.
            deadline = time.monotonic() + 0.5
            while time.monotonic() < deadline:
                command_pub.publish(command)
                spin(0.03)
            self.assertEqual(outputs[-1].linear.x, 0.0)
            self.assertEqual(outputs[-1].angular.z, 0.0)

            # With no commands either, the timer still actively publishes zero.
            outputs.clear()
            spin(0.25)
            self.assertTrue(outputs)
            self.assertTrue(all(msg == Twist() for msg in outputs))

            # Sensor recovery alone must not replay the previous motion command.
            outputs.clear()
            send_sensors()
            spin(0.05)
            self.assertTrue(all(msg == Twist() for msg in outputs))
            command_pub.publish(command)
            spin(0.05)
            self.assertEqual(outputs[-1].linear.x, 0.4)
        finally:
            executor.shutdown()
            safety.destroy_node()
            probe.destroy_node()
            rclpy.shutdown()
