"""ROS integration regressions; run inside the ROS container after building slam_nav."""

import subprocess
import time
import unittest

from ament_index_python.packages import get_package_prefix
import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header


class GroundObstacleFilterTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def check_cloud(self, configured_z, plane_z, expect_output, explicit=True):
        node = rclpy.create_node("ground_filter_probe")
        outputs, processed = [], []
        subscriptions = [
            node.create_subscription(
                PointCloud2, "/ground_test/obstacles", outputs.append, qos_profile_sensor_data
            ),
            node.create_subscription(
                PointCloud2, "/ground_test/self_filtered", processed.append,
                qos_profile_sensor_data,
            ),
        ]
        publisher = node.create_publisher(
            PointCloud2, "/ground_test/input", qos_profile_sensor_data
        )
        command = [
            f"{get_package_prefix('slam_nav')}/lib/slam_nav/ground_obstacle_filter",
            "--ros-args",
            "-p", "input_topic:=/ground_test/input",
            "-p", "output_topic:=/ground_test/obstacles",
            "-p", "self_filtered_topic:=/ground_test/self_filtered",
            "-p", "ground_search_height:=0.35",
        ]
        if explicit:
            command += ["-p", f"ground_z:={configured_z}"]
        process = subprocess.Popen(
            command, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True
        )
        try:
            deadline = time.monotonic() + 10
            while publisher.get_subscription_count() == 0 and time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=0.05)
                self.assertIsNone(process.poll(), "ground filter exited before discovery")
            self.assertGreater(publisher.get_subscription_count(), 0)
            # Dense ground plus a 30 cm obstacle and a 2 cm return that must be removed.
            ground = [
                (1 + x * 0.1, -2 + y * 0.1, plane_z)
                for x in range(40) for y in range(40)
            ]
            obstacle = [
                (2 + x * 0.1, 0.5 + y * 0.1, plane_z + height)
                for x in range(8) for y in range(8) for height in (0.02, 0.3)
            ]
            deadline = time.monotonic() + 2
            while time.monotonic() < deadline:
                header = Header(frame_id="base_link", stamp=node.get_clock().now().to_msg())
                publisher.publish(point_cloud2.create_cloud_xyz32(header, ground + obstacle))
                rclpy.spin_once(node, timeout_sec=0.05)
            self.assertTrue(processed, "input cloud was not processed")
            if expect_output:
                self.assertTrue(outputs, "valid ground plane was rejected")
                points = list(point_cloud2.read_points(
                    outputs[-1], field_names=("x", "y", "z"), skip_nans=True
                ))
                self.assertGreater(len(points), 0)
                self.assertEqual(outputs[-1].header.frame_id, "base_link")
                for point in points:
                    self.assertAlmostEqual(float(point[2]), plane_z + 0.3, places=5)
            else:
                self.assertFalse(outputs, "a plane at the wrong ground height was accepted")
        finally:
            process.terminate()
            try:
                process.communicate(timeout=5)
            except subprocess.TimeoutExpired:
                process.kill()
                process.communicate()
            for subscription in subscriptions:
                node.destroy_subscription(subscription)
            node.destroy_node()

    def test_default_preserves_zero_height_ground(self):
        self.check_cloud(0.0, 0.0, True, explicit=False)

    def test_axle_frame_ground_is_accepted(self):
        self.check_cloud(-0.24, -0.24, True)

    def test_search_is_centered_on_configured_height(self):
        self.check_cloud(-0.6, -0.6, True)

    def test_wrong_ground_height_is_rejected(self):
        self.check_cloud(-0.24, -0.10, False)


if __name__ == "__main__":
    unittest.main()
