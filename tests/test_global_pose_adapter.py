import math
import sys
import unittest
from pathlib import Path
from unittest.mock import Mock

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2

SCRIPTS = (
    Path(__file__).resolve().parents[1]
    / "ros2_ws/src/isaac_localization_3d/scripts"
)
sys.path.insert(0, str(SCRIPTS))
from global_pose_adapter import GlobalPoseAdapter
from localization_3d_pose import BODY_TO_BASE


def stamp(message, time):
    message.header.stamp.sec = int(time)
    message.header.stamp.nanosec = round((time - int(time)) * 1_000_000_000)
    return message


def odometry(parent, child, time, x=0.0, yaw=0.0):
    message = stamp(Odometry(), time)
    message.header.frame_id = parent
    message.child_frame_id = child
    message.pose.pose.position.x = float(x)
    message.pose.pose.orientation.z = math.sin(yaw / 2)
    message.pose.pose.orientation.w = math.cos(yaw / 2)
    return message


def scan(time, frame="camera_init"):
    message = stamp(PointCloud2(), time)
    message.header.frame_id = frame
    return message


class TestGlobalPoseAdapter(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = GlobalPoseAdapter()
        self.clock = Mock()
        self.node.get_clock = Mock(return_value=self.clock)
        self.node.pose_publisher.publish = Mock()
        self.node.accepted_publisher.publish = Mock()
        self.node.initial_pose_publisher.publish = Mock()
        self.at(10)

    def tearDown(self):
        self.node.destroy_node()

    def at(self, timestamp):
        self.clock.now.return_value.nanoseconds = round(timestamp * 1_000_000_000)

    def input(self, time=10, x=0):
        self.at(time)
        self.node.on_odometry(odometry("camera_init", "body", time, x))
        self.node.on_scan(scan(time))

    def correction(self, time=10, x=0, yaw=0):
        self.at(time)
        self.node.on_correction(odometry("map", "", time, x, yaw))

    def assert_count(self, count):
        self.assertEqual(self.node.pose_publisher.publish.call_count, count)
        self.assertEqual(self.node.accepted_publisher.publish.call_count, count)

    def test_publish_planar_composed_pose_covariance_and_matching_marker(self):
        self.input()
        self.correction(x=2, yaw=math.pi / 2)
        self.assert_count(1)
        pose = self.node.pose_publisher.publish.call_args.args[0]
        self.assertEqual(pose.header.frame_id, "map")
        self.assertEqual(pose.header.stamp.sec, 10)
        self.assertAlmostEqual(pose.pose.pose.position.x, 2 - BODY_TO_BASE[0][1])
        self.assertAlmostEqual(pose.pose.pose.position.y, BODY_TO_BASE[0][0])
        self.assertEqual(pose.pose.pose.position.z, 0)
        self.assertAlmostEqual(pose.pose.pose.orientation.z, -math.sqrt(0.5))
        self.assertAlmostEqual(pose.pose.pose.orientation.w, math.sqrt(0.5))
        self.assertEqual(
            [pose.pose.covariance[i] for i in (0, 7, 14, 21, 28, 35)],
            [0.25, 0.25, 1_000_000, 1_000_000, 1_000_000, 0.09],
        )
        marker = self.node.accepted_publisher.publish.call_args.args[0]
        self.assertEqual((marker.frame_id, marker.stamp.sec), ("map", 10))

    def test_validity_and_freshness_reject_without_fake_success(self):
        self.correction()
        self.assert_count(0)
        self.input()
        for bad in (
            odometry("odom", "", 10),
            odometry("map", "wrong", 10),
            odometry("map", "", 0),
        ):
            self.node.on_correction(bad)
        invalid = odometry("map", "", 10)
        invalid.pose.pose.orientation.w = float("nan")
        self.node.on_correction(invalid)
        self.assert_count(0)
        self.at(12)
        self.node.on_correction(odometry("map", "", 10))
        self.assert_count(0)
        self.input(12)
        self.node.on_correction(odometry("map", "", 12))
        self.assert_count(1)
        self.node.on_correction(odometry("map", "", 12))
        self.assert_count(1)

    def test_configurable_planar_covariance(self):
        self.node.covariance_xy = 0.4
        self.node.covariance_yaw = 0.16
        self.input()
        self.correction()
        covariance = self.node.pose_publisher.publish.call_args.args[0].pose.covariance
        self.assertEqual((covariance[0], covariance[7], covariance[35]), (0.4, 0.4, 0.16))

    def test_registration_covariance_propagates_to_base_pose(self):
        registration = [0.0] * 36
        for index, value in zip((0, 7, 14, 21, 28, 35), (0.01, 0.02, 0.5, 1e-4, 1e-4, 4e-4)):
            registration[index] = value
        registration[1] = registration[6] = 0.005
        self.node.registration_covariance_scale = 2.0
        self.input()
        self.at(10)
        message = odometry("map", "", 10, x=2, yaw=math.pi / 2)
        message.pose.covariance = registration
        self.node.on_correction(message)
        self.assert_count(1)
        pose = self.node.pose_publisher.publish.call_args.args[0]
        covariance = pose.pose.covariance
        x, y = pose.pose.pose.position.x, pose.pose.pose.position.y
        z = -0.526
        # Yaw error at the base position adds lever-arm variance to x/y.
        self.assertAlmostEqual(
            covariance[0], 2 * (0.01 + z * z * 1e-4 + y * y * 4e-4 + 1e-4)
        )
        self.assertAlmostEqual(
            covariance[7], 2 * (0.02 + z * z * 1e-4 + x * x * 4e-4 + 1e-4)
        )
        self.assertAlmostEqual(covariance[35], 2 * (4e-4 + 1e-5))
        self.assertAlmostEqual(covariance[1], 2 * (0.005 - x * y * 4e-4))
        self.assertEqual(covariance[1], covariance[6])
        self.assertAlmostEqual(covariance[5], 2 * -y * 4e-4)
        self.assertEqual(covariance[5], covariance[30])
        self.assertEqual([covariance[i] for i in (14, 21, 28)], [1_000_000] * 3)

    def test_pose_carries_composed_lio_stamp(self):
        self.at(9.9)
        self.node.on_odometry(odometry("camera_init", "body", 9.9))
        self.node.on_scan(scan(9.9))
        self.correction(10)
        self.assert_count(1)
        pose = self.node.pose_publisher.publish.call_args.args[0]
        self.assertEqual((pose.header.stamp.sec, pose.header.stamp.nanosec), (9, 900_000_000))
        marker = self.node.accepted_publisher.publish.call_args.args[0]
        self.assertEqual((marker.stamp.sec, marker.stamp.nanosec), (10, 0))

    def test_invalid_registration_covariance_is_rejected(self):
        self.input()
        self.at(10)
        message = odometry("map", "", 10)
        message.pose.covariance[0] = -1.0
        self.node.on_correction(message)
        self.assert_count(0)

    def test_scan_frame_alignment_and_new_scan_required(self):
        self.at(10)
        self.node.on_odometry(odometry("camera_init", "body", 10))
        self.node.on_scan(scan(10, "map"))
        self.correction()
        self.assert_count(0)
        self.node.on_scan(scan(10))
        self.correction()
        self.assert_count(1)
        self.at(11)
        self.node.on_odometry(odometry("camera_init", "body", 11))
        self.correction(11)
        self.assert_count(1)
        self.node.on_scan(scan(11))
        self.correction(11)
        self.assert_count(2)
        self.at(20)
        self.node.on_scan(scan(10.5))
        self.node.on_odometry(odometry("camera_init", "body", 20))
        self.correction(20)
        self.assert_count(2)

    def test_scan_must_be_newer_than_previous_correction_not_just_previous_scan(self):
        self.input(10)
        self.at(10.2)
        self.node.on_correction(odometry("map", "", 10.2))
        self.assert_count(1)
        self.at(10.4)
        self.node.on_scan(scan(10.1))
        self.node.on_odometry(odometry("camera_init", "body", 10.4))
        self.node.on_correction(odometry("map", "", 10.4))
        self.assert_count(1)
        self.node.on_scan(scan(10.4))
        self.node.on_correction(odometry("map", "", 10.4))
        self.assert_count(2)

    def test_invalid_lio_and_out_of_order_packets_do_not_clear_baseline(self):
        self.input(10)
        self.correction(10)
        self.at(11)
        bad = odometry("camera_init", "base_link", 11)
        self.node.on_odometry(bad)
        self.node.on_odometry(odometry("camera_init", "body", 11))
        self.node.on_odometry(odometry("camera_init", "body", 10.5))
        self.node.on_scan(scan(11))
        self.correction(11, x=5)
        self.assert_count(1)

    def test_jump_compensates_lio_motion_and_manual_init_resets_baseline(self):
        self.input()
        self.correction()
        self.input(11, x=5)
        self.correction(11, x=0.5)
        self.assert_count(2)
        self.input(12, x=5)
        self.correction(12, x=4)
        self.assert_count(2)
        self.input(13, x=5)
        self.correction(13, yaw=1)
        self.assert_count(2)
        manual = stamp(PoseWithCovarianceStamped(), 13)
        manual.header.frame_id = "map"
        manual.pose.pose.orientation.w = 1.0
        self.node.on_initial_pose(manual)
        self.input(14, x=5)
        self.correction(14, x=4)
        self.assert_count(3)
        self.node.on_correction(odometry("map", "", 14))
        self.assert_count(3)

    def test_auto_init_only_with_upstream_and_not_after_manual_pose(self):
        self.node.auto_initial_pose = True
        self.node.initial_pose_publisher.get_subscription_count = Mock(return_value=1)
        self.node.get_subscriptions_info_by_topic = Mock(
            return_value=[Mock(node_name="global_localization")]
        )
        self.input()
        self.node.initial_pose_publisher.publish.assert_not_called()
        self.node.initial_pose_publisher.get_subscription_count.return_value = 2
        self.at(11)
        self.node.get_subscriptions_info_by_topic.return_value = [Mock(node_name="rviz2")]
        self.node.on_odometry(odometry("camera_init", "body", 11))
        self.node.initial_pose_publisher.publish.assert_not_called()
        self.node.get_subscriptions_info_by_topic.return_value = [
            Mock(node_name="global_localization")
        ]
        self.node.on_odometry(odometry("camera_init", "body", 11))
        self.node.initial_pose_publisher.publish.assert_called_once()
        initial = self.node.initial_pose_publisher.publish.call_args.args[0]
        self.assertEqual(initial.header.frame_id, "map")
        self.assertEqual(initial.header.stamp.sec, 11)
        self.assertEqual(initial.pose.pose.orientation.w, 1)
        self.node.on_odometry(odometry("camera_init", "body", 11))
        self.node.initial_pose_publisher.publish.assert_called_once()

        self.node.initial_pose_received = False
        manual = stamp(PoseWithCovarianceStamped(), 11)
        manual.header.frame_id = "map"
        manual.pose.pose.orientation.w = 1.0
        self.node.on_initial_pose(manual)
        self.at(12)
        self.node.on_odometry(odometry("camera_init", "body", 12))
        self.node.initial_pose_publisher.publish.assert_called_once()

    def test_auto_init_sends_configured_spawn_pose(self):
        self.node.auto_initial_pose = True
        self.node.initial_pose = (3.0, -2.0, 0.1, math.pi / 2)
        self.node.initial_pose_publisher.get_subscription_count = Mock(return_value=2)
        self.node.get_subscriptions_info_by_topic = Mock(
            return_value=[Mock(node_name="global_localization")]
        )
        self.input()
        self.node.get_subscriptions_info_by_topic.assert_called_with("/initialpose")
        initial = self.node.initial_pose_publisher.publish.call_args.args[0]
        self.assertEqual(initial.pose.pose.position.x, 3.0)
        self.assertEqual(initial.pose.pose.position.y, -2.0)
        self.assertEqual(initial.pose.pose.position.z, 0.1)
        self.assertAlmostEqual(initial.pose.pose.orientation.z, math.sin(math.pi / 4))
        self.assertAlmostEqual(initial.pose.pose.orientation.w, math.cos(math.pi / 4))

    def test_clock_reset_discards_old_baseline_and_requires_fresh_inputs(self):
        self.input()
        self.correction()
        self.at(1)
        self.node.on_odometry(odometry("camera_init", "body", 1))
        self.correction(1)
        self.assert_count(1)
        self.node.on_scan(scan(1))
        self.correction(1)
        self.assert_count(2)


if __name__ == "__main__":
    unittest.main()
