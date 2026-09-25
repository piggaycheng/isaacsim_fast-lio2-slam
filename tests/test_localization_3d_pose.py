import math
import unittest
from unittest.mock import Mock

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry

from ros2_ws.src.isaac_localization_3d.scripts.localization_3d_pose import (
    LocalizationVisualization,
    compose_pose,
)


def odometry(parent, child, timestamp):
    message = Odometry()
    message.header.frame_id = parent
    message.header.stamp.sec = timestamp
    message.child_frame_id = child
    message.pose.pose.orientation.w = 1.0
    return message


class TestLocalizationVisualization(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = LocalizationVisualization()
        self.node.tf_broadcaster.sendTransform = Mock()
        self.node.pose_publisher.publish = Mock()
        self.node.marker_publisher.publish = Mock()
        self.node.odom_publisher.publish = Mock()
        self.node.path_publisher.publish = Mock()
        self.node.initial_pose_publisher.publish = Mock()

    def tearDown(self):
        self.node.destroy_node()

    def test_compose_rotates_body_offset(self):
        first = ((1.0, 2.0, 3.0), (0.0, 0.0, math.sqrt(0.5), math.sqrt(0.5)))
        translation, rotation = compose_pose(
            first, ((0.2317, 0.0, -0.526), (0.0, 0.0, 1.0, 0.0))
        )
        self.assertAlmostEqual(translation[0], 1)
        self.assertAlmostEqual(translation[1], 2.2317)
        self.assertAlmostEqual(translation[2], 2.474)
        self.assertAlmostEqual(sum(value**2 for value in rotation), 1)

    def test_auto_initial_pose_waits_for_upstream_and_does_not_override_manual_pose(self):
        self.node.auto_initial_pose = True
        self.node.initial_pose_publisher.get_subscription_count = Mock(return_value=1)
        self.node.on_odometry(odometry("camera_init", "body", 10))
        self.node.initial_pose_publisher.publish.assert_not_called()

        self.node.initial_pose_publisher.get_subscription_count.return_value = 2
        self.node.on_odometry(odometry("camera_init", "body", 11))
        initial_pose = self.node.initial_pose_publisher.publish.call_args.args[0]
        self.assertEqual(initial_pose.header.frame_id, "map")
        self.assertEqual(initial_pose.header.stamp.sec, 11)
        self.assertEqual(initial_pose.pose.pose.orientation.w, 1.0)
        self.node.on_odometry(odometry("camera_init", "body", 12))
        self.node.initial_pose_publisher.publish.assert_called_once()

        self.node.initial_pose_received = False
        manual = PoseWithCovarianceStamped()
        manual.header.frame_id = "map"
        self.node.on_initial_pose(manual)
        self.node.on_odometry(odometry("camera_init", "body", 13))
        self.node.initial_pose_publisher.publish.assert_called_once()

    def test_requires_fresh_registration_before_visualizing(self):
        self.node.on_odometry(odometry("camera_init", "body", 10))
        self.node.tf_broadcaster.sendTransform.assert_not_called()
        self.node.pose_publisher.publish.assert_not_called()
        self.node.marker_publisher.publish.assert_not_called()

        correction = odometry("map", "", 11)
        correction.pose.pose.position.x = 1.0
        self.node.on_correction(correction)
        self.node.on_odometry(odometry("camera_init", "body", 12))

        transform = self.node.tf_broadcaster.sendTransform.call_args.args[0]
        self.assertEqual((transform.header.frame_id, transform.child_frame_id),
                         ("map", "camera_init"))
        self.assertEqual(transform.header.stamp.sec, 12)
        self.assertEqual(transform.transform.translation.x, 1.0)
        base_odom = self.node.odom_publisher.publish.call_args.args[0]
        self.assertAlmostEqual(base_odom.pose.pose.position.x, 1.2317)
        self.assertAlmostEqual(base_odom.pose.pose.position.z, -0.526)
        projected_pose = self.node.pose_publisher.publish.call_args.args[0]
        self.assertEqual(projected_pose.header.frame_id, "map")
        self.assertEqual(projected_pose.pose.position.z, 0)
        self.assertAlmostEqual(projected_pose.pose.position.x, 1.2317)
        marker = self.node.marker_publisher.publish.call_args.args[0]
        self.assertEqual(marker.lifetime.sec, 2)
        self.assertEqual(marker.header.frame_id, "map")
        self.assertEqual(len(self.node.path.poses), 1)

        self.node.on_odometry(odometry("camera_init", "body", 20))
        self.node.tf_broadcaster.sendTransform.assert_called_once()
        self.node.pose_publisher.publish.assert_called_once()
        self.node.marker_publisher.publish.assert_called_once()

    def test_rejects_bad_frame_and_clock_reset(self):
        bad_correction = odometry("odom", "", 11)
        self.node.on_correction(bad_correction)
        self.assertIsNone(self.node.correction)

        self.node.on_correction(odometry("map", "", 11))
        self.node.on_odometry(odometry("camera_init", "body", 12))
        self.node.on_odometry(odometry("camera_init", "body", 1))
        self.assertIsNone(self.node.correction)
        self.assertEqual(len(self.node.path.poses), 0)


if __name__ == "__main__":
    unittest.main()
