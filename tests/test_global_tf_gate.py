import math
import sys
import unittest
from pathlib import Path
from unittest.mock import Mock

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Header

sys.path.insert(
    0, str(Path(__file__).resolve().parents[1] / "ros2_ws/src/isaac_localization_3d/scripts")
)
from global_tf_gate import GlobalTfGate, map_to_odom


def odometry(stamp):
    message = Odometry()
    message.header.frame_id = "map"
    message.header.stamp.sec = stamp
    message.child_frame_id = "base_link"
    message.pose.pose.position.x = 5.0
    message.pose.pose.orientation.w = 1.0
    return message


class TestGlobalTfGate(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = GlobalTfGate()
        self.node.broadcaster.sendTransform = Mock()
        self.node.get_clock().now = Mock(return_value=Mock(to_msg=Mock(
            return_value=odometry(10).header.stamp
        )))
        local = TransformStamped()
        local.transform.translation.x = 2.0
        local.transform.rotation.w = 1.0
        self.node.tf_buffer.lookup_transform = Mock(return_value=local)

    def tearDown(self):
        self.node.destroy_node()

    def test_composes_map_to_odom_instead_of_publishing_second_base_parent(self):
        header = Header()
        header.frame_id = "map"
        header.stamp.sec = 9
        self.node.on_correction(header)
        self.node.on_global_odometry(odometry(10))
        transform = self.node.broadcaster.sendTransform.call_args.args[0]
        self.assertEqual((transform.header.frame_id, transform.child_frame_id),
                         ("map", "odom"))
        self.assertAlmostEqual(transform.transform.translation.x, 3.0)

    def test_stale_or_not_yet_corrected_global_output_does_not_publish_tf(self):
        self.node.on_global_odometry(odometry(10))
        self.node.broadcaster.sendTransform.assert_not_called()
        header = Header()
        header.frame_id = "map"
        header.stamp.sec = 11
        self.node.on_correction(header)
        self.node.on_global_odometry(odometry(10))
        self.node.broadcaster.sendTransform.assert_not_called()
        self.node.get_clock().now.return_value.to_msg.return_value.sec = 20
        self.node.on_global_odometry(odometry(20))
        self.node.broadcaster.sendTransform.assert_not_called()

    def test_rotation_and_translation_compose(self):
        global_pose = odometry(10).pose.pose
        global_pose.position.x = 3.0
        global_pose.position.y = 4.0
        global_pose.orientation.z = math.sqrt(0.5)
        global_pose.orientation.w = math.sqrt(0.5)
        local = TransformStamped().transform
        local.rotation.w = 1.0
        local.translation.x = 2.0
        self.assertAlmostEqual(map_to_odom(global_pose, local)[0], 3.0)
        self.assertAlmostEqual(map_to_odom(global_pose, local)[1], 2.0)


if __name__ == "__main__":
    unittest.main()
