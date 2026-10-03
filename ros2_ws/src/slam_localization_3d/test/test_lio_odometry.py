import math
import sys
import unittest
from pathlib import Path

import rclpy
from nav_msgs.msg import Odometry
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))
import lio_odometry  # noqa: E402
from localization_3d_pose import compose_pose  # noqa: E402


def odometry(sec, x, y, yaw, z=0.526):
    message = Odometry()
    message.header.stamp.sec = sec
    message.header.frame_id = "camera_init"
    message.child_frame_id = "body"
    pose = message.pose.pose
    pose.position.x, pose.position.y, pose.position.z = x, y, z
    pose.orientation.z, pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
    return message


def yaw_of(pose):
    q = pose.orientation
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y ** 2 + q.z ** 2))


class InversePoseTest(unittest.TestCase):
    def test_inverse_composes_to_identity(self):
        half = 0.4
        pose = ((1.0, -2.0, 0.5), (0.1, -0.2, math.sin(half), math.cos(half)))
        norm = math.sqrt(sum(value ** 2 for value in pose[1]))
        pose = (pose[0], tuple(value / norm for value in pose[1]))
        translation, quaternion = compose_pose(lio_odometry.inverse_pose(pose), pose)
        for value in translation:
            self.assertAlmostEqual(value, 0.0, places=9)
        self.assertAlmostEqual(abs(quaternion[3]), 1.0, places=9)


class LioOdometryNodeTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        # base_link -> body 0.5 m ahead and 0.5 m up: a pure body rotation moves base_link.
        self.node = lio_odometry.LioOdometry(parameter_overrides=[
            Parameter("imu_mount", value=[0.5, 0.0, 0.5, 0.0, 0.0, 0.0]),
            Parameter("position_variance", value=0.04),
            Parameter("yaw_variance", value=0.01),
        ])
        self.received = []
        self.listener = Node("lio_odometry_listener")
        self.listener.create_subscription(Odometry, "lio/odom", self.received.append, 10)
        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.node)
        self.executor.add_node(self.listener)

    def tearDown(self):
        self.executor.shutdown()
        self.node.destroy_node()
        self.listener.destroy_node()

    def publish(self, message):
        count = len(self.received)
        self.node.on_odometry(message)
        for _ in range(50):
            self.executor.spin_once(timeout_sec=0.02)
            if len(self.received) > count:
                return self.received[-1]
        self.fail("lio/odom not published")

    def test_first_pose_is_origin_and_motion_is_in_base_link(self):
        first = self.publish(odometry(1, 3.0, 4.0, 0.3))
        self.assertEqual((first.header.frame_id, first.child_frame_id), ("odom", "base_link"))
        self.assertAlmostEqual(first.pose.pose.position.x, 0.0, places=9)
        self.assertAlmostEqual(first.pose.pose.position.y, 0.0, places=9)
        self.assertAlmostEqual(yaw_of(first.pose.pose), 0.0, places=9)
        self.assertEqual(first.pose.covariance[0], 0.04)
        self.assertEqual(first.pose.covariance[35], 0.01)
        # Body turns 90 degrees in place: base_link, 0.5 m behind it, swings around it.
        turned = self.publish(odometry(2, 3.0, 4.0, 0.3 + math.pi / 2))
        self.assertAlmostEqual(turned.pose.pose.position.x, 0.5, places=9)
        self.assertAlmostEqual(turned.pose.pose.position.y, -0.5, places=9)
        self.assertAlmostEqual(yaw_of(turned.pose.pose), math.pi / 2, places=9)

    def test_clock_reset_reanchors_and_invalid_pose_is_dropped(self):
        self.publish(odometry(5, 0.0, 0.0, 0.0))
        moved = self.publish(odometry(6, 1.0, 0.0, 0.0))
        self.assertAlmostEqual(moved.pose.pose.position.x, 1.0, places=9)
        reset = self.publish(odometry(1, 7.0, 7.0, 1.0))
        self.assertAlmostEqual(reset.pose.pose.position.x, 0.0, places=9)
        invalid = odometry(2, 0.0, 0.0, 0.0)
        invalid.pose.pose.orientation.w = 0.0
        count = len(self.received)
        self.node.on_odometry(invalid)
        self.executor.spin_once(timeout_sec=0.05)
        self.assertEqual(len(self.received), count)


if __name__ == "__main__":
    unittest.main()
