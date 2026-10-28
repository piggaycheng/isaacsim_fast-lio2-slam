import math
import sys
import unittest
from pathlib import Path

import rclpy
from geometry_msgs.msg import TwistWithCovarianceStamped
from nav_msgs.msg import Odometry
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from sensor_msgs.msg import Imu

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))
import lio_odometry  # noqa: E402
from localization_3d_pose import compose_pose  # noqa: E402


def odometry(sec, x, y, yaw, z=0.526, nanosec=0):
    message = Odometry()
    message.header.stamp.sec = sec
    message.header.stamp.nanosec = nanosec
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


class LioNodeTestCase(unittest.TestCase):
    PARAMETERS = ()

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
            *self.PARAMETERS,
        ])
        self.received = []
        self.twists = []
        self.listener = Node("lio_odometry_listener")
        self.listener.create_subscription(Odometry, "lio/odom", self.received.append, 10)
        self.listener.create_subscription(
            TwistWithCovarianceStamped, "lio/twist", self.twists.append, 10)
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
                for _ in range(5):
                    self.executor.spin_once(timeout_sec=0.01)
                return self.received[-1]
        self.fail("lio/odom not published")


class LioOdometryNodeTest(LioNodeTestCase):
    PARAMETERS = (
        Parameter("twist_linear_variance", value=3e-5),
        Parameter("twist_angular_variance", value=7e-6),
    )

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

    def test_twist_is_base_link_velocity_with_its_own_variances(self):
        self.publish(odometry(1, 0.0, 0.0, 0.0))
        self.assertEqual(self.twists, [])
        # Body moves 0.1 m forward and turns 0.05 rad in 0.1 s.
        self.publish(odometry(1, 0.1, 0.0, 0.05, nanosec=100_000_000))
        self.assertEqual(len(self.twists), 1)
        twist = self.twists[-1]
        self.assertEqual(twist.header.frame_id, "base_link")
        self.assertEqual((twist.header.stamp.sec, twist.header.stamp.nanosec), (1, 100_000_000))
        self.assertAlmostEqual(twist.twist.twist.angular.z, 0.5, places=6)
        # base_link is 0.5 m behind body: turning moves it sideways (to the right).
        # The increment is expressed in the previous base_link frame (yaw 0 here).
        expected = (0.1 - 0.5 * math.cos(0.05) + 0.5, -0.5 * math.sin(0.05))
        self.assertAlmostEqual(twist.twist.twist.linear.x, expected[0] * 10, places=6)
        self.assertAlmostEqual(twist.twist.twist.linear.y, expected[1] * 10, places=6)
        covariance = twist.twist.covariance
        self.assertEqual((covariance[0], covariance[7], covariance[35]), (3e-5, 3e-5, 7e-6))
        # Independent of the local pose variances.
        self.assertEqual(self.received[-1].pose.covariance[0], 0.04)

    def test_no_twist_across_gaps_or_reanchoring(self):
        self.publish(odometry(5, 0.0, 0.0, 0.0))
        self.publish(odometry(7, 1.0, 0.0, 0.0))
        self.assertEqual(self.twists, [], "2 s gap exceeds max_twist_interval")
        self.publish(odometry(1, 0.0, 0.0, 0.0))
        self.assertEqual(self.twists, [], "clock reset re-anchors without a twist")
        self.publish(odometry(1, 0.02, 0.0, 0.0, nanosec=100_000_000))
        self.assertEqual(len(self.twists), 1)
        self.assertAlmostEqual(self.twists[-1].twist.twist.linear.x, 0.2, places=6)


def roll_pitch_of(pose):
    q = pose.orientation
    roll = math.atan2(2 * (q.w * q.x + q.y * q.z), 1 - 2 * (q.x ** 2 + q.y ** 2))
    pitch = math.asin(max(-1.0, min(1.0, 2 * (q.w * q.y - q.z * q.x))))
    return roll, pitch


class GravityAlignedLioOdometryTest(LioNodeTestCase):
    """three_d: FAST-LIO's camera_init is the (tilted) IMU frame at start-up."""

    PARAMETERS = (
        Parameter("roll_pitch_variance", value=0.002),
        Parameter("gravity_align", value=True),
        Parameter("level_scans", value=2),
    )

    def imu_at_rest(self, roll, pitch):
        """Accelerometer specific force of a still IMU with this attitude."""
        message = Imu()
        g = 9.81
        acceleration = message.linear_acceleration
        acceleration.x = -g * math.sin(pitch)
        acceleration.y = g * math.cos(pitch) * math.sin(roll)
        acceleration.z = g * math.cos(pitch) * math.cos(roll)
        for _ in range(3):
            self.node.on_imu(message)

    def level(self, roll, pitch, x=0.0, y=0.0, body_pitch=0.0):
        """Two still scans; FAST-LIO reports body_pitch in camera_init. Returns the 2nd output."""
        for nanosec in (0, 100_000_000):
            self.imu_at_rest(roll, pitch)
            message = odometry(1, x, y, 0.0, z=0.0, nanosec=nanosec)
            orientation = message.pose.pose.orientation
            orientation.y, orientation.w = math.sin(body_pitch / 2), math.cos(body_pitch / 2)
            if nanosec == 0:
                self.node.on_odometry(message)
                self.executor.spin_once(timeout_sec=0.05)
                self.assertEqual(self.received, [], "published before levelling")
            else:
                return self.publish(message)

    def test_waits_for_imu_before_publishing(self):
        self.node.on_odometry(odometry(1, 0.0, 0.0, 0.0, z=0.0))
        self.executor.spin_once(timeout_sec=0.05)
        self.assertEqual(self.received, [])

    def test_slope_start_keeps_pitch_and_climbs_in_z(self):
        pitch = -0.2  # nose up (ROS pitch is positive nose down)
        # FAST-LIO starts at identity in its tilted camera_init and drives 1 m up the slope.
        first = self.level(0.0, pitch)
        position = first.pose.pose.position
        for value in (position.x, position.y, position.z):
            self.assertAlmostEqual(value, 0.0, places=9)
        roll, measured_pitch = roll_pitch_of(first.pose.pose)
        self.assertAlmostEqual(roll, 0.0, places=9)
        self.assertAlmostEqual(measured_pitch, pitch, places=9)
        self.assertAlmostEqual(yaw_of(first.pose.pose), 0.0, places=9)
        moved = self.publish(odometry(2, 1.0, 0.0, 0.0, z=0.0))
        self.assertAlmostEqual(moved.pose.pose.position.x, math.cos(pitch), places=9)
        self.assertAlmostEqual(moved.pose.pose.position.z, -math.sin(pitch), places=9)
        self.assertAlmostEqual(roll_pitch_of(moved.pose.pose)[1], pitch, places=9)
        covariance = moved.pose.covariance
        # z defaults to position_variance; roll/pitch use roll_pitch_variance.
        self.assertEqual([covariance[index * 7] for index in range(6)],
                         [0.04, 0.04, 0.04, 0.002, 0.002, 0.01])

    def test_camera_init_tilt_is_measured_with_the_lio_attitude(self):
        # The IMU really pitches -0.2 rad, but FAST-LIO already reports -0.05 in camera_init.
        first = self.level(0.0, -0.2, body_pitch=-0.05)
        self.assertAlmostEqual(roll_pitch_of(first.pose.pose)[1], -0.2, places=9)

    def test_initial_position_is_anchored_but_roll_is_kept(self):
        first = self.level(0.1, 0.0, x=2.0, y=3.0)
        roll, pitch = roll_pitch_of(first.pose.pose)
        self.assertAlmostEqual(roll, 0.1, places=9)
        self.assertAlmostEqual(pitch, 0.0, places=9)
        self.assertAlmostEqual(yaw_of(first.pose.pose), 0.0, places=9)
        position = first.pose.pose.position
        self.assertAlmostEqual(math.hypot(position.x, position.y), 0.0, places=9)


class PoseCovarianceTest(unittest.TestCase):
    def test_planar_mode_keeps_placeholder_variances(self):
        covariance = lio_odometry.pose_covariance(0.04, 0.01)
        self.assertEqual([covariance[index * 7] for index in range(6)],
                         [0.04, 0.04, lio_odometry.UNUSED_VARIANCE,
                          lio_odometry.UNUSED_VARIANCE, lio_odometry.UNUSED_VARIANCE, 0.01])


if __name__ == "__main__":
    unittest.main()
