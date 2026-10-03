#!/usr/bin/env python3
"""Republish FAST-LIO odometry as odom -> base_link odometry for local_ekf.

FAST-LIO publishes camera_init -> body (the IMU). This node moves it to base_link
with the robot profile's imu_mount and re-anchors it so the first base_link pose
is the odom origin (lio/odom, fused as an absolute pose by local_ekf).

It also publishes the base_link velocity between consecutive scans (lio/twist),
which global_ekf fuses when there is no wheel odometry. FAST-LIO publishes no
covariance, so each output gets its own variances: position_variance and
yaw_variance (per-scan pose noise, robot profile, covariance_calibration.py)
for lio/odom, and twist_linear_variance / twist_angular_variance
(global_fusion.yaml) for lio/twist, so calibrating one never changes the other.
A backwards clock (simulation reset) re-anchors the origin.
"""

import math

import rclpy
from geometry_msgs.msg import TwistWithCovarianceStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node

if __package__:
    from .localization_3d_pose import (
        compose_pose, imu_mount_parameter, pose_values, rotate_vector, seconds, valid_pose,
    )
else:
    from localization_3d_pose import (
        compose_pose, imu_mount_parameter, pose_values, rotate_vector, seconds, valid_pose,
    )

# Variance of the unused z/roll/pitch entries (two_d_mode ignores them).
UNUSED_VARIANCE = 1e-2


def inverse_pose(pose):
    translation, (x, y, z, w) = pose
    inverse = (-x, -y, -z, w)
    return tuple(-value for value in rotate_vector(inverse, translation)), inverse


def pose_covariance(position_variance, yaw_variance):
    """Diagonal planar covariance (x, y, yaw), also used for (vx, vy, vyaw)."""
    covariance = [0.0] * 36
    for index, variance in enumerate((
        position_variance, position_variance, UNUSED_VARIANCE,
        UNUSED_VARIANCE, UNUSED_VARIANCE, yaw_variance,
    )):
        covariance[index * 7] = variance
    return covariance


def quaternion_yaw(quaternion):
    x, y, z, w = quaternion
    return math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))


def planar_velocity(previous, current, dt):
    """base_link (vx, vy, wz) moving from previous to current pose in dt seconds."""
    (x, y, _), quaternion = compose_pose(inverse_pose(previous), current)
    return x / dt, y / dt, quaternion_yaw(quaternion) / dt


def positive_parameter(node, name, default):
    value = float(node.declare_parameter(name, default).value)
    if not (math.isfinite(value) and value > 0):
        raise ValueError(f"{name} must be positive")
    return value


class LioOdometry(Node):
    def __init__(self, **kwargs):
        super().__init__("lio_odometry", **kwargs)
        self.body_to_base = imu_mount_parameter(self)
        self.odom_frame = self.declare_parameter("odom_frame", "odom").value
        self.base_frame = self.declare_parameter("base_frame", "base_link").value
        self.covariance = pose_covariance(
            positive_parameter(self, "position_variance", 1e-4),
            positive_parameter(self, "yaw_variance", 1e-4),
        )
        self.twist_covariance = pose_covariance(
            positive_parameter(self, "twist_linear_variance", 2e-5),
            positive_parameter(self, "twist_angular_variance", 2e-5),
        )
        self.max_twist_interval = positive_parameter(self, "max_twist_interval", 0.5)
        self.origin = None
        self.last_stamp = None
        self.last_base = None
        self.publisher = self.create_publisher(Odometry, "lio/odom", 20)
        self.twist_publisher = self.create_publisher(TwistWithCovarianceStamped, "lio/twist", 20)
        self.create_subscription(Odometry, "/Odometry", self.on_odometry, 20)

    def on_odometry(self, message):
        if not valid_pose(message.pose.pose):
            self.get_logger().error("Invalid FAST-LIO odometry pose", throttle_duration_sec=5.0)
            return
        stamp = seconds(message.header.stamp)
        base = compose_pose(pose_values(message.pose.pose), self.body_to_base)
        if self.origin is None or stamp < self.last_stamp:
            if self.origin is not None:
                self.get_logger().warning("FAST-LIO clock jumped back; re-anchoring LIO odometry")
            self.origin = inverse_pose(base)
            self.last_base = None
        if self.last_base is not None and 0 < stamp - self.last_stamp <= self.max_twist_interval:
            self.publish_twist(message.header.stamp, planar_velocity(
                self.last_base, base, stamp - self.last_stamp))
        self.last_stamp = stamp
        self.last_base = base
        (x, y, z), (qx, qy, qz, qw) = compose_pose(self.origin, base)
        output = Odometry()
        output.header.stamp = message.header.stamp
        output.header.frame_id = self.odom_frame
        output.child_frame_id = self.base_frame
        pose = output.pose.pose
        pose.position.x, pose.position.y, pose.position.z = x, y, z
        pose.orientation.x, pose.orientation.y = qx, qy
        pose.orientation.z, pose.orientation.w = qz, qw
        output.pose.covariance = self.covariance
        self.publisher.publish(output)

    def publish_twist(self, stamp, velocity):
        output = TwistWithCovarianceStamped()
        output.header.stamp = stamp
        output.header.frame_id = self.base_frame
        twist = output.twist.twist
        twist.linear.x, twist.linear.y, twist.angular.z = velocity
        output.twist.covariance = self.twist_covariance
        self.twist_publisher.publish(output)


def main():
    rclpy.init()
    node = LioOdometry()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
