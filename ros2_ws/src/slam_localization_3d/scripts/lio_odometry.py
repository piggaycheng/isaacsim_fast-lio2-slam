#!/usr/bin/env python3
"""Republish FAST-LIO odometry as odom -> base_link odometry for local_ekf.

FAST-LIO publishes camera_init -> body (the IMU). This node moves it to base_link
with the robot profile's imu_mount and re-anchors it so the first base_link pose
is the odom origin. local_ekf fuses it as an absolute pose and global_ekf (without
wheel odometry) as increments (local_ekf_inputs.yaml, robot_fleet.py). FAST-LIO
publishes no covariance, so fixed variances are attached. A backwards clock
(simulation reset) re-anchors the origin.
"""

import math

import rclpy
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
    covariance = [0.0] * 36
    for index, variance in enumerate((
        position_variance, position_variance, UNUSED_VARIANCE,
        UNUSED_VARIANCE, UNUSED_VARIANCE, yaw_variance,
    )):
        covariance[index * 7] = variance
    return covariance


class LioOdometry(Node):
    def __init__(self, **kwargs):
        super().__init__("lio_odometry", **kwargs)
        self.body_to_base = imu_mount_parameter(self)
        self.odom_frame = self.declare_parameter("odom_frame", "odom").value
        self.base_frame = self.declare_parameter("base_frame", "base_link").value
        position_variance = float(self.declare_parameter("position_variance", 1e-4).value)
        yaw_variance = float(self.declare_parameter("yaw_variance", 1e-4).value)
        if not all(math.isfinite(value) and value > 0 for value in (position_variance, yaw_variance)):
            raise ValueError("position_variance and yaw_variance must be positive")
        self.covariance = pose_covariance(position_variance, yaw_variance)
        self.origin = None
        self.last_stamp = None
        self.publisher = self.create_publisher(Odometry, "lio/odom", 20)
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
        self.last_stamp = stamp
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
