#!/usr/bin/env python3

import math

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.time import Time
from std_msgs.msg import Header
from tf2_ros import Buffer, TransformBroadcaster, TransformException, TransformListener
from geometry_msgs.msg import TransformStamped

from localization_3d_pose import seconds, valid_pose


def yaw_of(quaternion):
    return math.atan2(
        2 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y),
        1 - 2 * (quaternion.y**2 + quaternion.z**2),
    )


def map_to_odom(global_pose, odom_pose):
    yaw = yaw_of(global_pose.orientation) - yaw_of(odom_pose.rotation)
    yaw = math.atan2(math.sin(yaw), math.cos(yaw))
    x = global_pose.position.x - (
        math.cos(yaw) * odom_pose.translation.x
        - math.sin(yaw) * odom_pose.translation.y
    )
    y = global_pose.position.y - (
        math.sin(yaw) * odom_pose.translation.x
        + math.cos(yaw) * odom_pose.translation.y
    )
    return x, y, yaw


class GlobalTfGate(Node):
    def __init__(self):
        super().__init__("global_tf_gate")
        self.max_correction_age = self.declare_parameter(
            "max_correction_age", 5.0
        ).value
        if self.max_correction_age <= 0:
            raise ValueError("max_correction_age must be positive")
        self.last_correction = None
        self.warned_stale = False
        self.last_tf_warning = None
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.broadcaster = TransformBroadcaster(self)
        self.create_subscription(
            Header, "/localization_3d/accepted_correction", self.on_correction, 10
        )
        self.create_subscription(
            Odometry, "/odometry/global", self.on_global_odometry, 10
        )

    def on_correction(self, message):
        if message.frame_id != "map" or seconds(message.stamp) <= 0:
            self.get_logger().error("Invalid accepted correction header")
            return
        correction_time = seconds(message.stamp)
        if self.last_correction is not None and correction_time < self.last_correction:
            self.get_logger().warning("Correction clock reset; clearing TF gate")
            self.last_tf_warning = None
        self.last_correction = correction_time
        self.warned_stale = False

    def on_global_odometry(self, message):
        if (
            message.header.frame_id != "map"
            or message.child_frame_id != "base_link"
            or not valid_pose(message.pose.pose)
        ):
            self.get_logger().error("Invalid global EKF odometry")
            return
        stamp = seconds(message.header.stamp)
        now = seconds(self.get_clock().now().to_msg())
        if self.last_correction is None or now < self.last_correction - 0.5:
            self.last_correction = None
            return
        if (
            stamp < self.last_correction
            or stamp > now + 0.5
            or now - self.last_correction > self.max_correction_age
        ):
            if not self.warned_stale:
                self.get_logger().warning("3D correction stale; withholding map -> odom TF")
                self.warned_stale = True
            return
        try:
            local = self.tf_buffer.lookup_transform(
                "odom", "base_link", Time.from_msg(message.header.stamp)
            )
        except TransformException as error:
            if (
                self.last_tf_warning is None
                or now < self.last_tf_warning
                or now - self.last_tf_warning >= 5
            ):
                self.get_logger().warning(f"No synchronized local odometry TF: {error}")
                self.last_tf_warning = now
            return
        if (
            not all(
                math.isfinite(value)
                for value in (
                    local.transform.translation.x,
                    local.transform.translation.y,
                    local.transform.translation.z,
                    local.transform.rotation.x,
                    local.transform.rotation.y,
                    local.transform.rotation.z,
                    local.transform.rotation.w,
                )
            )
            or not 0.98 <= sum(
                getattr(local.transform.rotation, axis) ** 2
                for axis in ("x", "y", "z", "w")
            ) <= 1.02
        ):
            self.get_logger().error("Invalid local odometry TF")
            return
        x, y, yaw = map_to_odom(message.pose.pose, local.transform)
        transform = TransformStamped()
        transform.header = message.header
        transform.child_frame_id = "odom"
        transform.transform.translation.x = x
        transform.transform.translation.y = y
        transform.transform.rotation.z = math.sin(yaw / 2)
        transform.transform.rotation.w = math.cos(yaw / 2)
        self.broadcaster.sendTransform(transform)


def main():
    rclpy.init()
    node = GlobalTfGate()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
