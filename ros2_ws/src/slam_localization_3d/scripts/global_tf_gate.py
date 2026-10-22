#!/usr/bin/env python3

import math
from collections import deque

import rclpy
from nav_msgs.msg import Odometry
from rclpy.clock import Clock
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.time import Time
from std_msgs.msg import Header
from std_srvs.srv import SetBool
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
        self.tf_future_tolerance = self.declare_parameter(
            "tf_future_tolerance", 0.1
        ).value
        if (
            type(self.tf_future_tolerance) not in (int, float)
            or not math.isfinite(self.tf_future_tolerance)
            or not 0 <= self.tf_future_tolerance <= 0.5
        ):
            raise ValueError("tf_future_tolerance must be between 0 and 0.5 seconds")
        self.tf_wait_timeout = self.declare_parameter("tf_wait_timeout", 0.5).value
        if (
            type(self.tf_wait_timeout) not in (int, float)
            or not math.isfinite(self.tf_wait_timeout)
            or self.tf_wait_timeout <= 0
        ):
            raise ValueError("tf_wait_timeout must be positive")
        # Global odometry waiting for the local odom -> base_link TF at the same stamp.
        self.pending = deque(maxlen=64)
        self.last_tf_error = None
        self.last_correction = None
        self.map_switch_hold = False
        self.minimum_correction_time = None
        self.warned_stale = False
        self.last_tf_warning = None
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.broadcaster = TransformBroadcaster(self)
        self.create_subscription(
            Header, "localization_3d/accepted_correction", self.on_correction, 10
        )
        self.create_subscription(
            Odometry, "odometry/global", self.on_global_odometry, 10
        )
        self.create_service(SetBool, "global_tf_gate/set_map_switch_hold", self.on_map_switch_hold)
        # Retry on wall time: the simulator may run slower than real time, so the
        # local EKF's next TF can arrive many wall-clock milliseconds later.
        self.create_timer(0.01, self.flush_pending, clock=Clock())

    def on_map_switch_hold(self, request, response):
        self.map_switch_hold = request.data
        self.last_correction = None
        self.pending.clear()
        self.minimum_correction_time = seconds(self.get_clock().now().to_msg())
        response.success = True
        response.message = "Global TF gate reset"
        return response

    def on_correction(self, message):
        if self.map_switch_hold or (
            self.minimum_correction_time is not None
            and seconds(message.stamp) <= self.minimum_correction_time
        ):
            return
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
        if self.map_switch_hold:
            return
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
        self.pending.append(message)
        self.flush_pending()

    def flush_pending(self):
        if not self.pending:
            return
        if self.map_switch_hold or self.last_correction is None:
            self.pending.clear()
            return
        now = seconds(self.get_clock().now().to_msg())
        # Publish the newest global pose whose local TF is available; older ones are superseded.
        for index in range(len(self.pending) - 1, -1, -1):
            message = self.pending[index]
            try:
                local = self.tf_buffer.lookup_transform(
                    "odom", "base_link", Time.from_msg(message.header.stamp)
                )
            except TransformException as error:
                self.last_tf_error = error
                continue
            for _ in range(index + 1):
                self.pending.popleft()
            self.publish(message, local)
            break
        expired = False
        while self.pending and not (
            now - self.tf_wait_timeout
            <= seconds(self.pending[0].header.stamp)
            <= now + 0.5
        ):
            self.pending.popleft()
            expired = True
        if expired and (
            self.last_tf_warning is None
            or now < self.last_tf_warning
            or now - self.last_tf_warning >= 5
        ):
            self.get_logger().warning(
                f"No synchronized local odometry TF: {self.last_tf_error}"
            )
            self.last_tf_warning = now

    def publish(self, message, local):
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
        # Keep the measured pose, but leave a short TF validity window for Nav2.
        transform.header.stamp = (
            Time.from_msg(message.header.stamp)
            + Duration(seconds=self.tf_future_tolerance)
        ).to_msg()
        transform.child_frame_id = "odom"
        transform.transform.translation.x = x
        transform.transform.translation.y = y
        transform.transform.rotation.z = math.sin(yaw / 2)
        transform.transform.rotation.w = math.cos(yaw / 2)
        self.broadcaster.sendTransform(transform)


def main():
    rclpy.init()
    node = GlobalTfGate()
    # Keep the TF subscriber responsive alongside the gate callbacks.
    executor = MultiThreadedExecutor(num_threads=2)
    try:
        rclpy.spin(node, executor=executor)
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
