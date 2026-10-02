#!/usr/bin/env python3
"""Gate fresh 3D registrations into planar global pose observations.

/map_to_odom is published only after upstream ICP passes its configured fitness
threshold (normally 0.8). global_localization_xyz.py fills its pose covariance
with the unscaled ICP covariance of map -> camera_init (a left perturbation in
the map frame). This node propagates it to the base_link x/y/yaw observation,
adds minimum variances and multiplies the sum by registration_covariance_scale.
Corrections without covariance fall back to covariance_xy/covariance_yaw.
"""

import math

import numpy as np
import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Header

if __package__:
    from .localization_3d_pose import (
        BODY_TO_BASE, compose_pose, imu_mount_parameter, pose_values, seconds,
        valid_pose,
    )
else:
    from localization_3d_pose import (
        BODY_TO_BASE, compose_pose, imu_mount_parameter, pose_values, seconds,
        valid_pose,
    )


def valid_stamp(stamp):
    return stamp.sec >= 0 and 0 <= stamp.nanosec < 1_000_000_000 and (
        stamp.sec > 0 or stamp.nanosec > 0
    )


def yaw_of(quaternion):
    x, y, z, w = quaternion
    return math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))


def angle_difference(first, second):
    return math.atan2(math.sin(first - second), math.cos(first - second))


def planar_base(correction, lio, body_to_base=BODY_TO_BASE):
    position, quaternion = base_pose(correction, lio, body_to_base)
    return position[0], position[1], yaw_of(quaternion)


def base_pose(correction, lio, body_to_base=BODY_TO_BASE):
    return compose_pose(compose_pose(pose_values(correction), pose_values(lio)), body_to_base)


def planar_registration_covariance(covariance, position, scale, min_xy, min_yaw):
    """Propagate a map-frame 6x6 registration covariance to base x/y/yaw."""
    covariance = np.asarray(covariance, dtype=float).reshape(6, 6)
    px, py, pz = position
    # Left perturbation: delta_p = t + w x p, delta_yaw = w_z (ROS x,y,z,roll,pitch,yaw).
    jacobian = np.array([
        [1.0, 0.0, 0.0, 0.0, pz, -py],
        [0.0, 1.0, 0.0, -pz, 0.0, px],
        [0.0, 0.0, 0.0, 0.0, 0.0, 1.0],
    ])
    # The scale multiplies the whole published covariance, floors included, so a
    # multiplier measured from published covariances can be applied directly.
    planar = jacobian @ covariance @ jacobian.T
    planar = 0.5 * (planar + planar.T) + np.diag((min_xy, min_xy, min_yaw))
    return scale * planar


class GlobalPoseAdapter(Node):
    def __init__(self, **kwargs):
        super().__init__("global_pose_adapter", **kwargs)
        self.auto_initial_pose = self.declare_parameter("auto_initial_pose", False).value
        self.body_to_base = imu_mount_parameter(self)
        # map -> camera_init guess (x, y, z, yaw) sent as the automatic initial pose.
        self.initial_pose = tuple(
            float(self.declare_parameter(f"initial_{name}", 0.0).value)
            for name in ("x", "y", "z", "yaw")
        )
        if not all(math.isfinite(value) for value in self.initial_pose):
            raise ValueError("initial_x, initial_y, initial_z and initial_yaw must be finite")
        self.upstream_node_name = self.declare_parameter(
            "upstream_node_name", "global_localization"
        ).value
        self.max_correction_age = self.declare_parameter("max_correction_age", 1.0).value
        self.max_lio_age = self.declare_parameter("max_lio_age", 0.5).value
        self.max_scan_age = self.declare_parameter("max_scan_age", 3.0).value
        self.max_alignment = self.declare_parameter("max_alignment", 3.0).value
        self.max_translation_jump = self.declare_parameter("max_translation_jump", 1.0).value
        self.jump_per_meter = self.declare_parameter("jump_per_meter", 0.2).value
        self.max_yaw_jump = self.declare_parameter("max_yaw_jump", 0.35).value
        self.jump_per_radian = self.declare_parameter("jump_per_radian", 0.2).value
        self.covariance_xy = self.declare_parameter("covariance_xy", 0.25).value
        self.covariance_yaw = self.declare_parameter("covariance_yaw", 0.09).value
        self.registration_covariance_scale = self.declare_parameter(
            "registration_covariance_scale", 1.0
        ).value
        self.min_covariance_xy = self.declare_parameter("min_covariance_xy", 1.0e-4).value
        self.min_covariance_yaw = self.declare_parameter("min_covariance_yaw", 1.0e-5).value
        self.covariance_unobserved = self.declare_parameter(
            "covariance_unobserved", 1_000_000.0
        ).value
        for name in (
            "max_correction_age", "max_lio_age", "max_scan_age", "max_alignment",
            "max_translation_jump", "max_yaw_jump", "covariance_xy",
            "covariance_yaw", "covariance_unobserved", "registration_covariance_scale",
            "min_covariance_xy", "min_covariance_yaw",
        ):
            value = getattr(self, name)
            if not isinstance(value, (float, int)) or not math.isfinite(value) or value <= 0:
                raise ValueError(f"{name} must be finite and positive")
        for name in ("jump_per_meter", "jump_per_radian"):
            value = getattr(self, name)
            if not isinstance(value, (float, int)) or not math.isfinite(value) or value < 0:
                raise ValueError(f"{name} must be finite and nonnegative")

        self.lio = None
        self.scan_time = None
        self.last_correction_time = None
        self.last_accepted_scan_time = None
        self.last_accepted_lio = None
        self.last_accepted_correction = None
        self.last_now = None
        self.initial_pose_received = False
        self.warned = set()
        self.pose_publisher = self.create_publisher(
            PoseWithCovarianceStamped, "localization_3d/global_pose", 10
        )
        self.accepted_publisher = self.create_publisher(
            Header, "localization_3d/accepted_correction", 10
        )
        self.initial_pose_publisher = self.create_publisher(
            PoseWithCovarianceStamped, "initialpose", 10
        )
        self.create_subscription(PoseWithCovarianceStamped, "initialpose", self.on_initial_pose, 10)
        self.create_subscription(Odometry, "Odometry", self.on_odometry, 10)
        self.create_subscription(PointCloud2, "cloud_registered", self.on_scan, 10)
        self.create_subscription(Odometry, "map_to_odom", self.on_correction, 10)

    def reject(self, reason):
        if reason not in self.warned:
            self.get_logger().warning(f"3D global pose rejected: {reason}")
            self.warned.add(reason)

    def reset_for_clock(self):
        self.lio = None
        self.scan_time = None
        self.last_correction_time = None
        self.last_accepted_scan_time = None
        self.last_accepted_lio = None
        self.last_accepted_correction = None
        self.initial_pose_received = False
        self.warned.clear()
        self.get_logger().warning("Simulation clock reset; waiting for fresh 3D inputs")

    def now(self):
        current = self.get_clock().now().nanoseconds * 1e-9
        if self.last_now is not None and current < self.last_now - 0.1:
            self.reset_for_clock()
        self.last_now = current
        return current

    def on_initial_pose(self, message):
        if (
            message.header.frame_id == "map"
            and valid_stamp(message.header.stamp)
            and valid_pose(message.pose.pose)
        ):
            self.initial_pose_received = True
            self.last_accepted_lio = None
            self.last_accepted_correction = None
        else:
            self.reject("invalid initial pose")

    def on_odometry(self, message):
        now = self.now()
        if (
            message.header.frame_id != "camera_init"
            or message.child_frame_id != "body"
            or not valid_stamp(message.header.stamp)
            or not valid_pose(message.pose.pose)
        ):
            self.reject("invalid LIO odometry")
            return
        timestamp = seconds(message.header.stamp)
        if self.lio is not None and timestamp < seconds(self.lio.header.stamp) - 0.1:
            self.reject("out-of-order LIO odometry")
            return
        if now < timestamp or now - timestamp > self.max_lio_age:
            self.reject("stale or future LIO odometry")
            return
        self.lio = message
        if (
            self.auto_initial_pose
            and not self.initial_pose_received
            and self.initial_pose_publisher.get_subscription_count() > 1
            and any(
                endpoint.node_name == self.upstream_node_name
                for endpoint in self.get_subscriptions_info_by_topic(
                    self.initial_pose_publisher.topic_name
                )
            )
        ):
            x, y, z, yaw = self.initial_pose
            initial = PoseWithCovarianceStamped()
            initial.header.stamp = message.header.stamp
            initial.header.frame_id = "map"
            initial.pose.pose.position.x = x
            initial.pose.pose.position.y = y
            initial.pose.pose.position.z = z
            initial.pose.pose.orientation.z = math.sin(yaw / 2)
            initial.pose.pose.orientation.w = math.cos(yaw / 2)
            self.initial_pose_received = True
            self.initial_pose_publisher.publish(initial)
            self.get_logger().info(
                f"Sent spawn initial pose ({x:.2f}, {y:.2f}, {yaw:.2f}) to 3D localizer"
            )

    def on_scan(self, message):
        now = self.now()
        if message.header.frame_id != "camera_init" or not valid_stamp(message.header.stamp):
            self.reject("invalid scan header")
            return
        timestamp = seconds(message.header.stamp)
        if timestamp > now or now - timestamp > self.max_scan_age:
            self.reject("stale or future scan")
            return
        if self.scan_time is not None and timestamp <= self.scan_time:
            self.reject("out-of-order scan")
            return
        self.scan_time = timestamp

    def planar_covariance(self, registration_covariance, position):
        registration = np.asarray(registration_covariance, dtype=float)
        if not np.any(registration):
            return np.diag((self.covariance_xy, self.covariance_xy, self.covariance_yaw))
        registration = registration.reshape(6, 6)
        if (
            not np.all(np.isfinite(registration))
            or np.any(np.diag(registration) < 0)
            or not np.allclose(registration, registration.T, rtol=1e-6, atol=1e-12)
        ):
            self.reject("invalid registration covariance")
            return None
        return planar_registration_covariance(
            registration, position, self.registration_covariance_scale,
            self.min_covariance_xy, self.min_covariance_yaw,
        )

    def on_correction(self, message):
        now = self.now()
        if (
            message.header.frame_id != "map"
            or message.child_frame_id not in ("", "camera_init")
            or not valid_stamp(message.header.stamp)
            or not valid_pose(message.pose.pose)
        ):
            self.reject("invalid map-to-camera_init correction")
            return
        timestamp = seconds(message.header.stamp)
        if self.last_correction_time is not None and timestamp <= self.last_correction_time:
            self.reject("duplicate or out-of-order correction")
            return
        if timestamp > now or now - timestamp > self.max_correction_age:
            self.reject("stale or future correction")
            return
        if self.lio is None or self.scan_time is None:
            self.reject("missing LIO odometry or scan")
            return
        lio_time = seconds(self.lio.header.stamp)
        if (
            now - lio_time > self.max_lio_age
            or now - self.scan_time > self.max_scan_age
            or abs(timestamp - lio_time) > self.max_alignment
            or timestamp - self.scan_time < 0
            or timestamp - self.scan_time > self.max_alignment
        ):
            self.reject("LIO/scan/correction time misalignment")
            return
        if (
            self.last_accepted_scan_time is not None
            and (
                self.scan_time <= self.last_accepted_scan_time
                or self.scan_time <= self.last_correction_time
            )
        ):
            self.reject("no new scan since previous correction")
            return
        if self.last_accepted_lio is not None:
            old_lio = self.last_accepted_lio
            old_pose = planar_base(self.last_accepted_correction, old_lio.pose.pose,
                                   self.body_to_base)
            expected = planar_base(self.last_accepted_correction, self.lio.pose.pose,
                                   self.body_to_base)
            proposed = planar_base(message.pose.pose, self.lio.pose.pose, self.body_to_base)
            distance = math.hypot(expected[0] - old_pose[0], expected[1] - old_pose[1])
            turn = abs(angle_difference(expected[2], old_pose[2]))
            jump = math.hypot(proposed[0] - expected[0], proposed[1] - expected[1])
            yaw_jump = abs(angle_difference(proposed[2], expected[2]))
            if (
                jump > self.max_translation_jump + self.jump_per_meter * distance
                or yaw_jump > self.max_yaw_jump + self.jump_per_radian * turn
            ):
                self.reject("registration innovation jump")
                return

        position, quaternion = base_pose(message.pose.pose, self.lio.pose.pose, self.body_to_base)
        x, y, yaw = position[0], position[1], yaw_of(quaternion)
        observation = PoseWithCovarianceStamped()
        observation.header.frame_id = "map"
        # map->camera_init is a frame correction; the base pose it yields is the
        # LIO pose's, so it must carry the LIO stamp (up to 0.2 s older than the
        # correction). The Global EKF replays lagged measurements at that time.
        observation.header.stamp = self.lio.header.stamp
        observation.pose.pose.position.x = x
        observation.pose.pose.position.y = y
        observation.pose.pose.orientation.z = math.sin(yaw / 2)
        observation.pose.pose.orientation.w = math.cos(yaw / 2)
        planar = self.planar_covariance(message.pose.covariance, position)
        if planar is None:
            return
        covariance = [0.0] * 36
        for row, ros_row in enumerate((0, 1, 5)):
            for column, ros_column in enumerate((0, 1, 5)):
                covariance[ros_row * 6 + ros_column] = float(planar[row, column])
        for index in (14, 21, 28):
            covariance[index] = self.covariance_unobserved
        observation.pose.covariance = covariance
        self.last_correction_time = timestamp
        self.last_accepted_scan_time = self.scan_time
        self.last_accepted_lio = self.lio
        self.last_accepted_correction = message.pose.pose
        self.warned.clear()
        self.pose_publisher.publish(observation)
        accepted = Header()
        accepted.frame_id = "map"
        accepted.stamp = message.header.stamp
        self.accepted_publisher.publish(accepted)


def main():
    rclpy.init()
    node = GlobalPoseAdapter()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
