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

FAST-LIO's camera_init is the IMU frame at start-up, so it is tilted whenever the
robot starts tilted. For a 3D odom -> base_link (gravity_align, set by the launch
files with local_odometry.three_d) the frame is levelled with gravity: over the
first level_scans scans the accelerometer mean between scans is rotated into
camera_init with FAST-LIO's attitude and averaged (the robot must stand still, as
FAST-LIO itself assumes). The origin keeps the initial roll/pitch: only position
and yaw are re-anchored. z_variance and roll_pitch_variance weight the 3D pose.
"""

import math
from collections import deque

import rclpy
from geometry_msgs.msg import TwistWithCovarianceStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu

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
# Accelerometer samples kept between scans for the gravity direction.
GRAVITY_SAMPLES = 200


def inverse_pose(pose):
    translation, (x, y, z, w) = pose
    inverse = (-x, -y, -z, w)
    return tuple(-value for value in rotate_vector(inverse, translation)), inverse


def pose_covariance(position_variance, yaw_variance, z_variance=UNUSED_VARIANCE,
                    roll_pitch_variance=UNUSED_VARIANCE):
    """Diagonal covariance (x, y, z, roll, pitch, yaw), also used for the twist."""
    covariance = [0.0] * 36
    for index, variance in enumerate((
        position_variance, position_variance, z_variance,
        roll_pitch_variance, roll_pitch_variance, yaw_variance,
    )):
        covariance[index * 7] = variance
    return covariance


def gravity_alignment(acceleration):
    """Quaternion rotating the measured specific force (up at rest) onto +z."""
    norm = math.sqrt(sum(value * value for value in acceleration))
    if not math.isfinite(norm) or norm < 1e-6:
        raise ValueError("no gravity in the accelerometer mean")
    x, y, z = (value / norm for value in acceleration)
    if z < -0.999999:
        return 1.0, 0.0, 0.0, 0.0
    w = 1 + z
    scale = math.sqrt(y * y + x * x + w * w)
    return y / scale, -x / scale, 0.0, w / scale


def yaw_only(quaternion):
    yaw = quaternion_yaw(quaternion)
    return 0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2)


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
        position_variance = positive_parameter(self, "position_variance", 1e-4)
        yaw_variance = positive_parameter(self, "yaw_variance", 1e-4)
        self.gravity_align = bool(self.declare_parameter("gravity_align", False).value)
        if self.gravity_align:
            self.covariance = pose_covariance(
                position_variance, yaw_variance,
                positive_parameter(self, "z_variance", position_variance),
                positive_parameter(self, "roll_pitch_variance", yaw_variance),
            )
        else:
            self.covariance = pose_covariance(position_variance, yaw_variance)
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
        self.level = None
        self.accelerations = deque(maxlen=GRAVITY_SAMPLES)
        self.gravity_sum = [0.0, 0.0, 0.0]
        self.gravity_scans = 0
        self.imu_subscription = None
        if self.gravity_align:
            self.level_scans = int(self.declare_parameter("level_scans", 10).value)
            if self.level_scans < 1:
                raise ValueError("level_scans must be at least 1")
            self.imu_subscription = self.create_subscription(
                Imu, self.declare_parameter("imu_topic", "/isaac/imu").value,
                self.on_imu, qos_profile_sensor_data,
            )
        self.create_subscription(Odometry, "/Odometry", self.on_odometry, 20)

    def on_imu(self, message):
        acceleration = message.linear_acceleration
        self.accelerations.append((acceleration.x, acceleration.y, acceleration.z))

    def levelled(self, body):
        """FAST-LIO body pose in a gravity-aligned camera_init, or None while levelling."""
        if self.level is None:
            if not self.accelerations:
                self.get_logger().warning(
                    "Waiting for IMU data to level the 3D LIO odometry", throttle_duration_sec=5.0)
                return None
            count = len(self.accelerations)
            mean = [sum(sample[axis] for sample in self.accelerations) / count for axis in range(3)]
            self.accelerations.clear()
            # Gravity seen in camera_init: the body-frame specific force rotated by FAST-LIO's attitude.
            for axis, value in enumerate(rotate_vector(body[1], mean)):
                self.gravity_sum[axis] += value
            self.gravity_scans += 1
            if self.gravity_scans < self.level_scans:
                return None
            self.level = gravity_alignment(self.gravity_sum)
            self.destroy_subscription(self.imu_subscription)
            self.imu_subscription = None
            self.get_logger().info(
                "Levelled LIO odometry over %d scans (camera_init tilt %.3f deg)" % (
                    self.gravity_scans,
                    math.degrees(2 * math.acos(max(-1.0, min(1.0, self.level[3]))))))
        return compose_pose(((0.0, 0.0, 0.0), self.level), body)

    def on_odometry(self, message):
        if not valid_pose(message.pose.pose):
            self.get_logger().error("Invalid FAST-LIO odometry pose", throttle_duration_sec=5.0)
            return
        stamp = seconds(message.header.stamp)
        body = pose_values(message.pose.pose)
        if self.gravity_align:
            body = self.levelled(body)
            if body is None:
                return
        base = compose_pose(body, self.body_to_base)
        if self.origin is None or stamp < self.last_stamp:
            if self.origin is not None:
                self.get_logger().warning("FAST-LIO clock jumped back; re-anchoring LIO odometry")
            # A levelled frame keeps the true roll/pitch: anchor position and yaw only.
            self.origin = inverse_pose((base[0], yaw_only(base[1])) if self.gravity_align
                                       else base)
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
