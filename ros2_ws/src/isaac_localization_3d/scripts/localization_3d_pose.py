#!/usr/bin/env python3

import math

import rclpy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, TransformStamped
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster
from visualization_msgs.msg import Marker


# FAST-LIO body (LiDAR/IMU) -> base_link. xy comes from covariance_calibration.py's lever-arm
# fit (USD mount is 0.2317, 0); keep in sync with global_fusion.launch.py's static TFs.
BODY_TO_BASE = ((0.213, -0.009, -0.526), (0.0, 0.0, 1.0, 0.0))


def rotate_vector(quaternion, vector):
    x, y, z, w = quaternion
    vx, vy, vz = vector
    tx = 2 * (y * vz - z * vy)
    ty = 2 * (z * vx - x * vz)
    tz = 2 * (x * vy - y * vx)
    return (
        vx + w * tx + y * tz - z * ty,
        vy + w * ty + z * tx - x * tz,
        vz + w * tz + x * ty - y * tx,
    )


def compose_pose(first, second):
    translation = rotate_vector(first[1], second[0])
    a, b = first[1], second[1]
    quaternion = (
        a[3] * b[0] + a[0] * b[3] + a[1] * b[2] - a[2] * b[1],
        a[3] * b[1] - a[0] * b[2] + a[1] * b[3] + a[2] * b[0],
        a[3] * b[2] + a[0] * b[1] - a[1] * b[0] + a[2] * b[3],
        a[3] * b[3] - a[0] * b[0] - a[1] * b[1] - a[2] * b[2],
    )
    return (
        tuple(first[0][axis] + translation[axis] for axis in range(3)),
        quaternion,
    )


def pose_values(pose):
    position, orientation = pose.position, pose.orientation
    return (
        (position.x, position.y, position.z),
        (orientation.x, orientation.y, orientation.z, orientation.w),
    )


def valid_pose(pose):
    translation, quaternion = pose_values(pose)
    norm = sum(component**2 for component in quaternion)
    return all(math.isfinite(value) for value in (*translation, *quaternion)) and (
        0.98 <= norm <= 1.02
    )


def seconds(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


class LocalizationVisualization(Node):
    def __init__(self):
        super().__init__("localization_3d_pose")
        self.correction = None
        self.correction_time = None
        self.last_odom_time = None
        self.path = Path()
        self.path.header.frame_id = "map"
        self.warned_stale = False
        self.auto_initial_pose = self.declare_parameter("auto_initial_pose", False).value
        self.initial_pose_received = False
        self.tf_broadcaster = TransformBroadcaster(self)
        self.static_tf_broadcaster = StaticTransformBroadcaster(self)
        body_base_tf = TransformStamped()
        body_base_tf.header.frame_id = "body"
        body_base_tf.child_frame_id = "base_link"
        (
            body_base_tf.transform.translation.x,
            body_base_tf.transform.translation.y,
            body_base_tf.transform.translation.z,
        ) = BODY_TO_BASE[0]
        (
            body_base_tf.transform.rotation.x,
            body_base_tf.transform.rotation.y,
            body_base_tf.transform.rotation.z,
            body_base_tf.transform.rotation.w,
        ) = BODY_TO_BASE[1]
        self.static_tf_broadcaster.sendTransform(body_base_tf)
        self.pose_publisher = self.create_publisher(
            PoseStamped, "/localization_3d/base_pose_2d", 10
        )
        self.marker_publisher = self.create_publisher(
            Marker, "/localization_3d/base_marker", 10
        )
        self.odom_publisher = self.create_publisher(
            Odometry, "/localization_3d/base_odom", 10
        )
        self.path_publisher = self.create_publisher(Path, "/localization_3d/path", 10)
        self.initial_pose_publisher = self.create_publisher(
            PoseWithCovarianceStamped, "/initialpose", 10
        )
        self.create_subscription(
            PoseWithCovarianceStamped, "/initialpose", self.on_initial_pose, 10
        )
        self.create_subscription(Odometry, "/map_to_odom", self.on_correction, 10)
        self.create_subscription(Odometry, "/Odometry", self.on_odometry, 10)

    def on_initial_pose(self, message):
        if message.header.frame_id == "map":
            self.initial_pose_received = True

    def on_correction(self, message):
        if message.header.frame_id != "map" or not valid_pose(message.pose.pose):
            self.get_logger().error("Ignoring invalid map-to-camera_init correction")
            return
        correction_time = seconds(message.header.stamp)
        if self.last_odom_time is not None and correction_time < self.last_odom_time - 5:
            self.get_logger().error("Ignoring expired localization correction")
            return
        self.correction = message.pose.pose
        self.correction_time = correction_time
        self.warned_stale = False
        self.get_logger().info("3D map registration accepted", once=True)

    def on_odometry(self, message):
        if (
            message.header.frame_id != "camera_init"
            or message.child_frame_id != "body"
            or not valid_pose(message.pose.pose)
        ):
            self.get_logger().error("Ignoring invalid FAST-LIO odometry")
            return
        odom_time = seconds(message.header.stamp)
        if self.last_odom_time is not None and odom_time < self.last_odom_time - 0.1:
            self.correction = None
            self.path.poses.clear()
            self.initial_pose_received = False
            self.get_logger().warning("Simulation clock reset; waiting for new 3D registration")
        self.last_odom_time = odom_time
        if (
            self.auto_initial_pose
            and not self.initial_pose_received
            and self.initial_pose_publisher.get_subscription_count() > 1
        ):
            initial_pose = PoseWithCovarianceStamped()
            initial_pose.header = message.header
            initial_pose.header.frame_id = "map"
            initial_pose.pose.pose.orientation.w = 1.0
            self.initial_pose_received = True
            self.initial_pose_publisher.publish(initial_pose)
            self.get_logger().info("Sent Office origin initial pose to 3D localizer")
        if self.correction is None:
            return
        if (
            self.correction_time > odom_time + 0.5
            or odom_time - self.correction_time > 5
        ):
            if not self.warned_stale:
                self.get_logger().warning("3D correction is stale; stopping visualization")
                self.warned_stale = True
            return
        correction = pose_values(self.correction)
        lio_body = pose_values(message.pose.pose)
        base_position, base_quaternion = compose_pose(
            compose_pose(correction, lio_body), BODY_TO_BASE
        )

        transform = TransformStamped()
        transform.header.stamp = message.header.stamp
        transform.header.frame_id = "map"
        transform.child_frame_id = "camera_init"
        (
            transform.transform.translation.x,
            transform.transform.translation.y,
            transform.transform.translation.z,
        ) = correction[0]
        (
            transform.transform.rotation.x,
            transform.transform.rotation.y,
            transform.transform.rotation.z,
            transform.transform.rotation.w,
        ) = correction[1]
        self.tf_broadcaster.sendTransform(transform)

        odometry = Odometry()
        odometry.header.stamp = message.header.stamp
        odometry.header.frame_id = "map"
        odometry.child_frame_id = "base_link"
        position, orientation = odometry.pose.pose.position, odometry.pose.pose.orientation
        position.x, position.y, position.z = base_position
        orientation.x, orientation.y, orientation.z, orientation.w = base_quaternion
        self.odom_publisher.publish(odometry)

        x, y, z, w = base_quaternion
        yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y**2 + z**2))
        pose_2d = PoseStamped()
        pose_2d.header = odometry.header
        pose_2d.pose.position.x = base_position[0]
        pose_2d.pose.position.y = base_position[1]
        pose_2d.pose.orientation.z = math.sin(yaw / 2)
        pose_2d.pose.orientation.w = math.cos(yaw / 2)
        self.pose_publisher.publish(pose_2d)
        marker = Marker()
        marker.header = pose_2d.header
        marker.ns = "carter_3d_localization"
        marker.id = 0
        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        marker.pose = pose_2d.pose
        marker.scale.x = 0.8
        marker.scale.y = 0.18
        marker.scale.z = 0.18
        marker.color.g = 1.0
        marker.color.b = 0.4
        marker.color.a = 1.0
        marker.lifetime.sec = 2
        self.marker_publisher.publish(marker)
        self.path.header.stamp = message.header.stamp
        self.path.poses.append(pose_2d)
        if len(self.path.poses) > 1500:
            del self.path.poses[: len(self.path.poses) - 1500]
        self.path_publisher.publish(self.path)


def main():
    rclpy.init()
    node = LocalizationVisualization()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
