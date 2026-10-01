#!/usr/bin/env python3
"""Gate Nav2 commands before Carter's native ROS 2 /cmd_vel subscriber."""

import math

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan, PointCloud2
from std_msgs.msg import Bool, Header


class CmdVelSafety(Node):
    def __init__(self, **kwargs):
        super().__init__("cmd_vel_safety", **kwargs)
        self.last_correction = None
        self.last_safety_warning = None
        self.last_speed_warning = None
        self.stop_latched = False
        self.sensor_timeout = self.declare_parameter("sensor_timeout", 1.0).value
        if not math.isfinite(self.sensor_timeout) or self.sensor_timeout <= 0:
            self.destroy_node()
            raise ValueError("sensor_timeout must be finite and positive")
        self.sensor_stamps = {"scan": None, "obstacles": None}
        self.last_watchdog_time = None
        self.sensors_stale = None
        self.publisher = self.create_publisher(Twist, "/cmd_vel", 10)
        self.create_subscription(
            LaserScan, "/scan", lambda msg: self.on_sensor("scan", msg),
            qos_profile_sensor_data,
        )
        self.create_subscription(
            PointCloud2, "/perception/obstacles",
            lambda msg: self.on_sensor("obstacles", msg), qos_profile_sensor_data,
        )
        self.create_subscription(
            Header, "/localization_3d/accepted_correction", self.on_correction, 10
        )
        self.create_subscription(Bool, "/navigation/emergency_stop", self.on_stop, 10)
        self.create_subscription(Twist, "/nav2/cmd_vel", self.on_command, 10)
        self.create_timer(0.1, self.on_watchdog)

    def on_sensor(self, source, message):
        now = self.watchdog_time()
        stamp = message.header.stamp.sec + message.header.stamp.nanosec * 1e-9
        if not message.header.frame_id or not 0 <= now - stamp < self.sensor_timeout:
            self.get_logger().warning(f"Rejected stale or invalid {source} header")
            return
        self.sensor_stamps[source] = stamp

    def watchdog_time(self):
        now = self.get_clock().now().nanoseconds * 1e-9
        if self.last_watchdog_time is not None and now < self.last_watchdog_time:
            self.sensor_stamps = dict.fromkeys(self.sensor_stamps)
            self.last_correction = None
            self.get_logger().warning("Clock reset; clearing safety freshness state")
        self.last_watchdog_time = now
        return now

    def sensors_fresh(self, now):
        fresh = any(
            stamp is not None and 0 <= now - stamp < self.sensor_timeout
            for stamp in self.sensor_stamps.values()
        )
        if self.sensors_stale != (not fresh):
            if fresh:
                self.get_logger().info("Obstacle sensor data fresh; new commands may pass")
            else:
                self.get_logger().warning("Obstacle sensor data missing or stale; stopping Carter")
            self.sensors_stale = not fresh
        return fresh

    def on_watchdog(self):
        now = self.watchdog_time()
        if not self.sensors_fresh(now) or self.stop_latched or (
            self.last_correction is None
            or not 0 <= now - self.last_correction < 4.0
        ):
            self.publisher.publish(Twist())

    def on_stop(self, message):
        if message.data:
            self.stop_latched = True
            self.get_logger().error("Navigation emergency stop latched; restart to re-enable driving")
            self.publisher.publish(Twist())

    def on_correction(self, message):
        self.watchdog_time()
        if message.frame_id != "map" or message.stamp.sec < 0:
            self.get_logger().error("Invalid 3D localization correction header")
            return
        self.last_correction = message.stamp.sec + message.stamp.nanosec * 1e-9

    def on_command(self, command):
        now = self.watchdog_time()
        sensors_fresh = self.sensors_fresh(now)
        values = (
            command.linear.x, command.linear.y, command.linear.z,
            command.angular.x, command.angular.y, command.angular.z,
        )
        output = Twist()
        if (
            not all(math.isfinite(value) for value in values)
            or any(value != 0 for value in values[1:3] + values[3:5])
        ):
            self.get_logger().error("Rejected non-finite or nonplanar Nav2 /cmd_vel; stopping Carter")
        elif self.stop_latched or (
            self.last_correction is None
            or not 0 <= now - self.last_correction < 4.0
        ):
            if not self.stop_latched and (
                self.last_safety_warning is None
                or now < self.last_safety_warning
                or now - self.last_safety_warning > 5
            ):
                self.get_logger().warning("3D localization correction stale; stopping Carter")
                self.last_safety_warning = now
        elif sensors_fresh:
            output.linear.x = max(-0.75, min(0.75, command.linear.x))
            output.angular.z = max(-0.7, min(0.7, command.angular.z))
            if (output.linear.x != command.linear.x or output.angular.z != command.angular.z) and (
                self.last_speed_warning is None
                or now < self.last_speed_warning
                or now - self.last_speed_warning > 5
            ):
                self.get_logger().warning(
                    "Clamped Nav2 /cmd_vel to Carter limits (0.75 m/s, 0.7 rad/s)"
                )
                self.last_speed_warning = now
        self.publisher.publish(output)


def main():
    rclpy.init()
    node = CmdVelSafety()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
