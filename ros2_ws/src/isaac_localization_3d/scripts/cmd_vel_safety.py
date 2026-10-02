#!/usr/bin/env python3
"""Gate Nav2 commands before Carter's native ROS 2 /cmd_vel subscriber."""

import math

import rclpy
from geometry_msgs.msg import Twist, TwistStamped
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
        self.max_linear_accel = self.declare_parameter("max_linear_accel", 0.8).value
        self.max_angular_accel = self.declare_parameter("max_angular_accel", 1.5).value
        self.command_timeout = self.declare_parameter("command_timeout", 0.5).value
        for name in ("sensor_timeout", "max_linear_accel", "max_angular_accel", "command_timeout"):
            value = getattr(self, name)
            if not math.isfinite(value) or value <= 0:
                self.destroy_node()
                raise ValueError(f"{name} must be finite and positive")
        self.require_adaptive_limits = self.declare_parameter("require_adaptive_limits", False).value
        self.adaptive_timeout = self.declare_parameter("adaptive_timeout", 0.2).value
        if (not math.isfinite(self.adaptive_timeout) or self.adaptive_timeout <= 0
                or self.require_adaptive_limits and self.adaptive_timeout > 0.2):
            self.destroy_node()
            raise ValueError("adaptive_timeout must be positive and at most 0.2 s in adaptive mode")
        self.adaptive_limits = None
        self.last_adaptive_stamp = None
        self.pending_adaptive_limits = None
        self.adaptive_blocked = None
        self.sensor_stamps = {"scan": None, "obstacles": None}
        self.last_watchdog_time = None
        self.last_output = Twist()
        self.last_output_time = None
        self.last_command_time = None
        self.sensors_stale = None
        self.publisher = self.create_publisher(Twist, "cmd_vel", 10)
        self.adaptive_ack = self.create_publisher(
            TwistStamped, "navigation/adaptive_surround_limits_ack", 10,
        )
        self.create_subscription(
            LaserScan, "scan", lambda msg: self.on_sensor("scan", msg),
            qos_profile_sensor_data,
        )
        self.create_subscription(
            PointCloud2, "perception/obstacles",
            lambda msg: self.on_sensor("obstacles", msg), qos_profile_sensor_data,
        )
        self.create_subscription(
            Header, "localization_3d/accepted_correction", self.on_correction, 10
        )
        self.create_subscription(Bool, "navigation/emergency_stop", self.on_stop, 10)
        self.create_subscription(Twist, "nav2/cmd_vel", self.on_command, 10)
        self.create_subscription(TwistStamped, "navigation/adaptive_surround_limits", self.on_adaptive_limits, 10)
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
            self.last_output = Twist()
            self.last_output_time = None
            self.last_command_time = None
            self.adaptive_limits = None
            self.last_adaptive_stamp = None
            self.pending_adaptive_limits = None
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

    def on_adaptive_limits(self, message):
        now = self.watchdog_time()
        self.apply_pending_limits(now)
        stamp = message.header.stamp.sec + message.header.stamp.nanosec * 1e-9
        values = (message.twist.linear.x, message.twist.linear.y, message.twist.linear.z,
                  message.twist.angular.x, message.twist.angular.y, message.twist.angular.z)
        if (message.header.frame_id != "base_link" or not -0.05 <= now - stamp < self.adaptive_timeout
                or not all(math.isfinite(value) for value in values)
                or any(values[index] != 0 for index in (1, 2, 3, 4))
                or (values[0] == 0) != (values[5] == 0)
                or self.last_adaptive_stamp is not None and stamp < self.last_adaptive_stamp
                or not 0 <= values[0] <= 0.75 or not 0 <= values[5] <= 0.7):
            self.adaptive_limits = None
            self.pending_adaptive_limits = None
            self.get_logger().error("Invalid or stale adaptive limits; stopping Carter")
            if self.require_adaptive_limits:
                self.publish_output(Twist(), now)
            return
        if now < stamp:
            self.pending_adaptive_limits = message
            return
        self.adaptive_limits = (stamp, values[0], values[5])
        self.last_adaptive_stamp = stamp
        if self.require_adaptive_limits:
            # A limits message may stop motion, but must never replay an old command.
            if (message.twist == Twist() or abs(self.last_output.linear.x) > values[0]
                    or abs(self.last_output.angular.z) > values[5]):
                self.publish_output(Twist(), now)
            self.adaptive_ack.publish(message)

    def apply_pending_limits(self, now):
        if self.pending_adaptive_limits is not None:
            message = self.pending_adaptive_limits
            stamp = message.header.stamp.sec + message.header.stamp.nanosec * 1e-9
            if stamp <= now:
                self.pending_adaptive_limits = None
                self.on_adaptive_limits(message)

    def adaptive_ready(self, now):
        ready = not self.require_adaptive_limits or (
            self.adaptive_limits is not None
            and 0 <= now - self.adaptive_limits[0] < self.adaptive_timeout
            and self.adaptive_limits[1] > 0 and self.adaptive_limits[2] > 0
        )
        if self.require_adaptive_limits and self.adaptive_blocked != (not ready):
            self.get_logger().info("Adaptive limits ready" if ready else "Adaptive limits unavailable; stopping Carter")
            self.adaptive_blocked = not ready
        return ready

    def on_watchdog(self):
        now = self.watchdog_time()
        self.apply_pending_limits(now)
        if not self.sensors_fresh(now) or not self.adaptive_ready(now) or self.stop_latched or (
            self.last_correction is None
            or not 0 <= now - self.last_correction < 4.0
        ) or (
            self.last_command_time is not None
            and now - self.last_command_time >= self.command_timeout
        ):
            if self.last_output != Twist():
                self.get_logger().warning("Safety gate or command timeout; stopping Carter")
            self.publish_output(Twist(), now)

    def on_stop(self, message):
        if message.data:
            self.stop_latched = True
            self.get_logger().error("Navigation emergency stop latched; restart to re-enable driving")
            self.publish_output(Twist(), self.watchdog_time())

    def publish_output(self, output, now):
        self.last_output = output
        self.last_output_time = now
        self.publisher.publish(output)

    @staticmethod
    def limit_acceleration(previous, requested, step):
        # Braking must never be delayed, including braking to zero before reversal.
        if previous * requested < 0:
            return 0.0
        if abs(requested) <= abs(previous):
            return requested
        return math.copysign(min(abs(requested), abs(previous) + step), requested)

    def on_correction(self, message):
        self.watchdog_time()
        if message.frame_id != "map" or message.stamp.sec < 0:
            self.get_logger().error("Invalid 3D localization correction header")
            return
        self.last_correction = message.stamp.sec + message.stamp.nanosec * 1e-9

    def on_command(self, command):
        now = self.watchdog_time()
        if (
            self.last_command_time is None
            or now - self.last_command_time >= self.command_timeout
        ):
            self.last_output = Twist()
            self.last_output_time = now
        self.last_command_time = now
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
        elif sensors_fresh and self.adaptive_ready(now):
            linear_limit, angular_limit = (self.adaptive_limits[1:] if self.require_adaptive_limits else (0.75, 0.7))
            output.linear.x = max(-linear_limit, min(linear_limit, command.linear.x))
            output.angular.z = max(-angular_limit, min(angular_limit, command.angular.z))
            if (output.linear.x != command.linear.x or output.angular.z != command.angular.z) and (
                self.last_speed_warning is None
                or now < self.last_speed_warning
                or now - self.last_speed_warning > 5
            ):
                self.get_logger().warning(
                    f"Clamped Nav2 /cmd_vel to active limits ({linear_limit} m/s, {angular_limit} rad/s)"
                )
                self.last_speed_warning = now
            # Do not bank acceleration while stopped or between delayed commands.
            elapsed = (
                0.0 if self.last_output_time is None
                else max(0.0, min(0.1, now - self.last_output_time))
            )
            output.linear.x = self.limit_acceleration(
                self.last_output.linear.x, output.linear.x, self.max_linear_accel * elapsed
            )
            output.angular.z = self.limit_acceleration(
                self.last_output.angular.z, output.angular.z, self.max_angular_accel * elapsed
            )
        self.publish_output(output, now)


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
