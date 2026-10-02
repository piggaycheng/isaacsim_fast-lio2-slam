#!/usr/bin/env python3
"""Fail-closed, acknowledged Humble surround profiles with downstream speed limits."""

import math

import rclpy
from geometry_msgs.msg import Point32, PolygonStamped, Twist, TwistStamped
from nav_msgs.msg import Odometry
from rcl_interfaces.msg import Parameter as ParameterMessage, ParameterType, ParameterValue
from rcl_interfaces.srv import GetParameters, SetParametersAtomically
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data


class AdaptiveSurround(Node):
    def __init__(self, **kwargs):
        super().__init__("adaptive_surround", **kwargs)
        self.full_points = self.declare_parameter(
            "full_points", [1.10, 0.75, 1.10, -0.75, -0.80, -0.75, -0.80, 0.75],
        ).value
        self.crawl_points = self.declare_parameter(
            "crawl_points", [0.90, 0.55, 0.90, -0.55, -0.45, -0.55, -0.45, 0.55],
        ).value
        self.crawl_linear = self.declare_parameter("crawl_linear", 0.10).value
        self.crawl_angular = self.declare_parameter("crawl_angular", 0.20).value
        self.motion_margin = self.declare_parameter("motion_margin", 0.02).value
        self.shrink_hold = self.declare_parameter("shrink_hold", 0.5).value
        self.odom_timeout = self.declare_parameter("odom_timeout", 0.3).value
        self.command_timeout = self.declare_parameter("command_timeout", 0.3).value
        self.switch_timeout = self.declare_parameter("switch_timeout", 1.0).value
        self.physical_footprint = self.declare_parameter(
            "physical_footprint", [0.65, 0.32, 0.65, -0.32, -0.20, -0.32, -0.20, 0.32],
        ).value
        self.footprint_padding = self.declare_parameter("footprint_padding", 0.01).value
        # Full-profile limits; must match cmd_vel_safety's max_linear/angular_speed.
        self.max_linear_speed = self.declare_parameter("max_linear_speed", 0.75).value
        self.max_angular_speed = self.declare_parameter("max_angular_speed", 0.7).value
        try:
            self.validate_configuration()
        except ValueError:
            self.destroy_node()
            raise
        self.command = None
        self.command_stamp = None
        self.motion = None
        self.odom_stamp = None
        self.previous_pose = None
        self.pending_odometry = None
        self.shrink_since = None
        self.last_time = None
        self.active = None
        self.checked = False
        self.future = None
        self.target = None
        self.request_stamp = None
        self.zero_barrier_stamp = None
        self.gate_ack_stamp = None
        self.publisher = self.create_publisher(Twist, "nav2/cmd_vel_adaptive", 10)
        self.limits = self.create_publisher(TwistStamped, "navigation/adaptive_surround_limits", 10)
        self.polygon = self.create_publisher(PolygonStamped, "collision_monitor/polygon_surround", 10)
        self.getter = self.create_client(GetParameters, "collision_monitor/get_parameters")
        self.setter = self.create_client(SetParametersAtomically, "collision_monitor/set_parameters_atomically")
        self.create_subscription(Twist, "nav2/cmd_vel", self.on_command, 10)
        self.create_subscription(Odometry, "odometry/local", self.on_odometry, qos_profile_sensor_data)
        self.create_subscription(
            TwistStamped, "navigation/adaptive_surround_limits_ack", self.on_gate_ack, 10,
        )
        self.create_timer(0.05, self.tick)

    def validate_configuration(self):
        for name in ("crawl_linear", "crawl_angular", "motion_margin", "shrink_hold",
                     "odom_timeout", "command_timeout", "switch_timeout",
                     "max_linear_speed", "max_angular_speed"):
            if not math.isfinite(getattr(self, name)) or getattr(self, name) <= 0:
                raise ValueError(f"{name} must be finite and positive")
        if self.crawl_linear > 0.10 or self.crawl_angular > 0.20 or self.motion_margin > 0.02:
            raise ValueError("Crawl limits exceed the experimental Carter profile bounds")
        if self.shrink_hold < 0.5 or self.odom_timeout > 0.3 or self.command_timeout > 0.3:
            raise ValueError("Adaptive dwell/freshness exceed the experimental Carter profile bounds")
        if not math.isfinite(self.footprint_padding) or self.footprint_padding < 0:
            raise ValueError("Footprint padding must be finite and nonnegative")
        for points in (self.full_points, self.crawl_points, self.physical_footprint):
            if len(points) != 8 or not all(math.isfinite(value) for value in points):
                raise ValueError("Surround profiles must have four finite vertices")
            xs, ys = points[::2], points[1::2]
            if min(xs) == max(xs) or min(ys) == max(ys) or list(zip(xs, ys)) != [
                (max(xs), max(ys)), (max(xs), min(ys)),
                (min(xs), min(ys)), (min(xs), max(ys)),
            ]:
                raise ValueError("Surround profiles must be clockwise axis-aligned rectangles")
        low_x, low_y = self.crawl_points[::2], self.crawl_points[1::2]
        xs, ys = self.physical_footprint[::2], self.physical_footprint[1::2]
        longitudinal = self.footprint_padding + 0.24
        lateral = self.footprint_padding + 0.22
        if (min(low_x) > min(xs) - longitudinal + 1e-9
                or max(low_x) < max(xs) + longitudinal - 1e-9
                or min(low_y) > min(ys) - lateral + 1e-9
                or max(low_y) < max(ys) + lateral - 1e-9):
            raise ValueError("Crawl must retain 24 cm fore/aft and 22 cm sideways beyond the padded footprint")
        if (min(self.full_points[::2]) > min(low_x) or max(self.full_points[::2]) < max(low_x)
                or min(self.full_points[1::2]) > min(low_y)
                or max(self.full_points[1::2]) < max(low_y)):
            raise ValueError("Full profile must enclose the crawl profile")

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def on_command(self, message):
        values = (message.linear.x, message.linear.y, message.linear.z,
                  message.angular.x, message.angular.y, message.angular.z)
        if not all(math.isfinite(value) for value in values) or any(values[i] != 0 for i in (1, 2, 3, 4)):
            self.command = None
            self.get_logger().error("Invalid adaptive input command; stopping")
            return
        self.command = message
        self.command_stamp = self.now()

    def on_odometry(self, message):
        stamp = message.header.stamp.sec + message.header.stamp.nanosec * 1e-9
        age = self.now() - stamp
        pose, velocity = message.pose.pose, message.twist.twist
        q = pose.orientation
        values = (pose.position.x, pose.position.y, q.x, q.y, q.z, q.w,
                  velocity.linear.x, velocity.linear.y, velocity.angular.z)
        if (message.header.frame_id != "odom" or message.child_frame_id != "base_link"
                or not -0.05 <= age < self.odom_timeout
                or not all(math.isfinite(value) for value in values)
                or abs(sum(value * value for value in (q.x, q.y, q.z, q.w)) - 1) > 0.01):
            self.motion = None
            self.previous_pose = None
            self.pending_odometry = None
            self.get_logger().warning("Rejected stale or invalid adaptive odometry; stopping")
            return
        # DDS may deliver odometry before this node's matching /clock update.
        if age < 0:
            self.pending_odometry = message
            return
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        current = (stamp, pose.position.x, pose.position.y, yaw)
        if self.previous_pose is not None:
            previous = self.previous_pose
            dt = stamp - previous[0]
            if dt <= 0:
                if self.motion is not None:
                    self.motion = (
                        max(self.motion[0], math.hypot(velocity.linear.x, velocity.linear.y)),
                        max(self.motion[1], abs(velocity.angular.z)),
                    )
                return
            if dt >= self.odom_timeout:
                self.motion = None
                self.shrink_since = None
            else:
                linear = math.hypot(current[1] - previous[1], current[2] - previous[2]) / dt
                angular = abs(math.atan2(math.sin(yaw - previous[3]), math.cos(yaw - previous[3]))) / dt
                self.motion = (max(linear, math.hypot(velocity.linear.x, velocity.linear.y)),
                               max(angular, abs(velocity.angular.z)))
        self.previous_pose = current
        self.odom_stamp = stamp

    def on_gate_ack(self, message):
        stamp = message.header.stamp.sec + message.header.stamp.nanosec * 1e-9
        if (message.header.frame_id == "base_link" and message.twist == Twist()
                and -0.05 <= self.now() - stamp < 0.2):
            self.gate_ack_stamp = stamp

    def requested_profile(self, now):
        # Nav2 behavior speed fields are float32; forwarded commands stay hard-capped.
        small = (abs(self.command.linear.x) <= self.crawl_linear + 1e-6
                 and abs(self.command.angular.z) <= self.crawl_angular + 1e-6
                 and self.motion[0] <= self.crawl_linear + self.motion_margin
                 and self.motion[1] <= self.crawl_angular + self.motion_margin)
        if not small:
            self.shrink_since = None
            return "full"
        if self.active == "crawl":
            return "crawl"
        if self.shrink_since is None:
            self.shrink_since = now
        return "crawl" if now - self.shrink_since >= self.shrink_hold else "full"

    def publish(self, ready):
        limits = TwistStamped()
        limits.header.frame_id = "base_link"
        limits.header.stamp = self.get_clock().now().to_msg()
        output = Twist()
        if ready:
            self.zero_barrier_stamp = None
            limits.twist.linear.x, limits.twist.angular.z = (
                (self.crawl_linear, self.crawl_angular) if self.active == "crawl"
                else (self.max_linear_speed, self.max_angular_speed)
            )
            output.linear.x = max(-limits.twist.linear.x, min(limits.twist.linear.x, self.command.linear.x))
            output.angular.z = max(-limits.twist.angular.z, min(limits.twist.angular.z, self.command.angular.z))
        elif self.zero_barrier_stamp is None:
            self.zero_barrier_stamp = self.now()
        self.limits.publish(limits)
        self.publisher.publish(output)
        if self.active is not None:
            message = PolygonStamped()
            message.header = limits.header
            points = self.crawl_points if self.active == "crawl" else self.full_points
            message.polygon.points = [Point32(x=float(x), y=float(y)) for x, y in zip(points[::2], points[1::2])]
            self.polygon.publish(message)

    def switch(self, target, now):
        request = SetParametersAtomically.Request(parameters=[
            ParameterMessage(name=name + ".enabled", value=ParameterValue(
                type=ParameterType.PARAMETER_BOOL, bool_value=enabled,
            )) for name, enabled in (("PolygonSurround", target == "full"),
                                     ("PolygonSurroundCrawl", target == "crawl"))
        ])
        self.future = self.setter.call_async(request)
        self.target = target
        self.request_stamp = now

    def tick(self):
        now = self.now()
        if self.last_time is not None and now < self.last_time:
            self.command = None
            self.motion = None
            self.previous_pose = None
            self.pending_odometry = None
            self.shrink_since = None
            self.checked = False
            self.zero_barrier_stamp = None
            self.gate_ack_stamp = None
            self.get_logger().warning("Clock reset; adaptive motion evidence cleared")
        self.last_time = now
        if self.pending_odometry is not None:
            message = self.pending_odometry
            stamp = message.header.stamp.sec + message.header.stamp.nanosec * 1e-9
            if stamp <= now:
                self.pending_odometry = None
                self.on_odometry(message)
        if self.future is not None:
            self.publish(False)
            if not self.future.done():
                if now - self.request_stamp > self.switch_timeout:
                    raise RuntimeError("Adaptive polygon parameter request timed out; navigation must stop")
                return
            response = self.future.result()
            self.future = None
            if self.target is None:
                values = response.values
                if len(values) != 4 or any(v.type == ParameterType.PARAMETER_NOT_SET for v in values):
                    self.get_logger().warning("Waiting for collision monitor profile configuration")
                    return
                for value, expected in zip(values[:2], (self.full_points, self.crawl_points)):
                    if (value.type != ParameterType.PARAMETER_DOUBLE_ARRAY
                            or len(value.double_array_value) != len(expected)
                            or any(abs(a - b) > 1e-9 for a, b in zip(value.double_array_value, expected))):
                        raise RuntimeError("Collision monitor surround profile geometry mismatch")
                if any(value.type != ParameterType.PARAMETER_BOOL for value in values[2:]):
                    raise RuntimeError("Collision monitor profile enable parameters have invalid types")
                self.active = "full" if values[2].bool_value and not values[3].bool_value else (
                    "crawl" if values[3].bool_value and not values[2].bool_value else None
                )
                self.checked = True
            else:
                if not response.result.successful:
                    raise RuntimeError("Adaptive profile switch rejected: " + response.result.reason)
                self.active = self.target
                self.get_logger().info(f"Adaptive surround active: {self.active}")
        if not self.checked:
            self.publish(False)
            if self.getter.service_is_ready():
                self.future = self.getter.call_async(GetParameters.Request(names=[
                    "PolygonSurround.points", "PolygonSurroundCrawl.points",
                    "PolygonSurround.enabled", "PolygonSurroundCrawl.enabled",
                ]))
                self.target = None
                self.request_stamp = now
            return
        healthy = (self.command is not None and self.motion is not None
                   and 0 <= now - self.command_stamp < self.command_timeout
                   and 0 <= now - self.odom_stamp < self.odom_timeout)
        target = self.requested_profile(now) if healthy else "full"
        if not healthy:
            self.shrink_since = None
        if target != self.active:
            self.publish(False)
            # The final gate must acknowledge zero before the native zone changes.
            if (self.setter.service_is_ready() and self.gate_ack_stamp is not None
                    and self.gate_ack_stamp >= self.zero_barrier_stamp
                    and 0 <= now - self.gate_ack_stamp < 0.2):
                if target == "full":
                    self.get_logger().info(
                        f"Expanding surround: measured={self.motion}, "
                        f"requested={None if self.command is None else (self.command.linear.x, self.command.angular.z)}"
                    )
                self.switch(target, now)
            return
        self.publish(healthy)


def main():
    rclpy.init()
    node = AdaptiveSurround()
    try:
        rclpy.spin(node)
    finally:
        if rclpy.ok():
            node.publish(False)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
