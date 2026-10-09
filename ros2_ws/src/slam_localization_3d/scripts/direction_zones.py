#!/usr/bin/env python3
"""Direction-aware collision_monitor zone sets for Nav2 Humble.

Humble stop polygons ignore the command direction, so an obstacle in front also
blocks reversing and in-place rotation. Like field-set switching on safety laser
scanners, this node enables the forward, reverse or rotate zone set matching the
command, toggling the native ``<polygon>.enabled`` parameters atomically and only
after a zero-command barrier with measured standstill.
"""

import math

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rcl_interfaces.msg import Parameter as ParameterMessage, ParameterType, ParameterValue
from rcl_interfaces.srv import GetParameters, SetParametersAtomically
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

# Keep in sync with configure_direction_zones() in global_fusion.launch.py.
ZONE_SETS = {
    "forward": ("PolygonStop", "PolygonSlow", "PolygonSurroundForward"),
    "reverse": ("PolygonSurroundReverse",),
    "rotate": ("PolygonRotate",),
}
POLYGONS = tuple(name for names in ZONE_SETS.values() for name in names)


class DirectionZones(Node):
    def __init__(self, **kwargs):
        super().__init__("direction_zones", **kwargs)
        self.linear_deadband = self.declare_parameter("linear_deadband", 0.01).value
        self.angular_deadband = self.declare_parameter("angular_deadband", 0.02).value
        self.standstill_linear = self.declare_parameter("standstill_linear", 0.03).value
        self.standstill_angular = self.declare_parameter("standstill_angular", 0.05).value
        self.settle_time = self.declare_parameter("settle_time", 0.2).value
        self.odom_timeout = self.declare_parameter("odom_timeout", 0.3).value
        self.command_timeout = self.declare_parameter("command_timeout", 0.3).value
        self.switch_timeout = self.declare_parameter("switch_timeout", 1.0).value
        for name in ("linear_deadband", "angular_deadband", "standstill_linear", "standstill_angular",
                     "settle_time", "odom_timeout", "command_timeout", "switch_timeout"):
            value = getattr(self, name)
            if not isinstance(value, (int, float)) or not math.isfinite(value) or value <= 0:
                self.destroy_node()
                raise ValueError(f"{name} must be finite and positive")
        self.command = None
        self.command_stamp = None
        self.motion = None
        self.odom_stamp = None
        self.previous_pose = None
        self.last_time = None
        self.active = None
        self.checked = False
        self.future = None
        self.target = None
        self.request_stamp = None
        self.barrier_since = None
        self.publisher = self.create_publisher(Twist, "nav2/cmd_vel_direction", 10)
        self.getter = self.create_client(GetParameters, "collision_monitor/get_parameters")
        self.setter = self.create_client(SetParametersAtomically, "collision_monitor/set_parameters_atomically")
        self.create_subscription(Twist, "nav2/cmd_vel", self.on_command, 10)
        self.create_subscription(Odometry, "odometry/local", self.on_odometry, qos_profile_sensor_data)
        self.create_timer(0.05, self.tick)

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def requested_mode(self, command):
        if command.linear.x > self.linear_deadband:
            return "forward"
        if command.linear.x < -self.linear_deadband:
            return "reverse"
        if abs(command.angular.z) > self.angular_deadband:
            return "rotate"
        return None

    def limited(self, command):
        output = Twist()
        if self.active is None or self.future is not None or self.requested_mode(command) not in (None, self.active):
            return output
        if self.active == "forward":
            output.linear.x = max(0.0, command.linear.x)
        elif self.active == "reverse":
            output.linear.x = min(0.0, command.linear.x)
        if self.active != "rotate" or abs(command.angular.z) > self.angular_deadband:
            output.angular.z = command.angular.z
        return output

    def on_command(self, message):
        values = (message.linear.x, message.linear.y, message.linear.z,
                  message.angular.x, message.angular.y, message.angular.z)
        if not all(math.isfinite(value) for value in values) or any(values[i] != 0 for i in (1, 2, 3, 4)):
            self.command = None
            self.publisher.publish(Twist())
            self.get_logger().error("Invalid direction-zone input command; stopping")
            return
        self.command = message
        self.command_stamp = self.now()
        self.publisher.publish(self.limited(message))

    def on_odometry(self, message):
        stamp = message.header.stamp.sec + message.header.stamp.nanosec * 1e-9
        age = self.now() - stamp
        pose, velocity = message.pose.pose, message.twist.twist
        q = pose.orientation
        values = (pose.position.x, pose.position.y, q.x, q.y, q.z, q.w,
                  velocity.linear.x, velocity.linear.y, velocity.angular.z)
        if (message.header.frame_id != "odom" or message.child_frame_id != "base_link"
                or not -0.05 <= age < self.odom_timeout
                or not all(math.isfinite(value) for value in values)):
            self.motion = None
            self.previous_pose = None
            return
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        linear = math.hypot(velocity.linear.x, velocity.linear.y)
        angular = abs(velocity.angular.z)
        # Pose differences catch a filter twist that lags the actual motion.
        if self.previous_pose is not None:
            dt = stamp - self.previous_pose[0]
            if dt <= 0:
                return
            if dt < self.odom_timeout:
                previous = self.previous_pose
                linear = max(linear, math.hypot(pose.position.x - previous[1], pose.position.y - previous[2]) / dt)
                angular = max(angular, abs(math.atan2(math.sin(yaw - previous[3]), math.cos(yaw - previous[3]))) / dt)
        self.previous_pose = (stamp, pose.position.x, pose.position.y, yaw)
        self.motion = (linear, angular)
        self.odom_stamp = stamp

    def standstill(self, now):
        return (self.motion is not None and self.odom_stamp is not None
                and self.odom_stamp >= self.barrier_since
                and 0 <= now - self.odom_stamp < self.odom_timeout
                and self.motion[0] <= self.standstill_linear
                and self.motion[1] <= self.standstill_angular)

    def switch(self, target, now):
        self.future = self.setter.call_async(SetParametersAtomically.Request(parameters=[
            ParameterMessage(name=name + ".enabled", value=ParameterValue(
                type=ParameterType.PARAMETER_BOOL, bool_value=name in ZONE_SETS[target],
            )) for name in POLYGONS
        ]))
        self.target = target
        self.request_stamp = now

    def finish_request(self):
        response = self.future.result()
        self.future = None
        if self.target is None:
            values = response.values
            if len(values) != len(POLYGONS) or any(v.type == ParameterType.PARAMETER_NOT_SET for v in values):
                self.get_logger().warning("Waiting for collision monitor direction zones")
                return
            if any(value.type != ParameterType.PARAMETER_BOOL for value in values):
                raise RuntimeError("Collision monitor zone enable parameters have invalid types")
            enabled = {name for name, value in zip(POLYGONS, values) if value.bool_value}
            self.active = next((mode for mode, names in ZONE_SETS.items() if enabled == set(names)), None)
            self.checked = True
            self.get_logger().info(f"Direction zones active: {self.active}")
            return
        if not response.result.successful:
            raise RuntimeError("Direction zone switch rejected: " + response.result.reason)
        self.active = self.target
        self.get_logger().info(f"Direction zones active: {self.active}")

    def tick(self):
        now = self.now()
        if self.last_time is not None and now < self.last_time:
            self.command = None
            self.motion = None
            self.previous_pose = None
            self.odom_stamp = None
            self.barrier_since = None
            self.get_logger().warning("Clock reset; direction-zone motion evidence cleared")
        self.last_time = now
        if self.future is not None:
            self.publisher.publish(Twist())
            if not self.future.done():
                if now - self.request_stamp > self.switch_timeout:
                    raise RuntimeError("Direction zone parameter request timed out; navigation must stop")
                return
            self.finish_request()
        if not self.checked:
            if self.getter.service_is_ready():
                self.future = self.getter.call_async(GetParameters.Request(
                    names=[name + ".enabled" for name in POLYGONS],
                ))
                self.target = None
                self.request_stamp = now
            return
        fresh = self.command is not None and 0 <= now - self.command_stamp < self.command_timeout
        target = self.requested_mode(self.command) if fresh else None
        if self.active is None:
            target = target or "forward"
        if target is None or target == self.active:
            self.barrier_since = None
            return
        # Field-set switching: zones change only while the robot is commanded and measured stopped.
        self.publisher.publish(Twist())
        if self.barrier_since is None:
            self.barrier_since = now
        if now - self.barrier_since >= self.settle_time and self.standstill(now) and self.setter.service_is_ready():
            self.switch(target, now)


def main():
    rclpy.init()
    node = DirectionZones()
    try:
        rclpy.spin(node)
    finally:
        if rclpy.ok():
            node.publisher.publish(Twist())
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
