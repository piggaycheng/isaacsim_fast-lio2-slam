#!/usr/bin/env python3
"""Drive a repeatable covariance-calibration pattern on /cmd_vel.

Standstill periods measure sensor noise; straight moves, in-place spins and
arcs excite distance, turn and combined wheel errors. Every move is retraced
in reverse, so the robot returns near its start even when the achieved turn
rate differs from the command. The footprint stays within about
max(linear*straight_time, arc length) of the start, about 1 m with the
defaults. Only use it in a clear area, or in simulation.
"""

import math

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node


def pattern(cycles, linear=0.25, angular=0.4, spin=0.5, straight_time=4.0, arc_time=3.0):
    segments = [(40.0, 0.0, 0.0)]
    for _ in range(cycles):
        segments += [
            (5.0, 0.0, 0.0),
            (straight_time, linear, 0.0), (2.0, 0.0, 0.0),
            (straight_time, -linear, 0.0), (5.0, 0.0, 0.0),
            (2 * math.pi / spin, 0.0, spin), (3.0, 0.0, 0.0),
            (2 * math.pi / spin, 0.0, -spin), (5.0, 0.0, 0.0),
        ]
        for direction in (1.0, -1.0):
            segments += [
                (arc_time, linear, direction * angular), (2.0, 0.0, 0.0),
                (arc_time, -linear, -direction * angular), (3.0, 0.0, 0.0),
            ]
    segments.append((20.0, 0.0, 0.0))
    return segments


class CovarianceDrive(Node):
    def __init__(self):
        super().__init__("covariance_drive")
        cycles = int(self.declare_parameter("cycles", 4).value)
        linear = float(self.declare_parameter("linear_speed", 0.25).value)
        angular = float(self.declare_parameter("arc_angular_speed", 0.4).value)
        spin = float(self.declare_parameter("spin_speed", 0.5).value)
        # Match Isaac's /cmd_vel receiver limits.
        if not (0 < linear <= 0.75 and 0 < angular <= 0.5 and 0 < spin <= 0.5 and cycles > 0):
            raise ValueError("speeds must be positive and within 0.75 m/s / 0.5 rad/s")
        self.segments = pattern(cycles, linear, angular, spin)
        self.total = sum(duration for duration, _, _ in self.segments)
        self.publisher = self.create_publisher(Twist, "/cmd_vel", 10)
        self.start = None
        self.last_report = -1
        self.timer = self.create_timer(0.05, self.tick)
        self.get_logger().info(f"Calibration drive: {self.total:.0f} s of simulation time")

    def command(self, elapsed):
        for duration, linear, angular in self.segments:
            if elapsed < duration:
                return linear, angular
            elapsed -= duration
        return None

    def tick(self):
        now = self.get_clock().now().nanoseconds * 1e-9
        if now <= 0.0:
            return
        if self.start is None:
            self.start = now
        elapsed = now - self.start
        command = self.command(elapsed)
        message = Twist()
        if command is None:
            self.publisher.publish(message)
            self.get_logger().info("Calibration drive finished")
            raise SystemExit
        message.linear.x, message.angular.z = command
        self.publisher.publish(message)
        if int(elapsed) // 30 != self.last_report:
            self.last_report = int(elapsed) // 30
            self.get_logger().info(f"{elapsed:.0f}/{self.total:.0f} s")


def main():
    rclpy.init()
    node = CovarianceDrive()
    try:
        rclpy.spin(node)
    except SystemExit:
        pass
    finally:
        node.publisher.publish(Twist())
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
