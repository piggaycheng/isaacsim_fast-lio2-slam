#!/usr/bin/env python3

import time

import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener


class CostmapReadiness(Node):
    def __init__(self):
        super().__init__("costmap_readiness")
        self.map_received = False
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.create_subscription(
            OccupancyGrid, "/map", self.on_map,
            QoSProfile(
                depth=1,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
                reliability=ReliabilityPolicy.RELIABLE,
            ),
        )

    def on_map(self, message):
        self.map_received = message.info.width > 0 and message.info.height > 0

    def ready(self):
        if not self.map_received:
            return False
        try:
            transform = self.buffer.lookup_transform("map", "base_link", Time())
        except TransformException:
            return False
        stamp = transform.header.stamp.sec + transform.header.stamp.nanosec * 1e-9
        now = self.get_clock().now().nanoseconds * 1e-9
        return stamp > 0 and 0 <= now - stamp < 2.0


def main():
    rclpy.init()
    node = CostmapReadiness()
    started = time.monotonic()
    try:
        while rclpy.ok() and time.monotonic() - started < 300:
            rclpy.spin_once(node, timeout_sec=0.2)
            if node.ready():
                node.get_logger().info("Map and recent map -> base_link TF ready for costmaps")
                return
        raise RuntimeError("Timed out waiting for /map and recent map -> base_link TF")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
