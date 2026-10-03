#!/usr/bin/env python3
"""Merge per-robot TF trees and robot-frame displays for one fleet RViz.

Each robot keeps its own TF on /<ns>/tf with unprefixed frames. This relay
republishes them on /fleet/tf(_static) as <ns>/<frame>, keeping the shared
`map` frame, so one RViz (with /tf remapped to /fleet/tf) shows every robot.
Topics stamped in robot frames are republished on /fleet/<ns>/<topic> with
prefixed frame ids; map-frame topics need no relay.
"""

import rclpy
from geometry_msgs.msg import PolygonStamped
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data,
)
from sensor_msgs.msg import LaserScan, PointCloud2
from tf2_msgs.msg import TFMessage

SHARED_FRAMES = frozenset({"map"})
RELAYED_TOPICS = (
    ("scan", LaserScan),
    ("perception/obstacles", PointCloud2),
    ("collision_monitor/polygon_stop", PolygonStamped),
    ("collision_monitor/polygon_surround", PolygonStamped),
    ("collision_monitor/polygon_slowdown", PolygonStamped),
)
STATIC_QOS = QoSProfile(
    depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE,
)


def prefix_frame(namespace, frame):
    frame = frame.lstrip("/")
    if not frame or frame in SHARED_FRAMES or frame.startswith(f"{namespace}/"):
        return frame
    return f"{namespace}/{frame}"


def prefix_transforms(namespace, message):
    for transform in message.transforms:
        transform.header.frame_id = prefix_frame(namespace, transform.header.frame_id)
        transform.child_frame_id = prefix_frame(namespace, transform.child_frame_id)
    return message


class FleetRelay(Node):
    def __init__(self):
        super().__init__("fleet_relay")
        robots = self.declare_parameter("robots", [""]).value
        robots = [name.strip().strip("/") for name in robots if name.strip().strip("/")]
        if not robots:
            raise ValueError("fleet_relay needs at least one robot namespace")
        self.tf_publisher = self.create_publisher(TFMessage, "/fleet/tf", 100)
        self.static_publisher = self.create_publisher(TFMessage, "/fleet/tf_static", STATIC_QOS)
        self.static_transforms = {}
        self.relays = []
        for namespace in robots:
            self.create_subscription(
                TFMessage, f"/{namespace}/tf",
                lambda message, ns=namespace: self.tf_publisher.publish(
                    prefix_transforms(ns, message)), 100)
            self.create_subscription(
                TFMessage, f"/{namespace}/tf_static",
                lambda message, ns=namespace: self.on_static(ns, message), STATIC_QOS)
            for topic, kind in RELAYED_TOPICS:
                publisher = self.create_publisher(kind, f"/fleet/{namespace}/{topic}", 10)
                self.relays.append(publisher)
                self.create_subscription(
                    kind, f"/{namespace}/{topic}",
                    lambda message, ns=namespace, out=publisher: self.relay(ns, out, message),
                    qos_profile_sensor_data)
        self.get_logger().info(f"Relaying TF and displays of {', '.join(robots)} to /fleet")

    def on_static(self, namespace, message):
        # A latched publisher keeps one message, so republish the merged set.
        for transform in prefix_transforms(namespace, message).transforms:
            self.static_transforms[transform.child_frame_id] = transform
        self.static_publisher.publish(TFMessage(transforms=list(self.static_transforms.values())))

    @staticmethod
    def relay(namespace, publisher, message):
        message.header.frame_id = prefix_frame(namespace, message.header.frame_id)
        publisher.publish(message)


def main():
    rclpy.init()
    node = FleetRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
