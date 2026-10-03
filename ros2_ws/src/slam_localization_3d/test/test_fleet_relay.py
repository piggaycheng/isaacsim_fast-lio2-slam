import sys
import time
import unittest
from pathlib import Path

import rclpy
from geometry_msgs.msg import Point32, PolygonStamped
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))
from fleet_relay import FleetRelay


class FleetSurroundRelayTest(unittest.TestCase):
    def test_each_robot_relays_changing_surround_with_prefixed_frame(self):
        rclpy.init(args=["--ros-args", "-p", "robots:=['carter1', 'carter2']"])
        relay = None
        probe = None
        executor = SingleThreadedExecutor()
        try:
            relay = FleetRelay()
            probe = Node("fleet_surround_probe")
            executor.add_node(relay)
            executor.add_node(probe)
            received = {}
            publishers = {}
            subscriptions = []
            for name in ("carter1", "carter2"):
                publishers[name] = probe.create_publisher(
                    PolygonStamped, f"/{name}/collision_monitor/polygon_surround", 10)
                subscriptions.append(probe.create_subscription(
                    PolygonStamped, f"/fleet/{name}/collision_monitor/polygon_surround",
                    lambda message, ns=name: received.__setitem__(ns, message), 10))
            for extent in (0.8, 1.1, 0.5):
                received.clear()
                deadline = time.monotonic() + 5.0
                while time.monotonic() < deadline:
                    for index, (name, publisher) in enumerate(publishers.items()):
                        message = PolygonStamped()
                        message.header.frame_id = "base_link"
                        message.header.stamp = probe.get_clock().now().to_msg()
                        message.polygon.points = [
                            Point32(x=extent + index, y=0.5),
                            Point32(x=extent + index, y=-0.5),
                            Point32(x=-0.5, y=-0.5),
                            Point32(x=-0.5, y=0.5),
                        ]
                        publisher.publish(message)
                    executor.spin_once(timeout_sec=0.05)
                    if all(name in received and
                           abs(received[name].polygon.points[0].x - extent - index) < 1e-6
                           for index, name in enumerate(publishers)):
                        break
                self.assertEqual(set(received), set(publishers))
                for index, name in enumerate(publishers):
                    self.assertEqual(received[name].header.frame_id, f"{name}/base_link")
                    self.assertAlmostEqual(
                        received[name].polygon.points[0].x, extent + index, places=6)
        finally:
            executor.shutdown()
            if probe is not None:
                probe.destroy_node()
            if relay is not None:
                relay.destroy_node()
            rclpy.try_shutdown()
