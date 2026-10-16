"""ROS integration: independent monitoring maps, without starting robot stacks."""

import json
import subprocess
import tempfile
import time
import unittest
from pathlib import Path

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.qos import DurabilityPolicy, QoSProfile
from visualization_msgs.msg import Marker, MarkerArray

from tests.test_fleet_ground_truth import session


class FleetFloorLaunchTest(unittest.TestCase):
    def test_launch_publishes_both_floor_maps_without_robot_map_servers(self):
        self.check_launch(simulation_reference=True)

    def test_auto_mode_does_not_publish_simulation_reference(self):
        self.check_launch(simulation_reference=False)

    def check_launch(self, simulation_reference):
        rclpy.init()
        node = rclpy.create_node("fleet_floor_maps_test")
        process = None
        try:
            with tempfile.TemporaryDirectory(prefix="fleet_floor_maps_") as directory:
                root = Path(directory)
                for floor, pixel in (("1F", 254), ("B1", 0), ("office", 128)):
                    path = root / floor
                    path.mkdir()
                    (path / "map_2d.pgm").write_bytes(b"P5\n2 1\n255\n" + bytes([pixel, pixel]))
                    (path / "map_2d.yaml").write_text(
                        "image: map_2d.pgm\nresolution: 0.1\norigin: [0, 0, 0]\n"
                        "negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n"
                    )
                    (path / "session.json").write_text(json.dumps(
                        session(floor, -3.3 if floor == "B1" else 0.0)))
                received = {}
                subscriptions = [
                    node.create_subscription(
                        OccupancyGrid, f"/fleet/maps/floor_{index}/map",
                        lambda msg, index=index: received.__setitem__(index, msg),
                        QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
                    for index in range(2)
                ]
                references = {}
                subscriptions.extend([
                    node.create_subscription(
                        MarkerArray, f"/fleet/maps/floor_{index}/simulation_reference",
                        lambda msg, index=index: references.__setitem__(index, msg),
                        QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
                    for index in range(2)
                ])
                truth = node.create_publisher(Odometry, "/floorprobe/isaac/ground_truth/odom", 10)
                second_truth = node.create_publisher(
                    Odometry, "/floorprobe2/isaac/ground_truth/odom", 10)
                with tempfile.TemporaryFile(mode="w+") as log:
                    process = subprocess.Popen([
                        "ros2", "launch", "slam_localization_3d", "fleet_rviz.launch.py",
                        "robots:=floorprobe@10,20,1.5707963267948966;"
                        "floorprobe2@13.5,20,1.5707963267948966",
                        f"maps_root:={root}", "initial_floor:=B1",
                        "use_sim_time:=false", "rviz:=false",
                        f"simulation_reference:={str(simulation_reference).lower()}",
                    ], stdout=log, stderr=subprocess.STDOUT)
                    try:
                        deadline = time.monotonic() + 30.0
                        while len(received) < 2 and time.monotonic() < deadline:
                            if process.poll() is not None:
                                break
                            rclpy.spin_once(node, timeout_sec=0.1)
                        log.seek(0)
                        self.assertEqual(len(received), 2, log.read())
                        self.assertEqual(list(received[0].data), [100, 100])
                        self.assertEqual(list(received[1].data), [0, 0])
                        for message in received.values():
                            self.assertEqual(message.header.frame_id, "map")
                            self.assertEqual((message.info.width, message.info.height), (2, 1))
                        self.assertEqual(node.count_publishers("/floorprobe/map"), 0)
                        self.assertEqual(node.count_publishers("/fleet/maps/floor_2/map"), 0)
                        self.assertEqual(node.count_publishers("/floorprobe/navigation/floor_request"), 0)
                        self.assertEqual(len(subscriptions), 4)
                        if not simulation_reference:
                            self.assertEqual(node.count_publishers(
                                "/fleet/maps/floor_1/simulation_reference"), 0)
                            self.assertEqual(truth.get_subscription_count(), 0)
                            return
                        deadline = time.monotonic() + 10.0
                        while (not truth.get_subscription_count()
                               or not second_truth.get_subscription_count()) and time.monotonic() < deadline:
                            rclpy.spin_once(node, timeout_sec=0.1)
                        self.assertGreater(truth.get_subscription_count(), 0)
                        self.assertGreater(second_truth.get_subscription_count(), 0)
                        message = Odometry()
                        message.header.frame_id = "isaac_world"
                        message.pose.pose.position.x, message.pose.pose.position.y = 3.0, -2.0
                        message.pose.pose.orientation.w = 1.0
                        truth.publish(message)
                        second_truth.publish(message)
                        deadline = time.monotonic() + 10.0
                        while time.monotonic() < deadline:
                            truth.publish(message)
                            second_truth.publish(message)
                            rclpy.spin_once(node, timeout_sec=0.1)
                            if 1 in references and sum(
                                    marker.action == Marker.ADD
                                    for marker in references[1].markers) == 4:
                                break
                        self.assertIn(1, references)
                        arrows = [marker for marker in references[1].markers
                                  if marker.action == Marker.ADD and marker.type == Marker.ARROW]
                        log.seek(0)
                        self.assertEqual(len(arrows), 2, log.read())
                        self.assertAlmostEqual(arrows[0].pose.position.x, 1.0)
                        self.assertAlmostEqual(arrows[0].pose.position.y, 3.0)
                        self.assertAlmostEqual(arrows[0].pose.orientation.z, 2 ** -0.5)
                        self.assertAlmostEqual(
                            arrows[1].pose.position.x - arrows[0].pose.position.x, 3.5)
                        self.assertAlmostEqual(
                            arrows[1].pose.position.y - arrows[0].pose.position.y, 0.0)
                        self.assertEqual(node.count_publishers("/floorprobe/initialpose"), 0)
                        self.assertEqual(node.count_publishers("/floorprobe/goal_pose"), 0)
                        manual = node.create_publisher(
                            PoseWithCovarianceStamped, "/floorprobe/initialpose", 10)
                        deadline = time.monotonic() + 5.0
                        while manual.get_subscription_count() == 0 and time.monotonic() < deadline:
                            rclpy.spin_once(node, timeout_sec=0.1)
                        self.assertGreater(manual.get_subscription_count(), 0)
                        initial = PoseWithCovarianceStamped()
                        initial.header.frame_id = "map"
                        initial.pose.pose.orientation.w = 1.0
                        manual.publish(initial)
                        while time.monotonic() < deadline:
                            truth.publish(message)
                            second_truth.publish(message)
                            rclpy.spin_once(node, timeout_sec=0.1)
                            arrows = [marker for marker in references[1].markers
                                      if marker.action == Marker.ADD and marker.type == Marker.ARROW]
                            if len(arrows) == 1:
                                break
                        self.assertEqual(len(arrows), 1)
                        self.assertEqual(arrows[0].ns, "simulation_reference/floorprobe2")
                    finally:
                        process.terminate()
                        process.wait(timeout=15)
                        process = None
        finally:
            if process is not None and process.poll() is None:
                process.terminate()
                process.wait(timeout=15)
            node.destroy_node()
            rclpy.shutdown()


if __name__ == "__main__":
    unittest.main()
