import math
import subprocess
import sys
import tempfile
import time
import unittest
from pathlib import Path

import numpy as np
import rclpy
from ament_index_python.packages import get_package_prefix, get_package_share_directory
from geometry_msgs.msg import Twist
from lifecycle_msgs.srv import ChangeState
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan, PointCloud2
from std_msgs.msg import Header

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))
from xyz_cloud import make_point_cloud


class NearObstacleTest(unittest.TestCase):
    def setUp(self):
        rclpy.init()
        self.node = Node("near_obstacle_test")
        self.messages = {}
        self.processes = []
        self.logs = []
        self.addCleanup(self.cleanup)
        for topic, kind, key in [
            ("/test/self_filtered", PointCloud2, "self"),
            ("/test/obstacles", PointCloud2, "obstacles"),
            ("/test/scan", LaserScan, "scan"),
            ("/test/out", Twist, "out"),
        ]:
            self.node.create_subscription(
                kind, topic, lambda message, key=key: self.messages.__setitem__(key, message),
                qos_profile_sensor_data,
            )
        self.cloud_pub = self.node.create_publisher(
            PointCloud2, "/test/raw", qos_profile_sensor_data,
        )
        self.cmd_pub = self.node.create_publisher(Twist, "/test/in", 10)
        self.start("isaac_nav", "ground_obstacle_filter", [
            "-p", "input_topic:=/test/raw", "-p", "output_topic:=/test/obstacles",
            "-p", "self_filtered_topic:=/test/self_filtered",
        ])
        self.start("pointcloud_to_laserscan", "pointcloud_to_laserscan_node", [
            "-r", "__node:=near_scan", "-r", "cloud_in:=/test/self_filtered",
            "-r", "scan:=/test/scan", "-p", "target_frame:=base_link",
            "-p", "range_min:=0.0", "-p", "range_max:=20.0",
            "-p", "min_height:=0.1", "-p", "max_height:=2.0",
            "-p", "angle_increment:=0.008726646", "-p", "use_inf:=true",
        ])
        config = Path(get_package_share_directory("isaac_localization_3d")) / "config"
        self.start("nav2_collision_monitor", "collision_monitor", [
            "--params-file", str(config / "collision_monitor.yaml"),
            "-p", "use_sim_time:=false", "-p", "base_shift_correction:=false",
            "-p", "FootprintApproach.enabled:=false",
            "-r", "/perception/obstacles:=/test/obstacles", "-r", "/scan:=/test/scan",
            "-r", "/nav2/cmd_vel:=/test/in",
            "-r", "/nav2/cmd_vel_monitored:=/test/out",
        ])
        client = self.node.create_client(ChangeState, "/collision_monitor/change_state")
        self.assertTrue(client.wait_for_service(timeout_sec=10))
        for transition in (1, 3):
            request = ChangeState.Request()
            request.transition.id = transition
            future = client.call_async(request)
            self.wait(future.done)
            self.assertTrue(future.result().success)

    def start(self, package, executable, args):
        log = tempfile.TemporaryFile(mode="w+")
        self.logs.append(log)
        executable_path = Path(get_package_prefix(package)) / "lib" / package / executable
        self.processes.append(subprocess.Popen(
            [str(executable_path), "--ros-args", *args], stdout=log, stderr=log,
        ))

    def cleanup(self):
        for process in self.processes:
            process.terminate()
        for process in self.processes:
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait(timeout=5)
        for log in self.logs:
            log.close()
        self.node.destroy_node()
        rclpy.shutdown()

    def wait(self, predicate, seconds=10, publish=None):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            if publish is not None:
                publish()
            rclpy.spin_once(self.node, timeout_sec=0.03)
            if predicate():
                return
        logs = []
        for log in self.logs:
            log.seek(0)
            logs.append(log.read()[-4000:])
        self.fail("Timed out\n" + "\n".join(logs))

    def send(self, points):
        header = Header(frame_id="base_link", stamp=self.node.get_clock().now().to_msg())
        self.cloud_pub.publish(make_point_cloud(header, np.asarray(points, dtype=np.float32)))

    @staticmethod
    def xyz(message):
        return np.ndarray(
            shape=(message.width, 3), dtype="<f4", buffer=message.data,
            strides=(message.point_step, 4),
        )

    def test_body_filter_and_side_rear_stop(self):
        ground = [(x, y, 0.0) for x in np.linspace(-2, 2, 15)
                  for y in np.linspace(-2, 2, 15)]
        body = [(0, 0, 0.3), (0.6, 0.3, 0.8), (-0.2, -0.32, 0.6)]
        distant = [(2, 2, 0.3)]
        clear_cloud = ground + body + distant

        def filtered_and_scan_ready():
            return all(key in self.messages for key in ("self", "obstacles", "scan"))

        self.wait(filtered_and_scan_ready, publish=lambda: self.send(clear_cloud))
        xyz = self.xyz(self.messages["self"])
        inside = ((xyz[:, 0] >= -0.20) & (xyz[:, 0] <= 0.65) &
                  (xyz[:, 1] >= -0.32) & (xyz[:, 1] <= 0.32))
        self.assertFalse(inside.any(), "Body returns leaked into scan input")

        command = Twist()
        command.linear.x = 0.2

        def send_command():
            self.send(clear_cloud)
            self.cmd_pub.publish(command)

        self.wait(lambda: "out" in self.messages and self.messages["out"].linear.x == 0.2,
                  publish=send_command)

        for location in ("left", "right", "rear"):
            with self.subTest(location=location):
                near = [
                    ((-0.25, offset, height) if location == "rear"
                     else (offset, 0.40 if location == "left" else -0.40, height))
                    for offset, height in zip(np.linspace(-0.04, 0.04, 5),
                                              np.linspace(0.2, 0.6, 5))
                ]
                points = clear_cloud + near
                phase_stamp = self.node.get_clock().now().nanoseconds
                self.messages.clear()
                self.wait(
                    lambda: filtered_and_scan_ready() and all(
                        message.header.stamp.sec * 1000000000 + message.header.stamp.nanosec
                        >= phase_stamp for key, message in self.messages.items()
                        if key in ("self", "obstacles", "scan")
                    ),
                    publish=lambda: self.send(points),
                )
                obstacles = self.xyz(self.messages["obstacles"])
                near_returns = obstacles[np.hypot(obstacles[:, 0], obstacles[:, 1]) < 0.5]
                self.assertGreaterEqual(len(near_returns), 4)
                scan = self.messages["scan"]
                finite_ranges = [r for r in scan.ranges if math.isfinite(r)]
                self.assertTrue(any(r < 0.5 for r in finite_ranges))

                for linear, angular in ((0.2, 0.0), (-0.1, 0.0), (0.0, 0.35)):
                    command.linear.x = linear
                    command.angular.z = angular
                    self.messages.pop("out", None)

                    def send_near_command():
                        self.send(points)
                        self.cmd_pub.publish(command)

                    self.wait(lambda: "out" in self.messages and self.messages["out"] == Twist(),
                              publish=send_near_command)

                # Removing the obstacle must release the gate, without body false positives.
                command.linear.x = 0.2
                command.angular.z = 0.0
                self.wait(lambda: "out" in self.messages and self.messages["out"].linear.x == 0.2,
                          publish=send_command)

    def test_scan_output_survives_missing_ground_plane(self):
        points = [(0.0, 0.40, z) for z in np.linspace(0.3, 0.8, 5)]
        self.wait(lambda: "self" in self.messages and "scan" in self.messages,
                  publish=lambda: self.send(points))
        self.assertNotIn("obstacles", self.messages)
        self.assertEqual(self.messages["self"].width, 5)
        self.assertTrue(any(math.isfinite(r) and r < 0.5
                            for r in self.messages["scan"].ranges))

    def test_monitor_stop_and_release_ramp_at_final_safety_gate(self):
        config = Path(get_package_share_directory("isaac_localization_3d")) / "config"
        self.start("isaac_localization_3d", "cmd_vel_safety.py", [
            "--params-file", str(config / "collision_monitor.yaml"),
            "-p", "use_sim_time:=false",
            "-r", "/nav2/cmd_vel:=/test/out", "-r", "/cmd_vel:=/test/final",
            "-r", "/scan:=/test/scan", "-r", "/perception/obstacles:=/test/obstacles",
            "-r", "/localization_3d/accepted_correction:=/test/correction",
        ])
        outputs = []
        self.node.create_subscription(Twist, "/test/final", outputs.append, 10)
        correction_pub = self.node.create_publisher(Header, "/test/correction", 10)
        clear = [(x, y, 0.0) for x in np.linspace(-2, 2, 15)
                 for y in np.linspace(-2, 2, 15)] + [(2, 2, 0.3)]
        near = clear + [(0.0, 0.40, z) for z in np.linspace(0.2, 0.6, 5)]
        command = Twist()

        def send(points):
            correction_pub.publish(Header(
                frame_id="map", stamp=self.node.get_clock().now().to_msg(),
            ))
            self.send(points)
            self.cmd_pub.publish(command)

        for linear, angular in ((0.5, 0.0), (-0.5, 0.0), (0.0, 0.35)):
            with self.subTest(linear=linear, angular=angular):
                command.linear.x = linear
                command.angular.z = angular
                self.wait(
                    lambda: bool(outputs) and outputs[-1] == command,
                    publish=lambda: send(clear),
                )
                self.messages.pop("out", None)
                self.wait(
                    lambda: self.messages.get("out") == Twist() and outputs[-1] == Twist(),
                    publish=lambda: send(near),
                )
                outputs.clear()
                self.wait(
                    lambda: any(msg != Twist() for msg in outputs),
                    publish=lambda: send(clear),
                )
                first = next(msg for msg in outputs if msg != Twist())
                self.assertLessEqual(abs(first.linear.x), 0.08 + 1e-9)
                self.assertLessEqual(abs(first.angular.z), 0.15 + 1e-9)
                self.wait(
                    lambda: outputs[-1] == command,
                    publish=lambda: send(clear),
                )
