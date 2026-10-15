"""ROS integration with deterministic map/ICP/Nav2 peers, plus optional real MQTT."""

import json
import os
from pathlib import Path
import queue
import subprocess
import sys
import tempfile
import time
import unittest
import uuid
from unittest.mock import patch

import rclpy
from action_msgs.srv import CancelGoal
from ament_index_python.packages import get_package_prefix
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
from lifecycle_msgs.msg import State
from lifecycle_msgs.msg import Transition
from lifecycle_msgs.srv import ChangeState, GetState
from nav2_msgs.srv import ClearEntireCostmap, LoadMap, ManageLifecycleNodes
from nav_msgs.msg import Odometry
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from robot_localization.srv import SetPose
from slam_localization_3d.srv import ChangeFloor, LoadLocalizationMap
from std_srvs.srv import SetBool
from tf2_ros import TransformBroadcaster

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "ros2_ws/src/slam_localization_3d/scripts"))
sys.path.insert(0, str(ROOT / "ros2_ws/src/slam_fleet_bridge/scripts"))
from floor_manager import FloorManager
from wait_for_navigation import NAVIGATION_NODES


class FloorSwitchTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.directory = tempfile.TemporaryDirectory()
        for name in ("1F", "B1"):
            path = Path(self.directory.name) / name
            path.mkdir()
            (path / "map.pcd").touch()
            (path / "map_2d.pgm").write_bytes(b"P5\n1 1\n255\n\xfe")
            (path / "map_2d.yaml").write_text("image: map_2d.pgm\n")
        self.namespace = f"floor_test_{uuid.uuid4().hex[:8]}"
        self.node = FloorManager(namespace=self.namespace, parameter_overrides=[
            Parameter("maps_root", value=self.directory.name),
            Parameter("switch_timeout", value=2.5),
            Parameter("service_timeout", value=0.8),
        ])
        self.peer = Node("floor_peers", namespace=self.namespace)
        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.node)
        self.executor.add_node(self.peer)
        self.events = []
        self.holds = {"safety": False, "adapter": False, "tf": False}
        self.loaded = False
        self.reject_pcd = False
        self.reject_map = False
        self.accept_icp = True
        self.moving = False
        self.nav_active = True
        self.bootstrap = False
        self.target = PoseWithCovarianceStamped()
        self.target.header.frame_id = "map"
        self.target.pose.pose.orientation.w = 1.0
        self.target.pose.pose.position.x = 3.0
        self.services = []
        for key, name in (
            ("safety", "navigation/set_map_switch_hold"),
            ("adapter", "localization_3d/set_map_switch_hold"),
            ("tf", "global_tf_gate/set_map_switch_hold"),
        ):
            def hold(request, response, key=key):
                self.holds[key] = request.data
                self.events.append((key, request.data))
                response.success = True
                return response
            self.services.append(self.peer.create_service(SetBool, name, hold))
        self.services += [
            self.peer.create_service(ManageLifecycleNodes,
                                     "lifecycle_manager_navigation/manage_nodes", self.navigation),
            self.peer.create_service(LoadMap, "map_server/load_map", self.map_load),
            self.peer.create_service(LoadLocalizationMap, "localization_3d/load_map", self.pcd_load),
            self.peer.create_service(SetPose, "global_ekf/set_pose", self.set_pose),
            self.peer.create_service(GetState, "bt_navigator/get_state", self.get_state),
        ]
        for name in NAVIGATION_NODES:
            if name != "bt_navigator":
                self.services.append(self.peer.create_service(
                    GetState, f"{name}/get_state", self.get_state))
        for name in ("navigate_to_pose", "navigate_through_poses"):
            self.services.append(self.peer.create_service(
                CancelGoal, f"{name}/_action/cancel_goal", self.cancel))
        for name in ("global_costmap", "local_costmap"):
            self.services.append(self.peer.create_service(
                ClearEntireCostmap, f"{name}/clear_entirely_{name}", self.clear))
        self.pose_pub = self.peer.create_publisher(
            PoseWithCovarianceStamped, "localization_3d/global_pose", 10)
        self.global_pub = self.peer.create_publisher(Odometry, "odometry/global", 10)
        self.local_pub = self.peer.create_publisher(Odometry, "odometry/local", 10)
        self.tf = TransformBroadcaster(self.peer)
        self.peer.create_timer(0.05, self.publish_inputs)
        self.client = self.peer.create_client(ChangeFloor, "navigation/change_floor")
        self.spin_until(lambda: all(client.service_is_ready()
                                   for client in self.node.service_clients.values()))
        self.spin_until(self.client.service_is_ready)
        self.extra_nodes = []

    def tearDown(self):
        for node in self.extra_nodes:
            node.close()
            self.executor.remove_node(node)
            node.destroy_node()
        for node in (self.peer, self.node):
            self.executor.remove_node(node)
            node.destroy_node()
        self.executor.shutdown()
        self.directory.cleanup()

    def spin_until(self, predicate, timeout=6.0):
        end = time.monotonic() + timeout
        while not predicate() and time.monotonic() < end:
            self.executor.spin_once(timeout_sec=0.02)
        self.assertTrue(predicate(), "Timed out waiting for test condition")

    def publish_inputs(self):
        stamp = self.peer.get_clock().now().to_msg()
        local = Odometry()
        local.header.stamp = stamp
        local.header.frame_id = "odom"
        local.child_frame_id = "base_link"
        local.twist.twist.linear.x = 0.2 if self.moving else 0.0
        self.local_pub.publish(local)
        if not self.loaded:
            return
        if self.accept_icp and not self.holds["adapter"]:
            self.target.header.stamp = stamp
            self.pose_pub.publish(self.target)
        global_odom = Odometry()
        global_odom.header.stamp = stamp
        global_odom.header.frame_id = "map"
        global_odom.child_frame_id = "base_link"
        global_odom.pose = self.target.pose
        self.global_pub.publish(global_odom)
        if not self.holds["tf"]:
            transform = TransformStamped()
            transform.header = global_odom.header
            transform.child_frame_id = "base_link"
            transform.transform.translation.x = self.target.pose.pose.position.x
            transform.transform.rotation.w = 1.0
            self.tf.sendTransform(transform)

    def navigation(self, request, response):
        self.events.append(("nav", request.command))
        self.nav_active = request.command == ManageLifecycleNodes.Request.RESUME
        response.success = True
        return response

    def map_load(self, request, response):
        self.assertTrue(self.holds["safety"])
        self.events.append(("map", request.map_url))
        response.result = (LoadMap.Response.RESULT_INVALID_MAP_DATA if self.reject_map
                           else LoadMap.Response.RESULT_SUCCESS)
        return response

    def pcd_load(self, request, response):
        self.assertTrue(self.holds["safety"])
        self.assertTrue(self.holds["adapter"])
        self.assertTrue(self.holds["tf"])
        self.events.append(("pcd", request.pcd_path))
        response.success = not self.reject_pcd
        self.target = request.initial_pose
        self.loaded = response.success
        response.message = "test PCD rejected" if self.reject_pcd else "loaded"
        return response

    def set_pose(self, request, response):
        self.assertTrue(self.holds["tf"])
        self.events.append(("ekf", request.pose.pose.pose.position.x))
        return response

    def get_state(self, request, response):
        if self.bootstrap and self.loaded and not self.holds["tf"]:
            self.nav_active = True
        response.current_state.id = (
            State.PRIMARY_STATE_ACTIVE if self.nav_active else State.PRIMARY_STATE_INACTIVE)
        return response

    def cancel(self, request, response):
        self.events.append(("cancel", True))
        return response

    def clear(self, request, response):
        self.assertTrue(self.holds["safety"])
        self.events.append(("clear", True))
        return response

    def request(self, floor="B1"):
        request = ChangeFloor.Request(floor_name=floor)
        request.initial_pose = self.target
        return request

    def switch(self, request=None):
        future = self.client.call_async(request or self.request())
        self.spin_until(future.done)
        self.assertIsNone(future.exception())
        return future.result()

    def test_success_waits_for_localization_resets_ekf_and_releases_last(self):
        result = self.switch()
        self.assertTrue(result.success, result.message)
        self.assertEqual(result.current_floor, "B1")
        self.assertFalse(any(self.holds.values()))
        kinds = [event[0] for event in self.events]
        self.assertLess(kinds.index("map"), kinds.index("pcd"))
        self.assertLess(kinds.index("pcd"), kinds.index("ekf"))
        self.assertLess(kinds.index("ekf"), kinds.index("clear"))
        self.assertEqual(self.events[-1], ("safety", False))

    def test_invalid_floor_and_pose_do_not_stop_or_change_healthy_stack(self):
        for request in (self.request("../B1"), self.request("missing")):
            result = self.switch(request)
            self.assertFalse(result.success)
            self.assertEqual(result.current_floor, "1F")
        invalid = self.request()
        invalid.initial_pose.pose.pose.orientation.w = 0.0
        self.assertFalse(self.switch(invalid).success)
        self.assertEqual(self.events, [])

    def test_pcd_failure_stays_stopped_and_new_request_can_recover(self):
        self.reject_pcd = True
        result = self.switch()
        self.assertFalse(result.success)
        self.assertEqual(result.current_floor, "")
        self.assertTrue(self.holds["safety"])
        self.assertFalse(self.nav_active)
        self.reject_pcd = False
        result = self.switch()
        self.assertTrue(result.success, result.message)

    def test_map_failure_does_not_load_pcd_or_release_stop(self):
        self.reject_map = True
        result = self.switch()
        self.assertFalse(result.success)
        self.assertTrue(self.holds["safety"])
        self.assertNotIn("pcd", [event[0] for event in self.events])

    def test_icp_timeout_does_not_report_success_from_initial_pose_alone(self):
        self.accept_icp = False
        result = self.switch()
        self.assertFalse(result.success)
        self.assertIn("ICP", result.message)
        self.assertTrue(self.holds["safety"])
        self.assertFalse(self.nav_active)

    def test_moving_robot_does_not_load_new_maps(self):
        self.moving = True
        result = self.switch()
        self.assertFalse(result.success)
        self.assertIn("stationary", result.message)
        self.assertNotIn("map", [event[0] for event in self.events])

    def test_concurrent_switch_is_rejected(self):
        first = self.client.call_async(self.request())
        self.spin_until(lambda: self.node.busy)
        second = self.switch(self.request("1F"))
        self.assertFalse(second.success)
        self.assertIn("already in progress", second.message)
        self.spin_until(first.done)
        self.assertTrue(first.result().success, first.result().message)

    def manual_request(self):
        return ChangeFloor.Request(floor_name="B1", manual_initial_pose=True)

    def rviz_pose(self):
        pose = PoseWithCovarianceStamped()
        pose.header.frame_id = "map"
        pose.pose.pose.position.x = 4.0
        pose.pose.pose.orientation.w = 1.0
        return pose

    def test_manual_pose_waits_for_rviz_estimate_after_map_is_shown(self):
        rviz = self.peer.create_publisher(PoseWithCovarianceStamped, "initialpose", 10)
        self.spin_until(lambda: self.peer.count_subscribers(rviz.topic_name) > 0)
        future = self.client.call_async(self.manual_request())
        self.spin_until(lambda: "map" in [event[0] for event in self.events])
        self.assertNotIn("pcd", [event[0] for event in self.events])
        rviz.publish(self.rviz_pose())
        self.spin_until(future.done, timeout=10.0)
        result = future.result()
        self.assertTrue(result.success, result.message)
        self.assertEqual(result.current_floor, "B1")
        self.assertFalse(any(self.holds.values()))
        position = self.target.pose.pose.position
        self.assertEqual((position.x, position.z), (4.0, self.node.floor_base_z))

    def test_manual_pose_sent_before_map_is_shown_is_ignored(self):
        rviz = self.peer.create_publisher(PoseWithCovarianceStamped, "initialpose", 10)
        self.spin_until(lambda: self.peer.count_subscribers(rviz.topic_name) > 0)
        rviz.publish(self.rviz_pose())
        self.spin_until(lambda: self.node.manual_pose is not None)
        self.node.manual_pose_timeout = 0.5
        result = self.switch(self.manual_request())
        self.assertFalse(result.success)
        self.assertIn("RViz 2D Pose Estimate", result.message)
        self.assertNotIn("pcd", [event[0] for event in self.events])
        self.assertTrue(self.holds["safety"])

    def test_command_fills_manual_or_explicit_change_floor_request(self):
        import rmf_protocol
        stamp = self.peer.get_clock().now().to_msg()
        manual = rmf_protocol.fill_change_floor_request(
            ChangeFloor.Request(), {"floor_name": "B1", "initial_pose": None}, -0.5, stamp)
        self.assertTrue(manual.manual_initial_pose)
        explicit = rmf_protocol.fill_change_floor_request(
            ChangeFloor.Request(),
            {"floor_name": "B1", "initial_pose": {"x": 1.0, "y": 2.0, "z": None, "yaw": 0.0}},
            -0.5, stamp)
        self.assertFalse(explicit.manual_initial_pose)
        self.assertEqual(explicit.initial_pose.pose.pose.position.z, -0.5)
        self.assertEqual(explicit.initial_pose.pose.pose.orientation.w, 1.0)

    def test_first_pose_can_bootstrap_before_navigation_manager_exists(self):
        self.peer.destroy_service(self.services[3])
        self.spin_until(lambda: not self.node.service_clients["nav"].service_is_ready())
        self.node.navigation_seen = False
        self.nav_active = False
        self.bootstrap = True
        result = self.switch(self.request("1F"))
        self.assertTrue(result.success, result.message)
        self.assertEqual(result.current_floor, "1F")
        self.assertFalse(any(self.holds.values()))
        self.assertNotIn("nav", [event[0] for event in self.events])

    @unittest.skipUnless(os.environ.get("FLOOR_TEST_MQTT_HOST"), "opt-in MQTT integration")
    def test_mqtt_command_reaches_floor_manager_and_duplicate_does_not_reload(self):
        import paho.mqtt.client as mqtt
        from fleet_bridge_node import FleetBridge
        import rmf_protocol
        fleet = self.namespace
        # Keep test registrations outside the production adapter's rmf/# subscription.
        real_topics = rmf_protocol.topics
        with patch.object(rmf_protocol, "topics", side_effect=lambda *args: {
            key: "floor-tests/" + value for key, value in real_topics(*args).items()
        }):
            bridge = FleetBridge(namespace=self.namespace, parameter_overrides=[
                Parameter("mqtt_host", value=os.environ["FLOOR_TEST_MQTT_HOST"]),
                Parameter("fleet_name", value=fleet),
                Parameter("footprint_radius", value=0.5),
                Parameter("max_linear_velocity", value=0.5),
                Parameter("max_angular_velocity", value=0.5),
            ])
        self.extra_nodes.append(bridge)
        self.executor.add_node(bridge)
        received = queue.Queue()
        probe = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
        probe.on_message = lambda client, userdata, message: received.put(json.loads(message.payload))
        subscribed = []
        probe.on_subscribe = lambda *args: subscribed.append(True)
        probe.connect(os.environ["FLOOR_TEST_MQTT_HOST"], 1883)
        probe.loop_start()
        base = f"floor-tests/rmf/{fleet}/robot/{self.namespace}"
        try:
            probe.subscribe(base + "/command_result", qos=1)
            self.spin_until(lambda: bool(subscribed) and bridge.client.is_connected())
            self.spin_until(bridge.floor_client.service_is_ready)
            payload = json.dumps({
                "robot_id": self.namespace, "cmd_id": 42, "action": "change_floor",
                "floor_name": "B1", "initial_pose": {"x": 3, "y": 0, "yaw": 0},
            })
            # No register_ack yet: first-floor initialization is allowed.
            probe.publish(base + "/command", payload, qos=1).wait_for_publish(2)
            self.spin_until(lambda: not received.empty())
            result = received.get_nowait()
            self.assertEqual((result["cmd_id"], result["status"], result["level_name"]),
                             (42, "completed", "B1"), result)
            # RMF sends planar poses: the bridge supplies the floor base_link height.
            self.assertAlmostEqual(self.target.pose.pose.position.z, -0.526)
            count = sum(event[0] == "pcd" for event in self.events)
            probe.publish(base + "/command", payload, qos=1).wait_for_publish(2)
            self.spin_until(lambda: not received.empty())
            self.assertEqual(received.get_nowait(), result)
            self.assertEqual(sum(event[0] == "pcd" for event in self.events), count)
            probe.publish(base + "/command", json.dumps({
                "robot_id": self.namespace, "cmd_id": 43, "action": "change_floor",
                "floor_name": "missing", "initial_pose": {"x": 3, "y": 0, "z": 0, "yaw": 0},
            }), qos=1).wait_for_publish(2)
            self.spin_until(lambda: not received.empty())
            self.assertEqual(received.get_nowait()["status"], "failed")
            self.assertTrue(bridge.floor_error)
            bridge.registered = True
            probe.publish(base + "/command", json.dumps({
                "robot_id": self.namespace, "cmd_id": 44, "action": "navigate",
                "target": {"x": 1, "y": 2, "yaw": 0},
            }), qos=1).wait_for_publish(2)
            self.spin_until(lambda: not received.empty())
            self.assertIn("retry change_floor", received.get_nowait()["message"])
        finally:
            probe.disconnect()
            probe.loop_stop()

    @unittest.skipUnless(os.environ.get("FLOOR_TEST_MQTT_HOST"), "opt-in MQTT integration")
    def test_standalone_mqtt_floor_command_waits_for_rviz_pose(self):
        import paho.mqtt.client as mqtt
        from floor_command_bridge import FloorCommandBridge
        fleet = f"floor-tests-{self.namespace}"
        bridge = FloorCommandBridge(namespace=self.namespace, parameter_overrides=[
            Parameter("mqtt_host", value=os.environ["FLOOR_TEST_MQTT_HOST"]),
            Parameter("fleet_name", value=fleet),
            Parameter("robot_name", value=self.namespace),
        ])
        self.extra_nodes.append(bridge)
        self.executor.add_node(bridge)
        rviz = self.peer.create_publisher(PoseWithCovarianceStamped, "initialpose", 10)
        self.spin_until(lambda: self.peer.count_subscribers(rviz.topic_name) > 0)
        received = queue.Queue()
        probe = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
        probe.on_message = lambda client, userdata, message: received.put(json.loads(message.payload))
        subscribed = []
        probe.on_subscribe = lambda *args: subscribed.append(True)
        probe.connect(os.environ["FLOOR_TEST_MQTT_HOST"], 1883)
        probe.loop_start()
        base = f"rmf/{fleet}/robot/{self.namespace}"
        try:
            probe.subscribe(base + "/command_result", qos=1)
            self.spin_until(lambda: bool(subscribed) and bridge.client.is_connected())
            self.spin_until(bridge.floor_client.service_is_ready)
            probe.publish(base + "/command", json.dumps({
                "robot_id": self.namespace, "cmd_id": 1, "action": "navigate",
                "target": {"x": 1, "y": 2, "yaw": 0},
            }), qos=1).wait_for_publish(2)
            self.spin_until(lambda: not received.empty())
            self.assertIn("Only change_floor", received.get_nowait()["message"])
            probe.publish(base + "/command", json.dumps({
                "robot_id": self.namespace, "cmd_id": 2, "action": "change_floor",
                "floor_name": "B1",
            }), qos=1).wait_for_publish(2)
            self.spin_until(lambda: "map" in [event[0] for event in self.events])
            rviz.publish(self.rviz_pose())
            self.spin_until(lambda: not received.empty(), timeout=10.0)
            result = received.get_nowait()
            self.assertEqual((result["cmd_id"], result["status"], result["level_name"]),
                             (2, "completed", "B1"), result)
            self.assertEqual(self.target.pose.pose.position.x, 4.0)
        finally:
            probe.disconnect()
            probe.loop_stop()



class InstalledFloorSurfaceTest(unittest.TestCase):
    """Exercise the actual installed map server, EKF service and PCD localizer."""

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def test_real_map_server_ekf_and_pcd_load_services(self):
        import numpy as np
        from std_msgs.msg import Header
        from sensor_msgs.msg import PointCloud2
        from xyz_cloud import make_point_cloud

        namespace = f"floor_surface_{uuid.uuid4().hex[:8]}"
        node = Node("surface_probe", namespace=namespace)
        executor = SingleThreadedExecutor()
        executor.add_node(node)
        processes, streams = [], []
        with tempfile.TemporaryDirectory() as directory:
            def start(package, executable, arguments):
                stream = open(Path(directory) / f"{executable}.log", "w+")
                streams.append(stream)
                path = Path(get_package_prefix(package)) / "lib" / package / executable
                process = subprocess.Popen([
                    str(path), "--ros-args", "-r", f"__ns:=/{namespace}", *arguments,
                ], stdout=stream, stderr=subprocess.STDOUT)
                processes.append(process)

            def spin_until(predicate, timeout=15):
                end = time.monotonic() + timeout
                while not predicate() and time.monotonic() < end:
                    executor.spin_once(timeout_sec=0.02)
                    for process in processes:
                        self.assertIsNone(process.poll(), "Installed node exited unexpectedly")
                self.assertTrue(predicate(), "Installed ROS service did not become ready")

            def call(kind, name, request):
                client = node.create_client(kind, name)
                spin_until(client.service_is_ready)
                future = client.call_async(request)
                spin_until(future.done)
                self.assertIsNone(future.exception())
                return future.result()

            try:
                start("nav2_map_server", "map_server", [
                    "-p", f"yaml_filename:={ROOT}/maps/1F/map_2d.yaml",
                ])
                for transition in (Transition.TRANSITION_CONFIGURE, Transition.TRANSITION_ACTIVATE):
                    request = ChangeState.Request()
                    request.transition.id = transition
                    self.assertTrue(call(
                        ChangeState, "map_server/change_state", request).success)
                for floor in ("B1", "1F"):
                    result = call(LoadMap, "map_server/load_map", LoadMap.Request(
                        map_url=str(ROOT / "maps" / floor / "map_2d.yaml")))
                    self.assertEqual(result.result, LoadMap.Response.RESULT_SUCCESS)
                    self.assertGreater(len(result.map.data), 0)
                    self.assertEqual(result.map.header.frame_id, "map")

                start("robot_localization", "ekf_node", [
                    "-r", "__node:=global_ekf", "-r", "set_pose:=global_ekf/set_pose",
                    "-r", "odometry/filtered:=odometry/global",
                    "-p", "world_frame:=map", "-p", "publish_tf:=false",
                    "-p", f"odom0:=/{namespace}/wheel_odom",
                    "-p", "odom0_config:=[false,false,false,false,false,false,"
                          "true,false,false,false,false,true,false,false,false]",
                ])
                observed = []
                node.create_subscription(Odometry, "odometry/global", observed.append, 10)
                wheel_pub = node.create_publisher(Odometry, "wheel_odom", 10)

                def wheel():
                    message = Odometry()
                    message.header.stamp = node.get_clock().now().to_msg()
                    message.header.frame_id = "odom"
                    message.child_frame_id = "base_link"
                    message.pose.pose.orientation.w = 1.0
                    message.twist.covariance[0] = 0.01
                    message.twist.covariance[35] = 0.01
                    wheel_pub.publish(message)

                node.create_timer(0.05, wheel)
                initial = PoseWithCovarianceStamped()
                initial.header.frame_id = "map"
                initial.header.stamp = node.get_clock().now().to_msg()
                initial.pose.pose.orientation.w = 1.0
                initial.pose.pose.position.x = 3.0
                call(SetPose, "global_ekf/set_pose", SetPose.Request(pose=initial))
                spin_until(lambda: bool(observed)
                           and abs(observed[-1].pose.pose.position.x - 3.0) < 1e-4)
                self.assertAlmostEqual(observed[-1].pose.pose.position.x, 3.0, places=4)
                self.assertEqual(observed[-1].header.frame_id, "map")

                topics = ("Odometry", "cloud_registered", "map_to_odom", "submap",
                          "initialpose", "cur_scan_in_map")
                start("slam_localization_3d", "global_localization_xyz.py", [
                    "-p", f"pcd_map_path:={ROOT}/maps/1F/map.pcd",
                    *(item for topic in topics for item in
                      ("-r", f"/{topic}:=/{namespace}/{topic}")),
                ])
                scan_pub = node.create_publisher(PointCloud2, "cloud_registered", 10)
                odom_pub = node.create_publisher(Odometry, "Odometry", 10)
                cloud = np.random.default_rng(42).normal(size=(100, 3)).astype(np.float32)

                def sensors():
                    stamp = node.get_clock().now().to_msg()
                    odom = Odometry()
                    odom.header = Header(stamp=stamp, frame_id="camera_init")
                    odom.child_frame_id = "body"
                    odom.pose.pose.orientation.w = 1.0
                    odom_pub.publish(odom)
                    scan_pub.publish(make_point_cloud(odom.header, cloud))

                node.create_timer(0.05, sensors)
                spin_until(lambda: scan_pub.get_subscription_count() > 0)
                # Wait for several real callbacks, not just DDS discovery.
                ready_at = time.monotonic() + 0.3
                spin_until(lambda: time.monotonic() >= ready_at)
                result = call(LoadLocalizationMap, "localization_3d/load_map",
                              LoadLocalizationMap.Request(
                                  pcd_path=str(ROOT / "maps/B1/map.pcd"), initial_pose=initial))
                self.assertTrue(result.success, result.message)
                self.assertIn("B1/map.pcd", result.message)
            finally:
                for process in processes:
                    process.terminate()
                for process in processes:
                    try:
                        process.wait(timeout=5)
                    except subprocess.TimeoutExpired:
                        process.kill()
                        process.wait(timeout=5)
                for stream in streams:
                    stream.close()
                executor.remove_node(node)
                node.destroy_node()
                executor.shutdown()


if __name__ == "__main__":
    unittest.main()
