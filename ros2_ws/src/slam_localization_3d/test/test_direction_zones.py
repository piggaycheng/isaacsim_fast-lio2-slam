import importlib.util
import copy
import math
import subprocess
import sys
import tempfile
import time
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path
from unittest.mock import Mock, patch

import rclpy
import yaml
from ament_index_python.packages import get_package_prefix
from geometry_msgs.msg import Point32, PolygonStamped, TransformStamped, Twist
from launch import LaunchContext
from launch.actions import SetLaunchConfiguration
from launch_ros.actions import Node as NodeAction
from launch_ros.utilities import evaluate_parameters
from lifecycle_msgs.msg import Transition
from lifecycle_msgs.srv import ChangeState
from nav_msgs.msg import Odometry
from rcl_interfaces.msg import ParameterType, ParameterValue
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Header
from tf2_ros import TransformBroadcaster

PACKAGE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE / "scripts"))
from cmd_vel_safety import CmdVelSafety
from direction_zones import POLYGONS, ZONE_SETS, DirectionZones

spec = importlib.util.spec_from_file_location("fusion_launch", PACKAGE / "launch/global_fusion.launch.py")
fusion_launch = importlib.util.module_from_spec(spec)
spec.loader.exec_module(fusion_launch)
from nav_geometry import footprint_bounds, rotation_radius, safety_geometry

CONFIGS = [str(PACKAGE / "config" / filename) for filename in (
    "collision_monitor.yaml", "navigation.yaml", "direction_zones.yaml",
)]


def twist(linear=0.0, angular=0.0):
    message = Twist()
    message.linear.x, message.angular.z = linear, angular
    return message


class DirectionLaunchTest(unittest.TestCase):
    def configure(self, robot_type="nova_carter", **overrides):
        context = LaunchContext()
        context.launch_configurations.update(
            namespace="", robot_type=robot_type, direction_zones="true",
            navigate="true", costmaps="true", obstacle_cloud="true",
            costmap_config=str(PACKAGE / "config/observation_costmaps.yaml"),
        )
        context.launch_configurations.update(overrides)
        factory = tempfile.TemporaryDirectory

        def directory(**kwargs):
            result = factory(**kwargs)
            self.addCleanup(result.cleanup)
            return result

        with patch.object(fusion_launch.tempfile, "TemporaryDirectory", side_effect=directory):
            actions = fusion_launch.configure_surround(context, *CONFIGS)
        for action in actions:
            if isinstance(action, SetLaunchConfiguration):
                action.execute(context)
        return context, actions

    def monitor(self, context):
        config = yaml.safe_load(Path(context.launch_configurations["collision_config"]).read_text())
        return config["collision_monitor"]["ros__parameters"]

    def test_nova_zone_sets_are_derived_from_body_and_braking_estimate(self):
        context, actions = self.configure()
        params = self.monitor(context)
        self.assertEqual(params["cmd_vel_in_topic"], "/nav2/cmd_vel_direction")
        self.assertEqual(set(POLYGONS), set(params["polygons"]) - {"FootprintApproach"})
        self.assertNotIn("PolygonStop", params)
        self.assertNotIn("PolygonSurround", params)
        for actual, expected in (
            (params["PolygonSurroundForward"]["points"],
             [0.711, 0.26, 0.711, -0.26, -0.598, -0.26, -0.598, 0.26]),
            (params["PolygonSurroundReverse"]["points"],
             [0.151, 0.26, 0.151, -0.26, -1.158, -0.26, -1.158, 0.26]),
        ):
            for a, b in zip(actual, expected):
                self.assertAlmostEqual(a, b)
        enabled = {name for name in POLYGONS if params[name]["enabled"]}
        self.assertEqual(enabled, set(ZONE_SETS["forward"]))
        self.assertTrue(params["FootprintApproach"]["enabled"])
        self.assertAlmostEqual(params["PolygonRotate"]["radius"], math.hypot(0.598, 0.26))
        self.assertEqual(params["PolygonRotate"]["type"], "circle")
        self.assertFalse(params["PolygonRotate"]["enabled"])
        topics = [params[name]["polygon_pub_topic"] for name in POLYGONS]
        self.assertEqual(len(topics), len(set(topics)))
        self.assertEqual(context.launch_configurations["navigation_config"], CONFIGS[1])
        self.assertTrue(any(isinstance(action, NodeAction) for action in actions))

    def test_carter_v1_zones_follow_its_profile(self):
        context, _ = self.configure("carter_v1")
        params = self.monitor(context)
        forward = params["PolygonSurroundForward"]["points"]
        reverse = params["PolygonSurroundReverse"]["points"]
        self.assertAlmostEqual(forward[4], -0.441)
        self.assertAlmostEqual(forward[0], 0.849)
        self.assertAlmostEqual(forward[1], 0.325)
        self.assertAlmostEqual(reverse[0], 0.289)
        self.assertAlmostEqual(reverse[4], -1.001)
        self.assertAlmostEqual(reverse[5], -0.325)
        self.assertAlmostEqual(params["PolygonRotate"]["radius"], math.hypot(0.441, 0.325))

    def test_slow_polygon_covers_union_with_required_offset(self):
        for kind in ("nova_carter", "carter_v1"):
            context, _ = self.configure(kind)
            params = self.monitor(context)
            radius = params["PolygonRotate"]["radius"]
            flat = params["PolygonSlow"]["points"]
            slow = list(zip(flat[::2], flat[1::2]))
            points = [(radius * math.cos(i * math.pi / 180),
                       radius * math.sin(i * math.pi / 180)) for i in range(360)]
            for name in ("PolygonSurroundForward", "PolygonSurroundReverse"):
                flat = params[name]["points"]
                points.extend(zip(flat[::2], flat[1::2]))
            # Clockwise polygon: every input point must be >= 0.10 m inside every edge.
            for a, b in zip(slow, slow[1:] + slow[:1]):
                dx, dy = b[0] - a[0], b[1] - a[1]
                for x, y in points:
                    distance = (dy * (x - a[0]) - dx * (y - a[1])) / math.hypot(dx, dy)
                    self.assertGreaterEqual(distance, 0.10 - 1e-8, (kind, x, y))

    def test_mismatched_or_invalid_footprints_fail_explicitly(self):
        for kind in ("nova_carter", "carter_v1"):
            for key, value, message in (
                ("footprint", "[]", "four-corner"),
                ("footprint", "[[0.2, 0.3], [0.2, -0.3], [-0.5, -0.3], [-0.4, 0.3]]", "Rectangular"),
                ("footprint", "[[0.2, 0.3], [-0.5, -0.3], [0.2, -0.3], [-0.5, 0.3]]", "Rectangular"),
                ("footprint_padding", -0.01, "nonnegative"),
                ("footprint_padding", float("nan"), "finite"),
                ("footprint_padding", 0.02, "match"),
            ):
                config = fusion_launch.load_parameters(str(PACKAGE / "config/observation_costmaps.yaml"), kind)
                planning = config["global_costmap"]["global_costmap"]["ros__parameters"]
                planning[key] = value
                with self.subTest(kind=kind, key=key), self.assertRaisesRegex(ValueError, message):
                    rotation_radius(config)

    def test_estimated_distances_and_invalid_inputs(self):
        costmaps = fusion_launch.load_parameters(str(PACKAGE / "config/observation_costmaps.yaml"), "nova_carter")
        settings = yaml.safe_load(Path(CONFIGS[2]).read_text())["direction_zones"]["ros__parameters"]
        geometry = safety_geometry(costmaps, 0.75, settings, 0.5)
        self.assertEqual(geometry["stop_distance"], 0.56)
        self.assertEqual(geometry["slow_padding"], 0.10)
        slower = safety_geometry(costmaps, 0.25, settings, 0.5)
        self.assertLess(slower["stop_distance"], geometry["stop_distance"])
        self.assertEqual(slower["slow_padding"], 0.10)
        custom = dict(settings, slow_padding=0.15)
        self.assertEqual(safety_geometry(costmaps, 0.75, custom, 0.5)["slow_padding"], 0.15)
        for key in ("reaction_time", "effective_deceleration", "distance_margin", "slow_padding"):
            invalid = copy.deepcopy(settings)
            invalid[key] = 0
            with self.assertRaisesRegex(ValueError, "Safety distance"):
                safety_geometry(costmaps, 0.75, invalid, 0.5)
        for ratio in (0, 1, float("nan")):
            with self.assertRaisesRegex(ValueError, "slowdown_ratio"):
                safety_geometry(costmaps, 0.75, settings, ratio)

    def test_svg_geometry_matches_both_robot_profiles(self):
        root = ET.parse(PACKAGE.parents[2] / "docs/images/nav_safety_zones.svg").getroot()
        elements = {item.attrib["id"]: item for item in root.iter() if "id" in item.attrib}
        circles = [item for item in root.iter()
                   if item.tag.endswith("circle") and item.attrib.get("class") == "rotate"]
        for index, (kind, prefix) in enumerate((("nova_carter", "nova"), ("carter_v1", "v1"))):
            context, _ = self.configure(kind)
            params = self.monitor(context)
            svg_slow = [[float(value) for value in point.split(",")]
                        for point in elements[prefix + "Slow"].attrib["points"].split()]
            flat = params["PolygonSlow"]["points"]
            expected = list(zip(flat[::2], flat[1::2]))
            self.assertEqual(len(svg_slow), len(expected))
            for (sx, sy), (x, y) in zip(svg_slow, expected):
                self.assertAlmostEqual(sx / 100, x, delta=0.00051)
                self.assertAlmostEqual(-sy / 100, y, delta=0.00051)
            self.assertAlmostEqual(float(circles[index].attrib["r"]) / 100,
                                   params["PolygonRotate"]["radius"], delta=1e-6)

    def test_static_without_navigation_or_when_disabled(self):
        for overrides in ({"navigate": "false"}, {"direction_zones": "false"}):
            context, actions = self.configure(**overrides)
            params = self.monitor(context)
            self.assertEqual(params["cmd_vel_in_topic"], "/nav2/cmd_vel")
            self.assertTrue(all(params[name]["enabled"] for name in POLYGONS))
            self.assertFalse(any(isinstance(action, NodeAction) for action in actions))

    def test_clear_topics_follow_both_robot_namespaces(self):
        for namespace, kind in (("carter1", "nova_carter"), ("carter2", "carter_v1")):
            context, actions = self.configure(kind, namespace=namespace)
            monitor = self.monitor(context)
            selector = next(action for action in actions if isinstance(action, NodeAction))
            parameters = evaluate_parameters(context, selector._Node__parameters)[-1]
            self.assertEqual(parameters["base_frame_id"], monitor["base_frame_id"])
            for name in ("PolygonSurroundForward", "PolygonSurroundReverse", "PolygonRotate"):
                self.assertEqual(parameters[name + ".polygon_pub_topic"],
                                 f"/{namespace}{monitor[name]['polygon_pub_topic']}")


class DirectionZonesTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = DirectionZones()
        self.now = 10.0
        clock = Mock()
        clock.now.side_effect = lambda: Time(seconds=self.now)
        self.clock_patch = patch.object(self.node, "get_clock", return_value=clock)
        self.clock_patch.start()
        for name in ("publisher", "getter", "setter"):
            setattr(self.node, name, Mock())
        self.node.zone_visualizers = {name: Mock() for name in self.node.zone_visualizers}
        self.node.checked = True
        self.node.active = "forward"

    def tearDown(self):
        self.clock_patch.stop()
        self.node.destroy_node()

    def output(self):
        return self.node.publisher.publish.call_args.args[0]

    def odometry(self, linear=0.0, angular=0.0, x=0.0):
        message = Odometry()
        message.header.frame_id, message.child_frame_id = "odom", "base_link"
        message.header.stamp = Time(seconds=self.now).to_msg()
        message.pose.pose.position.x = x
        message.pose.pose.orientation.w = 1.0
        message.twist.twist.linear.x, message.twist.twist.angular.z = linear, angular
        self.node.on_odometry(message)

    def step(self, seconds, command=None, **odometry):
        self.now += seconds
        if command is not None:
            self.node.on_command(command)
        self.odometry(**odometry)
        self.node.tick()

    def test_requested_mode_classification(self):
        cases = [((0.2, 0.5), "forward"), ((-0.1, 0.0), "reverse"), ((0.005, 0.3), "rotate"),
                 ((0.0, -0.3), "rotate"), ((0.005, 0.01), None), ((0.0, 0.0), None)]
        for (linear, angular), mode in cases:
            self.assertEqual(self.node.requested_mode(twist(linear, angular)), mode)

    def test_commands_are_only_forwarded_in_the_matching_direction(self):
        self.node.on_command(twist(0.4, 0.3))
        self.assertEqual((self.output().linear.x, self.output().angular.z), (0.4, 0.3))
        for command in (twist(-0.2), twist(0.0, 0.4)):
            self.node.on_command(command)
            self.assertEqual(self.output(), Twist())
        self.node.active = "rotate"
        self.node.on_command(twist(0.005, 0.4))
        self.assertEqual((self.output().linear.x, self.output().angular.z), (0.0, 0.4))
        self.node.active = "reverse"
        self.node.on_command(twist(-0.2, 0.1))
        self.assertEqual((self.output().linear.x, self.output().angular.z), (-0.2, 0.1))

    def test_only_inactive_visualizations_are_cleared_in_every_mode(self):
        for mode, enabled in ZONE_SETS.items():
            self.node.active = mode
            for publisher in self.node.zone_visualizers.values():
                publisher.reset_mock()
            self.node.clear_inactive_visualizations()
            for name, publisher in self.node.zone_visualizers.items():
                if name in enabled:
                    publisher.publish.assert_not_called()
                else:
                    message = publisher.publish.call_args.args[0]
                    self.assertEqual(message.header.frame_id, "base_link")
                    self.assertEqual(message.header.stamp.sec, 0)
                    self.assertEqual(message.header.stamp.nanosec, 0)
                    self.assertEqual(message.polygon.points, [])
            self.assertNotIn("PolygonSlow", self.node.zone_visualizers)

    def test_visualizations_wait_for_confirmed_native_state(self):
        for checked, active, future in ((False, "forward", None), (True, None, None),
                                        (True, "forward", Mock())):
            self.node.checked, self.node.active, self.node.future = checked, active, future
            self.node.clear_inactive_visualizations()
            for publisher in self.node.zone_visualizers.values():
                publisher.publish.assert_not_called()

    def test_switch_requires_settle_time_and_measured_standstill(self):
        self.step(0.0, twist(-0.2), linear=0.3)
        self.assertEqual(self.output(), Twist())
        self.step(0.25, twist(-0.2), linear=0.2, x=0.05)
        self.node.setter.call_async.assert_not_called()
        self.step(0.05, twist(-0.2), x=0.05)
        request = self.node.setter.call_async.call_args.args[0]
        flags = {p.name: p.value.bool_value for p in request.parameters}
        self.assertEqual(flags, {name + ".enabled": name in ZONE_SETS["reverse"] for name in POLYGONS})
        self.node.on_command(twist(-0.2))
        self.assertEqual(self.output(), Twist())
        self.node.future = Mock()
        self.node.future.done.return_value = True
        self.node.future.result.return_value = Mock(result=Mock(successful=True))
        self.node.tick()
        self.assertEqual(self.node.active, "reverse")
        self.node.zone_visualizers["PolygonSurroundForward"].publish.assert_called_once()
        self.node.zone_visualizers["PolygonRotate"].publish.assert_called_once()
        self.node.zone_visualizers["PolygonSurroundReverse"].publish.assert_not_called()
        self.node.on_command(twist(-0.2))
        self.assertEqual(self.output().linear.x, -0.2)

    def test_pose_motion_blocks_switch_despite_zero_twist(self):
        self.step(0.0, twist(0.0, 0.4))
        for index in range(1, 8):
            self.step(0.05, twist(0.0, 0.4), x=0.01 * index)
        self.node.setter.call_async.assert_not_called()

    def test_stale_odometry_or_command_blocks_switch(self):
        self.node.on_command(twist(-0.2))
        for _ in range(8):
            self.now += 0.05
            self.node.on_command(twist(-0.2))
            self.node.tick()
        self.node.setter.call_async.assert_not_called()
        self.step(0.5, None)
        self.assertIsNone(self.node.barrier_since)
        self.node.setter.call_async.assert_not_called()

    def test_rejected_and_timed_out_switches_fail_closed(self):
        self.node.target = "reverse"
        self.node.future = Mock()
        self.node.future.done.return_value = True
        self.node.future.result.return_value = Mock(result=Mock(successful=False, reason="no"))
        with self.assertRaisesRegex(RuntimeError, "rejected"):
            self.node.tick()
        self.node.future = Mock()
        self.node.future.done.return_value = False
        self.node.request_stamp = self.now - 2.0
        with self.assertRaisesRegex(RuntimeError, "timed out"):
            self.node.tick()

    def test_startup_reads_native_state_and_corrects_mixed_sets(self):
        def response(enabled):
            values = [ParameterValue(type=ParameterType.PARAMETER_BOOL, bool_value=name in enabled)
                      for name in POLYGONS]
            future = Mock()
            future.done.return_value = True
            future.result.return_value = Mock(values=values)
            return future

        self.node.checked, self.node.active = False, None
        self.node.getter.service_is_ready.return_value = True
        self.node.tick()
        self.node.future = response(set(ZONE_SETS["reverse"]))
        self.node.tick()
        self.assertEqual(self.node.active, "reverse")
        self.node.checked, self.node.active = False, None
        self.node.tick()
        self.node.future = response({"PolygonSurroundForward", "PolygonRotate", "PolygonSlow"})
        self.node.setter.service_is_ready.return_value = True
        self.now += 0.05
        self.odometry()
        self.node.tick()
        self.assertIsNone(self.node.active)
        self.step(0.25)
        self.assertEqual(self.node.setter.call_async.call_args.args[0].parameters[0].name,
                         "PolygonSurroundForward.enabled")
        self.assertEqual(self.node.target, "forward")

    def test_invalid_parameters_are_rejected(self):
        with self.assertRaisesRegex(ValueError, "settle_time"):
            DirectionZones(parameter_overrides=[rclpy.parameter.Parameter("settle_time", value=0.0)])


class RealHumbleDirectionZonesTest(unittest.TestCase):
    def test_front_obstacle_blocks_forward_but_allows_reverse_and_rotation(self):
        for kind in ("nova_carter", "carter_v1"):
            with self.subTest(kind=kind):
                self.check_modes(kind)

    def check_modes(self, kind):
        launch = DirectionLaunchTest()
        launch.addCleanup = self.addCleanup
        context, _ = launch.configure(kind)
        params = launch.monitor(context)
        costmaps = fusion_launch.load_parameters(str(PACKAGE / "config/observation_costmaps.yaml"), kind)
        front, rear, left, right = footprint_bounds(costmaps)
        radius = params["PolygonRotate"]["radius"]
        filename = context.launch_configurations["collision_config"]
        prefix = get_package_prefix("nav2_collision_monitor")
        process = subprocess.Popen([
            str(Path(prefix) / "lib/nav2_collision_monitor/collision_monitor"),
            "--ros-args", "--params-file", filename, "-p", "use_sim_time:=false",
        ], stdout=subprocess.DEVNULL)
        rclpy.init()
        peer = Node("direction_test_peer")
        selector = DirectionZones()
        gate = CmdVelSafety(cli_args=["--ros-args", "-r", "/nav2/cmd_vel:=/nav2/cmd_vel_monitored"])
        executor = SingleThreadedExecutor()
        for node in (peer, selector, gate):
            executor.add_node(node)
        commands = peer.create_publisher(Twist, "/nav2/cmd_vel", 10)
        scans = peer.create_publisher(LaserScan, "/scan", qos_profile_sensor_data)
        odometry = peer.create_publisher(Odometry, "/odometry/local", qos_profile_sensor_data)
        corrections = peer.create_publisher(Header, "/localization_3d/accepted_correction", 10)
        footprints = peer.create_publisher(PolygonStamped, "/local_costmap/published_footprint", 10)
        broadcaster = TransformBroadcaster(peer)
        change = peer.create_client(ChangeState, "/collision_monitor/change_state")
        state = {"command": twist(), "obstacle": None}
        visualizations = {}
        for name in POLYGONS:
            peer.create_subscription(
                PolygonStamped, params[name]["polygon_pub_topic"],
                lambda message, name=name: visualizations.__setitem__(name, message), 10,
            )

        def pump():
            stamp = peer.get_clock().now().to_msg()
            tf = TransformStamped()
            tf.header.frame_id, tf.child_frame_id = "odom", "base_link"
            tf.header.stamp = stamp
            tf.transform.rotation.w = 1.0
            broadcaster.sendTransform(tf)
            scan = LaserScan()
            scan.header.frame_id, scan.header.stamp = "base_link", stamp
            scan.range_min, scan.range_max = 0.05, 10.0
            # Five returns around the obstacle bearing; otherwise free space.
            bearing, distance = state["obstacle"] or (0.0, 20.0)
            scan.angle_min, scan.angle_increment = bearing - 0.02, 0.01
            scan.angle_max = bearing + 0.02
            scan.ranges = [distance] * 5
            scans.publish(scan)
            corrections.publish(Header(frame_id="map", stamp=stamp))
            polygon = PolygonStamped()
            polygon.header = Header(frame_id="base_link", stamp=stamp)
            polygon.polygon.points = [Point32(x=x, y=y) for x, y in (
                (front, left), (front, right), (rear, right), (rear, left),
            )]
            footprints.publish(polygon)
            odom = Odometry()
            odom.header = Header(frame_id="odom", stamp=stamp)
            odom.child_frame_id = "base_link"
            odom.pose.pose.orientation.w = 1.0
            odometry.publish(odom)
            commands.publish(state["command"])
            until = time.monotonic() + 0.04
            while time.monotonic() < until:
                executor.spin_once(timeout_sec=0.005)

        def wait(predicate, timeout=8):
            end = time.monotonic() + timeout
            while time.monotonic() < end:
                pump()
                if predicate():
                    return
            self.fail(f"Direction zones did not reach expected state: active={selector.active}, "
                      f"output={(gate.last_output.linear.x, gate.last_output.angular.z)}")

        def hold_zero(seconds=1.5):
            end = time.monotonic() + seconds
            while time.monotonic() < end:
                pump()
                self.assertEqual(gate.last_output, Twist())

        try:
            self.assertTrue(change.wait_for_service(timeout_sec=5))
            for transition in (Transition.TRANSITION_CONFIGURE, Transition.TRANSITION_ACTIVATE):
                future = change.call_async(ChangeState.Request(transition=Transition(id=transition)))
                wait(future.done)
                self.assertTrue(future.result().success)
            state["command"] = twist(0.3)
            wait(lambda: selector.active == "forward" and gate.last_output.linear.x > 0.1
                 and set(POLYGONS) <= visualizations.keys()
                 and bool(visualizations["PolygonSurroundForward"].polygon.points)
                 and bool(visualizations["PolygonSlow"].polygon.points)
                 and not visualizations["PolygonSurroundReverse"].polygon.points
                 and not visualizations["PolygonRotate"].polygon.points)
            message = visualizations["PolygonSurroundForward"]
            self.assertEqual(message.header.frame_id, "base_link")
            self.assertAlmostEqual(max(p.x for p in message.polygon.points),
                                   params["PolygonSurroundForward"]["points"][0], places=6)
            # Inside the forward stop zone, outside the rotation circle.
            state["obstacle"] = (0.0, (radius + params["PolygonSurroundForward"]["points"][0]) / 2)
            wait(lambda: gate.last_output == Twist())
            hold_zero()
            state["command"] = twist(-0.2)
            wait(lambda: selector.active == "reverse" and gate.last_output.linear.x < -0.05
                 and bool(visualizations["PolygonSurroundReverse"].polygon.points)
                 and not visualizations["PolygonSurroundForward"].polygon.points)
            state["command"] = twist(0.0, 0.4)
            wait(lambda: selector.active == "rotate" and gate.last_output.angular.z > 0.1
                 and bool(visualizations["PolygonRotate"].polygon.points)
                 and not visualizations["PolygonSurroundReverse"].polygon.points)
            for point in visualizations["PolygonRotate"].polygon.points:
                self.assertAlmostEqual(math.hypot(point.x, point.y), radius, places=6)
            self.assertEqual(gate.last_output.linear.x, 0.0)
            state["obstacle"] = (math.pi / 2, radius - 0.02)
            wait(lambda: gate.last_output == Twist())
            hold_zero()
            state["obstacle"] = (math.pi, abs(rear) + 0.2)
            state["command"] = twist(-0.2)
            wait(lambda: selector.active == "reverse")
            hold_zero()
            state["obstacle"] = None
            state["command"] = twist(0.3)
            wait(lambda: selector.active == "forward" and gate.last_output.linear.x > 0.1
                 and bool(visualizations["PolygonSurroundForward"].polygon.points)
                 and not visualizations["PolygonRotate"].polygon.points
                 and bool(visualizations["PolygonSlow"].polygon.points))
            executor.remove_node(selector)
            wait(lambda: gate.last_output == Twist(), timeout=2)
        finally:
            for node in (peer, selector, gate):
                if node is not selector:
                    executor.remove_node(node)
                node.destroy_node()
            executor.shutdown()
            rclpy.shutdown()
            process.terminate()
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait(timeout=5)


if __name__ == "__main__":
    unittest.main()
