import importlib.util
import math
import subprocess
import sys
import tempfile
import time
import unittest
from pathlib import Path
from unittest.mock import Mock, patch

import rclpy
import yaml
from ament_index_python.packages import get_package_prefix
from geometry_msgs.msg import Point32, PolygonStamped, TransformStamped, Twist
from launch import LaunchContext
from launch.actions import SetLaunchConfiguration
from launch_ros.actions import Node as NodeAction
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

    def test_nova_zone_sets_are_derived_from_surround_and_footprint(self):
        context, actions = self.configure()
        params = self.monitor(context)
        self.assertEqual(params["cmd_vel_in_topic"], "/nav2/cmd_vel_direction")
        self.assertEqual(set(POLYGONS), set(params["polygons"]) - {"FootprintApproach", "PolygonSurround"})
        self.assertFalse(params["PolygonSurround"]["enabled"])
        self.assertEqual(params["PolygonSurroundForward"]["points"],
                         [0.8, 0.75, 0.8, -0.75, -0.81, -0.75, -0.81, 0.75])
        self.assertEqual(params["PolygonSurroundReverse"]["points"],
                         [0.36, 0.75, 0.36, -0.75, -0.81, -0.75, -0.81, 0.75])
        self.assertEqual(params["PolygonSurround"]["points"], [0.8, 0.75, 0.8, -0.75, -1.35, -0.75, -1.35, 0.75])
        enabled = {name for name in POLYGONS if params[name]["enabled"]}
        self.assertEqual(enabled, set(ZONE_SETS["forward"]))
        self.assertTrue(params["FootprintApproach"]["enabled"])
        for name in ("PolygonSurround", "PolygonSurroundForward", "PolygonSurroundReverse", "PolygonRotate"):
            self.assertEqual(params[name]["polygon_pub_topic"], "/collision_monitor/polygon_surround")
        self.assertEqual(params["PolygonRotate"], {
            "type": "circle", "radius": 0.81, "action_type": "stop", "max_points": 3,
            "visualize": True, "polygon_pub_topic": "/collision_monitor/polygon_surround", "enabled": False,
        })
        self.assertEqual(context.launch_configurations["navigation_config"], CONFIGS[1])
        self.assertTrue(any(isinstance(action, NodeAction) for action in actions))

    def test_carter_v1_zones_follow_its_profile(self):
        context, _ = self.configure("carter_v1")
        params = self.monitor(context)
        forward = params["PolygonSurroundForward"]["points"]
        reverse = params["PolygonSurroundReverse"]["points"]
        self.assertAlmostEqual(forward[4], -0.66)
        self.assertEqual(forward[:2], [0.71, 0.71])
        self.assertAlmostEqual(reverse[0], 0.51)
        self.assertEqual(reverse[4:6], [-0.71, -0.71])
        self.assertEqual(params["PolygonRotate"]["radius"], 0.71)

    def test_direction_surrounds_stay_within_planning_circle(self):
        for kind in ("nova_carter", "carter_v1"):
            context, _ = self.configure(kind)
            params = self.monitor(context)
            radius = params["PolygonRotate"]["radius"]
            for name in ("PolygonSurroundForward", "PolygonSurroundReverse"):
                with self.subTest(kind=kind, zone=name):
                    self.assertLessEqual(max(abs(value) for value in params[name]["points"]), radius)

    def test_rotation_circle_must_cover_footprint_sweep_and_planning(self):
        for kind in ("nova_carter", "carter_v1"):
            for key, value, message in (
                ("robot_radius", None, "rotation sweep"),
                ("inflation_layer", "robot_radius", "inflation_radius"),
                ("footprint", "[[0.2, 0.32], [0.2, -0.32], [-0.65, -0.32], [-0.65, 0.32]]", "robot_radius"),
                ("footprint_padding", 0.01, "footprint_padding"),
                ("obstacle_layer", {"footprint_clearing_enabled": True}, "rotation circle"),
            ):
                config = fusion_launch.load_parameters(str(PACKAGE / "config/observation_costmaps.yaml"), kind)
                planning = config["global_costmap"]["global_costmap"]["ros__parameters"]
                if value is None:
                    planning[key] -= 0.02
                elif value == "robot_radius":
                    planning[key]["inflation_radius"] = planning[value]
                elif isinstance(value, dict):
                    planning[key].update(value)
                else:
                    planning[key] = value
                with self.subTest(kind=kind, key=key), self.assertRaisesRegex(ValueError, message):
                    fusion_launch.rotation_radius(config)
        config = yaml.safe_load((PACKAGE / "config/observation_costmaps.yaml").read_text())
        config["global_costmap"]["global_costmap"]["ros__parameters"]["obstacle_layer"][
            "footprint_clearing_enabled"] = True
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "costmaps.yaml"
            path.write_text(yaml.safe_dump(config))
            with self.assertRaisesRegex(ValueError, "rotation circle"):
                self.configure(costmap_config=str(path))

    def test_static_without_navigation_or_when_disabled(self):
        for overrides in ({"navigate": "false"}, {"direction_zones": "false"}):
            context, actions = self.configure(**overrides)
            self.assertEqual(len(actions), 2)
            self.assertEqual(context.launch_configurations["collision_config"], CONFIGS[0])


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
        self.node.future = response({"PolygonSurroundReverse"})
        self.node.tick()
        self.assertEqual(self.node.active, "reverse")
        self.node.checked, self.node.active = False, None
        self.node.tick()
        self.node.future = response({"PolygonStop", "PolygonRotate"})
        self.node.setter.service_is_ready.return_value = True
        self.now += 0.05
        self.odometry()
        self.node.tick()
        self.assertIsNone(self.node.active)
        self.step(0.25)
        self.assertEqual(self.node.setter.call_async.call_args.args[0].parameters[0].name, "PolygonStop.enabled")
        self.assertEqual(self.node.target, "forward")

    def test_invalid_parameters_are_rejected(self):
        with self.assertRaisesRegex(ValueError, "settle_time"):
            DirectionZones(parameter_overrides=[rclpy.parameter.Parameter("settle_time", value=0.0)])


class RealHumbleDirectionZonesTest(unittest.TestCase):
    def test_front_obstacle_blocks_forward_but_allows_reverse_and_rotation(self):
        launch = DirectionLaunchTest()
        launch.addCleanup = self.addCleanup
        context, _ = launch.configure()
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
                (0.66, 0.33), (0.66, -0.33), (-0.21, -0.33), (-0.21, 0.33),
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
            wait(lambda: selector.active == "forward" and gate.last_output.linear.x > 0.1)
            # Inside PolygonStop (front 0.85 m) but outside the 0.81 m rotation circle.
            state["obstacle"] = (0.0, 0.825)
            wait(lambda: gate.last_output == Twist())
            hold_zero()
            state["command"] = twist(-0.2)
            wait(lambda: selector.active == "reverse" and gate.last_output.linear.x < -0.1)
            state["command"] = twist(0.0, 0.4)
            wait(lambda: selector.active == "rotate" and gate.last_output.angular.z > 0.2)
            self.assertEqual(gate.last_output.linear.x, 0.0)
            # Beside the robot outside the old 0.75 m surround but inside the rotation circle.
            state["obstacle"] = (math.pi / 2, 0.78)
            wait(lambda: gate.last_output == Twist())
            hold_zero()
            # Behind the robot inside the reverse surround, which stops at the rotation circle.
            state["obstacle"] = (math.pi, 0.78)
            state["command"] = twist(-0.2)
            wait(lambda: selector.active == "reverse")
            hold_zero()
            state["obstacle"] = None
            state["command"] = twist(0.3)
            wait(lambda: selector.active == "forward" and gate.last_output.linear.x > 0.1)
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
