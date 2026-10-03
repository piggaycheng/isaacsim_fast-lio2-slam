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
from geometry_msgs.msg import Point32, PolygonStamped, TransformStamped, Twist, TwistStamped
from launch import LaunchContext
from launch.actions import SetLaunchConfiguration
from lifecycle_msgs.msg import Transition
from lifecycle_msgs.srv import ChangeState
from nav_msgs.msg import Odometry
from rcl_interfaces.msg import ParameterType, ParameterValue
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Header
from tf2_ros import TransformBroadcaster

PACKAGE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE / "scripts"))
from adaptive_surround import AdaptiveSurround
from cmd_vel_safety import CmdVelSafety

spec = importlib.util.spec_from_file_location("fusion_launch", PACKAGE / "launch/global_fusion.launch.py")
fusion_launch = importlib.util.module_from_spec(spec)
spec.loader.exec_module(fusion_launch)


class AdaptiveLaunchTest(unittest.TestCase):
    def test_static_mode_uses_original_configs_without_selector(self):
        context = LaunchContext()
        context.launch_configurations.update(adaptive_surround="false", direction_zones="false")
        paths = [str(PACKAGE / "config" / filename) for filename in (
            "collision_monitor.yaml", "navigation.yaml", "adaptive_surround.yaml",
            "direction_zones.yaml",
        )]
        actions = fusion_launch.configure_surround(context, *paths)
        self.assertEqual(len(actions), 2)
        for action in actions:
            self.assertIsInstance(action, SetLaunchConfiguration)
            action.execute(context)
        self.assertEqual(context.launch_configurations["collision_config"], paths[0])
        self.assertEqual(context.launch_configurations["navigation_config"], paths[1])

    def test_adaptive_mode_requires_complete_navigation_pipeline(self):
        context = LaunchContext()
        context.launch_configurations.update(
            adaptive_surround="true", navigate="false", costmaps="true", obstacle_cloud="true",
        )
        with self.assertRaisesRegex(ValueError, "requires navigation"):
            fusion_launch.configure_surround(context, "", "", "", "")


class AdaptiveTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = AdaptiveSurround()
        self.now = 10.0
        clock = Mock()
        clock.now.side_effect = lambda: Time(seconds=self.now)
        self.clock_patch = patch.object(self.node, "get_clock", return_value=clock)
        self.clock_patch.start()
        for name in ("publisher", "limits", "polygon", "getter", "setter"):
            setattr(self.node, name, Mock())
        self.node.checked = True
        self.node.active = "full"
        self.node.command = Twist()
        self.node.command.linear.x = 0.1
        self.node.motion = (0, 0)
        self.refresh()

    def tearDown(self):
        self.clock_patch.stop()
        self.node.destroy_node()

    def refresh(self):
        self.node.command_stamp = self.now
        self.node.odom_stamp = self.now

    def limits(self):
        return self.node.limits.publish.call_args.args[0]

    def begin_shrink(self):
        self.node.tick()
        self.now += 0.51
        self.refresh()
        self.node.tick()
        self.node.on_gate_ack(self.limits())
        self.node.tick()

    def acknowledge(self):
        self.node.future = Mock()
        self.node.future.done.return_value = True
        self.node.future.result.return_value = Mock(result=Mock(successful=True))
        self.node.tick()

    def test_full_limits_come_from_parameters(self):
        self.node.max_linear_speed, self.node.max_angular_speed = 0.4, 0.3
        self.node.tick()
        self.assertEqual((self.limits().twist.linear.x, self.limits().twist.angular.z), (0.4, 0.3))

    def test_shrink_requires_dwell_gate_ack_and_native_ack(self):
        self.node.tick()
        self.assertEqual(self.limits().twist.linear.x, 0.75)
        self.now += 0.49
        self.refresh()
        self.node.tick()
        self.assertEqual(self.node.active, "full")
        self.now += 0.02
        self.refresh()
        self.node.tick()
        self.assertEqual(self.limits().twist, Twist())
        self.node.setter.call_async.assert_not_called()
        self.node.on_gate_ack(self.limits())
        self.node.tick()
        request = self.node.setter.call_async.call_args.args[0]
        self.assertEqual([(p.name, p.value.bool_value) for p in request.parameters],
                         [("PolygonSurround.enabled", False), ("PolygonSurroundCrawl.enabled", True)])
        self.node.future.done.return_value = False
        self.node.tick()
        self.assertEqual(self.node.active, "full")
        self.assertEqual(self.limits().twist, Twist())
        self.acknowledge()
        self.assertEqual(self.node.active, "crawl")
        self.assertEqual(self.limits().twist.linear.x, 0.1)
        self.assertEqual(self.limits().twist.angular.z, 0.2)

    def test_expand_before_forwarding_fast_linear_or_angular_command(self):
        for linear, angular in ((0.5, 0), (0, 0.35), (-0.5, 0), (0, -0.35)):
            with self.subTest(linear=linear, angular=angular):
                self.node.active = "crawl"
                self.node.future = None
                self.node.zero_barrier_stamp = None
                self.node.gate_ack_stamp = None
                self.node.command.linear.x = float(linear)
                self.node.command.angular.z = float(angular)
                self.node.tick()
                self.assertEqual(self.node.publisher.publish.call_args.args[0], Twist())
                self.node.on_gate_ack(self.limits())
                self.node.tick()
                self.assertEqual(self.node.target, "full")
                self.acknowledge()
                self.assertEqual(self.node.publisher.publish.call_args.args[0].linear.x, linear)
                self.assertEqual(self.node.publisher.publish.call_args.args[0].angular.z, angular)

    def test_measured_linear_and_rotational_motion_prevent_shrink(self):
        for motion in ((0.121, 0), (0, 0.221)):
            self.node.motion = motion
            self.node.shrink_since = 1
            self.node.tick()
            self.assertEqual(self.node.active, "full")
            self.assertIsNone(self.node.shrink_since)
            self.node.setter.call_async.assert_not_called()

    def test_float32_commands_select_crawl_but_outputs_remain_hard_capped(self):
        self.node.command.linear.x = 0.10000000149011612
        self.node.command.angular.z = 0.20000000298023224
        self.begin_shrink()
        self.acknowledge()
        output = self.node.publisher.publish.call_args.args[0]
        self.assertEqual(output.linear.x, 0.1)
        self.assertEqual(output.angular.z, 0.2)
        self.node.command.linear.x = 0.101
        self.assertEqual(self.node.requested_profile(self.now), "full")

    def test_missing_stale_and_invalid_inputs_stop(self):
        for source in ("command", "motion", "command_stamp", "odom_stamp"):
            original = getattr(self.node, source)
            setattr(self.node, source, None if source in ("command", "motion") else 1.0)
            self.node.tick()
            self.assertEqual(self.limits().twist, Twist())
            setattr(self.node, source, original)
        self.node.command.linear.x = math.nan
        self.node.on_command(self.node.command)
        self.node.tick()
        self.assertEqual(self.limits().twist, Twist())

    def test_pose_difference_detects_inconsistent_zero_twist(self):
        message = Odometry()
        message.header.frame_id = "odom"
        message.child_frame_id = "base_link"
        message.pose.pose.orientation.w = 1.0
        message.header.stamp = Time(seconds=self.now).to_msg()
        self.node.on_odometry(message)
        self.now += 0.1
        message.header.stamp = Time(seconds=self.now).to_msg()
        message.pose.pose.position.x = 0.05
        self.node.on_odometry(message)
        self.assertAlmostEqual(self.node.motion[0], 0.5)
        self.assertEqual(self.node.requested_profile(self.now), "full")

    def test_odometry_ahead_of_clock_is_deferred_and_duplicates_do_not_refresh(self):
        message = Odometry()
        message.header.frame_id = "odom"
        message.child_frame_id = "base_link"
        message.pose.pose.orientation.w = 1.0
        message.header.stamp = Time(seconds=self.now + 0.02).to_msg()
        self.node.on_odometry(message)
        self.assertIsNone(self.node.previous_pose)
        self.assertIsNotNone(self.node.pending_odometry)
        self.now += 0.02
        self.node.tick()
        self.assertIsNone(self.node.pending_odometry)
        self.assertEqual(self.node.previous_pose[0], self.now)
        self.node.motion = (0.0, 0.0)
        self.now += 0.05
        self.node.on_odometry(message)
        self.assertEqual(self.node.odom_stamp, self.now - 0.05)
        self.assertEqual(self.node.motion, (0.0, 0.0))

    def test_clock_reset_invalidates_gate_ack_and_motion_evidence(self):
        self.node.tick()
        self.node.gate_ack_stamp = self.now
        self.now = 5.0
        self.node.getter.service_is_ready.return_value = False
        self.node.tick()
        self.assertIsNone(self.node.command)
        self.assertIsNone(self.node.motion)
        self.assertIsNone(self.node.gate_ack_stamp)
        self.assertFalse(self.node.checked)
        self.assertEqual(self.limits().twist, Twist())

    def test_rejected_and_timed_out_switches_fail_closed(self):
        self.begin_shrink()
        self.node.future.done.return_value = True
        self.node.future.result.return_value = Mock(result=Mock(successful=False, reason="test rejection"))
        with self.assertRaisesRegex(RuntimeError, "test rejection"):
            self.node.tick()
        self.assertEqual(self.limits().twist, Twist())
        self.node.future = Mock()
        self.node.future.done.return_value = False
        self.now += 1.01
        with self.assertRaisesRegex(RuntimeError, "timed out"):
            self.node.tick()
        self.assertEqual(self.limits().twist, Twist())

    def test_native_geometry_mismatch_fails_closed(self):
        self.node.future = Mock()
        self.node.future.done.return_value = True
        self.node.future.result.return_value = Mock(values=[
            ParameterValue(type=ParameterType.PARAMETER_DOUBLE_ARRAY, double_array_value=[0.0] * 8),
            ParameterValue(type=ParameterType.PARAMETER_DOUBLE_ARRAY, double_array_value=self.node.crawl_points),
            ParameterValue(type=ParameterType.PARAMETER_BOOL, bool_value=True),
            ParameterValue(type=ParameterType.PARAMETER_BOOL, bool_value=False),
        ])
        self.node.target = None
        with self.assertRaisesRegex(RuntimeError, "geometry mismatch"):
            self.node.tick()
        self.assertEqual(self.limits().twist, Twist())

    def test_geometry_and_speed_bounds_reject_unsafe_configuration(self):
        for name, value in (("crawl_linear", 0.11), ("crawl_angular", 0.21),
                            ("motion_margin", 0.03), ("max_linear_speed", 0.0),
                            ("max_angular_speed", float("nan")),
                            ("crawl_points", [0.9, 0.5, 0.9, -0.5, -0.45, -0.5, -0.45, 0.5]),
                            ("crawl_points", [0.85, 0.35, 0.85, -0.35, -0.35, -0.35, -0.35, 0.35]),
                            ("crawl_points", [0.85, 0.45, -0.35, -0.45, 0.85, -0.45, -0.35, 0.45])):
            with self.subTest(name=name):
                with self.assertRaises(ValueError):
                    AdaptiveSurround(parameter_overrides=[Parameter(name, value=value)])


class AdaptiveGateTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = CmdVelSafety(parameter_overrides=[
            Parameter("require_adaptive_limits", value=True),
            Parameter("max_linear_accel", value=100.0),
            Parameter("max_angular_accel", value=100.0),
        ])
        self.now = 10.0
        clock = Mock()
        clock.now.side_effect = lambda: Time(seconds=self.now)
        self.clock_patch = patch.object(self.node, "get_clock", return_value=clock)
        self.clock_patch.start()
        self.node.publisher = Mock()
        self.node.adaptive_ack = Mock()
        self.node.sensor_stamps["scan"] = self.now
        self.node.last_correction = self.now
        self.node.last_command_time = self.now - 0.1
        self.node.last_output_time = self.now - 0.1
        self.command = Twist()
        self.command.linear.x = 1.0
        self.command.angular.z = 0.75

    def tearDown(self):
        self.clock_patch.stop()
        self.node.destroy_node()

    def limits(self, linear=0.1, angular=0.2):
        message = TwistStamped()
        message.header.frame_id = "base_link"
        message.header.stamp = Time(seconds=self.now).to_msg()
        message.twist.linear.x, message.twist.angular.z = float(linear), float(angular)
        return message

    def test_missing_limits_and_expiry_stop_despite_fresh_commands(self):
        self.node.on_command(self.command)
        self.assertEqual(self.node.last_output, Twist())
        self.node.on_adaptive_limits(self.limits())
        self.now += 0.01
        self.node.on_command(self.command)
        self.assertEqual(self.node.last_output.linear.x, 0.1)
        self.now += 0.201
        self.node.on_watchdog()
        self.assertEqual(self.node.last_output, Twist())

    def test_zero_barrier_immediately_stops_and_acknowledges(self):
        self.node.on_adaptive_limits(self.limits(0.75, 0.5))
        self.node.on_command(self.command)
        self.assertEqual(self.node.last_output.linear.x, 0.75)
        message = self.limits(0, 0)
        self.node.on_adaptive_limits(message)
        self.assertEqual(self.node.last_output, Twist())
        self.node.adaptive_ack.publish.assert_called_with(message)
        self.node.on_command(self.command)
        self.assertEqual(self.node.last_output, Twist())

    def test_cap_reduction_stops_then_delayed_fast_commands_remain_capped(self):
        self.node.on_adaptive_limits(self.limits(0.75, 0.5))
        self.node.on_command(self.command)
        self.node.on_adaptive_limits(self.limits())
        self.assertEqual(self.node.last_output, Twist())
        self.now += 0.01
        self.node.on_command(self.command)
        self.assertEqual(self.node.last_output.linear.x, 0.1)
        self.assertEqual(self.node.last_output.angular.z, 0.2)

    def test_unchanged_heartbeat_does_not_reset_acceleration_clock(self):
        original = self.node.last_output_time
        self.node.on_adaptive_limits(self.limits())
        self.assertEqual(self.node.last_output_time, original)

    def test_bad_limits_are_not_acknowledged_and_stop_immediately(self):
        for kind in ("frame", "future", "stale", "nan", "negative", "overspeed", "nonplanar", "partial_zero"):
            with self.subTest(kind=kind):
                message = self.limits()
                if kind == "frame":
                    message.header.frame_id = "odom"
                elif kind in ("future", "stale"):
                    message.header.stamp = Time(seconds=self.now + (1 if kind == "future" else -1)).to_msg()
                elif kind == "nonplanar":
                    message.twist.linear.y = 0.1
                elif kind == "partial_zero":
                    message.twist.angular.z = 0.0
                else:
                    message.twist.linear.x = {"nan": math.nan, "negative": -0.1, "overspeed": 1.01}[kind]
                self.node.last_output = self.command
                self.node.adaptive_ack.reset_mock()
                self.node.on_adaptive_limits(message)
                self.assertEqual(self.node.last_output, Twist())
                self.node.adaptive_ack.publish.assert_not_called()

    def test_old_full_limits_cannot_replace_a_newer_crawl_limit(self):
        self.node.on_adaptive_limits(self.limits())
        old = self.limits(0.75, 0.5)
        old.header.stamp = Time(seconds=self.now - 0.05).to_msg()
        self.node.adaptive_ack.reset_mock()
        self.node.on_adaptive_limits(old)
        self.node.on_command(self.command)
        self.assertEqual(self.node.last_output, Twist())
        self.node.adaptive_ack.publish.assert_not_called()

    def test_future_limits_wait_for_local_clock_before_acknowledgment(self):
        message = self.limits(0, 0)
        message.header.stamp = Time(seconds=self.now + 0.02).to_msg()
        self.node.last_output = self.command
        self.node.on_adaptive_limits(message)
        self.node.adaptive_ack.publish.assert_not_called()
        self.assertIsNone(self.node.adaptive_limits)
        self.now += 0.02
        self.node.on_watchdog()
        self.assertEqual(self.node.last_output, Twist())
        self.node.adaptive_ack.publish.assert_called_with(message)


class RealHumbleSurroundTest(unittest.TestCase):
    def test_native_profiles_gate_barrier_odometry_loss_and_selector_death(self):
        context = LaunchContext()
        context.launch_configurations.update(
            adaptive_surround="true", navigate="true", costmaps="true", obstacle_cloud="true",
            costmap_config=str(PACKAGE / "config/observation_costmaps.yaml"),
        )
        factory = tempfile.TemporaryDirectory

        def directory(**kwargs):
            result = factory(**kwargs)
            self.addCleanup(result.cleanup)
            return result

        with patch.object(fusion_launch.tempfile, "TemporaryDirectory", side_effect=directory):
            actions = fusion_launch.configure_surround(context, *[
                str(PACKAGE / "config" / filename) for filename in (
                    "collision_monitor.yaml", "navigation.yaml", "adaptive_surround.yaml",
                    "direction_zones.yaml",
                )
            ])
        for action in actions:
            if isinstance(action, SetLaunchConfiguration):
                action.execute(context)
        filename = context.launch_configurations["collision_config"]
        config = yaml.safe_load(Path(filename).read_text())
        params = config["collision_monitor"]["ros__parameters"]
        self.assertEqual(params["PolygonSurround"]["points"], [0.8, 0.75, 0.8, -0.75, -1.35, -0.75, -1.35, 0.75])
        self.assertFalse(params["PolygonSurround"]["visualize"])
        self.assertTrue(config["cmd_vel_safety"]["ros__parameters"]["require_adaptive_limits"])
        controller = yaml.safe_load(Path(context.launch_configurations["navigation_config"]).read_text())
        self.assertEqual(controller["controller_server"]["ros__parameters"]["FollowPath"]["desired_linear_vel"], 0.1)
        self.assertEqual(controller["velocity_smoother"]["ros__parameters"]["max_velocity"],
                         [0.75, 0.0, 0.2])
        self.assertEqual(controller["velocity_smoother"]["ros__parameters"]["min_velocity"],
                         [-0.75, 0.0, -0.2])
        prefix = get_package_prefix("nav2_collision_monitor")
        process = subprocess.Popen([
            str(Path(prefix) / "lib/nav2_collision_monitor/collision_monitor"),
            "--ros-args", "--params-file", filename, "-p", "use_sim_time:=false",
        ], stdout=subprocess.DEVNULL)
        rclpy.init()
        peer = Node("surround_test_peer")
        selector = AdaptiveSurround()
        gate = CmdVelSafety(parameter_overrides=[Parameter("require_adaptive_limits", value=True)],
                            cli_args=["--ros-args", "-r", "/nav2/cmd_vel:=/nav2/cmd_vel_monitored"])
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
        command = Twist()
        command.linear.x = 0.1
        publish_odom = True

        def pump():
            stamp = peer.get_clock().now().to_msg()
            tf = TransformStamped()
            tf.header.frame_id, tf.child_frame_id = "odom", "base_link"
            tf.header.stamp = stamp
            tf.transform.rotation.w = 1.0
            broadcaster.sendTransform(tf)
            scan = LaserScan()
            scan.header.frame_id, scan.header.stamp = "base_link", stamp
            scan.angle_min, scan.angle_increment, scan.angle_max = 1.55, 0.01, 1.59
            scan.range_min, scan.range_max = 0.05, 10.0
            scan.ranges = [0.6] * 5
            scans.publish(scan)
            corrections.publish(Header(frame_id="map", stamp=stamp))
            polygon = PolygonStamped()
            polygon.header = Header(frame_id="base_link", stamp=stamp)
            polygon.polygon.points = [Point32(x=x, y=y) for x, y in (
                (0.21, 0.33), (0.21, -0.33), (-0.66, -0.33), (-0.66, 0.33),
            )]
            footprints.publish(polygon)
            if publish_odom:
                odom = Odometry()
                odom.header = Header(frame_id="odom", stamp=stamp)
                odom.child_frame_id = "base_link"
                odom.pose.pose.orientation.w = 1.0
                odometry.publish(odom)
            commands.publish(command)
            until = time.monotonic() + 0.04
            while time.monotonic() < until:
                executor.spin_once(timeout_sec=0.005)

        def wait(predicate, timeout=8):
            end = time.monotonic() + timeout
            while time.monotonic() < end:
                pump()
                if predicate():
                    return
            self.fail("Real Humble surround pipeline did not reach expected state")

        try:
            self.assertTrue(change.wait_for_service(timeout_sec=5))
            for transition in (Transition.TRANSITION_CONFIGURE, Transition.TRANSITION_ACTIVATE):
                future = change.call_async(ChangeState.Request(
                    transition=Transition(id=transition),
                ))
                wait(future.done)
                self.assertTrue(future.result().success)
            wait(lambda: selector.active == "crawl" and gate.last_output.linear.x > 0.05)
            self.assertLessEqual(gate.last_output.linear.x, 0.1)
            command.linear.x = 0.5
            wait(lambda: selector.active == "full" and gate.last_output == Twist())
            command.linear.x = 0.1
            wait(lambda: selector.active == "crawl" and gate.last_output.linear.x > 0.05)
            publish_odom = False
            wait(lambda: selector.active == "full" and gate.last_output == Twist())
            publish_odom = True
            wait(lambda: selector.active == "crawl" and gate.last_output.linear.x > 0.05)
            executor.remove_node(selector)
            wait(lambda: gate.last_output == Twist(), timeout=1)
        finally:
            for node in (peer, selector, gate):
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
