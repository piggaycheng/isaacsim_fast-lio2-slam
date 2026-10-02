import json
import signal
import struct
import subprocess
import sys
import tempfile
import time
import unittest
from pathlib import Path

import numpy as np
import yaml
import rclpy
from ament_index_python.packages import get_package_prefix
from geometry_msgs.msg import TransformStamped
from lifecycle_msgs.msg import Transition
from lifecycle_msgs.srv import ChangeState
from rcl_interfaces.msg import ParameterType
from rcl_interfaces.srv import GetParameters
from nav_msgs.msg import OccupancyGrid
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan, PointCloud2, PointField
from std_msgs.msg import Header
from tf2_ros import TransformBroadcaster

sys.path.insert(0, str(Path(__file__).resolve().parents[4] / "tests"))
from validate_navigation_environment import (
    classify_clearance_event, clearance_metrics, configured_corridor_width, local_position, obstacle_polygon,
    planning_corridor_width, scenario, walls,
)


class NavigationEnvironmentTest(unittest.TestCase):
    def test_physical_footprints_and_global_obstacle_clearing(self):
        config_dir = Path(__file__).resolve().parents[1] / "config"
        config = yaml.safe_load((config_dir / "observation_costmaps.yaml").read_text())
        global_params = config["global_costmap"]["global_costmap"]["ros__parameters"]
        local_params = config["local_costmap"]["local_costmap"]["ros__parameters"]
        for params in (global_params, local_params):
            self.assertEqual(json.loads(params["footprint"]),
                             [[0.65, 0.32], [0.65, -0.32], [-0.2, -0.32], [-0.2, 0.32]])
            self.assertEqual(params["footprint_padding"], 0.01)
            self.assertEqual(params["inflation_layer"]["inflation_radius"], 0.9)
            self.assertTrue(params["obstacle_layer"]["scan_clearing"]["clearing"])
            self.assertFalse(params["obstacle_layer"]["obstacles"]["clearing"])
        self.assertTrue(global_params["obstacle_layer"].get("footprint_clearing_enabled", True))
        self.assertTrue(local_params["obstacle_layer"].get("footprint_clearing_enabled", True))
        self.assertAlmostEqual(planning_corridor_width(), 0.66)
        self.assertAlmostEqual(configured_corridor_width(), 1.5)
        self.assertTrue(scenario("corridor_1.4")[2])
        self.assertFalse(scenario("corridor_1.6")[2])
        self.assertFalse(scenario("corridor_1.8")[2])
        self.assertAlmostEqual(configured_corridor_width(adaptive_surround=True), 1.1)
        self.assertFalse(scenario("corridor_1.4", configured_corridor_width(adaptive_surround=True))[2])
        self.assertTrue(scenario("corridor_0.6", configured_corridor_width(adaptive_surround=True))[2])

    def crossing_evidence(self):
        samples = [{"stamp": float(t), "position": [0, 0], "yaw": 0,
                    "speed": 0, "angular_speed": 0} for t in np.arange(0, 0.82, 0.02)]
        commands = [{"stamp": row["stamp"], "source": "final", "linear": 0, "angular": 0}
                    for row in samples]
        actor = {"center": [0.8, -1, 0.6], "velocity": [0, 1],
                 "duration": 2, "sizes": [0.4, 0.4, 1.2], "yaw": 0}
        return samples, commands, actor

    def classify(self, samples, commands, actor, verified=True):
        return classify_clearance_event(samples, commands, [actor], 0,
                                        [-0.2, 0.65, -0.32, 0.32], 0, verified,
                                        [row["stamp"] for row in samples])

    def test_stationary_actor_intrusion_is_diagnosed_but_overlap_remains(self):
        samples, commands, actor = self.crossing_evidence()
        result = self.classify(samples, commands, actor)
        self.assertEqual(result["clearance_classification"], "stationary_moving_actor_intrusion")
        self.assertTrue(result["stop_before_intrusion_verified"])
        self.assertTrue(clearance_metrics(samples, [actor], 0, [-0.2, 0.65, -0.32, 0.32],
                                          [-0.2, 0.65, -0.32, 0.32])["geometric_overlap"])

    def test_translation_and_rotation_before_intrusion_are_not_passive(self):
        for key in ("speed", "angular_speed"):
            samples, commands, actor = self.crossing_evidence()
            samples[15][key] = 0.1
            result = self.classify(samples, commands, actor)
            self.assertEqual(result["clearance_classification"], "insufficient_stationary_dwell")
            self.assertFalse(result["stop_before_intrusion_verified"])

    def test_robot_moving_at_first_breach_is_distinct_from_recent_motion(self):
        samples, commands, actor = self.crossing_evidence()
        event = self.classify(samples, commands, actor)["first_clearance_breach_sim_s"]
        next(row for row in samples if row["stamp"] == event)["speed"] = 0.1
        result = self.classify(samples, commands, actor)
        self.assertEqual(result["clearance_classification"], "robot_motion_clearance_breach")
        self.assertFalse(result["stop_before_intrusion_verified"])

    def test_missing_or_stale_evidence_does_not_certify_stationary_intrusion(self):
        for mode in ("angular", "commands", "command_gap", "truth_gap", "telemetry", "pose_drift", "nan_truth", "nan_command"):
            samples, commands, actor = self.crossing_evidence()
            if mode == "angular":
                del samples[15]["angular_speed"]
            elif mode == "commands":
                commands[15]["angular"] = 0.1
            elif mode == "command_gap":
                commands = commands[:2]
            elif mode == "truth_gap":
                samples = samples[:2] + samples[20:]
            elif mode == "pose_drift":
                samples[15]["position"] = [0.02, 0]
            elif mode == "nan_truth":
                samples[15]["speed"] = float("nan")
            elif mode == "nan_command":
                commands[15]["angular"] = float("nan")
            result = self.classify(samples, commands, actor, verified=mode != "telemetry")
            self.assertFalse(result["stop_before_intrusion_verified"], mode)
            self.assertEqual(result["clearance_classification"], "clearance_breach_unverified", mode)

    def test_static_obstacle_and_insufficient_history_are_not_passive(self):
        samples, commands, actor = self.crossing_evidence()
        actor.update(center=[0.8, -0.4, 0.6], velocity=[0, 0])
        self.assertFalse(self.classify(samples, commands, actor)["stop_before_intrusion_verified"])
        samples, commands, actor = self.crossing_evidence()
        self.assertFalse(self.classify(samples[23:], commands, actor)["stop_before_intrusion_verified"])

    def test_stale_actor_telemetry_cannot_establish_passive_intrusion(self):
        samples, commands, actor = self.crossing_evidence()
        for stamps in ([], [0, 0.02]):
            result = classify_clearance_event(samples, commands, [actor], 0,
                                               [-0.2, 0.65, -0.32, 0.32], 0, True, stamps)
            self.assertFalse(result["stop_before_intrusion_verified"])
            self.assertEqual(result["clearance_classification"], "clearance_breach_unverified")

    def test_continuous_allowance_is_used_for_first_breach(self):
        samples, commands, actor = self.crossing_evidence()
        earlier = classify_clearance_event(samples, commands, [actor], 0,
                                           [-0.2, 0.65, -0.32, 0.32], 0.03, True,
                                           [row["stamp"] for row in samples])
        self.assertLess(earlier["first_clearance_breach_sim_s"],
                        self.classify(samples, commands, actor)["first_clearance_breach_sim_s"])
        actor["center"][0] = 2
        self.assertEqual(self.classify(samples, commands, actor)["clearance_classification"],
                         "no_clearance_breach")

    def test_unknown_scenario_is_rejected(self):
        with self.assertRaisesRegex(ValueError, "Unknown"):
            scenario("unsupported")

    def test_corridor_inner_width_and_expected_rejection(self):
        for width in (0.6, 1.4, 1.6, 1.8):
            boxes = walls(width)
            self.assertAlmostEqual(
                boxes[1]["offset"][1] - boxes[1]["sizes"][1] / 2, width / 2,
            )
            self.assertAlmostEqual(
                boxes[0]["offset"][1] + boxes[0]["sizes"][1] / 2, -width / 2,
            )
            _, distance, blocked = scenario(f"corridor_{width}")
            self.assertEqual(blocked, width <= configured_corridor_width())
            self.assertEqual(distance, 3.5)

    def test_opposite_crossings_have_real_motion(self):
        for name, speed in (("crossing_slow", 0.35), ("crossing_fast", 0.65)):
            definitions, _, blocked = scenario(name)
            actor = definitions[0]
            self.assertFalse(blocked)
            self.assertAlmostEqual(abs(actor["velocity"][1]), speed)
            self.assertLess(actor["offset"][1] * actor["velocity"][1], 0)
            self.assertLess(
                actor["offset"][1] *
                (actor["offset"][1] + actor["duration"] * actor["velocity"][1]), 0,
            )

    def test_gate_release_isolates_open_space_from_corridor_deadlock(self):
        definitions, _, blocked = scenario("blocked_then_clear")
        self.assertFalse(blocked)
        self.assertEqual(definitions[:2], walls(1.8))
        self.assertEqual(definitions[2]["sizes"][1], 1.8)
        definitions, _, blocked = scenario("blocked_then_clear_open")
        self.assertFalse(blocked)
        self.assertEqual(len(definitions), 1)
        actor = definitions[0]
        self.assertGreater(actor["offset"][0] - actor["sizes"][0] / 2, 0.65)
        self.assertLess(actor["offset"][0], 1.3)
        self.assertGreater(actor["velocity"][1] * actor["duration"], actor["sizes"][1])

    def test_world_to_forward_left_and_clamped_motion(self):
        np.testing.assert_allclose(local_position([1, 3], [2, 3, np.pi]), [1, 0], atol=1e-9)
        obstacle = {
            "center": [2, 3, 0.6], "velocity": [0, -0.5],
            "duration": 2, "sizes": [0.2, 0.6, 1.2], "yaw": 0,
        }
        np.testing.assert_allclose(obstacle_polygon(obstacle, 10, None).mean(axis=0), [2, 3])
        np.testing.assert_allclose(obstacle_polygon(obstacle, 11, 10).mean(axis=0), [2, 2.5])
        np.testing.assert_allclose(obstacle_polygon(obstacle, 20, 10).mean(axis=0), [2, 2])

    def test_clearance_accounts_for_overlap_and_sampling_motion(self):
        samples = [
            {"stamp": t, "position": [0, 0], "yaw": 0} for t in (1, 1.02, 1.04)
        ]
        bounds = [-0.2, 0.65, -0.32, 0.32]
        obstacle = {
            "center": [1, 0, 0.6], "velocity": [0, 0],
            "duration": 0, "sizes": [0.2, 0.6, 1.2], "yaw": 0,
        }
        result = clearance_metrics(samples, [obstacle], None, bounds, bounds)
        self.assertAlmostEqual(result["min_body_separation_m"], 0.25)
        self.assertLess(result["min_continuous_clearance_lower_bound_m"], 0.25)
        self.assertFalse(result["geometric_overlap"])
        obstacle["center"][0] = 0.5
        self.assertTrue(clearance_metrics(samples, [obstacle], None, bounds, bounds)["geometric_overlap"])
        with self.assertRaises(ValueError):
            clearance_metrics(samples[:1], [], None, bounds, bounds)
        with self.assertRaises(ValueError):
            clearance_metrics([samples[0], samples[0]], [], None, bounds, bounds)


class GlobalObstacleClearingTest(unittest.TestCase):
    def test_only_physical_footprint_is_cleared_and_scan_still_clears(self):
        config_path = Path(__file__).resolve().parents[1] / "config/observation_costmaps.yaml"
        params = yaml.safe_load(config_path.read_text())["global_costmap"]["global_costmap"]["ros__parameters"]
        params.update(use_sim_time=False, global_frame="map", robot_base_frame="clearing_test_base",
                      width=4, height=4, origin_x=-2.0, origin_y=-2.0, resolution=0.05,
                      plugins=["obstacle_layer"], update_frequency=10.0, publish_frequency=10.0)
        executable = Path(get_package_prefix("nav2_costmap_2d")) / "lib/nav2_costmap_2d/nav2_costmap_2d"
        rclpy.init()
        node = rclpy.create_node("global_obstacle_clearing_test")
        process = None
        try:
            with tempfile.TemporaryDirectory() as directory, tempfile.TemporaryFile(mode="w+") as log:
                filename = Path(directory) / "costmap.yaml"
                filename.write_text(yaml.safe_dump({"/**": {"ros__parameters": params}}))
                process = subprocess.Popen(
                    [str(executable), "--ros-args", "--params-file", str(filename)], stdout=log, stderr=log,
                )
                broadcaster = TransformBroadcaster(node)
                cloud = node.create_publisher(PointCloud2, "/perception/obstacles", 10)
                scan = node.create_publisher(LaserScan, "/scan", 10)
                points = [[0.31, 0.11, 0.5], [0.91, 0.61, 0.5]]
                state = {"points": points, "clear": False}
                maps = []
                subscriptions = []

                def publish():
                    stamp = node.get_clock().now().to_msg()
                    transform = TransformStamped()
                    transform.header = Header(frame_id="map", stamp=stamp)
                    transform.child_frame_id = "clearing_test_base"
                    transform.transform.rotation.w = 1.0
                    broadcaster.sendTransform(transform)
                    cloud.publish(PointCloud2(
                        header=Header(frame_id="map", stamp=stamp), height=1,
                        width=len(state["points"]), is_bigendian=False, is_dense=True,
                        fields=[PointField(name=name, offset=i * 4, datatype=PointField.FLOAT32, count=1)
                                for i, name in enumerate(("x", "y", "z"))],
                        point_step=12, row_step=len(state["points"]) * 12,
                        data=b"".join(struct.pack("<fff", *point) for point in state["points"]),
                    ))
                    if state["clear"]:
                        angles = [float(np.arctan2(point[1], point[0])) for point in points]
                        scan.publish(LaserScan(
                            header=Header(frame_id="clearing_test_base", stamp=stamp),
                            angle_min=angles[0], angle_max=angles[1],
                            angle_increment=angles[1] - angles[0],
                            range_min=0.0, range_max=7.0, ranges=[2.0, 2.0],
                        ))

                def wait(predicate, seconds=20):
                    deadline = time.monotonic() + seconds
                    while time.monotonic() < deadline:
                        publish()
                        rclpy.spin_once(node, timeout_sec=0.03)
                        if predicate():
                            return
                        if process.poll() is not None:
                            break
                    log.seek(0)
                    self.fail("Obstacle clearing regression timed out\n" + log.read()[-8000:])

                services = []
                def discover():
                    services[:] = [name for name, types in node.get_service_names_and_types()
                                   if name.startswith("/costmap/")
                                   and "lifecycle_msgs/srv/ChangeState" in types]
                    return len(services) == 1
                wait(discover)
                client = node.create_client(ChangeState, services[0])
                wait(client.service_is_ready)
                for transition in (Transition.TRANSITION_CONFIGURE, Transition.TRANSITION_ACTIVATE):
                    future = client.call_async(ChangeState.Request(transition=Transition(id=transition)))
                    wait(future.done)
                    self.assertTrue(future.result().success)

                parameter_client = node.create_client(
                    GetParameters, services[0].rsplit("/", 1)[0] + "/get_parameters",
                )
                wait(parameter_client.service_is_ready)
                future = parameter_client.call_async(GetParameters.Request(names=[
                    "footprint", "obstacle_layer.footprint_clearing_enabled", "robot_base_frame",
                ]))
                wait(future.done)
                values = future.result().values
                self.assertEqual(json.loads(values[0].string_value), json.loads(params["footprint"]))
                self.assertEqual(values[1].type, ParameterType.PARAMETER_BOOL)
                self.assertTrue(values[1].bool_value)
                self.assertEqual(values[2].string_value, "clearing_test_base")

                def subscribe():
                    topics = [name for name, types in node.get_topic_names_and_types()
                              if name.startswith("/costmap/") and "nav_msgs/msg/OccupancyGrid" in types]
                    if len(topics) != 1:
                        return False
                    subscriptions.append(node.create_subscription(
                        OccupancyGrid, topics[0], maps.append,
                        QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                                   reliability=ReliabilityPolicy.RELIABLE),
                    ))
                    return True
                wait(subscribe)

                def cost(point):
                    message = maps[-1]
                    col = int((point[0] - message.info.origin.position.x) / message.info.resolution)
                    row = int((point[1] - message.info.origin.position.y) / message.info.resolution)
                    return message.data[row * message.info.width + col]

                # The old planning envelope must not clear returns outside the physical footprint.
                wait(lambda: maps and cost(points[0]) == 0 and cost(points[1]) == 100)
                state.update(points=[], clear=True)
                wait(lambda: maps and all(cost(point) == 0 for point in points))
                process.send_signal(signal.SIGINT)
                process.wait(timeout=20)
        finally:
            if process is not None and process.poll() is None:
                process.kill()
                process.wait(timeout=10)
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()


if __name__ == "__main__":
    unittest.main()
