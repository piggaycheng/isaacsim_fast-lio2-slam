import copy
import importlib.util
import math
import sys
import tempfile
import unittest
from pathlib import Path

import yaml
from launch import LaunchContext

PACKAGE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE / "launch"))
import robot_fleet  # noqa: E402
import robot_namespace  # noqa: E402

SPEC = importlib.util.spec_from_file_location(
    "fusion_launch_ns", PACKAGE / "launch/global_fusion.launch.py",
)
FUSION = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(FUSION)
CONFIG = PACKAGE / "config"


def write_profile(directory, name, **changes):
    profile = yaml.safe_load((CONFIG / "robots/nova_carter.yaml").read_text())
    profile.update(changes)
    (Path(directory) / f"{name}.yaml").write_text(yaml.safe_dump(profile))


class RobotSpecTest(unittest.TestCase):
    def test_parses_type_and_optional_yaw(self):
        first, second = robot_fleet.parse_robot_specs("carter1@0,0; carter2:nova_carter@2,-3,1.5")
        self.assertEqual(first, robot_fleet.RobotSpec("carter1", "nova_carter", 0.0, 0.0, 0.0))
        self.assertEqual(second, robot_fleet.RobotSpec("carter2", "nova_carter", 2.0, -3.0, 1.5))
        self.assertEqual(
            robot_fleet.parse_robot_specs(robot_fleet.format_robot_specs([first, second])),
            [first, second],
        )

    def test_rejects_invalid_fleets(self):
        for text, message in (
            ("", "At least one"),
            ("carter1", "NAME"),
            ("1robot@0,0", "Invalid robot namespace"),
            ("carter1@0", "X,Y"),
            ("carter1@0,nan", "X,Y"),
            ("carter1:bad-type@0,0", "NAME|Invalid robot type"),
            ("a@0,0;a@5,0", "unique"),
            ("Carter1@0,0;carter1@5,0", "unique"),
            ("a@0,0;b@1,0", "closer"),
        ):
            with self.subTest(text=text), self.assertRaisesRegex(ValueError, message):
                robot_fleet.parse_robot_specs(text)


class RobotProfileTest(unittest.TestCase):
    def test_nova_carter_profile_matches_sensor_mount(self):
        profile = robot_fleet.load_robot_profile("nova_carter")
        self.assertEqual(profile["simulation"]["wheel_joints"],
                         ["joint_wheel_left", "joint_wheel_right"])
        self.assertAlmostEqual(profile["sensor_frames"]["lidar_link"]["yaw"], math.pi, places=6)

    def test_nova_carter_overrides_match_base_configs(self):
        # Required vehicle parameters exist only in profiles; the remaining overrides
        # (footprint, safety zones, speeds) keep Nova Carter's values as base defaults.
        overrides = robot_fleet.load_robot_profile("nova_carter")["parameter_overrides"]
        for node in ("wheel_encoder_odometry", "nav_imu_adapter", "global_pose_adapter",
                     "local_costmap", "collision_monitor", "ground_obstacle_filter"):
            self.assertIn(node, overrides)
        defaults = copy.deepcopy(overrides)
        for node, keys in robot_fleet.REQUIRED_OVERRIDES.items():
            for key in keys:
                del defaults[node]["ros__parameters"][key]
        used = set()
        for path in (*CONFIG.glob("*.yaml"), *(PACKAGE.parent / "slam_nav/config").glob("*.yaml")):
            config = yaml.safe_load(path.read_text())
            used |= set(overrides) & set(config)
            with self.subTest(path=path.name):
                self.assertEqual(robot_fleet.merge_overrides(config, defaults), config)
                for node, keys in robot_fleet.REQUIRED_OVERRIDES.items():
                    parameters = config.get(node, {}).get("ros__parameters", {})
                    self.assertFalse(set(keys) & set(parameters), (node, path.name))
        self.assertEqual(used, set(overrides))

    def test_profiles_must_define_vehicle_parameters(self):
        for robot_type in ("nova_carter", "carter_v1"):
            robot_fleet.load_robot_profile(robot_type)
        overrides = robot_fleet.load_robot_profile("nova_carter")["parameter_overrides"]
        for node, keys in robot_fleet.REQUIRED_OVERRIDES.items():
            with self.subTest(node=node), tempfile.TemporaryDirectory() as directory:
                incomplete = copy.deepcopy(overrides)
                del incomplete[node]["ros__parameters"][keys[-1]]
                write_profile(directory, "incomplete", parameter_overrides=incomplete)
                with self.assertRaisesRegex(ValueError, f"{node}.ros__parameters needs {keys[-1]}"):
                    robot_fleet.load_robot_profile("incomplete", directory)

    def test_carter_v1_has_independent_geometry_and_drive_parameters(self):
        profile = robot_fleet.load_robot_profile("carter_v1")
        simulation = profile["simulation"]
        self.assertEqual(simulation["wheel_joints"], ["left_wheel", "right_wheel"])
        self.assertEqual(simulation["forward_sign"], 1.0)
        self.assertEqual(robot_fleet.imu_mount(profile), [-0.06, 0.0, 0.50, 0.0, 0.0, 0.0])
        self.assertEqual(simulation["lidar_translation"], robot_fleet.imu_mount(profile)[:3])
        overrides = profile["parameter_overrides"]
        wheel = overrides["wheel_encoder_odometry"]["ros__parameters"]
        self.assertEqual(wheel["wheel_radius"], simulation["wheel_radius"])
        self.assertEqual(wheel["wheel_base"], simulation["wheel_base"])
        self.assertEqual([wheel["left_joint"], wheel["right_joint"]], simulation["wheel_joints"])
        config = yaml.safe_load((CONFIG / "observation_costmaps.yaml").read_text())
        merged = robot_fleet.merge_overrides(config, overrides)
        self.assertNotEqual(merged, config)
        footprint = merged["local_costmap"]["local_costmap"]["ros__parameters"]["footprint"]
        self.assertEqual(footprint,
                         merged["global_costmap"]["global_costmap"]["ros__parameters"]["footprint"])
        self.assertEqual(overrides["ground_obstacle_filter"]["ros__parameters"]["self_filter_bounds"],
                         [-0.50, 0.35, -0.38, 0.38])
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "local_odometry.yaml"
            path.write_text(yaml.safe_dump({"wheel_encoder_odometry": {"ros__parameters": {}}}))
            merged_path = robot_namespace.robot_parameter_file(str(path), "carter2", "carter_v1")
            result = yaml.safe_load(Path(merged_path).read_text())
            self.assertEqual(result["carter2"]["wheel_encoder_odometry"]["ros__parameters"], wheel)

    def test_unknown_and_incomplete_profiles_fail_early(self):
        with tempfile.TemporaryDirectory() as directory:
            write_profile(directory, "no_frames", sensor_frames={"lidar_link": {}})
            with self.assertRaisesRegex(ValueError, "available: \\['no_frames'\\]"):
                robot_fleet.load_robot_profile("missing", directory)
            with self.assertRaisesRegex(ValueError, "sensor_frames"):
                robot_fleet.load_robot_profile("no_frames", directory)

    def test_invalid_lidar_mount_override_is_rejected(self):
        for translation in ([0.0, 0.0], [0.0, 0.0, float("nan")], "0,0,0"):
            with self.subTest(translation=translation), tempfile.TemporaryDirectory() as directory:
                simulation = dict(robot_fleet.load_robot_profile("carter_v1")["simulation"])
                simulation["lidar_translation"] = translation
                write_profile(directory, "invalid_mount", simulation=simulation)
                with self.assertRaisesRegex(ValueError, "lidar_translation"):
                    robot_fleet.load_robot_profile("invalid_mount", directory)

    def test_overrides_only_touch_nodes_in_the_file(self):
        config = {"a": {"ros__parameters": {"x": 1, "nested": {"y": 2, "z": 3}}}}
        merged = robot_fleet.merge_overrides(config, {
            "a": {"ros__parameters": {"nested": {"y": 5}}},
            "b": {"ros__parameters": {"x": 9}},
        })
        self.assertEqual(merged, {"a": {"ros__parameters": {"x": 1, "nested": {"y": 5, "z": 3}}}})
        self.assertEqual(config["a"]["ros__parameters"]["nested"]["y"], 2)


class InitialPoseTest(unittest.TestCase):
    def setUp(self):
        self.profile = robot_fleet.load_robot_profile("nova_carter")

    def pose(self, x, y, yaw, profile=None):
        return robot_fleet.initial_pose(
            robot_fleet.RobotSpec("r", "nova_carter", x, y, yaw), profile or self.profile,
        )

    def test_map_reference_spawn_is_identity(self):
        self.assertEqual(self.pose(0, 0, 0), (0.0, 0.0, 0.0, 0.0))

    def test_translation_and_rotation_about_body_origin(self):
        self.assertEqual(self.pose(3.5, -1, 0), (3.5, -1.0, 0.0, 0.0))
        x, y, z, yaw = self.pose(3.5, 0, math.pi / 2)
        # Map origin is the body at the reference spawn: base_link faces prim -x
        # and imu_link sits 0.213 m ahead, 0.009 m right of base_link.
        self.assertAlmostEqual(x, 3.5 + 0.213 - 0.009, places=6)
        self.assertAlmostEqual(y, -0.213 - 0.009, places=6)
        self.assertEqual(z, 0.0)
        self.assertAlmostEqual(yaw, math.pi / 2, places=6)

    def test_other_robot_type_uses_its_own_mount(self):
        profile = robot_fleet.load_robot_profile("nova_carter")
        profile["simulation"]["forward_sign"] = 1.0
        profile["sensor_frames"]["imu_link"] = dict(
            x=0.0, y=0.0, z=0.3, roll=0.0, pitch=0.0, yaw=0.0,
        )
        x, y, z, yaw = self.pose(0, 0, 0, profile)
        self.assertAlmostEqual(x, 0.213, places=6)
        self.assertAlmostEqual(y, -0.009, places=6)
        self.assertAlmostEqual(z, 0.3 - 0.526, places=6)
        self.assertAlmostEqual(yaw, 0.0, places=6)


class NamespaceConfigTest(unittest.TestCase):
    def load(self, name, namespace):
        path = str(CONFIG / name)
        return yaml.safe_load(Path(
            robot_namespace.robot_parameter_file(path, namespace, "nova_carter")
        ).read_text())

    def test_root_namespace_reuses_original_files(self):
        for name in ("navigation.yaml", "collision_monitor.yaml", "fast_lio_localization_3d.yaml"):
            path = str(CONFIG / name)
            self.assertEqual(robot_namespace.robot_parameter_file(path, "", "nova_carter"), path)
        rviz = str(CONFIG / "global_fusion.rviz")
        self.assertEqual(robot_namespace.namespaced_rviz_file(rviz, ""), rviz)

    def test_nav2_topics_and_nested_costmaps_are_namespaced(self):
        config = self.load("observation_costmaps.yaml", "carter1")
        self.assertEqual(set(config), {"carter1"})
        local = config["carter1"]["local_costmap"]["local_costmap"]["ros__parameters"]
        sources = [
            local[layer][source]
            for layer in ("obstacle_layer", "voxel_layer") if layer in local
            for source in local[layer]["observation_sources"].split()
        ]
        self.assertTrue(sources)
        for source in sources:
            self.assertTrue(source["topic"].startswith("/carter1/"), source)
        bt = self.load("navigation.yaml", "carter1")["carter1"]["bt_navigator"]["ros__parameters"]
        self.assertEqual(bt["odom_topic"], "/carter1/odometry/local")

    def test_wildcard_and_robot_localization_inputs(self):
        fastlio = self.load("fast_lio_localization_3d.yaml", "carter1")
        common = fastlio["/**"]["ros__parameters"]["common"]
        self.assertEqual(common["lid_topic"], "/carter1/livox/lidar")
        fusion = self.load("global_fusion.yaml", "carter1")["carter1"]
        ekf = fusion["global_ekf"]["ros__parameters"]
        inputs = [value for key, value in ekf.items()
                  if key[:-1] in ("odom", "pose", "imu", "twist") and key[-1].isdigit()]
        self.assertTrue(inputs)
        self.assertTrue(all(value.startswith("/carter1/") for value in inputs), inputs)

    def test_global_topics_stay_global(self):
        self.assertEqual(robot_fleet.namespaced_topic("carter1", "/clock"), "/clock")
        self.assertEqual(robot_fleet.namespaced_topic("carter1", "relative"), "relative")
        self.assertEqual(robot_fleet.namespaced_topic("/carter1/", "/scan"), "/carter1/scan")

    def test_rviz_topics_follow_robot(self):
        path = robot_namespace.namespaced_rviz_file(str(CONFIG / "global_fusion.rviz"), "carter2")
        topics = []

        def walk(value):
            if isinstance(value, dict):
                for key, item in value.items():
                    if key == "Topic" and isinstance(item, dict) and "Value" in item:
                        topics.append(item["Value"])
                    walk(item)
            elif isinstance(value, list):
                for item in value:
                    walk(item)

        walk(yaml.safe_load(Path(path).read_text()))
        self.assertIn("/carter2/goal_pose", topics)
        self.assertTrue(all(
            topic.startswith("/carter2/") or not topic.startswith("/") for topic in topics
        ), topics)


class LaunchNamespaceTest(unittest.TestCase):
    def context(self, namespace):
        context = LaunchContext()
        context.launch_configurations.update(namespace=namespace, robot_type="nova_carter")
        return context

    def test_namespace_isolates_tf_without_sharing_parent_remaps(self):
        context = self.context("carter1")
        parent = [("/foo", "/bar")]
        context.launch_configurations["ros_remaps"] = parent
        for action in FUSION.push_robot_namespace(context):
            action.execute(context)
        self.assertEqual(context.launch_configurations["ros_namespace"], "/carter1")
        self.assertIn(("/tf", "/carter1/tf"), context.launch_configurations["ros_remaps"])
        self.assertIn(("/tf_static", "/carter1/tf_static"), context.launch_configurations["ros_remaps"])
        self.assertEqual(parent, [("/foo", "/bar")])

    def test_root_namespace_is_unchanged(self):
        context = self.context("")
        self.assertEqual(FUSION.push_robot_namespace(context), [])
        self.assertNotIn("ros_remaps", context.launch_configurations)

    def test_sensor_transforms_follow_profile(self):
        nodes = FUSION.sensor_transforms(self.context("carter1"))
        arguments = {
            node._Node__arguments[-1]: node._Node__arguments for node in nodes
        }
        self.assertEqual(set(arguments), {"imu_link", "lidar_link"})
        lidar = arguments["lidar_link"]
        self.assertEqual(lidar[lidar.index("--z") + 1], "0.526")
        self.assertEqual(lidar[lidar.index("--frame-id") + 1], "base_link")

    def test_imu_mount_parameter_follows_profile(self):
        profile = robot_fleet.load_robot_profile("nova_carter")
        mount = profile["sensor_frames"]["imu_link"]
        self.assertEqual(robot_fleet.imu_mount(profile),
                         [mount[key] for key in robot_fleet.FRAME_KEYS])

    def test_event_started_nodes_get_explicit_namespace(self):
        actions = FUSION.readiness_actions("carter1", True, '[""]')
        event = type("Event", (), {"returncode": 0})()
        started = actions[1].event_handler.handle(event, LaunchContext())
        self.assertEqual(len(started), 2)
        for node in started:
            self.assertEqual(node._Node__node_namespace, "/carter1")
        failed = actions[1].event_handler.handle(
            type("Event", (), {"returncode": 1})(), LaunchContext(),
        )
        self.assertIn("[carter1]", failed[0].msg[0].text)


def load_launch(name):
    spec = importlib.util.spec_from_file_location(name.replace(".", "_"), PACKAGE / "launch" / name)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class FleetRvizTest(unittest.TestCase):
    def test_surround_display_is_enabled_for_each_robot(self):
        config = robot_fleet.fleet_rviz(["carter1", "carter2"])
        groups = [d for d in config["Visualization Manager"]["Displays"]
                  if d["Class"] == "rviz_common/Group"]
        for group in groups:
            surround = next(d for d in group["Displays"]
                            if d["Name"] == "Collision surround stop zone")
            self.assertTrue(surround["Enabled"])
            self.assertEqual(surround["Class"], "rviz_default_plugins/Polygon")
            self.assertEqual(surround["Topic"]["Value"],
                             f"/fleet/{group['Name']}/collision_monitor/polygon_surround")

    def topics(self, value):
        if isinstance(value, dict):
            found = [value["Value"]] if isinstance(value.get("Value"), str) and \
                value["Value"].startswith("/") else []
            return found + [t for item in value.values() for t in self.topics(item)]
        if isinstance(value, list):
            return [t for item in value for t in self.topics(item)]
        return []

    def test_one_group_per_robot_and_shared_tools_with_selector(self):
        config = robot_fleet.fleet_rviz(["carter1", "carter2"])
        manager = config["Visualization Manager"]
        groups = [d for d in manager["Displays"] if d["Class"] == "rviz_common/Group"]
        self.assertEqual([g["Name"] for g in groups], ["carter1", "carter2"])
        tool_topics = [t["Topic"]["Value"] for t in manager["Tools"] if "Topic" in t]
        self.assertEqual(tool_topics, [
            "/carter1/initialpose", "/carter1/goal_pose",
        ])
        panel = next(p for p in config["Panels"]
                     if p["Class"] == "slam_localization_3d/FleetPanel")
        self.assertEqual(panel["Robots"], ["carter1", "carter2"])
        self.assertEqual(panel["Selected Robot"], "carter1")
        self.assertEqual(manager["Global Options"]["Fixed Frame"], "map")

    def test_robot_frame_topics_come_from_the_relay(self):
        topics = self.topics(robot_fleet.fleet_rviz(["carter2"]))
        self.assertIn("/fleet/carter2/scan", topics)
        self.assertIn("/carter2/odometry/global", topics)
        self.assertNotIn("/carter2/scan", topics)
        self.assertTrue(all(t.startswith(("/carter2/", "/fleet/carter2/")) for t in topics))

    def test_launch_accepts_names_or_specs(self):
        launch = load_launch("fleet_rviz.launch.py")
        self.assertEqual(launch.robot_names("carter1;b:nova_carter@3,0,1"), ["carter1", "b"])
        self.assertEqual(launch.robot_names(" a ;b:nova_carter"), ["a", "b"])
        for text in ("", "a;A", "1x"):
            with self.subTest(text=text), self.assertRaises(ValueError):
                launch.robot_names(text)


if __name__ == "__main__":
    unittest.main()
