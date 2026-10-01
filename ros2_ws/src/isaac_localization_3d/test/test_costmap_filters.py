import importlib.util
import json
import signal
import subprocess
import sys
import tempfile
import time
import unittest
from unittest.mock import Mock
from pathlib import Path

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PointStamped, PoseStamped, TransformStamped, Twist
from launch import LaunchContext, LaunchDescription, LaunchService
from launch.actions import DeclareLaunchArgument, OpaqueFunction, RegisterEventHandler, SetLaunchConfiguration
from launch.events import Shutdown
from launch_ros.actions import Node
from nav2_msgs.action import ComputePathToPose, FollowPath
from nav2_msgs.msg import SpeedLimit
from nav_msgs.msg import OccupancyGrid, Odometry, Path as NavPath
from rclpy.action import ActionClient
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from tf2_ros import TransformBroadcaster
from visualization_msgs.msg import InteractiveMarkerFeedback
from visualization_msgs.srv import GetInteractiveMarkers


PACKAGE = Path(get_package_share_directory("isaac_localization_3d"))
SPEC = importlib.util.spec_from_file_location(
    "fusion_launch", PACKAGE / "launch/global_fusion.launch.py",
)
FUSION = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(FUSION)


def write_map(directory, name, cells):
    image = directory / f"{name}.pgm"
    # ROS grids start at the bottom left; PGM rows start at the top.
    image.write_bytes(b"P5\n60 60\n255\n" + bytes(
        255 - round(cells(x, y) * 255 / 100)
        for y in reversed(range(60)) for x in range(60)
    ))
    filename = directory / f"{name}.yaml"
    filename.write_text(yaml.safe_dump({
        "image": image.name, "mode": "scale", "resolution": 0.1,
        "origin": [0.0, 0.0, 0.0], "negate": 0,
        "occupied_thresh": 1.0, "free_thresh": 0.0,
    }))
    return str(filename)


def filter_context(editor=False):
    context = LaunchContext()
    context.launch_configurations.update(
        filter_editor="true" if editor else "false", costmaps="true",
    )
    return context


def configure(context):
    actions = FUSION.configure_filters(
        context, str(PACKAGE / "config/observation_costmaps.yaml"),
    )
    for action in actions:
        if isinstance(action, SetLaunchConfiguration):
            action.execute(context)
    return actions


def serve(directory):
    def setup(context):
        context.launch_configurations.update(
            costmaps="true", filter_editor="true",
            filter_state=str(directory / "zones.json"),
        )
        actions = configure(context)
        filename = context.launch_configurations["costmap_config"]
        config = yaml.safe_load(Path(filename).read_text())
        for name in ("global_costmap", "local_costmap"):
            params = config[name][name]["ros__parameters"]
            params.update(
                use_sim_time=False, global_frame="map",
                rolling_window=name == "local_costmap",
                width=6, height=6, origin_x=0.0, origin_y=0.0, resolution=0.1,
                update_frequency=10.0, publish_frequency=10.0,
                plugins=["static_layer"],
            )
            params["static_layer"] = {
                "plugin": "nav2_costmap_2d::StaticLayer", "map_topic": "/map",
                "map_subscribe_transient_local": True,
            }
        Path(filename).write_text(yaml.safe_dump(config))
        actions.append(Node(
            package="nav2_map_server", executable="map_server", name="map_server",
            parameters=[{"yaml_filename": str(directory / "map.yaml"), "use_sim_time": False}],
        ))
        navigation = str(PACKAGE / "config/navigation.yaml")
        actions.extend([
            Node(
                package="nav2_planner", executable="planner_server", name="planner_server",
                parameters=[navigation, filename, {"use_sim_time": False}],
                output="screen",
            ),
            Node(
                package="nav2_controller", executable="controller_server",
                name="controller_server", output="screen",
                parameters=[navigation, filename, {"use_sim_time": False}],
                remappings=[("/cmd_vel", "/test/filter_cmd")],
            ),
            Node(
                package="nav2_lifecycle_manager", executable="lifecycle_manager",
                name="test_navigation_manager", output="screen",
                parameters=[{"autostart": True, "use_sim_time": False,
                             "node_names": ["map_server", "planner_server", "controller_server"]}],
            ),
        ])
        return actions

    service = LaunchService()
    service.include_launch_description(LaunchDescription([OpaqueFunction(function=setup)]))
    return service.run()


class FilterConfigurationTest(unittest.TestCase):
    def test_editor_state_default_uses_map_directory(self):
        argument = next(
            action for action in FUSION.generate_launch_description().entities
            if isinstance(action, DeclareLaunchArgument) and action.name == "filter_state"
        )
        context = LaunchContext()
        argument.execute(context)
        self.assertEqual(
            context.launch_configurations["filter_state"],
            str(Path.cwd() / "maps/costmap_filters/editor.json"),
        )

    def test_rviz_connects_to_editor_namespace(self):
        config = yaml.safe_load((PACKAGE / "config/global_fusion.rviz").read_text())
        display = next(
            item for item in config["Visualization Manager"]["Displays"]
            if item["Class"] == "rviz_default_plugins/InteractiveMarkers"
        )
        self.assertTrue(display["Enabled"])
        self.assertEqual(display["Interactive Markers Namespace"], "/costmap_filter_editor")
        self.assertNotIn("Update Topic", display)

    def test_editor_enables_both_filters_without_static_servers(self):
        context = filter_context()
        context.launch_configurations.update(filter_editor="true", filter_state="/tmp/zones.json")
        actions = configure(context)
        config = yaml.safe_load(Path(context.launch_configurations["costmap_config"]).read_text())
        self.assertIn("keepout_filter", config["global_costmap"]["global_costmap"]["ros__parameters"]["filters"])
        self.assertIn("speed_filter", config["local_costmap"]["local_costmap"]["ros__parameters"]["filters"])
        for action in actions:
            if isinstance(action, RegisterEventHandler):
                event = Shutdown()
                if action.event_handler.matches(event):
                    for cleanup in action.event_handler.handle(event, context):
                        cleanup.execute(context)

    def test_readiness_waits_for_every_selected_mask(self):
        spec = importlib.util.spec_from_file_location(
            "filter_readiness",
            PACKAGE.parent.parent / "lib/isaac_localization_3d/wait_for_costmap_tf.py",
        )
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        rclpy.init(args=[
            "--ros-args", "-p",
            "filter_mask_topics:=['/costmap_filters/keepout_mask', '/costmap_filters/speed_mask']",
        ])
        node = module.CostmapReadiness()
        try:
            message = OccupancyGrid()
            message.info.width = message.info.height = 1
            message.header.frame_id = "map"
            node.on_map(message)
            transform = TransformStamped()
            transform.header.stamp = node.get_clock().now().to_msg()
            node.buffer.lookup_transform = Mock(return_value=transform)
            self.assertFalse(node.ready())
            node.on_mask("/costmap_filters/keepout_mask", message)
            self.assertFalse(node.ready())
            message.header.frame_id = "odom"
            node.on_mask("/costmap_filters/speed_mask", message)
            self.assertFalse(node.ready())
            message.header.frame_id = "map"
            node.on_mask("/costmap_filters/speed_mask", message)
            self.assertTrue(node.ready())
        finally:
            node.destroy_node()
            rclpy.shutdown()

    def test_disabled_editor_preserves_original_config(self):
        context = filter_context()
        configure(context)
        self.assertEqual(context.launch_configurations["costmap_config"],
                         str(PACKAGE / "config/observation_costmaps.yaml"))

    def test_editor_requires_costmaps(self):
        context = filter_context(editor=True)
        context.launch_configurations["costmaps"] = "false"
        with self.assertRaisesRegex(ValueError, "costmaps"):
            configure(context)


class CostmapFilterIntegrationTest(unittest.TestCase):
    def test_live_rviz_feedback_changes_filters_without_restart(self):
        rclpy.init()
        node = rclpy.create_node("costmap_filter_test")
        directory = tempfile.TemporaryDirectory()
        log = tempfile.TemporaryFile(mode="w+")
        process = None
        try:
            root = Path(directory.name)
            write_map(root, "map", lambda x, y: 0)
            process = subprocess.Popen(
                [sys.executable, __file__, "--serve", str(root)],
                stdout=log, stderr=log,
            )
            maps, limits, commands = {}, [], []
            qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
            for name in ("global_costmap", "local_costmap"):
                node.create_subscription(
                    OccupancyGrid, f"/{name}/costmap",
                    lambda message, name=name: maps.__setitem__(name, message), qos,
                )
            node.create_subscription(SpeedLimit, "/speed_limit", limits.append, 10)
            node.create_subscription(Twist, "/test/filter_cmd", commands.append, 10)
            broadcaster = TransformBroadcaster(node)
            odometry = node.create_publisher(Odometry, "/odometry/local", 10)
            position = [1.0, 1.0]

            def publish():
                stamp = node.get_clock().now().to_msg()
                transforms = []
                for parent, child, x, y in (
                    ("map", "odom", 0.0, 0.0),
                    ("odom", "base_link", *position),
                ):
                    transform = TransformStamped()
                    transform.header.frame_id = parent
                    transform.header.stamp = stamp
                    transform.child_frame_id = child
                    transform.transform.translation.x = x
                    transform.transform.translation.y = y
                    transform.transform.rotation.w = 1.0
                    transforms.append(transform)
                broadcaster.sendTransform(transforms)
                message = Odometry()
                message.header.frame_id = "odom"
                message.header.stamp = stamp
                message.child_frame_id = "base_link"
                message.pose.pose.position.x, message.pose.pose.position.y = position
                message.pose.pose.orientation.w = 1.0
                odometry.publish(message)

            def wait(predicate, seconds=30):
                deadline = time.monotonic() + seconds
                while time.monotonic() < deadline:
                    publish()
                    rclpy.spin_once(node, timeout_sec=0.03)
                    if predicate():
                        return
                    if process.poll() is not None:
                        break
                log.seek(0)
                self.fail("Filter integration timed out\n" + log.read()[-12000:])

            def cost(message, x, y):
                col = int((x - message.info.origin.position.x) / message.info.resolution)
                row = int((y - message.info.origin.position.y) / message.info.resolution)
                return message.data[row * message.info.width + col]

            points = node.create_publisher(PointStamped, "/clicked_point", 10)
            feedback = node.create_publisher(
                InteractiveMarkerFeedback, "/costmap_filter_editor/feedback", 10,
            )
            markers = node.create_client(
                GetInteractiveMarkers, "/costmap_filter_editor/get_interactive_markers",
            )
            wait(lambda: markers.service_is_ready() and points.get_subscription_count() > 0
                 and feedback.get_subscription_count() > 0 and len(maps) == 2)

            def get_marker(name):
                future = markers.call_async(GetInteractiveMarkers.Request())
                wait(future.done)
                return next((marker for marker in future.result().markers
                             if marker.name == name), None)

            def menu(name, title):
                marker = get_marker(name)
                self.assertIsNotNone(marker)
                entry = next(entry for entry in marker.menu_entries if entry.title == title)
                message = InteractiveMarkerFeedback()
                message.header.frame_id = "map"
                message.marker_name = name
                message.event_type = message.MENU_SELECT
                message.menu_entry_id = entry.id
                message.pose = marker.pose
                feedback.publish(message)

            def polygon(vertices, title, count):
                for index, (x, y) in enumerate(vertices):
                    message = PointStamped()
                    message.header.frame_id = "map"
                    message.point.x, message.point.y = x, y
                    points.publish(message)
                    wait(lambda: (draft := get_marker("draft")) is not None
                         and len(draft.controls[0].markers[0].points) == index + 2)
                menu("draft", title)
                wait(lambda: (root / "zones.json").exists()
                     and len(json.loads((root / "zones.json").read_text())["zones"]) == count)

            polygon([(2.5, 0.5), (3.6, 0.5), (3.6, 1.6), (2.5, 1.6)],
                    "Apply draft: Keepout", 1)
            polygon([(0.5, 0.5), (2.1, 0.5), (2.1, 2.1), (0.5, 2.1)], "50%", 2)
            menu("zone_2", "Edit this zone's vertices")
            wait(lambda: get_marker("vertex_2_0") is not None)
            message = InteractiveMarkerFeedback()
            message.header.frame_id = "map"
            message.marker_name = "vertex_2_0"
            message.event_type = message.MOUSE_UP
            message.pose.position.x, message.pose.position.y = 0.4, 0.5
            message.pose.orientation.w = 1.0
            feedback.publish(message)
            wait(lambda: json.loads((root / "zones.json").read_text())["zones"][1]["points"][0][0] == 0.4)

            wait(lambda: len(maps) == 2 and bool(limits)
                 and all(cost(message, 3.0, 1.0) == 100 for message in maps.values()))
            self.assertGreater(cost(maps["global_costmap"], 2.3, 1.0), 0,
                               "Keepout zone must be inflated for body clearance")
            self.assertTrue(limits[-1].percentage)
            self.assertAlmostEqual(limits[-1].speed_limit, 50.0, delta=1.0)

            def pose(x, y):
                message = PoseStamped()
                message.header.frame_id = "map"
                message.header.stamp = node.get_clock().now().to_msg()
                message.pose.position.x, message.pose.position.y = x, y
                message.pose.orientation.w = 1.0
                return message

            planner = ActionClient(node, ComputePathToPose, "/compute_path_to_pose")
            wait(planner.server_is_ready)
            request = ComputePathToPose.Goal()
            request.start, request.goal = pose(1.0, 1.0), pose(5.0, 1.0)
            request.use_start = True
            request.planner_id = "GridBased"
            future = planner.send_goal_async(request)
            wait(future.done)
            self.assertTrue(future.result().accepted)
            result = future.result().get_result_async()
            wait(result.done)
            self.assertEqual(result.result().status, 4)
            path = result.result().result.path
            self.assertGreater(len(path.poses), 2)
            for point in path.poses:
                self.assertLess(cost(maps["global_costmap"], point.pose.position.x,
                                     point.pose.position.y), 100,
                                f"Path intersects keepout at {point.pose.position}; "
                                f"map origin={maps['global_costmap'].info.origin}")
            self.assertTrue(any(abs(point.pose.position.y - 1.0) > 0.5
                                for point in path.poses), "Planner did not detour")

            menu("zone_1", "Delete this zone")
            wait(lambda: all(cost(message, 3.0, 1.0) == 0 for message in maps.values()))
            future = planner.send_goal_async(request)
            wait(future.done)
            result = future.result().get_result_async()
            wait(result.done)
            self.assertEqual(result.result().status, 4)
            self.assertTrue(all(abs(point.pose.position.y - 1.0) < 0.2
                                for point in result.result().result.path.poses))

            controller = ActionClient(node, FollowPath, "/follow_path")
            wait(controller.server_is_ready)

            def follow(x):
                request = FollowPath.Goal()
                request.controller_id = "FollowPath"
                request.goal_checker_id = "general_goal_checker"
                request.path = NavPath()
                request.path.header.frame_id = "map"
                request.path.poses = [
                    pose(x + i * 0.05, 1.0) for i in range(21)
                ]
                future = controller.send_goal_async(request)
                wait(future.done)
                self.assertTrue(future.result().accepted)
                return future.result()

            handle = follow(1.0)
            wait(lambda: any(command.linear.x > 0.1 for command in commands))
            self.assertLessEqual(max(command.linear.x for command in commands), 0.255)
            cancelled = handle.cancel_goal_async()
            wait(cancelled.done)
            self.assertTrue(cancelled.result().goals_canceling)
            completed = handle.get_result_async()
            wait(completed.done)
            self.assertEqual(completed.result().status, 5)

            menu("zone_2", "Delete this zone")
            position[:] = [4.0, 1.0]
            wait(lambda: limits[-1].speed_limit == 0.0)
            commands.clear()
            handle = follow(4.0)
            wait(lambda: any(command.linear.x > 0.4 for command in commands))
            cancelled = handle.cancel_goal_async()
            wait(cancelled.done)
        finally:
            if process is not None:
                process.send_signal(signal.SIGINT)
                try:
                    process.wait(timeout=15)
                except subprocess.TimeoutExpired:
                    process.kill()
                    process.wait(timeout=5)
            log.close()
            directory.cleanup()
            node.destroy_node()
            rclpy.shutdown()


if __name__ == "__main__":
    if len(sys.argv) == 3 and sys.argv[1] == "--serve":
        sys.exit(serve(Path(sys.argv[2])))
    unittest.main()
