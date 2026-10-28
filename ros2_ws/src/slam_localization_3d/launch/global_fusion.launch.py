import os
import sys
import tempfile

import yaml

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, EmitEvent, LogInfo, OpaqueFunction, RegisterEventHandler,
    SetLaunchConfiguration,
)
from launch.event_handlers import OnProcessExit, OnShutdown
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from robot_fleet import (  # noqa: E402
    DEFAULT_ROBOT_TYPE, UPSTREAM_TOPICS, imu_mount, load_robot_profile, local_odometry_inputs,
    local_odometry_three_d, namespaced_topic, normalize_namespace, with_global_ekf_inputs,
    with_three_d_costmaps,
)
from nav_geometry import rotation_radius, safety_geometry
from robot_namespace import (  # noqa: E402
    RobotParameterFile, load_parameters, local_odometry_file, namespaced_rviz_file,
    robot_parameter_file,
)

sys.path.insert(0, os.path.join(
    get_package_prefix("slam_localization_3d"), "lib", "slam_localization_3d"))
from floor_maps import fleet_mask_topics  # noqa: E402


def robot_namespace(context):
    return normalize_namespace(LaunchConfiguration("namespace", default="").perform(context))


def robot_type(context):
    return LaunchConfiguration("robot_type", default=DEFAULT_ROBOT_TYPE).perform(context)


def push_robot_namespace(context):
    """Run this robot under /<namespace> with its own /<namespace>/tf tree."""
    namespace = robot_namespace(context)
    load_robot_profile(robot_type(context))
    if not namespace:
        return []
    # Copy rather than append so an enclosing scope's remap list is not mutated.
    context.launch_configurations["ros_remaps"] = [
        *context.launch_configurations.get("ros_remaps", []),
        ("/tf", f"/{namespace}/tf"), ("/tf_static", f"/{namespace}/tf_static"),
    ]
    return [PushRosNamespace(namespace)]


def sensor_transforms(context):
    frames = load_robot_profile(robot_type(context))["sensor_frames"]
    return [
        Node(
            package="tf2_ros", executable="static_transform_publisher",
            name=f"base_to_{frame.removesuffix('_link')}_tf",
            arguments=[
                *(item for key in ("x", "y", "z", "roll", "pitch", "yaw")
                  for item in (f"--{key}", str(float(frames[frame][key])))),
                "--frame-id", "base_link", "--child-frame-id", frame,
            ],
            parameters=[{"use_sim_time": True}],
        )
        for frame in ("imu_link", "lidar_link")
    ]


def configure_filters(context, observation_config):
    def flag(name):
        return LaunchConfiguration(name, default="false").perform(context).lower() == "true"

    editor, fleet = flag("filter_editor"), flag("fleet_filters")
    if not editor and not fleet:
        return [SetLaunchConfiguration("costmap_config", observation_config)]
    if editor and fleet:
        raise ValueError("filter_editor and fleet_filters are mutually exclusive")
    if not flag("costmaps"):
        raise ValueError("Costmap filter editor requires costmaps:=true")
    namespace = robot_namespace(context)
    if fleet:
        if not flag("navigate"):
            # floor_manager points the filters at the current floor's fleet masks.
            raise ValueError("fleet_filters requires navigate:=true")
        initial_floor = os.path.basename(os.path.dirname(
            LaunchConfiguration("map_pgm").perform(context)))
        mask_topics = fleet_mask_topics(initial_floor)
    else:
        mask_topics = [
            namespaced_topic(namespace, f"/costmap_filters/{kind}_mask")
            for kind in ("keepout", "speed")
        ]
    config = load_parameters(observation_config, robot_type(context))
    for name in ("global_costmap", "local_costmap"):
        parameters = config[name][name]["ros__parameters"]
        parameters["filters"] = []
        for kind in ("keepout", "speed"):
            if kind == "speed" and name == "global_costmap":
                continue
            plugin = f"{kind}_filter"
            parameters["filters"].append(plugin)
            parameters[plugin] = {
                "plugin": f"nav2_costmap_2d::{'KeepoutFilter' if kind == 'keepout' else 'SpeedFilter'}",
                "enabled": True,
                "filter_info_topic": f"/costmap_filters/{kind}_info",
            }
            if kind == "speed":
                parameters[plugin]["speed_limit_topic"] = "/speed_limit"
            else:
                parameters["filters"].append("keepout_inflation")
                parameters["keepout_inflation"] = {
                    "plugin": "nav2_costmap_2d::InflationLayer",
                    "inflation_radius": parameters["inflation_layer"]["inflation_radius"],
                    "cost_scaling_factor": parameters["inflation_layer"]["cost_scaling_factor"],
                }
    directory = tempfile.TemporaryDirectory(prefix="isaac_costmap_filters_")
    filename = os.path.join(directory.name, "costmaps.yaml")
    with open(filename, "w", encoding="utf-8") as stream:
        yaml.safe_dump(config, stream)

    def cleanup(event, context):
        directory.cleanup()
        return []

    actions = [
        RegisterEventHandler(OnShutdown(on_shutdown=cleanup)),
        SetLaunchConfiguration("costmap_config", filename),
        SetLaunchConfiguration(
            "filter_mask_topics", yaml.safe_dump(mask_topics, default_flow_style=True).strip(),
        ),
    ]
    if fleet:
        return actions + [LogInfo(msg=(
            f"Costmap filters follow the fleet zone editor of the current floor "
            f"(initially {', '.join(mask_topics)})"))]
    editor_node = Node(
        package="slam_localization_3d", executable="costmap_filter_editor.py",
        name="costmap_filter_editor", output="screen",
        parameters=[{
            "use_sim_time": True,
            "state_file": LaunchConfiguration("filter_state"),
        }],
    )
    return actions + [
        editor_node,
        RegisterEventHandler(OnProcessExit(
            target_action=editor_node,
            on_exit=lambda event, context: [] if context.is_shutdown else [
                LogInfo(msg="ERROR: Costmap filter editor exited; shutting down navigation"),
                EmitEvent(event=Shutdown(reason="Costmap filter editor exited")),
            ],
        )),
    ]


def configure_direction_zones(context, collision_config, navigation_config, direction_config):
    """Generate body-derived zones for both direction switching and static mode."""
    kind = robot_type(context)
    collision = load_parameters(collision_config, kind)
    settings = load_parameters(direction_config, kind)["direction_zones"]["ros__parameters"]
    costmaps = load_parameters(LaunchConfiguration("costmap_config").perform(context), kind)
    navigation = load_parameters(navigation_config, kind)
    smoother = navigation["velocity_smoother"]["ros__parameters"]
    speed = max(abs(float(smoother[key][0])) for key in ("max_velocity", "min_velocity"))
    monitor = collision["collision_monitor"]["ros__parameters"]
    geometry = safety_geometry(costmaps, speed, settings, float(monitor["PolygonSlow"]["slowdown_ratio"]))
    switching = (LaunchConfiguration("direction_zones").perform(context).lower() == "true"
                 and LaunchConfiguration("navigate").perform(context).lower() == "true")
    if switching:
        monitor["cmd_vel_in_topic"] = "/nav2/cmd_vel_direction"
    for name, key in (("PolygonSurroundForward", "forward"), ("PolygonSurroundReverse", "reverse"),
                      ("PolygonSlow", "slow")):
        monitor[name]["points"] = geometry[key]
    monitor["PolygonRotate"]["radius"] = geometry["radius"]
    monitor["PolygonSurroundReverse"]["enabled"] = not switching
    monitor["PolygonRotate"]["enabled"] = not switching
    directory = tempfile.TemporaryDirectory(prefix="isaac_direction_zones_")
    path = os.path.join(directory.name, "collision.yaml")
    with open(path, "w", encoding="utf-8") as stream:
        yaml.safe_dump(collision, stream)

    def cleanup(event, context):
        directory.cleanup()
        return []

    actions = [
        RegisterEventHandler(OnShutdown(on_shutdown=cleanup)),
        SetLaunchConfiguration("collision_config", path),
        SetLaunchConfiguration("navigation_config", navigation_config),
        LogInfo(msg=(f"[{kind}] stop extension {geometry['stop_distance']:.2f} m; "
                     f"slow expansion {geometry['slow_padding']:.2f} m; "
                     f"rotation radius {geometry['radius']:.3f} m (base_link)")),
    ]
    if not switching:
        return actions
    selector = Node(
        package="slam_localization_3d", executable="direction_zones.py",
        name="direction_zones", output="screen",
        parameters=[RobotParameterFile(
            direction_config, LaunchConfiguration("namespace"), LaunchConfiguration("robot_type"),
        ), {
            "use_sim_time": True,
            "base_frame_id": monitor["base_frame_id"],
            **{name + ".polygon_pub_topic": namespaced_topic(
                robot_namespace(context), monitor[name]["polygon_pub_topic"],
            ) for name in ("PolygonSurroundForward", "PolygonSurroundReverse", "PolygonRotate")},
        }],
    )
    return actions + [
        selector,
        RegisterEventHandler(OnProcessExit(
            target_action=selector,
            on_exit=lambda event, context: [] if context.is_shutdown else [
                # Commands stop flowing; cmd_vel_safety's command timeout stops the robot.
                LogInfo(msg="ERROR: Direction zones exited; navigation stopped, restart required"),
            ],
        )),
    ]


def configure_surround(context, collision_config, navigation_config, direction_config):
    return configure_direction_zones(context, collision_config, navigation_config, direction_config)


def robot_nodes(context, package, nav):
    """All per-robot nodes, with namespace-dependent names resolved to strings."""
    namespace = robot_namespace(context)
    kind = robot_type(context)

    def value(name):
        return LaunchConfiguration(name).perform(context)

    def enabled(name):
        return value(name).lower() == "true"

    def topic(name):
        return namespaced_topic(namespace, name)

    def config(path):
        return robot_parameter_file(path, namespace, kind)

    sim = {"use_sim_time": True}
    obstacle_cloud = enabled("obstacle_cloud")
    costmaps = enabled("costmaps")
    navigate = enabled("navigate")
    profile = load_robot_profile(kind)
    if costmaps:
        rotation_radius(load_parameters(value("costmap_config"), kind))
    inputs = local_odometry_inputs(profile, value("local_odometry_inputs"))
    three_d = local_odometry_three_d(profile, inputs, value("local_odometry_3d"))
    nav_parameters = [local_odometry_file(
        os.path.join(nav, "config", "local_odometry.yaml"),
        os.path.join(nav, "config", "local_ekf_inputs.yaml"), namespace, kind, inputs, three_d,
    ), sim]
    observation_config = robot_parameter_file(
        value("costmap_config"), namespace, kind,
        transform=with_three_d_costmaps if three_d else None,
    )
    navigation_config = config(value("navigation_config"))
    collision_config = config(value("collision_config"))
    fusion_config = robot_parameter_file(
        os.path.join(package, "config", "global_fusion.yaml"), namespace, kind,
        transform=lambda fusion: with_global_ekf_inputs(fusion, inputs),
    )
    upstream = [(name, topic(name)) for name in UPSTREAM_TOPICS] if namespace else []
    actions = [
        *sensor_transforms(context),
        LogInfo(msg=f"local_ekf inputs ({namespace or 'root'}): {', '.join(inputs)}"
                    f"; odom -> base_link {'3D' if three_d else '2D'}"),
        # The global EKF also fuses the IMU, so its adapter always runs.
        Node(
            package="slam_nav", executable="imu_covariance_adapter",
            name="nav_imu_adapter", output="screen", parameters=nav_parameters,
        ),
    ]
    if "wheel" in inputs:
        actions.append(Node(
            package="slam_nav", executable="wheel_encoder_odometry",
            name="wheel_encoder_odometry", output="screen", parameters=nav_parameters,
        ))
    if "lio" in inputs:
        actions.append(Node(
            package="slam_localization_3d", executable="lio_odometry.py",
            name="lio_odometry", output="screen",
            # local_odometry.yaml: lio/odom; global_fusion.yaml: lio/twist variances.
            parameters=[*nav_parameters, fusion_config, {"imu_mount": imu_mount(profile)}],
            remappings=upstream,
        ))
    actions += [
        Node(
            package="robot_localization", executable="ekf_node",
            name="local_ekf", output="screen",
            parameters=[*nav_parameters, {"reset_on_time_jump": True}],
            remappings=[
                ("odometry/filtered", topic("/odometry/local")),
                ("set_pose", topic("/local_ekf/set_pose")),
            ],
        ),
        Node(
            package="pointcloud_to_laserscan",
            executable="pointcloud_to_laserscan_node",
            name="pointcloud_to_laserscan",
            output="screen",
            remappings=[
                ("cloud_in", topic(
                    "/perception/self_filtered_points" if obstacle_cloud
                    else "/isaac/lidar_points"
                )),
                ("scan", topic("/scan")),
            ],
            parameters=[*nav_parameters, {
                "range_min": 0.0 if obstacle_cloud else 0.5,
                "range_max": 20.0 if obstacle_cloud else 30.0,
            }],
        ),
    ]
    if obstacle_cloud:
        actions.append(Node(
            package="slam_nav", executable="ground_obstacle_filter",
            name="ground_obstacle_filter", output="screen",
            parameters=[config(os.path.join(nav, "config", "ground_obstacle_filter.yaml")), sim],
        ))
    actions += [
        Node(
            package="isaac_fastlio_adapter", executable="pointcloud2_to_livox",
            output="screen", parameters=[sim, {
                "input_topic": topic("/isaac/lidar_points"),
                "output_topic": topic("/livox/lidar"),
            }],
        ),
        Node(
            package="fast_lio_localization", executable="fastlio_mapping",
            name="localization_fastlio", output="screen",
            parameters=[
                config(os.path.join(package, "config", "fast_lio_localization_3d.yaml")), sim
            ],
            remappings=upstream,
        ),
        Node(
            package="slam_localization_3d", executable="global_localization_xyz.py",
            name="global_localization", output="screen",
            parameters=[{
                "use_sim_time": True,
                "pcd_map_path": value("map_pcd"),
                "imu_mount": imu_mount(profile),
                "map_voxel_size": 0.4,
                "scan_voxel_size": 0.1,
                "freq_localization": 0.5,
                "localization_threshold": 0.8,
                "fov": 6.28319,
                "fov_far": 30,
            }],
            remappings=upstream,
        ),
        Node(
            package="slam_localization_3d", executable="global_pose_adapter.py",
            name="global_pose_adapter", output="screen",
            parameters=[fusion_config, {
                "use_sim_time": True, "auto_initial_pose": enabled("auto_initial_pose"),
                "imu_mount": imu_mount(profile),
                **{f"initial_{axis}": float(value(f"initial_{axis}"))
                   for axis in ("x", "y", "z", "yaw")},
            }],
        ),
        Node(
            package="robot_localization", executable="ekf_node",
            name="global_ekf", output="screen",
            parameters=[fusion_config, sim],
            remappings=[
                ("odometry/filtered", topic("/odometry/global")),
                ("set_pose", topic("/global_ekf/set_pose")),
            ],
        ),
        Node(
            package="slam_localization_3d", executable="global_tf_gate.py",
            name="global_tf_gate", output="screen", parameters=[sim],
        ),
        Node(
            package="nav2_map_server", executable="map_server",
            name="map_server", output="screen",
            parameters=[{"yaml_filename": value("map_pgm"), "use_sim_time": True}],
        ),
        Node(
            package="nav2_lifecycle_manager", executable="lifecycle_manager",
            name="lifecycle_manager_fusion_map", output="screen",
            parameters=[{
                "use_sim_time": True, "autostart": True, "node_names": ["map_server"]
            }],
        ),
    ]
    if costmaps and not navigate:
        actions += [
            Node(
                package="slam_localization_3d", executable="costmap_observer",
                namespace=name, name=name, output="screen",
                parameters=[observation_config, sim],
            )
            for name in ("global_costmap", "local_costmap")
        ]
    if navigate:
        maps_root = LaunchConfiguration("maps_root", default="").perform(context)
        initial_floor = os.path.basename(os.path.dirname(value("map_pgm")))
        actions.append(Node(
            package="slam_localization_3d", executable="floor_manager.py",
            name="floor_manager", output="screen",
            parameters=[sim, {
                "maps_root": maps_root or os.path.dirname(os.path.dirname(value("map_pgm"))),
                "initial_floor": initial_floor,
                "filter_editor_active": enabled("filter_editor"),
                "fleet_filters": enabled("fleet_filters"),
                "floor_base_z": -imu_mount(profile)[2],
            }],
        ))
        mqtt_host = value("floor_mqtt_host")
        if mqtt_host:
            actions.append(Node(
                package="slam_fleet_bridge", executable="floor_command_bridge.py",
                name="floor_command_bridge", output="screen",
                parameters=[sim, {
                    "mqtt_host": mqtt_host,
                    "mqtt_port": int(value("floor_mqtt_port")),
                    "fleet_name": value("floor_mqtt_fleet"),
                    "robot_name": value("floor_mqtt_robot"),
                    "floor_base_z": -imu_mount(profile)[2],
                }],
            ))
        # Nav2 publishes relative cmd_vel; recovery motions go through the same
        # smoother, collision_monitor and cmd_vel_safety gates as the controller.
        nav_cmd = [(topic("/cmd_vel"), topic("/nav2/cmd_vel_nav"))]
        actions += [
            Node(
                package="nav2_planner", executable="planner_server",
                name="planner_server", output="screen",
                parameters=[navigation_config, observation_config, sim],
            ),
            Node(
                package="nav2_controller", executable="controller_server",
                name="controller_server", output="screen",
                parameters=[navigation_config, observation_config, sim],
                remappings=nav_cmd,
            ),
            Node(
                package="nav2_behaviors", executable="behavior_server",
                name="behavior_server", output="screen",
                parameters=[navigation_config, sim],
                remappings=nav_cmd,
            ),
            Node(
                package="nav2_bt_navigator", executable="bt_navigator",
                name="bt_navigator", output="screen",
                parameters=[
                    navigation_config,
                    {"default_nav_to_pose_bt_xml": os.path.join(
                        package, "config", "navigate_to_pose.xml"
                    ), "default_nav_through_poses_bt_xml": os.path.join(
                        package, "config", "navigate_through_poses.xml"
                    )}, sim,
                ],
            ),
            Node(
                package="nav2_velocity_smoother", executable="velocity_smoother",
                name="velocity_smoother", output="screen",
                parameters=[navigation_config, sim],
                # Smooth before the safety gates so their stops stay immediate.
                remappings=[
                    ("cmd_vel", topic("/nav2/cmd_vel_nav")),
                    ("cmd_vel_smoothed", topic("/nav2/cmd_vel")),
                ],
            ),
            Node(
                package="nav2_collision_monitor", executable="collision_monitor",
                name="collision_monitor", output="screen",
                parameters=[collision_config, sim],
            ),
            Node(
                package="slam_localization_3d", executable="cmd_vel_safety.py",
                name="cmd_vel_safety", output="screen", parameters=[collision_config, sim],
                remappings=[(topic("/nav2/cmd_vel"), topic("/nav2/cmd_vel_monitored"))],
            ),
        ]
    if costmaps:
        actions += readiness_actions(namespace, navigate, value("filter_mask_topics"))
    if enabled("rviz"):
        actions.append(Node(
            package="rviz2", executable="rviz2", output="screen",
            arguments=["-d", namespaced_rviz_file(
                os.path.join(package, "config", "global_fusion.rviz"), namespace,
            )],
            parameters=[sim],
        ))
    return actions


def readiness_actions(namespace, navigate, filter_mask_topics):
    # Event handlers may run outside this robot's launch scope (fleet group) or
    # inside its pushed namespace (robot.launch.py), so their nodes get an
    # absolute namespace and need no TF.
    label = namespace
    namespace = f"/{namespace}" if namespace else namespace
    readiness = Node(
        package="slam_localization_3d", executable="wait_for_costmap_tf.py",
        output="screen", parameters=[{
            "use_sim_time": True, "filter_mask_topics": yaml.safe_load(filter_mask_topics),
        }],
    )
    if navigate:
        manager = Node(
            package="nav2_lifecycle_manager", executable="lifecycle_manager",
            namespace=namespace, name="lifecycle_manager_navigation", output="screen",
            parameters=[{
                "use_sim_time": True, "autostart": True, "bond_timeout": 10.0,
                "node_names": [
                    "planner_server", "controller_server", "behavior_server", "bt_navigator",
                    "velocity_smoother", "collision_monitor",
                ],
            }],
        )
        navigation_readiness = Node(
            package="slam_localization_3d", executable="wait_for_navigation.py",
            namespace=namespace, output="screen", parameters=[{"use_sim_time": True}],
        )
        started = [manager, navigation_readiness]
    else:
        started = [Node(
            package="nav2_lifecycle_manager", executable="lifecycle_manager",
            namespace=namespace, name="lifecycle_manager_fusion_costmaps", output="screen",
            # Standalone Costmap2DROS activates without creating a Nav2 lifecycle bond.
            parameters=[{
                "use_sim_time": True, "autostart": True,
                "bond_timeout": 0.0,
                "node_names": [
                    "local_costmap/local_costmap",
                    "global_costmap/global_costmap",
                ],
            }],
        )]
    label = f" [{label}]" if label else ""
    actions = [
        readiness,
        RegisterEventHandler(OnProcessExit(
            target_action=readiness,
            on_exit=lambda event, context: started if event.returncode == 0 else [
                LogInfo(msg=f"ERROR{label}: Costmaps not started: map or localization TF unavailable"),
                EmitEvent(event=Shutdown(reason="Costmap readiness failed")),
            ],
        )),
    ]
    if navigate:
        actions.append(RegisterEventHandler(OnProcessExit(
            target_action=navigation_readiness,
            on_exit=lambda event, _: [] if event.returncode == 0 else [
                LogInfo(msg=f"ERROR{label}: Nav2 navigation stack failed to activate"),
                EmitEvent(event=Shutdown(reason="Navigation activation failed")),
            ],
        )))
    return actions


def generate_launch_description():
    package = get_package_share_directory("slam_localization_3d")
    nav = get_package_share_directory("slam_nav")
    return LaunchDescription(
        [
            DeclareLaunchArgument("map_pcd", default_value=os.path.join(
                os.getcwd(), "maps/1F/map.pcd"), description="Absolute PCD map path"),
            DeclareLaunchArgument("map_pgm", default_value=os.path.join(
                os.getcwd(), "maps/1F/map_2d.yaml"), description="Absolute Nav2 map YAML path"),
            DeclareLaunchArgument(
                "maps_root", default_value="",
                description="Floor map directory; default is the parent of the initial map folder",
            ),
            DeclareLaunchArgument(
                "namespace", default_value="",
                description="Robot namespace; also isolates TF on /<namespace>/tf",
            ),
            DeclareLaunchArgument(
                "robot_type", default_value=DEFAULT_ROBOT_TYPE,
                description="Profile in config/robots/<robot_type>.yaml",
            ),
            *(DeclareLaunchArgument(
                f"initial_{axis}", default_value="0.0",
                description="auto_initial_pose map -> camera_init guess; "
                            "robot.launch.py derives it from the spawn",
            ) for axis in ("x", "y", "z", "yaw")),
            DeclareLaunchArgument(
                "local_odometry_inputs", default_value="",
                description="Comma-separated local_ekf inputs (wheel, imu, lio) overriding "
                            "the robot profile's local_odometry.inputs",
            ),
            DeclareLaunchArgument(
                "local_odometry_3d", default_value="",
                description="true/false overriding the robot profile's local_odometry.three_d "
                            "(3D odom -> base_link with z/roll/pitch; requires the lio input)",
            ),
            DeclareLaunchArgument("rviz", default_value="true"),
            DeclareLaunchArgument("auto_initial_pose", default_value="false"),
            DeclareLaunchArgument("obstacle_cloud", default_value="false"),
            DeclareLaunchArgument("costmaps", default_value="false"),
            DeclareLaunchArgument("navigate", default_value="false"),
            DeclareLaunchArgument(
                "floor_mqtt_host", default_value="",
                description="MQTT broker for standalone change_floor commands (navigate only; "
                            "empty disables)",
            ),
            DeclareLaunchArgument("floor_mqtt_port", default_value="1883"),
            DeclareLaunchArgument("floor_mqtt_fleet", default_value="manual"),
            DeclareLaunchArgument("floor_mqtt_robot", default_value="carter"),
            DeclareLaunchArgument(
                "direction_zones", default_value="true",
                description="Switch forward/reverse/rotate collision zones by command direction "
                            "(navigate only)",
            ),
            DeclareLaunchArgument("filter_editor", default_value="false",
                                  description="Enable live RViz polygon annotation"),
            DeclareLaunchArgument(
                "filter_state",
                default_value=os.path.join(os.getcwd(), "maps/costmap_filters/editor.json"),
                description="Persistent JSON zone state for the RViz editor",
            ),
            DeclareLaunchArgument(
                "fleet_filters", default_value="false",
                description="Apply Keepout/Speed zones served per floor by fleet_rviz.launch.py "
                            "filter_editor:=true (/fleet/zones/floor_FLOOR); requires navigate",
            ),
            OpaqueFunction(function=push_robot_namespace),
            SetLaunchConfiguration("filter_mask_topics", '[""]'),
            OpaqueFunction(
                function=configure_filters,
                args=[os.path.join(package, "config", "observation_costmaps.yaml")],
            ),
            OpaqueFunction(
                function=configure_surround,
                args=[os.path.join(package, "config", filename) for filename in (
                    "collision_monitor.yaml", "navigation.yaml", "direction_zones.yaml",
                )],
            ),
            OpaqueFunction(function=robot_nodes, args=[package, nav]),
        ]
    )
