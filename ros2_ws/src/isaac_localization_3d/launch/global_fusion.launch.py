import json
import os
import tempfile

import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, EmitEvent, LogInfo, OpaqueFunction, RegisterEventHandler,
    SetLaunchConfiguration,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit, OnShutdown
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def configure_filters(context, observation_config):
    editor = LaunchConfiguration("filter_editor", default="false").perform(context).lower() == "true"
    if not editor:
        return [SetLaunchConfiguration("costmap_config", observation_config)]
    if LaunchConfiguration("costmaps").perform(context).lower() != "true":
        raise ValueError("Costmap filter editor requires costmaps:=true")
    with open(observation_config, encoding="utf-8") as stream:
        config = yaml.safe_load(stream)
    mask_topics = [f"/costmap_filters/{kind}_mask" for kind in ("keepout", "speed")]
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

    editor_node = Node(
        package="isaac_localization_3d", executable="costmap_filter_editor.py",
        name="costmap_filter_editor", output="screen",
        parameters=[{
            "use_sim_time": True,
            "state_file": LaunchConfiguration("filter_state"),
        }],
    )
    return [
        RegisterEventHandler(OnShutdown(on_shutdown=cleanup)),
        SetLaunchConfiguration("costmap_config", filename),
        SetLaunchConfiguration(
            "filter_mask_topics", yaml.safe_dump(mask_topics, default_flow_style=True).strip(),
        ),
        editor_node,
        RegisterEventHandler(OnProcessExit(
            target_action=editor_node,
            on_exit=lambda event, context: [] if context.is_shutdown else [
                LogInfo(msg="ERROR: Costmap filter editor exited; shutting down navigation"),
                EmitEvent(event=Shutdown(reason="Costmap filter editor exited")),
            ],
        )),
    ]


def configure_surround(context, collision_config, navigation_config, adaptive_config):
    enabled = LaunchConfiguration("adaptive_surround").perform(context).lower() == "true"
    if not enabled:
        return [
            SetLaunchConfiguration("collision_config", collision_config),
            SetLaunchConfiguration("navigation_config", navigation_config),
        ]
    if any(LaunchConfiguration(name).perform(context).lower() != "true"
           for name in ("navigate", "costmaps", "obstacle_cloud")):
        raise ValueError("Adaptive surround requires navigation, costmaps and obstacle cloud")
    with open(collision_config, encoding="utf-8") as stream:
        collision = yaml.safe_load(stream)
    with open(navigation_config, encoding="utf-8") as stream:
        navigation = yaml.safe_load(stream)
    with open(adaptive_config, encoding="utf-8") as stream:
        adaptive = yaml.safe_load(stream)["adaptive_surround"]["ros__parameters"]
    with open(LaunchConfiguration("costmap_config").perform(context), encoding="utf-8") as stream:
        local = yaml.safe_load(stream)["local_costmap"]["local_costmap"]["ros__parameters"]
    monitor = collision["collision_monitor"]["ros__parameters"]
    monitor["cmd_vel_in_topic"] = "/nav2/cmd_vel_adaptive"
    monitor["PolygonSurround"]["visualize"] = False
    monitor["polygons"].append("PolygonSurroundCrawl")
    monitor["PolygonSurroundCrawl"] = dict(
        monitor["PolygonSurround"], points=adaptive["crawl_points"], enabled=False,
        polygon_pub_topic="/collision_monitor/polygon_surround_crawl",
    )
    collision["cmd_vel_safety"]["ros__parameters"]["require_adaptive_limits"] = True
    controller = navigation["controller_server"]["ros__parameters"]["FollowPath"]
    controller["desired_linear_vel"] = adaptive["crawl_linear"]
    controller["rotate_to_heading_angular_vel"] = 0.15
    smoother = navigation["velocity_smoother"]["ros__parameters"]
    smoother["max_velocity"][2] = adaptive["crawl_angular"]
    smoother["min_velocity"][2] = -adaptive["crawl_angular"]
    directory = tempfile.TemporaryDirectory(prefix="isaac_adaptive_surround_")
    paths = {}
    for name, config in (("collision", collision), ("navigation", navigation)):
        filename = os.path.join(directory.name, name + ".yaml")
        with open(filename, "w", encoding="utf-8") as stream:
            yaml.safe_dump(config, stream)
        paths[name] = filename

    def cleanup(event, context):
        directory.cleanup()
        return []

    selector = Node(
        package="isaac_localization_3d", executable="adaptive_surround.py",
        name="adaptive_surround", output="screen",
        parameters=[adaptive_config, {
            "use_sim_time": True, "full_points": monitor["PolygonSurround"]["points"],
            "physical_footprint": [
                float(value) for point in json.loads(local["footprint"]) for value in point
            ],
            "footprint_padding": float(local["footprint_padding"]),
        }],
    )
    return [
        RegisterEventHandler(OnShutdown(on_shutdown=cleanup)),
        SetLaunchConfiguration("collision_config", paths["collision"]),
        SetLaunchConfiguration("navigation_config", paths["navigation"]),
        selector,
        RegisterEventHandler(OnProcessExit(
            target_action=selector,
            on_exit=lambda event, context: [] if context.is_shutdown else [
                # Leave the safety watchdog alive to stop on the expired heartbeat.
                LogInfo(msg="ERROR: Adaptive surround exited; safety gates remain active, restart required"),
            ],
        )),
    ]


def generate_launch_description():
    package = get_package_share_directory("isaac_localization_3d")
    nav = get_package_share_directory("isaac_nav")
    map_pcd = LaunchConfiguration("map_pcd")
    map_pgm = LaunchConfiguration("map_pgm")
    rviz = LaunchConfiguration("rviz")
    auto_initial_pose = LaunchConfiguration("auto_initial_pose")
    obstacle_cloud = LaunchConfiguration("obstacle_cloud")
    costmaps = LaunchConfiguration("costmaps")
    navigate = LaunchConfiguration("navigate")
    sim = {"use_sim_time": True}
    nav_parameters = [os.path.join(nav, "config", "local_odometry.yaml"), sim]
    observation_config = LaunchConfiguration("costmap_config")
    navigation_config = LaunchConfiguration("navigation_config")
    fusion_config = os.path.join(package, "config", "global_fusion.yaml")
    collision_config = LaunchConfiguration("collision_config")
    observing = IfCondition(PythonExpression([
        "'", costmaps, "' == 'true' and '", navigate, "' == 'false'",
    ]))
    readiness = Node(
        condition=IfCondition(costmaps),
        package="isaac_localization_3d", executable="wait_for_costmap_tf.py",
        output="screen", parameters=[sim, {
            "filter_mask_topics": LaunchConfiguration("filter_mask_topics"),
        }],
    )
    costmap_manager = Node(
        package="nav2_lifecycle_manager", executable="lifecycle_manager",
        name="lifecycle_manager_fusion_costmaps", output="screen",
        # Standalone Costmap2DROS activates without creating a Nav2 lifecycle bond.
        parameters=[{
            "use_sim_time": True, "autostart": True,
            "bond_timeout": 0.0,
            "node_names": [
                "local_costmap/local_costmap",
                "global_costmap/global_costmap",
            ],
        }],
    )
    navigation_manager = Node(
        package="nav2_lifecycle_manager", executable="lifecycle_manager",
        name="lifecycle_manager_navigation", output="screen",
        parameters=[{
            "use_sim_time": True, "autostart": True, "bond_timeout": 10.0,
            "node_names": [
                "planner_server", "controller_server", "behavior_server", "bt_navigator",
                "velocity_smoother", "collision_monitor",
            ],
        }],
    )
    navigation_readiness = Node(
        package="isaac_localization_3d", executable="wait_for_navigation.py",
        output="screen", parameters=[sim],
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument("map_pcd", description="Absolute PCD map path"),
            DeclareLaunchArgument("map_pgm", description="Absolute Nav2 map YAML path"),
            DeclareLaunchArgument("rviz", default_value="true"),
            DeclareLaunchArgument("auto_initial_pose", default_value="false"),
            DeclareLaunchArgument("obstacle_cloud", default_value="false"),
            DeclareLaunchArgument("costmaps", default_value="false"),
            DeclareLaunchArgument("navigate", default_value="false"),
            DeclareLaunchArgument("adaptive_surround", default_value="false",
                                  description="Experimental acknowledged low-speed surround profiles"),
            DeclareLaunchArgument("filter_editor", default_value="false",
                                  description="Enable live RViz polygon annotation"),
            DeclareLaunchArgument(
                "filter_state",
                default_value=os.path.join(os.getcwd(), "maps/costmap_filters/editor.json"),
                description="Persistent JSON zone state for the RViz editor",
            ),
            SetLaunchConfiguration("filter_mask_topics", '[""]'),
            OpaqueFunction(
                function=configure_filters,
                args=[os.path.join(package, "config", "observation_costmaps.yaml")],
            ),
            OpaqueFunction(
                function=configure_surround,
                args=[os.path.join(package, "config", filename) for filename in (
                    "collision_monitor.yaml", "navigation.yaml", "adaptive_surround.yaml",
                )],
            ),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="base_to_imu_tf",
                arguments=[
                    "--x", "0.213", "--y", "-0.009", "--z", "0.526",
                    "--roll", "0", "--pitch", "0", "--yaw", "3.141592654",
                    "--frame-id", "base_link", "--child-frame-id", "imu_link",
                ],
                parameters=[sim],
            ),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="base_to_lidar_tf",
                arguments=[
                    "--x", "0.213", "--y", "-0.009", "--z", "0.526",
                    "--roll", "0", "--pitch", "0", "--yaw", "3.141592654",
                    "--frame-id", "base_link", "--child-frame-id", "lidar_link",
                ],
                parameters=[sim],
            ),
            Node(
                package="isaac_nav", executable="wheel_encoder_odometry",
                name="wheel_encoder_odometry", output="screen", parameters=nav_parameters,
            ),
            Node(
                package="isaac_nav", executable="imu_covariance_adapter",
                name="nav_imu_adapter", output="screen", parameters=nav_parameters,
            ),
            Node(
                package="robot_localization", executable="ekf_node",
                name="local_ekf", output="screen",
                parameters=[*nav_parameters, {"reset_on_time_jump": True}],
                remappings=[("odometry/filtered", "/odometry/local")],
            ),
            Node(
                package="pointcloud_to_laserscan",
                executable="pointcloud_to_laserscan_node",
                name="pointcloud_to_laserscan",
                output="screen",
                remappings=[
                    ("cloud_in", PythonExpression([
                        "'/perception/self_filtered_points' if '", obstacle_cloud,
                        "' == 'true' else '/isaac/lidar_points'",
                    ])),
                    ("scan", "/scan"),
                ],
                parameters=[*nav_parameters, {
                    "range_min": PythonExpression([
                        "0.0 if '", obstacle_cloud, "' == 'true' else 0.5",
                    ]),
                    "range_max": PythonExpression([
                        "20.0 if '", obstacle_cloud, "' == 'true' else 30.0",
                    ]),
                }],
            ),
            Node(
                condition=IfCondition(obstacle_cloud),
                package="isaac_nav",
                executable="ground_obstacle_filter",
                name="ground_obstacle_filter",
                output="screen",
                parameters=[
                    os.path.join(nav, "config", "ground_obstacle_filter.yaml"), sim
                ],
            ),
            Node(
                package="isaac_fastlio_adapter", executable="pointcloud2_to_livox",
                output="screen", parameters=[sim],
            ),
            Node(
                package="fast_lio_localization", executable="fastlio_mapping",
                name="localization_fastlio", output="screen",
                parameters=[
                    os.path.join(package, "config", "fast_lio_localization_3d.yaml"), sim
                ],
            ),
            Node(
                package="isaac_localization_3d", executable="global_localization_xyz.py",
                name="global_localization", output="screen",
                parameters=[{
                    "use_sim_time": True,
                    "pcd_map_path": map_pcd,
                    "map_voxel_size": 0.4,
                    "scan_voxel_size": 0.1,
                    "freq_localization": 0.5,
                    "localization_threshold": 0.8,
                    "fov": 6.28319,
                    "fov_far": 30,
                }],
            ),
            Node(
                package="isaac_localization_3d", executable="global_pose_adapter.py",
                name="global_pose_adapter", output="screen",
                parameters=[
                    fusion_config,
                    {"use_sim_time": True, "auto_initial_pose": auto_initial_pose},
                ],
            ),
            Node(
                package="robot_localization", executable="ekf_node",
                name="global_ekf", output="screen",
                parameters=[fusion_config, sim],
                remappings=[("odometry/filtered", "/odometry/global")],
            ),
            Node(
                package="isaac_localization_3d", executable="global_tf_gate.py",
                name="global_tf_gate", output="screen", parameters=[sim],
            ),
            Node(
                package="nav2_map_server", executable="map_server",
                name="map_server", output="screen",
                parameters=[{"yaml_filename": map_pgm, "use_sim_time": True}],
            ),
            Node(
                package="nav2_lifecycle_manager", executable="lifecycle_manager",
                name="lifecycle_manager_fusion_map", output="screen",
                parameters=[{
                    "use_sim_time": True, "autostart": True, "node_names": ["map_server"]
                }],
            ),
            Node(
                condition=observing,
                package="isaac_localization_3d", executable="costmap_observer",
                namespace="global_costmap", name="global_costmap",
                output="screen",
                parameters=[
                    observation_config, sim
                ],
            ),
            Node(
                condition=observing,
                package="isaac_localization_3d", executable="costmap_observer",
                namespace="local_costmap", name="local_costmap",
                output="screen",
                parameters=[
                    observation_config, sim
                ],
            ),
            Node(
                condition=IfCondition(navigate),
                package="nav2_planner", executable="planner_server",
                name="planner_server", output="screen",
                parameters=[navigation_config, observation_config, sim],
            ),
            Node(
                condition=IfCondition(navigate),
                package="nav2_controller", executable="controller_server",
                name="controller_server", output="screen",
                parameters=[navigation_config, observation_config, sim],
                remappings=[("/cmd_vel", "/nav2/cmd_vel_nav")],
            ),
            Node(
                condition=IfCondition(navigate),
                package="nav2_behaviors", executable="behavior_server",
                name="behavior_server", output="screen",
                parameters=[navigation_config, sim],
                # Recovery motions go through the same smoother, collision_monitor
                # and cmd_vel_safety gates as the controller.
                remappings=[("/cmd_vel", "/nav2/cmd_vel_nav")],
            ),
            Node(
                condition=IfCondition(navigate),
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
                condition=IfCondition(navigate),
                package="nav2_velocity_smoother", executable="velocity_smoother",
                name="velocity_smoother", output="screen",
                parameters=[navigation_config, sim],
                # Smooth before the safety gates so their stops stay immediate.
                remappings=[
                    ("cmd_vel", "/nav2/cmd_vel_nav"), ("cmd_vel_smoothed", "/nav2/cmd_vel"),
                ],
            ),
            Node(
                condition=IfCondition(navigate),
                package="nav2_collision_monitor", executable="collision_monitor",
                name="collision_monitor", output="screen",
                parameters=[collision_config, sim],
            ),
            Node(
                condition=IfCondition(navigate),
                package="isaac_localization_3d", executable="cmd_vel_safety.py",
                name="cmd_vel_safety", output="screen", parameters=[collision_config, sim],
                remappings=[("/nav2/cmd_vel", "/nav2/cmd_vel_monitored")],
            ),
            readiness,
            RegisterEventHandler(
                OnProcessExit(
                    target_action=readiness,
                    on_exit=lambda event, context: (
                        ([navigation_manager, navigation_readiness]
                         if navigate.perform(context).lower() == "true"
                         else [costmap_manager]) if event.returncode == 0 else [
                            LogInfo(msg="ERROR: Costmaps not started: map or localization TF unavailable"),
                            EmitEvent(event=Shutdown(reason="Costmap readiness failed")),
                        ]
                    ),
                )
            ),
            RegisterEventHandler(
                OnProcessExit(
                    target_action=navigation_readiness,
                    on_exit=lambda event, _: [] if event.returncode == 0 else [
                        LogInfo(msg="ERROR: Nav2 navigation stack failed to activate"),
                        EmitEvent(event=Shutdown(reason="Navigation activation failed")),
                    ],
                )
            ),
            Node(
                condition=IfCondition(rviz), package="rviz2", executable="rviz2",
                output="screen",
                arguments=["-d", os.path.join(package, "config", "global_fusion.rviz")],
                parameters=[sim],
            ),
        ]
    )
