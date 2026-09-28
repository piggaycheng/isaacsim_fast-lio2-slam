import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, LogInfo, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


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
    nav_parameters = [os.path.join(nav, "config", "localization_2d.yaml"), sim]
    observation_config = os.path.join(package, "config", "observation_costmaps.yaml")
    navigation_config = os.path.join(package, "config", "navigation.yaml")
    observing = IfCondition(PythonExpression([
        "'", costmaps, "' == 'true' and '", navigate, "' == 'false'",
    ]))
    readiness = Node(
        condition=IfCondition(costmaps),
        package="isaac_localization_3d", executable="wait_for_costmap_tf.py",
        output="screen", parameters=[sim],
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
            "node_names": ["planner_server", "controller_server", "bt_navigator"],
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
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="base_to_imu_tf",
                arguments=[
                    "--x", "0.2317", "--y", "0", "--z", "0.526",
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
                    "--x", "0.2317", "--y", "0", "--z", "0.526",
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
                    ("cloud_in", "/isaac/lidar_points"),
                    ("scan", "/scan"),
                ],
                parameters=nav_parameters,
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
                package="fast_lio_localization", executable="global_localization.py",
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
                parameters=[{"use_sim_time": True, "auto_initial_pose": auto_initial_pose}],
            ),
            Node(
                package="robot_localization", executable="ekf_node",
                name="global_ekf", output="screen",
                parameters=[os.path.join(package, "config", "global_fusion.yaml"), sim],
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
                remappings=[("/cmd_vel", "/nav2/cmd_vel")],
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
                package="isaac_localization_3d", executable="cmd_vel_safety.py",
                name="cmd_vel_safety", output="screen", parameters=[sim],
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
