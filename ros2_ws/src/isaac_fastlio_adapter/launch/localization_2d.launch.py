from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():
    package_share = get_package_share_directory("isaac_fastlio_adapter")
    config_file = os.path.join(package_share, "config", "localization_2d.yaml")
    rviz_config = os.path.join(package_share, "config", "localization_2d.rviz")

    map_file = LaunchConfiguration("map")
    use_sim_time = LaunchConfiguration("use_sim_time")
    use_rviz = LaunchConfiguration("rviz")

    common_parameters = [config_file, {"use_sim_time": use_sim_time}]

    return LaunchDescription(
        [
            DeclareLaunchArgument("map", description="Absolute path to a Nav2 map YAML file"),
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("rviz", default_value="true"),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="base_to_lidar_tf",
                arguments=[
                    "--x", "0.2317",
                    "--y", "0.0",
                    "--z", "0.526",
                    "--roll", "0.0",
                    "--pitch", "0.0",
                    "--yaw", "3.141592654",
                    "--frame-id", "base_link",
                    "--child-frame-id", "lidar_link",
                ],
                parameters=[{"use_sim_time": use_sim_time}],
            ),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="base_to_imu_tf",
                arguments=[
                    "--x", "0.2317",
                    "--y", "0.0",
                    "--z", "0.526",
                    "--roll", "0.0",
                    "--pitch", "0.0",
                    "--yaw", "3.141592654",
                    "--frame-id", "base_link",
                    "--child-frame-id", "imu_link",
                ],
                parameters=[{"use_sim_time": use_sim_time}],
            ),
            Node(
                package="isaac_fastlio_adapter",
                executable="wheel_encoder_odometry",
                name="wheel_encoder_odometry",
                output="screen",
                parameters=common_parameters,
            ),
            Node(
                package="isaac_fastlio_adapter",
                executable="imu_scale_adapter",
                name="nav_imu_adapter",
                output="screen",
                parameters=common_parameters,
            ),
            Node(
                package="pointcloud_to_laserscan",
                executable="pointcloud_to_laserscan_node",
                name="pointcloud_to_laserscan",
                remappings=[
                    ("cloud_in", "/isaac/lidar_points"),
                    ("scan", "/scan"),
                ],
                parameters=common_parameters,
            ),
            Node(
                package="robot_localization",
                executable="ekf_node",
                name="local_ekf",
                output="screen",
                parameters=common_parameters,
                remappings=[("odometry/filtered", "/odometry/local")],
            ),
            Node(
                package="nav2_map_server",
                executable="map_server",
                name="map_server",
                output="screen",
                parameters=[{"yaml_filename": map_file, "use_sim_time": use_sim_time}],
            ),
            Node(
                package="nav2_amcl",
                executable="amcl",
                name="amcl",
                output="screen",
                parameters=common_parameters,
            ),
            Node(
                package="nav2_lifecycle_manager",
                executable="lifecycle_manager",
                name="lifecycle_manager_localization",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": use_sim_time,
                        "autostart": True,
                        "node_names": ["map_server", "amcl"],
                    }
                ],
            ),
            Node(
                condition=IfCondition(use_rviz),
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                output="screen",
                arguments=["-d", rviz_config],
                parameters=[{"use_sim_time": use_sim_time}],
            ),
        ]
    )
