import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_share = get_package_share_directory("isaac_localization_3d")
    map_path = LaunchConfiguration("map_pcd")
    map_pgm = LaunchConfiguration("map_pgm")
    rviz = LaunchConfiguration("rviz")
    auto_initial_pose = LaunchConfiguration("auto_initial_pose")
    return LaunchDescription(
        [
            DeclareLaunchArgument("map_pcd", description="Absolute path to a PCD map"),
            DeclareLaunchArgument("map_pgm", description="Absolute path to a Nav2 map YAML"),
            DeclareLaunchArgument("rviz", default_value="true"),
            DeclareLaunchArgument("auto_initial_pose", default_value="false"),
            Node(
                package="isaac_fastlio_adapter",
                executable="pointcloud2_to_livox",
                output="screen",
                parameters=[{"use_sim_time": True}],
            ),
            Node(
                package="fast_lio_localization",
                executable="fastlio_mapping",
                name="localization_fastlio",
                output="screen",
                parameters=[
                    os.path.join(package_share, "config", "fast_lio_localization_3d.yaml"),
                    {"use_sim_time": True},
                ],
            ),
            Node(
                package="isaac_localization_3d",
                executable="global_localization_xyz.py",
                name="global_localization",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": True,
                        "pcd_map_path": map_path,
                        "map_voxel_size": 0.4,
                        "scan_voxel_size": 0.1,
                        "freq_localization": 0.5,
                        "localization_threshold": 0.8,
                        "fov": 6.28319,
                        "fov_far": 30,
                    }
                ],
            ),
            Node(
                package="isaac_localization_3d",
                executable="localization_3d_pose.py",
                output="screen",
                parameters=[
                    {"use_sim_time": True, "auto_initial_pose": auto_initial_pose}
                ],
            ),
            Node(
                package="nav2_map_server",
                executable="map_server",
                name="map_server",
                output="screen",
                parameters=[{"yaml_filename": map_pgm, "use_sim_time": True}],
            ),
            Node(
                package="nav2_lifecycle_manager",
                executable="lifecycle_manager",
                name="lifecycle_manager_3d_map",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": True,
                        "autostart": True,
                        "node_names": ["map_server"],
                    }
                ],
            ),
            Node(
                condition=IfCondition(rviz),
                package="rviz2",
                executable="rviz2",
                output="screen",
                arguments=[
                    "-d", os.path.join(package_share, "config", "localization_3d.rviz")
                ],
                parameters=[{"use_sim_time": True}],
            ),
        ]
    )
