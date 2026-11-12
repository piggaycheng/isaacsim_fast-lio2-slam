import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    adapter_share = get_package_share_directory("isaac_fastlio_adapter")
    emap_share = get_package_share_directory("elevation_mapping_cupy")
    # Mounted project config; the container sees the repository at /workspace.
    default_elevation_config = os.environ.get("ELEVATION_CONFIG", "/workspace/config/elevation_isaac.yaml")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "fastlio",
                default_value="true",
                description="Also start standalone FAST-LIO2 (set false when global_fusion/localization_fastlio is already running)",
            ),
            DeclareLaunchArgument(
                "elevation_config",
                default_value=default_elevation_config,
                description="Path to elevation_mapping_cupy override YAML",
            ),
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(adapter_share, "launch", "fastlio.launch.py")),
                condition=IfCondition(LaunchConfiguration("fastlio")),
            ),
            Node(
                package="isaac_fastlio_adapter",
                executable="grid_map_to_points.py",
                name="grid_map_to_points",
                output="screen",
                parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}],
            ),
            Node(
                package="elevation_mapping_cupy",
                executable="elevation_mapping_node.py",
                name="elevation_mapping_node",
                output="screen",
                parameters=[
                    os.path.join(emap_share, "config", "core", "core_param.yaml"),
                    LaunchConfiguration("elevation_config"),
                    {"use_sim_time": LaunchConfiguration("use_sim_time")},
                ],
            ),
        ]
    )
