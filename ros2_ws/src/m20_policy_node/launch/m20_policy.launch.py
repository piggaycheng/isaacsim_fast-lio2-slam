import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    height_map_launch = os.path.join(
        get_package_share_directory("isaac_fastlio_adapter"), "launch", "height_map.launch.py"
    )
    default_elevation_config = os.environ.get("ELEVATION_CONFIG", "/workspace/config/elevation_isaac.yaml")
    return LaunchDescription(
        [
            DeclareLaunchArgument("policy_path", description="TorchScript policy.pt exported by isaaclab play"),
            DeclareLaunchArgument("device", default_value="cpu"),
            DeclareLaunchArgument("use_sim_time", default_value="true", description="Follow /clock; set false on the real robot"),
            DeclareLaunchArgument(
                "use_height_scan",
                default_value="true",
                description="Rough-terrain policy: observe a height scan from elevation_mapping_cupy (false for the flat policy)",
            ),
            DeclareLaunchArgument(
                "flat_height_scan",
                default_value="false",
                description="Debug: ignore the elevation map and observe flat ground",
            ),
            DeclareLaunchArgument(
                "height_map",
                default_value="true",
                description="Also start elevation_mapping_cupy (set false if it runs elsewhere)",
            ),
            DeclareLaunchArgument(
                "fastlio",
                default_value="true",
                description="Also start standalone FAST-LIO2 inside height_map.launch.py (set false when global_fusion is running)",
            ),
            DeclareLaunchArgument(
                "elevation_config",
                default_value=default_elevation_config,
                description="Path to elevation_mapping_cupy override YAML",
            ),
            DeclareLaunchArgument(
                "control_rate_hz",
                default_value="50.0",
                description="Inference rate (timer on the latest state); 0 = one inference per joint_states message",
            ),
            DeclareLaunchArgument(
                "best_effort_qos",
                default_value="true",
                description="Best effort QoS for joint_states/imu/base_velocity/joint_commands (false = reliable)",
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(height_map_launch),
                launch_arguments={
                    "fastlio": LaunchConfiguration("fastlio"),
                    "elevation_config": LaunchConfiguration("elevation_config"),
                    "use_sim_time": LaunchConfiguration("use_sim_time"),
                }.items(),
                condition=IfCondition(LaunchConfiguration("height_map")),
            ),
            Node(
                package="m20_policy_node",
                executable="m20_policy_node",
                name="m20_policy_node",
                output="screen",
                parameters=[
                    {
                        "policy_path": LaunchConfiguration("policy_path"),
                        "device": LaunchConfiguration("device"),
                        "use_sim_time": LaunchConfiguration("use_sim_time"),
                        "use_height_scan": LaunchConfiguration("use_height_scan"),
                        "flat_height_scan": LaunchConfiguration("flat_height_scan"),
                        "control_rate_hz": LaunchConfiguration("control_rate_hz"),
                        "best_effort_qos": LaunchConfiguration("best_effort_qos"),
                    }
                ],
            ),
        ]
    )
