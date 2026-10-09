"""Launch the namespaced global_fusion stack of ONE robot from its spawn spec.

Each robot runs this in its own container (as it would on its own computer):
  ros2 launch slam_localization_3d robot.launch.py robot:=carter2:nova_carter@3.5,0,1.57 ...
The spec sets the namespace (/NAME, TF on /NAME/tf), the robot type profile
(config/robots/TYPE.yaml) and the auto_initial_pose guess derived from the
simulator spawn pose X,Y,YAW. A real robot without a spawn pose can launch
global_fusion.launch.py directly with namespace:=, robot_type:= and initial_*:=.
"""

import os
import sys

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from robot_fleet import initial_pose, load_robot_profile, parse_robot_spec, rotation_radius  # noqa: E402
from robot_namespace import load_parameters  # noqa: E402

FORWARDED = (
    "map_pcd", "map_pgm", "rviz", "auto_initial_pose", "obstacle_cloud", "costmaps", "navigate",
    "adaptive_surround", "direction_zones", "local_odometry_inputs",
)


def robot_stack(context, package):
    spec = parse_robot_spec(LaunchConfiguration("robot").perform(context))
    pose = initial_pose(spec, load_robot_profile(spec.robot_type))
    arguments = {name: LaunchConfiguration(name) for name in FORWARDED}
    arguments.update(
        namespace=spec.name, robot_type=spec.robot_type,
        **{f"initial_{axis}": str(value) for axis, value in zip(("x", "y", "z", "yaw"), pose)},
    )
    actions = [
        LogInfo(msg=f"Robot {spec.name}: type={spec.robot_type} "
                    f"fleet={spec.fleet} spawn=({spec.x}, {spec.y}, {spec.yaw}) initial_pose={pose}"),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(package, "launch", "global_fusion.launch.py")),
            launch_arguments=arguments.items(),
        ),
    ]
    mqtt_host = LaunchConfiguration("mqtt_host").perform(context)
    if mqtt_host:
        # Open-RMF plans traffic with the same circle the planner and rotate zone use.
        radius = rotation_radius(load_parameters(
            os.path.join(package, "config", "observation_costmaps.yaml"), spec.robot_type))
        # cmd_vel_safety's caps bound every command the robot can execute.
        safety = load_parameters(
            os.path.join(package, "config", "collision_monitor.yaml"), spec.robot_type,
        )["cmd_vel_safety"]["ros__parameters"]
        actions.append(Node(
            package="slam_fleet_bridge", executable="fleet_bridge_node.py",
            # Absolute: global_fusion's pushed namespace would otherwise prefix it again.
            namespace=f"/{spec.name}", output="screen",
            parameters=[{
                "use_sim_time": True,
                "mqtt_host": mqtt_host,
                "mqtt_port": int(LaunchConfiguration("mqtt_port").perform(context)),
                "mqtt_username": LaunchConfiguration("mqtt_username").perform(context),
                "mqtt_password": LaunchConfiguration("mqtt_password").perform(context),
                "fleet_name": spec.fleet,
                "footprint_radius": radius,
                "max_linear_velocity": float(safety["max_linear_speed"]),
                "max_angular_velocity": float(safety["max_angular_speed"]),
            }],
        ))
    return actions


def generate_launch_description():
    package = get_package_share_directory("slam_localization_3d")
    return LaunchDescription([
        DeclareLaunchArgument(
            "robot", description="NAME[:TYPE][#FLEET]@X,Y[,YAW] simulator spawn pose of this robot "
                                  "(FLEET is its Open-RMF fleet, default default_fleet)",
        ),
        DeclareLaunchArgument("map_pcd", description="Absolute PCD map path"),
        DeclareLaunchArgument("map_pgm", description="Absolute Nav2 map YAML path"),
        DeclareLaunchArgument("rviz", default_value="false"),
        DeclareLaunchArgument("auto_initial_pose", default_value="true"),
        DeclareLaunchArgument("obstacle_cloud", default_value="true"),
        DeclareLaunchArgument("costmaps", default_value="true"),
        DeclareLaunchArgument("navigate", default_value="true"),
        DeclareLaunchArgument("adaptive_surround", default_value="false"),
        DeclareLaunchArgument("direction_zones", default_value="true"),
        DeclareLaunchArgument(
            "local_odometry_inputs", default_value="",
            description="Comma-separated local_ekf inputs (wheel, imu, lio); empty uses the profile",
        ),
        DeclareLaunchArgument(
            "mqtt_host", default_value="",
            description="MQTT broker; when set, starts the fleet bridge (Open-RMF protocol over MQTT)",
        ),
        DeclareLaunchArgument("mqtt_port", default_value="1883"),
        DeclareLaunchArgument("mqtt_username", default_value=""),
        DeclareLaunchArgument("mqtt_password", default_value=""),
        OpaqueFunction(function=robot_stack, args=[package]),
    ])
