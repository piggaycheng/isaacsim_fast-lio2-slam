"""Bring up everything ONE robot runs, from its spawn spec.

Each robot runs this in its own container (as it would on its own computer):
  ros2 launch slam_bringup robot.launch.py robot:=carter2:nova_carter@3.5,0,1.57 ...
It composes slam_localization_3d's global_fusion stack (localization and
navigation), slam_sensor_control's gimbals.launch.py when the robot profile
has a gimbal, and slam_fleet_bridge's MQTT fleet bridge when mqtt_host is set.
The spec sets the namespace (/NAME, TF on /NAME/tf), the robot type profile
(slam_localization_3d config/robots/TYPE.yaml) and the auto_initial_pose guess
derived from the simulator spawn pose X,Y,YAW. A real robot without a spawn
pose can launch global_fusion.launch.py directly with namespace:=,
robot_type:= and initial_*:=.
"""

import os
import sys
from pathlib import Path

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

LOCALIZATION = get_package_share_directory("slam_localization_3d")
sys.path.insert(0, os.path.join(LOCALIZATION, "launch"))
from robot_fleet import body_pose, imu_mount, initial_pose, load_robot_profile, parse_robot_spec, rotation_radius  # noqa: E402
from robot_namespace import load_parameters  # noqa: E402

sys.path.insert(0, os.path.join(
    get_package_prefix("slam_localization_3d"), "lib", "slam_localization_3d"))
from fleet_ground_truth import FloorReference  # noqa: E402

FORWARDED = (
    "map_pcd", "map_pgm", "rviz", "auto_initial_pose", "obstacle_cloud", "costmaps", "navigate",
    "direction_zones", "local_odometry_inputs", "local_odometry_3d", "maps_root", "fleet_filters",
)


def robot_stack(context, package):
    spec = parse_robot_spec(LaunchConfiguration("robot").perform(context))
    profile = load_robot_profile(spec.robot_type)
    pose = initial_pose(spec, profile)
    if LaunchConfiguration("auto_initial_pose").perform(context).lower() == "true":
        map_yaml = Path(LaunchConfiguration("map_pgm").perform(context))
        session = map_yaml.parent / "session.json"
        if map_yaml.parent.name != "office" or session.exists():
            try:
                if Path(LaunchConfiguration("map_pcd").perform(context)).parent != map_yaml.parent:
                    raise ValueError("Automatic initialization requires PCD and YAML in the same floor directory")
                reference = FloorReference.load(map_yaml.parent.name, session)
                pose = reference.initial_body_pose(body_pose(spec, profile))
            except (OSError, ValueError, KeyError, TypeError) as error:
                raise ValueError(
                    f"Cannot derive automatic initial pose from {session}: {error}. "
                    "Use --manual-initial-pose (auto_initial_pose:=false) to initialize manually."
                ) from error
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
    gimbal = profile["simulation"].get("gimbal")
    if gimbal is not None:
        # Action server, plus MQTT slam/gimbal/NAME/command when mqtt_host is set.
        limits = gimbal.get("tilt_limits_deg", (-90.0, 90.0))
        actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory("slam_sensor_control"), "launch", "gimbals.launch.py")),
            launch_arguments={
                "namespaces": spec.name,
                "tilt_limits_deg": f"{float(limits[0])},{float(limits[1])}",
                **{name: LaunchConfiguration(name) for name in (
                    "mqtt_host", "mqtt_port", "mqtt_username", "mqtt_password")},
            }.items(),
        ))
    if mqtt_host:
        # Open-RMF's circular traffic envelope covers the padded body's rotation sweep.
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
                "level_name": os.path.basename(os.path.dirname(
                    LaunchConfiguration("map_pgm").perform(context))),
                # base_link height in each floor map (map = mapping robot's body frame).
                "floor_base_z": pose[2] - imu_mount(profile)[2],
                "footprint_radius": radius,
                "max_linear_velocity": float(safety["max_linear_speed"]),
                "max_angular_velocity": float(safety["max_angular_speed"]),
            }],
        ))
    return actions


def generate_launch_description():
    package = LOCALIZATION
    return LaunchDescription([
        DeclareLaunchArgument(
            "robot", description="NAME[:TYPE][#FLEET]@X,Y[,YAW] simulator spawn pose of this robot "
                                  "(FLEET is its Open-RMF fleet, default default_fleet)",
        ),
        DeclareLaunchArgument("map_pcd", default_value=os.path.join(
            os.getcwd(), "maps/1F/map.pcd"), description="Absolute PCD map path"),
        DeclareLaunchArgument("map_pgm", default_value=os.path.join(
            os.getcwd(), "maps/1F/map_2d.yaml"), description="Absolute Nav2 map YAML path"),
        DeclareLaunchArgument("maps_root", default_value=""),
        DeclareLaunchArgument("rviz", default_value="false"),
        DeclareLaunchArgument("auto_initial_pose", default_value="false",
                              description="Send one spawn/session-derived prior after LIO and ICP are ready"),
        DeclareLaunchArgument("obstacle_cloud", default_value="true"),
        DeclareLaunchArgument("costmaps", default_value="true"),
        DeclareLaunchArgument("navigate", default_value="true"),
        DeclareLaunchArgument("direction_zones", default_value="true"),
        DeclareLaunchArgument(
            "fleet_filters", default_value="false",
            description="Apply the fleet RViz editor's per-floor Keepout/Speed zones "
                        "(fleet_rviz.launch.py filter_editor:=true must serve them)",
        ),
        DeclareLaunchArgument(
            "local_odometry_inputs", default_value="",
            description="Comma-separated local_ekf inputs (wheel, imu, lio); empty uses the profile",
        ),
        DeclareLaunchArgument(
            "local_odometry_3d", default_value="",
            description="true/false 3D odom -> base_link (needs lio); empty uses the profile",
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
