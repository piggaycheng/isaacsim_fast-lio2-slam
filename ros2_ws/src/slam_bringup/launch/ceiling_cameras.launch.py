"""Bring up the gimbal control of the simulator's ceiling cameras.

The ceiling cameras belong to the site, not to a robot, so they run in their
own container:
  ros2 launch slam_bringup ceiling_cameras.launch.py mqtt_host:=HOST
Starts one gimbal action server (plus an MQTT bridge when mqtt_host is set)
per camera under /ceiling_cams/ceiling_cam_N, matching standalone.py
CEILING_CAMERA_NAMESPACE, CEILING_CAMERA_POSITIONS and
CEILING_GIMBAL_TILT_LIMITS_DEG.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

NAMESPACE = "ceiling_cams"
MQTT_ARGUMENTS = ("mqtt_host", "mqtt_port", "mqtt_username", "mqtt_password", "mqtt_topic")


def gimbals(context):
    count = int(LaunchConfiguration("count").perform(context))
    if count < 1:
        raise ValueError(f"count must be at least 1, got {count}")
    namespaces = ";".join(f"{NAMESPACE}/ceiling_cam_{index}" for index in range(1, count + 1))
    return [IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory("slam_sensor_control"), "launch", "gimbals.launch.py")),
        launch_arguments={
            "namespaces": namespaces,
            "tilt_limits_deg": LaunchConfiguration("tilt_limits_deg"),
            **{name: LaunchConfiguration(name) for name in MQTT_ARGUMENTS},
        }.items(),
    )]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("count", default_value="4",
                              description="Number of ceiling cameras (ceiling_cam_1..N)"),
        DeclareLaunchArgument("tilt_limits_deg", default_value="-90,90",
                              description="Must match the simulator's ceiling gimbal limits"),
        DeclareLaunchArgument("mqtt_host", default_value="",
                              description="MQTT broker; empty starts only the action servers"),
        DeclareLaunchArgument("mqtt_port", default_value="1883"),
        DeclareLaunchArgument("mqtt_username", default_value=""),
        DeclareLaunchArgument("mqtt_password", default_value=""),
        DeclareLaunchArgument("mqtt_topic", default_value="slam/gimbal"),
        OpaqueFunction(function=gimbals),
    ])
