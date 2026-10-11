"""Gimbal action server (and MQTT bridge when mqtt_host is set) for one or more gimbals.

  ros2 launch slam_sensor_control gimbals.launch.py \
      "namespaces:=ceiling_cams/ceiling_cam_1;ceiling_cams/ceiling_cam_2" mqtt_host:=HOST

Each namespace gets gimbal/move and gimbal/rotate, and MQTT <mqtt_topic>/<namespace>/command.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def gimbal_nodes(context):
    value = lambda name: LaunchConfiguration(name).perform(context)
    namespaces = [ns.strip().strip("/") for ns in value("namespaces").split(";") if ns.strip().strip("/")]
    if not namespaces:
        raise RuntimeError("gimbals.launch.py needs namespaces:=NS[;NS...]")
    tilt_limits = [float(v) for v in value("tilt_limits_deg").split(",")]
    if len(tilt_limits) != 2 or tilt_limits[0] >= tilt_limits[1]:
        raise RuntimeError("tilt_limits_deg must be LOW,HIGH (deg)")
    mqtt_host = value("mqtt_host")
    nodes = []
    for namespace in namespaces:
        nodes.append(Node(
            package="slam_sensor_control", executable="gimbal_action_server.py",
            namespace=f"/{namespace}", output="screen",
            parameters=[{"tilt_limits_deg": tilt_limits}],
        ))
        if mqtt_host:
            nodes.append(Node(
                package="slam_sensor_control", executable="gimbal_mqtt_bridge.py",
                namespace=f"/{namespace}", output="screen",
                parameters=[{
                    "mqtt_host": mqtt_host,
                    "mqtt_port": int(value("mqtt_port")),
                    "mqtt_username": value("mqtt_username"),
                    "mqtt_password": value("mqtt_password"),
                    "mqtt_topic": value("mqtt_topic"),
                }],
            ))
    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("namespaces", description="Semicolon-separated gimbal namespaces"),
        DeclareLaunchArgument("tilt_limits_deg", default_value="-90,90",
                              description="Tilt limits LOW,HIGH (deg), as in the simulator"),
        DeclareLaunchArgument("mqtt_host", default_value="",
                              description="MQTT broker; when empty only the action servers start"),
        DeclareLaunchArgument("mqtt_port", default_value="1883"),
        DeclareLaunchArgument("mqtt_username", default_value=""),
        DeclareLaunchArgument("mqtt_password", default_value=""),
        DeclareLaunchArgument("mqtt_topic", default_value="slam/gimbal"),
        OpaqueFunction(function=gimbal_nodes),
    ])
