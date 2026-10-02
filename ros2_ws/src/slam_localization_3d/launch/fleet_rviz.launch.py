"""One RViz for the whole fleet, run in its own (operator) container.

robots:="carter1;carter2" (names or NAME[:TYPE]@X,Y[,YAW] specs, ";"-separated). fleet_relay
merges each /<ns>/tf into /fleet/tf as <ns>/<frame>; RViz reads that tree.
Fleet Control panel selects the robot for the shared goal and initial-pose tools.
"""

import os
import sys
import tempfile

import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from robot_fleet import fleet_rviz, normalize_namespace, parse_robot_spec  # noqa: E402

TF_REMAPS = [("/tf", "/fleet/tf"), ("/tf_static", "/fleet/tf_static")]


def robot_names(text):
    names = []
    for item in (part.strip() for part in text.split(";")):
        if item:
            names.append(parse_robot_spec(item).name if "@" in item
                         else normalize_namespace(item.split(":")[0]))
    if not names or len({name.lower() for name in names}) != len(names):
        raise ValueError(f"robots must list unique robot names, got {text!r}")
    return names


def fleet_nodes(context):
    names = robot_names(LaunchConfiguration("robots").perform(context))
    handle = tempfile.NamedTemporaryFile(
        "w", prefix="fleet_", suffix=".rviz", delete=False)
    with handle:
        yaml.safe_dump(fleet_rviz(names), handle, sort_keys=False)
    sim_time = {"use_sim_time": LaunchConfiguration("use_sim_time")}
    actions = [
        LogInfo(msg=f"Fleet RViz for {', '.join(names)}"),
        Node(package="slam_localization_3d", executable="fleet_relay.py", name="fleet_relay",
             output="screen", parameters=[sim_time, {"robots": names}]),
    ]
    if LaunchConfiguration("rviz").perform(context).lower() == "true":
        actions.append(Node(
            package="rviz2", executable="rviz2", name="fleet_rviz", output="screen",
            arguments=["-d", handle.name], parameters=[sim_time], remappings=TF_REMAPS))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("robots", description="Robot names or specs, separated by ;"),
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("rviz", default_value="true",
                              description="false runs only the relay (headless checks)"),
        OpaqueFunction(function=fleet_nodes),
    ])
