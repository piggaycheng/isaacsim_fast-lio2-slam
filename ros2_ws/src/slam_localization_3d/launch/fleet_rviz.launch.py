"""One RViz for the whole fleet, run in its own (operator) container.

robots:="carter1;carter2" (names or NAME[:TYPE][#FLEET]@X,Y[,YAW] specs, ";"-separated). fleet_relay
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
from robot_fleet import fleet_floor_maps, fleet_rviz, normalize_namespace, parse_robot_spec  # noqa: E402

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
    robots = LaunchConfiguration("robots").perform(context)
    names = robot_names(robots)
    specs = [parse_robot_spec(item.strip()) for item in robots.split(";") if "@" in item]
    simulation_reference = LaunchConfiguration("simulation_reference").perform(context).lower() == "true"
    maps = fleet_floor_maps(
        LaunchConfiguration("maps_root").perform(context),
        LaunchConfiguration("initial_floor").perform(context),
        floors=[name.strip() for name in LaunchConfiguration("floors").perform(context).split(";")],
    )
    handle = tempfile.NamedTemporaryFile(
        "w", prefix="fleet_", suffix=".rviz", delete=False)
    with handle:
        yaml.safe_dump(fleet_rviz(names, [name for name, _ in maps], simulation_reference),
                       handle, sort_keys=False)
    sim_time = {"use_sim_time": LaunchConfiguration("use_sim_time")}
    actions = [
        LogInfo(msg=f"Fleet RViz for {', '.join(names)}"),
        Node(package="slam_localization_3d", executable="fleet_relay.py", name="fleet_relay",
             output="screen", parameters=[sim_time, {"robots": names}]),
    ]
    if simulation_reference:
        actions.append(Node(package="slam_localization_3d", executable="fleet_ground_truth.py",
             name="fleet_ground_truth", output="screen", parameters=[sim_time, {
                 "robots": names,
                 "spawn_robots": [spec.name for spec in specs] or [""],
                 "spawn_poses": [value for spec in specs for value in (spec.x, spec.y, spec.yaw)]
                                or [0.0, 0.0, 0.0],
                 "floors": [name for name, _ in maps],
                 "session_files": [str(path.parent / "session.json") for _, path in maps],
             }]))
    for index, (_, path) in enumerate(maps):
        actions.append(Node(
            package="nav2_map_server", executable="map_server",
            namespace=f"/fleet/maps/floor_{index}", name="map_server", output="screen",
            parameters=[sim_time, {"yaml_filename": str(path), "frame_id": "map"}],
        ))
        actions.append(Node(
            package="nav2_lifecycle_manager", executable="lifecycle_manager",
            namespace=f"/fleet/maps/floor_{index}", name="lifecycle_manager", output="screen",
            parameters=[sim_time, {"autostart": True, "node_names": ["map_server"]}],
        ))
    if LaunchConfiguration("rviz").perform(context).lower() == "true":
        actions.append(Node(
            package="rviz2", executable="rviz2", name="fleet_rviz", output="screen",
            arguments=["-d", handle.name], parameters=[sim_time], remappings=TF_REMAPS))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("robots", description="Robot names or specs, separated by ;"),
        DeclareLaunchArgument("maps_root", default_value=os.path.join(os.getcwd(), "maps"),
                              description="Directory containing FLOOR/map_2d.yaml monitoring maps"),
        DeclareLaunchArgument("initial_floor", default_value="1F"),
        DeclareLaunchArgument("floors", default_value="1F;B1",
                              description="Monitoring floors separated by ; (initial floor is included)"),
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("rviz", default_value="true",
                              description="false runs only the relay and monitoring maps"),
        DeclareLaunchArgument("simulation_reference", default_value="true",
                              description="Show reference arrows until each robot receives initialpose"),
        OpaqueFunction(function=fleet_nodes),
    ])
