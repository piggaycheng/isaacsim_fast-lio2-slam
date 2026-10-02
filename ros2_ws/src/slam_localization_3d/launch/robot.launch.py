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

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from robot_fleet import initial_pose, load_robot_profile, parse_robot_spec  # noqa: E402

FORWARDED = (
    "map_pcd", "map_pgm", "rviz", "auto_initial_pose", "obstacle_cloud", "costmaps", "navigate",
    "adaptive_surround",
)


def robot_stack(context, package):
    spec = parse_robot_spec(LaunchConfiguration("robot").perform(context))
    pose = initial_pose(spec, load_robot_profile(spec.robot_type))
    arguments = {name: LaunchConfiguration(name) for name in FORWARDED}
    arguments.update(
        namespace=spec.name, robot_type=spec.robot_type,
        **{f"initial_{axis}": str(value) for axis, value in zip(("x", "y", "z", "yaw"), pose)},
    )
    return [
        LogInfo(msg=f"Robot {spec.name}: type={spec.robot_type} "
                    f"spawn=({spec.x}, {spec.y}, {spec.yaw}) initial_pose={pose}"),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(package, "launch", "global_fusion.launch.py")),
            launch_arguments=arguments.items(),
        ),
    ]


def generate_launch_description():
    package = get_package_share_directory("slam_localization_3d")
    return LaunchDescription([
        DeclareLaunchArgument(
            "robot", description="NAME[:TYPE]@X,Y[,YAW] simulator spawn pose of this robot",
        ),
        DeclareLaunchArgument("map_pcd", description="Absolute PCD map path"),
        DeclareLaunchArgument("map_pgm", description="Absolute Nav2 map YAML path"),
        DeclareLaunchArgument("rviz", default_value="false"),
        DeclareLaunchArgument("auto_initial_pose", default_value="true"),
        DeclareLaunchArgument("obstacle_cloud", default_value="true"),
        DeclareLaunchArgument("costmaps", default_value="true"),
        DeclareLaunchArgument("navigate", default_value="true"),
        DeclareLaunchArgument("adaptive_surround", default_value="false"),
        OpaqueFunction(function=robot_stack, args=[package]),
    ])
