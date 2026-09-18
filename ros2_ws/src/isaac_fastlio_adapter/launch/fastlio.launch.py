from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    lio_config_path = PathJoinSubstitution(
        [FindPackageShare("isaac_fastlio_adapter"), "config", "isaac_lio.yaml"]
    )
    pgo_config_path = PathJoinSubstitution(
        [FindPackageShare("pgo"), "config", "pgo.yaml"]
    )

    return LaunchDescription(
        [
            Node(
                package="isaac_fastlio_adapter",
                executable="pointcloud2_to_livox",
                name="pointcloud2_to_livox",
                output="screen",
                parameters=[{"use_sim_time": True}],
            ),
            Node(
                package="isaac_fastlio_adapter",
                executable="imu_scale_adapter",
                name="imu_scale_adapter",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": True,
                        "acceleration_scale": 0.1,
                    }
                ],
            ),
            Node(
                package="fastlio2",
                executable="lio_node",
                namespace="fastlio2",
                name="lio_node",
                output="screen",
                parameters=[
                    {
                        "config_path": lio_config_path,
                        "use_sim_time": True,
                    }
                ],
            ),
            Node(
                package="pgo",
                executable="pgo_node",
                namespace="pgo",
                name="pgo_node",
                output="screen",
                parameters=[
                    {
                        "config_path": pgo_config_path,
                        "use_sim_time": True,
                    }
                ],
            ),
        ]
    )
