from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([Node(package="m20_jog", executable="m20_jog", name="m20_jog", output="screen")])
