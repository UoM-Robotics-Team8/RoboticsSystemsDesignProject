import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    ld = LaunchDescription()

    pkg_name = ""

    node_depth_sensor = Node(
        package="object_detect_package",
        executable="image_subscriber",
        output="screen"
    )

    ld .add_action(node_depth_sensor)

    return ld
