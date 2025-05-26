import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    ld = LaunchDescription()

    node_listener = Node(
        package='hardware_package',
        executable='object_detection_listener',
        output='screen'
    )

    ld.add_action(node_listener)

    return ld
