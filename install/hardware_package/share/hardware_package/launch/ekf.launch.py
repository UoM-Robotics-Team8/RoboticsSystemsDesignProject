from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
import os
import yaml
from launch.substitutions import EnvironmentVariable
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node

def generate_launch_description():

    ld = LaunchDescription()

    launch_efk = Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[os.path.join(get_package_share_directory("hardware_package"), 'config', 'ekf_node.yaml')],

    )

    ld.add_action(launch_efk)

    return ld