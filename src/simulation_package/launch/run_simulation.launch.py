import os
from ament_index_python.packages import get_package_share_directory, get_package_prefix
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch_ros.actions import Node, SetParameter
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution

def generate_launch_description():

    ld = LaunchDescription()

    launch_slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([get_package_share_directory('simulation_package'), '/launch', '/slam.launch.py']),
        launch_arguments={}.items(),
    )
    
    launch_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([get_package_share_directory('simulation_package'), '/launch', '/simulation_bringup.launch.py']),
        launch_arguments={}.items(),
    )

    launch_nav = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([get_package_share_directory('simulation_package'), '/launch', '/nav.launch.py']),
        launch_arguments={}.items(),
    )
    
    ld.add_action(launch_sim)

    ld.add_action(launch_slam)
    
    ld.add_action(launch_nav)

    return ld
