import os
from ament_index_python.packages import get_package_share_directory, get_package_prefix
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch_ros.actions import Node, SetParameter
from launch.launch_description_sources import PythonLaunchDescriptionSource
import time

def generate_launch_description():
    ld = LaunchDescription()

    pkg_name = "hardware_package"
    
    launch_slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(get_package_share_directory(pkg_name), 'launch'),
                                                    '/slam.launch.py'])
    )

    launch_lidar = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(get_package_share_directory(pkg_name), 'launch'),
                                                    '/lidar.launch.py'])
    )

    launch_nav = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(get_package_share_directory(pkg_name), 'launch'),
                                                    '/nav.launch.py'])
    )

    launch_explore = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([get_package_share_directory('explore_lite'), '/launch', 
                                                    '/explore.launch.py']),
        launch_arguments={
            "return_to_init": True
        }.items(),
    )

    launch_ekf = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(get_package_share_directory(pkg_name), 'launch'),
                                                    '/ekf.launch.py'])
    )

    launch_listener = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(get_package_share_directory(pkg_name), 'launch'),
                                                    '/launch_listener.launch.py'])
    )

    ld.add_action(launch_ekf)
    time.sleep(5)
    ld.add_action(launch_lidar)
    time.sleep(5)
    ld.add_action(launch_slam)
    time.sleep(5)
    ld.add_action(launch_nav)
    time.sleep(5)
    ld.add_action(launch_explore)
    time.sleep(5)
    ld.add_action(launch_listener)


    return ld
