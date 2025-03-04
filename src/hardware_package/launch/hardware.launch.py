import os
from ament_index_python.packages import get_package_share_directory, get_package_prefix
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch_ros.actions import Node, SetParameter
from launch.launch_description_sources import PythonLaunchDescriptionSource

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
        launch_arguments={}.items(),
    )

    launch_depth_camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(get_package_share_directory(pkg_name), 'launch'),
                                                    '/depth_camera.launch.py'])
    )

    ld.add_action(launch_slam)
    ld.add_action(launch_lidar)
    ld.add_action(launch_nav)
    ld.add_action(launch_explore)
    ld.add_action(launch_depth_camera)

    return ld
