import os
from ament_index_python.packages import get_package_share_directory, get_package_prefix
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch_ros.actions import Node, SetParameter
from launch.launch_description_sources import PythonLaunchDescriptionSource
import time
from launch.substitutions import LaunchConfiguration

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

    # launch_pointcloud_to_laser = Node(
    #         package='pointcloud_to_laserscan', executable='dummy_pointcloud_publisher',
    #         remappings=[('cloud', [LaunchConfiguration(variable_name='scanner'), '/cloud'])],
    #         parameters=[{'cloud_frame_id': 'cloud', 'cloud_extent': 2.0, 'cloud_size': 500}],
    #         name='cloud_publisher'
    # )

    cloud_to_map_transform = Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_transform_publisher',
            arguments=[
                '--x', '0', '--y', '0', '--z', '-0.15',
                '--qx', '0', '--qy', '0', '--qz', '0', '--qw', '1',
                '--frame-id', 'camera_camera_link', '--child-frame-id', 'cloud'
            ]
    )

    point_cloud_to_laser_node = Node(
            package='pointcloud_to_laserscan', executable='pointcloud_to_laserscan_node',
            remappings=[('cloud_in', '/camera/camera/depth/color/points'),
                        ('scan', 'depth/scan')],
            parameters=[{
                'target_frame': 'camera_camera_link',
                'transform_tolerance': 0.01,
                'min_height': 0.0,
                'max_height': 1.0,
                'angle_min': -1.5708,  # -M_PI/2
                'angle_max': 1.5708,  # M_PI/2
                'angle_increment': 0.0087,  # M_PI/360.0
                'scan_time': 0.3333,
                'range_min': 0.45,
                'range_max': 4.0,
                'use_inf': True,
                'inf_epsilon': 1.0
            }],
            name='pointcloud_to_laserscan'
        )

    ld.add_action(launch_ekf)
    time.sleep(5)
    ld.add_action(launch_lidar)
    time.sleep(5)
    ld.add_action(launch_slam)
    time.sleep(5)
    ld.add_action(launch_nav)
    time.sleep(5)
    ld.add_action(launch_listener)
    time.sleep(1)
    #ld.add_action(launch_pointcloud_to_laser)
    ld.add_action(cloud_to_map_transform)
    ld.add_action(point_cloud_to_laser_node)
    time.sleep(5)
    ld.add_action(launch_explore)

    return ld
