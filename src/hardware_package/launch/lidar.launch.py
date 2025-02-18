import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    ld = LaunchDescription()

    pkg_name = 'hardware_package'

    yaml_path = os.path.join(get_package_share_directory(pkg_name), 'config/lidar_filter.yaml')

    launch_lidar = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(get_package_share_directory('rplidar_ros'), 'launch'),
                                       '/view_rplidar_a2m12_launch.py'])
    )

    node_lidar = Node(
        package='hardware_package',
        executable='lidar_node.py',
        output='screen'
    )

    node_laser_filter = Node(
        package='laser_filters',
        executable='scan_to_scan_filter_chain',
        parameters=[{'yaml_filename': yaml_path}]
    )

    # node_rviz = Node(
    #     package='rviz2',
    #     namespace='',
    #     executable='rviz2',
    #     name='rviz2',
    #     #arguments=['-d']
    # )

    ld.add_action(launch_lidar)
    ld.add_action(node_lidar)
    ld.add_action(node_laser_filter)
    #ld.add_action(node_rviz)

    return ld
