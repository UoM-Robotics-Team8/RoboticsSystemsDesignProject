import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    ld = LaunchDescription()

    channel_type =  LaunchConfiguration('channel_type', default='serial')
    serial_port = LaunchConfiguration('serial_port', default='/dev/ttyUSB0')
    serial_baudrate = LaunchConfiguration('serial_baudrate', default='256000')
    frame_id = LaunchConfiguration('frame_id', default='lidar_link')
    inverted = LaunchConfiguration('inverted', default='false')
    angle_compensate = LaunchConfiguration('angle_compensate', default='true')
    scan_mode = LaunchConfiguration('scan_mode', default='Sensitivity')

    pkg_name = 'hardware_package'

    yaml_path = os.path.join(get_package_share_directory(pkg_name), 'config/lidar_filter.yaml')

    # launch_lidar = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource([os.path.join(get_package_share_directory('rplidar_ros'), 'launch'),
    #                                    '/rplidar_a2m12_launch.py'])
    # )

    launch_lidar = Node(
        package='rplidar_ros',
        executable='rplidar_node',
        name='rplidar_node',
        output='screen',
        parameters=([{
            'channel_type':channel_type,
            'serial_port': serial_port,
            'serial_baudrate': serial_baudrate,
            'frame_id': frame_id,
            'inverted': inverted,
            'angle_compensate': angle_compensate,
            'scan_mode': scan_mode   
        }])
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
