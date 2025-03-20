import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    ld = LaunchDescription()

    pkg_name = 'sensors_package'

    lifecycle_nodes = ['example_map_server', 'amcl']
    node_lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        output='screen',
        parameters=[{'node_names': lifecycle_nodes, 'autostart': True}]  # add other parameters here if required
    )

    yaml_path = os.path.join(get_package_share_directory(pkg_name), 'config/filter.yaml')

    node_laser_filter = Node(
        package='laser_filters',
        executable='scan_to_scan_filter_chain',
        parameters=[{'yaml_filename': yaml_path}]
    )

    ld.add_action(node_lifecycle_manager)
    ld.add_action(node_laser_filter)

    return ld
