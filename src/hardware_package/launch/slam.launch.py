import os
from ament_index_python.packages import get_package_share_directory, get_package_prefix
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch_ros.actions import Node, SetParameter
from launch.launch_description_sources import PythonLaunchDescriptionSource
import xacro


def generate_launch_description():
    ld = LaunchDescription()

    # Specify the name of the package and path to xacro file in external package
    pkg_name = 'hardware_package'

    file_subpath = 'urdf/leo.urdf.xacro'

    # Use xacro to process the URDF file
    xacro_file = os.path.join(get_package_share_directory(pkg_name),file_subpath)
    robot_description_raw = xacro.process_file(xacro_file).toxml()                                                  

    # robot state publisher node
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_raw}] # add other parameters here if required
    )

    # Start SLAM Toolbox with default parameters
    # launch_slam_toolbox = IncludeLaunchDescription(
    #   PythonLaunchDescriptionSource([os.path.join(
    #      get_package_share_directory('slam_toolbox'), 'launch'),
    #      '/online_async_launch.py'])
    # )

    launch_slam_toolbox = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        #remappings=[('/scan', '/scan_filtered')],
        parameters=[{
            'transform_timeout' : 1.0,
            'max_laser_queue_size' : 100,
            'use_sim_time' : False,
            'odom_frame' : "odom",
            'base_frame' : "base_footprint"
        }]
    )

    # Rviz node
    rviz_config_file = os.path.join(get_package_share_directory(pkg_name), 'rviz', 'nav2.rviz')
    node_rviz = Node(
        package='rviz2',
        namespace='',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file]
    )


    # Add actions to LaunchDescription
    ld.add_action(node_robot_state_publisher)
    # SLAM Toolbox
    ld.add_action(launch_slam_toolbox)
    # Visualisation
    ld.add_action(node_rviz)

    return ld