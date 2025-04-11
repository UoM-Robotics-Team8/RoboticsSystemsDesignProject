import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node, SetParameter
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    ld = LaunchDescription()

    # Specify the name of the package and path to xacro file in external package
    pkg_name = 'simulation_package'

    # Start simulation
    # launch_sim_world = IncludeLaunchDescription(
    #   PythonLaunchDescriptionSource([os.path.join(
    #      get_package_share_directory(pkg_name), 'launch'),
    #      '/simulation_bringup.launch.py'])
    #   )


    # Start SLAM Toolbox with default parameters
    launch_slam_toolbox = IncludeLaunchDescription(
      PythonLaunchDescriptionSource([os.path.join(
         get_package_share_directory('slam_toolbox'), 'launch'),
         '/online_async_launch.py'])
    )

    # Rviz node
    rviz_config_file = os.path.join(get_package_share_directory(pkg_name), 'rviz', 'nav2.rviz')
    node_rviz = Node(
        package='rviz2',
        namespace='',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', [rviz_config_file]]
    )

    # map_saver_node = Node(
    #     package='simulation_package',
    #     executable='periodic_map_saver',
    #     name='periodic_map_saver',
    # )

    # Add actions to LaunchDescription
    # Simulation
    ld.add_action(SetParameter(name='use_sim_time', value=True))
    #ld.add_action(launch_sim_world)
    # SLAM Toolbox
    ld.add_action(launch_slam_toolbox)
    # Visualisation
    ld.add_action(node_rviz)

    #ld.add_action(map_saver_node)

    return ld