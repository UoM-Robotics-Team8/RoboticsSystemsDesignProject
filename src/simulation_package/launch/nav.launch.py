from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import SetParameter, Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    ld = LaunchDescription()

    # Parameters, Nodes and Launch files go here

    # Declare package directory
    pkg_name = get_package_share_directory('simulation_package')

        # LOAD PARAMETERS FROM YAML FILES
    config_bt_nav = PathJoinSubstitution([pkg_name, 'config', 'regulated_pure_pursuit.yaml'])

    map_yaml = PathJoinSubstitution([pkg_name, 'config', 'map.yaml'])

    # Necessary fixes
    launch_nav = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([get_package_share_directory('nav2_bringup'), '/launch',
                                                    '/bringup_launch.py']),
        launch_arguments={
            'params_file': config_bt_nav,
            'map': map_yaml}.items(),
    )

    # Add actions to LaunchDescription
    ld.add_action(SetParameter(name='use_sim_time', value=True))
    ld.add_action(launch_nav)

    return ld