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
    # Necessary fixes
    remappings = [('/tf', 'tf'), ('/tf_static', 'tf_static')]

    lifecycle_nodes = [
        'planner_server',
        'behaviour_server',
        'bt_navigator',
    ]


    # LOAD PARAMETERS FROM YAML FILES
    config_bt_nav = PathJoinSubstitution([pkg_name, 'config', 'bt_nav.yaml'])
    config_planner = PathJoinSubstitution([pkg_name, 'config', 'planner.yaml'])

    # Behaviour Tree Navigator
    node_bt_nav = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[config_bt_nav],
        remappings=remappings,
    )

    # Behaviour Tree Server
    node_behaviour = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behaviour_server',
        output='screen',
        parameters=[config_bt_nav],
        remappings=remappings,
    )

    node_planner = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[config_planner],
        remappings=remappings
    )

    # Lifecycle Node Manager to automatically start lifecycles nodes (from list)

    node_lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[{'autostart': True}, {'node_names': lifecycle_nodes}],
    )


    # Add actions to LaunchDescription
    ld.add_action(SetParameter(name='use_sim_time', value=True))
    ld.add_action(node_bt_nav)
    ld.add_action(node_behaviour)
    ld.add_action(node_planner)
    ld.add_action(node_lifecycle_manager)

    return ld