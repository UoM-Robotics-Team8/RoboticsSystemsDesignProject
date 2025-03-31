from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import SetParameter, Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
import xacro
import os


def generate_launch_description():
    ld = LaunchDescription()

    # Parameters, Nodes and Launch files go here
     # Specify the name of the package and path to xacro file in external package
    pkg_name = 'hardware_package'
    file_subpath = 'urdf/leo.urdf.xacro'

    # Use xacro to process the file
    xacro_file = os.path.join(get_package_share_directory(pkg_name),file_subpath)
    robot_description_raw = xacro.process_file(xacro_file).toxml()

    # Declare package directory
    pkg_nav_demos = get_package_share_directory('hardware_package')
    # Necessary fixes
    remappings = [('/tf', 'tf'), ('/tf_static', 'tf_static')]
    
    # lifecycle_nodes = [
    #     'controller_server',
    #     'planner_server',
    #     'behaviour_server',
    #     'bt_navigator',
    # ]


    # LOAD PARAMETERS FROM YAML FILES
    config_bt_nav     = PathJoinSubstitution([pkg_nav_demos, 'config', 'bt_nav.yaml'])
    config_planner    = PathJoinSubstitution([pkg_nav_demos, 'config', 'planner.yaml'])
    config_controller = PathJoinSubstitution([pkg_nav_demos, 'config', 'controller.yaml'])

    # # Behaviour Tree Navigator
    # node_bt_nav = Node(
    #     package='nav2_bt_navigator',
    #     executable='bt_navigator',
    #     name='bt_navigator',
    #     output='screen',
    #     parameters=[config_bt_nav],
    #     remappings=remappings,
    # )

    # # Behaviour Tree Server
    # node_behaviour = Node(
    #     package='nav2_behaviors',
    #     executable='behavior_server',
    #     name='behaviour_server',
    #     output='screen',
    #     parameters=[config_bt_nav],
    #     remappings=remappings,
    # )
    
    # # Planner Server Node
    # node_planner = Node(
    #     package='nav2_planner',
    #     executable='planner_server',
    #     name='planner_server',
    #     output='screen',
    #     parameters=[config_planner],
    #     remappings=remappings,
    # )
    
    # # Controller Server Node
    # node_controller = Node(
    #     package='nav2_controller',
    #     executable='controller_server',
    #     name='controller_server',
    #     output='screen',
    #     parameters=[config_controller],
    #     remappings=remappings,
    # )

    # # Lifecycle Node Manager to automatically start lifecycles nodes (from list)
    # node_lifecycle_manager = Node(
    #     package='nav2_lifecycle_manager',
    #     executable='lifecycle_manager',
    #     name='lifecycle_manager_navigation',
    #     output='screen',
    #     parameters=[{'autostart': True}, {'node_names': lifecycle_nodes}],
    # )
    
    nav_node = Node(
        package='nav2_bringup',
        executable='bringup_launch',
        name='nav2_bringup',
        output='screen',
        parameters=[
            config_bt_nav,
            config_controller,
            config_planner
        ]
    )

    # robot state publisher node
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_raw}] # add other parameters here if required
    )

    # Add actions to LaunchDescription
    # ld.add_action(node_bt_nav)
    # ld.add_action(node_behaviour)
    # ld.add_action(node_planner)
    # ld.add_action(node_controller)
    # ld.add_action(node_lifecycle_manager)
    ld.add_action(nav_node)
    ld.add_action(node_robot_state_publisher)
    
    return ld
    