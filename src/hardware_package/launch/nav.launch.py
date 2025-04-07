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
    pkg_name = get_package_share_directory('hardware_package')
    file_subpath = 'urdf/leo.urdf.xacro'

    # Use xacro to process the file
    xacro_file = os.path.join(pkg_name,file_subpath)
    robot_description_raw = xacro.process_file(xacro_file).toxml()

    # Declare package directory
    # Necessary fixes
    # remappings = [('/tf', 'tf'), ('/tf_static', 'tf_static')]
    
    # lifecycle_nodes = [
    #     'planner_server',
    #     'behaviour_server',
    #     'bt_navigator',
    #     'controller_server'
    # ]

    # Define nav_to_pose behaviour tree

    bt_xml_navtopose_file = PathJoinSubstitution([pkg_name, 'behaviour', 'navigate_through_poses_w_replanning_and_recovery.xml'])

    # LOAD PARAMETERS FROM YAML FILES
    config_bt_nav = PathJoinSubstitution([pkg_name, 'config', 'regulated_pure_pursuit.yaml'])

    map_yaml = PathJoinSubstitution([pkg_name, 'config', 'map.yaml'])

    # # Behaviour Tree Navigator

    # node_bt_nav = Node(

    #     package='nav2_bt_navigator',
    #     executable='bt_navigator',
    #     name='bt_navigator',
    #     output='screen',
    #     parameters=[config_bt_nav,{'default_nav_to_pose_bt_xml' : bt_xml_navtopose_file}],
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

    # node_planner = Node(
    #     package='nav2_planner',
    #     executable='planner_server',
    #     name='planner_server',
    #     output='screen',
    #     parameters=[config_planner],
    #     remappings=remappings
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

    #nav2_bringup_node = PythonLaunchDescriptionSource(get_package_share_directory('nav2_bringup'), '/launch', '/bringup_launch.py')

    launch_nav = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([get_package_share_directory('nav2_bringup'), '/launch',
                                                    '/bringup_launch.py']),
        launch_arguments={
            'params_file': config_bt_nav,
            'map': map_yaml}.items(),
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
    ld.add_action(launch_nav)
    ld.add_action(node_robot_state_publisher)
    
    return ld
    