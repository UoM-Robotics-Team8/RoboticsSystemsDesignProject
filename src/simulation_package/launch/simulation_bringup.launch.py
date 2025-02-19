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
    pkg_name = 'simulation_package'
    file_subpath = 'urdf/leo_rover.urdf.xacro'

    # Set ignition resource path (to be able to render meshes)
    resource_paths = [os.path.join(get_package_prefix(pkg_name), 'share')]

    # Add any existing declared resources
    if 'IGN_GAZEBO_RESOURCE_PATH' in os.environ:
        resource_paths.insert(0, os.environ['IGN_GAZEBO_RESOURCE_PATH'])

    # Concatenate a path seperator (':') between all paths and update the environment variable
    ign_resource_path_update = SetEnvironmentVariable(name='IGN_GAZEBO_RESOURCE_PATH',value=[os.pathsep.join(resource_paths)])


    # Use xacro to process the URDF file
    xacro_file = os.path.join(get_package_share_directory(pkg_name),file_subpath)
    robot_description_raw = xacro.process_file(xacro_file).toxml()

    # Include the Gazebo launch file
    launch_gazebo = IncludeLaunchDescription(
    PythonLaunchDescriptionSource([get_package_share_directory('ros_gz_sim'), '/launch', '/gz_sim.launch.py']),
    launch_arguments={
    'gz_args' : '-r empty.sdf'
    }.items(),
    )

    # Include extra models in the world
    sdf_path = os.path.join(get_package_share_directory('simulation_package'), 'worlds', 'maze.sdf')
    # Spawn extra models (not robots) into the empty world
    gz_spawn_objects = Node(package='ros_gz_sim', executable='create',
    arguments=['-file', sdf_path,
    '-x', '-10',
    '-y', '-10',
    '-z', '0.0'],
    output='screen'
    )
 
    # robot state publisher node
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_raw}] # add other parameters here if required
    )

    # Run the spawner node from the gazebo_ros package. The entity name doesn't really matter if you only have a single robot.
    node_spawn_entity = Node(package='ros_gz_sim', executable='create',
                        arguments=['-topic', '/robot_description',
                                   '-z', '0.5'],
                        output='screen')
    
    # Bridge between Gazebo and ROS2 topics
    # https://github.com/gazebosim/ros_gz/tree/humble/ros_gz_bridge
    node_ros_gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=  [
            "/clock" + "@rosgraph_msgs/msg/Clock" + "[" + "ignition.msgs.Clock",
            "/model/leo_rover/odometry" + "@nav_msgs/msg/Odometry" + "[" + "ignition.msgs.Odometry",
            "/model/leo_rover/scan" + "@sensor_msgs/msg/LaserScan" + "[" + "ignition.msgs.LaserScan",
            "/model/leo_rover/tf" + "@tf2_msgs/msg/TFMessage" + "[" + "ignition.msgs.Pose_V",
            "/model/leo_rover/imu" + "@sensor_msgs/msg/Imu" + "[" + "ignition.msgs.IMU",
            "/model/leo_rover/cmd_vel" + "@geometry_msgs/msg/Twist" + "@" + "ignition.msgs.Twist",
            "/model/leo_rover/camera/image_raw" + "@sensor_msgs/msg/Image" + "[" + "ignition.msgs.Image",
            "/model/leo_rover/joint_state" + "@sensor_msgs/msg/JointState" + "[" + "ignition.msgs.Model",
        ],
        parameters=[{'qos_overrides./leo_rover/subscriber/reliability': 'reliable'}],
        remappings=[
            ('/model/leo_rover/odometry', '/odom'),
            ('/model/leo_rover/scan', '/scan'),
            ('/model/leo_rover/tf', '/tf'),
            ('/model/leo_rover/imu', '/imu/data_raw'),
            ('/model/leo_rover/cmd_vel', '/cmd_vel'),
            ('/model/leo_rover/camera/image_raw', '/camera'),
            ('/model/leo_rover/joint_state', '/joint_states'),
        ],
        output="screen",
    )

    ld.add_action(SetParameter(name='use_sim_time', value=True))
    ld.add_action(ign_resource_path_update)
    ld.add_action(launch_gazebo)
    ld.add_action(gz_spawn_objects)
    ld.add_action(node_robot_state_publisher)
    ld.add_action(node_spawn_entity)
    ld.add_action(node_ros_gz_bridge)

    return ld

