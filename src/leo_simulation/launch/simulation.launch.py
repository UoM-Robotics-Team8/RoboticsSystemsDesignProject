import os
from ament_index_python.packages import get_package_share_directory, get_package_prefix
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch_ros.actions import Node, SetParameter
from launch.launch_description_sources import PythonLaunchDescriptionSource
import xacro
import os

def generate_launch_description():
    """
    Launches Gazebo to simulate the Leo Rover.
    """
    ld = LaunchDescription()

    pkg_name = "leo_simulation"
    file_subpath = "urdf/leo_sim.urdf.xacro"

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


    # Bridge
    # https://github.com/gazebosim/ros_gz/tree/humble/ros_gz_bridge
    node_ros_gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=  [
            "/clock" + "@rosgraph_msgs/msg/Clock" + "[" + "ignition.msgs.Clock",
            "leo_rover/odometry" + "@nav_msgs/msg/Odometry" + "[" + "ignition.msgs.Odometry",
            "leo_rover/scan" + "@sensor_msgs/msgs/LaserScan" + "[" + "ignition.msgs.LaserScan",
            "leo_rover/imu/data_raw" + "@sensor_msgs/msg/Imu" + "[" + "ignition.msgs.IMU",
            "leo_rover/cmd_vel" + "@geometry_msgs/msg/Twist" + "]" + "ignition.msgs.Twist",
            "leo_rover/camera/camera_info" + "@sensor_msgs/msg/CameraInfo" + "[" + "ignition.msgs.CameraInfo",
            "leo_rover/joint_state" + "@sensor_msgs/msgs/JointState" + "[" + "ignition.msgs.Model"
        ],
        parameters=[{'qos_overrides./tf_static.publisher.durability": "transient_local'}],
        remappings=[
            ('/leo_rover/cmd_vel', '/cmd_vel'),
            ('/leo_rover/odometry', '/odom'),
            ('/leo_rover/scan', '/scan'),
            ('/leo_rover/tf', '/tf'),
            ('/leo_rover/imu', '/imu_raw'),
            ('/leo_rover/joint_state', 'joint_states')
        ],
        output="screen",
    )

    ld.add_action(SetParameter(name='use_sim_time', value=True))
    ld.add_action(ign_resource_path_update)
    ld.add_action(launch_gazebo)
    ld.add_action(node_robot_state_publisher)
    ld.add_action(node_spawn_entity)
    ld.add_action(node_ros_gz_bridge)

    return ld