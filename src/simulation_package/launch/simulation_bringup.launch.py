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
    file_subpath = 'urdf/leo_sim.urdf.xacro'

    # Set ignition resource path (so it can find your world files)
    ign_resource_path = SetEnvironmentVariable(name='IGN_GAZEBO_RESOURCE_PATH',
    value=[os.path.join(get_package_share_directory(pkg_name),'worlds')])

    # Use xacro to process the file
    xacro_file = os.path.join(get_package_share_directory(pkg_name),file_subpath)
    robot_description_raw = xacro.process_file(xacro_file).toxml()


    # Include extra models in the world
    sdf_path = os.path.join(get_package_share_directory(pkg_name), 'worlds', 'tb3.sdf')

    # if 'IGN_GAZEBO_RESOURCE_PATH' in os.environ:
    #     gz_world_path =  os.environ['IGN_GAZEBO_RESOURCE_PATH'] + os.pathsep + os.path.join(get_package_share_directory(pkg_name), "worlds")
    # else:
    #     gz_world_path =  os.path.join(get_package_share_directory(pkg_name), "worlds")

    # ign_resource_path_update = SetEnvironmentVariable(name='IGN_GAZEBO_RESOURCE_PATH',value=[gz_world_path])

    # Get paths
    package_path = get_package_share_directory(pkg_name)
    gz_world_path = os.path.join(package_path, "worlds")
    mesh_path = os.path.join(package_path, "meshes")

    # gz_world_path = "/home/delewlew/RoboticsSystemsDesignProject/src/simulation_package/worlds"
    # mesh_path = "/home/delewlew/RoboticsSystemsDesignProject/src/simulation_package/meshes"


    # Update IGN_GAZEBO_RESOURCE_PATH to include both worlds and meshes
    if 'IGN_GAZEBO_RESOURCE_PATH' in os.environ:
        gz_resource_path = os.environ['IGN_GAZEBO_RESOURCE_PATH'] + os.pathsep + gz_world_path + os.pathsep + mesh_path
    else:
        gz_resource_path = gz_world_path + os.pathsep + mesh_path

    # Set environment variable
    ign_resource_path_update = SetEnvironmentVariable(name='IGN_GAZEBO_RESOURCE_PATH', value=gz_resource_path)

    # Include the Gazebo launch file
    launch_gazebo = IncludeLaunchDescription(
    PythonLaunchDescriptionSource([get_package_share_directory('ros_gz_sim'), '/launch', '/gz_sim.launch.py']),
    launch_arguments={
    'gz_args' : '-r empty.sdf'
    }.items(),
    )

    # Add features
    gz_spawn_objects = Node(package='ros_gz_sim', executable='create',
    arguments=['-file', sdf_path,
    '-x', '2.0',
    '-y', '0.5',
    '-z', '0.0'],
    output='screen'
    )
 
    # Run the spawner node from the gazebo_ros package. The entity name doesn't really matter if you only have a single robot.
    node_spawn_entity = Node(package='ros_gz_sim', executable='create',
                        arguments=['-topic', '/robot_description',
                                   '-z', '0.5'],
                        output='screen')
 
    # robot state publisher node
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_raw}] # add other parameters here if required
    )
    
    # Bridge between Gazebo and ROS2 topics
    # https://github.com/gazebosim/ros_gz/tree/humble/ros_gz_bridge
    node_ros_gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=  [
            "/clock" + "@rosgraph_msgs/msg/Clock" + "[" + "ignition.msgs.Clock",
            "/model/leo_sim/odometry" + "@nav_msgs/msg/Odometry" + "[" + "ignition.msgs.Odometry",
            "/model/leo_sim/scan" + "@sensor_msgs/msg/LaserScan" + "[" + "ignition.msgs.LaserScan",
            "/model/leo_sim/tf" + "@tf2_msgs/msg/TFMessage" + "[" + "ignition.msgs.Pose_V",
            "/model/leo_sim/imu" + "@sensor_msgs/msg/Imu" + "[" + "ignition.msgs.IMU",
            "/model/leo_sim/cmd_vel" + "@geometry_msgs/msg/Twist" + "@" + "ignition.msgs.Twist",
            #"/model/leo_sim/camera/image_raw" + "@sensor_msgs/msg/Image" + "[" + "ignition.msgs.Image",
            "/model/leo_sim/joint_state" + "@sensor_msgs/msg/JointState" + "[" + "ignition.msgs.Model",
            "/model/leo_sim/depth_camera/points" + "@sensor_msgs/msg/PointCloud2" + "[" + "ignition.msgs.PointCloudPacked",
            "/model/leo_sim/depth_camera" + "@sensor_msgs/msg/Image" + "[" + "ignition.msgs.Image",
            "/model/leo_sim/camera_info" + "@sensor_msgs/msg/CameraInfo" + "@" + 'ignition.msgs.CameraInfo',
        ],
        parameters=[{'qos_overrides./leo_sim/subscriber/reliability': 'reliable'}],
        remappings=[
            ('/model/leo_sim/odometry', '/odom'),
            ('/model/leo_sim/scan', '/scan'),
            ('/model/leo_sim/tf', '/tf'),
            ('/model/leo_sim/imu', '/imu/data_raw'),
            ('/model/leo_sim/cmd_vel', '/cmd_vel'),
            #('/model/leo_sim/camera/image_raw', '/camera'),
            ('/model/leo_sim/joint_state', '/joint_states'),
            ('/model/leo_sim/depth_camera/points', '/camera/camera/depth/color/points'),
            ('/model/leo_sim/depth_camera', '/camera/camera/depth/camera'),
            ('/model/leo_sim/camera_info', '/camera/camera/depth/camera_info')
        ],
        output="screen",
    )

    depth_cam_data2cam_link_tf = Node(package='tf2_ros',
                     executable='static_transform_publisher',
                     name='cam3Tolink',
                     output='log',
                     arguments=['0.0', '0.0', '0.0', '0.0', '0.0', '0.0', 'camera_camera_link', 'leo_sim/base_footprint/depth_sensor'])


    ld.add_action(SetParameter(name='use_sim_time', value=True))
    ld.add_action(ign_resource_path)
    ld.add_action(ign_resource_path_update)
    ld.add_action(launch_gazebo)
    ld.add_action(gz_spawn_objects)
    ld.add_action(node_robot_state_publisher)
    ld.add_action(node_spawn_entity)
    ld.add_action(node_ros_gz_bridge)
    ld.add_action(depth_cam_data2cam_link_tf)

    return ld

