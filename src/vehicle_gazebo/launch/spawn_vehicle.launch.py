import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Package paths
    pkg_vehicle_description = get_package_share_directory('vehicle_description')
    pkg_vehicle_gazebo = get_package_share_directory('vehicle_gazebo')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    
    # Files
    urdf_file = os.path.join(pkg_vehicle_description, 'urdf', 'ackermann_vehicle.urdf.xacro')
    world_file = os.path.join(pkg_vehicle_gazebo, 'worlds', 'parking_lot.world')
    
    # Robot description
    robot_description = ParameterValue(
        Command(['xacro ', urdf_file]),
        value_type=str
    )
    
    # Launch configurations
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    x_pose = LaunchConfiguration('x_pose', default='0.0')
    y_pose = LaunchConfiguration('y_pose', default='0.0')
    z_pose = LaunchConfiguration('z_pose', default='0.5')
    yaw = LaunchConfiguration('yaw', default='0.0')
    
    return LaunchDescription([
        # Arguments
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock'
        ),
        DeclareLaunchArgument(
            'x_pose',
            default_value='0.0',
            description='Initial X position'
        ),
        DeclareLaunchArgument(
            'y_pose',
            default_value='0.0',
            description='Initial Y position'
        ),
        DeclareLaunchArgument(
            'z_pose',
            default_value='0.5',
            description='Initial Z position'
        ),
        DeclareLaunchArgument(
            'yaw',
            default_value='0.0',
            description='Initial yaw angle'
        ),
        
        # Gazebo Fortress (Ignition)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
            ),
            launch_arguments={'gz_args': f"{world_file} -r"}.items()
        ),
        
        # Robot state publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': robot_description,
                'use_sim_time': use_sim_time
            }]
        ),
        
        # Spawn robot in Ignition
        Node(
            package='ros_gz_sim',
            executable='create',
            name='spawn_entity',
            output='screen',
            arguments=[
                '-topic', 'robot_description',
                '-name', 'ackermann_vehicle',
                '-x', x_pose,
                '-y', y_pose,
                '-z', z_pose,
                '-Y', yaw
            ]
        ),
        
        # Bridge for topics
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='bridge',
            output='screen',
            arguments=[
                '/cmd_vel@geometry_msgs/msg/Twist@ignition.msgs.Twist',
                '/odom@nav_msgs/msg/Odometry@ignition.msgs.Odometry',
                '/scan@sensor_msgs/msg/LaserScan@ignition.msgs.LaserScan',
                '/tf@tf2_msgs/msg/TFMessage@ignition.msgs.Pose_V',
                '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock',
                '/camera/image_raw@sensor_msgs/msg/Image@ignition.msgs.Image',
                '/camera/camera_info@sensor_msgs/msg/CameraInfo@ignition.msgs.CameraInfo',
                '--ros-args', '-p', 'use_sim_time:=true'
            ],
            remappings=[
                ('/tf', 'tf'),
            ]
        ),
    ])
