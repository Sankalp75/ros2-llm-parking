import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    """Launch the complete Ackermann LLM parking demo."""
    
    # Package paths
    pkg_vehicle_gazebo = get_package_share_directory('vehicle_gazebo')
    
    # Launch configurations
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    ollama_url = LaunchConfiguration('ollama_url', default='http://localhost:11434')
    model_name = LaunchConfiguration('model_name', default='qwen2.5-vl:7b')
    
    return LaunchDescription([
        # Arguments
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation time'
        ),
        DeclareLaunchArgument(
            'ollama_url',
            default_value='http://localhost:11434',
            description='Ollama API URL'
        ),
        DeclareLaunchArgument(
            'model_name',
            default_value='qwen2.5-vl:7b',
            description='LLM model name in Ollama'
        ),
        
        # Gazebo simulation with vehicle
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_vehicle_gazebo, 'launch', 'spawn_vehicle.launch.py')
            ),
        ),
        
        # Parking controller (delayed start to wait for Gazebo)
        TimerAction(
            period=5.0,
            actions=[
                Node(
                    package='parking_controller',
                    executable='parking_controller_node',
                    name='parking_controller',
                    output='screen',
                    parameters=[{
                        'use_sim_time': use_sim_time,
                        'linear_speed': 2.0,
                        'position_tolerance': 0.5,
                        'angle_tolerance': 0.1,
                    }]
                ),
            ]
        ),
        
        # LLM Agent (delayed start)
        TimerAction(
            period=7.0,
            actions=[
                Node(
                    package='llm_agent',
                    executable='llm_agent_node',
                    name='llm_agent',
                    output='screen',
                    parameters=[{
                        'use_sim_time': use_sim_time,
                        'ollama_url': ollama_url,
                        'model_name': model_name,
                        'use_vision': False,
                    }]
                ),
            ]
        ),
    ])
