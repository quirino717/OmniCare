from ament_index_python.packages import get_package_share_path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    dataset_generator_dir = get_package_share_path('dataset_generator')
    default_config_path = dataset_generator_dir / 'config' / 'dataset_generator_config.yaml'

    log_level = DeclareLaunchArgument(
        name='log_level', 
        default_value='INFO', 
        choices=['DEBUG','INFO','WARN','ERROR','FATAL'],
        description='Flag to set log level'
    )
        
    config_path_arg = DeclareLaunchArgument(name='dataset_generator_config', default_value=str(default_config_path),
                                            description='Absolute path to floor detector config file')
    
        
    dataset_generator_node = Node(
        package='dataset_generator',
        executable='dataset_generator_node',
        output='screen',
        parameters=[LaunchConfiguration('dataset_generator_config')],
        arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],

    )

    return LaunchDescription([
        config_path_arg,
        log_level,
        dataset_generator_node
    ])