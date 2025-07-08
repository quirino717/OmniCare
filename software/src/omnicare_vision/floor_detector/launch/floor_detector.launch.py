from ament_index_python.packages import get_package_share_path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

import subprocess
import platform

def generate_launch_description():
    floor_detector_dir = get_package_share_path('floor_detector')

    log_level = DeclareLaunchArgument(
        name='log_level', 
        default_value='INFO', 
        choices=['DEBUG','INFO','WARN','ERROR','FATAL'],
        description='Flag to set log level'
    )
    
    floor_detector_node = Node(
        package='floor_detector',
        executable='floor_detector_node',
        output='screen',
        parameters=[{
            'model_display': f'{floor_detector_dir}/weights/best_display',
            'model_floor': f'{floor_detector_dir}/weights/best_floor'
        }],
        arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')]
    )

    return LaunchDescription([
        log_level,
        floor_detector_node
    ])