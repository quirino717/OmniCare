from ament_index_python.packages import get_package_share_path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

import subprocess
import platform

def launch_serial_interface(context, *args, **kwargs):
    serial_interface_pkg_dir = get_package_share_path('serial_interface_pkg')
    robot_part = LaunchConfiguration("robot_part").perform(context)

    default_config_path = (serial_interface_pkg_dir / "config")
    node_namespace = None

    if robot_part == "base":
        print("Launching BASE part...")
        default_config_path = default_config_path / "serial_base_config.yaml"
        node_namespace = "base"
    
    elif robot_part == "manip":
        print("Launching MANIPULATOR part...")
        default_config_path = default_config_path / "serial_manip_config.yaml"
        node_namespace = "manip"

    else:
        print(f"Unknown robot_part: {robot_part}")
        raise ValueError(f"Unknown robot_part: {robot_part}")

    return [
        Node(
            package = "serial_interface_pkg",
            executable = "serial_interface",
            output = "screen",
            parameters = [default_config_path],
            namespace = node_namespace 
        )
    ]

def generate_launch_description():

    log_level = DeclareLaunchArgument(
        name='log_level', 
        default_value='INFO', 
        choices=['DEBUG','INFO','WARN','ERROR','FATAL'],
        description='Flag to set log level'
    )
        
    robot_part_arg = DeclareLaunchArgument(
        name = "robot_part", 
        default_value = "base",
        choices = ["base", "manip"],
        description =
            "Select which part of the robot to use: base or manipulator"
    )
    

    return LaunchDescription([
        log_level,
        robot_part_arg,
        OpaqueFunction(function=launch_serial_interface)
    ])