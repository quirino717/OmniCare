from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node

import os
import xacro


def generate_launch_description():
    pkg_name = 'omnicare_manipulator'
    pkg_share = get_package_share_directory(pkg_name)
    
    xacro_file_path = os.path.join(pkg_share, 'urdf', 'manipulator.xacro')
    manip_description = xacro.process_file(xacro_file_path).toxml()
    
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': manip_description,
                     'use_sim_time': False}]
    )
    
    node_joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher'
)
    
    node_joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen'
    )
    
    
    return LaunchDescription([
        node_robot_state_publisher,
        node_joint_state_publisher_gui,
        # node_joint_state_publisher,
    ])
    
    
    
    