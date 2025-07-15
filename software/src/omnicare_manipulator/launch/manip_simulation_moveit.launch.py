# import os
# from launch import LaunchDescription
# from launch.actions import DeclareLaunchArgument
# from launch.substitutions import LaunchConfiguration
# from launch.conditions import IfCondition, UnlessCondition
# from launch_ros.actions import Node
# from launch.actions import ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler
# from launch.launch_description_sources import PythonLaunchDescriptionSource
# from launch.event_handlers import OnProcessExit
# from ament_index_python.packages import get_package_share_directory
# from moveit_configs_utils import MoveItConfigsBuilder

# def generate_launch_description():
#     moveit_config = (
#         MoveItConfigsBuilder("custom_robot", package_name='manip_moveit_config')
#         .robot_description(file_path="config/manipulator.urdf.xacro")
#         .robot_description_semantic(file_path="config/manipulator.srdf")
#         .trajectory_execution(file_path="config/moveit_controllers.yaml")
#         .planning_pipelines(
#             pipelines=["ompl", "chomp", "pilz_industrial_motion_planner"]
#         )
#         .to_moveit_configs()
#     )
    
#     ros2_controllers_path = os.path.join(
#         get_package_share_directory("omnicare_manipulator"),
#         "config",
#         "manipulator_controller.yaml",
#     )
    
#     joint_state_broadcaster_spawner = Node(
#         package="controller_manager",
#         executable="spawner",
#         arguments=[
#             "joint_state_broadcaster",
#             "--controller-manager",
#             "/controller_manager",
#         ],
#     )
    
#     arm_controller_spawner = Node(
#         package="controller_manager",
#         executable="spawner",
#         arguments=["manip_joint_trajectory_controller",
#                    "-c",
#                    "/controller_manager"],
#     )
    
#     move_group_node = Node(
#         package="moveit_ros_move_group",
#         executable="move_group",
#         output="screen",
#         parameters=[moveit_config.to_dict()],
#         arguments=["--ros-args", "--log-level", "info"],
#     )
    
#     rviz_config_path = os.path.join(
#         get_package_share_directory("manip_moveit_config"),
#         "config",
#         "moveit.rviz")

#     rviz_node = Node(
#         package="rviz2",
#         executable="rviz2",
#         name="rviz2",
#         output="log",
#         arguments=["-d", rviz_config_path],
#         parameters=[
#             moveit_config.robot_description,
#             moveit_config.robot_description_semantic,
#             moveit_config.planning_pipelines,
#             moveit_config.robot_description_kinematics,
#         ]
#     )
    
#     robot_state_publisher = Node(
#         package="robot_state_publisher",
#         executable="robot_state_publisher",
#         name="robot_state_publisher",
#         output="both",
#         parameters=[moveit_config.robot_description],
#     )
    
#     gazebo = IncludeLaunchDescription(
#             PythonLaunchDescriptionSource([os.path.join(
#                 get_package_share_directory('gazebo_ros'), 'launch'), '/gazebo.launch.py']),
#             )
    
    
#     # Spawner no Gazebo
#     spawn_entity = Node(
#         package='gazebo_ros',
#         executable='spawn_entity.py',
#         arguments=['-topic', 'robot_description', '-entity', 'manipulator'],
#         output='screen'
#     )
    
#     return LaunchDescription([
        
        
#         gazebo,
#         spawn_entity,
#         RegisterEventHandler(
#             event_handler=OnProcessExit(
#                 target_action=spawn_entity,
#                 on_exit=[joint_state_broadcaster_spawner]
#             )
#         ),
#         RegisterEventHandler(
#             event_handler=OnProcessExit(
#                 target_action=joint_state_broadcaster_spawner,
#                 on_exit=[arm_controller_spawner]
#             )
#         ),
#         robot_state_publisher,
#         move_group_node,
#         rviz_node
#     ])

import os
from launch import LaunchDescription
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder
from launch.actions import IncludeLaunchDescription

def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder("custom_robot", package_name='manip_moveit_config')
        .robot_description(file_path="config/manipulator.urdf.xacro")
        .robot_description_semantic(file_path="config/manipulator.srdf")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_pipelines(
            pipelines=["ompl", "chomp", "pilz_industrial_motion_planner"]
        )
        .to_moveit_configs()
    )

    # Caminho para o arquivo de controladores
    ros2_controllers_path = os.path.join(
        get_package_share_directory("omnicare_manipulator"),
        "config",
        "manipulator_controller.yaml",
    )

    # Spawner do joint_state_broadcaster
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
        ],
        parameters=[{"use_sim_time": True}],
        output="screen",
    )

    # Spawner do controller de trajetória
    arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["manip_joint_trajectory_controller", "-c", "/controller_manager"],
        parameters=[{"use_sim_time": True}],
        output="screen",
    )

    # Nó do move_group (MoveIt)
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[moveit_config.to_dict(), {"use_sim_time": True}],
        arguments=["--ros-args", "--log-level", "info"],
    )

    # Caminho para configuração do RViz
    rviz_config_path = os.path.join(
        get_package_share_directory("manip_moveit_config"),
        "config",
        "moveit.rviz"
    )

    # Nó do RViz
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_path],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.planning_pipelines,
            moveit_config.robot_description_kinematics,
            {"use_sim_time": True},
        ]
    )

    # Publisher do estado do robô
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[moveit_config.robot_description, {"use_sim_time": True}],
    )

    # Gazebo
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('gazebo_ros'), 'launch'),
            '/gazebo.launch.py'
        ])
    )

    # Spawner do robô no Gazebo
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', 'manipulator'],
        parameters=[{"use_sim_time": True}],
        output='screen'
    )

    return LaunchDescription([
        gazebo,
        spawn_entity,

        # Spawner do joint_state_broadcaster depois que o robô estiver no Gazebo
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=spawn_entity,
                on_exit=[joint_state_broadcaster_spawner]
            )
        ),

        # Spawner do controller de trajetória após o joint_state_broadcaster
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=joint_state_broadcaster_spawner,
                on_exit=[arm_controller_spawner]
            )
        ),

        robot_state_publisher,
        move_group_node,
        rviz_node,
    ])
