from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.conditions import IfCondition, UnlessCondition
from launch.actions import DeclareLaunchArgument, TimerAction, RegisterEventHandler
from launch.event_handlers import  OnProcessStart


def generate_launch_description():

    use_sim_time_arg = DeclareLaunchArgument(name='use_sim_time', default_value='false',
                                        description='Flag to enable simulation')
    
    params_file_arg = DeclareLaunchArgument(name='params_file', default_value= get_package_share_directory('navigation_pkg') + '/config/nav/sim_nav2_params.yaml',
                                        description='Flag to change the path')
    
# -----------------------------------------------------

    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([get_package_share_directory('omnicare_simulation'), '/launch/simulation.launch.py']),
           launch_arguments={
                'world_path': [get_package_share_directory('omnicare_simulation'), '/simulation/worlds/simple_room_with_fixed_boxes.world'],
            }.items(),
            condition=IfCondition(LaunchConfiguration('use_sim_time'))
    )



# -----------------------------------------------------


    load_sim_robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([get_package_share_directory('omnicare_bringup'),'/launch/load_sim_robot.launch.py']),
        launch_arguments={
            'rvizconfig': [get_package_share_directory('navigation_pkg'), '/config/rviz/navigation.rviz'],
        }.items(),
        condition=IfCondition(LaunchConfiguration('use_sim_time'))
    )

    load_real_robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([get_package_share_directory('omnicare_bringup'),'/launch/load_real_robot.launch.py']),
        launch_arguments={
            'rvizconfig': [get_package_share_directory('navigation_pkg'), '/config/rviz/navigation.rviz'],
        }.items(),
        condition=UnlessCondition(LaunchConfiguration('use_sim_time'))
    )




# -----------------------------------------------------

    bringup_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([get_package_share_directory('nav2_bringup'),'/launch/bringup_launch.py']),
        launch_arguments={
            # 'namespace': LaunchConfiguration('namespace'),
            'use_namespace': 'False',
            'slam': 'False',
            'map': [get_package_share_directory('navigation_pkg'),'/config/map/maps/quintoAndarElevador.yaml'],
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            # 'params_file': [get_package_share_directory('nav2_bringup'),'/params/nav2_params.yaml'],
            'params_file': [LaunchConfiguration('params_file')],
            'autostart': 'True',
            'use_composition': 'True',
            'use_respawn': 'False'
        }.items()
    )

    delayed_nav2 = TimerAction(period=10.0, actions=[bringup_cmd])

    checkpoint = Node(
        package='navigation_pkg',
        executable='checkpoints',
        name='checkpoints',
        parameters=[{
            'checkpoints_file': get_package_share_directory('navigation_pkg')+'/config/map/checkpoints/quinto_andar_checkpoints.json'
        }],
        # arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
    )

    switchFloor = Node(
        package='navigation_pkg',
        executable='switchFloorService',
        name='switchFloorService',
        output='screen',
        # arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
    )


# -----------------------------------------------------

    ld = LaunchDescription()
    ld.add_action(use_sim_time_arg)
    ld.add_action(params_file_arg)
    ld.add_action(simulation)
    ld.add_action(load_sim_robot)
    ld.add_action(load_real_robot)
    ld.add_action(delayed_nav2)
    ld.add_action(checkpoint)
    ld.add_action(switchFloor)

    return ld
