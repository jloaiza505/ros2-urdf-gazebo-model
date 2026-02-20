from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    xacro_path = PathJoinSubstitution([
        FindPackageShare('robot_description_pkg'),
        'urdf',
        'two_link_arm.urdf.xacro'
    ])

    robot_description = ParameterValue(Command(['xacro ', xacro_path]), value_type=str)
    world_name = LaunchConfiguration('world_name')

    gazebo_server_only = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([FindPackageShare('ros_gz_sim'), 'launch', 'gz_sim.launch.py'])
        ]),
        launch_arguments={
            'gz_args': ['-r -s -v 4 ', world_name, '.sdf'],
        }.items(),
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description}, {'use_sim_time': True}],
    )

    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-world', world_name, '-name', 'two_link_arm', '-topic', 'robot_description'],
        output='screen',
    )

    delayed_spawn_entity = TimerAction(
        period=4.0,
        actions=[spawn_entity],
    )

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
        output='screen',
    )

    forward_position_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['forward_position_controller', '--controller-manager', '/controller_manager'],
        output='screen',
    )

    joint_command_node = Node(
        package='robot_sim_pkg',
        executable='joint_command_node',
        name='joint_command_node',
        output='screen',
    )

    spawn_controllers = RegisterEventHandler(
        OnProcessExit(
            target_action=spawn_entity,
            on_exit=[joint_state_broadcaster_spawner, forward_position_controller_spawner],
        )
    )

    start_command_node = RegisterEventHandler(
        OnProcessExit(
            target_action=forward_position_controller_spawner,
            on_exit=[joint_command_node],
        )
    )

    return LaunchDescription([
        DeclareLaunchArgument('world_name', default_value='empty'),
        gazebo_server_only,
        robot_state_publisher,
        delayed_spawn_entity,
        spawn_controllers,
        start_command_node,
    ])
