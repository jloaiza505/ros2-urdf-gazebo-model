from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
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

    rviz_config = PathJoinSubstitution([
        FindPackageShare('robot_sim_pkg'),
        'rviz',
        'demo.rviz'
    ])

    use_sim_time = LaunchConfiguration('use_sim_time')

    robot_description = ParameterValue(Command(['xacro ', xacro_path]), value_type=str)

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[
            {'robot_description': robot_description},
            {'use_sim_time': use_sim_time}
        ]
    )

    robot_sim_node = Node(
        package='robot_sim_pkg',
        executable='robot_sim_node',
        name='robot_sim_node'
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': use_sim_time}]
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        robot_state_publisher_node,
        robot_sim_node,
        rviz_node,
    ])
