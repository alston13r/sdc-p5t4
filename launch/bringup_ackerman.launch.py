from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')

    pkg_share = FindPackageShare('project5_avrodri3_ackermann')
    xacro_file = PathJoinSubstitution([pkg_share, 'urdf', 'ackermann_robot.urdf.xacro'])
    controllers_file = PathJoinSubstitution([pkg_share, 'config', 'controllers.yaml'])

    robot_description = ParameterValue(Command(['xacro ', xacro_file]), value_type=str)

    control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[
            {'robot_description': robot_description, 'use_sim_time': use_sim_time},
            controllers_file,
        ],
        output='screen',
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description, 'use_sim_time': use_sim_time}],
        output='screen',
    )

    joint_state_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
        output='screen',
    )

    steering_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['steering_position_controller', '--controller-manager', '/controller_manager'],
        output='screen',
    )

    traction_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['traction_velocity_controller', '--controller-manager', '/controller_manager'],
        output='screen',
    )

    cmd_converter = Node(
        package='project5_avrodri3_ackermann',
        executable='cmd_vel_to_ackermann',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        robot_state_publisher,
        control_node,
        joint_state_spawner,
        steering_spawner,
        traction_spawner,
        cmd_converter,
    ])