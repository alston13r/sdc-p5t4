import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')

    pkg_share = get_package_share_directory('p5t4')
    urdf_path = os.path.join(pkg_share, 'urdf', 'car.urdf.xacro')
    controllers_file = os.path.join(pkg_share, 'config', 'controllers.yaml')
    rviz_config_file = os.path.join(pkg_share, 'config', 'rviz_config.rviz')
    zed_config_dir = os.path.join(pkg_share, 'config')
    slam_params_path = os.path.join(pkg_share, 'config', 'slam_params.yaml')

    robot_description = ParameterValue(Command(['xacro ', urdf_path]), value_type=str)

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description, 'use_sim_time': use_sim_time}],
        output='screen',
    )

    control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[
            {'robot_description': robot_description, 'use_sim_time': use_sim_time},
            controllers_file,
        ],
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
        package='p5t4',
        executable='cmd_vel_to_ackermann',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
    )

    sllidar_ros2 = Node(
        package='sllidar_ros2',
        executable='sllidar_node',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'frame_id': 'lidar_scan_frame'},
            {'angle_compensate': True},
        ],
        output='screen',
    )

    lidar_frame_compat_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='lidar_frame_compat_tf',
        arguments=['0', '0', '0', '0', '0', '0', 'lidar_scan_frame', 'laser_frame'],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
    )

    zed_to_base_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='zed_to_base_tf',
        arguments=['0', '0', '0', '3.14159265359', '0', '0', 'zed_camera_link', 'base_footprint'],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
    )

    zed_camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('zed_wrapper'), 'launch', 'zed_camera.launch.py'])
        ),
        launch_arguments={
            'camera_model': 'zed2i',
            'config_path': zed_config_dir,
            'config_file': 'zed_config.yaml',
        }.items(),
    )

    slam_toolbox_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('slam_toolbox'), 'launch', 'online_async_launch.py'])
        ),
        launch_arguments={
            'slam_params_file': slam_params_path,
        }.items(),
    )

    return LaunchDescription([
        # Keep false on hardware unless a /clock source exists.
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        robot_state_publisher,
        control_node,
        joint_state_spawner,
        steering_spawner,
        traction_spawner,
        cmd_converter,
        rviz2,
        sllidar_ros2,
        lidar_frame_compat_tf,
        zed_to_base_tf,
        zed_camera_launch,
        slam_toolbox_launch,
    ])