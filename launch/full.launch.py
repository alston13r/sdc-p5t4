import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch import LaunchService



def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    
    urdf_path = '/home/wolfwagen/ros2_ws/src/full_slam/car.urdf.xacro'
    robot_description = ParameterValue(Command(['xacro ', urdf_path]), value_type=str)

    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen'
    )

    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
    )


    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description},
                    {'use_sim_time': use_sim_time}],
        output='screen',
    )

    sllidar_ros2 = Node(
        package='sllidar_ros2',
        executable="sllidar_node",
        parameters=[
            {'use_sim_time': use_sim_time},
            {'frame_id': 'lidar_scan_frame'},
        ],
        output="screen"
    )

    # Compatibility bridge in case the lidar driver still publishes laser_frame.
    lidar_frame_compat_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='lidar_frame_compat_tf',
        arguments=['0', '0', '0', '0', '0', '0', 'lidar_scan_frame', 'laser_frame'],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
    )

    # Bridge ZED odom chain to the car base chain.
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
            'config_path': '/home/wolfwagen/ros2_ws/src/full_slam',
            'config_file': 'zed_config.yaml',
        }.items(),
    )

    slam_toolbox_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('slam_toolbox'), 'launch', 'online_async_launch.py'])
        ),
        launch_arguments={
            'slam_params_file': '/home/wolfwagen/ros2_ws/src/full_slam/slam_params.yaml',
        }.items(),
    )


    '''cmd_converter = Node(
        package='project5_avrodri3_ackermann',
        executable='cmd_vel_to_ackermann',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )'''

    return LaunchDescription([
        # On real hardware, keep this false unless a /clock source is running.
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        rviz2,
        joint_state_publisher,
        robot_state_publisher,
        sllidar_ros2,
        lidar_frame_compat_tf,
        zed_to_base_tf,
        zed_camera_launch,
        slam_toolbox_launch,
    ])


    
def main():
    ld = generate_launch_description()

    ls = LaunchService()
    ls.include_launch_description(ld)
    return ls.run()

if __name__ == '__main__':
    main()