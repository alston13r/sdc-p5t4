from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        output='screen',
        parameters=[{'autorepeat_rate': 0.0}],
    )

    xbox_controller_node = Node(
        package='p5t4',
        executable='xbox_controller',
        name='xbox_controller_node',
        output='screen',
    )

    driver_node = Node(
        package='p5t4',
        executable='driver',
        name='driver',
        output='screen',
    )

    return LaunchDescription([
        joy_node,
        xbox_controller_node,
        driver_node,
    ])