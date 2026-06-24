from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ground_gimbal_robot',
            executable='motion_demo',
            output='screen',
        ),
    ])
