import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_dir = get_package_share_directory('ground_gimbal_robot')
    xacro_file = os.path.join(package_dir, 'urdf', 'ground_gimbal_robot.xacro')
    rviz_config = os.path.join(package_dir, 'config', 'ground_gimbal_robot.rviz')

    use_sim_time = LaunchConfiguration('use_sim_time')
    robot_description = {'robot_description': Command(['xacro ', xacro_file])}

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[robot_description, {'use_sim_time': use_sim_time}],
            output='screen',
        ),
        Node(
            package='ground_gimbal_robot',
            executable='gimbal_demo',
            output='screen',
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', rviz_config],
            output='screen',
        ),
    ])
