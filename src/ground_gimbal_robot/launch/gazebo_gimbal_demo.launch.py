import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    package_dir = get_package_share_directory('ground_gimbal_robot')
    gazebo_launch = os.path.join(package_dir, 'launch', 'gazebo.launch.py')

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gazebo_launch),
            launch_arguments={'use_sim_time': 'true'}.items(),
        ),
        Node(
            package='ground_gimbal_robot',
            executable='gimbal_demo',
            parameters=[{'use_sim_time': True}],
            output='screen',
        ),
    ])
