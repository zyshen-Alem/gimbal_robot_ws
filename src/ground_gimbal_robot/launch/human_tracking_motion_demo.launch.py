import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    package_dir = get_package_share_directory('ground_gimbal_robot')
    tracking_launch = os.path.join(
        package_dir, 'launch', 'human_tracking_demo.launch.py'
    )

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(tracking_launch),
        ),
        TimerAction(
            period=5.0,
            actions=[
                Node(
                    package='ground_gimbal_robot',
                    executable='tracking_motion_demo',
                    name='tracking_motion_demo',
                    parameters=[{'use_sim_time': True}],
                    output='screen',
                ),
            ],
        ),
    ])
