import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    package_dir = get_package_share_directory('ground_gimbal_robot')
    cinematic_launch = os.path.join(package_dir, 'launch', 'cinematic_tracking_demo.launch.py')
    obstacle_world = os.path.join(package_dir, 'worlds', 'human_tracking_obstacles.world')

    mode = LaunchConfiguration('mode')
    move_target = LaunchConfiguration('move_target')

    return LaunchDescription([
        DeclareLaunchArgument(
            'mode',
            default_value='orbit',
            description='Obstacle demo mode: orbit, follow_behind, or side_tracking.',
        ),
        DeclareLaunchArgument(
            'move_target',
            default_value='false',
            description='Move the green target during the obstacle avoidance demo.',
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(cinematic_launch),
            launch_arguments={
                'world': obstacle_world,
                'mode': mode,
                'move_target': move_target,
                'enable_obstacle_avoidance': 'true',
                'orbit_speed': '0.16',
                'max_linear_speed': '0.22',
                'max_angular_speed': '0.38',
            }.items(),
        ),
    ])