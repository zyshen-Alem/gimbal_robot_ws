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

    return LaunchDescription([
        DeclareLaunchArgument(
            'mode',
            default_value='orbit',
            description='Tracking mode for Week 7 demo: orbit, showcase, side_tracking, or follow_behind.',
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(cinematic_launch),
            launch_arguments={
                'world': obstacle_world,
                'mode': mode,
                'move_target': 'true',
                'enable_obstacle_avoidance': 'true',
                'direct_gazebo_drive': 'false',
                'gimbal_tracking_source': 'odom',
                'tracking_filter_prediction_timeout': '1.2',
                'tracking_filter_measurement_noise': '0.055',
                'showcase_period': '12.0',
                'orbit_speed': '0.16',
                'orbit_lookahead_angle_deg': '24.0',
                'side_speed': '0.18',
                'follow_speed': '0.22',
                'max_linear_speed': '0.16',
                'max_angular_speed': '0.32',
                'search_creep_speed': '0.04',
            }.items(),
        ),
    ])
