import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_dir = get_package_share_directory('ground_gimbal_robot')
    gazebo_ros_dir = get_package_share_directory('gazebo_ros')
    xacro_file = os.path.join(package_dir, 'urdf', 'ground_gimbal_robot.xacro')

    use_sim_time = LaunchConfiguration('use_sim_time')
    world_file = LaunchConfiguration('world')
    gazebo_static_gimbal = LaunchConfiguration('gazebo_static_gimbal')
    spawn_x = LaunchConfiguration('spawn_x')
    spawn_y = LaunchConfiguration('spawn_y')
    spawn_z = LaunchConfiguration('spawn_z')
    spawn_yaw = LaunchConfiguration('spawn_yaw')
    robot_description = {
        'robot_description': Command([
            'xacro ',
            xacro_file,
            ' gazebo_static_gimbal:=',
            gazebo_static_gimbal,
        ])
    }

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument(
            'world',
            default_value=os.path.join(package_dir, 'worlds', 'empty.world'),
        ),
        DeclareLaunchArgument('spawn_x', default_value='0.0'),
        DeclareLaunchArgument('spawn_y', default_value='0.0'),
        DeclareLaunchArgument('spawn_z', default_value='0.08'),
        DeclareLaunchArgument('spawn_yaw', default_value='0.0'),
        DeclareLaunchArgument(
            'gazebo_static_gimbal',
            default_value='true',
            description='Use fixed gimbal joints for Gazebo-only layout checks.',
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(gazebo_ros_dir, 'launch', 'gazebo.launch.py')
            ),
            launch_arguments={'world': world_file}.items(),
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[robot_description, {'use_sim_time': use_sim_time}],
            output='screen',
        ),
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=[
                '-topic', 'robot_description',
                '-entity', 'ground_gimbal_robot',
                '-x', spawn_x,
                '-y', spawn_y,
                '-z', spawn_z,
                '-Y', spawn_yaw,
            ],
            output='screen',
        ),
    ])
