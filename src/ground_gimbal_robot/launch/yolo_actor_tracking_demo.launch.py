import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_dir = get_package_share_directory('ground_gimbal_robot')
    gazebo_launch = os.path.join(package_dir, 'launch', 'gazebo.launch.py')
    actor_world = os.path.join(package_dir, 'worlds', 'yolo_actor_test.world')

    use_sim_time = LaunchConfiguration('use_sim_time')
    detector = LaunchConfiguration('detector')
    yolo_model = LaunchConfiguration('yolo_model')
    image_topic = LaunchConfiguration('image_topic')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument(
            'detector',
            default_value='yolo',
            description='Human detector backend: yolo is the expected backend for this actor test.',
        ),
        DeclareLaunchArgument(
            'yolo_model',
            default_value='yolov8n.pt',
            description='Path to a YOLO model such as yolov8n.pt.',
        ),
        DeclareLaunchArgument(
            'image_topic',
            default_value='/gimbal_camera/gimbal_camera/image_raw',
            description='Camera image topic used by the human tracking node.',
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gazebo_launch),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'world': actor_world,
                'spawn_x': '0.0',
                'spawn_y': '0.0',
                'spawn_z': '0.08',
                'spawn_yaw': '0.0',
            }.items(),
        ),
        Node(
            package='ground_gimbal_robot',
            executable='human_tracking',
            name='human_tracking',
            parameters=[{
                'use_sim_time': use_sim_time,
                'detector': detector,
                'yolo_model': yolo_model,
                'image_topic': image_topic,
                'min_confidence': 0.25,
            }],
            output='screen',
        ),
        Node(
            package='ground_gimbal_robot',
            executable='gimbal_demo',
            name='vision_gimbal_controller',
            parameters=[{
                'use_sim_time': use_sim_time,
                'tracking_source': 'topic',
            }],
            output='screen',
        ),
    ])