import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    package_dir = get_package_share_directory('ground_gimbal_robot')
    gazebo_launch = os.path.join(package_dir, 'launch', 'gazebo.launch.py')
    actor_world = os.path.join(package_dir, 'worlds', 'yolo_actor_following.world')

    use_sim_time = LaunchConfiguration('use_sim_time')
    yolo_model = LaunchConfiguration('yolo_model')
    image_topic = LaunchConfiguration('image_topic')
    depth_topic = LaunchConfiguration('depth_topic')
    tracking_mode = LaunchConfiguration('tracking_mode')
    side_distance = LaunchConfiguration('side_distance')
    orbit_radius = LaunchConfiguration('orbit_radius')
    orbit_speed = LaunchConfiguration('orbit_speed')
    gazebo_static_gimbal = LaunchConfiguration('gazebo_static_gimbal')
    enable_gimbal_controllers = LaunchConfiguration('enable_gimbal_controllers')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
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
        DeclareLaunchArgument(
            'depth_topic',
            default_value='/gimbal_camera/gimbal_depth_camera/depth/image_raw',
            description='Depth image topic used for camera-based following distance.',
        ),
        DeclareLaunchArgument(
            'tracking_mode',
            default_value='follow',
            description='Base filming mode: follow, side_left, side_right, orbit_cw, orbit_ccw.',
        ),
        DeclareLaunchArgument('side_distance', default_value='2.0'),
        DeclareLaunchArgument('orbit_radius', default_value='2.0'),
        DeclareLaunchArgument('orbit_speed', default_value='0.19'),
        DeclareLaunchArgument(
            'gazebo_static_gimbal',
            default_value='false',
            description='Use movable gimbal for runtime orbit/side modes.',
        ),
        DeclareLaunchArgument(
            'enable_gimbal_controllers',
            default_value='true',
            description='Start gimbal ros2_control controllers when gazebo_static_gimbal is false.',
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gazebo_launch),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'world': actor_world,
                'gazebo_static_gimbal': gazebo_static_gimbal,
                'spawn_x': '0.0',
                'spawn_y': '0.0',
                'spawn_z': '0.08',
                'spawn_yaw': '0.0',
            }.items(),
        ),
        TimerAction(
            period=2.0,
            condition=IfCondition(enable_gimbal_controllers),
            actions=[
                Node(
                    package='controller_manager',
                    executable='spawner',
                    arguments=[
                        'joint_state_broadcaster',
                        '--controller-manager',
                        '/controller_manager',
                    ],
                    output='screen',
                ),
                Node(
                    package='controller_manager',
                    executable='spawner',
                    arguments=[
                        'gimbal_position_controller',
                        '--controller-manager',
                        '/controller_manager',
                    ],
                    output='screen',
                ),
            ],
        ),

        Node(
            package='ground_gimbal_robot',
            executable='human_tracking',
            name='human_tracking',
            parameters=[{
                'use_sim_time': use_sim_time,
                'detector': 'yolo',
                'yolo_model': yolo_model,
                'image_topic': image_topic,
                'depth_topic': depth_topic,
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
                'publish_joint_states': False,
                'enable_search': True,
                'search_pan_limit': 1.35,
                'search_period': 4.0,
                'search_tilt_angle': 0.0,
                'target_height': 0.45,
                'pan_rate_limit': 0.8,
                'tilt_rate_limit': 0.55,
            }],
            output='screen',
        ),
        Node(
            package='ground_gimbal_robot',
            executable='tracking_mode_director',
            name='tracking_mode_director',
            parameters=[{
                'use_sim_time': use_sim_time,
                'side_actor_speed': 0.16,
                'side_start_x': 12.8,
                'side_start_y': 0.0,
                'side_start_z': 0.0,
                'side_start_yaw': 3.14159,
            }],
            output='screen',
        ),
        TimerAction(
            period=3.0,
            actions=[
                Node(
                    package='ground_gimbal_robot',
                    executable='person_following',
                    name='yolo_person_following',
                    parameters=[{
                        'use_sim_time': use_sim_time,
                        'target_offset_topic': 'gimbal/target_offset',
                        'target_visible_topic': 'gimbal/target_visible',
                        'lost_target_speed': 0.08,
                        'lost_target_turn_speed': 0.20,
                        'lost_target_timeout': 3.0,
                        'tracking_mode': tracking_mode,
                        'side_distance': ParameterValue(side_distance, value_type=float),
                        'orbit_radius': ParameterValue(orbit_radius, value_type=float),
                        'orbit_speed': ParameterValue(orbit_speed, value_type=float),
                        'enable_obstacle_avoidance': True,
                        'use_scan_obstacles': True,
                        'scan_topic': 'scan',
                        'scan_clear_distance': 1.10,
                        'avoidance_trigger_distance': 0.82,
                        'scan_wide_side_angle': 1.57,
                        'scan_stop_distance': 0.42,
                        'scan_turn_gain': 0.65,
                        'avoid_turn_duration': 0.8,
                        'avoid_arc_duration': 5.5,
                        'avoid_arc_min_duration': 2.5,
                        'avoid_arc_speed': 0.13,
                        'avoid_turn_speed': 0.30,
                        'avoid_arc_turn_speed': 0.20,
                        'avoid_edge_distance': 0.75,
                        'cinematic_avoid_edge_distance': 0.75,
                        'cinematic_avoidance_trigger_distance': 0.82,
                        'cinematic_avoid_arc_speed': 0.13,
                        'desired_distance': 1.05,
                        'close_hold_distance': 0.80,
                        'safe_stop_distance': 0.55,
                        'deadband_distance': 0.10,
                        'turn_kp': 0.55,
                        'distance_kp': 0.45,
                        'min_forward_speed': 0.08,
                        'visible_cruise_speed': 0.05,
                        'curve_follow_speed': 0.10,
                        'max_linear_speed': 0.20,
                        'max_reverse_speed': 0.0,
                        'max_angular_speed': 0.35,
                        'linear_accel_limit': 0.35,
                        'angular_accel_limit': 0.45,
                        'angular_decel_limit': 0.80,
                        'turn_before_forward_error': 0.70,
                    }],
                    output='screen',
                ),
            ],
        ),
    ])