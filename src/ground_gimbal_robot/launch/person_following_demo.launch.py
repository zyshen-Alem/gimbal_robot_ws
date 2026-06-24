import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.actions import TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    package_dir = get_package_share_directory('ground_gimbal_robot')
    tracking_launch = os.path.join(
        package_dir, 'launch', 'human_tracking_demo.launch.py'
    )

    use_sim_time = LaunchConfiguration('use_sim_time')
    detector = LaunchConfiguration('detector')
    turn_kp = LaunchConfiguration('turn_kp')
    turn_prediction_time = LaunchConfiguration('turn_prediction_time')
    deadband_x = LaunchConfiguration('deadband_x')
    lost_target_timeout = LaunchConfiguration('lost_target_timeout')
    lost_target_speed = LaunchConfiguration('lost_target_speed')
    lost_target_turn_speed = LaunchConfiguration('lost_target_turn_speed')
    lost_target_turn_deadband = LaunchConfiguration('lost_target_turn_deadband')
    visible_cruise_speed = LaunchConfiguration('visible_cruise_speed')
    desired_confidence = LaunchConfiguration('desired_confidence')
    close_hold_confidence = LaunchConfiguration('close_hold_confidence')
    safe_stop_confidence = LaunchConfiguration('safe_stop_confidence')
    min_forward_speed = LaunchConfiguration('min_forward_speed')
    curve_follow_speed = LaunchConfiguration('curve_follow_speed')
    max_linear_speed = LaunchConfiguration('max_linear_speed')
    max_angular_speed = LaunchConfiguration('max_angular_speed')
    linear_accel_limit = LaunchConfiguration('linear_accel_limit')
    angular_accel_limit = LaunchConfiguration('angular_accel_limit')
    angular_decel_limit = LaunchConfiguration('angular_decel_limit')
    x_error_filter_alpha = LaunchConfiguration('x_error_filter_alpha')
    x_error_rate_alpha = LaunchConfiguration('x_error_rate_alpha')
    turn_slowdown_start = LaunchConfiguration('turn_slowdown_start')
    min_turn_linear_scale = LaunchConfiguration('min_turn_linear_scale')
    enable_obstacle_avoidance = LaunchConfiguration('enable_obstacle_avoidance')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument(
            'detector',
            default_value='color',
            description='Human detector backend: color, hog, or yolo.',
        ),
        DeclareLaunchArgument(
            'turn_kp',
            default_value='0.82',
            description='Target-centering yaw proportional gain.',
        ),
        DeclareLaunchArgument(
            'turn_prediction_time',
            default_value='0.15',
            description='Seconds of image-offset prediction used during turns.',
        ),
        DeclareLaunchArgument(
            'deadband_x',
            default_value='0.06',
            description='Image-space target offset ignored before turning.',
        ),
        DeclareLaunchArgument(
            'lost_target_timeout',
            default_value='1.0',
            description='Seconds to keep pursuing after the target leaves view.',
        ),
        DeclareLaunchArgument(
            'lost_target_speed',
            default_value='0.36',
            description='Forward speed while pursuing a recently lost target.',
        ),
        DeclareLaunchArgument(
            'lost_target_turn_speed',
            default_value='0.70',
            description='Yaw speed while turning toward the last seen target side.',
        ),
        DeclareLaunchArgument(
            'lost_target_turn_deadband',
            default_value='0.25',
            description='Last-seen image offset required before turning after target loss.',
        ),
        DeclareLaunchArgument(
            'visible_cruise_speed',
            default_value='0.24',
            description='Forward creep speed while the target is visible and centered.',
        ),
        DeclareLaunchArgument(
            'desired_confidence',
            default_value='0.72',
            description='Target confidence proxy for the preferred following distance.',
        ),
        DeclareLaunchArgument(
            'close_hold_confidence',
            default_value='0.82',
            description='Confidence proxy where the robot stops creeping forward.',
        ),
        DeclareLaunchArgument(
            'safe_stop_confidence',
            default_value='0.92',
            description='Confidence proxy where the robot starts backing away.',
        ),
        DeclareLaunchArgument(
            'min_forward_speed',
            default_value='0.36',
            description='Minimum forward speed while target is visible.',
        ),
        DeclareLaunchArgument(
            'curve_follow_speed',
            default_value='0.50',
            description='Forward speed used while arcing toward an off-center target.',
        ),
        DeclareLaunchArgument(
            'max_linear_speed',
            default_value='0.72',
            description='Maximum forward following speed in m/s.',
        ),
        DeclareLaunchArgument(
            'max_angular_speed',
            default_value='1.05',
            description='Maximum target-centering yaw speed in rad/s.',
        ),
        DeclareLaunchArgument(
            'linear_accel_limit',
            default_value='1.60',
            description='Maximum linear velocity change per second.',
        ),
        DeclareLaunchArgument(
            'angular_accel_limit',
            default_value='1.80',
            description='Maximum yaw velocity change per second.',
        ),
        DeclareLaunchArgument(
            'angular_decel_limit',
            default_value='4.50',
            description='Yaw deceleration limit used to avoid turning past the target.',
        ),
        DeclareLaunchArgument(
            'x_error_filter_alpha',
            default_value='0.90',
            description='Low-pass filter alpha for image-space target offset.',
        ),
        DeclareLaunchArgument(
            'x_error_rate_alpha',
            default_value='0.55',
            description='Low-pass filter alpha for image-space target offset rate.',
        ),
        DeclareLaunchArgument(
            'turn_slowdown_start',
            default_value='0.85',
            description='Image offset where curve following begins slowing linear speed.',
        ),
        DeclareLaunchArgument(
            'min_turn_linear_scale',
            default_value='0.95',
            description='Minimum linear-speed scale while turning toward the target.',
        ),
        DeclareLaunchArgument(
            'enable_obstacle_avoidance',
            default_value='false',
            description='Enable the Week 5 obstacle-aware local planner.',
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(tracking_launch),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'detector': detector,
            }.items(),
        ),
        Node(
            package='ground_gimbal_robot',
            executable='person_following',
            name='person_following',
            parameters=[{
                'use_sim_time': use_sim_time,
                'turn_kp': ParameterValue(turn_kp, value_type=float),
                'turn_prediction_time': ParameterValue(
                    turn_prediction_time,
                    value_type=float,
                ),
                'deadband_x': ParameterValue(deadband_x, value_type=float),
                'lost_target_timeout': ParameterValue(
                    lost_target_timeout,
                    value_type=float,
                ),
                'lost_target_speed': ParameterValue(
                    lost_target_speed,
                    value_type=float,
                ),
                'lost_target_turn_speed': ParameterValue(
                    lost_target_turn_speed,
                    value_type=float,
                ),
                'lost_target_turn_deadband': ParameterValue(
                    lost_target_turn_deadband,
                    value_type=float,
                ),
                'visible_cruise_speed': ParameterValue(
                    visible_cruise_speed,
                    value_type=float,
                ),
                'desired_confidence': ParameterValue(
                    desired_confidence, value_type=float
                ),
                'close_hold_confidence': ParameterValue(
                    close_hold_confidence, value_type=float
                ),
                'safe_stop_confidence': ParameterValue(
                    safe_stop_confidence, value_type=float
                ),
                'min_forward_speed': ParameterValue(
                    min_forward_speed,
                    value_type=float,
                ),
                'curve_follow_speed': ParameterValue(
                    curve_follow_speed,
                    value_type=float,
                ),
                'max_linear_speed': ParameterValue(max_linear_speed, value_type=float),
                'max_angular_speed': ParameterValue(
                    max_angular_speed, value_type=float
                ),
                'linear_accel_limit': ParameterValue(
                    linear_accel_limit,
                    value_type=float,
                ),
                'angular_accel_limit': ParameterValue(
                    angular_accel_limit,
                    value_type=float,
                ),
                'angular_decel_limit': ParameterValue(
                    angular_decel_limit,
                    value_type=float,
                ),
                'x_error_filter_alpha': ParameterValue(
                    x_error_filter_alpha,
                    value_type=float,
                ),
                'x_error_rate_alpha': ParameterValue(
                    x_error_rate_alpha,
                    value_type=float,
                ),
                'turn_slowdown_start': ParameterValue(
                    turn_slowdown_start,
                    value_type=float,
                ),
                'min_turn_linear_scale': ParameterValue(
                    min_turn_linear_scale,
                    value_type=float,
                ),
                'enable_obstacle_avoidance': ParameterValue(
                    enable_obstacle_avoidance,
                    value_type=bool,
                ),
                'obstacles': [
                    1.35, 0.85, 0.32,
                    2.05, -0.85, 0.34,
                ],
            }],
            output='screen',
        ),
        TimerAction(
            period=3.0,
            actions=[
                Node(
                    package='ground_gimbal_robot',
                    executable='moving_target_demo',
                    name='moving_target_demo',
                    parameters=[{'use_sim_time': use_sim_time}],
                    output='screen',
                ),
            ],
        ),
    ])
