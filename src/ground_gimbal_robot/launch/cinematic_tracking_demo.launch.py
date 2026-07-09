import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.actions import TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    package_dir = get_package_share_directory('ground_gimbal_robot')
    gazebo_launch = os.path.join(package_dir, 'launch', 'gazebo.launch.py')
    tracking_world = os.path.join(package_dir, 'worlds', 'human_tracking.world')

    world = LaunchConfiguration('world')

    use_sim_time = LaunchConfiguration('use_sim_time')
    detector = LaunchConfiguration('detector')
    depth_topic = LaunchConfiguration('depth_topic')
    raw_target_offset_topic = LaunchConfiguration('raw_target_offset_topic')
    raw_target_visible_topic = LaunchConfiguration('raw_target_visible_topic')
    filtered_target_offset_topic = LaunchConfiguration('filtered_target_offset_topic')
    filtered_target_visible_topic = LaunchConfiguration('filtered_target_visible_topic')
    target_odom_topic = LaunchConfiguration('target_odom_topic')
    mode = LaunchConfiguration('mode')
    enable_base_tracking = LaunchConfiguration('enable_base_tracking')
    direct_gazebo_drive = LaunchConfiguration('direct_gazebo_drive')
    enable_gimbal_controller = LaunchConfiguration('enable_gimbal_controller')
    move_target = LaunchConfiguration('move_target')
    desired_confidence = LaunchConfiguration('desired_confidence')
    desired_distance = LaunchConfiguration('desired_distance')
    distance_kp = LaunchConfiguration('distance_kp')
    showcase_period = LaunchConfiguration('showcase_period')
    orbit_radius = LaunchConfiguration('orbit_radius')
    side_distance = LaunchConfiguration('side_distance')
    side_preferred_side = LaunchConfiguration('side_preferred_side')
    side_reference_mode = LaunchConfiguration('side_reference_mode')
    orbit_speed = LaunchConfiguration('orbit_speed')
    orbit_turn_bias = LaunchConfiguration('orbit_turn_bias')
    side_speed = LaunchConfiguration('side_speed')
    side_tracking_offset = LaunchConfiguration('side_tracking_offset')
    follow_speed = LaunchConfiguration('follow_speed')
    max_linear_speed = LaunchConfiguration('max_linear_speed')
    max_reverse_speed = LaunchConfiguration('max_reverse_speed')
    max_angular_speed = LaunchConfiguration('max_angular_speed')
    recenter_speed = LaunchConfiguration('recenter_speed')
    search_creep_speed = LaunchConfiguration('search_creep_speed')
    enable_obstacle_avoidance = LaunchConfiguration('enable_obstacle_avoidance')
    manual_override = LaunchConfiguration('manual_override')
    gimbal_tracking_source = LaunchConfiguration('gimbal_tracking_source')
    tracking_filter_prediction_timeout = LaunchConfiguration('tracking_filter_prediction_timeout')
    tracking_filter_measurement_noise = LaunchConfiguration('tracking_filter_measurement_noise')
    gimbal_desired_x_offset = LaunchConfiguration('gimbal_desired_x_offset')
    gimbal_desired_y_offset = LaunchConfiguration('gimbal_desired_y_offset')
    pan_offset = LaunchConfiguration('pan_offset')
    tilt_offset = LaunchConfiguration('tilt_offset')
    pan_sign = LaunchConfiguration('pan_sign')
    tilt_sign = LaunchConfiguration('tilt_sign')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument(
            'world',
            default_value=tracking_world,
            description='Gazebo world file for the cinematic demo.',
        ),
        DeclareLaunchArgument(
            'detector',
            default_value='color',
            description='Human detector backend: color, hog, or yolo.',
        ),
        DeclareLaunchArgument(
            'depth_topic',
            default_value='/gimbal_camera/gimbal_depth_camera/depth/image_raw',
            description='Depth image topic aligned with the gimbal RGB camera.',
        ),
        DeclareLaunchArgument(
            'raw_target_offset_topic',
            default_value='gimbal/target_offset_raw',
            description='Unfiltered visual target offset published by human_tracking.',
        ),
        DeclareLaunchArgument(
            'raw_target_visible_topic',
            default_value='gimbal/target_visible_raw',
            description='Unfiltered target visibility published by human_tracking.',
        ),
        DeclareLaunchArgument(
            'filtered_target_offset_topic',
            default_value='gimbal/target_offset',
            description='Week 7 EKF-style filtered target offset topic.',
        ),
        DeclareLaunchArgument(
            'filtered_target_visible_topic',
            default_value='gimbal/target_visible',
            description='Week 7 filtered/predicted target visibility topic.',
        ),
        DeclareLaunchArgument(
            'target_odom_topic',
            default_value='/tracking_subject/odom',
            description='Odometry topic for the green tracking subject.',
        ),
        DeclareLaunchArgument(
            'enable_base_tracking',
            default_value='true',
            description='Start the mobile-base cinematic tracking controller.',
        ),
        DeclareLaunchArgument(
            'direct_gazebo_drive',
            default_value='true',
            description='Directly update the Gazebo model pose for demos; set false to drive through /cmd_vel and the differential-drive plugin.',
        ),
        DeclareLaunchArgument(
            'mode',
            default_value='showcase',
            description='Week 6 cinematic mode: showcase, orbit, follow_behind, or side_tracking.',
        ),
        DeclareLaunchArgument(
            'enable_gimbal_controller',
            default_value='true',
            description='Start the gimbal tracking controller.',
        ),
        DeclareLaunchArgument(
            'move_target',
            default_value='false',
            description='Move the green target for follow-behind and side-tracking demos.',
        ),
        DeclareLaunchArgument(
            'showcase_period',
            default_value='14.0',
            description='Seconds per shot type when mode:=showcase.',
        ),
        DeclareLaunchArgument(
            'orbit_radius',
            default_value='2.0',
            description='Radius in meters for orbit shots around the subject.',
        ),
        DeclareLaunchArgument(
            'side_distance',
            default_value='2.0',
            description='Lateral distance in meters to hold from the subject during side tracking.',
        ),
        DeclareLaunchArgument(
            'side_preferred_side',
            default_value='left',
            description='Subject-relative side for side tracking: left, right, or orbit_direction.',
        ),
        DeclareLaunchArgument(
            'side_reference_mode',
            default_value='target_motion',
            description='Use target_motion for true parallel side tracking, or bearing for tangent fallback.',
        ),
        DeclareLaunchArgument(
            'desired_confidence',
            default_value='0.58',
            description='Visual size proxy for the preferred filming distance.',
        ),
        DeclareLaunchArgument(
            'desired_distance',
            default_value='1.55',
            description='Preferred camera-to-subject distance in meters.',
        ),
        DeclareLaunchArgument(
            'distance_kp',
            default_value='0.35',
            description='Distance-control gain for depth-based subject spacing.',
        ),
        DeclareLaunchArgument(
            'orbit_speed',
            default_value='0.18',
            description='Forward speed used during orbit shots.',
        ),
        DeclareLaunchArgument(
            'orbit_turn_bias',
            default_value='0.20',
            description='Base yaw bias that creates a circular orbit path.',
        ),
        DeclareLaunchArgument(
            'side_speed',
            default_value='0.20',
            description='Forward speed used during side-tracking shots.',
        ),
        DeclareLaunchArgument(
            'side_tracking_offset',
            default_value='0.32',
            description='Image-space offset used to hold the subject to one side.',
        ),
        DeclareLaunchArgument(
            'follow_speed',
            default_value='0.24',
            description='Forward speed used during follow-behind shots.',
        ),
        DeclareLaunchArgument(
            'max_linear_speed',
            default_value='0.34',
            description='Maximum cinematic base speed in m/s.',
        ),
        DeclareLaunchArgument(
            'max_reverse_speed',
            default_value='0.12',
            description='Maximum reverse cinematic base speed in m/s.',
        ),
        DeclareLaunchArgument(
            'max_angular_speed',
            default_value='0.45',
            description='Maximum cinematic base yaw speed in rad/s.',
        ),
        DeclareLaunchArgument(
            'recenter_speed',
            default_value='0.08',
            description='Forward creep speed while recentering a far off-center subject.',
        ),
        DeclareLaunchArgument(
            'search_creep_speed',
            default_value='0.05',
            description='Forward creep speed while recovering a recently lost target.',
        ),
        DeclareLaunchArgument(
            'enable_obstacle_avoidance',
            default_value='false',
            description='Enable static-obstacle avoidance for cinematic paths.',
        ),
        DeclareLaunchArgument(
            'manual_override',
            default_value='false',
            description='Pause automatic base control so a teleop/manual publisher can drive /cmd_vel.',
        ),
        DeclareLaunchArgument(
            'gimbal_tracking_source',
            default_value='topic',
            description='Use topic to consume the Week 7 filtered target offset, or odom for ground-truth gimbal aiming.',
        ),
        DeclareLaunchArgument(
            'tracking_filter_prediction_timeout',
            default_value='1.2',
            description='Seconds to keep predicting filtered target offset after short detection loss.',
        ),
        DeclareLaunchArgument(
            'tracking_filter_measurement_noise',
            default_value='0.055',
            description='Measurement noise used by the Week 7 target Kalman filter.',
        ),
        DeclareLaunchArgument(
            'gimbal_desired_x_offset',
            default_value='0.0',
            description='Desired horizontal subject offset for camera framing.',
        ),
        DeclareLaunchArgument(
            'gimbal_desired_y_offset',
            default_value='0.0',
            description='Desired vertical subject offset for camera framing.',
        ),
        DeclareLaunchArgument(
            'pan_offset',
            default_value='0.0',
            description='Calibration offset added to gimbal pan angle in radians.',
        ),
        DeclareLaunchArgument(
            'tilt_offset',
            default_value='0.0',
            description='Calibration offset added to gimbal tilt angle in radians.',
        ),
        DeclareLaunchArgument(
            'pan_sign',
            default_value='1.0',
            description='Use -1.0 if pan moves opposite the target.',
        ),
        DeclareLaunchArgument(
            'tilt_sign',
            default_value='-1.0',
            description='Use 1.0 if tilt moves opposite the target.',
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gazebo_launch),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'world': world,
                'gazebo_static_gimbal': 'false',
                'spawn_x': '1.8',
                'spawn_y': '-1.2',
                'spawn_z': '0.08',
                'spawn_yaw': '0.0',
            }.items(),
        ),
        TimerAction(
            period=2.0,
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
                'detector': detector,
                'image_topic': '/gimbal_camera/gimbal_camera/image_raw',
                'depth_topic': depth_topic,
                'target_offset_topic': raw_target_offset_topic,
                'target_visible_topic': raw_target_visible_topic,
            }],
            output='screen',
        ),
        Node(
            package='ground_gimbal_robot',
            executable='tracking_filter',
            name='tracking_filter',
            parameters=[{
                'use_sim_time': use_sim_time,
                'input_offset_topic': raw_target_offset_topic,
                'input_visible_topic': raw_target_visible_topic,
                'output_offset_topic': filtered_target_offset_topic,
                'output_visible_topic': filtered_target_visible_topic,
                'prediction_timeout': ParameterValue(
                    tracking_filter_prediction_timeout,
                    value_type=float,
                ),
                'measurement_noise': ParameterValue(
                    tracking_filter_measurement_noise,
                    value_type=float,
                ),
            }],
            output='screen',
        ),
        Node(
            package='ground_gimbal_robot',
            executable='cinematic_tracking',
            name='cinematic_tracking',
            condition=IfCondition(enable_base_tracking),
            parameters=[{
                'use_sim_time': use_sim_time,
                'mode': mode,
                'tracking_source': 'model_states',
                'direct_gazebo_drive': ParameterValue(direct_gazebo_drive, value_type=bool),
                'target_odom_topic': target_odom_topic,
                'target_offset_topic': filtered_target_offset_topic,
                'target_visible_topic': filtered_target_visible_topic,
                'model_states_topic': '/gazebo/model_states',
                'robot_model_name': 'ground_gimbal_robot',
                'target_model_name': 'tracking_subject',
                'orbit_radius': ParameterValue(orbit_radius, value_type=float),
                'follow_distance': 1.35,
                'side_distance': ParameterValue(side_distance, value_type=float),
                'side_angle_deg': 82.0,
                'side_preferred_side': side_preferred_side,
                'side_reference_mode': side_reference_mode,
                'heading_kp': 1.45,
                'orbit_turn_gain': 1.35,
                'orbit_lookahead_angle_deg': 24.0,
                'startup_delay': 3.0,
                'linear_accel_limit': 0.18,
                'angular_accel_limit': 0.35,
                'showcase_period': ParameterValue(
                    showcase_period,
                    value_type=float,
                ),
                'desired_confidence': ParameterValue(
                    desired_confidence,
                    value_type=float,
                ),
                'desired_distance': ParameterValue(
                    desired_distance,
                    value_type=float,
                ),
                'distance_kp': ParameterValue(distance_kp, value_type=float),
                'orbit_speed': ParameterValue(orbit_speed, value_type=float),
                'orbit_turn_bias': ParameterValue(
                    orbit_turn_bias,
                    value_type=float,
                ),
                'side_speed': ParameterValue(side_speed, value_type=float),
                'side_tracking_offset': ParameterValue(
                    side_tracking_offset,
                    value_type=float,
                ),
                'follow_speed': ParameterValue(follow_speed, value_type=float),
                'max_linear_speed': ParameterValue(
                    max_linear_speed,
                    value_type=float,
                ),
                'max_reverse_speed': ParameterValue(
                    max_reverse_speed,
                    value_type=float,
                ),
                'max_angular_speed': ParameterValue(
                    max_angular_speed,
                    value_type=float,
                ),
                'recenter_speed': ParameterValue(
                    recenter_speed,
                    value_type=float,
                ),
                'search_creep_speed': ParameterValue(
                    search_creep_speed,
                    value_type=float,
                ),
                'enable_obstacle_avoidance': ParameterValue(
                    enable_obstacle_avoidance,
                    value_type=bool,
                ),
                'manual_override': ParameterValue(manual_override, value_type=bool),
                'obstacles': [
                    0.95, -0.85, 0.28,
                    1.80, 0.85, 0.30,
                ],
                'obstacle_inflation_radius': 0.55,
                'avoidance_orbit_radius_offset': 0.85,
                'target_clearance_radius': 0.95,
                'avoidance_turn_gain': 0.65,
            }],
            output='screen',
        ),
        Node(
            package='ground_gimbal_robot',
            executable='gimbal_demo',
            name='cinematic_gimbal_controller',
            condition=IfCondition(enable_gimbal_controller),
            parameters=[{
                'use_sim_time': use_sim_time,
                'tracking_source': gimbal_tracking_source,
                'desired_x_offset': ParameterValue(
                    gimbal_desired_x_offset,
                    value_type=float,
                ),
                'desired_y_offset': ParameterValue(
                    gimbal_desired_y_offset,
                    value_type=float,
                ),
                'publish_joint_states': False,
                'gazebo_command_topic': '/gimbal_position_controller/commands',
                'robot_odom_topic': '/odom',
                'target_odom_topic': '/tracking_subject/odom',
                'camera_height': 0.55,
                'target_height': 0.82,
                'pan_offset': ParameterValue(pan_offset, value_type=float),
                'tilt_offset': ParameterValue(tilt_offset, value_type=float),
                'pan_sign': ParameterValue(pan_sign, value_type=float),
                'tilt_sign': ParameterValue(tilt_sign, value_type=float),
                'deadband_x': 0.045,
                'deadband_y': 0.055,
                'error_filter_alpha': 0.55,
            }],
            output='screen',
        ),
        TimerAction(
            period=3.0,
            condition=IfCondition(move_target),
            actions=[
                Node(
                    package='ground_gimbal_robot',
                    executable='moving_target_demo',
                    name='moving_target_demo',
                    parameters=[{
                        'use_sim_time': use_sim_time,
                        'forward_speed': 0.22,
                        'turn_speed': 0.18,
                    }],
                    output='screen',
                ),
            ],
        ),
    ])
