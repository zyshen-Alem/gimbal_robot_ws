import math

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from rclpy.node import Node
from std_msgs.msg import Bool
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import String


class PersonFollowing(Node):
    """Convert camera target offset into smooth mobile-base velocity commands."""

    def __init__(self):
        super().__init__('person_following')

        self.declare_parameter('target_offset_topic', 'gimbal/target_offset')
        self.declare_parameter('target_visible_topic', 'gimbal/target_visible')
        self.declare_parameter('odom_topic', 'odom')
        self.declare_parameter('cmd_vel_topic', 'cmd_vel')
        self.declare_parameter('control_topic', 'person_following/control_state')
        self.declare_parameter('gimbal_command_topic', 'gimbal/pid_command')
        self.declare_parameter('mode_command_topic', '/tracking_mode_cmd')
        self.declare_parameter('control_rate', 10.0)
        self.declare_parameter('target_timeout', 0.75)
        self.declare_parameter('lost_target_timeout', 1.0)
        self.declare_parameter('lost_target_speed', 0.36)
        self.declare_parameter('lost_target_turn_speed', 0.70)
        self.declare_parameter('lost_target_turn_deadband', 0.25)
        self.declare_parameter('turn_kp', 0.82)
        self.declare_parameter('turn_prediction_time', 0.15)
        self.declare_parameter('distance_kp', 0.90)
        self.declare_parameter('tracking_mode', 'follow')
        self.declare_parameter('camera_horizontal_fov', 1.3962634)
        self.declare_parameter('side_distance', 2.0)
        self.declare_parameter('side_speed', 0.16)
        self.declare_parameter('side_slot_gain', 0.22)
        self.declare_parameter('side_heading_blend_distance', 0.90)
        self.declare_parameter('side_min_target_speed', 0.03)
        self.declare_parameter('orbit_radius', 2.0)
        self.declare_parameter('orbit_speed', 0.16)
        self.declare_parameter('orbit_direction', 1.0)
        self.declare_parameter('cinematic_avoid_edge_distance', 0.75)
        self.declare_parameter('cinematic_avoidance_trigger_distance', 0.82)
        self.declare_parameter('cinematic_avoid_arc_speed', 0.13)
        self.declare_parameter('desired_distance', 1.40)
        self.declare_parameter('close_hold_distance', 1.05)
        self.declare_parameter('safe_stop_distance', 0.75)
        self.declare_parameter('deadband_distance', 0.12)
        self.declare_parameter('desired_confidence', 0.72)
        self.declare_parameter('close_hold_confidence', 0.82)
        self.declare_parameter('safe_stop_confidence', 0.92)
        self.declare_parameter('deadband_x', 0.06)
        self.declare_parameter('deadband_confidence', 0.05)
        self.declare_parameter('min_forward_speed', 0.36)
        self.declare_parameter('visible_cruise_speed', 0.24)
        self.declare_parameter('curve_follow_speed', 0.50)
        self.declare_parameter('max_linear_speed', 0.72)
        self.declare_parameter('max_reverse_speed', 0.10)
        self.declare_parameter('max_angular_speed', 1.05)
        self.declare_parameter('linear_accel_limit', 1.60)
        self.declare_parameter('angular_accel_limit', 1.80)
        self.declare_parameter('angular_decel_limit', 4.50)
        self.declare_parameter('turn_before_forward_error', 0.90)
        self.declare_parameter('x_error_filter_alpha', 0.90)
        self.declare_parameter('x_error_rate_alpha', 0.55)
        self.declare_parameter('turn_slowdown_start', 0.85)
        self.declare_parameter('min_turn_linear_scale', 0.95)
        self.declare_parameter('enable_obstacle_avoidance', True)
        self.declare_parameter('use_scan_obstacles', True)
        self.declare_parameter('scan_topic', 'scan')
        self.declare_parameter('scan_timeout', 0.50)
        self.declare_parameter('scan_front_angle', 0.45)
        self.declare_parameter('scan_side_angle', 0.95)
        self.declare_parameter('scan_wide_side_angle', 1.57)
        self.declare_parameter('scan_clear_distance', 1.20)
        self.declare_parameter('avoidance_trigger_distance', 0.85)
        self.declare_parameter('scan_stop_distance', 0.45)
        self.declare_parameter('scan_turn_gain', 0.65)
        self.declare_parameter('avoid_turn_duration', 1.2)
        self.declare_parameter('avoid_arc_duration', 3.0)
        self.declare_parameter('avoid_arc_min_duration', 1.0)
        self.declare_parameter('avoid_edge_distance', 0.75)
        self.declare_parameter('avoid_rejoin_angle', 0.65)
        self.declare_parameter('avoid_arc_speed', 0.13)
        self.declare_parameter('avoid_turn_speed', 0.30)
        self.declare_parameter('avoid_arc_turn_speed', 0.20)
        self.declare_parameter(
            'obstacles',
            [
                1.35, 0.85, 0.32,
                2.05, -0.85, 0.34,
            ],
        )
        self.declare_parameter('avoidance_slow_distance', 0.85)
        self.declare_parameter('avoidance_stop_distance', 0.34)
        self.declare_parameter('avoidance_corridor_width', 0.30)
        self.declare_parameter('avoidance_turn_gain', 0.55)

        self.target_offset_topic = self.get_parameter('target_offset_topic').value
        self.target_visible_topic = self.get_parameter('target_visible_topic').value
        self.odom_topic = self.get_parameter('odom_topic').value
        self.cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        self.control_topic = self.get_parameter('control_topic').value
        self.gimbal_command_topic = self.get_parameter('gimbal_command_topic').value
        self.mode_command_topic = self.get_parameter('mode_command_topic').value
        self.control_rate = float(self.get_parameter('control_rate').value)
        self.target_timeout = float(self.get_parameter('target_timeout').value)
        self.lost_target_timeout = float(
            self.get_parameter('lost_target_timeout').value
        )
        self.lost_target_speed = float(
            self.get_parameter('lost_target_speed').value
        )
        self.lost_target_turn_speed = float(
            self.get_parameter('lost_target_turn_speed').value
        )
        self.lost_target_turn_deadband = float(
            self.get_parameter('lost_target_turn_deadband').value
        )
        self.turn_kp = float(self.get_parameter('turn_kp').value)
        self.turn_prediction_time = float(
            self.get_parameter('turn_prediction_time').value
        )
        self.distance_kp = float(self.get_parameter('distance_kp').value)
        self.tracking_mode = str(self.get_parameter('tracking_mode').value).lower()
        self.camera_horizontal_fov = float(
            self.get_parameter('camera_horizontal_fov').value
        )
        self.side_distance = float(self.get_parameter('side_distance').value)
        self.side_speed = float(self.get_parameter('side_speed').value)
        self.side_slot_gain = float(self.get_parameter('side_slot_gain').value)
        self.side_heading_blend_distance = float(
            self.get_parameter('side_heading_blend_distance').value
        )
        self.side_min_target_speed = float(
            self.get_parameter('side_min_target_speed').value
        )
        self.orbit_radius = float(self.get_parameter('orbit_radius').value)
        self.orbit_speed = float(self.get_parameter('orbit_speed').value)
        self.orbit_direction = 1.0 if float(
            self.get_parameter('orbit_direction').value
        ) >= 0.0 else -1.0
        self.cinematic_avoid_edge_distance = float(
            self.get_parameter('cinematic_avoid_edge_distance').value
        )
        self.cinematic_avoidance_trigger_distance = float(
            self.get_parameter('cinematic_avoidance_trigger_distance').value
        )
        self.cinematic_avoid_arc_speed = float(
            self.get_parameter('cinematic_avoid_arc_speed').value
        )
        self.desired_distance = float(self.get_parameter('desired_distance').value)
        self.close_hold_distance = float(self.get_parameter('close_hold_distance').value)
        self.safe_stop_distance = float(self.get_parameter('safe_stop_distance').value)
        self.deadband_distance = float(self.get_parameter('deadband_distance').value)
        self.desired_confidence = float(
            self.get_parameter('desired_confidence').value
        )
        self.safe_stop_confidence = float(
            self.get_parameter('safe_stop_confidence').value
        )
        self.close_hold_confidence = float(
            self.get_parameter('close_hold_confidence').value
        )
        self.deadband_x = float(self.get_parameter('deadband_x').value)
        self.deadband_confidence = float(
            self.get_parameter('deadband_confidence').value
        )
        self.min_forward_speed = float(
            self.get_parameter('min_forward_speed').value
        )
        self.visible_cruise_speed = float(
            self.get_parameter('visible_cruise_speed').value
        )
        self.curve_follow_speed = float(
            self.get_parameter('curve_follow_speed').value
        )
        self.max_linear_speed = float(self.get_parameter('max_linear_speed').value)
        self.max_reverse_speed = float(self.get_parameter('max_reverse_speed').value)
        self.max_angular_speed = float(self.get_parameter('max_angular_speed').value)
        self.linear_accel_limit = float(
            self.get_parameter('linear_accel_limit').value
        )
        self.angular_accel_limit = float(
            self.get_parameter('angular_accel_limit').value
        )
        self.angular_decel_limit = float(
            self.get_parameter('angular_decel_limit').value
        )
        self.turn_before_forward_error = float(
            self.get_parameter('turn_before_forward_error').value
        )
        self.x_error_filter_alpha = float(
            self.get_parameter('x_error_filter_alpha').value
        )
        self.x_error_rate_alpha = float(
            self.get_parameter('x_error_rate_alpha').value
        )
        self.turn_slowdown_start = float(
            self.get_parameter('turn_slowdown_start').value
        )
        self.min_turn_linear_scale = float(
            self.get_parameter('min_turn_linear_scale').value
        )
        self.enable_obstacle_avoidance = bool(
            self.get_parameter('enable_obstacle_avoidance').value
        )
        self.use_scan_obstacles = bool(
            self.get_parameter('use_scan_obstacles').value
        )
        self.scan_topic = self.get_parameter('scan_topic').value
        self.scan_timeout = float(self.get_parameter('scan_timeout').value)
        self.scan_front_angle = float(self.get_parameter('scan_front_angle').value)
        self.scan_side_angle = float(self.get_parameter('scan_side_angle').value)
        self.scan_wide_side_angle = float(
            self.get_parameter('scan_wide_side_angle').value
        )
        self.scan_clear_distance = float(self.get_parameter('scan_clear_distance').value)
        self.avoidance_trigger_distance = float(
            self.get_parameter('avoidance_trigger_distance').value
        )
        self.scan_stop_distance = float(self.get_parameter('scan_stop_distance').value)
        self.scan_turn_gain = float(self.get_parameter('scan_turn_gain').value)
        self.avoid_turn_duration = float(self.get_parameter('avoid_turn_duration').value)
        self.avoid_arc_duration = float(self.get_parameter('avoid_arc_duration').value)
        self.avoid_arc_min_duration = float(
            self.get_parameter('avoid_arc_min_duration').value
        )
        self.avoid_edge_distance = float(self.get_parameter('avoid_edge_distance').value)
        self.avoid_rejoin_angle = float(self.get_parameter('avoid_rejoin_angle').value)
        self.avoid_arc_speed = float(self.get_parameter('avoid_arc_speed').value)
        self.avoid_turn_speed = float(self.get_parameter('avoid_turn_speed').value)
        self.avoid_arc_turn_speed = float(self.get_parameter('avoid_arc_turn_speed').value)
        self.obstacles = self.parse_obstacles(self.get_parameter('obstacles').value)
        self.avoidance_slow_distance = float(
            self.get_parameter('avoidance_slow_distance').value
        )
        self.avoidance_stop_distance = float(
            self.get_parameter('avoidance_stop_distance').value
        )
        self.avoidance_corridor_width = float(
            self.get_parameter('avoidance_corridor_width').value
        )
        self.avoidance_turn_gain = float(
            self.get_parameter('avoidance_turn_gain').value
        )

        self.target_x_error = 0.0
        self.target_y_error = 0.0
        self.target_confidence = 0.0
        self.target_distance = -1.0
        self.gimbal_pan_angle = 0.0
        self.gimbal_tilt_angle = 0.0
        self.person_pose = None
        self.last_person_pose_sample = None
        self.last_person_pose_time = None
        self.person_velocity = (0.0, 0.0)
        self.locked_person_pose = None
        self.filtered_x_error = 0.0
        self.filtered_x_error_rate = 0.0
        self.last_seen_x_error = 0.0
        self.last_offset_time = None
        self.target_visible = False
        self.last_target_time = None
        self.last_timer_time = self.get_clock().now()
        self.current_linear = 0.0
        self.current_angular = 0.0
        self.robot_pose = None
        self.latest_scan = None
        self.last_scan_time = None
        self.avoidance_active = False
        self.avoidance_mode = 'follow_goal'
        self.avoidance_started_time = None
        self.avoidance_side = 1.0
        self.last_state = None

        self.odom_subscription = self.create_subscription(
            Odometry,
            self.odom_topic,
            self.update_odom,
            10,
        )
        self.offset_subscription = self.create_subscription(
            Float64MultiArray,
            self.target_offset_topic,
            self.update_target_offset,
            10,
        )
        self.visible_subscription = self.create_subscription(
            Bool,
            self.target_visible_topic,
            self.update_target_visible,
            10,
        )
        self.gimbal_command_subscription = self.create_subscription(
            Float64MultiArray,
            self.gimbal_command_topic,
            self.update_gimbal_command,
            10,
        )
        self.scan_subscription = self.create_subscription(
            LaserScan,
            self.scan_topic,
            self.update_scan,
            10,
        )
        self.mode_command_subscription = self.create_subscription(
            String,
            self.mode_command_topic,
            self.update_tracking_mode,
            10,
        )

        self.cmd_publisher = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        self.control_publisher = self.create_publisher(
            Float64MultiArray, self.control_topic, 10
        )

        timer_period = 1.0 / max(self.control_rate, 1.0)
        self.timer = self.create_timer(timer_period, self.publish_command)

        self.get_logger().info(
            'Starting YOLO cinematic tracking: mode=%s target_offset=%s cmd_vel=%s '
            'obstacle_avoidance=%s'
            % (
                self.tracking_mode,
                self.target_offset_topic,
                self.cmd_vel_topic,
                self.enable_obstacle_avoidance,
            )
        )

    def update_odom(self, msg):
        orientation = msg.pose.pose.orientation
        yaw = self.quaternion_to_yaw(
            orientation.x,
            orientation.y,
            orientation.z,
            orientation.w,
        )
        self.robot_pose = (
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            yaw,
        )

    def update_scan(self, msg):
        self.latest_scan = msg
        self.last_scan_time = self.get_clock().now()

    def prepare_mode_switch(self, requested):
        self.reset_avoidance()
        self.current_angular = 0.0
        if requested != self.tracking_mode:
            self.get_logger().info('Switching tracking mode: %s -> %s' % (self.tracking_mode, requested))
    def update_tracking_mode(self, msg):
        requested = str(msg.data).strip().lower()
        aliases = {
            'orbitcw': 'orbit_cw',
            'cw': 'orbit_cw',
            'orbitccw': 'orbit_ccw',
            'ccw': 'orbit_ccw',
            'side': 'side_tracking',
            'sidetracking': 'side_tracking',
        }
        requested = aliases.get(requested, requested)
        if requested == 'side_tracking':
            requested = 'side_left'
        valid_modes = ('follow', 'orbit_cw', 'orbit_ccw', 'side_left', 'side_right')
        if requested not in valid_modes:
            self.get_logger().warning('Ignoring unknown tracking mode command: %s' % msg.data)
            return

        self.prepare_mode_switch(requested)
        self.tracking_mode = requested
        if requested.startswith('orbit'):
            self.locked_person_pose = self.person_pose
            if self.locked_person_pose is None:
                self.get_logger().warning('Orbit requested before person pose is available.')
            else:
                self.get_logger().info(
                    'Mode command: %s, locked orbit center at (%.2f, %.2f)'
                    % (requested, self.locked_person_pose[0], self.locked_person_pose[1])
                )
        elif requested == 'follow':
            self.locked_person_pose = None
            self.reset_avoidance()
            self.get_logger().info('Mode command: follow')
        else:
            self.locked_person_pose = None
            self.get_logger().info('Mode command: %s' % requested)
    def update_gimbal_command(self, msg):
        if len(msg.data) >= 1 and math.isfinite(float(msg.data[0])):
            self.gimbal_pan_angle = float(msg.data[0])
        if len(msg.data) >= 2 and math.isfinite(float(msg.data[1])):
            self.gimbal_tilt_angle = float(msg.data[1])

    def update_visual_person_pose(self, now):
        if self.robot_pose is None or self.target_distance <= 0.0:
            return
        robot_x, robot_y, robot_yaw = self.robot_pose
        image_bearing = self.filtered_x_error * self.camera_horizontal_fov * 0.5
        bearing = robot_yaw + self.gimbal_pan_angle + image_bearing
        person_x = robot_x + self.target_distance * math.cos(bearing)
        person_y = robot_y + self.target_distance * math.sin(bearing)
        self.person_pose = (person_x, person_y)

        if self.last_person_pose_sample is not None and self.last_person_pose_time is not None:
            dt = (now - self.last_person_pose_time).nanoseconds / 1_000_000_000
            if dt > 0.02:
                vx = (person_x - self.last_person_pose_sample[0]) / dt
                vy = (person_y - self.last_person_pose_sample[1]) / dt
                if math.isfinite(vx) and math.isfinite(vy):
                    alpha = 0.45
                    self.person_velocity = (
                        alpha * vx + (1.0 - alpha) * self.person_velocity[0],
                        alpha * vy + (1.0 - alpha) * self.person_velocity[1],
                    )

        self.last_person_pose_sample = self.person_pose
        self.last_person_pose_time = now
    def update_target_offset(self, msg):
        if len(msg.data) < 2:
            return

        self.target_x_error = self.clamp(float(msg.data[0]), -1.0, 1.0)
        self.target_y_error = self.clamp(float(msg.data[1]), -1.0, 1.0)
        now = self.get_clock().now()
        previous_filtered_x = self.filtered_x_error
        alpha = self.clamp(self.x_error_filter_alpha, 0.0, 1.0)
        self.filtered_x_error = (
            alpha * self.target_x_error + (1.0 - alpha) * self.filtered_x_error
        )
        if self.last_offset_time is not None:
            dt = (now - self.last_offset_time).nanoseconds / 1_000_000_000
            if dt > 0.001:
                raw_rate = (self.filtered_x_error - previous_filtered_x) / dt
                rate_alpha = self.clamp(self.x_error_rate_alpha, 0.0, 1.0)
                self.filtered_x_error_rate = (
                    rate_alpha * raw_rate
                    + (1.0 - rate_alpha) * self.filtered_x_error_rate
                )
                self.filtered_x_error_rate = self.clamp(
                    self.filtered_x_error_rate,
                    -2.5,
                    2.5,
                )
        self.last_offset_time = now
        self.last_seen_x_error = self.filtered_x_error
        if len(msg.data) >= 3:
            self.target_confidence = self.clamp(float(msg.data[2]), 0.0, 1.0)
        else:
            self.target_confidence = self.desired_confidence
        if len(msg.data) >= 4 and math.isfinite(float(msg.data[3])) and float(msg.data[3]) > 0.0:
            self.target_distance = float(msg.data[3])
        else:
            self.target_distance = -1.0

        self.target_visible = True
        self.last_target_time = now
        self.update_visual_person_pose(now)

    def update_target_visible(self, msg):
        if not msg.data:
            self.target_visible = False

    def publish_command(self):
        now = self.get_clock().now()
        dt = (now - self.last_timer_time).nanoseconds / 1_000_000_000
        self.last_timer_time = now

        target_available = self.is_target_recent(now)
        target_recently_lost = self.is_target_recently_lost(now)
        cinematic_locked = (
            self.tracking_mode.startswith('orbit')
            and self.locked_person_pose is not None
        )
        desired_linear = 0.0
        desired_angular = 0.0

        if target_available or cinematic_locked:
            if self.tracking_mode == 'follow':
                desired_angular = self.compute_follow_angular_command()
                desired_linear = self.compute_linear_command(
                    self.filtered_x_error, self.target_confidence, self.target_distance
                )
                desired_linear = self.keep_forward_motion_on_turn(
                    desired_linear,
                    self.filtered_x_error,
                )
            else:
                desired_linear, desired_angular = self.compute_cinematic_command()
            desired_linear, desired_angular = self.apply_local_planner(
                desired_linear,
                desired_angular,
            )
        elif target_recently_lost:
            desired_linear = self.lost_target_speed
            desired_angular = self.compute_lost_target_angular_command()
            desired_linear, desired_angular = self.apply_local_planner(
                desired_linear,
                desired_angular,
            )
        elif self.avoidance_active:
            desired_linear = self.avoid_arc_speed
            desired_angular = self.avoidance_side * self.avoid_arc_turn_speed
            desired_linear, desired_angular = self.apply_local_planner(
                desired_linear,
                desired_angular,
            )

        self.current_linear = self.slew(
            self.current_linear, desired_linear, self.linear_accel_limit, dt
        )
        self.current_angular = self.slew_angular(
            self.current_angular, desired_angular, dt
        )

        twist = Twist()
        twist.linear.x = self.current_linear
        twist.angular.z = self.current_angular
        self.cmd_publisher.publish(twist)

        control_msg = Float64MultiArray()
        control_msg.data = [
            self.current_linear,
            self.current_angular,
            self.filtered_x_error,
            self.filtered_x_error_rate,
            self.target_confidence,
            self.target_distance,
            1.0 if (target_available or cinematic_locked) else 0.0,
            1.0 if self.avoidance_active else 0.0,
            1.0 if target_recently_lost else 0.0,
            self.mode_index(),
        ]
        self.control_publisher.publish(control_msg)

        self.log_state(target_available, target_recently_lost)

    def compute_cinematic_command(self):
        if self.robot_pose is None or self.person_pose is None:
            return 0.0, 0.0

        mode = self.tracking_mode
        if mode in ('side_left', 'side_right', 'side'):
            return self.compute_side_tracking_command(mode)
        if mode in ('orbit_cw', 'orbit_clockwise', 'orbit_ccw', 'orbit_counterclockwise', 'orbit'):
            return self.compute_orbit_tracking_command(mode)

        return 0.0, 0.0

    def compute_side_tracking_command(self, mode):
        robot_x, robot_y, robot_yaw = self.robot_pose
        person_x, person_y = self.person_pose
        person_vx, person_vy = self.person_velocity
        person_speed = math.hypot(person_vx, person_vy)

        if person_speed >= self.side_min_target_speed:
            person_heading = math.atan2(person_vy, person_vx)
        else:
            person_heading = robot_yaw

        side_sign = -1.0 if mode == 'side_right' else 1.0
        side_normal_x = -math.sin(person_heading) * side_sign
        side_normal_y = math.cos(person_heading) * side_sign
        goal_x = person_x + side_normal_x * self.side_distance
        goal_y = person_y + side_normal_y * self.side_distance

        goal_dx = goal_x - robot_x
        goal_dy = goal_y - robot_y
        slot_error = math.hypot(goal_dx, goal_dy)
        slot_heading = math.atan2(goal_dy, goal_dx)
        blend = self.clamp(
            slot_error / max(self.side_heading_blend_distance, 0.05),
            0.0,
            1.0,
        )
        desired_heading = self.interpolate_angle(person_heading, slot_heading, blend)
        heading_error = self.normalize_angle(desired_heading - robot_yaw)

        along_error = math.cos(person_heading) * goal_dx + math.sin(person_heading) * goal_dy
        linear = self.side_speed + self.side_slot_gain * along_error
        if slot_error > 0.25:
            linear += 0.14 * slot_error * max(0.0, math.cos(heading_error))
        if abs(heading_error) > math.radians(95.0):
            linear = min(linear, 0.04)
        elif linear > 0.0:
            linear *= max(0.45, math.cos(heading_error))

        return (
            self.clamp(linear, -self.max_reverse_speed, self.max_linear_speed),
            self.clamp(self.turn_kp * heading_error, -self.max_angular_speed, self.max_angular_speed),
        )

    def compute_orbit_tracking_command(self, mode):
        robot_x, robot_y, robot_yaw = self.robot_pose
        person_x, person_y = self.locked_person_pose or self.person_pose
        dx = robot_x - person_x
        dy = robot_y - person_y
        radius = max(math.hypot(dx, dy), 0.05)
        current_angle = math.atan2(dy, dx)
        direction = self.orbit_direction
        if mode in ('orbit_cw', 'orbit_clockwise'):
            direction = -1.0
        elif mode in ('orbit_ccw', 'orbit_counterclockwise'):
            direction = 1.0

        radial_error = radius - self.orbit_radius
        tangent_heading = self.normalize_angle(current_angle + direction * math.pi / 2.0)
        radial_heading_offset = -direction * math.atan(
            self.clamp(-1.5 * radial_error, -1.0, 1.0)
        )
        tangent_heading = self.normalize_angle(tangent_heading + radial_heading_offset)
        heading_error = self.normalize_angle(tangent_heading - robot_yaw)

        linear = abs(self.orbit_speed) + 0.16 * self.clamp(abs(radial_error), 0.0, 0.7)
        linear = self.clamp(linear, 0.06, self.max_linear_speed)
        if abs(heading_error) > math.radians(100.0):
            linear = 0.04
        else:
            linear *= max(0.45, math.cos(heading_error))

        orbit_curvature = direction * linear / max(self.orbit_radius, 0.20)
        heading_correction = 0.80 * heading_error
        radial_correction = -0.22 * direction * self.clamp(radial_error, -0.7, 0.7)
        angular = orbit_curvature + heading_correction + radial_correction
        return (
            self.clamp(linear, -self.max_reverse_speed, self.max_linear_speed),
            self.clamp(angular, -self.max_angular_speed, self.max_angular_speed),
        )
    def compute_follow_angular_command(self):
        return self.compute_angular_command(self.predicted_x_error())
    def compute_angular_command(self, x_error):
        if abs(x_error) < self.deadband_x:
            return 0.0

        command = -self.turn_kp * x_error
        return self.clamp(command, -self.max_angular_speed, self.max_angular_speed)

    def compute_lost_target_angular_command(self):
        if abs(self.last_seen_x_error) < self.lost_target_turn_deadband:
            return 0.0

        command = -self.lost_target_turn_speed * self.last_seen_x_error
        return self.clamp(command, -self.max_angular_speed, self.max_angular_speed)

    def predicted_x_error(self):
        prediction = (
            self.filtered_x_error
            + self.turn_prediction_time * self.filtered_x_error_rate
        )
        if (
            abs(self.filtered_x_error) > self.deadband_x
            and prediction * self.filtered_x_error < 0.0
        ):
            return self.filtered_x_error

        return self.clamp(prediction, -1.0, 1.0)

    def compute_linear_command(self, x_error, confidence, distance):
        if abs(x_error) > self.turn_before_forward_error:
            return self.curve_follow_speed

        if distance > 0.0:
            distance_error = distance - self.desired_distance
            if distance <= self.safe_stop_distance:
                return -self.max_reverse_speed
            if distance <= self.close_hold_distance:
                if abs(x_error) > self.deadband_x * 3.0:
                    return min(self.visible_cruise_speed, self.curve_follow_speed)
                return 0.0
            if abs(distance_error) < self.deadband_distance:
                if abs(x_error) > self.deadband_x * 2.0:
                    return self.curve_follow_speed
                return self.visible_cruise_speed
            command = self.distance_kp * distance_error
            if command > 0.0:
                command = max(command, self.min_forward_speed)
            if command > 0.0 and abs(x_error) > self.deadband_x * 2.0:
                command = max(command, self.curve_follow_speed)
            return self.clamp(command, -self.max_reverse_speed, self.max_linear_speed)

        distance_error = self.desired_confidence - confidence
        if confidence >= self.safe_stop_confidence:
            return -self.max_reverse_speed
        if confidence >= self.close_hold_confidence:
            if abs(x_error) > self.deadband_x * 3.0:
                return min(self.visible_cruise_speed, self.curve_follow_speed)
            return 0.0

        if abs(distance_error) < self.deadband_confidence:
            if abs(x_error) > self.deadband_x * 2.0:
                return self.curve_follow_speed
            return self.visible_cruise_speed

        command = self.distance_kp * distance_error
        if command > 0.0:
            command = max(command, self.min_forward_speed)
        if command > 0.0 and abs(x_error) > self.deadband_x * 2.0:
            command = max(command, self.curve_follow_speed)

        return self.clamp(command, -self.max_reverse_speed, self.max_linear_speed)

    def keep_forward_motion_on_turn(self, linear, x_error):
        if linear <= 0.0:
            return linear

        turn_amount = abs(x_error)
        if turn_amount <= self.turn_slowdown_start:
            return linear

        scale_range = max(1.0 - self.turn_slowdown_start, 0.01)
        turn_fraction = self.clamp(
            (turn_amount - self.turn_slowdown_start) / scale_range,
            0.0,
            1.0,
        )
        scale = 1.0 - (1.0 - self.min_turn_linear_scale) * turn_fraction
        return max(linear * scale, self.visible_cruise_speed)

    def apply_local_planner(self, linear, angular):
        if not self.enable_obstacle_avoidance:
            self.avoidance_active = False
            return linear, angular

        if self.use_scan_obstacles:
            if self.has_recent_scan():
                return self.apply_scan_avoidance(linear, angular)
            self.avoidance_active = False
            return linear, angular

        return self.apply_coordinate_obstacle_avoidance(linear, angular)

    def has_recent_scan(self):
        if self.latest_scan is None or self.last_scan_time is None:
            return False

        age = (
            self.get_clock().now() - self.last_scan_time
        ).nanoseconds / 1_000_000_000
        return age <= self.scan_timeout

    def apply_scan_avoidance(self, linear, angular):
        front_min = self.scan_sector_min(-self.scan_front_angle, self.scan_front_angle)
        left_min = self.scan_sector_min(self.scan_front_angle, self.scan_wide_side_angle)
        right_min = self.scan_sector_min(-self.scan_wide_side_angle, -self.scan_front_angle)
        now = self.get_clock().now()

        person_in_front = self.scan_hit_matches_tracked_person(front_min)
        trigger_distance = self.avoidance_trigger_distance
        if self.tracking_mode != 'follow':
            trigger_distance = self.cinematic_avoidance_trigger_distance
        front_blocked = (
            front_min < trigger_distance and not person_in_front
        )
        corridor_tight = (
            left_min < self.scan_stop_distance
            and right_min < self.scan_stop_distance
        )
        if self.avoidance_mode == 'follow_goal':
            if not front_blocked and not corridor_tight:
                self.avoidance_active = False
                return linear, angular
            self.start_avoidance(now, left_min, right_min)

        elapsed = 0.0
        if self.avoidance_started_time is not None:
            elapsed = (now - self.avoidance_started_time).nanoseconds / 1_000_000_000

        self.avoidance_active = True
        if self.avoidance_mode == 'avoid_turn':
            if elapsed >= self.avoid_turn_duration:
                self.avoidance_mode = 'avoid_edge'
                self.avoidance_started_time = now
            return (
                max(self.avoid_arc_speed * 0.75, 0.04),
                self.avoidance_side * self.avoid_turn_speed,
            )

        if self.avoidance_mode == 'avoid_edge':
            if self.can_rejoin_goal(front_min, left_min, right_min, elapsed):
                self.avoidance_mode = 'rejoin_goal'
                self.avoidance_started_time = now
                return linear, angular

            return self.edge_follow_command(front_min, left_min, right_min)

        if self.avoidance_mode == 'rejoin_goal':
            if front_blocked or corridor_tight:
                self.avoidance_mode = 'avoid_edge'
                self.avoidance_started_time = now
                return self.edge_follow_command(front_min, left_min, right_min)
            self.reset_avoidance()
            return linear, angular

        self.reset_avoidance()
        return linear, angular

    def can_rejoin_goal(self, front_min, left_min, right_min, elapsed):
        min_duration = self.avoid_arc_min_duration
        clear_distance = self.scan_clear_distance
        side_clear_distance = self.scan_stop_distance
        if self.tracking_mode.startswith('orbit'):
            min_duration = max(min_duration, 3.2)
            clear_distance = max(clear_distance, 1.25)
            side_clear_distance = max(side_clear_distance, 0.62)

        if elapsed < min_duration:
            return False
        if front_min < clear_distance:
            return False
        if left_min < side_clear_distance or right_min < side_clear_distance:
            return False

        if self.tracking_mode.startswith('orbit'):
            obstacle_side_min = right_min if self.avoidance_side > 0.0 else left_min
            if obstacle_side_min < clear_distance:
                return False

        if not self.is_target_recent(self.get_clock().now()):
            return False
        return abs(self.filtered_x_error) <= self.avoid_rejoin_angle

    def edge_follow_command(self, front_min, left_min, right_min):
        obstacle_side_min = right_min if self.avoidance_side > 0.0 else left_min
        side_obstacle_lost = (
            not math.isfinite(obstacle_side_min)
            or obstacle_side_min > self.scan_clear_distance
        )
        if not math.isfinite(obstacle_side_min):
            obstacle_side_min = self.scan_clear_distance

        desired_edge_distance = self.avoid_edge_distance
        if self.tracking_mode != 'follow':
            desired_edge_distance = self.cinematic_avoid_edge_distance
        edge_error = desired_edge_distance - obstacle_side_min
        angular = (
            self.avoidance_side * self.avoid_arc_turn_speed
            + self.avoidance_side * self.scan_turn_gain * edge_error
        )
        if side_obstacle_lost and front_min >= self.scan_stop_distance:
            if self.tracking_mode.startswith('orbit'):
                angular = 0.0
            else:
                angular = -self.avoidance_side * self.avoid_arc_turn_speed
        if front_min < self.scan_stop_distance:
            angular += self.avoidance_side * self.avoid_turn_speed

        linear = self.avoid_arc_speed
        if front_min < self.scan_stop_distance:
            linear = max(self.avoid_arc_speed * 0.5, 0.03)

        if self.tracking_mode != 'follow':
            linear = min(linear, self.cinematic_avoid_arc_speed)

        return (
            self.clamp(linear, 0.03, self.max_linear_speed),
            self.clamp(angular, -self.max_angular_speed, self.max_angular_speed),
        )

    def scan_hit_matches_tracked_person(self, front_min):
        if not math.isfinite(front_min) or not math.isfinite(self.target_distance):
            return False
        if not self.target_visible:
            return False
        if abs(self.filtered_x_error) > 0.35:
            return False
        return front_min >= self.target_distance - 0.35

    def start_avoidance(self, now, left_min, right_min):
        self.avoidance_mode = 'avoid_turn'
        self.avoidance_started_time = now
        self.avoidance_side = 1.0 if left_min >= right_min else -1.0
        if not math.isfinite(left_min) and not math.isfinite(right_min):
            self.avoidance_side = 1.0 if self.target_x_error <= 0.0 else -1.0

        orbit_side = self.orbit_outward_avoidance_side(left_min, right_min)
        if orbit_side is not None:
            self.avoidance_side = orbit_side

    def orbit_outward_avoidance_side(self, left_min, right_min):
        if not self.tracking_mode.startswith('orbit'):
            return None
        if self.robot_pose is None:
            return None
        center = self.locked_person_pose or self.person_pose
        if center is None:
            return None

        robot_x, robot_y, robot_yaw = self.robot_pose
        center_x, center_y = center
        outward_x = robot_x - center_x
        outward_y = robot_y - center_y
        outward_norm = math.hypot(outward_x, outward_y)
        if outward_norm < 0.05:
            return None
        outward_x /= outward_norm
        outward_y /= outward_norm

        left_normal_x = -math.sin(robot_yaw)
        left_normal_y = math.cos(robot_yaw)
        left_score = left_normal_x * outward_x + left_normal_y * outward_y
        right_score = -left_score
        preferred_side = 1.0 if left_score >= right_score else -1.0

        preferred_clearance = left_min if preferred_side > 0.0 else right_min
        opposite_clearance = right_min if preferred_side > 0.0 else left_min
        if not math.isfinite(preferred_clearance):
            preferred_clearance = self.scan_clear_distance
        if not math.isfinite(opposite_clearance):
            opposite_clearance = self.scan_clear_distance

        if preferred_clearance >= self.scan_stop_distance:
            return preferred_side
        if preferred_clearance + 0.20 >= opposite_clearance:
            return preferred_side
        return None

    def reset_avoidance(self):
        self.avoidance_mode = 'follow_goal'
        self.avoidance_started_time = None
        self.avoidance_active = False

    def scan_sector_min(self, start_angle, end_angle):
        if self.latest_scan is None:
            return float('inf')

        scan = self.latest_scan
        lower = min(start_angle, end_angle)
        upper = max(start_angle, end_angle)
        best = float('inf')

        for index, value in enumerate(scan.ranges):
            if not math.isfinite(value):
                continue
            if value < scan.range_min or value > scan.range_max:
                continue
            angle = scan.angle_min + index * scan.angle_increment
            if lower <= angle <= upper:
                best = min(best, value)

        return best

    def apply_coordinate_obstacle_avoidance(self, linear, angular):
        if self.robot_pose is None or not self.obstacles:
            self.avoidance_active = False
            return linear, angular

        robot_x, robot_y, robot_yaw = self.robot_pose
        planned_linear = linear
        planned_angular = angular
        tracking_angular = angular
        self.avoidance_active = False

        for obstacle_x, obstacle_y, obstacle_radius in self.obstacles:
            forward, lateral = self.obstacle_in_robot_frame(
                robot_x,
                robot_y,
                robot_yaw,
                obstacle_x,
                obstacle_y,
            )
            corridor = self.avoidance_corridor_width + obstacle_radius
            edge_distance = forward - obstacle_radius
            if forward <= 0.0 or abs(lateral) > corridor:
                continue
            if edge_distance >= self.avoidance_slow_distance:
                continue

            self.avoidance_active = True
            strength = 1.0 - self.clamp(
                edge_distance / self.avoidance_slow_distance,
                0.0,
                1.0,
            )
            side = 1.0 if lateral >= 0.0 else -1.0
            if abs(lateral) < 0.05:
                side = 1.0 if self.target_x_error <= 0.0 else -1.0

            planned_angular += -side * self.avoidance_turn_gain * strength
            planned_angular = self.clamp(
                planned_angular,
                -self.max_angular_speed,
                self.max_angular_speed,
            )

            speed_scale = self.clamp(
                (edge_distance - self.avoidance_stop_distance)
                / max(
                    self.avoidance_slow_distance - self.avoidance_stop_distance,
                    0.01,
                ),
                0.0,
                1.0,
            )
            planned_linear = min(planned_linear, self.max_linear_speed * speed_scale)
            if edge_distance <= self.avoidance_stop_distance:
                planned_linear = min(planned_linear, 0.0)

        if (
            self.avoidance_active
            and abs(tracking_angular) > self.deadband_x
            and abs(planned_angular) < 0.15
        ):
            planned_angular = math.copysign(0.15, tracking_angular)

        return planned_linear, planned_angular
    @staticmethod
    def obstacle_in_robot_frame(robot_x, robot_y, robot_yaw, obstacle_x, obstacle_y):
        dx = obstacle_x - robot_x
        dy = obstacle_y - robot_y
        cos_yaw = math.cos(robot_yaw)
        sin_yaw = math.sin(robot_yaw)
        forward = cos_yaw * dx + sin_yaw * dy
        lateral = -sin_yaw * dx + cos_yaw * dy
        return forward, lateral

    def is_target_recent(self, now):
        if not self.target_visible or self.last_target_time is None:
            return False

        age = (now - self.last_target_time).nanoseconds / 1_000_000_000
        return age <= self.target_timeout

    def is_target_recently_lost(self, now):
        if self.last_target_time is None:
            return False

        age = (now - self.last_target_time).nanoseconds / 1_000_000_000
        return self.target_timeout < age <= self.lost_target_timeout

    def mode_index(self):
        if self.tracking_mode == 'follow':
            return 0.0
        if self.tracking_mode == 'orbit_cw':
            return 1.0
        if self.tracking_mode == 'orbit_ccw':
            return 2.0
        if self.tracking_mode in ('side_left', 'side_right'):
            return 3.0
        return -1.0
    def log_state(self, target_available, target_recently_lost):
        if not target_available:
            state = (
                'pursuing last seen target'
                if target_recently_lost
                else 'waiting for target'
            )
        elif self.avoidance_active:
            state = 'local planner avoiding obstacle'
        elif self.tracking_mode != 'follow':
            state = '%s cinematic tracking' % self.tracking_mode
        elif self.current_linear > 0.02:
            state = 'following forward'
        elif self.current_linear < -0.02:
            state = 'backing up to safe distance'
        elif abs(self.current_angular) > 0.02:
            state = 'centering target'
        else:
            state = 'holding safe distance'

        if state != self.last_state:
            self.get_logger().info('Following state: %s' % state)
            self.last_state = state

    @staticmethod
    def slew(current, target, rate_limit, dt):
        max_step = max(rate_limit, 0.0) * max(dt, 0.0)
        delta = target - current
        if delta > max_step:
            return current + max_step
        if delta < -max_step:
            return current - max_step
        return target

    def slew_angular(self, current, target, dt):
        braking = abs(target) < abs(current) or current * target < 0.0
        rate_limit = self.angular_decel_limit if braking else self.angular_accel_limit
        next_value = self.slew(current, target, rate_limit, dt)
        if abs(target) < 0.03 and abs(next_value) < 0.06:
            return 0.0
        return next_value

    @staticmethod
    def clamp(value, lower, upper):
        return max(lower, min(upper, value))

    @staticmethod
    def normalize_angle(angle):
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

    def interpolate_angle(self, start, end, blend):
        return self.normalize_angle(
            start + self.normalize_angle(end - start) * self.clamp(blend, 0.0, 1.0)
        )
    @staticmethod
    def parse_obstacles(values):
        if values is None:
            return []

        values = [float(value) for value in values]
        obstacles = []
        for index in range(0, len(values) - 2, 3):
            obstacles.append((values[index], values[index + 1], values[index + 2]))
        return obstacles

    @staticmethod
    def quaternion_to_yaw(x, y, z, w):
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        return math.atan2(siny_cosp, cosy_cosp)


def main(args=None):
    rclpy.init(args=args)
    node = PersonFollowing()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
