import math

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Bool
from std_msgs.msg import Float64MultiArray


class PersonFollowing(Node):
    """Convert camera target offset into smooth mobile-base velocity commands."""

    def __init__(self):
        super().__init__('person_following')

        self.declare_parameter('target_offset_topic', 'gimbal/target_offset')
        self.declare_parameter('target_visible_topic', 'gimbal/target_visible')
        self.declare_parameter('odom_topic', 'odom')
        self.declare_parameter('cmd_vel_topic', 'cmd_vel')
        self.declare_parameter('control_topic', 'person_following/control_state')
        self.declare_parameter('control_rate', 10.0)
        self.declare_parameter('target_timeout', 0.75)
        self.declare_parameter('lost_target_timeout', 1.0)
        self.declare_parameter('lost_target_speed', 0.36)
        self.declare_parameter('lost_target_turn_speed', 0.70)
        self.declare_parameter('lost_target_turn_deadband', 0.25)
        self.declare_parameter('turn_kp', 0.82)
        self.declare_parameter('turn_prediction_time', 0.15)
        self.declare_parameter('distance_kp', 0.90)
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
        self.avoidance_active = False
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
        self.cmd_publisher = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        self.control_publisher = self.create_publisher(
            Float64MultiArray, self.control_topic, 10
        )

        timer_period = 1.0 / max(self.control_rate, 1.0)
        self.timer = self.create_timer(timer_period, self.publish_command)

        self.get_logger().info(
            'Starting Week 5 person following: target_offset=%s cmd_vel=%s '
            'obstacle_avoidance=%s'
            % (
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

        self.target_visible = True
        self.last_target_time = now

    def update_target_visible(self, msg):
        if not msg.data:
            self.target_visible = False

    def publish_command(self):
        now = self.get_clock().now()
        dt = (now - self.last_timer_time).nanoseconds / 1_000_000_000
        self.last_timer_time = now

        target_available = self.is_target_recent(now)
        target_recently_lost = self.is_target_recently_lost(now)
        desired_linear = 0.0
        desired_angular = 0.0

        if target_available:
            desired_angular = self.compute_angular_command(
                self.predicted_x_error()
            )
            desired_linear = self.compute_linear_command(
                self.filtered_x_error, self.target_confidence
            )
            desired_linear = self.keep_forward_motion_on_turn(
                desired_linear,
                self.filtered_x_error,
            )
            desired_linear, desired_angular = self.apply_local_planner(
                desired_linear,
                desired_angular,
            )
        elif target_recently_lost:
            desired_linear = self.lost_target_speed
            desired_angular = self.compute_lost_target_angular_command()

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
            1.0 if target_available else 0.0,
            1.0 if self.avoidance_active else 0.0,
            1.0 if target_recently_lost else 0.0,
        ]
        self.control_publisher.publish(control_msg)

        self.log_state(target_available, target_recently_lost)

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

    def compute_linear_command(self, x_error, confidence):
        if abs(x_error) > self.turn_before_forward_error:
            return self.curve_follow_speed

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
        if (
            not self.enable_obstacle_avoidance
            or self.robot_pose is None
            or not self.obstacles
        ):
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

    def log_state(self, target_available, target_recently_lost):
        if not target_available:
            state = (
                'pursuing last seen target'
                if target_recently_lost
                else 'waiting for target'
            )
        elif self.avoidance_active:
            state = 'local planner avoiding obstacle'
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
