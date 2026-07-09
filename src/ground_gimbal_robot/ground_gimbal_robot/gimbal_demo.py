import math

import rclpy
from gazebo_msgs.msg import ModelStates
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import String


class PidAxis:
    """Small PID helper for image-space gimbal stabilization."""

    def __init__(self, kp, ki, kd, output_limit):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.output_limit = output_limit
        self.integral = 0.0
        self.previous_error = 0.0

    def update(self, error, dt):
        self.integral += error * dt
        derivative = (error - self.previous_error) / dt if dt > 0.0 else 0.0
        self.previous_error = error

        output = self.kp * error + self.ki * self.integral + self.kd * derivative
        return max(-self.output_limit, min(self.output_limit, output))


class GimbalDemo(Node):
    """Aim the pan/tilt camera at the tracked subject for Week 6 filming."""

    def __init__(self):
        super().__init__('gimbal_demo')

        self.declare_parameter('tracking_source', 'odom')
        self.declare_parameter('target_timeout', 0.75)
        self.declare_parameter('desired_x_offset', 0.0)
        self.declare_parameter('desired_y_offset', 0.0)
        self.declare_parameter('deadband_x', 0.045)
        self.declare_parameter('deadband_y', 0.055)
        self.declare_parameter('error_filter_alpha', 0.55)
        self.declare_parameter('publish_joint_states', True)
        self.declare_parameter(
            'gazebo_command_topic',
            '/gimbal_position_controller/commands',
        )
        self.declare_parameter('robot_odom_topic', '/odom')
        self.declare_parameter('target_odom_topic', '/tracking_subject/odom')
        self.declare_parameter('mode_command_topic', '/tracking_mode_cmd')
        self.declare_parameter('model_states_topic', '/gazebo/model_states')
        self.declare_parameter('robot_model_name', 'ground_gimbal_robot')
        self.declare_parameter('target_model_name', 'tracking_subject')
        self.declare_parameter('camera_height', 0.55)
        self.declare_parameter('target_height', 0.82)
        self.declare_parameter('gimbal_forward_offset', 0.08)
        self.declare_parameter('gimbal_lateral_offset', 0.0)
        self.declare_parameter('pan_offset', 0.0)
        self.declare_parameter('tilt_offset', 0.0)
        self.declare_parameter('pan_sign', 1.0)
        self.declare_parameter('tilt_sign', -1.0)
        self.declare_parameter('pan_rate_limit', 2.8)
        self.declare_parameter('tilt_rate_limit', 1.8)
        self.declare_parameter('enable_search', True)
        self.declare_parameter('search_pan_limit', 1.35)
        self.declare_parameter('search_period', 5.0)
        self.declare_parameter('search_tilt_angle', 0.0)

        self.tracking_source = self.get_parameter('tracking_source').value
        self.target_timeout = float(self.get_parameter('target_timeout').value)
        self.desired_x_offset = float(
            self.get_parameter('desired_x_offset').value
        )
        self.desired_y_offset = float(
            self.get_parameter('desired_y_offset').value
        )
        self.deadband_x = float(self.get_parameter('deadband_x').value)
        self.deadband_y = float(self.get_parameter('deadband_y').value)
        self.error_filter_alpha = float(
            self.get_parameter('error_filter_alpha').value
        )
        self.publish_joint_states = bool(
            self.get_parameter('publish_joint_states').value
        )
        self.gazebo_command_topic = self.get_parameter('gazebo_command_topic').value
        self.robot_odom_topic = self.get_parameter('robot_odom_topic').value
        self.target_odom_topic = self.get_parameter('target_odom_topic').value
        self.mode_command_topic = self.get_parameter('mode_command_topic').value
        self.model_states_topic = self.get_parameter('model_states_topic').value
        self.robot_model_name = self.get_parameter('robot_model_name').value
        self.target_model_name = self.get_parameter('target_model_name').value
        self.camera_height = float(self.get_parameter('camera_height').value)
        self.target_height = float(self.get_parameter('target_height').value)
        self.gimbal_forward_offset = float(
            self.get_parameter('gimbal_forward_offset').value
        )
        self.gimbal_lateral_offset = float(
            self.get_parameter('gimbal_lateral_offset').value
        )
        self.pan_offset = float(self.get_parameter('pan_offset').value)
        self.tilt_offset = float(self.get_parameter('tilt_offset').value)
        self.pan_sign = float(self.get_parameter('pan_sign').value)
        self.tilt_sign = float(self.get_parameter('tilt_sign').value)
        self.pan_rate_limit = float(self.get_parameter('pan_rate_limit').value)
        self.tilt_rate_limit = float(self.get_parameter('tilt_rate_limit').value)
        self.enable_search = bool(self.get_parameter('enable_search').value)
        self.search_pan_limit = float(self.get_parameter('search_pan_limit').value)
        self.search_period = float(self.get_parameter('search_period').value)
        self.search_tilt_angle = float(self.get_parameter('search_tilt_angle').value)

        self.joint_state_publisher = None
        if self.publish_joint_states:
            self.joint_state_publisher = self.create_publisher(
                JointState, 'joint_states', 10
            )
        self.gazebo_command_publisher = None
        if self.gazebo_command_topic:
            self.gazebo_command_publisher = self.create_publisher(
                Float64MultiArray, self.gazebo_command_topic, 10
            )
        self.error_publisher = None
        if self.tracking_source == 'simulated':
            self.error_publisher = self.create_publisher(
                Float64MultiArray, 'gimbal/target_offset', 10
            )
        self.command_publisher = self.create_publisher(
            Float64MultiArray, 'gimbal/pid_command', 10
        )
        self.target_offset_subscription = None
        self.target_visible_subscription = None
        if self.tracking_source == 'topic':
            self.target_offset_subscription = self.create_subscription(
                Float64MultiArray,
                'gimbal/target_offset',
                self.update_target_offset,
                10,
            )
            self.target_visible_subscription = self.create_subscription(
                Bool,
                'gimbal/target_visible',
                self.update_target_visible,
                10,
            )
        self.mode_command_subscription = self.create_subscription(
            String,
            self.mode_command_topic,
            self.update_mode_command,
            10,
        )
        self.model_states_subscription = self.create_subscription(
            ModelStates,
            self.model_states_topic,
            self.update_model_states,
            10,
        )
        self.robot_odom_subscription = None
        self.target_odom_subscription = None
        if self.tracking_source == 'odom':
            self.robot_odom_subscription = self.create_subscription(
                Odometry,
                self.robot_odom_topic,
                self.update_robot_odom,
                10,
            )
            self.target_odom_subscription = self.create_subscription(
                Odometry,
                self.target_odom_topic,
                self.update_target_odom,
                10,
            )

        self.pan_pid = PidAxis(kp=0.20, ki=0.0, kd=0.015, output_limit=0.010)
        self.tilt_pid = PidAxis(kp=0.18, ki=0.0, kd=0.012, output_limit=0.008)

        self.pan_angle = 0.0
        self.tilt_angle = 0.0
        self.pan_limit = 4.0 * math.pi
        self.tilt_limit = math.radians(45.0)
        self.start_time = self.get_clock().now()
        self.last_time = self.start_time
        self.target_x_error = 0.0
        self.target_y_error = 0.0
        self.filtered_x_error = 0.0
        self.filtered_y_error = 0.0
        self.robot_pose = None
        self.target_pose = None
        self.model_state_target = None
        self.target_visible = False
        self.current_mode = 'follow'
        self.last_target_time = None
        self.last_tracking_state = None
        self.timer = self.create_timer(0.05, self.publish_gimbal_state)

        self.get_logger().info(
            'Starting gimbal controller: tracking_source=%s' % self.tracking_source
        )

    def update_mode_command(self, msg):
        requested = str(msg.data).strip().lower()
        aliases = {
            'orbitcw': 'orbit_cw',
            'cw': 'orbit_cw',
            'orbitccw': 'orbit_ccw',
            'ccw': 'orbit_ccw',
            'side': 'side_tracking',
            'sidetracking': 'side_tracking',
        }
        self.current_mode = aliases.get(requested, requested)
        if self.current_mode == 'follow':
            self.model_state_target = (0.0, 0.0)
            self.target_visible = True
            self.last_target_time = self.get_clock().now()
        self.get_logger().info('Gimbal mode command: %s' % self.current_mode)

    def update_model_states(self, msg):
        if self.current_mode == 'follow':
            return
        try:
            robot_index = msg.name.index(self.robot_model_name)
            target_index = msg.name.index(self.target_model_name)
        except ValueError:
            return

        robot_pose = msg.pose[robot_index]
        target_pose = msg.pose[target_index]
        robot_yaw = self.quaternion_to_yaw(robot_pose.orientation)
        self.robot_pose = (
            robot_pose.position.x,
            robot_pose.position.y,
            robot_pose.position.z,
            robot_yaw,
        )
        self.target_pose = (
            target_pose.position.x,
            target_pose.position.y,
            target_pose.position.z,
        )
        self.update_odom_target()
    def update_robot_odom(self, msg):
        pose = msg.pose.pose
        self.robot_pose = (
            pose.position.x,
            pose.position.y,
            pose.position.z,
            self.quaternion_to_yaw(pose.orientation),
        )
        self.update_odom_target()

    def update_target_odom(self, msg):
        pose = msg.pose.pose
        self.target_pose = (
            pose.position.x,
            pose.position.y,
            pose.position.z,
        )
        self.update_odom_target()

    def update_odom_target(self):
        if self.robot_pose is None or self.target_pose is None:
            return

        robot_x, robot_y, robot_z, robot_yaw = self.robot_pose
        target_x, target_y, target_z_raw = self.target_pose
        cos_yaw = math.cos(robot_yaw)
        sin_yaw = math.sin(robot_yaw)
        gimbal_x = (
            robot_x
            + cos_yaw * self.gimbal_forward_offset
            - sin_yaw * self.gimbal_lateral_offset
        )
        gimbal_y = (
            robot_y
            + sin_yaw * self.gimbal_forward_offset
            + cos_yaw * self.gimbal_lateral_offset
        )
        dx = target_x - gimbal_x
        dy = target_y - gimbal_y
        horizontal_distance = max(math.hypot(dx, dy), 0.05)
        target_z = target_z_raw + self.target_height
        camera_z = robot_z + self.camera_height
        dz = target_z - camera_z

        world_bearing = math.atan2(dy, dx)
        relative_bearing = self.normalize_angle(world_bearing - robot_yaw)
        desired_pan = self.pan_sign * relative_bearing + self.pan_offset
        desired_tilt = self.tilt_sign * math.atan2(dz, horizontal_distance)
        desired_tilt += self.tilt_offset
        self.model_state_target = (
            self.clamp(desired_pan, -self.pan_limit, self.pan_limit),
            self.clamp(desired_tilt, -self.tilt_limit, self.tilt_limit),
        )
        self.target_visible = True
        self.last_target_time = self.get_clock().now()

    def update_target_offset(self, msg):
        if len(msg.data) < 2:
            return

        raw_x_error = self.clamp(float(msg.data[0]), -1.0, 1.0)
        raw_y_error = self.clamp(float(msg.data[1]), -1.0, 1.0)
        alpha = self.clamp(self.error_filter_alpha, 0.0, 1.0)
        self.filtered_x_error = (
            alpha * raw_x_error + (1.0 - alpha) * self.filtered_x_error
        )
        self.filtered_y_error = (
            alpha * raw_y_error + (1.0 - alpha) * self.filtered_y_error
        )
        self.target_x_error = self.filtered_x_error
        self.target_y_error = self.filtered_y_error
        self.target_visible = True
        self.last_target_time = self.get_clock().now()

    def update_target_visible(self, msg):
        if msg.data and self.tracking_source not in ('odom', 'model_states'):
            self.target_visible = True
            self.last_target_time = self.get_clock().now()
        # Do not clear target_visible on a single missed frame; timeout handles loss.

    def publish_gimbal_state(self):
        now = self.get_clock().now()
        elapsed = (now - self.start_time).nanoseconds / 1_000_000_000
        dt = (now - self.last_time).nanoseconds / 1_000_000_000
        self.last_time = now

        target_available = self.tracking_source == 'simulated'
        pan_step = 0.0
        tilt_step = 0.0
        if self.current_mode == 'follow':
            target_available = True
            previous_pan = self.pan_angle
            previous_tilt = self.tilt_angle
            self.pan_angle = self.slew_angle(
                self.pan_angle,
                0.0,
                self.pan_rate_limit,
                dt,
            )
            self.tilt_angle = self.slew(
                self.tilt_angle,
                0.0,
                self.tilt_rate_limit,
                dt,
            )
            pan_step = self.normalize_angle(self.pan_angle - previous_pan)
            tilt_step = self.tilt_angle - previous_tilt
            target_x_error = 0.0
            target_y_error = 0.0
        elif self.current_mode != 'follow' and self.model_state_target is not None:
            target_available = self.is_target_recent(now)
            if target_available:
                desired_pan, desired_tilt = self.model_state_target
                previous_pan = self.pan_angle
                previous_tilt = self.tilt_angle
                self.pan_angle = self.slew_angle(
                    self.pan_angle,
                    desired_pan,
                    self.pan_rate_limit,
                    dt,
                )
                self.tilt_angle = self.slew(
                    self.tilt_angle,
                    desired_tilt,
                    self.tilt_rate_limit,
                    dt,
                )
                pan_step = self.normalize_angle(self.pan_angle - previous_pan)
                tilt_step = self.tilt_angle - previous_tilt
            target_x_error = 0.0
            target_y_error = 0.0
        elif self.tracking_source == 'simulated':
            target_x_error = 0.45 * math.sin(0.55 * elapsed)
            target_y_error = 0.30 * math.sin(0.38 * elapsed + 0.9)
        else:
            target_available = self.is_target_recent(now)
            target_x_error = self.target_x_error if target_available else 0.0
            target_y_error = self.target_y_error if target_available else 0.0

        search_active = (
            self.tracking_source == 'topic'
            and self.enable_search
            and not target_available
        )

        if self.tracking_source not in ('odom', 'model_states') and target_available:
            framing_x_error = target_x_error - self.desired_x_offset
            framing_y_error = target_y_error - self.desired_y_offset
            if abs(framing_x_error) > self.deadband_x:
                pan_step = self.pan_pid.update(-framing_x_error, dt)
            if abs(framing_y_error) > self.deadband_y:
                tilt_step = self.tilt_pid.update(framing_y_error, dt)

            self.pan_angle = self.clamp(
                self.pan_angle + pan_step, -self.pan_limit, self.pan_limit
            )
            self.tilt_angle = self.clamp(
                self.tilt_angle + tilt_step, -self.tilt_limit, self.tilt_limit
            )
        elif search_active:
            desired_pan = self.search_pan_limit * math.sin(
                2.0 * math.pi * elapsed / max(self.search_period, 0.1)
            )
            desired_tilt = self.search_tilt_angle
            previous_pan = self.pan_angle
            previous_tilt = self.tilt_angle
            self.pan_angle = self.slew_angle(
                self.pan_angle,
                self.clamp(desired_pan, -self.pan_limit, self.pan_limit),
                self.pan_rate_limit,
                dt,
            )
            self.tilt_angle = self.slew(
                self.tilt_angle,
                self.clamp(desired_tilt, -self.tilt_limit, self.tilt_limit),
                self.tilt_rate_limit,
                dt,
            )
            pan_step = self.normalize_angle(self.pan_angle - previous_pan)
            tilt_step = self.tilt_angle - previous_tilt

        self.log_tracking_state(target_available, search_active)

        if self.joint_state_publisher is not None:
            joint_state = JointState()
            joint_state.header.stamp = now.to_msg()
            joint_state.name = [
                'left_wheel_joint',
                'right_wheel_joint',
                'gimbal_pan_joint',
                'gimbal_tilt_joint',
            ]
            joint_state.position = [0.0, 0.0, self.pan_angle, self.tilt_angle]
            joint_state.velocity = [
                0.0,
                0.0,
                pan_step / dt if dt > 0.0 else 0.0,
                tilt_step / dt if dt > 0.0 else 0.0,
            ]
            self.joint_state_publisher.publish(joint_state)

        if self.error_publisher is not None:
            error_msg = Float64MultiArray()
            error_msg.data = [target_x_error, target_y_error]
            self.error_publisher.publish(error_msg)

        command_msg = Float64MultiArray()
        command_msg.data = [self.pan_angle, self.tilt_angle]
        self.command_publisher.publish(command_msg)
        if self.gazebo_command_publisher is not None:
            self.gazebo_command_publisher.publish(command_msg)

    def is_target_recent(self, now):
        if not self.target_visible or self.last_target_time is None:
            return False
        age = (now - self.last_target_time).nanoseconds / 1_000_000_000
        return age <= self.target_timeout

    def log_tracking_state(self, target_available, search_active=False):
        if target_available:
            state = 'tracking'
        elif search_active:
            state = 'searching for target'
        else:
            state = 'waiting for target'
        if state != self.last_tracking_state:
            self.get_logger().info('Gimbal state: %s' % state)
            self.last_tracking_state = state
    @staticmethod
    def quaternion_to_yaw(quaternion):
        siny_cosp = 2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y)
        cosy_cosp = 1.0 - 2.0 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z)
        return math.atan2(siny_cosp, cosy_cosp)

    @staticmethod
    def slew(current, target, rate_limit, dt):
        max_step = max(rate_limit, 0.0) * max(dt, 0.0)
        delta = target - current
        if delta > max_step:
            return current + max_step
        if delta < -max_step:
            return current - max_step
        return target

    @classmethod
    def slew_angle(cls, current, target, rate_limit, dt):
        delta = cls.normalize_angle(target - current)
        next_angle = current + cls.slew(0.0, delta, rate_limit, dt)
        return cls.normalize_angle(next_angle)

    @staticmethod
    def normalize_angle(value):
        while value > math.pi:
            value -= 2.0 * math.pi
        while value < -math.pi:
            value += 2.0 * math.pi
        return value

    @staticmethod
    def clamp(value, lower, upper):
        return max(lower, min(upper, value))


def main(args=None):
    rclpy.init(args=args)
    node = GimbalDemo()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()