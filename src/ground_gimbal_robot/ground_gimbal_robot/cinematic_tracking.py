import math

from gazebo_msgs.msg import EntityState
from gazebo_msgs.msg import ModelStates
from gazebo_msgs.srv import SetEntityState
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Bool
from std_msgs.msg import Float64MultiArray


class CinematicTracking(Node):
    """Week 6 cinematic base controller for orbit, follow-behind, and side shots."""

    def __init__(self):
        super().__init__('cinematic_tracking')

        self.declare_parameter('mode', 'showcase')
        self.declare_parameter('tracking_source', 'model_states')
        self.declare_parameter('showcase_period', 14.0)
        self.declare_parameter('model_states_topic', '/gazebo/model_states')
        self.declare_parameter('robot_model_name', 'ground_gimbal_robot')
        self.declare_parameter('target_model_name', 'tracking_subject')
        self.declare_parameter('target_offset_topic', 'gimbal/target_offset')
        self.declare_parameter('target_visible_topic', 'gimbal/target_visible')
        self.declare_parameter('odom_topic', 'odom')
        self.declare_parameter('target_odom_topic', '/tracking_subject/odom')
        self.declare_parameter('cmd_vel_topic', 'cmd_vel')
        self.declare_parameter('control_topic', 'cinematic_tracking/control_state')
        self.declare_parameter('direct_gazebo_drive', True)
        self.declare_parameter('set_entity_state_service', '/gazebo/set_entity_state')
        self.declare_parameter('control_rate', 15.0)
        self.declare_parameter('target_timeout', 1.2)
        self.declare_parameter('lost_target_timeout', 2.5)
        self.declare_parameter('orbit_radius', 1.20)
        self.declare_parameter('follow_distance', 1.35)
        self.declare_parameter('side_distance', 1.35)
        self.declare_parameter('side_angle_deg', 82.0)
        self.declare_parameter('orbit_speed', 0.18)
        self.declare_parameter('side_speed', 0.22)
        self.declare_parameter('follow_speed', 0.24)
        self.declare_parameter('max_linear_speed', 0.24)
        self.declare_parameter('max_reverse_speed', 0.16)
        self.declare_parameter('max_angular_speed', 0.45)
        self.declare_parameter('linear_accel_limit', 0.55)
        self.declare_parameter('angular_accel_limit', 1.35)
        self.declare_parameter('heading_kp', 1.45)
        self.declare_parameter('distance_kp', 0.28)
        self.declare_parameter('orbit_turn_gain', 1.35)
        self.declare_parameter('search_turn_speed', 0.34)
        self.declare_parameter('search_creep_speed', 0.05)
        self.declare_parameter('enable_obstacle_avoidance', False)
        self.declare_parameter('obstacles', [0.95, -0.85, 0.28, 1.80, 0.85, 0.30])
        self.declare_parameter('obstacle_inflation_radius', 0.35)
        self.declare_parameter('avoidance_orbit_radius_offset', 0.55)
        self.declare_parameter('avoidance_turn_gain', 0.65)

        self.mode = self.get_parameter('mode').value
        self.tracking_source = self.get_parameter('tracking_source').value
        self.showcase_period = float(self.get_parameter('showcase_period').value)
        self.model_states_topic = self.get_parameter('model_states_topic').value
        self.robot_model_name = self.get_parameter('robot_model_name').value
        self.target_model_name = self.get_parameter('target_model_name').value
        self.target_offset_topic = self.get_parameter('target_offset_topic').value
        self.target_visible_topic = self.get_parameter('target_visible_topic').value
        self.odom_topic = self.get_parameter('odom_topic').value
        self.target_odom_topic = self.get_parameter('target_odom_topic').value
        self.cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        self.control_topic = self.get_parameter('control_topic').value
        self.direct_gazebo_drive = bool(self.get_parameter('direct_gazebo_drive').value)
        self.set_entity_state_service = self.get_parameter('set_entity_state_service').value
        self.control_rate = float(self.get_parameter('control_rate').value)
        self.target_timeout = float(self.get_parameter('target_timeout').value)
        self.lost_target_timeout = float(self.get_parameter('lost_target_timeout').value)
        self.orbit_radius = float(self.get_parameter('orbit_radius').value)
        self.follow_distance = float(self.get_parameter('follow_distance').value)
        self.side_distance = float(self.get_parameter('side_distance').value)
        self.side_angle = math.radians(float(self.get_parameter('side_angle_deg').value))
        self.orbit_speed = float(self.get_parameter('orbit_speed').value)
        self.side_speed = float(self.get_parameter('side_speed').value)
        self.follow_speed = float(self.get_parameter('follow_speed').value)
        self.max_linear_speed = float(self.get_parameter('max_linear_speed').value)
        self.max_reverse_speed = float(self.get_parameter('max_reverse_speed').value)
        self.max_angular_speed = float(self.get_parameter('max_angular_speed').value)
        self.linear_accel_limit = float(self.get_parameter('linear_accel_limit').value)
        self.angular_accel_limit = float(self.get_parameter('angular_accel_limit').value)
        self.heading_kp = float(self.get_parameter('heading_kp').value)
        self.distance_kp = float(self.get_parameter('distance_kp').value)
        self.orbit_turn_gain = float(self.get_parameter('orbit_turn_gain').value)
        self.search_turn_speed = float(self.get_parameter('search_turn_speed').value)
        self.search_creep_speed = float(self.get_parameter('search_creep_speed').value)
        self.enable_obstacle_avoidance = bool(self.get_parameter('enable_obstacle_avoidance').value)
        self.obstacles = self.parse_obstacles(self.get_parameter('obstacles').value)
        self.obstacle_inflation_radius = float(self.get_parameter('obstacle_inflation_radius').value)
        self.avoidance_orbit_radius_offset = float(self.get_parameter('avoidance_orbit_radius_offset').value)
        self.avoidance_turn_gain = float(self.get_parameter('avoidance_turn_gain').value)

        self.filtered_x_error = 0.0
        self.filtered_y_error = 0.0
        self.filtered_confidence = 0.0
        self.filtered_distance = -1.0
        self.last_seen_x_error = 0.0
        self.target_visible = False
        self.last_target_time = None
        self.last_timer_time = self.get_clock().now()
        self.start_time = self.last_timer_time
        self.current_linear = 0.0
        self.current_angular = 0.0
        self.robot_pose = None
        self.target_pose = None
        self.orbit_direction = -1.0
        self.last_state = None
        self.warned_model_names = False
        self.warned_direct_drive_service = False
        self.avoidance_active = False
        self.direct_drive_future = None

        self.create_subscription(Odometry, self.odom_topic, self.update_odom, 10)
        self.create_subscription(Odometry, self.target_odom_topic, self.update_target_odom, 10)
        self.create_subscription(ModelStates, self.model_states_topic, self.update_model_states, 10)
        self.create_subscription(Float64MultiArray, self.target_offset_topic, self.update_target_offset, 10)
        self.create_subscription(Bool, self.target_visible_topic, self.update_target_visible, 10)
        self.cmd_publisher = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        self.control_publisher = self.create_publisher(Float64MultiArray, self.control_topic, 10)
        self.set_entity_state_client = None
        if self.direct_gazebo_drive:
            self.set_entity_state_client = self.create_client(SetEntityState, self.set_entity_state_service)
        self.timer = self.create_timer(1.0 / max(self.control_rate, 1.0), self.publish_command)

        self.get_logger().info(
            'Starting Week 6 cinematic tracking: mode=%s tracking_source=%s'
            % (self.mode, self.tracking_source)
        )

    def update_model_states(self, msg):
        robot_index = self.find_model_index(msg.name, self.robot_model_name)
        target_index = self.find_model_index(msg.name, self.target_model_name)
        if robot_index is None or target_index is None:
            if not self.warned_model_names:
                self.get_logger().warning(
                    'Waiting for model states. Wanted robot=%s target=%s; available=%s'
                    % (self.robot_model_name, self.target_model_name, ', '.join(msg.name))
                )
                self.warned_model_names = True
            return

        robot_pose = msg.pose[robot_index]
        target_pose = msg.pose[target_index]
        yaw = self.quaternion_to_yaw(
            robot_pose.orientation.x,
            robot_pose.orientation.y,
            robot_pose.orientation.z,
            robot_pose.orientation.w,
        )
        self.robot_pose = (robot_pose.position.x, robot_pose.position.y, robot_pose.position.z, yaw)
        self.target_pose = (target_pose.position.x, target_pose.position.y)
        self.target_visible = True
        self.last_target_time = self.get_clock().now()

    def update_odom(self, msg):
        if self.tracking_source == 'model_states':
            return
        orientation = msg.pose.pose.orientation
        yaw = self.quaternion_to_yaw(orientation.x, orientation.y, orientation.z, orientation.w)
        self.robot_pose = (msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z, yaw)

    def update_target_odom(self, msg):
        if self.tracking_source == 'model_states':
            return
        self.target_pose = (msg.pose.pose.position.x, msg.pose.pose.position.y)
        self.target_visible = True
        self.last_target_time = self.get_clock().now()

    def update_target_offset(self, msg):
        if len(msg.data) < 2:
            return
        self.filtered_x_error = self.clamp(float(msg.data[0]), -1.0, 1.0)
        self.filtered_y_error = self.clamp(float(msg.data[1]), -1.0, 1.0)
        if len(msg.data) >= 3:
            self.filtered_confidence = self.clamp(float(msg.data[2]), 0.0, 1.0)
        if len(msg.data) >= 4 and math.isfinite(float(msg.data[3])) and float(msg.data[3]) > 0.0:
            self.filtered_distance = float(msg.data[3])
        self.last_seen_x_error = self.filtered_x_error
        if self.tracking_source not in ('odom', 'model_states'):
            self.target_visible = True
            self.last_target_time = self.get_clock().now()

    def update_target_visible(self, msg):
        if self.tracking_source not in ('odom', 'model_states') and not msg.data:
            self.target_visible = False

    def publish_command(self):
        now = self.get_clock().now()
        dt = (now - self.last_timer_time).nanoseconds / 1_000_000_000
        self.last_timer_time = now

        target_available = self.is_target_recent(now)
        recently_lost = self.is_target_recently_lost(now)
        desired_linear = 0.0
        desired_angular = 0.0

        if target_available and self.robot_pose and self.target_pose:
            desired_linear, desired_angular = self.compute_odom_command()
        elif recently_lost:
            desired_linear, desired_angular = self.compute_search_command()

        self.current_linear = self.slew(self.current_linear, desired_linear, self.linear_accel_limit, dt)
        self.current_angular = self.slew(self.current_angular, desired_angular, self.angular_accel_limit, dt)

        twist = Twist()
        twist.linear.x = self.current_linear
        twist.angular.z = self.current_angular
        self.cmd_publisher.publish(twist)
        self.drive_gazebo_model_directly(self.current_linear, self.current_angular, dt)

        control_msg = Float64MultiArray()
        control_msg.data = [
            self.current_linear,
            self.current_angular,
            self.filtered_x_error,
            self.filtered_y_error,
            self.filtered_confidence,
            self.subject_distance(),
            1.0 if target_available else 0.0,
            1.0 if self.avoidance_active else 0.0,
            self.mode_index(),
            self.orbit_direction,
        ]
        self.control_publisher.publish(control_msg)
        self.log_state(target_available, recently_lost)

    def drive_gazebo_model_directly(self, linear, angular, dt):
        if not self.direct_gazebo_drive or self.tracking_source != 'model_states':
            return
        if not self.set_entity_state_client or not self.robot_pose or not self.target_pose:
            return
        if dt <= 0.0:
            return
        if not self.set_entity_state_client.service_is_ready():
            if not self.warned_direct_drive_service:
                self.get_logger().warning(
                    'Waiting for %s; /cmd_vel is still being published.'
                    % self.set_entity_state_service
                )
                self.warned_direct_drive_service = True
            return
        if self.direct_drive_future is not None and not self.direct_drive_future.done():
            return

        robot_x, robot_y, robot_z, robot_yaw = self.robot_pose
        target_x, target_y = self.target_pose
        mode = self.active_mode()

        self.avoidance_active = False
        if mode == 'orbit' and not self.enable_obstacle_avoidance:
            current_angle = math.atan2(robot_y - target_y, robot_x - target_x)
            angular_rate = self.orbit_direction * abs(self.orbit_speed / max(self.orbit_radius, 0.05))
            next_angle = current_angle + angular_rate * dt
            new_x = target_x + self.orbit_radius * math.cos(next_angle)
            new_y = target_y + self.orbit_radius * math.sin(next_angle)
            new_yaw = self.normalize_angle(next_angle + self.orbit_direction * math.pi / 2.0)
        else:
            new_yaw = self.normalize_angle(robot_yaw + angular * dt)
            distance = linear * dt
            mid_yaw = robot_yaw + 0.5 * angular * dt
            new_x = robot_x + distance * math.cos(mid_yaw)
            new_y = robot_y + distance * math.sin(mid_yaw)

        state = EntityState()
        state.name = self.robot_model_name
        state.pose.position.x = new_x
        state.pose.position.y = new_y
        state.pose.position.z = 0.0
        qx, qy, qz, qw = self.yaw_to_quaternion(new_yaw)
        state.pose.orientation.x = qx
        state.pose.orientation.y = qy
        state.pose.orientation.z = qz
        state.pose.orientation.w = qw
        state.reference_frame = 'world'

        request = SetEntityState.Request()
        request.state = state
        self.direct_drive_future = self.set_entity_state_client.call_async(request)
    def compute_odom_command(self):
        robot_x, robot_y, _, robot_yaw = self.robot_pose
        target_x, target_y = self.target_pose
        dx = target_x - robot_x
        dy = target_y - robot_y
        distance = max(math.hypot(dx, dy), 0.05)
        bearing_to_target = math.atan2(dy, dx)
        mode = self.active_mode()

        if mode == 'orbit':
            radial_error = distance - self.orbit_radius
            linear = self.orbit_speed + 0.08 * radial_error
            angular = -self.orbit_direction * (self.orbit_speed / self.orbit_radius)
            angular += -0.12 * self.orbit_direction * radial_error
        elif mode == 'side_tracking':
            desired_heading = bearing_to_target - self.orbit_direction * self.side_angle
            heading_error = self.normalize_angle(desired_heading - robot_yaw)
            radial_error = distance - self.side_distance
            linear = self.side_speed + self.distance_kp * radial_error
            angular = self.heading_kp * heading_error
        else:
            heading_error = self.normalize_angle(bearing_to_target - robot_yaw)
            radial_error = distance - self.follow_distance
            linear = self.follow_speed + self.distance_kp * radial_error
            angular = self.heading_kp * heading_error

        if 0.0 <= linear < 0.10:
            linear = 0.10
        linear = self.clamp(linear, -self.max_reverse_speed, self.max_linear_speed)
        angular = self.clamp_angular(angular)
        linear, angular = self.nav2_style_local_planner(linear, angular, robot_x, robot_y, robot_yaw)
        return linear, angular

    def nav2_style_orbit_radius(self, target_x, target_y, angle):
        radius = self.orbit_radius
        if not self.enable_obstacle_avoidance or not self.obstacles:
            return radius

        nominal_x = target_x + self.orbit_radius * math.cos(angle)
        nominal_y = target_y + self.orbit_radius * math.sin(angle)
        extra_radius = 0.0
        for obstacle_x, obstacle_y, obstacle_radius in self.obstacles:
            clearance = math.hypot(nominal_x - obstacle_x, nominal_y - obstacle_y) - obstacle_radius
            if clearance >= self.obstacle_inflation_radius:
                continue
            self.avoidance_active = True
            strength = 1.0 - self.clamp(
                clearance / max(self.obstacle_inflation_radius, 0.01),
                0.0,
                1.0,
            )
            extra_radius = max(extra_radius, self.avoidance_orbit_radius_offset * strength)
        return radius + extra_radius

    def nav2_style_local_planner(self, linear, angular, robot_x, robot_y, robot_yaw):
        if not self.enable_obstacle_avoidance or not self.obstacles or linear <= 0.0:
            self.avoidance_active = False
            return linear, angular

        candidates = [-0.70, -0.35, 0.0, 0.35, 0.70]
        best_linear = linear
        best_angular = angular
        best_score = None
        horizon = 1.2
        step = 0.20

        for offset in candidates:
            candidate_angular = self.clamp_angular(angular + offset * self.avoidance_turn_gain)
            min_clearance = float('inf')
            sim_x = robot_x
            sim_y = robot_y
            sim_yaw = robot_yaw
            t = 0.0
            while t < horizon:
                sim_yaw = self.normalize_angle(sim_yaw + candidate_angular * step)
                sim_x += linear * math.cos(sim_yaw) * step
                sim_y += linear * math.sin(sim_yaw) * step
                for obstacle_x, obstacle_y, obstacle_radius in self.obstacles:
                    clearance = math.hypot(sim_x - obstacle_x, sim_y - obstacle_y) - obstacle_radius
                    min_clearance = min(min_clearance, clearance)
                t += step

            collision_cost = 100.0 if min_clearance < self.obstacle_inflation_radius else 0.0
            clearance_cost = 1.0 / max(min_clearance - self.obstacle_inflation_radius + 0.05, 0.05)
            path_cost = abs(candidate_angular - angular) * 0.35
            score = collision_cost + clearance_cost + path_cost
            if best_score is None or score < best_score:
                best_score = score
                best_angular = candidate_angular
                if min_clearance < self.obstacle_inflation_radius:
                    best_linear = max(0.05, linear * 0.55)
                else:
                    best_linear = linear

        self.avoidance_active = best_score is not None and abs(best_angular - angular) > 0.03
        return best_linear, best_angular
    def compute_search_command(self):
        return self.search_creep_speed, self.clamp_angular(self.search_turn_speed)

    def subject_distance(self):
        if not self.robot_pose or not self.target_pose:
            return -1.0
        return math.hypot(self.target_pose[0] - self.robot_pose[0], self.target_pose[1] - self.robot_pose[1])

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

    def log_state(self, target_available, recently_lost):
        mode = self.active_mode()
        if target_available:
            state = '%s cinematic tracking distance=%.2f' % (mode, self.subject_distance())
        elif recently_lost:
            state = 'recovering last target'
        else:
            state = 'waiting for target'
        if state != self.last_state:
            self.get_logger().info('Cinematic state: %s' % state)
            self.last_state = state

    def mode_index(self):
        mode = self.active_mode()
        if mode == 'follow_behind':
            return 1.0
        if mode == 'side_tracking':
            return 2.0
        if self.mode.lower() == 'showcase':
            return 3.0
        return 0.0

    def active_mode(self):
        mode = self.mode.lower()
        if mode != 'showcase':
            return mode
        elapsed = (self.get_clock().now() - self.start_time).nanoseconds / 1_000_000_000
        phase = int(elapsed / max(self.showcase_period, 1.0)) % 3
        if phase == 1:
            return 'side_tracking'
        if phase == 2:
            return 'follow_behind'
        return 'orbit'

    def clamp_angular(self, value):
        return self.clamp(value, -self.max_angular_speed, self.max_angular_speed)

    @staticmethod
    def slew(current, target, rate_limit, dt):
        max_step = max(rate_limit, 0.0) * max(dt, 0.0)
        delta = target - current
        if delta > max_step:
            return current + max_step
        if delta < -max_step:
            return current - max_step
        return target

    @staticmethod
    def clamp(value, lower, upper):
        return max(lower, min(upper, value))

    @staticmethod
    def normalize_angle(value):
        while value > math.pi:
            value -= 2.0 * math.pi
        while value < -math.pi:
            value += 2.0 * math.pi
        return value

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
    def find_model_index(names, requested_name):
        if requested_name in names:
            return names.index(requested_name)
        for index, name in enumerate(names):
            if name.endswith('::' + requested_name) or requested_name in name:
                return index
        return None

    @staticmethod
    def yaw_to_quaternion(yaw):
        half_yaw = 0.5 * yaw
        return 0.0, 0.0, math.sin(half_yaw), math.cos(half_yaw)

    @staticmethod
    def quaternion_to_yaw(x, y, z, w):
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        return math.atan2(siny_cosp, cosy_cosp)


def main(args=None):
    rclpy.init(args=args)
    node = CinematicTracking()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()