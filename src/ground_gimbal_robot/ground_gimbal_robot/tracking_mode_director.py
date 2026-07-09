import math

from gazebo_msgs.msg import EntityState
from gazebo_msgs.msg import ModelState
from gazebo_msgs.msg import ModelStates
from gazebo_msgs.srv import SetEntityState
from gazebo_msgs.srv import SetModelState
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class TrackingModeDirector(Node):
    """Runtime demo director for mode commands and scripted actor motion."""

    def __init__(self):
        super().__init__('tracking_mode_director')

        self.declare_parameter('mode_command_topic', '/tracking_mode_cmd')
        self.declare_parameter('model_states_topic', '/gazebo/model_states')
        self.declare_parameter('set_entity_state_service', '/gazebo/set_entity_state')
        self.declare_parameter('set_model_state_service', '/gazebo/set_model_state')
        self.declare_parameter('target_model_name', 'tracking_subject')
        self.declare_parameter('side_actor_speed', 0.16)
        self.declare_parameter('side_start_x', 12.8)
        self.declare_parameter('side_start_y', 0.0)
        self.declare_parameter('side_start_z', 0.0)
        self.declare_parameter('side_start_yaw', 3.14159)
        self.declare_parameter('control_rate', 20.0)

        self.mode_command_topic = self.get_parameter('mode_command_topic').value
        self.model_states_topic = self.get_parameter('model_states_topic').value
        self.set_entity_state_service = self.get_parameter('set_entity_state_service').value
        self.set_model_state_service = self.get_parameter('set_model_state_service').value
        self.target_model_name = self.get_parameter('target_model_name').value
        self.side_actor_speed = float(self.get_parameter('side_actor_speed').value)
        self.side_start_pose = (
            float(self.get_parameter('side_start_x').value),
            float(self.get_parameter('side_start_y').value),
            float(self.get_parameter('side_start_z').value),
        )
        self.side_start_yaw = float(self.get_parameter('side_start_yaw').value)
        self.control_rate = float(self.get_parameter('control_rate').value)

        self.current_mode = 'follow'
        self.target_pose = None
        self.target_yaw = math.pi
        self.last_timer_time = self.get_clock().now()
        self.pending_entity_future = None
        self.pending_model_future = None
        self.last_service_warning_time = self.get_clock().now()
        self.last_pose_warning_time = self.get_clock().now()
        self.motion_started = False

        self.create_subscription(String, self.mode_command_topic, self.update_mode, 10)
        self.create_subscription(ModelStates, self.model_states_topic, self.update_model_states, 10)
        self.set_entity_state_client = self.create_client(
            SetEntityState,
            self.set_entity_state_service,
        )
        self.set_model_state_client = self.create_client(
            SetModelState,
            self.set_model_state_service,
        )
        self.timer = self.create_timer(
            1.0 / max(self.control_rate, 1.0),
            self.update_actor_motion,
        )

        self.get_logger().info('Tracking mode director ready on %s' % self.mode_command_topic)

    def update_mode(self, msg):
        requested = str(msg.data).strip().lower()
        aliases = {
            'orbitcw': 'orbit_cw',
            'cw': 'orbit_cw',
            'orbitccw': 'orbit_ccw',
            'ccw': 'orbit_ccw',
            'side': 'side_tracking',
            'sidetracking': 'side_tracking',
            'sideleft': 'side_left',
            'sideright': 'side_right',
        }
        self.current_mode = aliases.get(requested, requested)
        if self.current_mode.startswith('side') and self.target_pose is None:
            self.target_pose = self.side_start_pose
            self.target_yaw = self.side_start_yaw
            self.get_logger().warning(
                'No actor pose received from %s; using side start pose (%.2f, %.2f, %.2f).'
                % (
                    self.model_states_topic,
                    self.target_pose[0],
                    self.target_pose[1],
                    self.target_pose[2],
                )
            )
        if not self.current_mode.startswith('side'):
            self.motion_started = False
        self.get_logger().info('Director mode command: %s' % self.current_mode)

    def update_model_states(self, msg):
        try:
            target_index = msg.name.index(self.target_model_name)
        except ValueError:
            return
        pose = msg.pose[target_index]
        self.target_pose = (pose.position.x, pose.position.y, pose.position.z)
        self.target_yaw = self.quaternion_to_yaw(pose.orientation)

    def update_actor_motion(self):
        now = self.get_clock().now()
        dt = (now - self.last_timer_time).nanoseconds / 1_000_000_000
        self.last_timer_time = now

        if not self.current_mode.startswith('side'):
            return
        if self.target_pose is None:
            self.warn_pose_missing(now)
            return
        if dt <= 0.0:
            return

        entity_ready = self.set_entity_state_client.service_is_ready()
        model_ready = self.set_model_state_client.service_is_ready()
        if not entity_ready and not model_ready:
            self.warn_service_waiting(now)
            return

        x, y, z = self.target_pose
        x += self.side_actor_speed * dt
        self.target_pose = (x, y, z)
        qx, qy, qz, qw = self.yaw_to_quaternion(self.side_start_yaw)
        if not self.motion_started:
            self.motion_started = True
            self.get_logger().info(
                'Side actor motion started from (%.2f, %.2f, %.2f).'
                % (x, y, z)
            )

        if entity_ready and self.future_done(self.pending_entity_future):
            entity_state = EntityState()
            entity_state.name = self.target_model_name
            entity_state.pose.position.x = x
            entity_state.pose.position.y = y
            entity_state.pose.position.z = z
            entity_state.pose.orientation.x = qx
            entity_state.pose.orientation.y = qy
            entity_state.pose.orientation.z = qz
            entity_state.pose.orientation.w = qw
            entity_state.reference_frame = 'world'

            request = SetEntityState.Request()
            request.state = entity_state
            self.pending_entity_future = self.set_entity_state_client.call_async(request)

        if model_ready and self.future_done(self.pending_model_future):
            model_state = ModelState()
            model_state.model_name = self.target_model_name
            model_state.pose.position.x = x
            model_state.pose.position.y = y
            model_state.pose.position.z = z
            model_state.pose.orientation.x = qx
            model_state.pose.orientation.y = qy
            model_state.pose.orientation.z = qz
            model_state.pose.orientation.w = qw
            model_state.reference_frame = 'world'

            request = SetModelState.Request()
            request.model_state = model_state
            self.pending_model_future = self.set_model_state_client.call_async(request)

    def warn_pose_missing(self, now):
        elapsed = (now - self.last_pose_warning_time).nanoseconds / 1_000_000_000
        if elapsed < 2.0:
            return
        self.last_pose_warning_time = now
        self.get_logger().warning(
            'Side tracking has no actor pose. Check whether %s contains %s.'
            % (self.model_states_topic, self.target_model_name)
        )
    def warn_service_waiting(self, now):
        elapsed = (now - self.last_service_warning_time).nanoseconds / 1_000_000_000
        if elapsed < 2.0:
            return
        self.last_service_warning_time = now
        self.get_logger().warning(
            'Side tracking is waiting for Gazebo state service: %s or %s'
            % (self.set_entity_state_service, self.set_model_state_service)
        )

    @staticmethod
    def future_done(future):
        return future is None or future.done()

    @staticmethod
    def quaternion_to_yaw(quaternion):
        siny_cosp = 2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y)
        cosy_cosp = 1.0 - 2.0 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z)
        return math.atan2(siny_cosp, cosy_cosp)

    @staticmethod
    def yaw_to_quaternion(yaw):
        half = yaw * 0.5
        return 0.0, 0.0, math.sin(half), math.cos(half)


def main(args=None):
    rclpy.init(args=args)
    node = TrackingModeDirector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()