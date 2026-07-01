import math

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node


class MovingTargetDemo(Node):
    """Drive the green Gazebo target along a repeatable walking path."""

    def __init__(self):
        super().__init__('moving_target_demo')

        self.declare_parameter('cmd_vel_topic', '/tracking_subject/cmd_vel')
        self.declare_parameter('update_rate', 10.0)
        self.declare_parameter('forward_speed', 0.20)
        self.declare_parameter('turn_speed', 0.24)

        self.cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        self.update_rate = float(self.get_parameter('update_rate').value)
        self.forward_speed = float(self.get_parameter('forward_speed').value)
        self.turn_speed = float(self.get_parameter('turn_speed').value)

        self.publisher = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        self.start_time = self.get_clock().now()
        self.last_phase = None
        self.timer = self.create_timer(
            1.0 / max(self.update_rate, 1.0),
            self.publish_target_motion,
        )

        self.get_logger().info(
            'Starting moving target demo: cmd_vel_topic=%s' % self.cmd_vel_topic
        )

    def publish_target_motion(self):
        elapsed = (
            self.get_clock().now() - self.start_time
        ).nanoseconds / 1_000_000_000
        msg = Twist()
        phase = 'pause'

        if elapsed < 1.0:
            phase = 'initial pause'
        elif elapsed < 8.0:
            phase = 'long forward walk'
            msg.linear.x = self.forward_speed
            msg.angular.z = 0.015 * math.sin(0.8 * elapsed)
        elif elapsed < 9.5:
            phase = 'brief walking pause'
        elif elapsed < 15.5:
            phase = 'wide left curve'
            msg.linear.x = self.forward_speed * 0.85
            msg.angular.z = self.turn_speed * 0.70
        elif elapsed < 17.0:
            phase = 'pause after wide turn'
        elif elapsed < 24.0:
            phase = 'extended final walk'
            msg.linear.x = self.forward_speed * 0.90
            msg.angular.z = 0.015
        else:
            phase = 'demo target stopped'

        if phase != self.last_phase:
            self.get_logger().info('Moving target phase: %s' % phase)
            self.last_phase = phase

        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = MovingTargetDemo()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
