import math

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node


class MotionDemo(Node):
    """Publish a repeatable Week 2 motion test sequence to /cmd_vel."""

    def __init__(self):
        super().__init__('motion_demo')
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.start_time = self.get_clock().now()
        self.timer = self.create_timer(0.1, self.publish_command)
        self.last_phase = None
        self.get_logger().info(
            'Starting motion demo: forward, left pivot, wide arc, right pivot, stop.'
        )

    def publish_command(self):
        elapsed = (
            self.get_clock().now() - self.start_time
        ).nanoseconds / 1_000_000_000

        msg = Twist()
        phase = 'stop'
        if elapsed < 2.0:
            phase = 'initial pause'
        elif elapsed < 7.0:
            phase = 'drive forward'
            msg.linear.x = 0.45
        elif elapsed < 8.0:
            phase = 'pause after forward'
        elif elapsed < 13.0:
            phase = 'left pivot turn'
            msg.angular.z = 1.2
        elif elapsed < 14.0:
            phase = 'pause after left turn'
        elif elapsed < 21.0:
            phase = 'forward left arc'
            msg.linear.x = 0.45
            msg.angular.z = 0.65
        elif elapsed < 22.0:
            phase = 'pause after arc'
        elif elapsed < 27.0:
            phase = 'right pivot turn'
            msg.angular.z = -1.2
        else:
            self.timer.cancel()
            self.publisher.publish(msg)
            self.get_logger().info('Motion demo complete. Robot stopped.')
            return

        if phase != self.last_phase:
            self.get_logger().info('Motion phase: %s' % phase)
            self.last_phase = phase

        radius = math.inf if abs(msg.angular.z) < 1e-6 else msg.linear.x / msg.angular.z
        self.get_logger().debug('Command radius: %.3f m' % radius)
        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = MotionDemo()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
