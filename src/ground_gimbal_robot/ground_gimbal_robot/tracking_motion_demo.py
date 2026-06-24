import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node


class TrackingMotionDemo(Node):
    """Slow in-place yaw motion for the Week 4 camera tracking demo."""

    def __init__(self):
        super().__init__('tracking_motion_demo')
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.start_time = self.get_clock().now()
        self.timer = self.create_timer(0.1, self.publish_command)
        self.last_phase = None
        self.get_logger().info(
            'Starting safe tracking motion demo: in-place camera sweep only.'
        )

    def publish_command(self):
        elapsed = (
            self.get_clock().now() - self.start_time
        ).nanoseconds / 1_000_000_000

        msg = Twist()
        phase = 'stop'

        if elapsed < 2.0:
            phase = 'initial pause'
        elif elapsed < 8.0:
            phase = 'slow left camera sweep'
            msg.angular.z = 0.28
        elif elapsed < 10.0:
            phase = 'pause after left sweep'
        elif elapsed < 16.0:
            phase = 'slow right camera sweep'
            msg.angular.z = -0.28
        elif elapsed < 18.0:
            phase = 'pause after right sweep'
        elif elapsed < 24.0:
            phase = 'small left recenter sweep'
            msg.angular.z = 0.18
        else:
            self.timer.cancel()
            self.publisher.publish(msg)
            self.get_logger().info('Tracking motion demo complete. Robot stopped.')
            return

        if phase != self.last_phase:
            self.get_logger().info('Motion phase: %s' % phase)
            self.last_phase = phase

        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = TrackingMotionDemo()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
