import math

import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from std_msgs.msg import Float64MultiArray


class ConstantVelocityKalmanFilter:
    """Small constant-velocity Kalman filter for image-space target tracking."""

    def __init__(self, process_noise, measurement_noise):
        self.process_noise = float(process_noise)
        self.measurement_noise = float(measurement_noise)
        self.state = np.zeros((6, 1), dtype=float)
        self.covariance = np.eye(6, dtype=float)
        self.initialized = False

    def predict(self, dt):
        dt = max(float(dt), 0.0)
        transition = np.eye(6, dtype=float)
        transition[0, 2] = dt
        transition[1, 3] = dt
        transition[4, 5] = dt

        q = self.process_noise
        process = np.diag([
            q * max(dt, 0.01),
            q * max(dt, 0.01),
            q,
            q,
            q * max(dt, 0.01),
            q,
        ])
        self.state = transition @ self.state
        self.covariance = transition @ self.covariance @ transition.T + process

    def update(self, x_error, y_error, distance):
        measurement_values = [x_error, y_error]
        measurement_rows = [
            [1.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            [0.0, 1.0, 0.0, 0.0, 0.0, 0.0],
        ]
        if distance is not None:
            measurement_values.append(distance)
            measurement_rows.append([0.0, 0.0, 0.0, 0.0, 1.0, 0.0])

        measurement = np.array(measurement_values, dtype=float).reshape((-1, 1))
        observation = np.array(measurement_rows, dtype=float)
        noise = np.eye(len(measurement_values), dtype=float) * self.measurement_noise

        innovation = measurement - observation @ self.state
        innovation_covariance = observation @ self.covariance @ observation.T + noise
        gain = self.covariance @ observation.T @ np.linalg.inv(innovation_covariance)
        self.state = self.state + gain @ innovation
        identity = np.eye(6, dtype=float)
        self.covariance = (identity - gain @ observation) @ self.covariance

    def reset(self, x_error, y_error, distance):
        self.state = np.zeros((6, 1), dtype=float)
        self.state[0, 0] = x_error
        self.state[1, 0] = y_error
        if distance is not None:
            self.state[4, 0] = distance
        self.covariance = np.eye(6, dtype=float) * 0.08
        self.initialized = True


class TrackingFilter(Node):
    """Week 7 sensor-fusion layer for smoother target offset and short loss prediction."""

    def __init__(self):
        super().__init__('tracking_filter')

        self.declare_parameter('input_offset_topic', 'gimbal/target_offset_raw')
        self.declare_parameter('input_visible_topic', 'gimbal/target_visible_raw')
        self.declare_parameter('output_offset_topic', 'gimbal/target_offset')
        self.declare_parameter('output_visible_topic', 'gimbal/target_visible')
        self.declare_parameter('filter_rate', 30.0)
        self.declare_parameter('measurement_timeout', 0.25)
        self.declare_parameter('prediction_timeout', 1.2)
        self.declare_parameter('process_noise', 0.035)
        self.declare_parameter('measurement_noise', 0.055)
        self.declare_parameter('confidence_decay_rate', 0.55)

        self.input_offset_topic = self.get_parameter('input_offset_topic').value
        self.input_visible_topic = self.get_parameter('input_visible_topic').value
        self.output_offset_topic = self.get_parameter('output_offset_topic').value
        self.output_visible_topic = self.get_parameter('output_visible_topic').value
        self.filter_rate = float(self.get_parameter('filter_rate').value)
        self.measurement_timeout = float(self.get_parameter('measurement_timeout').value)
        self.prediction_timeout = float(self.get_parameter('prediction_timeout').value)
        self.confidence_decay_rate = float(self.get_parameter('confidence_decay_rate').value)

        self.filter = ConstantVelocityKalmanFilter(
            self.get_parameter('process_noise').value,
            self.get_parameter('measurement_noise').value,
        )
        self.last_filter_time = self.get_clock().now()
        self.last_measurement_time = None
        self.raw_visible = False
        self.filtered_confidence = 0.0

        self.create_subscription(
            Float64MultiArray,
            self.input_offset_topic,
            self.update_offset,
            10,
        )
        self.create_subscription(Bool, self.input_visible_topic, self.update_visible, 10)
        self.offset_publisher = self.create_publisher(Float64MultiArray, self.output_offset_topic, 10)
        self.visible_publisher = self.create_publisher(Bool, self.output_visible_topic, 10)
        self.timer = self.create_timer(1.0 / max(self.filter_rate, 1.0), self.publish_filtered_target)

        self.get_logger().info(
            'Starting Week 7 tracking filter: %s -> %s'
            % (self.input_offset_topic, self.output_offset_topic)
        )

    def update_offset(self, msg):
        if len(msg.data) < 2:
            return

        now = self.get_clock().now()
        dt = self.elapsed_seconds(self.last_filter_time, now)
        self.filter.predict(dt)
        self.last_filter_time = now

        x_error = self.clamp(float(msg.data[0]), -1.0, 1.0)
        y_error = self.clamp(float(msg.data[1]), -1.0, 1.0)
        confidence = self.clamp(float(msg.data[2]), 0.0, 1.0) if len(msg.data) >= 3 else 1.0
        distance = None
        if len(msg.data) >= 4 and math.isfinite(float(msg.data[3])) and float(msg.data[3]) > 0.0:
            distance = float(msg.data[3])

        if not self.filter.initialized:
            self.filter.reset(x_error, y_error, distance)
        else:
            self.filter.update(x_error, y_error, distance)

        self.filtered_confidence = confidence
        self.raw_visible = True
        self.last_measurement_time = now

    def update_visible(self, msg):
        self.raw_visible = bool(msg.data)

    def publish_filtered_target(self):
        now = self.get_clock().now()
        dt = self.elapsed_seconds(self.last_filter_time, now)
        if self.filter.initialized:
            self.filter.predict(dt)
        self.last_filter_time = now

        visible = self.filtered_target_is_visible(now)
        visible_msg = Bool()
        visible_msg.data = visible
        self.visible_publisher.publish(visible_msg)

        if not visible or not self.filter.initialized:
            return

        age = self.measurement_age(now)
        predicted = age > self.measurement_timeout
        confidence = self.filtered_confidence
        if predicted:
            confidence *= math.exp(-self.confidence_decay_rate * age)

        offset_msg = Float64MultiArray()
        offset_msg.data = [
            self.clamp(float(self.filter.state[0, 0]), -1.0, 1.0),
            self.clamp(float(self.filter.state[1, 0]), -1.0, 1.0),
            self.clamp(confidence, 0.0, 1.0),
            float(self.filter.state[4, 0]) if self.filter.state[4, 0] > 0.0 else -1.0,
            1.0 if predicted else 0.0,
        ]
        self.offset_publisher.publish(offset_msg)

    def filtered_target_is_visible(self, now):
        if not self.filter.initialized or self.last_measurement_time is None:
            return False
        age = self.measurement_age(now)
        if self.raw_visible and age <= self.measurement_timeout:
            return True
        return age <= self.prediction_timeout

    def measurement_age(self, now):
        if self.last_measurement_time is None:
            return float('inf')
        return self.elapsed_seconds(self.last_measurement_time, now)

    @staticmethod
    def elapsed_seconds(start, end):
        return (end - start).nanoseconds / 1_000_000_000

    @staticmethod
    def clamp(value, lower, upper):
        return max(lower, min(upper, value))


def main(args=None):
    rclpy.init(args=args)
    node = TrackingFilter()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
