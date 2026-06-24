import math

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, Float64MultiArray


class HumanTracking(Node):
    """Detect a person in camera images and publish normalized frame offset."""

    def __init__(self):
        super().__init__('human_tracking')

        self.declare_parameter('image_topic', '/gimbal_camera/image_raw')
        self.declare_parameter('depth_topic', '')
        self.declare_parameter('debug_image_topic', '/gimbal/human_tracking/debug_image')
        self.declare_parameter('target_offset_topic', 'gimbal/target_offset')
        self.declare_parameter('target_visible_topic', 'gimbal/target_visible')
        self.declare_parameter('detector', 'hog')
        self.declare_parameter('yolo_model', '')
        self.declare_parameter('min_confidence', 0.35)
        self.declare_parameter('publish_debug_image', True)
        self.declare_parameter('target_hue_min', 45)
        self.declare_parameter('target_hue_max', 85)
        self.declare_parameter('target_saturation_min', 80)
        self.declare_parameter('target_value_min', 80)

        self.image_topic = self.get_parameter('image_topic').value
        self.depth_topic = self.get_parameter('depth_topic').value
        self.target_offset_topic = self.get_parameter('target_offset_topic').value
        self.target_visible_topic = self.get_parameter('target_visible_topic').value
        self.debug_image_topic = self.get_parameter('debug_image_topic').value
        self.detector_name = self.get_parameter('detector').value.lower()
        self.yolo_model_path = self.get_parameter('yolo_model').value
        self.min_confidence = float(self.get_parameter('min_confidence').value)
        self.publish_debug_image = bool(self.get_parameter('publish_debug_image').value)
        self.target_hue_min = int(self.get_parameter('target_hue_min').value)
        self.target_hue_max = int(self.get_parameter('target_hue_max').value)
        self.target_saturation_min = int(
            self.get_parameter('target_saturation_min').value
        )
        self.target_value_min = int(self.get_parameter('target_value_min').value)

        self.yolo_model = None
        if self.detector_name == 'yolo':
            self.yolo_model = self.load_yolo_model(self.yolo_model_path)
            if self.yolo_model is None:
                self.detector_name = 'hog'

        self.hog = cv2.HOGDescriptor()
        self.hog.setSVMDetector(cv2.HOGDescriptor_getDefaultPeopleDetector())
        self.latest_depth_image = None
        self.latest_depth_stamp = None

        self.image_subscription = self.create_subscription(
            Image, self.image_topic, self.process_image, 10
        )
        self.depth_subscription = None
        if self.depth_topic:
            self.depth_subscription = self.create_subscription(
                Image, self.depth_topic, self.process_depth_image, 10
            )
        self.offset_publisher = self.create_publisher(
            Float64MultiArray, self.target_offset_topic, 10
        )
        self.visible_publisher = self.create_publisher(
            Bool, self.target_visible_topic, 10
        )
        self.debug_publisher = None
        if self.publish_debug_image:
            self.debug_publisher = self.create_publisher(Image, self.debug_image_topic, 10)

        self.get_logger().info(
            'Starting Week 4 human tracking node: detector=%s image_topic=%s'
            % (self.detector_name, self.image_topic)
        )


    def process_depth_image(self, depth_msg):
        try:
            self.latest_depth_image = self.depth_to_array(depth_msg)
            self.latest_depth_stamp = self.get_clock().now()
        except ValueError as exc:
            self.get_logger().warning(str(exc))
    def load_yolo_model(self, model_path):
        try:
            from ultralytics import YOLO
        except ImportError:
            self.get_logger().warning(
                'ultralytics is not installed. Falling back to OpenCV HOG detector.'
            )
            return None

        if not model_path:
            self.get_logger().warning(
                'detector:=yolo was selected but yolo_model is empty. Falling back to HOG.'
            )
            return None

        try:
            return YOLO(model_path)
        except Exception as exc:
            self.get_logger().warning(
                'Could not load YOLO model "%s": %s. Falling back to HOG.'
                % (model_path, exc)
            )
            return None

    def process_image(self, image_msg):
        try:
            frame = self.image_to_bgr(image_msg)
        except ValueError as exc:
            self.get_logger().warning(str(exc))
            return

        detection = self.detect_person(frame)
        visible_msg = Bool()
        visible_msg.data = detection is not None
        self.visible_publisher.publish(visible_msg)

        if detection is not None:
            x, y, w, h, confidence = detection
            height, width = frame.shape[:2]
            center_x = x + w / 2.0
            center_y = y + h / 2.0
            x_error = (center_x - width / 2.0) / (width / 2.0)
            y_error = (center_y - height / 2.0) / (height / 2.0)

            offset_msg = Float64MultiArray()
            target_distance = self.estimate_target_distance(x, y, w, h, width, height)
            offset_msg.data = [
                self.clamp(x_error, -1.0, 1.0),
                self.clamp(y_error, -1.0, 1.0),
                float(confidence),
                float(target_distance) if target_distance is not None else -1.0,
            ]
            self.offset_publisher.publish(offset_msg)

            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
            cv2.circle(frame, (int(center_x), int(center_y)), 4, (0, 255, 0), -1)
            cv2.putText(
                frame,
                'person %.2f offset=(%.2f, %.2f) depth=%.2fm' % (
                    confidence,
                    x_error,
                    y_error,
                    target_distance if target_distance is not None else -1.0,
                ),
                (max(0, x), max(20, y - 8)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (0, 255, 0),
                1,
                cv2.LINE_AA,
            )
        else:
            cv2.putText(
                frame,
                'no person detected',
                (16, 28),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (0, 0, 255),
                2,
                cv2.LINE_AA,
            )

        if self.debug_publisher is not None:
            debug_msg = self.bgr_to_image(frame, image_msg.header)
            self.debug_publisher.publish(debug_msg)

    def detect_person(self, frame):
        if self.detector_name == 'color':
            return self.detect_person_color_target(frame)
        if self.detector_name == 'yolo' and self.yolo_model is not None:
            return self.detect_person_yolo(frame)
        return self.detect_person_hog(frame)

    def detect_person_color_target(self, frame):
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        lower = (
            self.target_hue_min,
            self.target_saturation_min,
            self.target_value_min,
        )
        upper = (self.target_hue_max, 255, 255)
        mask = cv2.inRange(hsv, lower, upper)
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return None

        contour = max(contours, key=cv2.contourArea)
        area = cv2.contourArea(contour)
        height, width = frame.shape[:2]
        min_area = max(80.0, width * height * 0.001)
        if area < min_area:
            return None

        x, y, w, h = cv2.boundingRect(contour)
        confidence = self.clamp(area / float(width * height * 0.08), 0.0, 1.0)
        return x, y, w, h, confidence

    def detect_person_yolo(self, frame):
        results = self.yolo_model(frame, verbose=False)
        best_detection = None
        best_confidence = -math.inf

        for result in results:
            for box in result.boxes:
                cls = int(box.cls[0])
                confidence = float(box.conf[0])
                if cls != 0 or confidence < self.min_confidence:
                    continue

                x1, y1, x2, y2 = [int(v) for v in box.xyxy[0]]
                if confidence > best_confidence:
                    best_confidence = confidence
                    best_detection = (x1, y1, x2 - x1, y2 - y1, confidence)

        return best_detection

    def detect_person_hog(self, frame):
        resized = frame
        scale = 1.0
        height, width = frame.shape[:2]
        max_width = 640
        if width > max_width:
            scale = max_width / float(width)
            resized = cv2.resize(frame, (max_width, int(height * scale)))

        boxes, weights = self.hog.detectMultiScale(
            resized,
            winStride=(8, 8),
            padding=(8, 8),
            scale=1.05,
        )

        best_detection = None
        best_score = -math.inf
        for (x, y, w, h), weight in zip(boxes, weights):
            confidence = float(weight)
            if confidence < self.min_confidence:
                continue
            area = w * h
            score = confidence * area
            if score > best_score:
                best_score = score
                inv_scale = 1.0 / scale
                best_detection = (
                    int(x * inv_scale),
                    int(y * inv_scale),
                    int(w * inv_scale),
                    int(h * inv_scale),
                    confidence,
                )

        return best_detection


    def estimate_target_distance(self, x, y, w, h, image_width, image_height):
        if self.latest_depth_image is None:
            return None

        depth = self.latest_depth_image
        depth_height, depth_width = depth.shape[:2]
        x_scale = depth_width / float(image_width)
        y_scale = depth_height / float(image_height)
        x1 = int((x + 0.25 * w) * x_scale)
        x2 = int((x + 0.75 * w) * x_scale)
        y1 = int((y + 0.25 * h) * y_scale)
        y2 = int((y + 0.75 * h) * y_scale)
        x1 = max(0, min(depth_width - 1, x1))
        x2 = max(x1 + 1, min(depth_width, x2))
        y1 = max(0, min(depth_height - 1, y1))
        y2 = max(y1 + 1, min(depth_height, y2))

        region = depth[y1:y2, x1:x2]
        valid = region[np.isfinite(region)]
        valid = valid[(valid > 0.20) & (valid < 20.0)]
        if valid.size == 0:
            return None
        return float(np.median(valid))

    @staticmethod
    def depth_to_array(depth_msg):
        data = memoryview(depth_msg.data)
        if depth_msg.encoding == '32FC1':
            array = np.frombuffer(data, dtype=np.float32).reshape(
                depth_msg.height,
                depth_msg.step // 4,
            )[:, : depth_msg.width]
            return array.copy()
        if depth_msg.encoding == '16UC1':
            array = np.frombuffer(data, dtype=np.uint16).reshape(
                depth_msg.height,
                depth_msg.step // 2,
            )[:, : depth_msg.width]
            return array.astype(np.float32) / 1000.0
        raise ValueError('Unsupported depth image encoding: %s' % depth_msg.encoding)
    @staticmethod
    def image_to_bgr(image_msg):
        channels = 3
        if image_msg.encoding in ('mono8', '8UC1'):
            channels = 1
        elif image_msg.encoding not in ('rgb8', 'bgr8', 'rgba8', 'bgra8'):
            raise ValueError('Unsupported image encoding: %s' % image_msg.encoding)

        image = memoryview(image_msg.data)

        if channels == 1:
            array = np.frombuffer(image, dtype=np.uint8).reshape(
                image_msg.height, image_msg.step
            )[:, : image_msg.width]
            return cv2.cvtColor(array, cv2.COLOR_GRAY2BGR)

        channel_count = 4 if image_msg.encoding in ('rgba8', 'bgra8') else 3
        array = np.frombuffer(image, dtype=np.uint8).reshape(
            image_msg.height, image_msg.step
        )[:, : image_msg.width * channel_count]
        array = array.reshape(image_msg.height, image_msg.width, channel_count)

        if image_msg.encoding == 'rgb8':
            return cv2.cvtColor(array, cv2.COLOR_RGB2BGR)
        if image_msg.encoding == 'rgba8':
            return cv2.cvtColor(array, cv2.COLOR_RGBA2BGR)
        if image_msg.encoding == 'bgra8':
            return cv2.cvtColor(array, cv2.COLOR_BGRA2BGR)
        return array.copy()

    @staticmethod
    def bgr_to_image(frame, header):
        msg = Image()
        msg.header = header
        msg.height = frame.shape[0]
        msg.width = frame.shape[1]
        msg.encoding = 'bgr8'
        msg.is_bigendian = 0
        msg.step = frame.shape[1] * 3
        msg.data = frame.tobytes()
        return msg

    @staticmethod
    def clamp(value, lower, upper):
        return max(lower, min(upper, value))


def main(args=None):
    rclpy.init(args=args)
    node = HumanTracking()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
