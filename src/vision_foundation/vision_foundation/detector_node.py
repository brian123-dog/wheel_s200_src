#!/usr/bin/env python3
"""ROS2 node for YOLOE open-vocabulary object detection.

Service (using example_interfaces/srv/SetBool):
    ros2 service call /vision_foundation/detect example_interfaces/srv/SetBool \
      "{data: true, message: '{\"categories\":[\"meter\",\"switch\"],\"confidence\":0.25}'}"

The node subscribes to a camera topic and caches the latest frame. When the
service is called, it runs YOLOE inference on the cached frame with the given
text prompts and returns bounding box detections as JSON.
"""

import json
import os
import threading
from typing import List, Optional

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from example_interfaces.srv import SetBool

from .yoloe_detector import YOLOEDetector


class VisionFoundationNode(Node):
    """ROS2 Node wrapping YOLOE for on-demand open-vocabulary detection."""

    def __init__(self):
        super().__init__('vision_foundation')

        # ── parameters ──────────────────────────────────────────────────────
        self.declare_parameter('model_path', '')
        self.declare_parameter('device', 'cpu')
        self.declare_parameter('image_topic', '/camera_dcw2/color/image_raw')
        self.declare_parameter('default_confidence', 0.25)
        self.declare_parameter('default_categories', ['meter', 'switch', 'valve'])
        self.declare_parameter('publish_debug_image', True)

        model_path = self.get_parameter('model_path').get_parameter_value().string_value
        if not model_path:
            from ament_index_python.packages import get_package_share_directory
            model_path = os.path.join(
                get_package_share_directory('vision_foundation'),
                'models', 'yoloe-v8s.pt')
        else:
            model_path = os.path.expanduser(model_path)

        self._device = self.get_parameter('device').get_parameter_value().string_value
        self._image_topic = self.get_parameter('image_topic').get_parameter_value().string_value
        self._default_confidence = self.get_parameter('default_confidence').value
        self._default_categories = self.get_parameter('default_categories').value
        self._publish_debug = self.get_parameter('publish_debug_image').value

        # ── model ───────────────────────────────────────────────────────────
        self._detector: Optional[YOLOEDetector] = None
        try:
            self._detector = YOLOEDetector(model_path, device=self._device)
            self.get_logger().info(f'YOLOE model loaded: {model_path}')
        except Exception as e:
            self.get_logger().fatal(f'Failed to load model: {e}')

        # ── state ───────────────────────────────────────────────────────────
        self._bridge = CvBridge()
        self._latest_image: Optional[np.ndarray] = None
        self._latest_header = None
        self._image_lock = threading.Lock()

        # ── image subscriber (best-effort, depth=1 to avoid backlog) ────────
        qos = rclpy.qos.QoSProfile(
            depth=1,
            reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
        )
        self._sub = self.create_subscription(
            Image, self._image_topic, self._image_callback, qos)

        # ── service ─────────────────────────────────────────────────────────
        self._srv = self.create_service(
            SetBool, '/vision_foundation/detect', self._detect_callback)

        # ── debug image publisher ───────────────────────────────────────────
        self._debug_pub = None
        if self._publish_debug:
            self._debug_pub = self.create_publisher(
                Image, '/vision_foundation/debug_image', 10)

        self.get_logger().info(
            f'VisionFoundationNode ready\n'
            f'  image_topic: {self._image_topic}\n'
            f'  service:     /vision_foundation/detect\n'
            f'  device:      {self._device}\n'
            f'  default categories: {self._default_categories}')

    # ── image subscriber ─────────────────────────────────────────────────────

    def _image_callback(self, msg: Image):
        """Cache the latest camera frame."""
        try:
            img = self._bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            with self._image_lock:
                self._latest_image = img
                self._latest_header = msg.header
        except Exception as e:
            self.get_logger().error(f'Image conversion failed: {e}')

    # ── service callback ─────────────────────────────────────────────────────

    def _detect_callback(self, request: SetBool.Request, response: SetBool.Response):
        """Handle /vision_foundation/detect service calls.

        request.data:  True to trigger detection.
        request.message: JSON string with optional 'categories' and 'confidence'.

        response.success: True if detection completed.
        response.message: JSON string with detection results or error info.
        """
        # ── parse request JSON ──────────────────────────────────────────
        categories = list(self._default_categories)
        confidence = self._default_confidence

        if request.message:
            try:
                req_json = json.loads(request.message)
                if 'categories' in req_json:
                    categories = req_json['categories']
                if 'confidence' in req_json:
                    confidence = float(req_json['confidence'])
            except (json.JSONDecodeError, ValueError, TypeError) as e:
                response.success = False
                response.message = json.dumps({
                    'error': f'Invalid request JSON: {e}',
                    'hint': 'Use format: {"categories": ["meter"], "confidence": 0.25}',
                })
                return response

        # ── get latest image ────────────────────────────────────────────
        with self._image_lock:
            if self._latest_image is None:
                response.success = False
                response.message = json.dumps({
                    'error': 'No image received from camera yet',
                })
                self.get_logger().warn(response.message)
                return response
            img = self._latest_image.copy()

        # ── run detection ───────────────────────────────────────────────
        try:
            detections = self._detector.detect(img, categories, confidence)

            result = {
                'success': True,
                'num_detections': len(detections),
                'categories_requested': categories,
                'confidence_threshold': confidence,
                'detections': [
                    {
                        'class': d.class_name,
                        'confidence': round(d.confidence, 4),
                        'bbox': list(d.bbox),  # [x1, y1, x2, y2]
                    }
                    for d in detections
                ],
            }

            response.success = True
            response.message = json.dumps(result, ensure_ascii=False)
            self.get_logger().info(
                f'Detection done: {categories} → {len(detections)} objects')

            # Publish debug visualization
            if self._publish_debug and self._debug_pub is not None:
                self._publish_debug_image(img, detections)

        except Exception as e:
            response.success = False
            response.message = json.dumps({'error': str(e)})
            self.get_logger().error(f'Detection failed: {e}')

        return response

    # ── debug visualization ──────────────────────────────────────────────────

    def _publish_debug_image(self, img: np.ndarray, detections):
        """Draw bounding boxes on image and publish."""
        out = img.copy()
        colors = [
            (0, 255, 0), (0, 0, 255), (255, 0, 0),
            (255, 255, 0), (255, 0, 255), (0, 255, 255),
        ]

        for i, d in enumerate(detections):
            color = colors[i % len(colors)]
            x1, y1, x2, y2 = d.bbox
            cv2.rectangle(out, (x1, y1), (x2, y2), color, 2)
            label = f'{d.class_name} {d.confidence:.2f}'
            cv2.putText(out, label, (x1, max(y1 - 8, 20)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)

        debug_msg = self._bridge.cv2_to_imgmsg(out, encoding='bgr8')
        debug_msg.header = self._latest_header
        self._debug_pub.publish(debug_msg)


def main(args=None):
    rclpy.init(args=args)
    node = VisionFoundationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
