#!/usr/bin/env python3
"""Meter reader ROS2 node.

Polls the meter_results directory for new PPM images, runs YOLO pose
inference, and publishes the meter reading value.
"""

import os
import math
import threading
from pathlib import Path

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32, String
from cv_bridge import CvBridge

import requests
from ultralytics import YOLO

from .meter_reader_core import calc_angle, angle_to_reading, run_inference

# ── drawing helpers ──────────────────────────────────────────────────────────

def _draw_annotation(img: np.ndarray, pts: list, quad_pts: list,
                     reading: float, angle: float, unit: str) -> np.ndarray:
    """Draw keypoints, lines, quad box, and reading text on a copy."""
    out = img.copy()

    # ── draw quad box (4 corners) ───────────────────────────────────────
    if len(quad_pts) == 4:
        quad = np.array(quad_pts, dtype=np.int32).reshape((-1, 1, 2))
        cv2.polylines(out, [quad], True, (0, 255, 255), 2)
        for i, (x, y) in enumerate(quad_pts):
            cv2.circle(out, (x, y), 5, (0, 255, 255), -1)
            cv2.putText(out, str(i + 1), (x + 7, y - 7),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 255), 2)

    # ── draw reading keypoints ──────────────────────────────────────────
    colors = [(0, 0, 255), (255, 0, 0), (0, 255, 0)]  # BGR
    labels = ["Center", "Zero", "Tip"]

    for i, (x, y) in enumerate(pts):
        cv2.circle(out, (x, y), 6, colors[i], -1)
        cv2.putText(out, labels[i], (x + 10, y - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, colors[i], 1)

    if len(pts) >= 2:
        cv2.line(out, pts[0], pts[1], (255, 0, 0), 2)
    if len(pts) >= 3:
        cv2.line(out, pts[0], pts[2], (0, 255, 0), 3)

    text = f"{reading:.1f} {unit}  |  {angle:.1f} deg"
    cv2.putText(out, text, (20, 40), cv2.FONT_HERSHEY_SIMPLEX,
                0.8, (0, 255, 255), 2)
    return out


# ── ROS2 Node ────────────────────────────────────────────────────────────────

class MeterReaderNode(Node):
    """Polls a directory for new PPM images and publishes meter readings."""

    def __init__(self):
        super().__init__('meter_reader')

        # ── parameters ──────────────────────────────────────────────────
        self.declare_parameter('results_dir', 'meter_results')
        self.declare_parameter('model_path', '')
        self.declare_parameter('total_angle', 90.0)
        self.declare_parameter('max_value', 500.0)
        self.declare_parameter('direction', 'cw')
        self.declare_parameter('unit', 'V')
        self.declare_parameter('poll_interval', 1.0)
        self.declare_parameter('publish_debug_image', True)
        self.declare_parameter('kpts_confidence_threshold', 0.1)
        self.declare_parameter('box_confidence', 0.01)
        self.declare_parameter('file_pattern', '*.ppm')
        self.declare_parameter('save_debug_images', True)
        self.declare_parameter('upload_enabled', True)
        self.declare_parameter('upload_url', 'http://192.168.110.219:5000/api/meter/upload')

        self._results_dir = Path(
            self.get_parameter('results_dir').get_parameter_value().string_value)
        self._total_angle = self.get_parameter('total_angle').value
        self._max_value = self.get_parameter('max_value').value
        self._direction = self.get_parameter('direction').value
        self._unit = self.get_parameter('unit').value
        self._poll_interval = self.get_parameter('poll_interval').value
        self._publish_debug = self.get_parameter('publish_debug_image').value
        self._kpts_thresh = self.get_parameter('kpts_confidence_threshold').value
        self._box_conf = self.get_parameter('box_confidence').value
        self._file_pattern = self.get_parameter('file_pattern').value
        self._save_debug = self.get_parameter('save_debug_images').value
        self._upload_enabled = self.get_parameter('upload_enabled').value
        self._upload_url = self.get_parameter('upload_url').get_parameter_value().string_value

        # resolve results_dir relative to CWD if needed
        if not self._results_dir.is_absolute():
            self._results_dir = Path.cwd() / self._results_dir

        # ── model ───────────────────────────────────────────────────────
        model_path = self.get_parameter('model_path').get_parameter_value().string_value
        if not model_path:
            from ament_index_python.packages import get_package_share_directory
            model_path = os.path.join(
                get_package_share_directory('meter_reader'),
                'models', 'best.pt')
        else:
            model_path = os.path.expanduser(model_path)

        self._model = None
        try:
            self._model = YOLO(model_path)
            self.get_logger().info(f'Model loaded: {model_path}')
        except Exception as e:
            self.get_logger().fatal(f'Failed to load model from {model_path}: {e}')

        # ── state ───────────────────────────────────────────────────────
        self._bridge = CvBridge()
        self._processed: set = set()  # filenames already processed

        # ── publishers ──────────────────────────────────────────────────
        qos = rclpy.qos.QoSProfile(depth=10)
        self._reading_pub = self.create_publisher(Float32, '/meter_reader/reading', qos)
        self._angle_pub = self.create_publisher(Float32, '/meter_reader/angle', qos)
        self._status_pub = self.create_publisher(String, '/meter_reader/status', qos)
        self._debug_pub = self.create_publisher(Image, '/meter_reader/debug_image', qos)

        # ── poll timer ──────────────────────────────────────────────────
        self._timer = self.create_timer(self._poll_interval, self._poll_callback)

        self.get_logger().info(
            f'MeterReader started, watching {self._results_dir}')
        self.get_logger().info(
            f'Params: box_conf={self._box_conf}, kpts_thresh={self._kpts_thresh}, '
            f'save_debug={self._save_debug}, pattern={self._file_pattern}')
        self._publish_status('WAITING')

    # ── polling ─────────────────────────────────────────────────────────────

    def _poll_callback(self):
        if self._model is None:
            return

        # find matching files, sorted by modification time (newest last)
        ppm_files = sorted(
            self._results_dir.glob(self._file_pattern),
            key=lambda p: p.stat().st_mtime)

        if not ppm_files:
            return

        # process all unprocessed files (newest first)
        new_files = [f for f in reversed(ppm_files)
                     if f.name not in self._processed]
        if not new_files:
            return

        for f in new_files:
            self._processed.add(f.name)
            self.get_logger().info(f'Processing: {f.name}')
            self._process_image(f)

        # keep set from growing unbounded (keep last 1000)
        if len(self._processed) > 1000:
            self._processed = set(list(self._processed)[-500:])

    # ── inference pipeline ──────────────────────────────────────────────────

    def _process_image(self, path: Path):
        try:
            # read image (supports PPM, JPG, PNG, etc.)
            raw = cv2.imread(str(path))
            if raw is None:
                self.get_logger().error(f'Failed to read image: {path}')
                return

            # convert to JPG in-memory to match training pipeline
            _, jpg_buf = cv2.imencode('.jpg', raw)
            img = cv2.imdecode(jpg_buf, cv2.IMREAD_COLOR)
            if img is None:
                self.get_logger().error(f'Failed to decode image: {path}')
                return

            pts, quad_pts, ok = run_inference(
                self._model, img, self._kpts_thresh, self._box_conf)

            if not ok:
                self.get_logger().warn('No meter detected')
                self._publish_status('NO_METER_DETECTED')
                self._publish_nan()
                return

            angle = calc_angle(pts[0], pts[1], pts[2], self._direction)
            reading = angle_to_reading(angle, self._total_angle, self._max_value)

            self.get_logger().info(
                f'  kpts={pts}  quad_pts={quad_pts}  '
                f'angle={angle:.1f}°  reading={reading:.1f}{self._unit}')

            # publish
            self._reading_pub.publish(Float32(data=float(reading)))
            self._angle_pub.publish(Float32(data=float(angle)))
            self._publish_status('OK')

            if self._publish_debug or self._save_debug:
                annotated = _draw_annotation(img, pts, quad_pts, reading, angle, self._unit)

            if self._publish_debug:
                debug_msg = self._bridge.cv2_to_imgmsg(annotated, encoding='bgr8')
                debug_msg.header.stamp = self.get_clock().now().to_msg()
                debug_msg.header.frame_id = 'meter_reader'
                self._debug_pub.publish(debug_msg)

            if self._save_debug:
                debug_dir = self._results_dir / 'debug'
                debug_dir.mkdir(exist_ok=True)
                stem = path.stem
                save_path = debug_dir / f'{stem}_result.jpg'
                cv2.imwrite(str(save_path), annotated)
                self.get_logger().info(f'  debug saved: {save_path}')

                # ── upload result image to Web dashboard ────────────────
                if self._upload_enabled and self._upload_url:
                    threading.Thread(
                        target=self._upload_result,
                        args=(save_path,),
                        daemon=True,
                    ).start()

        except Exception as e:
            self.get_logger().error(f'Error processing {path.name}: {e}')

    # ── helpers ─────────────────────────────────────────────────────────────

    def _publish_status(self, status: str):
        self._status_pub.publish(String(data=status))

    def _publish_nan(self):
        self._reading_pub.publish(Float32(data=float('nan')))
        self._angle_pub.publish(Float32(data=float('nan')))

    # ── upload ──────────────────────────────────────────────────────────────

    def _upload_result(self, image_path: Path):
        """POST the annotated result image to the Web dashboard.

        Runs in a separate thread to avoid blocking the ROS2 spin loop.
        Failures are logged but never raise — the inference pipeline is
        unaffected regardless of upload outcome.
        """
        try:
            with open(image_path, 'rb') as f:
                resp = requests.post(
                    self._upload_url,
                    files={'image': (image_path.name, f, 'image/jpeg')},
                    timeout=5.0,
                )
            if resp.status_code == 200:
                try:
                    body = resp.json()
                except Exception:
                    body = resp.text
                self.get_logger().info(
                    f'Uploaded to {self._upload_url}  success=True  '
                    f'response={body}')
            else:
                self.get_logger().warning(
                    f'Upload failed  status={resp.status_code}  url={self._upload_url}')
        except requests.exceptions.Timeout:
            self.get_logger().warning(
                f'Upload timeout (5s): {self._upload_url}')
        except requests.exceptions.ConnectionError:
            self.get_logger().warning(
                f'Upload connection error: {self._upload_url}')
        except Exception as e:
            self.get_logger().warning(
                f'Upload error: {e}  url={self._upload_url}')


def main(args=None):
    rclpy.init(args=args)
    node = MeterReaderNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
