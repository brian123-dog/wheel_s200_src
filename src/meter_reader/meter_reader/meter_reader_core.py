"""Core inference logic for meter reading - no ROS dependencies."""

import math
import numpy as np


def normalize_angle(a: float) -> float:
    """Normalize angle to [0, 360)."""
    return a % 360.0


def get_math_angle(cx: float, cy: float, px: float, py: float) -> float:
    """Calculate mathematical angle from center (cx,cy) to point (px,py).

    Returns angle in degrees, 0° = right, counter-clockwise.
    """
    dx = px - cx
    dy = cy - py  # image Y-axis is inverted
    return normalize_angle(math.degrees(math.atan2(dy, dx)))


def calc_angle(center: tuple, zero: tuple, tip: tuple,
               direction: str = "cw") -> float:
    """Calculate the angle between zero-mark and needle tip relative to center.

    Args:
        center: (x, y) of meter center
        zero: (x, y) of zero-scale point
        tip: (x, y) of needle tip
        direction: "cw" (clockwise) or "ccw" (counter-clockwise)

    Returns:
        Angle in degrees.
    """
    za = get_math_angle(*center, *zero)
    ta = get_math_angle(*center, *tip)
    if direction == "cw":
        return normalize_angle(za - ta)
    else:
        return normalize_angle(ta - za)


def angle_to_reading(angle: float, total_angle: float,
                     max_value: float) -> float:
    """Convert angle to meter reading.

    Args:
        angle: Measured angle in degrees
        total_angle: Full-scale angle of the meter
        max_value: Maximum reading value

    Returns:
        Reading value, clamped to [0, max_value].
    """
    if angle > total_angle:
        angle = total_angle
    reading = max(0.0, min(max_value, angle / total_angle * max_value))
    return reading


def run_inference(model, image: np.ndarray,
                  confidence_threshold: float = 0.5,
                  box_conf: float = 0.02):
    """Run YOLO pose inference on an image.

    Supports both 3-keypoint (old) and 7-keypoint (new) models.
    - 7-keypoint model: indices 0-3 = quad corners, 4-6 = center/zero/tip
    - 3-keypoint model: indices 0-2 = center/zero/tip

    Args:
        model: Loaded ultralytics YOLO model.
        image: BGR image as numpy array (H, W, 3).
        confidence_threshold: Minimum keypoint confidence.
        box_conf: Minimum box confidence for detection.

    Returns:
        (pts, quad_pts, success) where:
        - pts is a list of 3 (x, y) tuples [center, zero, tip]
        - quad_pts is a list of 4 (x, y) tuples (empty for old model)
        - success is True if 3 valid reading keypoints were found
    """
    results = model(image, conf=box_conf, verbose=False)[0]

    if results.keypoints is None or results.keypoints.data.shape[0] == 0:
        return [], [], False

    kpts = results.keypoints.data[0].cpu().numpy()
    num_kpts = len(kpts)

    if num_kpts >= 7:
        # New 7-keypoint model: indices 4,5,6 = center, zero, tip
        read_indices = [4, 5, 6]
        quad_pts = [(int(kpts[i][0]), int(kpts[i][1])) for i in range(4)]
    elif num_kpts >= 3:
        # Old 3-keypoint model: indices 0,1,2 = center, zero, tip
        read_indices = [0, 1, 2]
        quad_pts = []
    else:
        return [], [], False

    pts = []
    for i in read_indices:
        x, y, conf = kpts[i][0], kpts[i][1], kpts[i][2]
        if conf < confidence_threshold:
            return [], [], False
        pts.append((int(x), int(y)))

    return pts, quad_pts, True
