from __future__ import annotations

import time
from dataclasses import dataclass
from typing import Any


@dataclass
class CameraFrameCache:
    color: Any | None = None
    depth: Any | None = None
    camera_info: Any | None = None
    _color_stamp_s: float = 0.0
    _depth_stamp_s: float = 0.0

    @property
    def ready(self) -> bool:
        return (
            self.color is not None
            and self.depth is not None
            and self.camera_info is not None
        )

    def mark_fresh(self) -> None:
        """标记当前帧为"新帧基准"，后续 wait_fresh 以此为界"""
        self._color_stamp_s = self._color_timestamp()
        self._depth_stamp_s = self._depth_timestamp()

    def wait_fresh(self, timeout_s: float = 2.0) -> bool:
        """等待比 mark_fresh 时刻更新的帧到达"""
        deadline = time.monotonic() + float(timeout_s)
        while time.monotonic() < deadline:
            if (
                self._color_timestamp() > self._color_stamp_s
                and self._depth_timestamp() > self._depth_stamp_s
            ):
                return True
            time.sleep(0.05)
        return False

    def _color_timestamp(self) -> float:
        if self.color is None:
            return 0.0
        stamp = getattr(self.color, "header", None)
        if stamp is None:
            return 0.0
        return stamp.stamp.sec + stamp.stamp.nanosec * 1e-9

    def _depth_timestamp(self) -> float:
        if self.depth is None:
            return 0.0
        stamp = getattr(self.depth, "header", None)
        if stamp is None:
            return 0.0
        return stamp.stamp.sec + stamp.stamp.nanosec * 1e-9

    def summary(self) -> dict[str, bool]:
        return {
            "has_color": self.color is not None,
            "has_depth": self.depth is not None,
            "has_camera_info": self.camera_info is not None,
            "ready": self.ready,
        }

    def snapshot(self):
        # Keep references to this pair: callbacks may replace the cache while
        # OpenCV is working, but must not change which depth image it uses.
        return FrameSnapshot(self.color, self.depth, self.camera_info)

    def wait_snapshot(self, *, after, not_before_s, timeout_s, now_s,
                      max_age_s, max_skew_s):
        deadline = time.monotonic() + timeout_s
        while True:
            frame = self.snapshot()
            diagnostics = frame.diagnostics(now_s(), after, not_before_s,
                                            max_age_s, max_skew_s)
            if not diagnostics['problems']:
                return frame, diagnostics
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                raise FreshFrameTimeout(frame, diagnostics)
            time.sleep(min(0.02, remaining))


def image_stamp_s(image):
    stamp = getattr(getattr(image, 'header', None), 'stamp', None)
    return (float(stamp.sec) + float(stamp.nanosec) * 1e-9) if stamp is not None else 0.0


@dataclass(frozen=True)
class FrameSnapshot:
    color: Any
    depth: Any
    camera_info: Any

    @property
    def stamps(self):
        return image_stamp_s(self.color), image_stamp_s(self.depth)

    def diagnostics(self, now_s, after, not_before_s, max_age_s, max_skew_s):
        import math
        stamps = self.stamps
        ages = [now_s - stamp for stamp in stamps]
        problems = []
        for i, name in enumerate(('color', 'depth')):
            if not math.isfinite(stamps[i]) or stamps[i] <= 0:
                problems.append(name + ': invalid timestamp')
            elif stamps[i] <= after[i] or stamps[i] <= not_before_s:
                problems.append(name + ': no new frame after sampling started')
            if not math.isfinite(ages[i]) or ages[i] > max_age_s:
                problems.append(name + ': stale frame')
            elif ages[i] < -0.1:
                problems.append(name + ': timestamp in future')
        skew = abs(stamps[0] - stamps[1])
        if skew > max_skew_s:
            problems.append('color/depth timestamp skew too large')
        if self.camera_info is None:
            problems.append('camera_info missing')
        return {'color_stamp_s': stamps[0], 'depth_stamp_s': stamps[1],
                'color_age_s': ages[0], 'depth_age_s': ages[1],
                'rgbd_skew_s': skew, 'not_before_s': not_before_s,
                'previous_stamps_s': list(after), 'problems': problems}


class FreshFrameTimeout(RuntimeError):
    def __init__(self, frame, diagnostics):
        self.frame = frame
        self.diagnostics = diagnostics
        super().__init__('Fresh RGB-D frame timeout: ' + '; '.join(diagnostics['problems']))
