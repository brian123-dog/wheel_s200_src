"""Bounded, stationary sampling. This module cannot command robot motion."""
from dataclasses import dataclass, fields
import math
import time

from rg_cabinet_ros2.camera_cache import FreshFrameTimeout


class PanelLayoutRejected(RuntimeError):
    """Image evidence was rejected; another fresh image may resolve it."""


@dataclass(frozen=True)
class SamplingPolicy:
    max_attempts: int = 5
    total_timeout_s: float = 6.0
    frame_wait_timeout_s: float = 2.0
    max_frame_age_s: float = 0.75
    max_rgbd_skew_s: float = 0.15
    confirmation_frames: int = 2
    max_center_delta_px: float = 5.0
    max_depth_delta_m: float = 0.015

    @classmethod
    def from_config(cls, config):
        policy = cls(**{f.name: config[f.name] for f in fields(cls) if f.name in config})
        if not isinstance(policy.max_attempts, int) or not 2 <= policy.max_attempts <= 10:
            raise ValueError('fresh_sampling.max_attempts must be an integer in [2, 10]')
        if not isinstance(policy.confirmation_frames, int) or not 1 <= policy.confirmation_frames <= policy.max_attempts:
            raise ValueError('fresh_sampling.confirmation_frames must be between 1 and max_attempts')
        for f in fields(cls):
            value = getattr(policy, f.name)
            if not math.isfinite(value) or value <= 0:
                raise ValueError('fresh_sampling.' + f.name + ' must be finite and positive')
        if policy.total_timeout_s > 15:
            raise ValueError('fresh_sampling.total_timeout_s must not exceed 15 seconds')
        return policy


def consistent(first, second, policy):
    a, b = first.details, second.details
    if (first.name, first.pose.frame, a.get('camera_frame'), a.get('image_size')) != (
            second.name, second.pose.frame, b.get('camera_frame'), b.get('image_size')):
        return False
    pa, pb = a['axis_center_pixel'], b['axis_center_pixel']
    return (math.hypot(pa[0] - pb[0], pa[1] - pb[1]) <= policy.max_center_delta_px
            and abs(a['depth_m'] - b['depth_m']) <= policy.max_depth_delta_m)


def sample_target(cache, policy, now_s, detect, on_failure):
    start = time.monotonic()
    deadline = start + policy.total_timeout_s
    not_before = now_s()
    previous = cache.snapshot().stamps
    accepted = None
    confirmations = 0
    history = []
    for attempt in range(1, policy.max_attempts + 1):
        remaining = deadline - time.monotonic()
        if remaining <= 0:
            break
        item = {'attempt': attempt}
        frame = None
        wait_start = time.monotonic()
        detection_start = None
        try:
            frame, frame_debug = cache.wait_snapshot(
                after=previous, not_before_s=not_before,
                timeout_s=min(policy.frame_wait_timeout_s, remaining), now_s=now_s,
                max_age_s=policy.max_frame_age_s, max_skew_s=policy.max_rgbd_skew_s)
            item['frame_wait_s'] = time.monotonic() - wait_start
            previous = frame.stamps
            item['frames'] = frame_debug
            detection_start = time.monotonic()
            target = detect(frame)
            item['detection_s'] = time.monotonic() - detection_start
            if time.monotonic() > deadline:
                raise PanelLayoutRejected(
                    f'Vision sampling time budget exhausted during detection '
                    f'(elapsed={time.monotonic() - start:.2f}s, budget={policy.total_timeout_s:.2f}s; '
                    f'detection accepted, fresh-frame confirmation incomplete)')
            if accepted is not None and not consistent(accepted, target, policy):
                item['reason'] = 'Accepted detections disagree in axis position or depth'
                on_failure(item, frame)
                confirmations = 0
                accepted = None
            if accepted is None:
                accepted = target
            confirmations += 1
            item['status'] = 'confirmed' if confirmations >= policy.confirmation_frames else 'candidate'
            history.append(item)
            if confirmations >= policy.confirmation_frames:
                return target, {'enabled': True, 'attempts': attempt,
                                'confirmations': confirmations,
                                'elapsed_s': time.monotonic() - start,
                                'total_timeout_s': policy.total_timeout_s, 'history': history}
        except (PanelLayoutRejected, FreshFrameTimeout) as exc:
            if detection_start is not None:
                item['detection_s'] = time.monotonic() - detection_start
            else:
                item['frame_wait_s'] = time.monotonic() - wait_start
            if isinstance(exc, FreshFrameTimeout):
                frame = exc.frame
                item['frames'] = exc.diagnostics
                item['frame_timeout'] = True
            item.update(status='rejected', reason=str(exc))
            failure_start = time.monotonic()
            on_failure(item, frame)
            item['failure_artifacts_s'] = time.monotonic() - failure_start
            history.append(item)
            accepted = None
            confirmations = 0
    report = {'enabled': True, 'attempts': len(history), 'confirmations': confirmations,
              'elapsed_s': time.monotonic() - start,
              'total_timeout_s': policy.total_timeout_s, 'history': history}
    reason = history[-1].get('reason', 'Not enough consistent accepted frames') if history else 'Sampling deadline reached'
    raise VisionSamplingError(reason, report)


class VisionSamplingError(RuntimeError):
    def __init__(self, reason, report):
        self.report = report
        super().__init__(f"Vision sampling failed after {report['attempts']} attempts: {reason}")
