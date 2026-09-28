from __future__ import annotations

from dataclasses import dataclass, field
from statistics import median
from typing import Any, Iterable

from rg_cabinet.adapters.camera import RGBDFrame
from rg_cabinet.geometry import Pose, backproject_pixel


class ElementNotFound(RuntimeError):
    pass


@dataclass(frozen=True)
class ColorRange:
    lower_rgb: tuple[int, int, int]
    upper_rgb: tuple[int, int, int]

    def contains(self, rgb: tuple[int, int, int]) -> bool:
        return all(
            low <= value <= high
            for value, low, high in zip(rgb, self.lower_rgb, self.upper_rgb)
        )


@dataclass(frozen=True)
class DetectionSpec:
    name: str
    kind: str
    color_range: ColorRange
    min_pixels: int = 8
    target_frame: str = "base"


@dataclass(frozen=True)
class ElementTarget:
    name: str
    kind: str
    pose: Pose
    confidence: float = 1.0
    details: dict[str, Any] = field(default_factory=dict)


class ColorBlobElementDetector:
    """Simple RGB-D detector for early cabinet prototypes.

    It expects a color image indexable as color[v][u] with RGB or BGR-like
    triplets already normalized to RGB ordering by the camera adapter. Depth is
    indexable as depth[v][u].
    """

    def detect(self, frame: RGBDFrame, spec: DetectionSpec) -> ElementTarget:
        pixels: list[tuple[int, int]] = []
        for v, row in enumerate(frame.color):
            for u, pixel in enumerate(row):
                if spec.color_range.contains(_coerce_rgb(pixel)):
                    pixels.append((u, v))

        if len(pixels) < spec.min_pixels:
            raise ElementNotFound(
                f"{spec.name} found {len(pixels)} pixels; need {spec.min_pixels}"
            )

        u_center = sum(u for u, _ in pixels) / len(pixels)
        v_center = sum(v for _, v in pixels) / len(pixels)
        depth = _median_positive_depth(frame.depth, pixels)
        camera_pose = backproject_pixel(
            u_center, v_center, depth, frame.intrinsics, frame=frame.frame_id
        )

        if frame.camera_pose_in_base is None:
            pose = Pose(
                camera_pose.x,
                camera_pose.y,
                camera_pose.z,
                frame=spec.target_frame,
            )
        else:
            # This translation-only composition is a conservative placeholder.
            # Replace with calibrated SE(3) math before production use.
            pose = frame.camera_pose_in_base.translated(
                camera_pose.x,
                camera_pose.y,
                camera_pose.z,
                frame=spec.target_frame,
            )

        return ElementTarget(
            name=spec.name,
            kind=spec.kind,
            pose=pose,
            confidence=min(1.0, len(pixels) / max(spec.min_pixels, 1)),
        )


def _coerce_rgb(pixel: Any) -> tuple[int, int, int]:
    values = list(pixel)
    if len(values) < 3:
        raise ValueError("Color pixels must contain at least three channels")
    return tuple(int(max(0, min(255, v))) for v in values[:3])


def _median_positive_depth(depth_image: Any, pixels: Iterable[tuple[int, int]]) -> float:
    depths = []
    for u, v in pixels:
        value = float(depth_image[v][u])
        if value > 0.0:
            depths.append(value)
    if not depths:
        raise ElementNotFound("Detected color blob has no positive depth values")
    return float(median(depths))
