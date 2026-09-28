from __future__ import annotations

from abc import ABC, abstractmethod
from dataclasses import dataclass
from typing import Any

from rg_cabinet.geometry import Intrinsics, Pose


@dataclass(frozen=True)
class RGBDFrame:
    color: Any
    depth: Any
    intrinsics: Intrinsics
    frame_id: str = "camera"
    camera_pose_in_base: Pose | None = None
    stamp_s: float | None = None


class CameraAdapter(ABC):
    @abstractmethod
    def get_frame(self) -> RGBDFrame:
        raise NotImplementedError
