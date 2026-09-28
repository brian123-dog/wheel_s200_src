from __future__ import annotations

from dataclasses import dataclass
from typing import Any

from rg_cabinet.adapters.camera import CameraAdapter, RGBDFrame
from rg_cabinet.adapters.robot import AdapterUnavailableError
from rg_cabinet.geometry import Intrinsics, Pose


@dataclass(frozen=True)
class Gemini2CommandMap:
    get_rgbd_frame: str | None = "get_rgbd_frame"
    get_color: str | None = None
    get_depth: str | None = None
    get_intrinsics: str | None = None
    get_camera_pose: str | None = None


class Gemini2Camera(CameraAdapter):
    """Wrapper for Gemini2 RGB-D SDK clients."""

    def __init__(
        self,
        client: Any,
        commands: Gemini2CommandMap | None = None,
        frame_id: str = "gemini2_color_optical_frame",
    ) -> None:
        self.client = client
        self.commands = commands or Gemini2CommandMap()
        self.frame_id = frame_id

    def _method(self, method_name: str | None):
        if method_name is None or not hasattr(self.client, method_name):
            raise AdapterUnavailableError(
                f"Gemini2 SDK client does not provide command: {method_name}"
            )
        return getattr(self.client, method_name)

    def get_frame(self) -> RGBDFrame:
        if self.commands.get_rgbd_frame and hasattr(
            self.client, self.commands.get_rgbd_frame
        ):
            return _coerce_frame(
                self._method(self.commands.get_rgbd_frame)(),
                default_frame_id=self.frame_id,
            )

        color = self._method(self.commands.get_color)()
        depth = self._method(self.commands.get_depth)()
        intrinsics = _coerce_intrinsics(self._method(self.commands.get_intrinsics)())
        camera_pose = None
        if self.commands.get_camera_pose and hasattr(
            self.client, self.commands.get_camera_pose
        ):
            camera_pose = Pose.from_sequence(
                self._method(self.commands.get_camera_pose)(), frame="base"
            )
        return RGBDFrame(
            color=color,
            depth=depth,
            intrinsics=intrinsics,
            frame_id=self.frame_id,
            camera_pose_in_base=camera_pose,
        )


def _coerce_frame(value: Any, default_frame_id: str) -> RGBDFrame:
    if isinstance(value, RGBDFrame):
        return value
    if not isinstance(value, dict):
        raise ValueError("Gemini2 get_rgbd_frame() must return RGBDFrame or dict")
    return RGBDFrame(
        color=value["color"],
        depth=value["depth"],
        intrinsics=_coerce_intrinsics(value["intrinsics"]),
        frame_id=str(value.get("frame_id", default_frame_id)),
        camera_pose_in_base=(
            Pose.from_sequence(value["camera_pose_in_base"], frame="base")
            if value.get("camera_pose_in_base") is not None
            else None
        ),
        stamp_s=value.get("stamp_s"),
    )


def _coerce_intrinsics(value: Any) -> Intrinsics:
    if isinstance(value, Intrinsics):
        return value
    if isinstance(value, dict):
        return Intrinsics(
            fx=float(value["fx"]),
            fy=float(value["fy"]),
            cx=float(value["cx"]),
            cy=float(value["cy"]),
            depth_scale=float(value.get("depth_scale", 1.0)),
        )
    return Intrinsics(*[float(v) for v in value])
