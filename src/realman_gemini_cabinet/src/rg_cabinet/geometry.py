from __future__ import annotations

from dataclasses import dataclass
from math import cos, sin, sqrt
from typing import Iterable


@dataclass(frozen=True)
class Pose:
    """Cartesian pose in a named frame.

    The rotation fields use a compact rx, ry, rz representation. Hardware
    adapters can interpret this as roll-pitch-yaw or rotation-vector depending
    on the underlying SDK; skills only need a stable container.
    """

    x: float
    y: float
    z: float
    rx: float = 0.0
    ry: float = 0.0
    rz: float = 0.0
    frame: str = "base"

    @classmethod
    def from_sequence(cls, values: Iterable[float], frame: str = "base") -> "Pose":
        data = list(values)
        if len(data) not in (3, 6):
            raise ValueError("Pose sequences must contain either 3 or 6 numbers")
        if len(data) == 3:
            data.extend([0.0, 0.0, 0.0])
        return cls(*[float(v) for v in data], frame=frame)

    def as_list(self) -> list[float]:
        return [self.x, self.y, self.z, self.rx, self.ry, self.rz]

    def translated(
        self,
        dx: float = 0.0,
        dy: float = 0.0,
        dz: float = 0.0,
        frame: str | None = None,
    ) -> "Pose":
        return Pose(
            self.x + dx,
            self.y + dy,
            self.z + dz,
            self.rx,
            self.ry,
            self.rz,
            frame or self.frame,
        )

    def translated_local(
        self,
        axis: str,
        distance: float,
        frame: str | None = None,
    ) -> "Pose":
        dx, dy, dz = local_axis_to_world_delta(
            axis,
            distance,
            self.rx,
            self.ry,
            self.rz,
        )
        return self.translated(dx=dx, dy=dy, dz=dz, frame=frame)

    def rotated(
        self,
        drx: float = 0.0,
        dry: float = 0.0,
        drz: float = 0.0,
        frame: str | None = None,
    ) -> "Pose":
        return Pose(
            self.x,
            self.y,
            self.z,
            self.rx + drx,
            self.ry + dry,
            self.rz + drz,
            frame or self.frame,
        )

    def distance_to(self, other: "Pose") -> float:
        return sqrt(
            (self.x - other.x) ** 2
            + (self.y - other.y) ** 2
            + (self.z - other.z) ** 2
        )


@dataclass(frozen=True)
class Twist:
    linear_x: float = 0.0
    linear_y: float = 0.0
    linear_z: float = 0.0
    angular_x: float = 0.0
    angular_y: float = 0.0
    angular_z: float = 0.0

    def as_list(self) -> list[float]:
        return [
            self.linear_x,
            self.linear_y,
            self.linear_z,
            self.angular_x,
            self.angular_y,
            self.angular_z,
        ]


@dataclass(frozen=True)
class Intrinsics:
    fx: float
    fy: float
    cx: float
    cy: float
    depth_scale: float = 1.0


def backproject_pixel(
    u: float,
    v: float,
    depth_value: float,
    intrinsics: Intrinsics,
    frame: str = "camera",
) -> Pose:
    z = float(depth_value) * intrinsics.depth_scale
    if z <= 0.0:
        raise ValueError("Depth must be positive for back-projection")
    x = (float(u) - intrinsics.cx) * z / intrinsics.fx
    y = (float(v) - intrinsics.cy) * z / intrinsics.fy
    return Pose(x=x, y=y, z=z, frame=frame)


def local_axis_to_world_delta(
    axis: str,
    distance: float,
    roll: float,
    pitch: float,
    yaw: float,
) -> tuple[float, float, float]:
    if axis == "x":
        local = (float(distance), 0.0, 0.0)
    elif axis == "y":
        local = (0.0, float(distance), 0.0)
    elif axis == "z":
        local = (0.0, 0.0, float(distance))
    else:
        raise ValueError(f"Unsupported local axis: {axis}")
    return rotate_rpy(local, roll, pitch, yaw)


def rotate_rpy(
    vector: tuple[float, float, float],
    roll: float,
    pitch: float,
    yaw: float,
) -> tuple[float, float, float]:
    x, y, z = vector
    cr = cos(roll)
    sr = sin(roll)
    cp = cos(pitch)
    sp = sin(pitch)
    cy = cos(yaw)
    sy = sin(yaw)

    # Rotation order matches the common fixed-axis roll-pitch-yaw convention:
    # R = Rz(yaw) * Ry(pitch) * Rx(roll).
    x1 = x
    y1 = cr * y - sr * z
    z1 = sr * y + cr * z

    x2 = cp * x1 + sp * z1
    y2 = y1
    z2 = -sp * x1 + cp * z1

    x3 = cy * x2 - sy * y2
    y3 = sy * x2 + cy * y2
    z3 = z2
    return x3, y3, z3
