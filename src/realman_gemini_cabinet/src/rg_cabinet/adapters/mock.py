from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Any, Sequence

from rg_cabinet.adapters.robot import ContactThreshold, RobotAdapter
from rg_cabinet.geometry import Pose, Twist


@dataclass(frozen=True)
class RecordedCommand:
    name: str
    payload: dict[str, Any] = field(default_factory=dict)


class MockRobotAdapter(RobotAdapter):
    """Deterministic robot adapter for task development and tests."""

    def __init__(
        self,
        initial_pose: Pose | None = None,
        contact_after_servo_calls: int | None = None,
    ) -> None:
        self.current_pose = initial_pose or Pose(0.0, 0.0, 0.0)
        self.contact_after_servo_calls = contact_after_servo_calls
        self.commands: list[RecordedCommand] = []
        self.gripper_position = 0.0
        self._servo_calls = 0

    def move_pose(
        self,
        pose: Pose,
        speed: float = 0.05,
        acceleration: float = 0.1,
        wait: bool = True,
    ) -> None:
        self.commands.append(
            RecordedCommand(
                "move_pose",
                {
                    "pose": pose,
                    "speed": speed,
                    "acceleration": acceleration,
                    "wait": wait,
                },
            )
        )
        self.current_pose = pose

    def move_joint(
        self,
        joints: Sequence[float],
        speed: float = 0.2,
        acceleration: float = 0.4,
        wait: bool = True,
    ) -> None:
        self.commands.append(
            RecordedCommand(
                "move_joint",
                {
                    "joints": list(joints),
                    "speed": speed,
                    "acceleration": acceleration,
                    "wait": wait,
                },
            )
        )

    def move_joint_pose(
        self,
        position: dict[str, float],
        orientation: dict[str, float],
        speed: float = 0.05,
        acceleration: float = 0.1,
        wait: bool = True,
    ) -> None:
        rx, ry, rz = _quaternion_to_rpy(
            float(orientation["x"]),
            float(orientation["y"]),
            float(orientation["z"]),
            float(orientation["w"]),
        )
        pose = Pose(
            float(position["x"]),
            float(position["y"]),
            float(position["z"]),
            rx,
            ry,
            rz,
        )
        self.commands.append(
            RecordedCommand(
                "move_joint_pose",
                {
                    "position": dict(position),
                    "orientation": dict(orientation),
                    "speed": speed,
                    "acceleration": acceleration,
                    "wait": wait,
                },
            )
        )
        self.current_pose = pose

    def servo_twist(self, twist: Twist, duration_s: float, frame: str = "tool") -> None:
        self.commands.append(
            RecordedCommand(
                "servo_twist",
                {"twist": twist, "duration_s": duration_s, "frame": frame},
            )
        )
        self._servo_calls += 1
        self.current_pose = self.current_pose.translated(
            dx=twist.linear_x * duration_s,
            dy=twist.linear_y * duration_s,
            dz=twist.linear_z * duration_s,
        ).rotated(
            drx=twist.angular_x * duration_s,
            dry=twist.angular_y * duration_s,
            drz=twist.angular_z * duration_s,
        )

    def stop(self) -> None:
        self.commands.append(RecordedCommand("stop", {}))

    def set_gripper(self, position: float, speed: float | None = None) -> None:
        self.commands.append(
            RecordedCommand("set_gripper", {"position": position, "speed": speed})
        )
        self.gripper_position = position

    def get_tcp_pose(self) -> Pose:
        self.commands.append(RecordedCommand("get_tcp_pose", {}))
        return self.current_pose

    def force_or_current_limit(self, threshold: ContactThreshold) -> bool:
        self.commands.append(
            RecordedCommand("force_or_current_limit", {"threshold": threshold})
        )
        if self.contact_after_servo_calls is None:
            return False
        return self._servo_calls >= self.contact_after_servo_calls


def _quaternion_to_rpy(x: float, y: float, z: float, w: float) -> tuple[float, float, float]:
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if norm <= 1e-12:
        raise ValueError("Quaternion must not be zero")
    x /= norm
    y /= norm
    z /= norm
    w /= norm

    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (w * y - z * x)
    if abs(sinp) >= 1.0:
        pitch = math.copysign(math.pi / 2.0, sinp)
    else:
        pitch = math.asin(sinp)

    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return roll, pitch, yaw
