from __future__ import annotations

from abc import ABC, abstractmethod
from dataclasses import dataclass
from typing import Sequence

from rg_cabinet.geometry import Pose, Twist


class AdapterError(RuntimeError):
    """Base class for robot/camera adapter failures."""


class AdapterUnavailableError(AdapterError):
    """Raised when a configured SDK command is not available."""


@dataclass(frozen=True)
class ContactThreshold:
    max_force_n: float | None = None
    max_current_a: float | None = None


class RobotAdapter(ABC):
    """Hardware-neutral manipulator interface used by cabinet skills."""

    @abstractmethod
    def move_pose(
        self,
        pose: Pose,
        speed: float = 0.05,
        acceleration: float = 0.1,
        wait: bool = True,
    ) -> None:
        raise NotImplementedError

    @abstractmethod
    def move_joint(
        self,
        joints: Sequence[float],
        speed: float = 0.2,
        acceleration: float = 0.4,
        wait: bool = True,
    ) -> None:
        raise NotImplementedError

    @abstractmethod
    def servo_twist(self, twist: Twist, duration_s: float, frame: str = "tool") -> None:
        raise NotImplementedError

    @abstractmethod
    def stop(self) -> None:
        raise NotImplementedError

    @abstractmethod
    def set_gripper(self, position: float, speed: float | None = None) -> None:
        raise NotImplementedError

    @abstractmethod
    def get_tcp_pose(self) -> Pose:
        raise NotImplementedError

    @abstractmethod
    def force_or_current_limit(self, threshold: ContactThreshold) -> bool:
        raise NotImplementedError
