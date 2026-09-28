from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Sequence

from rg_cabinet.adapters.robot import (
    AdapterUnavailableError,
    ContactThreshold,
    RobotAdapter,
)
from rg_cabinet.geometry import Pose, Twist


@dataclass(frozen=True)
class RealManCommandMap:
    move_pose: str = "move_pose"
    move_joint: str = "move_joint"
    servo_twist: str = "servo_twist"
    stop: str = "stop"
    set_gripper: str = "set_gripper"
    get_tcp_pose: str = "get_tcp_pose"
    get_wrench: str | None = None
    get_joint_currents: str | None = None


class RealManAdapter(RobotAdapter):
    """Thin wrapper around a RealMan SDK client.

    The exact SDK method names vary by installation, so command names are
    configurable. Methods are deliberately small and easy to audit before
    enabling hardware motion.
    """

    def __init__(
        self,
        client: Any,
        commands: RealManCommandMap | None = None,
    ) -> None:
        self.client = client
        self.commands = commands or RealManCommandMap()

    def _method(self, method_name: str | None):
        if method_name is None or not hasattr(self.client, method_name):
            raise AdapterUnavailableError(
                f"RealMan SDK client does not provide command: {method_name}"
            )
        return getattr(self.client, method_name)

    def move_pose(
        self,
        pose: Pose,
        speed: float = 0.05,
        acceleration: float = 0.1,
        wait: bool = True,
    ) -> None:
        self._method(self.commands.move_pose)(pose.as_list(), speed, acceleration, wait)

    def move_joint(
        self,
        joints: Sequence[float],
        speed: float = 0.2,
        acceleration: float = 0.4,
        wait: bool = True,
    ) -> None:
        self._method(self.commands.move_joint)(list(joints), speed, acceleration, wait)

    def servo_twist(self, twist: Twist, duration_s: float, frame: str = "tool") -> None:
        self._method(self.commands.servo_twist)(twist.as_list(), duration_s, frame)

    def stop(self) -> None:
        self._method(self.commands.stop)()

    def set_gripper(self, position: float, speed: float | None = None) -> None:
        if speed is None:
            self._method(self.commands.set_gripper)(position)
        else:
            self._method(self.commands.set_gripper)(position, speed)

    def get_tcp_pose(self) -> Pose:
        value = self._method(self.commands.get_tcp_pose)()
        return _coerce_pose(value)

    def force_or_current_limit(self, threshold: ContactThreshold) -> bool:
        if threshold.max_force_n is not None and self.commands.get_wrench:
            wrench = self._method(self.commands.get_wrench)()
            if max(abs(v) for v in _coerce_vector(wrench)[:3]) >= threshold.max_force_n:
                return True

        if threshold.max_current_a is not None and self.commands.get_joint_currents:
            currents = self._method(self.commands.get_joint_currents)()
            if max(abs(v) for v in _coerce_vector(currents)) >= threshold.max_current_a:
                return True

        return False


def _coerce_pose(value: Any) -> Pose:
    if isinstance(value, Pose):
        return value
    if isinstance(value, dict):
        frame = str(value.get("frame", "base"))
        if "pose" in value:
            return Pose.from_sequence(value["pose"], frame=frame)
        return Pose(
            float(value["x"]),
            float(value["y"]),
            float(value["z"]),
            float(value.get("rx", 0.0)),
            float(value.get("ry", 0.0)),
            float(value.get("rz", 0.0)),
            frame,
        )
    return Pose.from_sequence(value, frame="base")


def _coerce_vector(value: Any) -> list[float]:
    if isinstance(value, dict):
        if "values" in value:
            return [float(v) for v in value["values"]]
        ordered = [value[k] for k in sorted(value)]
        return [float(v) for v in ordered]
    return [float(v) for v in value]
