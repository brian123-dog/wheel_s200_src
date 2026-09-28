from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Callable

from rg_cabinet.adapters.robot import ContactThreshold, RobotAdapter
from rg_cabinet.geometry import Pose, Twist


@dataclass(frozen=True)
class ContactMotionConfig:
    approach_speed_mps: float = 0.005
    servo_period_s: float = 0.1
    max_contact_travel_m: float = 0.03
    timeout_s: float = 6.0
    contact_force_n: float | None = 5.0
    contact_current_a: float | None = None


@dataclass
class SkillContext:
    robot: RobotAdapter
    target_refresher: Callable[[], Any | None] | None = None
    relax_vision: bool = False
    camera_cache: Any | None = None
    precontact_centering: bool = False


@dataclass(frozen=True)
class SkillResult:
    name: str
    success: bool
    details: dict[str, Any] = field(default_factory=dict)


def guarded_approach(
    robot: RobotAdapter,
    config: ContactMotionConfig,
    axis: str = "z",
    sign: float = -1.0,
    frame: str = "tool",
) -> bool:
    elapsed = 0.0
    traveled = 0.0
    threshold = ContactThreshold(
        max_force_n=config.contact_force_n,
        max_current_a=config.contact_current_a,
    )
    velocity = abs(config.approach_speed_mps) * (1.0 if sign >= 0 else -1.0)

    while (
        elapsed < config.timeout_s
        and traveled < config.max_contact_travel_m
    ):
        robot.servo_twist(
            _linear_twist(axis, velocity),
            duration_s=config.servo_period_s,
            frame=frame,
        )
        elapsed += config.servo_period_s
        traveled += abs(velocity * config.servo_period_s)
        if robot.force_or_current_limit(threshold):
            robot.stop()
            return True

    robot.stop()
    return False


def servo_distance(
    robot: RobotAdapter,
    axis: str,
    distance_m: float,
    speed_mps: float,
    frame: str = "tool",
) -> None:
    if speed_mps <= 0.0:
        raise ValueError("speed_mps must be positive")
    duration = abs(distance_m) / speed_mps
    velocity = speed_mps if distance_m >= 0.0 else -speed_mps
    robot.servo_twist(_linear_twist(axis, velocity), duration_s=duration, frame=frame)
    robot.stop()


def servo_rotation(
    robot: RobotAdapter,
    axis: str,
    angle_rad: float,
    speed_radps: float,
    frame: str = "tool",
) -> None:
    if speed_radps <= 0.0:
        raise ValueError("speed_radps must be positive")
    duration = abs(angle_rad) / speed_radps
    velocity = speed_radps if angle_rad >= 0.0 else -speed_radps
    robot.servo_twist(_angular_twist(axis, velocity), duration_s=duration, frame=frame)
    robot.stop()


def offset_pose(
    pose: Pose,
    axis: str,
    distance_m: float,
    frame: str = "base",
) -> Pose:
    if frame in ("tool", "target", "local"):
        return pose.translated_local(axis, distance_m)
    if frame in ("base", pose.frame):
        if axis == "x":
            return pose.translated(dx=distance_m)
        if axis == "y":
            return pose.translated(dy=distance_m)
        if axis == "z":
            return pose.translated(dz=distance_m)
    raise ValueError(f"Unsupported offset frame/axis: frame={frame}, axis={axis}")


def _linear_twist(axis: str, velocity: float) -> Twist:
    if axis == "x":
        return Twist(linear_x=velocity)
    if axis == "y":
        return Twist(linear_y=velocity)
    if axis == "z":
        return Twist(linear_z=velocity)
    raise ValueError(f"Unsupported linear axis: {axis}")


def _angular_twist(axis: str, velocity: float) -> Twist:
    if axis == "x":
        return Twist(angular_x=velocity)
    if axis == "y":
        return Twist(angular_y=velocity)
    if axis == "z":
        return Twist(angular_z=velocity)
    raise ValueError(f"Unsupported angular axis: {axis}")
