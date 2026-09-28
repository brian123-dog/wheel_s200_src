from __future__ import annotations

from dataclasses import dataclass

from rg_cabinet.perception.element_detector import ElementTarget
from rg_cabinet.skills.base import (
    ContactMotionConfig,
    SkillContext,
    SkillResult,
    guarded_approach,
    offset_pose,
    servo_distance,
)


@dataclass(frozen=True)
class ToggleSwitchConfig:
    pre_offset_m: float = 0.035
    contact_start_offset_m: float = 0.006
    retract_m: float = 0.025
    approach_axis: str = "z"
    approach_sign: float = -1.0
    approach_offset_frame: str = "base"
    axis: str = "y"
    throw_distance_m: float = 0.025
    slide_speed_mps: float = 0.015
    move_speed_mps: float = 0.05
    gripper_position: float = 1.0
    contact: ContactMotionConfig = ContactMotionConfig()


class ToggleSwitchSkill:
    def __init__(self, config: ToggleSwitchConfig | None = None) -> None:
        self.config = config or ToggleSwitchConfig()

    def execute(self, context: SkillContext, target: ElementTarget) -> SkillResult:
        robot = context.robot
        cfg = self.config
        robot.set_gripper(cfg.gripper_position)
        robot.move_pose(
            offset_pose(
                target.pose,
                cfg.approach_axis,
                -cfg.approach_sign * cfg.pre_offset_m,
                cfg.approach_offset_frame,
            ),
            cfg.move_speed_mps,
        )
        robot.move_pose(
            offset_pose(
                target.pose,
                cfg.approach_axis,
                -cfg.approach_sign * cfg.contact_start_offset_m,
                cfg.approach_offset_frame,
            ),
            cfg.move_speed_mps,
        )
        contact_detected = guarded_approach(
            robot,
            cfg.contact,
            axis=cfg.approach_axis,
            sign=cfg.approach_sign,
            frame=cfg.approach_offset_frame,
        )
        servo_distance(robot, cfg.axis, cfg.throw_distance_m, cfg.slide_speed_mps)
        servo_distance(
            robot,
            cfg.approach_axis,
            -cfg.approach_sign * cfg.retract_m,
            max(cfg.contact.approach_speed_mps, 0.001),
            frame=cfg.approach_offset_frame,
        )
        return SkillResult(
            name=target.name,
            success=True,
            details={
                "contact_detected": contact_detected,
                "throw_distance_m": cfg.throw_distance_m,
            },
        )
