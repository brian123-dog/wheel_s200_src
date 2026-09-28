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
class ButtonPressConfig:
    pre_offset_m: float = 0.04
    press_start_offset_m: float = 0.008
    retract_m: float = 0.03
    approach_axis: str = "z"
    approach_sign: float = -1.0
    approach_offset_frame: str = "base"
    move_speed_mps: float = 0.05
    gripper_position: float = 1.0
    contact: ContactMotionConfig = ContactMotionConfig()


class ButtonPressSkill:
    def __init__(self, config: ButtonPressConfig | None = None) -> None:
        self.config = config or ButtonPressConfig()

    def execute(self, context: SkillContext, target: ElementTarget) -> SkillResult:
        robot = context.robot
        cfg = self.config
        pre_pose = offset_pose(
            target.pose,
            cfg.approach_axis,
            -cfg.approach_sign * cfg.pre_offset_m,
            cfg.approach_offset_frame,
        )
        start_pose = offset_pose(
            target.pose,
            cfg.approach_axis,
            -cfg.approach_sign * cfg.press_start_offset_m,
            cfg.approach_offset_frame,
        )

        robot.set_gripper(cfg.gripper_position)
        robot.move_pose(pre_pose, speed=cfg.move_speed_mps)
        robot.move_pose(start_pose, speed=cfg.move_speed_mps)
        contact_detected = guarded_approach(
            robot,
            cfg.contact,
            axis=cfg.approach_axis,
            sign=cfg.approach_sign,
            frame=cfg.approach_offset_frame,
        )
        servo_distance(
            robot,
            axis=cfg.approach_axis,
            distance_m=-cfg.approach_sign * cfg.retract_m,
            speed_mps=max(cfg.contact.approach_speed_mps, 0.001),
            frame=cfg.approach_offset_frame,
        )

        return SkillResult(
            name=target.name,
            success=True,
            details={"contact_detected": contact_detected, "kind": target.kind},
        )
