from __future__ import annotations

import time
from dataclasses import dataclass

from rg_cabinet.perception.element_detector import ElementTarget
from rg_cabinet.skills.base import (
    ContactMotionConfig,
    SkillContext,
    SkillResult,
    guarded_approach,
    offset_pose,
    servo_distance,
    servo_rotation,
)


@dataclass(frozen=True)
class KnobTurnConfig:
    pre_offset_m: float = 0.045
    contact_start_offset_m: float = 0.008
    retract_m: float = 0.35
    approach_axis: str = "z"
    approach_sign: float = -1.0
    approach_offset_frame: str = "base"
    turn_axis: str = "z"
    turn_angle_rad: float = -0.78539816339
    angular_speed_radps: float = 0.35
    move_speed_mps: float = 0.04
    gripper_open: float = 0.4
    gripper_closed: float = 1.0
    gripper_settle_s: float = 1.5
    contact: ContactMotionConfig = ContactMotionConfig()


class KnobTurnSkill:
    def __init__(self, config: KnobTurnConfig | None = None) -> None:
        self.config = config or KnobTurnConfig()

    def execute(self, context: SkillContext, target: ElementTarget) -> SkillResult:
        robot = context.robot
        cfg = self.config
        robot.set_gripper(cfg.gripper_open)
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
        robot.set_gripper(cfg.gripper_closed)
        time.sleep(cfg.gripper_settle_s)
        servo_rotation(
            robot,
            axis=cfg.turn_axis,
            angle_rad=cfg.turn_angle_rad,
            speed_radps=cfg.angular_speed_radps,
        )
        robot.set_gripper(cfg.gripper_open)
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
            details={
                "contact_detected": contact_detected,
                "turn_angle_rad": cfg.turn_angle_rad,
            },
        )
