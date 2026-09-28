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
    servo_rotation,
)


@dataclass(frozen=True)
class DoorHandleConfig:
    pre_offset_m: float = 0.05
    contact_start_offset_m: float = 0.01
    retract_m: float = 0.05
    approach_axis: str = "z"
    approach_sign: float = -1.0
    approach_offset_frame: str = "base"
    unlatch_axis: str = "z"
    unlatch_angle_rad: float = 0.0
    unlatch_speed_radps: float = 0.35
    pull_distance_m: float = 0.18
    lift_distance_m: float = 0.03
    steps: int = 8
    move_speed_mps: float = 0.04
    gripper_open: float = 0.45
    gripper_closed: float = 1.0
    contact: ContactMotionConfig = ContactMotionConfig()


class DoorHandleSkill:
    def __init__(self, config: DoorHandleConfig | None = None) -> None:
        self.config = config or DoorHandleConfig()

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
        )
        robot.set_gripper(cfg.gripper_closed)

        if abs(cfg.unlatch_angle_rad) > 1e-6:
            servo_rotation(
                robot,
                cfg.unlatch_axis,
                cfg.unlatch_angle_rad,
                cfg.unlatch_speed_radps,
            )

        start_pose = robot.get_tcp_pose()
        step_count = max(1, cfg.steps)
        for idx in range(1, step_count + 1):
            ratio = idx / step_count
            robot.move_pose(
                start_pose.translated(
                    dx=cfg.pull_distance_m * ratio,
                    dz=cfg.lift_distance_m * ratio,
                ),
                speed=cfg.move_speed_mps,
            )

        robot.set_gripper(cfg.gripper_open)
        servo_distance(
            robot,
            axis=cfg.approach_axis,
            distance_m=-cfg.approach_sign * cfg.retract_m,
            speed_mps=max(cfg.contact.approach_speed_mps, 0.001),
        )
        return SkillResult(
            name=target.name,
            success=True,
            details={
                "contact_detected": contact_detected,
                "pull_distance_m": cfg.pull_distance_m,
                "steps": step_count,
            },
        )
