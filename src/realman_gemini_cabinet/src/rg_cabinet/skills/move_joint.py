from __future__ import annotations

import time
from dataclasses import dataclass
from typing import Sequence

from rg_cabinet.perception.element_detector import ElementTarget
from rg_cabinet.skills.base import SkillContext, SkillResult


@dataclass(frozen=True)
class MoveJointConfig:
    """Absolute joint-space move, used for post-sequence safe poses.

    ``joint_positions`` are absolute joint angles in radians, in the same order
    as the controller joint vector (J1..Jn). No cartesian target is used, so a
    joint step does not need ``target.pose`` in the task config.
    """

    joint_positions: Sequence[float] = ()
    speed_mps: float = 0.05
    wait: bool = True
    settle_s: float = 0.0
    gripper_position: float | None = None


class MoveJointSkill:
    def __init__(self, config: MoveJointConfig | None = None) -> None:
        self.config = config or MoveJointConfig()

    def execute(self, context: SkillContext, target: ElementTarget) -> SkillResult:
        robot = context.robot
        cfg = self.config
        joints = [float(value) for value in cfg.joint_positions]
        if not joints:
            raise RuntimeError(
                "move_joint requires params.joint_positions "
                "(absolute joint angles in radians)"
            )
        if cfg.gripper_position is not None:
            robot.set_gripper(float(cfg.gripper_position))
        robot.move_joint(joints, speed=cfg.speed_mps, wait=cfg.wait)
        if cfg.settle_s > 0.0:
            time.sleep(float(cfg.settle_s))
        return SkillResult(
            name=target.name,
            success=True,
            details={
                "kind": target.kind,
                "joint_positions": joints,
                "speed_mps": float(cfg.speed_mps),
                "settle_s": float(cfg.settle_s),
                "gripper_position": cfg.gripper_position,
            },
        )
