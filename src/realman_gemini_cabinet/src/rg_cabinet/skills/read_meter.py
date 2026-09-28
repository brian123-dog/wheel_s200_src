from __future__ import annotations

from dataclasses import dataclass

from rg_cabinet.perception.element_detector import ElementTarget
from rg_cabinet.skills.base import SkillContext, SkillResult


@dataclass(frozen=True)
class ReadMeterConfig:
    save_dir: str = "meter_results"
    image_prefix: str = "panel_meter"


class ReadMeterSkill:
    def __init__(self, config: ReadMeterConfig | None = None) -> None:
        self.config = config or ReadMeterConfig()

    def execute(self, context: SkillContext, target: ElementTarget) -> SkillResult:
        del context
        return SkillResult(
            name=target.name,
            success=True,
            details={
                "kind": target.kind,
                "non_contact": True,
                "image_prefix": self.config.image_prefix,
            },
        )
