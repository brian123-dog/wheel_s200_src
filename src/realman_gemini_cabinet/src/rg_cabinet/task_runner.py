from __future__ import annotations

import argparse
import copy
from dataclasses import fields, is_dataclass, replace
from pathlib import Path
from typing import Any, Callable

from rg_cabinet.adapters.mock import MockRobotAdapter
from rg_cabinet.adapters.robot import RobotAdapter
from rg_cabinet.config import load_task_config
from rg_cabinet.geometry import Pose
from rg_cabinet.perception.element_detector import ElementTarget
from rg_cabinet.skills import (
    ButtonPressConfig,
    ButtonPressSkill,
    ContactMotionConfig,
    DoorHandleConfig,
    DoorHandleSkill,
    KnobCameraServoConfig,
    KnobCameraServoSkill,
    KnobTurnConfig,
    KnobTurnSkill,
    MoveJointConfig,
    MoveJointSkill,
    ReadMeterConfig,
    ReadMeterSkill,
    SkillContext,
    ToggleSwitchConfig,
    ToggleSwitchSkill,
)
from rg_cabinet.verification import (
    MockStateVerifier,
    StateVerifier,
    VerificationRequest,
)


SKILL_TYPES = {
    "button": (ButtonPressSkill, ButtonPressConfig),
    "toggle": (ToggleSwitchSkill, ToggleSwitchConfig),
    "slider": (ToggleSwitchSkill, ToggleSwitchConfig),
    "knob": (KnobTurnSkill, KnobTurnConfig),
    "knob_camera_servo": (KnobCameraServoSkill, KnobCameraServoConfig),
    "door": (DoorHandleSkill, DoorHandleConfig),
    "door_handle": (DoorHandleSkill, DoorHandleConfig),
    "read_meter": (ReadMeterSkill, ReadMeterConfig),
    "move_joint": (MoveJointSkill, MoveJointConfig),
}

# Skills that never resolve a vision target and never need step["target"].
# read_meter builds a placeholder target internally; move_joint is pure
# joint-space motion and would otherwise fail in _build_target on a missing
# target.pose.
NO_VISION_TARGET_SKILLS = frozenset({"read_meter", "move_joint"})


def _relax_step_for_near(step: dict[str, Any]) -> dict[str, Any]:
    """返回 step 的深拷贝，清除 max 上限和 ROI 约束（用于近距二次检测）"""
    relaxed = copy.deepcopy(step)
    target: dict[str, Any] = relaxed.get("target", {})
    # 所有需要清除的 key 必须显式设为 None，不能 pop ——
    # _merged_detection_config 从 vision_cfg["targets"][detection_name] 初始化，
    # 然后只在 target_cfg 中存在对应 key 时才覆盖。
    # 如果用 pop 删掉 key，named target 里的旧值不会被覆盖。
    relax_keys = (
        "max_pixels",
        "max_bbox_width_px",
        "max_bbox_height_px",
        "plate_max_area_px",
        "plate_max_width_px",
        "plate_max_height_px",
        "frame_max_size_px",
        "roi",
        "min_roi_margin_px",
        # 走近后深度变小，必须放宽 min/max_depth_m，
        # 否则 _validate_depth_m 会因距离太近而拒绝检测结果。
        "min_depth_m",
        "max_depth_m",
        # 走近后目标像素位置改变，清除 desired_pixel 避免候选排名
        # 因 center_distance_px 偏向错误候选（正确的 plate 离原
        # desired_pixel 可能很远，而错误候选刚好更近）。
        "desired_pixel",
        "servo_desired_pixel",
    )
    for key in relax_keys:
        target[key] = None
    dc = target.get("detection_config")
    if isinstance(dc, dict):
        for key in relax_keys:
            dc[key] = None
    return relaxed


def _step_for_refresh(step, context):
    if context.precontact_centering:
        # Preserve every detection/depth/workspace gate. Near checks save actual
        # images so a residual calibration bias can be diagnosed afterwards.
        near = copy.deepcopy(step)
        near.setdefault("target", {})["save_debug_images"] = True
        near_policy = near["target"].pop("precontact_fresh_sampling", None)
        if near_policy is not None:
            if not isinstance(near_policy, dict):
                raise ValueError('precontact_fresh_sampling must be a mapping')
            near["target"]["fresh_sampling"] = near_policy
        templates = step.get("params", {}).get("precontact_panel_layout_templates", ())
        if templates:
            near["target"]["panel_layout_templates"] = list(templates)
            near["target"]["panel_layout_template"] = templates[0]
        return near
    return _relax_step_for_near(step) if context.relax_vision else step


def run_sequence(
    config: dict[str, Any],
    robot: RobotAdapter,
    verifier: StateVerifier | None = None,
    target_resolver: Callable[[dict[str, Any]], ElementTarget | None] | None = None,
    before_step: Callable[[dict[str, Any]], None] | None = None,
    camera_cache: Any | None = None,
) -> list[dict[str, Any]]:
    default_contact = _build_contact(config.get("default_contact", {}))
    results: list[dict[str, Any]] = []

    for step in config.get("sequence", []):
        if before_step is not None:
            before_step(step)
        skill_name = str(step["skill"])
        skill_cls, config_cls = SKILL_TYPES[skill_name]
        skill_config = _build_skill_config(
            config_cls,
            step.get("params", {}),
            default_contact,
        )
        target = (
            target_resolver(step)
            if target_resolver is not None and skill_name not in NO_VISION_TARGET_SKILLS
            else None
        )
        if target is None:
            target = _build_fallback_target(step, skill_name)
        context = SkillContext(robot=robot, camera_cache=camera_cache)
        if target_resolver is not None and skill_name not in NO_VISION_TARGET_SKILLS:
            target_refresher: Callable[[], Any] | None = (
                lambda step=step, ctx=context: target_resolver(
                    _step_for_refresh(step, ctx)
                )
            )
        else:
            target_refresher = None
        context.target_refresher = target_refresher
        result = skill_cls(skill_config).execute(context, target)
        details = dict(result.details)
        success = result.success
        if step.get("verify"):
            verification = _verify_step(step["verify"], verifier)
            details["verification"] = verification
            success = success and verification.get("passed", False)
        results.append(
            {
                "name": result.name,
                "skill": skill_name,
                "success": success,
                "details": details,
            }
        )

    return results


def _build_target(step: dict[str, Any]) -> ElementTarget:
    target_cfg = step["target"]
    frame = str(target_cfg.get("frame", "base"))
    pose = Pose.from_sequence(target_cfg["pose"], frame=frame)
    return ElementTarget(
        name=str(step.get("name", target_cfg.get("name", "target"))),
        kind=str(target_cfg.get("kind", step["skill"])),
        pose=pose,
        confidence=float(target_cfg.get("confidence", 1.0)),
    )


def _build_meter_target(step: dict[str, Any]) -> ElementTarget:
    return ElementTarget(
        name=str(step.get("name", "panel_meter")),
        kind=str(step.get("kind", "meter_reading")),
        pose=Pose(0.0, 0.0, 0.0),
        confidence=1.0,
    )


def _build_joint_target(step: dict[str, Any]) -> ElementTarget:
    """Placeholder target for joint-space steps that carry no cartesian pose."""
    return ElementTarget(
        name=str(step.get("name", "move_joint")),
        kind=str(step.get("kind", "joint_pose")),
        pose=Pose(0.0, 0.0, 0.0),
        confidence=1.0,
    )


def _build_fallback_target(step: dict[str, Any], skill_name: str) -> ElementTarget:
    if skill_name == "read_meter":
        return _build_meter_target(step)
    if skill_name == "move_joint":
        return _build_joint_target(step)
    return _build_target(step)


def _build_contact(values: dict[str, Any]) -> ContactMotionConfig:
    return _dataclass_from_mapping(ContactMotionConfig, values)


def _build_skill_config(
    config_cls,
    values: dict[str, Any],
    default_contact: ContactMotionConfig,
):
    prepared = dict(values)
    contact_values = prepared.pop("contact", None)
    config = _dataclass_from_mapping(config_cls, prepared)
    field_names = {field.name for field in fields(config_cls)}
    if "contact" not in field_names:
        if contact_values is not None:
            raise ValueError(f"{config_cls.__name__} does not support contact config")
        return config
    if contact_values is None:
        return replace(config, contact=default_contact)
    return replace(config, contact=_build_contact(contact_values))


def _verify_step(
    verify_cfg: dict[str, Any],
    verifier: StateVerifier | None,
) -> dict[str, Any]:
    request = VerificationRequest(
        sensor=str(verify_cfg["sensor"]),
        expected=verify_cfg.get("expected"),
        timeout_s=float(verify_cfg.get("timeout_s", 1.0)),
    )
    if verifier is None:
        return {
            "sensor": request.sensor,
            "expected": request.expected,
            "observed": None,
            "passed": False,
            "reason": "no_verifier",
        }
    result = verifier.verify(request)
    return {
        "sensor": result.sensor,
        "expected": result.expected,
        "observed": result.observed,
        "passed": result.passed,
    }


def _dataclass_from_mapping(cls, values: dict[str, Any]):
    if not is_dataclass(cls):
        raise TypeError(f"{cls} is not a dataclass")
    allowed = {field.name for field in fields(cls)}
    unknown = set(values) - allowed
    if unknown:
        raise ValueError(f"Unknown fields for {cls.__name__}: {sorted(unknown)}")
    return cls(**{key: value for key, value in values.items() if key in allowed})


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--config",
        default=str(Path(__file__).resolve().parents[2] / "config/cabinet_demo.yaml"),
    )
    parser.add_argument(
        "--mock",
        action="store_true",
        help="Run with MockRobotAdapter. Real hardware clients should use run_sequence().",
    )
    args = parser.parse_args(argv)

    if not args.mock:
        raise SystemExit("Only --mock is wired in the CLI until a RealMan client is passed")

    config = load_task_config(args.config)
    robot = MockRobotAdapter(contact_after_servo_calls=2)
    verifier = MockStateVerifier(config.get("mock_states", {}))
    results = run_sequence(config, robot, verifier=verifier)
    for item in results:
        print(f"{item['name']}: {item['skill']} success={item['success']} {item['details']}")
    print(f"recorded_commands={len(robot.commands)}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
