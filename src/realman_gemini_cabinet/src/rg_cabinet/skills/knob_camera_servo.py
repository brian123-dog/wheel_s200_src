
from __future__ import annotations

import math
import time
from dataclasses import dataclass
from typing import Sequence

from rg_cabinet.geometry import Pose
from rg_cabinet.perception.element_detector import ElementTarget
from rg_cabinet.skills.base import (
    ContactMotionConfig,
    SkillContext,
    SkillResult,
    guarded_approach,
    servo_distance,
    servo_rotation,
)


@dataclass(frozen=True)
class KnobCameraServoConfig:
    # ── Y/Z 对准参数 ──
    align_axes: Sequence[str] = ("y", "z")
    align_speed_mps: float = 0.01
    min_align_m: float = 0.001
    max_align_axis_m: float = 0.15
    max_align_total_m: float = 0.22
    require_pixel_error: bool = True

    # Optional kinematic preflight of alignment MoveL segments only.
    alignment_preflight: bool = False
    alignment_preflight_sdk_library: str = ""
    alignment_preflight_step_m: float = 0.001
    alignment_preflight_min_margin_deg: float = 5.0
    alignment_preflight_max_joint_step_deg: float = 5.0
    alignment_reroute: bool = False
    # Explicitly configured free-space allowance below the alignment start.
    alignment_reroute_drop_clearance_m: float = 0.0

    # ── 中位数采样参数（替代旧迭代参数） ──
    median_samples: int = 3
    detection_interval_s: float = 0.3

    # ── 中途二次检测 ──
    intermediate_check_distance_m: float = 0.25

    # Optional near-target check after fast approach, before any slow advance.
    precontact_centering: bool = False
    precontact_panel_layout_templates: Sequence[str] = ()
    precontact_centering_tolerance_m: float = 0.002
    precontact_centering_max_axis_m: float = 0.015
    # None preserves the original reject-above-command-limit behavior.
    precontact_centering_max_error_m: float | None = None
    precontact_centering_max_total_m: float = 0.020
    precontact_centering_max_corrections: int = 2
    precontact_centering_speed_mps: float = 0.005
    precontact_centering_settle_s: float = 0.3

    # ── 观察位姿态调整 ──
    actuation_rpy_base: Sequence[float] | None = (0.0, -1.5708, 0.0)
    orientation_speed_mps: float = 0.02

    # ── 进给（X轴）参数 ──
    approach_axis: str = "x"
    approach_sign: float = -1.0
    approach_frame: str = "base"
    approach_distance_m: float = 0.03
    approach_speed_mps: float = 0.05
    retract_m: float = 0.40

    # ── 夹爪 ──
    gripper_open: float = 0.4
    gripper_closed: float = 1.0
    gripper_settle_s: float = 1.5

    # ── 旋转 ──
    turn_axis: str = "z"
    turn_frame: str = "tool"
    turn_angle_rad: float = 1.5708
    angular_speed_radps: float = 0.25
    release_after_turn: bool = True

    # ── 接触/开环进给 ──
    require_contact: bool = False
    contact_mode: str = "sensor"
    open_loop_advance_m: float = 0.0
    open_loop_advance_speed_mps: float = 0.004
    contact: ContactMotionConfig = ContactMotionConfig()


class KnobCameraServoSkill:
    def __init__(self, config: KnobCameraServoConfig | None = None) -> None:
        self.config = config or KnobCameraServoConfig()

    def execute(self, context: SkillContext, target: ElementTarget) -> SkillResult:
        robot = context.robot
        cfg = self.config
        contact_mode = self._contact_mode()
        if cfg.precontact_centering:
            from .precontact_centering import validate_precontact_centering
            validate_precontact_centering(cfg, context)
        stages = _stage_status()
        precontact_debug = None
        initial_align_delta = self._alignment_delta(target)
        self._validate_alignment_delta(initial_align_delta)

        tcp_pose = robot.get_tcp_pose()
        _report_stage(robot, "open_gripper")
        robot.set_gripper(cfg.gripper_open)
        _report_stage(robot, "orientation")
        self._move_to_actuation_orientation(robot, tcp_pose)
        stages["orientation"] = True
        _report_stage(robot, "alignment")
        target, align_delta, alignment_debug, alignment_stop_reason = self._align_to_target(
            context,
            target,
            initial_align_delta,
        )
        stages["visual_alignment"] = True

        _report_stage(robot, "approach")
        # ---- intermediate check: 在剩余 intermediate_check_distance_m 处二次检测 ----
        tcp_pose = robot.get_tcp_pose()
        total_x_delta = target.pose.x - tcp_pose.x

        if (
            cfg.intermediate_check_distance_m > 0.0
            and abs(total_x_delta) > cfg.intermediate_check_distance_m + cfg.approach_distance_m
        ):
            # Phase 1: 走到距目标 intermediate_check_distance_m 处
            phase1_distance = total_x_delta - cfg.approach_sign * cfg.intermediate_check_distance_m
            # 标记当前相机帧时间戳，用于逼近后等待新帧
            if context.camera_cache is not None and not target.details.get("fresh_sampling", {}).get("enabled", False):
                context.camera_cache.mark_fresh()
            servo_distance(
                robot,
                axis=cfg.approach_axis,
                distance_m=phase1_distance,
                speed_mps=cfg.approach_speed_mps,
                frame=cfg.approach_frame,
            )
            if cfg.detection_interval_s > 0.0:
                time.sleep(float(cfg.detection_interval_s))
            # 等待相机新帧（深度传感器需要时间适应近距离）
            if context.camera_cache is not None and not target.details.get("fresh_sampling", {}).get("enabled", False):
                if not context.camera_cache.wait_fresh(timeout_s=3.0):
                    raise RuntimeError(
                        "knob_camera_servo intermediate check: "
                        "camera frames not refreshed after approach"
                    )

            # Phase 2: 二次全量检测（放宽 max 上限和 ROI）
            target_refresher = context.target_refresher
            if target_refresher is None:
                raise RuntimeError(
                    "knob_camera_servo intermediate check requires target_refresher"
                )
            _report_stage(robot, "recheck")
            context.relax_vision = True
            refreshed = target_refresher()
            context.relax_vision = False
            if refreshed is None:
                raise RuntimeError(
                    "knob_camera_servo intermediate check: second detection failed, no target found"
                )
            target = refreshed

            # 快速 Y/Z 对齐（单次，基于二次检测结果）
            second_align_delta = self._alignment_delta(target)
            self._validate_alignment_delta(second_align_delta)
            _report_stage(robot, "alignment")
            self._servo_alignment(robot, second_align_delta, min_distance_m=0.0)

            # Phase 3: 二次检测后夹爪已贴近旋钮，跳过 X 进给，直接进入接触/夹爪闭合流程
            approach_distance = 0.0
        else:
            # 根据相机测得深度减去相机到指尖偏移，计算进给距离
            # depth_m 已在对齐阶段由 vision_target_resolver 写入 target.details
            measured_depth_m = float(target.details.get("depth_m", 0.0))
            tcp_in_camera = target.details.get("tcp_center_camera_m")
            if tcp_in_camera is not None and len(tcp_in_camera) == 3:
                camera_to_tcp_optical = abs(float(tcp_in_camera[2]))
            else:
                camera_to_tcp_optical = 0.0
            approach_distance = max(0.0, measured_depth_m - camera_to_tcp_optical - 0.12)
            # camera_to_tcp_optical 从 TF 实时计算：tcp_center_camera_m[2]

        if approach_distance > 0.0:
            servo_distance(
                robot,
                axis=cfg.approach_axis,
                distance_m=cfg.approach_sign * approach_distance,
                speed_mps=cfg.approach_speed_mps,
                frame=cfg.approach_frame,
            )
            stages["approach_distance"] = True
        if cfg.precontact_centering:
            from .precontact_centering import center_before_contact
            target, precontact_debug = center_before_contact(self, context, target)
            stages["precontact_centering"] = True
        _report_stage(robot, "contact")
        contact_detected: bool | None
        if contact_mode == "open_loop":
            contact_detected = None
            if cfg.open_loop_advance_m > 0.0:
                servo_distance(
                    robot,
                    axis=cfg.approach_axis,
                    distance_m=cfg.approach_sign * cfg.open_loop_advance_m,
                    speed_mps=cfg.open_loop_advance_speed_mps,
                    frame=cfg.approach_frame,
                )
                stages["open_loop_advance"] = True
        else:
            contact_detected = guarded_approach(
                robot,
                cfg.contact,
                axis=cfg.approach_axis,
                sign=cfg.approach_sign,
                frame=cfg.approach_frame,
            )
            if cfg.require_contact and not contact_detected:
                if cfg.retract_m > 0.0:
                    servo_distance(
                        robot,
                        axis=cfg.approach_axis,
                        distance_m=-cfg.approach_sign * cfg.retract_m,
                        speed_mps=self._retract_speed_mps(contact_mode),
                        frame=cfg.approach_frame,
                    )
                raise RuntimeError("Knob camera servo did not detect contact")

        _report_stage(robot, "grip")
        robot.set_gripper(cfg.gripper_closed)
        time.sleep(cfg.gripper_settle_s)
        _report_stage(robot, "turn")
        servo_rotation(
            robot,
            axis=cfg.turn_axis,
            angle_rad=cfg.turn_angle_rad,
            speed_radps=cfg.angular_speed_radps,
            frame=cfg.turn_frame,
        )
        stages["turn"] = True
        if cfg.release_after_turn:
            _report_stage(robot, "release")
            robot.set_gripper(cfg.gripper_open)
        if cfg.retract_m > 0.0:
            _report_stage(robot, "retract")
            servo_distance(
                robot,
                axis=cfg.approach_axis,
                distance_m=-cfg.approach_sign * cfg.retract_m,
                speed_mps=self._retract_speed_mps(contact_mode),
                frame=cfg.approach_frame,
            )
            stages["retract"] = True

        _report_stage(robot, "done")
        return SkillResult(
            name=target.name,
            success=True,
            details={
                "stages": stages,
                **({"precontact_centering": precontact_debug} if precontact_debug is not None else {}),
                "contact_detected": contact_detected,
                "contact_mode": contact_mode,
                "planned_approach_m": self._planned_approach_m(contact_mode),
                "alignment_stop_reason": alignment_stop_reason,
                "initial_align_delta_base_m": initial_align_delta,
                "final_align_delta_base_m": align_delta,
                "alignment_error_source": self._alignment_delta_source(target),
                "alignment_debug": alignment_debug,
                "final_pixel": target.details.get("pixel"),
                "final_desired_pixel": target.details.get("desired_pixel"),
                "final_alignment_reference_pixel": target.details.get(
                    "alignment_reference_pixel"
                ),
                "final_pixel_error_base_m": target.details.get("pixel_error_base_m"),
                "final_alignment_error_base_m": target.details.get(
                    "alignment_error_base_m"
                ),
                "turn_angle_rad": cfg.turn_angle_rad,
                "pixel": target.details.get("pixel"),
                "bbox": target.details.get("bbox"),
                "pixel_count": target.details.get("pixel_count"),
            },
        )

    def _contact_mode(self) -> str:
        mode = str(self.config.contact_mode)
        if mode not in ("sensor", "open_loop"):
            raise RuntimeError(f"Unsupported knob_camera_servo contact_mode: {mode}")
        return mode

    def _planned_approach_m(self, contact_mode: str) -> float:
        planned = max(0.0, float(self.config.approach_distance_m))
        if contact_mode == "open_loop":
            planned += max(0.0, float(self.config.open_loop_advance_m))
        else:
            planned += max(0.0, float(self.config.contact.max_contact_travel_m))
        return planned

    def _retract_speed_mps(self, contact_mode: str) -> float:
        if contact_mode == "open_loop":
            return max(float(self.config.approach_speed_mps), 0.001)
        return max(float(self.config.contact.approach_speed_mps), 0.001)

    def _align_to_target(
        self,
        context: SkillContext,
        target: ElementTarget,
        initial_align_delta: dict[str, float],
    ) -> tuple[ElementTarget, dict[str, float], list[dict[str, object]], str]:
        """采集 N 次独立检测，逐轴取中位数，一次性运动到位。"""
        robot = context.robot
        target_refresher = context.target_refresher
        cfg = self.config

        # 没有 refresher 时单次走
        if target_refresher is None:
            self._servo_alignment(robot, initial_align_delta)
            return target, initial_align_delta, [], "single_pass_no_refresher"

        # 收集样本
        # One trusted far-view sample is allowed for the right-knob mode;
        # every other configured sample count keeps its previous behavior.
        samples = 1 if int(cfg.median_samples) == 1 else max(2, int(cfg.median_samples))
        all_deltas: list[dict[str, float]] = [initial_align_delta]
        last_target = target

        for i in range(samples - 1):
            # 等待新相机帧
            if context.camera_cache is not None and not target.details.get("fresh_sampling", {}).get("enabled", False):
                context.camera_cache.mark_fresh()
                if not context.camera_cache.wait_fresh(timeout_s=2.0):
                    raise RuntimeError(
                        "knob_camera_servo median alignment: "
                        "camera frames not refreshed"
                    )

            refreshed = target_refresher()
            if refreshed is None:
                raise RuntimeError(
                    "knob_camera_servo median alignment: "
                    "target refresh returned no target"
                )
            new_delta = self._alignment_delta(refreshed)
            self._validate_alignment_delta(new_delta)
            all_deltas.append(new_delta)
            last_target = refreshed

        # 逐轴取中位数
        median_delta: dict[str, float] = {"x": 0.0}
        for axis in ("y", "z"):
            col = sorted(d[axis] for d in all_deltas)
            median_delta[axis] = col[len(col) // 2]

        self._validate_alignment_delta(median_delta)

        # 调试信息
        debug_info = [
            {
                "method": "median",
                "samples": samples,
                "all_deltas_base_m": {
                    axis: [round(d[axis], 6) for d in all_deltas]
                    for axis in ("y", "z")
                },
                "median_delta_base_m": {
                    axis: round(median_delta[axis], 6)
                    for axis in ("y", "z")
                },
            }
        ]

        # 一次性运动到位
        self._servo_alignment(robot, median_delta)

        return last_target, median_delta, debug_info, "median_alignment_done"

    def _alignment_delta(self, target: ElementTarget) -> dict[str, float]:
        source = self._alignment_delta_source(target)
        raw = target.details.get(source) if source is not None else None
        if raw is None:
            if self.config.require_pixel_error:
                raise RuntimeError(
                    "knob_camera_servo requires target.details.alignment_error_base_m "
                    "or target.details.pixel_error_base_m"
                )
            return {"x": 0.0, "y": 0.0, "z": 0.0}
        values = [float(value) for value in raw]
        if len(values) != 3:
            raise RuntimeError(f"{source} must contain [x, y, z]")
        enabled = {str(axis) for axis in self.config.align_axes}
        return {
            "x": values[0] if "x" in enabled else 0.0,
            "y": values[1] if "y" in enabled else 0.0,
            "z": values[2] if "z" in enabled else 0.0,
        }

    def _alignment_delta_source(self, target: ElementTarget) -> str | None:
        if target.details.get("alignment_error_base_m") is not None:
            return "alignment_error_base_m"
        if target.details.get("pixel_error_base_m") is not None:
            return "pixel_error_base_m"
        return None

    def _validate_alignment_delta(self, align_delta: dict[str, float]) -> None:
        for axis, distance_m in align_delta.items():
            if abs(distance_m) > self.config.max_align_axis_m:
                raise RuntimeError(
                    f"Knob camera servo {axis} alignment {distance_m:.3f}m exceeds "
                    f"max_align_axis_m {self.config.max_align_axis_m:.3f}m"
                )
        total = math.sqrt(sum(distance_m * distance_m for distance_m in align_delta.values()))
        if total > self.config.max_align_total_m:
            raise RuntimeError(
                f"Knob camera servo alignment {total:.3f}m exceeds "
                f"max_align_total_m {self.config.max_align_total_m:.3f}m"
            )

    def _move_to_actuation_orientation(self, robot, tcp_pose: Pose) -> None:
        rpy = self.config.actuation_rpy_base
        if rpy is None:
            return
        values = [float(value) for value in rpy]
        if len(values) != 3:
            raise RuntimeError("actuation_rpy_base must contain [roll, pitch, yaw]")
        robot.move_pose(
            Pose(
                tcp_pose.x,
                tcp_pose.y,
                tcp_pose.z,
                values[0],
                values[1],
                values[2],
                frame=tcp_pose.frame,
            ),
            speed=self.config.orientation_speed_mps,
        )



    def _servo_alignment(
        self,
        robot,
        align_delta: dict[str, float],
        min_distance_m: float | None = None,
    ) -> None:
        min_distance = self.config.min_align_m if min_distance_m is None else min_distance_m
        if self.config.alignment_reroute:
            moves = [(axis, align_delta[axis]) for axis in ("x", "y", "z")
                     if abs(align_delta[axis]) > min_distance]
            if moves:
                executor = getattr(robot, "execute_movel_alignment_route", None)
                if not callable(executor):
                    raise RuntimeError("Selected backend does not support MoveL alignment rerouting")
                executor(moves, self.config)
            return
        if self.config.alignment_preflight:
            moves = [(axis, align_delta[axis]) for axis in ("x", "y", "z")
                     if abs(align_delta[axis]) > min_distance]
            if moves:
                checker = getattr(robot, "preflight_movel_alignment", None)
                if not callable(checker):
                    raise RuntimeError("Selected backend does not support MoveL alignment preflight")
                # Reject an impossible later axis before moving the first axis.
                checker(moves, self.config)
        for axis in ("x", "y", "z"):
            distance_m = align_delta[axis]
            if abs(distance_m) <= min_distance:
                continue
            if self.config.alignment_preflight:
                # Recheck from actual feedback after the preceding axis completes.
                checker([(axis, distance_m)], self.config)
            servo_distance(
                robot,
                axis=axis,
                distance_m=distance_m,
                speed_mps=self.config.align_speed_mps,
                frame="base",
            )


def _clamp(value: float, lower: float, upper: float) -> float:
    return min(upper, max(lower, float(value)))


def _pair(raw: object) -> tuple[float, float] | None:
    if raw is None:
        return None
    try:
        values = [float(value) for value in raw]  # type: ignore[operator]
    except (TypeError, ValueError):
        return None
    if len(values) != 2:
        return None
    return values[0], values[1]


def _round_sequence(raw: object) -> list[float] | None:
    pair = _pair(raw)
    if pair is None:
        return None
    return [round(pair[0], 3), round(pair[1], 3)]


def _round_mapping(values: dict[str, float]) -> dict[str, float]:
    return {axis: round(float(value), 6) for axis, value in values.items()}


def _stage_status() -> dict[str, bool]:
    return {
        "orientation": False,
        "visual_alignment": False,
        "approach_distance": False,
        "open_loop_advance": False,
        "turn": False,
        "retract": False,
    }


def _report_stage(robot, stage: str) -> None:
    """Optional reporting hook; does not alter motion commands or their order."""
    report = getattr(robot, "report_task_stage", None)
    if callable(report):
        report(stage)
