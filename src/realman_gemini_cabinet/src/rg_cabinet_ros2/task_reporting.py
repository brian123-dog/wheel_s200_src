"""Human-readable task output; full payloads remain available for diagnosis."""
from __future__ import annotations

import json
import math
from datetime import datetime
from pathlib import Path
from uuid import uuid4


STAGES = {
    "prepare": "准备任务", "observation": "前往观察位", "vision": "识别目标",
    "open_gripper": "张开夹爪", "orientation": "调整操作姿态",
    "alignment": "视觉对齐/路径预算", "approach": "接近旋钮",
    "precontact_centering": "接触前居中",
    "recheck": "近距离复检", "contact": "接触旋钮", "grip": "夹紧旋钮",
    "turn": "旋转旋钮", "release": "松开夹爪", "retract": "撤退",
    "done": "动作完成",
}


def short(value, limit=200):
    text = " ".join(str(value).split())
    return text if len(text) <= limit else text[:limit] + "…"


def number(value, scale=1., digits=3):
    try:
        value = float(value) * scale
        return f"{value:.{digits}f}" if math.isfinite(value) else "未知"
    except (TypeError, ValueError, OverflowError):
        return "未知"


def status_summary(payload):
    event = str(payload.get("event", "任务状态"))
    step = payload.get("step") or payload.get("active_step", {}).get("name")
    stage = payload.get("stage")
    context = " / ".join(short(v, 70) for v in (step, STAGES.get(stage, stage)) if v)
    prefix = f"[柜体任务] {context} | " if context else "[柜体任务] "
    motion = payload.get("motion_command_debug") or {}
    if event == "task_stage":
        return prefix + ("完成" if stage == "done" else "开始")
    if event == "alignment_preflight":
        if payload.get("status") != "PASSED":
            return prefix + "路径预算未通过 | " + short(payload.get("reason", "原因未知"))
        selected = payload.get("selected") or {}
        moves = selected.get("moves", [])
        route = " → ".join(str(axis).upper() + " " + number(distance, 1000, 1) + " mm"
                           for axis, distance in moves)
        margin = selected.get("minimum_margin_deg", payload.get("minimum_margin_deg"))
        return prefix + "路径预算通过" + (" | " + route if route else "") + " | 最小关节余量=" + number(margin, digits=1) + "°"
    if event == "node_ready":
        return prefix + "已就绪 | 模式=" + Path(str(payload.get("task_config", "未知"))).name
    failed = payload.get("success") is False or motion.get("result") in (False, "timeout")
    if failed:
        error = str(payload.get("error") or "")
        if "rm_driver reported" in error or (not error and motion.get("result") is False):
            reason = "驱动返回运动失败；具体原因需核对控制器报警"
        elif "Missing or ambiguous central hub cap" in error:
            reason = "旋钮轴心缺失或存在多个候选"
        elif "outside workspace" in error:
            reason = "目标坐标超出允许工作区：" + short(error, 140)
        elif "timeout" in error.lower() or motion.get("result") == "timeout":
            reason = "等待超时：" + short(error, 140)
        else:
            reason = short(error or "任务结果未通过验证")
        parts = [prefix + "失败", reason]
        # A previous successful move is not the cause of a later vision error.
        if motion.get("result") in (False, "timeout"):
            parts.append("指令=" + short(motion.get("result_name", motion.get("command", "未知")), 40))
            if "elapsed_s" in motion:
                parts.append("耗时=" + number(motion["elapsed_s"]) + " s")
            if "distance_m" in motion:
                parts.append("平移=" + number(motion["distance_m"], 1000) + " mm")
        preflight = payload.get("alignment_preflight") or {}
        if preflight.get("status") == "REJECTED":
            parts.append("预算=" + short(preflight.get("reason", "未通过"), 100))
        if payload.get("diagnostic_write_error"):
            parts.append("诊断文件保存失败；完整数据见 /rg_cabinet/status")
        return " | ".join(parts)
    if event in ("sequence_complete", "qr_sequence_complete"):
        results = payload.get("results", [])
        return prefix + f"完成 | 步骤 {sum(bool(r.get('success')) for r in results)}/{len(results)} 成功"
    labels = {"motion_command_completed": "运动完成", "motion_command_sent": "运动指令已发送",
              "targets_resolved": "目标识别完成", "meter_image_saved": "仪表图像已保存",
              "manual_gripper_set": "夹爪指令完成"}
    return prefix + labels.get(event, short(event, 100))


def save_diagnostic(payload, directory):
    """Do not let a disk/permission error replace the original task result."""
    try:
        root = Path(directory).expanduser().resolve()
        root.mkdir(parents=True, exist_ok=True)
        path = root / (datetime.now().strftime("%Y%m%d_%H%M%S_%f_") + uuid4().hex[:8] + ".json")
        payload["diagnostic_file"] = str(path)
        with path.open("x", encoding="utf-8") as stream:
            json.dump(payload, stream, ensure_ascii=False, indent=2)
            stream.write("\n")
    except (OSError, ValueError, TypeError) as exc:
        payload.pop("diagnostic_file", None)
        payload["diagnostic_write_error"] = short(exc)
