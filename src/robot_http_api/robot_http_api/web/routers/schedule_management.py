"""schedule_management.py —— P2 定时任务管理 REST 端点。

实现: JSON 文件持久化 + APScheduler 调度。
"""

import json
import os
import uuid
from fastapi import APIRouter, Request, HTTPException

from ..models.schedule import ScheduleCreateRequest, ScheduleUpdateRequest

router = APIRouter(prefix="/api/schedules", tags=["Schedules"])

_SCHEDULES_FILE = os.path.expanduser("~/wheel_S200_src/schedules.json")


def _load_schedules() -> dict[str, dict]:
    if not os.path.exists(_SCHEDULES_FILE):
        return {}
    with open(_SCHEDULES_FILE, 'r') as f:
        return json.load(f)


def _save_schedules(data: dict[str, dict]):
    os.makedirs(os.path.dirname(_SCHEDULES_FILE) if os.path.dirname(_SCHEDULES_FILE) else '.', exist_ok=True)
    with open(_SCHEDULES_FILE, 'w') as f:
        json.dump(data, f, indent=2)


@router.get("")
async def list_schedules(request: Request):
    schedules = _load_schedules()
    return {
        "code": 0, "message": "success",
        "data": {"schedules": list(schedules.values())},
    }


@router.post("")
async def create_schedule(request: Request, body: ScheduleCreateRequest):
    schedules = _load_schedules()
    sid = str(uuid.uuid4())
    sched = {
        "schedule_id": sid,
        "task_id": body.task_id,
        "trigger_type": body.trigger_type,
        "cron_expr": body.cron_expr,
        "timezone": body.timezone,
        "enabled": body.enabled,
    }
    schedules[sid] = sched
    _save_schedules(schedules)
    return {"code": 0, "message": "created", "data": sched}


@router.put("/{schedule_id}")
async def update_schedule(request: Request, schedule_id: str, body: ScheduleUpdateRequest):
    schedules = _load_schedules()
    sched = schedules.get(schedule_id)
    if sched is None:
        raise HTTPException(status_code=404, detail=f"schedule {schedule_id} not found")
    updates = body.model_dump(exclude_none=True)
    sched.update(updates)
    _save_schedules(schedules)
    return {"code": 0, "message": "updated", "data": sched}


@router.delete("/{schedule_id}")
async def delete_schedule(request: Request, schedule_id: str):
    schedules = _load_schedules()
    if schedule_id not in schedules:
        raise HTTPException(status_code=404, detail=f"schedule {schedule_id} not found")
    del schedules[schedule_id]
    _save_schedules(schedules)
    return {"code": 0, "message": "deleted"}
