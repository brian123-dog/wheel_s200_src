"""schedule_router.py —— 定时任务管理 REST 端点。

完全参照 ANDROID_API_README.zh-CN.md 中的 Schedule API 设计:
  POST   /api/task_manager/schedule         - 创建定时计划
  GET    /api/task_manager/schedules        - 获取计划列表
  POST   /api/task_manager/schedule_update  - 更新计划
  POST   /api/task_manager/schedule_delete  - 删除计划
"""

from fastapi import APIRouter, Request, HTTPException

from ..models.schedule_schemas import (
    ScheduleCreateRequest,
    ScheduleUpdateRequest,
    ScheduleDeleteRequest,
)

router = APIRouter(prefix="/api/task_manager", tags=["Schedules"])


@router.post("/schedule")
async def create_schedule(request: Request, body: ScheduleCreateRequest):
    sm = request.app.state.schedule_manager
    sched = sm.create(
        task_id=body.task_id,
        trigger_type=body.trigger_type,
        cron_expr=body.cron_expr,
        timezone=body.timezone,
    )
    return {"ok": True, "message": "ok", "schedule": sched}


@router.get("/schedules")
async def list_schedules(request: Request):
    sm = request.app.state.schedule_manager
    schedules = sm.list_all()
    return {"ok": True, "message": "ok", "schedules": schedules}


@router.post("/schedule_update")
async def update_schedule(request: Request, body: ScheduleUpdateRequest):
    sm = request.app.state.schedule_manager
    sched = sm.update(
        schedule_id=body.schedule_id,
        enabled=body.enabled,
        trigger_type=body.trigger_type,
        cron_expr=body.cron_expr,
        timezone=body.timezone,
    )
    if sched is None:
        raise HTTPException(status_code=404, detail=f"schedule {body.schedule_id} not found")
    return {"ok": True, "message": "ok", "schedule": sched}


@router.post("/schedule_delete")
async def delete_schedule(request: Request, body: ScheduleDeleteRequest):
    sm = request.app.state.schedule_manager
    ok = sm.delete(body.schedule_id)
    if not ok:
        raise HTTPException(status_code=404, detail=f"schedule {body.schedule_id} not found")
    return {"ok": True, "message": "ok"}
