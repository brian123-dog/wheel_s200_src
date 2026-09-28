"""task_router.py —— 任务管理 REST 端点。

完全参照 ANDROID_API_README.zh-CN.md 中的任务 API 设计:
  GET    /api/task_manager/tasks           - 任务列表
  GET    /api/task_manager/task/<task_id>   - 任务详情
  POST   /api/task_manager/create_charger  - 创建充电任务
  POST   /api/task_manager/create_dropoff  - 创建 dropoff 任务
  POST   /api/task_manager/create_feeding  - 创建区域 feeding 任务
  POST   /api/task_manager/create_shelf_feeding - 创建货架 feeding 任务
  POST   /api/task_manager/run             - 运行任务
  POST   /api/task_manager/cancel          - 取消任务
  POST   /api/task_manager/delete          - 删除任务
"""

from fastapi import APIRouter, Request, HTTPException

from ..models.task_schemas import (
    CreateChargerRequest,
    CreateDropoffRequest,
    CreateFeedingRequest,
    CreateShelfFeedingRequest,
    TaskIdRequest,
)

router = APIRouter(prefix="/api/task_manager", tags=["Task Manager"])


# ==================== 查询 ====================

@router.get("/tasks")
async def list_tasks(request: Request):
    tm = request.app.state.task_manager
    tasks = tm.list_tasks()
    return {"ok": True, "message": "cached", "tasks": tasks}


@router.get("/task/{task_id}")
async def get_task(request: Request, task_id: str):
    tm = request.app.state.task_manager
    task = tm.get_task(task_id)
    if task is None:
        raise HTTPException(status_code=404, detail=f"task {task_id} not found")
    unit_tasks = task.pop('unit_tasks', [])
    return {"ok": True, "message": "ok", "task": task, "unit_tasks": unit_tasks}


# ==================== 创建 ====================

@router.post("/create_charger")
async def create_charger(request: Request, body: CreateChargerRequest):
    tm = request.app.state.task_manager
    tm.create_charger(
        fleet=body.fleet, robot=body.robot,
        waypoint=body.waypoint, name=body.name,
    )
    return {"ok": True, "message": "ok"}


@router.post("/create_dropoff")
async def create_dropoff(request: Request, body: CreateDropoffRequest):
    tm = request.app.state.task_manager
    tm.create_dropoff(
        fleet=body.fleet, robot=body.robot,
        waypoint=body.waypoint, name=body.name,
    )
    return {"ok": True, "message": "ok"}


@router.post("/create_feeding")
async def create_feeding(request: Request, body: CreateFeedingRequest):
    tm = request.app.state.task_manager
    tm.create_feeding(
        fleet=body.fleet, robot=body.robot,
        shelf_area=body.shelf_area,
        pickup_waypoint=body.pickup_waypoint,
        dropoff_waypoint=body.dropoff_waypoint,
        name=body.name,
    )
    return {"ok": True, "message": "ok"}


@router.post("/create_shelf_feeding")
async def create_shelf_feeding(request: Request, body: CreateShelfFeedingRequest):
    tm = request.app.state.task_manager
    # 兼容旧字段 shelf_index
    if body.shelf_indices is None:
        if body.shelf_index is not None:
            shelf_indices = [body.shelf_index]
        else:
            shelf_indices = []
    else:
        shelf_indices = body.shelf_indices

    tm.create_shelf_feeding(
        fleet=body.fleet, robot=body.robot,
        shelf_area=body.shelf_area,
        shelf_indices=shelf_indices,
        pickup_waypoint=body.pickup_waypoint,
        dropoff_waypoint=body.dropoff_waypoint,
        name=body.name,
    )
    return {"ok": True, "message": "ok"}


# ==================== 执行 ====================

@router.post("/run")
async def run_task(request: Request, body: TaskIdRequest):
    tm = request.app.state.task_manager
    ok = tm.run_task(body.task_id)
    if not ok:
        raise HTTPException(status_code=400, detail="任务无法运行 (可能已执行或不存在)")
    return {"ok": True, "message": "queued"}


@router.post("/cancel")
async def cancel_task(request: Request, body: TaskIdRequest):
    tm = request.app.state.task_manager
    ok = tm.cancel_task(body.task_id)
    if not ok:
        raise HTTPException(status_code=404, detail=f"task {body.task_id} not found")
    return {"ok": True, "message": "ok"}


@router.post("/delete")
async def delete_task(request: Request, body: TaskIdRequest):
    tm = request.app.state.task_manager
    ok = tm.delete_task(body.task_id)
    if not ok:
        raise HTTPException(status_code=404, detail=f"task {body.task_id} not found")
    return {"ok": True, "message": "ok"}
