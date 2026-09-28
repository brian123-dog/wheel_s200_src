"""task_management.py —— P0 任务管理 REST 端点。"""

import time
import uuid
from fastapi import APIRouter, Request, HTTPException

from ..models.commands import TaskCreateRequest, Task, TaskRunResponse

router = APIRouter(prefix="/api/tasks", tags=["Task Management"])


@router.get("")
async def list_tasks(request: Request):
    cr = request.app.state.command_router
    tasks = cr.list_tasks()
    return {
        "ok": True,
        "message": "cached",
        "tasks": tasks,
    }


@router.get("/{task_id}")
async def get_task(request: Request, task_id: str):
    cr = request.app.state.command_router
    task = cr.get_task(task_id)
    if task is None:
        raise HTTPException(status_code=404, detail=f"task {task_id} not found")
    return {
        "ok": True,
        "message": "ok",
        "task": task,
        "unit_tasks": [],
    }


@router.post("")
async def create_task(request: Request, body: TaskCreateRequest):
    cr = request.app.state.command_router
    task = cr.create_task(
        name=body.name,
        task_type=body.type.value,
        waypoints=body.waypoints,
        robot_name=body.robot_name,
    )
    return {
        "ok": True,
        "message": "created",
        "task": task,
    }


@router.post("/{task_id}/run")
async def run_task(request: Request, task_id: str):
    cr = request.app.state.command_router
    task = cr.get_task(task_id)
    if task is None:
        raise HTTPException(status_code=404, detail=f"task {task_id} not found")

    cr.set_task_status(task_id, "PENDING")
    ros2 = request.app.state.ros2_interface

    if task['type'] == 'inspection':
        ros2.start_inspection()
    elif task['type'] == 'navigation':
        waypoints = task.get('waypoints', [])
        if waypoints:
            wp = waypoints[0]
            ros2.start_navigation({'x': wp.get('x', 0), 'y': wp.get('y', 0), 'yaw': wp.get('yaw', 0)})
        else:
            raise HTTPException(status_code=400, detail="task has no waypoints")

    cr.set_task_status(task_id, "RUNNING")
    return {"ok": True, "message": "queued"}


@router.post("/{task_id}/cancel")
async def cancel_task(request: Request, task_id: str):
    cr = request.app.state.command_router
    task = cr.set_task_status(task_id, "CANCELED")
    if task is None:
        raise HTTPException(status_code=404, detail=f"task {task_id} not found")
    ros2 = request.app.state.ros2_interface
    ros2.stop_navigation()
    return {"ok": True, "message": "cancelled"}


@router.post("/{task_id}/delete")
async def delete_task(request: Request, task_id: str):
    cr = request.app.state.command_router
    deleted = cr.delete_task(task_id)
    if not deleted:
        raise HTTPException(status_code=404, detail=f"task {task_id} not found")
    return {"ok": True, "message": "deleted"}
