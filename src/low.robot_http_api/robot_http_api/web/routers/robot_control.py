"""robot_control.py —— P0 机器人控制 REST 端点。

所有端点直接调 ros2_interface:
  - get_status / get_push_data: 读缓存, 线程安全
  - start_navigation / start_inspection: 异步 Action, 不阻塞
  - stop_* / confirm: 无副作用操作
"""

from fastapi import APIRouter, Request

from ..models.commands import (
    NavigateRequest, CancelRequest, InspectionStartRequest,
)

router = APIRouter(prefix="/api/robot", tags=["Robot Control"])


@router.get("/status")
async def get_status(request: Request):
    data = request.app.state.ros2_interface.get_status()
    return {"code": 0, "message": "success", "data": data}


@router.post("/navigate")
async def navigate(body: NavigateRequest, request: Request):
    data = request.app.state.ros2_interface.start_navigation(body.model_dump())
    return {"code": 0, "message": "success", "data": data}


@router.post("/cancel")
async def cancel(body: CancelRequest, request: Request):
    ros2 = request.app.state.ros2_interface
    ros2.stop_navigation()
    ros2.stop_inspection()
    return {"code": 0, "message": "success", "data": {"message": "cancelled"}}


@router.post("/inspection/start")
async def start_inspection(body: InspectionStartRequest, request: Request):
    data = request.app.state.ros2_interface.start_inspection()
    return {"code": 0, "message": "success", "data": data}


@router.post("/inspection/stop")
async def stop_inspection(request: Request):
    data = request.app.state.ros2_interface.stop_inspection()
    return {"code": 0, "message": "success", "data": data}


@router.post("/confirm_status")
async def confirm_status(request: Request):
    data = request.app.state.ros2_interface.confirm_status()
    return {"code": 0, "message": "success", "data": data}
