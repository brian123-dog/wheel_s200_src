"""waypoint_management.py —— P1 路径点管理 REST 端点。

依赖: nav2_interface 的 /save_position 服务 (调用 C++ position_recorder)。
数据: 路径点 JSON 文件 (与 waypoint_action_publisher 共享格式)。
"""

import json
import os
import uuid
from fastapi import APIRouter, Request, HTTPException

from ..models.waypoint import (
    WaypointCreateRequest, WaypointUpdateRequest, SaveCurrentRequest,
)

router = APIRouter(prefix="/api/waypoints", tags=["Waypoints"])


def _get_waypoints_file(request: Request) -> str:
    """获取路径点文件路径 (从 gateway_node 参数读取)。"""
    node = request.app.state.ros2_interface._node
    path = node.get_parameter('waypoints_file').get_parameter_value().string_value
    if not path:
        raise HTTPException(status_code=500, detail="waypoints_file 未配置")
    return path


def _load_waypoints(request: Request) -> list[dict]:
    filepath = _get_waypoints_file(request)
    if not os.path.exists(filepath):
        return []
    with open(filepath, 'r') as f:
        data = json.load(f)
    # 兼容多种 JSON 格式
    if isinstance(data, list):
        return data
    if isinstance(data, dict):
        return data.get('position', data.get('waypoints', []))
    return []


def _save_waypoints(request: Request, waypoints: list[dict]):
    filepath = _get_waypoints_file(request)
    os.makedirs(os.path.dirname(filepath), exist_ok=True)
    data = {'position': waypoints}
    with open(filepath, 'w') as f:
        json.dump(data, f, indent=2)


@router.get("")
async def list_waypoints(request: Request):
    waypoints = _load_waypoints(request)
    return {"code": 0, "message": "success", "data": {"waypoints": waypoints}}


@router.post("")
async def create_waypoints(request: Request, body: WaypointCreateRequest):
    existing = _load_waypoints(request)
    for wp in body.waypoints:
        if not wp.point_id:
            wp.point_id = str(uuid.uuid4())[:8]
        existing.append(wp.model_dump())
    _save_waypoints(request, existing)
    return {"code": 0, "message": "created", "data": {"count": len(body.waypoints)}}


@router.put("/{point_id}")
async def update_waypoint(request: Request, point_id: str, body: WaypointUpdateRequest):
    waypoints = _load_waypoints(request)
    for wp in waypoints:
        if wp.get('point_id') == point_id:
            updates = body.model_dump(exclude_none=True)
            wp.update(updates)
            _save_waypoints(request, waypoints)
            return {"code": 0, "message": "updated", "data": wp}
    raise HTTPException(status_code=404, detail=f"waypoint {point_id} not found")


@router.delete("/{point_id}")
async def delete_waypoint(request: Request, point_id: str):
    waypoints = _load_waypoints(request)
    filtered = [wp for wp in waypoints if wp.get('point_id') != point_id]
    if len(filtered) == len(waypoints):
        raise HTTPException(status_code=404, detail=f"waypoint {point_id} not found")
    _save_waypoints(request, filtered)
    return {"code": 0, "message": "deleted"}


@router.post("/save_current")
async def save_current_position(request: Request, body: SaveCurrentRequest):
    """调用 C++ position_recorder 的 /save_position 服务记录当前位置。"""
    ros2 = request.app.state.ros2_interface
    try:
        result = ros2.save_current_position()
        return {"code": 0, "message": "saved", "data": result}
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))
