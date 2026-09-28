"""waypoint.py —— 路径点 Pydantic 数据模型。"""

from typing import Optional
from pydantic import BaseModel


class WaypointTask(BaseModel):
    action: str = ""
    params: dict = {}


class Waypoint(BaseModel):
    point_id: str = ""
    x: float
    y: float
    yaw: float = 0.0
    execute_tasks: bool = False
    tasks: list[WaypointTask] = []
    wait_time: float = 10.0


class WaypointCreateRequest(BaseModel):
    waypoints: list[Waypoint]


class WaypointUpdateRequest(BaseModel):
    x: Optional[float] = None
    y: Optional[float] = None
    yaw: Optional[float] = None
    execute_tasks: Optional[bool] = None
    wait_time: Optional[float] = None


class SaveCurrentRequest(BaseModel):
    point_id: Optional[str] = None
    execute_tasks: bool = False
