"""task_schemas.py —— 任务相关 Pydantic 数据模型 (参照 ANDROID_API_README.zh-CN.md)。"""

from __future__ import annotations
from typing import Optional
from pydantic import BaseModel, Field
from enum import Enum


# ==================== 枚举 ====================

class TaskStatus(str, Enum):
    CREATED = "CREATED"
    PENDING = "PENDING"
    RUNNING = "RUNNING"
    SUCCEEDED = "SUCCEEDED"
    FAILED = "FAILED"
    CANCELED = "CANCELED"
    PAUSED = "PAUSED"


# ==================== 任务 ====================

class UnitTask(BaseModel):
    unit_id: str
    seq: int = 0
    waypoint: str = ""
    action_name: str = ""
    status: str = "CREATED"
    attempts: int = 0
    last_error: str = ""


class TaskSummary(BaseModel):
    task_id: str
    name: str
    robot_name: str
    fleet_name: str
    status: str
    created_at_unix_ms: int
    updated_at_unix_ms: int


class TaskDetail(BaseModel):
    task_id: str
    name: str
    robot_name: str
    fleet_name: str
    status: str
    created_at_unix_ms: int
    updated_at_unix_ms: int
    unit_tasks: list[UnitTask] = []


# ==================== 创建任务请求 ====================

class CreateChargerRequest(BaseModel):
    fleet: str = "nest"
    robot: str = "Huangshan"
    waypoint: str = "charger_1"
    name: str = "go_to_charger"


class CreateDropoffRequest(BaseModel):
    fleet: str = "nest"
    robot: str = "Huangshan"
    waypoint: str = "dropoff_1"
    name: str = "dropoff"


class CreateFeedingRequest(BaseModel):
    fleet: str = "nest"
    robot: str = "Huangshan"
    shelf_area: int = 1
    pickup_waypoint: str = "pickup_1"
    dropoff_waypoint: str = "dropoff_1"
    name: str = "feeding_task"


class CreateShelfFeedingRequest(BaseModel):
    fleet: str = "nest"
    robot: str = "Huangshan"
    shelf_area: int = 1
    shelf_indices: Optional[list[int]] = None
    shelf_index: Optional[int] = None  # 兼容旧字段
    pickup_waypoint: str = "pickup_1"
    dropoff_waypoint: str = "dropoff_1"
    name: str = "shelf_feeding_task"


# ==================== 运行/取消/删除请求 ====================

class TaskIdRequest(BaseModel):
    task_id: str


# ==================== 响应 ====================

class TaskOkResponse(BaseModel):
    ok: bool = True
    message: str = "ok"


class TaskListResponse(BaseModel):
    ok: bool = True
    message: str = "cached"
    tasks: list[dict] = []


class TaskDetailResponse(BaseModel):
    ok: bool = True
    message: str = "ok"
    task: Optional[dict] = None
    unit_tasks: list[dict] = []
