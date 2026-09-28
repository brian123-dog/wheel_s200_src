"""Pydantic 数据模型 —— 所有 API 请求/响应的类型定义。"""

from __future__ import annotations
from typing import Optional
from pydantic import BaseModel, Field
from enum import Enum


# ==================== 响应信封 ====================

class ApiResponse(BaseModel):
    code: int = 0
    message: str = "success"
    data: Optional[dict] = None


# ==================== 机器人状态 ====================

class RobotPose(BaseModel):
    x: float
    y: float
    yaw: float


class RobotStatusData(BaseModel):
    robot_pose: Optional[RobotPose] = None
    battery: Optional[float] = None
    task_status: str = "idle"
    current_task_id: Optional[str] = None


# ==================== 导航 ====================

class NavigateRequest(BaseModel):
    x: float
    y: float
    yaw: float = 0.0
    frame_id: str = "map"
    precision_xy: float = 0.05
    precision_yaw: float = 0.05


class NavigateResponse(BaseModel):
    task_id: str
    status: str = "navigating"


# ==================== 巡检 ====================

class InspectionStartRequest(BaseModel):
    waypoints_file: Optional[str] = None
    mission_id: Optional[str] = None


class InspectionResponse(BaseModel):
    task_id: str
    status: str


# ==================== 取消 ====================

class CancelRequest(BaseModel):
    task_id: Optional[str] = None


class CancelResponse(BaseModel):
    message: str


# ==================== 任务 ====================

class TaskStatus(str, Enum):
    CREATED = "CREATED"
    PENDING = "PENDING"
    RUNNING = "RUNNING"
    SUCCEEDED = "SUCCEEDED"
    FAILED = "FAILED"
    CANCELED = "CANCELED"
    PAUSED = "PAUSED"


class TaskType(str, Enum):
    NAVIGATION = "navigation"
    INSPECTION = "inspection"
    CHARGER = "charger"
    DROPOFF = "dropoff"


class UnitTask(BaseModel):
    unit_id: str
    seq: int = 0
    waypoint: str = ""
    action_name: str = ""
    status: TaskStatus = TaskStatus.CREATED
    attempts: int = 0
    last_error: str = ""


class Task(BaseModel):
    task_id: str
    name: str
    type: TaskType = TaskType.NAVIGATION
    robot_name: str = "R001"
    fleet_name: str = "default"
    status: TaskStatus = TaskStatus.CREATED
    created_at_unix_ms: int = 0
    updated_at_unix_ms: int = 0
    unit_tasks: list[UnitTask] = []


class TaskCreateRequest(BaseModel):
    name: str
    type: TaskType = TaskType.NAVIGATION
    waypoints: Optional[list[dict]] = None
    robot_name: str = "R001"
    fleet_name: str = "default"


class TaskRunResponse(BaseModel):
    ok: bool = True
    message: str = "queued"


# ==================== 确认状态 ====================

class ConfirmStatusResponse(BaseModel):
    code: int = 0
    message: str = "confirmed"
