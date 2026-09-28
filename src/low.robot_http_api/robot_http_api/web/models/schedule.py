"""schedule.py —— P2 定时任务 Pydantic 数据模型。"""

from typing import Optional
from pydantic import BaseModel, Field


class ScheduleCreateRequest(BaseModel):
    task_id: str
    trigger_type: str = "CRON"
    cron_expr: str = "0 8 * * *"
    timezone: str = "Asia/Shanghai"
    enabled: bool = True


class ScheduleUpdateRequest(BaseModel):
    task_id: Optional[str] = None
    enabled: Optional[bool] = None
    trigger_type: Optional[str] = None
    cron_expr: Optional[str] = None
    timezone: Optional[str] = None
