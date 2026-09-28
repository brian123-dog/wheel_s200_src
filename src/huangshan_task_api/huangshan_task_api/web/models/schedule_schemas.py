"""schedule_schemas.py —— 定时任务 Pydantic 数据模型 (参照 ANDROID_API_README.zh-CN.md)。"""

from typing import Optional
from pydantic import BaseModel


class ScheduleCreateRequest(BaseModel):
    task_id: str
    trigger_type: str = "CRON"
    cron_expr: str = "0 8 * * *"
    timezone: str = "Asia/Shanghai"


class ScheduleUpdateRequest(BaseModel):
    schedule_id: str
    enabled: Optional[bool] = None
    trigger_type: Optional[str] = None
    cron_expr: Optional[str] = None
    timezone: Optional[str] = None


class ScheduleDeleteRequest(BaseModel):
    schedule_id: str


class ScheduleOkResponse(BaseModel):
    ok: bool = True
    message: str = "ok"
