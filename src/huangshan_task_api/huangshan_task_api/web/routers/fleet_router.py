"""fleet_router.py —— Fleet/机器人状态 REST 端点。

参照 ANDROID_API_README.zh-CN.md 中的辅助接口:
  GET    /api/fleets   - 获取 fleet 和机器人状态
  GET    /api/tasks    - 获取 RMF 任务缓存
"""

from fastapi import APIRouter, Request

router = APIRouter(tags=["Fleet & RMF"])


@router.get("/api/fleets")
async def get_fleets(request: Request):
    fm = request.app.state.fleet_manager
    data = fm.get_fleets()
    return data


@router.get("/api/tasks")
async def get_rmf_tasks(request: Request):
    fm = request.app.state.fleet_manager
    data = fm.get_rmf_tasks()
    return data
