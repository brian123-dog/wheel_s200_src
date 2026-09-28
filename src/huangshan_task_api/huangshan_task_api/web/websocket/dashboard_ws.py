"""dashboard_ws.py —— WebSocket 仪表盘推送。

双通道推送:
  1. car_pose: 6Hz（位姿 + 电池）
  2. bunker_status: 每 2~3 秒（料仓容量/重量/百分比）
"""

import asyncio
import random
from datetime import datetime, timezone

from fastapi import APIRouter, WebSocket, WebSocketDisconnect

router = APIRouter()


@router.websocket("/ws/dashboard")
async def dashboard(websocket: WebSocket):
    nav2 = websocket.app.state.nav2_interface
    await websocket.accept()

    async def push_car_pose():
        """6Hz 推送位姿 & 电池。"""
        while True:
            pose = nav2.get_current_pose_dict()
            battery = nav2.get_battery_percent()
            await websocket.send_json({
                "type": "car_pose",
                "robotId": "R001",
                "x": pose['x'] if pose else None,
                "y": pose['y'] if pose else None,
                "yaw": pose['yaw'] if pose else None,
                "battery": battery,
            })
            await asyncio.sleep(1.0 / 6.0)

    async def push_bunker():
        """每 2~3 秒推送料仓状态。"""
        while True:
            bunker = nav2.get_bunker_status()
            bunker["updatedAt"] = datetime.now(timezone.utc).strftime(
                "%Y-%m-%dT%H:%M:%S.000Z"
            )
            await websocket.send_json({
                "type": "bunker_status",
                "robotId": "R001",
                **bunker,
            })
            await asyncio.sleep(random.uniform(2.0, 3.0))

    try:
        await asyncio.gather(push_car_pose(), push_bunker())
    except WebSocketDisconnect:
        pass
