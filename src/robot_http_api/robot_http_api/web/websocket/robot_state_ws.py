"""robot_state_ws.py —— P2 WebSocket 小车实时状态推送 (2Hz 轮询)。

注意: 此 WebSocket 推送的是机器人底层的执行状态 (idle/nav_running/succeeded 等),
     与业务层的 /api/tasks 任务管理无关, 不要混淆。
"""

import asyncio
from fastapi import APIRouter, WebSocket, WebSocketDisconnect

router = APIRouter()


@router.websocket("/ws/robot_state")
async def robot_state(websocket: WebSocket):
    ros2 = websocket.app.state.ros2_interface
    await websocket.accept()

    last_status = None
    try:
        while True:
            current = ros2._task_status
            if current != last_status:
                last_status = current
                await websocket.send_json({
                    'robot_state': current,
                    'current_task_id': ros2._current_task_id,
                })
            await asyncio.sleep(0.5)
    except WebSocketDisconnect:
        pass
