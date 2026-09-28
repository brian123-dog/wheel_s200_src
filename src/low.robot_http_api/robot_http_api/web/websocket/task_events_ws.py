"""task_events_ws.py —— P2 WebSocket 任务状态变更推送 (2Hz 轮询)。"""

import asyncio
from fastapi import APIRouter, WebSocket, WebSocketDisconnect

router = APIRouter()


@router.websocket("/ws/task_events")
async def task_events(websocket: WebSocket):
    ros2 = websocket.app.state.ros2_interface
    await websocket.accept()

    last_status = None
    try:
        while True:
            current = ros2._task_status
            if current != last_status:
                last_status = current
                await websocket.send_json({
                    'task_status': current,
                    'current_task_id': ros2._current_task_id,
                })
            await asyncio.sleep(0.5)
    except WebSocketDisconnect:
        pass
