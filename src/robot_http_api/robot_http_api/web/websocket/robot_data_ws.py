"""robot_data_ws.py —— WebSocket 10Hz 机器人状态推送。"""

import asyncio
from fastapi import APIRouter, WebSocket, WebSocketDisconnect

router = APIRouter()


@router.websocket("/ws/robot_data")
async def robot_data(websocket: WebSocket):
    ros2 = websocket.app.state.ros2_interface
    await websocket.accept()

    try:
        while True:
            data = ros2.get_push_data()
            await websocket.send_json(data)
            await asyncio.sleep(0.1)
    except WebSocketDisconnect:
        pass
