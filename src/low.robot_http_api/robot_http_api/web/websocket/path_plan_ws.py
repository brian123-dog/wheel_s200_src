"""path_plan_ws.py —— P1 WebSocket 1Hz 全局路径推送。"""

import asyncio
from fastapi import APIRouter, WebSocket, WebSocketDisconnect

router = APIRouter()


@router.websocket("/ws/path_plan")
async def path_plan(websocket: WebSocket):
    ros2 = websocket.app.state.ros2_interface
    await websocket.accept()

    try:
        while True:
            data = ros2.get_push_data()
            await websocket.send_json({'path': data.get('path')})
            await asyncio.sleep(1.0)
    except WebSocketDisconnect:
        pass
