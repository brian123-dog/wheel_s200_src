"""velocity_control_ws.py —— P2 WebSocket 双向速度遥操作。

客户端发 {linear, angular} → 直接 publish /cmd_vel (线程安全)。
断开自动停车。
"""

from fastapi import APIRouter, WebSocket, WebSocketDisconnect

router = APIRouter()


@router.websocket("/ws/velocity_control")
async def velocity_control(websocket: WebSocket):
    ros2 = websocket.app.state.ros2_interface
    await websocket.accept()

    try:
        while True:
            data = await websocket.receive_json()
            linear = float(data.get('linear', 0.0))
            angular = float(data.get('angular', 0.0))
            ros2.set_velocity(linear, angular)
    except WebSocketDisconnect:
        ros2.stop_velocity()
    except (ValueError, KeyError):
        ros2.stop_velocity()
