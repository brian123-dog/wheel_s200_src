"""app.py —— FastAPI 应用工厂 + CORS 配置。"""

from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware

from .routers import (
    robot_control, task_management,        # P0
    waypoint_management, arm_control,       # P1
    map_management, storage_management,     # P2
    schedule_management, system_control,    # P2
)
from .websocket import (
    robot_data_ws, path_plan_ws,           # P0, P1
    velocity_control_ws, task_events_ws,    # P2
)


def create_app(ros2_interface, command_router, robot_id: str) -> FastAPI:
    app = FastAPI(
        title="S100 Robot API",
        version="1.0.0",
        docs_url="/docs",
        redoc_url=None,
    )

    app.add_middleware(
        CORSMiddleware,
        allow_origins=["*"],
        allow_credentials=True,
        allow_methods=["*"],
        allow_headers=["*"],
    )

    # 将 ROS2 侧对象注入 app.state, 供路由和 WebSocket 访问
    app.state.ros2_interface = ros2_interface
    app.state.command_router = command_router
    app.state.robot_id = robot_id

    # ---- P0 路由 ----
    app.include_router(robot_control.router)
    app.include_router(task_management.router)
    app.include_router(robot_data_ws.router)

    # ---- P1 路由 (注释即可禁用) ----
    app.include_router(waypoint_management.router)
    app.include_router(arm_control.router)
    app.include_router(path_plan_ws.router)

    # ---- P2 路由 (注释即可禁用) ----
    app.include_router(map_management.router)
    app.include_router(storage_management.router)
    app.include_router(schedule_management.router)
    app.include_router(system_control.router)
    app.include_router(velocity_control_ws.router)
    app.include_router(task_events_ws.router)

    return app
