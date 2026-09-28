"""app.py —— FastAPI 应用工厂 + CORS 配置。"""

from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware

from .routers.task_router import router as task_router
from .routers.storage_router import router as storage_router
from .routers.schedule_router import router as schedule_router
from .routers.fleet_router import router as fleet_router
from .websocket.dashboard_ws import router as dashboard_router


def create_app() -> FastAPI:
    app = FastAPI(
        title="Huangshan Task & Storage API",
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

    # 注册路由
    app.include_router(task_router)
    app.include_router(storage_router)
    app.include_router(schedule_router)
    app.include_router(fleet_router)
    app.include_router(dashboard_router)

    return app
