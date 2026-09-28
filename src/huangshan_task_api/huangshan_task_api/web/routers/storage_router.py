"""storage_router.py —— 库位管理 REST 端点。

完全参照 ANDROID_API_README.zh-CN.md 中的 Storage API 设计:
  GET    /api/storage                   - 获取库位状态
  POST   /api/storage/set_stock         - 设置库存
  POST   /api/storage/enable            - 启用/禁用库位
  GET    /api/storage/init_yaml         - 读取初始化布局
  POST   /api/storage/init_yaml         - 保存初始化布局
  POST   /api/storage/reinit            - 应用初始化布局
"""

from fastapi import APIRouter, Request, HTTPException

from ..models.storage_schemas import (
    SetStockRequest,
    EnableCellRequest,
    InitYamlRequest,
)

router = APIRouter(prefix="/api/storage", tags=["Storage"])


@router.get("")
async def get_storage(request: Request):
    sm = request.app.state.storage_manager
    data = sm.get_layout_data()
    return data


@router.post("/set_stock")
async def set_stock(request: Request, body: SetStockRequest):
    sm = request.app.state.storage_manager
    try:
        result = sm.set_stock(
            cell_id=body.cell_id,
            cell_ids=body.cell_ids,
            stock=body.stock,
        )
        return {"success": True, "message": f"updated {result['updated']} cells"}
    except ValueError as e:
        raise HTTPException(status_code=400, detail=str(e))


@router.post("/enable")
async def enable_cell(request: Request, body: EnableCellRequest):
    sm = request.app.state.storage_manager
    try:
        result = sm.set_enable(
            cell_id=body.cell_id,
            cell_ids=body.cell_ids,
            enable=body.enable,
        )
        action = "enabled" if body.enable else "disabled"
        return {"success": True, "message": f"{action} {result['updated']} cells"}
    except ValueError as e:
        raise HTTPException(status_code=400, detail=str(e))


@router.get("/init_yaml")
async def get_init_yaml(request: Request):
    sm = request.app.state.storage_manager
    data = sm.get_init_yaml()
    return data


@router.post("/init_yaml")
async def save_init_yaml(request: Request, body: InitYamlRequest):
    sm = request.app.state.storage_manager
    sm.save_init_yaml(layout=body.layout, content=body.content)
    return {"success": True, "message": "init yaml saved"}


@router.post("/reinit")
async def reinit_storage(request: Request):
    sm = request.app.state.storage_manager
    sm.reinit()
    return {"success": True, "message": "storage reinitialized"}
