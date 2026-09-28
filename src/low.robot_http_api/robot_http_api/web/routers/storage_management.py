"""storage_management.py —— P2 库位管理 REST 端点 (参照 NETX2)。

数据持久化: JSON 文件，格式完全参照 NETX2 的 /api/storage 响应。
"""

import json
import os
import time
import copy
from fastapi import APIRouter, Request, HTTPException

from ..models.storage import SetStockRequest, EnableCellRequest, InitYamlRequest

router = APIRouter(prefix="/api/storage", tags=["Storage"])

_STORAGE_FILE = os.path.expanduser("~/wheel_S200_src/storage.json")

# 默认布局 (参照 NETX2: area_1, shelf_1, 7列5行, left/right)
_DEFAULT_LAYOUT = {
    "layout": [
        {
            "area_index": 1,
            "shelves": [
                {"shelf_index": 1, "columns": 7, "rows": 5, "shelf_side": "left"},
                {"shelf_index": 2, "columns": 7, "rows": 5, "shelf_side": "right"},
            ],
        }
    ],
    "cells": {},
    "summary": {"total": 0, "disabled": 0, "with_stock": 0},
    "timestamp": 0.0,
}


def _make_cell_id(area: int, shelf: int, col: int, row: int) -> str:
    return f"area_{area}_shelf_{shelf}_column_{col}_row_{row}"


def _build_cells(layout: list[dict]) -> dict[str, dict]:
    cells = {}
    for area in layout:
        ai = area["area_index"]
        for shelf in area.get("shelves", []):
            si = shelf["shelf_index"]
            for col in range(1, shelf["columns"] + 1):
                for row in range(1, shelf["rows"] + 1):
                    cid = _make_cell_id(ai, si, col, row)
                    cells[cid] = {"enabled": True, "stock": 0}
    return cells


def _load_storage(request: Request) -> dict:
    try:
        path = request.app.state.ros2_interface._node.get_parameter(
            'storage_file'
        ).get_parameter_value().string_value
    except Exception:
        path = _STORAGE_FILE

    if not path or not os.path.exists(path):
        return copy.deepcopy(_DEFAULT_LAYOUT)

    with open(path, 'r') as f:
        return json.load(f)


def _save_storage(request: Request, data: dict):
    try:
        path = request.app.state.ros2_interface._node.get_parameter(
            'storage_file'
        ).get_parameter_value().string_value
    except Exception:
        path = _STORAGE_FILE

    os.makedirs(os.path.dirname(path) if os.path.dirname(path) else '.', exist_ok=True)
    with open(path, 'w') as f:
        json.dump(data, f, indent=2)


def _recalc_summary(data: dict):
    cells = data.get("cells", {})
    total = len(cells)
    disabled = sum(1 for c in cells.values() if not c.get("enabled", True))
    with_stock = sum(1 for c in cells.values() if c.get("stock", 0) > 0)
    data["summary"] = {"total": total, "disabled": disabled, "with_stock": with_stock}
    data["timestamp"] = time.time()


@router.get("")
async def get_storage(request: Request):
    data = _load_storage(request)
    _recalc_summary(data)
    return {"success": True, "message": "ok", **data}


@router.post("/set_stock")
async def set_stock(request: Request, body: SetStockRequest):
    data = _load_storage(request)
    cell_ids = body.cell_ids or ([body.cell_id] if body.cell_id else [])
    updated = 0
    for cid in cell_ids:
        cell = data["cells"].get(cid)
        if cell and cell.get("enabled", True):
            cell["stock"] = body.stock
            updated += 1
    _recalc_summary(data)
    _save_storage(request, data)
    return {"success": True, "message": f"updated {updated} cells"}


@router.post("/enable")
async def enable_cells(request: Request, body: EnableCellRequest):
    data = _load_storage(request)
    cell_ids = body.cell_ids or ([body.cell_id] if body.cell_id else [])
    updated = 0
    for cid in cell_ids:
        cell = data["cells"].get(cid)
        if cell:
            cell["enabled"] = body.enable
            updated += 1
    _recalc_summary(data)
    _save_storage(request, data)
    return {"success": True, "message": f"updated {updated} cells"}


@router.get("/init_yaml")
async def get_init_yaml(request: Request):
    data = _load_storage(request)
    return {
        "success": True,
        "message": "ok",
        "layout": data.get("layout", []),
        "content": json.dumps(data.get("layout", []), indent=2),
    }


@router.post("/init_yaml")
async def save_init_yaml(request: Request, body: InitYamlRequest):
    data = _load_storage(request)
    if body.layout:
        data["layout"] = body.layout
    if body.content:
        try:
            data["layout"] = json.loads(body.content)
        except json.JSONDecodeError:
            raise HTTPException(status_code=400, detail="content 不是合法 JSON")
    _save_storage(request, data)
    return {"success": True, "message": "layout saved (not applied)"}


@router.post("/reinit")
async def reinit_storage(request: Request):
    data = _load_storage(request)
    layout = data.get("layout", [])
    cells = _build_cells(layout)
    data["cells"] = cells
    _recalc_summary(data)
    _save_storage(request, data)
    return {"success": True, "message": f"reinitialized {len(cells)} cells"}
