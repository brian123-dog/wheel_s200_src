"""storage_schemas.py —— 库位相关 Pydantic 数据模型 (参照 ANDROID_API_README.zh-CN.md)。"""

from typing import Optional
from pydantic import BaseModel, Field


# ==================== 布局 ====================

class ShelfLayout(BaseModel):
    shelf_index: Optional[int] = None
    columns: int = 7
    rows: int = 5
    shelf_side: str = "left"


class AreaLayout(BaseModel):
    area_index: int = 1
    shelves: list[ShelfLayout] = []


# ==================== 请求体 ====================

class SetStockRequest(BaseModel):
    cell_id: Optional[str] = None
    cell_ids: Optional[list[str]] = None
    stock: int = Field(default=0, ge=0)


class EnableCellRequest(BaseModel):
    cell_id: Optional[str] = None
    cell_ids: Optional[list[str]] = None
    enable: bool = True


class InitYamlRequest(BaseModel):
    layout: Optional[list[dict]] = None
    content: Optional[str] = None


class ReinitRequest(BaseModel):
    """POST /api/storage/reinit 请求体 (空对象)。"""
    pass


# ==================== 响应 ====================

class StorageOkResponse(BaseModel):
    success: bool = True
    message: str = "ok"
