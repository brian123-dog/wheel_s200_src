"""storage.py —— P2 库位/货架 Pydantic 数据模型 (参照 NETX2)。"""

from typing import Optional
from pydantic import BaseModel, Field


class Shelf(BaseModel):
    shelf_index: int
    columns: int
    rows: int
    shelf_side: str = "left"


class Area(BaseModel):
    area_index: int
    shelves: list[Shelf]


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
