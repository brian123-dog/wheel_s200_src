"""storage_manager.py —— 库位管理核心逻辑。

参照 ANDROID_API_README.zh-CN.md 中的 Storage API:
  - Cell ID 规则: area_{area}_shelf_{shelf}_column_{column}_row_{row}
  - 库存设置/查询/启用禁用
  - 布局 CRUD (初始化 YAML)
  - 数据持久化到 JSON 文件
"""

import json
import os
import time
import copy
import logging
from typing import Optional
from threading import Lock

logger = logging.getLogger(__name__)

# 默认布局 (参照 NETX2: area_1, shelf_1/2, 7列5行, left/right)
_DEFAULT_LAYOUT = [
    {
        "area_index": 1,
        "shelves": [
            {"shelf_index": 1, "columns": 7, "rows": 5, "shelf_side": "left"},
            {"shelf_index": 2, "columns": 7, "rows": 5, "shelf_side": "right"},
        ],
    }
]


def _make_cell_id(area: int, shelf: int, col: int, row: int) -> str:
    return f"area_{area}_shelf_{shelf}_column_{col}_row_{row}"


class StorageManager:
    """库位管理器: 库存、布局、启用/禁用。"""

    def __init__(self, storage_file: str = ""):
        self._cells: dict[str, dict] = {}
        self._layout: list[dict] = []
        self._init_yaml_content: str = ""
        self._lock = Lock()
        self._storage_file = storage_file
        if storage_file:
            self._load()
        else:
            self._layout = copy.deepcopy(_DEFAULT_LAYOUT)
            self._rebuild_from_layout()

    # ==================== 持久化 ====================

    def _load(self):
        if self._storage_file and os.path.exists(self._storage_file):
            try:
                with open(self._storage_file, 'r') as f:
                    data = json.load(f)
                    self._layout = data.get('layout', copy.deepcopy(_DEFAULT_LAYOUT))
                    self._cells = data.get('cells', {})
                    self._init_yaml_content = data.get('init_yaml_content', '')
            except Exception as e:
                logger.warning(f'加载库位文件失败: {e}')
                self._layout = copy.deepcopy(_DEFAULT_LAYOUT)
                self._rebuild_from_layout()
        else:
            self._layout = copy.deepcopy(_DEFAULT_LAYOUT)
            self._rebuild_from_layout()

    def _save(self):
        if self._storage_file:
            try:
                os.makedirs(os.path.dirname(self._storage_file) if os.path.dirname(self._storage_file) else '.', exist_ok=True)
                with open(self._storage_file, 'w') as f:
                    json.dump({
                        'layout': self._layout,
                        'cells': self._cells,
                        'init_yaml_content': self._init_yaml_content,
                    }, f, indent=2)
            except Exception as e:
                logger.warning(f'保存库位文件失败: {e}')

    # ==================== 布局 ====================

    def _rebuild_from_layout(self):
        """根据 layout 重建所有 cell (保留已有 stock/enabled 状态)。"""
        new_cells: dict[str, dict] = {}
        for area in self._layout:
            ai = area.get('area_index', 1)
            for shelf in area.get('shelves', []):
                si = shelf.get('shelf_index', 1)
                for col in range(1, shelf.get('columns', 7) + 1):
                    for row in range(1, shelf.get('rows', 5) + 1):
                        cid = _make_cell_id(ai, si, col, row)
                        if cid in self._cells:
                            new_cells[cid] = dict(self._cells[cid])
                        else:
                            new_cells[cid] = {'enabled': True, 'stock': 0}
        self._cells = new_cells
        self._save()

    def get_layout_data(self) -> dict:
        """返回完整库位快照 (与 ANDROID_API_README 中 GET /api/storage 格式一致)。"""
        with self._lock:
            layout = copy.deepcopy(self._layout)
            cells = copy.deepcopy(self._cells)
            total = len(cells)
            disabled = sum(1 for c in cells.values() if not c.get('enabled', True))
            with_stock = sum(1 for c in cells.values() if c.get('stock', 0) > 0)
            return {
                'layout': layout,
                'cells': cells,
                'summary': {
                    'total': total,
                    'disabled': disabled,
                    'with_stock': with_stock,
                },
                'timestamp': time.time(),
            }

    # ==================== 库存 ====================

    def set_stock(self, cell_id: Optional[str], cell_ids: Optional[list[str]], stock: int) -> dict:
        """设置库存: stock >= 0。批量设置时跳过禁用 cell。"""
        if stock < 0:
            raise ValueError('库存数量不能小于 0')

        targets = []
        if cell_id:
            targets = [cell_id]
        elif cell_ids:
            targets = cell_ids
        else:
            raise ValueError('必须提供 cell_id 或 cell_ids')

        updated = 0
        skipped = 0
        not_found = 0
        with self._lock:
            for cid in targets:
                cell = self._cells.get(cid)
                if cell is None:
                    not_found += 1
                    continue
                if not cell.get('enabled', True):
                    skipped += 1
                    continue
                cell['stock'] = stock
                updated += 1
            self._save()
        return {'updated': updated, 'skipped': skipped, 'not_found': not_found}

    def set_enable(self, cell_id: Optional[str], cell_ids: Optional[list[str]], enable: bool) -> dict:
        """启用/禁用 cell。"""
        targets = []
        if cell_id:
            targets = [cell_id]
        elif cell_ids:
            targets = cell_ids
        else:
            raise ValueError('必须提供 cell_id 或 cell_ids')

        updated = 0
        not_found = 0
        with self._lock:
            for cid in targets:
                cell = self._cells.get(cid)
                if cell is None:
                    not_found += 1
                    continue
                cell['enabled'] = enable
                updated += 1
            self._save()
        return {'updated': updated, 'not_found': not_found}

    # ==================== 初始化 YAML ====================

    def get_init_yaml(self) -> dict:
        with self._lock:
            return {
                'content': self._init_yaml_content,
                'layout': copy.deepcopy(self._layout),
            }

    def save_init_yaml(self, layout: Optional[list[dict]] = None,
                       content: Optional[str] = None) -> dict:
        """保存初始化布局, 不会立刻重建当前库位。"""
        with self._lock:
            if layout is not None:
                self._layout = layout
            if content is not None:
                self._init_yaml_content = content
            self._save()
        return {'message': 'init yaml saved'}

    def reinit(self) -> dict:
        """按初始化布局重建库位, 重置所有库存状态。"""
        with self._lock:
            self._cells.clear()
            self._rebuild_from_layout()
        return {'message': 'storage reinitialized'}
