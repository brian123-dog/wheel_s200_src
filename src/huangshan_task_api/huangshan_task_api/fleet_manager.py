"""fleet_manager.py —— Fleet 与机器人状态管理。

参照 ANDROID_API_README.zh-CN.md 中的 GET /api/fleets / GET /api/tasks。
管理 RMF 风格的 fleet/robot 状态。
"""

import json
import os
import time
import logging
from typing import Optional
from threading import Lock

logger = logging.getLogger(__name__)

# 默认 fleet 配置
_DEFAULT_FLEET_DATA = {
    "fleets": [
        {
            "fleet_name": "nest",
            "robots": [
                {
                    "robot_name": "Huangshan",
                    "status": "online",
                    "battery": 85.0,
                    "current_task": None,
                    "current_waypoint": None,
                }
            ],
        }
    ]
}


class FleetManager:
    """Fleet/机器人状态管理。"""

    def __init__(self, fleets_file: str = ""):
        self._data: dict = {}
        self._lock = Lock()
        self._fleets_file = fleets_file
        self._load()

    def _load(self):
        if self._fleets_file and os.path.exists(self._fleets_file):
            try:
                with open(self._fleets_file, 'r') as f:
                    self._data = json.load(f)
            except Exception as e:
                logger.warning(f'加载 fleet 文件失败: {e}')
                self._data = dict(_DEFAULT_FLEET_DATA)
        else:
            self._data = dict(_DEFAULT_FLEET_DATA)

    def _save(self):
        if self._fleets_file:
            try:
                os.makedirs(os.path.dirname(self._fleets_file) if os.path.dirname(self._fleets_file) else '.', exist_ok=True)
                with open(self._fleets_file, 'w') as f:
                    json.dump(self._data, f, indent=2)
            except Exception as e:
                logger.warning(f'保存 fleet 文件失败: {e}')

    def get_fleets(self) -> dict:
        with self._lock:
            return dict(self._data)

    def get_rmf_tasks(self) -> dict:
        """返回 RMF 原始任务缓存 (参照 /api/tasks)。"""
        with self._lock:
            return {
                'tasks': self._data.get('rmf_tasks', {}),
                'logs': self._data.get('rmf_logs', []),
                'history': self._data.get('rmf_history', []),
            }

    def update_robot_status(self, fleet_name: str = None, robot_name: str = None,
                            status: str = None, battery: float = None,
                            current_task: str = None, current_waypoint: str = None):
        """更新机器人状态。如果不指定 fleet/robot，更新默认 fleet 的第一个机器人。"""
        with self._lock:
            for fleet in self._data.get('fleets', []):
                if fleet_name is not None and fleet.get('fleet_name') != fleet_name:
                    continue
                for robot in fleet.get('robots', []):
                    if robot_name is not None and robot.get('robot_name') != robot_name:
                        continue
                    if status is not None:
                        robot['status'] = status
                    if battery is not None:
                        robot['battery'] = battery
                    if current_task is not None:
                        robot['current_task'] = current_task
                    if current_waypoint is not None:
                        robot['current_waypoint'] = current_waypoint
                    self._save()
                    return True
        return False
