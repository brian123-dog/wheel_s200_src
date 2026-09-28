"""command_router.py —— 命令路由 + 任务注册表。

职责：
  - 接收来自 HTTP 层的命令，分发到 Nav2Interface
  - 管理任务注册表 (内存 + JSON 文件持久化)
  - 生成任务 ID、维护任务状态
"""

import json
import os
import time
import uuid
from typing import Optional


class CommandRouter:
    """命令路由器: HTTP → ROS2 的分发层 + 任务注册表。"""

    def __init__(self, nav2_interface):
        self._nav2 = nav2_interface
        self._logger = nav2_interface._logger
        self._tasks: dict[str, dict] = {}
        self._tasks_file: Optional[str] = None

    def set_tasks_file(self, path: str):
        self._tasks_file = path
        self._load_tasks()

    # ==================== 命令分发 ====================

    def dispatch(self, command: str, payload: dict) -> dict:
        handler = {
            'GET_STATUS': self._handle_get_status,
            'START_INSPECTION': self._handle_start_inspection,
            'STOP_INSPECTION': self._handle_stop_inspection,
            'START_NAVIGATION': self._handle_start_navigation,
            'STOP_NAVIGATION': self._handle_stop_navigation,
        }.get(command)
        if handler is None:
            raise ValueError(f'未识别的命令: {command}')
        return handler(payload)

    def _handle_get_status(self, payload: dict) -> dict:
        return self._nav2.get_status()

    def _handle_start_inspection(self, payload: dict) -> dict:
        return self._nav2.start_inspection()

    def _handle_stop_inspection(self, payload: dict) -> dict:
        return self._nav2.stop_inspection()

    def _handle_start_navigation(self, payload: dict) -> dict:
        return self._nav2.start_navigation(payload)

    def _handle_stop_navigation(self, payload: dict) -> dict:
        return self._nav2.stop_navigation()

    # ==================== 任务注册表 ====================

    def create_task(self, name: str, task_type: str, waypoints: Optional[list] = None,
                    robot_name: str = "R001") -> dict:
        now_ms = int(time.time() * 1000)
        task = {
            'task_id': str(uuid.uuid4()),
            'name': name,
            'type': task_type,
            'robot_name': robot_name,
            'fleet_name': 'default',
            'status': 'CREATED',
            'waypoints': waypoints or [],
            'created_at_unix_ms': now_ms,
            'updated_at_unix_ms': now_ms,
        }
        self._tasks[task['task_id']] = task
        self._save_tasks()
        return task

    def list_tasks(self) -> list:
        return list(self._tasks.values())

    def get_task(self, task_id: str) -> Optional[dict]:
        return self._tasks.get(task_id)

    def set_task_status(self, task_id: str, status: str) -> Optional[dict]:
        task = self._tasks.get(task_id)
        if task is None:
            return None
        task['status'] = status
        task['updated_at_unix_ms'] = int(time.time() * 1000)
        self._save_tasks()
        return task

    def delete_task(self, task_id: str) -> bool:
        if task_id not in self._tasks:
            return False
        del self._tasks[task_id]
        self._save_tasks()
        return True

    def _load_tasks(self):
        if self._tasks_file and os.path.exists(self._tasks_file):
            try:
                with open(self._tasks_file, 'r') as f:
                    self._tasks = json.load(f)
            except Exception as e:
                self._logger.warning(f'加载任务文件失败: {e}')

    def _save_tasks(self):
        if self._tasks_file:
            try:
                with open(self._tasks_file, 'w') as f:
                    json.dump(self._tasks, f, indent=2)
            except Exception as e:
                self._logger.warning(f'保存任务文件失败: {e}')
