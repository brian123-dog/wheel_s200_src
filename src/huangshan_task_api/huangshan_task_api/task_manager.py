"""task_manager.py —— 任务管理核心逻辑。

参照 ANDROID_API_README.zh-CN.md 中的任务 API:
  - 任务 CRUD (创建/查询/运行/取消/删除)
  - 任务状态机: CREATED → PENDING → RUNNING → SUCCEEDED/FAILED/CANCELED
  - 任务持久化到 JSON 文件
"""

import json
import os
import time
import uuid
import logging
from typing import Optional, Callable
from threading import Lock

logger = logging.getLogger(__name__)


class TaskManager:
    """任务管理器: 创建、查询、运行、取消、删除任务。

    注册 execute_callback 以真正执行任务（如导航到航点）。
    若不注册，则在 run_task 后直接标记为 SUCCEEDED（向后兼容）。
    """

    def __init__(self, tasks_file: str = ""):
        self._tasks: dict[str, dict] = {}
        self._unit_tasks: dict[str, list[dict]] = {}  # task_id → list of unit_tasks
        self._lock = Lock()
        self._tasks_file = tasks_file
        self._execute_callback: Optional[Callable] = None  # (task_id, task) -> bool
        if tasks_file:
            self._load()

    # ==================== 持久化 ====================

    def _load(self):
        if self._tasks_file and os.path.exists(self._tasks_file):
            try:
                with open(self._tasks_file, 'r') as f:
                    data = json.load(f)
                    self._tasks = data.get('tasks', {})
                    self._unit_tasks = data.get('unit_tasks', {})
            except Exception as e:
                logger.warning(f'加载任务文件失败: {e}')

    def _save(self):
        if self._tasks_file:
            try:
                os.makedirs(os.path.dirname(self._tasks_file) if os.path.dirname(self._tasks_file) else '.', exist_ok=True)
                with open(self._tasks_file, 'w') as f:
                    json.dump({
                        'tasks': self._tasks,
                        'unit_tasks': self._unit_tasks,
                    }, f, indent=2)
            except Exception as e:
                logger.warning(f'保存任务文件失败: {e}')

    # ==================== 任务 CRUD ====================

    def _now_ms(self) -> int:
        return int(time.time() * 1000)

    def _make_task(self, name: str, robot_name: str, fleet_name: str,
                   task_type: str, extra: Optional[dict] = None) -> dict:
        """创建任务内部数据结构。"""
        now = self._now_ms()
        task = {
            'task_id': str(uuid.uuid4()),
            'name': name,
            'robot_name': robot_name,
            'fleet_name': fleet_name,
            'status': 'CREATED',
            'type': task_type,
            'created_at_unix_ms': now,
            'updated_at_unix_ms': now,
        }
        if extra:
            task.update(extra)
        return task

    def create_charger(self, fleet: str, robot: str, waypoint: str, name: str) -> dict:
        with self._lock:
            task = self._make_task(
                name=name or 'go_to_charger',
                robot_name=robot,
                fleet_name=fleet,
                task_type='charger',
                extra={
                    'waypoint': waypoint,
                    'pickup_waypoint': None,
                    'dropoff_waypoint': None,
                    'shelf_area': None,
                    'shelf_indices': None,
                },
            )
            self._tasks[task['task_id']] = task
            # 创建 unit_tasks
            unit = {
                'unit_id': str(uuid.uuid4()),
                'seq': 0,
                'waypoint': waypoint,
                'action_name': name or 'go_to_charger',
                'status': 'CREATED',
                'attempts': 0,
                'last_error': '',
            }
            self._unit_tasks[task['task_id']] = [unit]
            self._save()
            return task

    def create_dropoff(self, fleet: str, robot: str, waypoint: str, name: str) -> dict:
        with self._lock:
            task = self._make_task(
                name=name or 'dropoff',
                robot_name=robot,
                fleet_name=fleet,
                task_type='dropoff',
                extra={
                    'waypoint': waypoint,
                    'pickup_waypoint': None,
                    'dropoff_waypoint': waypoint,
                    'shelf_area': None,
                    'shelf_indices': None,
                },
            )
            self._tasks[task['task_id']] = task
            unit = {
                'unit_id': str(uuid.uuid4()),
                'seq': 0,
                'waypoint': waypoint,
                'action_name': name or 'dropoff',
                'status': 'CREATED',
                'attempts': 0,
                'last_error': '',
            }
            self._unit_tasks[task['task_id']] = [unit]
            self._save()
            return task

    def create_feeding(self, fleet: str, robot: str, shelf_area: int,
                       pickup_waypoint: str, dropoff_waypoint: str, name: str) -> dict:
        with self._lock:
            task = self._make_task(
                name=name or 'feeding_task',
                robot_name=robot,
                fleet_name=fleet,
                task_type='feeding',
                extra={
                    'waypoint': None,
                    'pickup_waypoint': pickup_waypoint,
                    'dropoff_waypoint': dropoff_waypoint,
                    'shelf_area': shelf_area,
                    'shelf_indices': None,
                },
            )
            self._tasks[task['task_id']] = task
            # 区域 feeding: 一个 unit_task 代表整个区域
            unit = {
                'unit_id': str(uuid.uuid4()),
                'seq': 0,
                'waypoint': pickup_waypoint,
                'action_name': name or 'feeding_task',
                'status': 'CREATED',
                'attempts': 0,
                'last_error': '',
            }
            self._unit_tasks[task['task_id']] = [unit]
            self._save()
            return task

    def create_shelf_feeding(self, fleet: str, robot: str, shelf_area: int,
                             shelf_indices: list, pickup_waypoint: str,
                             dropoff_waypoint: str, name: str) -> dict:
        with self._lock:
            task = self._make_task(
                name=name or 'shelf_feeding_task',
                robot_name=robot,
                fleet_name=fleet,
                task_type='shelf_feeding',
                extra={
                    'waypoint': None,
                    'pickup_waypoint': pickup_waypoint,
                    'dropoff_waypoint': dropoff_waypoint,
                    'shelf_area': shelf_area,
                    'shelf_indices': shelf_indices,
                },
            )
            self._tasks[task['task_id']] = task
            # 每个 shelf 一个 unit_task
            units = []
            for idx in shelf_indices:
                units.append({
                    'unit_id': str(uuid.uuid4()),
                    'seq': idx - 1,
                    'waypoint': f'{pickup_waypoint}_shelf_{idx}',
                    'action_name': name or 'shelf_feeding_task',
                    'status': 'CREATED',
                    'attempts': 0,
                    'last_error': '',
                })
            self._unit_tasks[task['task_id']] = units
            self._save()
            return task

    def list_tasks(self) -> list[dict]:
        with self._lock:
            result = []
            for tid, t in self._tasks.items():
                result.append({
                    'task_id': t['task_id'],
                    'name': t['name'],
                    'robot_name': t['robot_name'],
                    'fleet_name': t['fleet_name'],
                    'status': t['status'],
                    'created_at_unix_ms': t['created_at_unix_ms'],
                    'updated_at_unix_ms': t['updated_at_unix_ms'],
                })
            return result

    def get_task(self, task_id: str) -> Optional[dict]:
        with self._lock:
            task = self._tasks.get(task_id)
            if task is None:
                return None
            result = dict(task)
            result['unit_tasks'] = self._unit_tasks.get(task_id, [])
            return result

    def set_execute_callback(self, callback: Callable):
        """注册任务执行回调。callback(task_id, task_dict) → bool (是否成功)。"""
        self._execute_callback = callback

    def run_task(self, task_id: str) -> bool:
        """将任务从 CREATED → PENDING → RUNNING → 执行 → SUCCEEDED/FAILED。

        若注册了 execute_callback，会在独立线程中真正执行（如导航到航点）。
        否则直接标记 SUCCEEDED（向后兼容）。
        """
        # 1. 状态流转 CREATED → PENDING
        with self._lock:
            task = self._tasks.get(task_id)
            if task is None:
                return False
            if task['status'] not in ('CREATED', 'PENDING'):
                return False
            task['status'] = 'PENDING'
            task['updated_at_unix_ms'] = self._now_ms()
            for u in self._unit_tasks.get(task_id, []):
                if u['status'] == 'CREATED':
                    u['status'] = 'PENDING'
            self._save()

        # 2. 若有执行回调，异步执行
        if self._execute_callback is not None:
            import threading
            t = threading.Thread(
                target=self._run_task_async,
                args=(task_id,),
                daemon=True,
                name=f'TaskExec-{task_id[:8]}',
            )
            t.start()
        else:
            # 3. 无回调: 直接标记成功
            self.set_task_status(task_id, 'SUCCEEDED')

        return True

    def _run_task_async(self, task_id: str):
        """在子线程中执行实际任务（导航等）。"""
        logger.info(f'[TaskManager] 开始执行任务 {task_id}')
        with self._lock:
            task = self._tasks.get(task_id)
            if task is None:
                return
            task['status'] = 'RUNNING'
            task['updated_at_unix_ms'] = self._now_ms()
            for u in self._unit_tasks.get(task_id, []):
                if u['status'] in ('CREATED', 'PENDING'):
                    u['status'] = 'RUNNING'
            self._save()

        try:
            success = self._execute_callback(task_id, task)
        except Exception as e:
            logger.error(f'[TaskManager] 任务执行异常 task={task_id}: {e}')
            success = False

        if success:
            self.set_task_status(task_id, 'SUCCEEDED')
        else:
            self.set_task_status(task_id, 'FAILED')

    def cancel_task(self, task_id: str) -> bool:
        with self._lock:
            task = self._tasks.get(task_id)
            if task is None:
                return False
            task['status'] = 'CANCELED'
            task['updated_at_unix_ms'] = self._now_ms()
            for u in self._unit_tasks.get(task_id, []):
                if u['status'] in ('CREATED', 'PENDING', 'RUNNING'):
                    u['status'] = 'CANCELED'
            self._save()
            return True

    def delete_task(self, task_id: str) -> bool:
        with self._lock:
            if task_id not in self._tasks:
                return False
            del self._tasks[task_id]
            self._unit_tasks.pop(task_id, None)
            self._save()
            return True

    def set_task_status(self, task_id: str, status: str) -> Optional[dict]:
        """手工设置任务状态 (供外部调度器调用)。"""
        with self._lock:
            task = self._tasks.get(task_id)
            if task is None:
                return None
            valid_statuses = ['CREATED', 'PENDING', 'RUNNING', 'SUCCEEDED', 'FAILED', 'CANCELED', 'PAUSED']
            if status not in valid_statuses:
                raise ValueError(f'无效的任务状态: {status}')
            task['status'] = status
            task['updated_at_unix_ms'] = self._now_ms()
            self._save()
            return task
