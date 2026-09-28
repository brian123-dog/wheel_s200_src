"""schedule_manager.py —— 定时任务管理核心逻辑。

参照 ANDROID_API_README.zh-CN.md 中的 Schedule API:
  - CRON 定时触发任务
  - 计划 CRUD (创建/列表/更新/删除)
  - APScheduler 集成
"""

import json
import os
import uuid
import logging
from typing import Optional
from threading import Lock

logger = logging.getLogger(__name__)

try:
    from apscheduler.schedulers.background import BackgroundScheduler
    from apscheduler.triggers.cron import CronTrigger
    HAS_APSCHEDULER = True
except ImportError:
    HAS_APSCHEDULER = False
    logger.warning('apscheduler 未安装, 定时功能不可用。请运行: pip install apscheduler')


class ScheduleManager:
    """定时任务管理器: CRON 定时调度。"""

    def __init__(self, schedules_file: str = "", task_run_callback=None):
        self._schedules: dict[str, dict] = {}
        self._lock = Lock()
        self._schedules_file = schedules_file
        self._task_run_callback = task_run_callback  # task_id → 触发执行的回调

        if schedules_file:
            self._load()

        # APScheduler
        if HAS_APSCHEDULER:
            self._scheduler = BackgroundScheduler(timezone='Asia/Shanghai')
            self._scheduler.start()
            self._sync_jobs()
        else:
            self._scheduler = None

    # ==================== 持久化 ====================

    def _load(self):
        if self._schedules_file and os.path.exists(self._schedules_file):
            try:
                with open(self._schedules_file, 'r') as f:
                    self._schedules = json.load(f)
            except Exception as e:
                logger.warning(f'加载定时任务文件失败: {e}')

    def _save(self):
        if self._schedules_file:
            try:
                os.makedirs(os.path.dirname(self._schedules_file) if os.path.dirname(self._schedules_file) else '.', exist_ok=True)
                with open(self._schedules_file, 'w') as f:
                    json.dump(self._schedules, f, indent=2)
            except Exception as e:
                logger.warning(f'保存定时任务文件失败: {e}')

    # ==================== APScheduler 同步 ====================

    def _sync_jobs(self):
        """将 schedules 同步到 APScheduler。"""
        if not self._scheduler:
            return
        with self._lock:
            # 清除旧 job
            for job in self._scheduler.get_jobs():
                self._scheduler.remove_job(job.id)

            # 添加新 job
            for sid, sched in self._schedules.items():
                if not sched.get('enabled', True):
                    continue
                try:
                    self._scheduler.add_job(
                        func=self._fire_schedule,
                        trigger=CronTrigger.from_crontab(
                            sched.get('cron_expr', '0 8 * * *'),
                            timezone=sched.get('timezone', 'Asia/Shanghai'),
                        ),
                        id=sid,
                        replace_existing=True,
                    )
                except Exception as e:
                    logger.error(f'添加定时任务 {sid} 失败: {e}')

    def _fire_schedule(self, schedule_id: str = None):
        """APScheduler 回调: 触发任务执行。"""
        if schedule_id is None:
            return
        sched = self._schedules.get(schedule_id)
        if sched is None:
            return
        task_id = sched.get('task_id')
        logger.info(f'定时触发: schedule={schedule_id}, task={task_id}')
        if self._task_run_callback and task_id:
            self._task_run_callback(task_id)

    # ==================== CRUD ====================

    def set_run_callback(self, callback):
        self._task_run_callback = callback

    def create(self, task_id: str, trigger_type: str = "CRON",
               cron_expr: str = "0 8 * * *", timezone: str = "Asia/Shanghai") -> dict:
        with self._lock:
            sid = str(uuid.uuid4())
            sched = {
                'schedule_id': sid,
                'task_id': task_id,
                'trigger_type': trigger_type,
                'cron_expr': cron_expr,
                'timezone': timezone,
                'enabled': True,
            }
            self._schedules[sid] = sched
            self._save()
            self._sync_jobs()
            return sched

    def list_all(self) -> list[dict]:
        with self._lock:
            return list(self._schedules.values())

    def get(self, schedule_id: str) -> Optional[dict]:
        with self._lock:
            return self._schedules.get(schedule_id)

    def update(self, schedule_id: str, **kwargs) -> Optional[dict]:
        with self._lock:
            sched = self._schedules.get(schedule_id)
            if sched is None:
                return None
            allowed = ['enabled', 'trigger_type', 'cron_expr', 'timezone']
            for k, v in kwargs.items():
                if k in allowed and v is not None:
                    sched[k] = v
            self._save()
            self._sync_jobs()
            return sched

    def delete(self, schedule_id: str) -> bool:
        with self._lock:
            if schedule_id not in self._schedules:
                return False
            del self._schedules[schedule_id]
            self._save()
            self._sync_jobs()
            return True

    def shutdown(self):
        if self._scheduler:
            self._scheduler.shutdown(wait=False)
