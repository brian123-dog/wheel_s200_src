"""gateway_node.py —— ROS2 主节点 + FastAPI 线程管理。

架构:
  FastAPI 线程 (uvicorn)
       │
       ├── task_router   → TaskManager (CRUD + 状态机)
       ├── storage_router → StorageManager (库位布局 + 库存)
       ├── schedule_router → ScheduleManager (CRON 定时)
       └── fleet_router   → FleetManager (fleet/robot 状态)

  ROS2 主线程: 维护节点生命周期 + Nav2Interface (导航)。
"""

import json
import os
import threading
import time
import logging

import uvicorn
import rclpy
from rclpy.node import Node

from .task_manager import TaskManager
from .storage_manager import StorageManager
from .schedule_manager import ScheduleManager
from .fleet_manager import FleetManager
from .nav2_interface import Nav2Interface
from .web.app import create_app

logger = logging.getLogger(__name__)


class GatewayNode(Node):
    def __init__(self):
        super().__init__('huangshan_task_api')

        # ---- 参数 ----
        self.declare_parameter('http_host', '0.0.0.0')
        self.declare_parameter('http_port', 6080)
        self.declare_parameter('tasks_file', '')
        self.declare_parameter('storage_file', '')
        self.declare_parameter('schedules_file', '')
        self.declare_parameter('fleets_file', '')
        self.declare_parameter('waypoints_file', '')
        self.declare_parameter('nav_timeout_sec', 120.0)

        http_host = self.get_parameter('http_host').get_parameter_value().string_value
        http_port = self.get_parameter('http_port').get_parameter_value().integer_value
        tasks_file = self.get_parameter('tasks_file').get_parameter_value().string_value
        storage_file = self.get_parameter('storage_file').get_parameter_value().string_value
        schedules_file = self.get_parameter('schedules_file').get_parameter_value().string_value
        fleets_file = self.get_parameter('fleets_file').get_parameter_value().string_value
        waypoints_file = self.get_parameter('waypoints_file').get_parameter_value().string_value
        nav_timeout = self.get_parameter('nav_timeout_sec').get_parameter_value().double_value

        # ---- 核心组件 ----
        self.task_manager = TaskManager(tasks_file)
        self.storage_manager = StorageManager(storage_file)
        self.fleet_manager = FleetManager(fleets_file)

        # ---- 导航接口 ----
        self.nav2_interface = Nav2Interface(self)
        self._waypoints_file = waypoints_file
        self._nav_timeout = nav_timeout

        # ---- 加载航点配置文件 ----
        self._waypoints: dict[str, dict] = {}
        if waypoints_file and os.path.exists(waypoints_file):
            self._waypoints = self._load_waypoints(waypoints_file)
            self.get_logger().info(f'已加载 {len(self._waypoints)} 个航点')

        # ---- 注册任务执行回调 ----
        self.task_manager.set_execute_callback(self._execute_task)

        # ScheduleManager 需要回调来触发任务运行
        self.schedule_manager = ScheduleManager(
            schedules_file=schedules_file,
            task_run_callback=self._on_schedule_fire,
        )

        # ---- FastAPI 应用 ----
        app = create_app()
        app.state.task_manager = self.task_manager
        app.state.storage_manager = self.storage_manager
        app.state.schedule_manager = self.schedule_manager
        app.state.fleet_manager = self.fleet_manager
        app.state.nav2_interface = self.nav2_interface
        self._app = app

        # ---- FastAPI 线程 ----
        self._fastapi_thread = threading.Thread(
            target=self._run_fastapi,
            args=(http_host, http_port),
            daemon=True,
            name='FastAPIThread',
        )
        self._fastapi_thread.start()
        self.get_logger().info(
            f'Huangshan Task API 启动完毕: http://{http_host}:{http_port}'
        )
        self.get_logger().info('API 文档: http://{}:{}/docs'.format(http_host, http_port))

    def _run_fastapi(self, host: str, port: int):
        uvicorn.run(self._app, host=host, port=port, log_level="info")

    def _load_waypoints(self, file_path: str) -> dict[str, dict]:
        """加载航点 JSON 文件，返回 {waypoint_name: {x, y, yaw}}。"""
        with open(file_path, 'r') as f:
            data = json.load(f)

        waypoints = {}
        points = []
        if isinstance(data, dict):
            if 'position' in data:
                points = data['position']
            elif 'waypoints' in data:
                points = data['waypoints']
            elif 'points' in data:
                points = data['points']
        elif isinstance(data, list):
            points = data

        for i, pt in enumerate(points):
            name = pt.get('name', pt.get('point_id', f'waypoint_{i}'))
            waypoints[name] = {
                'x': float(pt['x']),
                'y': float(pt['y']),
                'yaw': float(pt.get('yaw', 0.0)),
            }
        return waypoints

    def _execute_task(self, task_id: str, task: dict) -> bool:
        """任务执行回调: 调用 Nav2Interface 导航到目标航点。

        task 包含:
          - waypoint: 充电/dropoff 任务的直接航点名
          - pickup_waypoint / dropoff_waypoint: feeding 任务的取/送航点
        """
        task_type = task.get('type', '')
        waypoint_name = task.get('waypoint')
        pickup = task.get('pickup_waypoint')
        dropoff = task.get('dropoff_waypoint')

        self.get_logger().info(
            f'[执行任务] task_id={task_id} type={task_type} '
            f'wp={waypoint_name} pickup={pickup} dropoff={dropoff}'
        )

        success = True

        if task_type in ('charger', 'dropoff'):
            # 单航点任务: 去充电桩 / 去 dropoff
            wp_name = waypoint_name
            if wp_name and wp_name in self._waypoints:
                wp = self._waypoints[wp_name]
                result = self.nav2_interface.navigate_to_waypoint(
                    task_id, wp['x'], wp['y'], wp['yaw'],
                    timeout_sec=self._nav_timeout,
                )
                success = (result == 'SUCCEEDED')
                # 更新 fleet 状态
                self.fleet_manager.update_robot_status(
                    current_task=task['name'], current_waypoint=wp_name)
            else:
                self.get_logger().warning(
                    f'航点 "{wp_name}" 未在配置文件中找到，'
                    f'将直接标记成功（仅测试用）')
                success = True

        elif task_type in ('feeding', 'shelf_feeding'):
            # 多航点任务: pickup → dropoff
            pickup_name = pickup
            dropoff_name = dropoff

            if pickup_name and pickup_name in self._waypoints:
                wp = self._waypoints[pickup_name]
                self.get_logger().info(f'导航到取货点: {pickup_name}')
                result = self.nav2_interface.navigate_to_waypoint(
                    task_id, wp['x'], wp['y'], wp['yaw'],
                    timeout_sec=self._nav_timeout,
                )
                if result != 'SUCCEEDED':
                    self.get_logger().error(f'取货点导航失败: {result}')
                    return False
                self.fleet_manager.update_robot_status(
                    current_task=f'{task["name"]} (pickup)',
                    current_waypoint=pickup_name)
            else:
                self.get_logger().warning(
                    f'取货点 "{pickup_name}" 未在配置文件中找到')

            if dropoff_name and dropoff_name in self._waypoints:
                wp = self._waypoints[dropoff_name]
                self.get_logger().info(f'导航到送货点: {dropoff_name}')
                result = self.nav2_interface.navigate_to_waypoint(
                    task_id, wp['x'], wp['y'], wp['yaw'],
                    timeout_sec=self._nav_timeout,
                )
                if result != 'SUCCEEDED':
                    self.get_logger().error(f'送货点导航失败: {result}')
                    return False
                self.fleet_manager.update_robot_status(
                    current_task=f'{task["name"]} (dropoff)',
                    current_waypoint=dropoff_name)
            else:
                self.get_logger().warning(
                    f'送货点 "{dropoff_name}" 未在配置文件中找到')

            success = True

        # 任务结束后更新状态
        self.fleet_manager.update_robot_status(
            current_task=None, current_waypoint=None)
        return success

    def _on_schedule_fire(self, task_id: str):
        """定时任务触发: 自动将 CREATED 任务设为 PENDING。"""
        self.task_manager.run_task(task_id)
        self.get_logger().info(f'定时触发任务: {task_id}')

    def destroy_node(self):
        self.get_logger().info('正在关闭 Huangshan Task API ...')
        self.nav2_interface.cancel_navigation()
        self.schedule_manager.shutdown()
        super().destroy_node()
