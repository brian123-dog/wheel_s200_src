"""gateway_node.py —— ROS2 主节点 + FastAPI 线程管理。

架构:
  FastAPI 线程 (uvicorn)
       │
       ├── REST 端点: 直接调 ros2_interface (读缓存/发异步 action)
       └── WebSocket: 读 ros2_interface.get_push_data() (线程安全)

  ROS2 主线程
       │
       ├── 10Hz Timer: 消费命令队列 (向后兼容旧格式)
       └── 10Hz Timer: 更新 push_data (供 WebSocket 读取)
"""

import json
import queue
import threading
import time
from dataclasses import dataclass

import uvicorn
import rclpy
from rclpy.node import Node

from .nav2_interface import Nav2Interface
from .command_router import CommandRouter
from .web.app import create_app


@dataclass
class Command:
    command: str
    payload: dict
    response_queue: queue.Queue


class GatewayNode(Node):
    def __init__(self):
        super().__init__('robot_http_api')

        # ---- 参数 ----
        self.declare_parameter('robot_id', 'R001')
        self.declare_parameter('http_host', '0.0.0.0')
        self.declare_parameter('http_port', 9099)
        self.declare_parameter('nav2_action_name', '/navigate_to_pose')
        self.declare_parameter('inspection_trigger_topic', '/inspection/start_cmd')
        self.declare_parameter('waypoints_file', '')
        self.declare_parameter('tasks_file', '')

        robot_id = self.get_parameter('robot_id').get_parameter_value().string_value
        http_host = self.get_parameter('http_host').get_parameter_value().string_value
        http_port = self.get_parameter('http_port').get_parameter_value().integer_value
        tasks_file = self.get_parameter('tasks_file').get_parameter_value().string_value

        # ---- 核心组件 ----
        self.nav2_interface = Nav2Interface(self)
        self.command_router = CommandRouter(self.nav2_interface)
        if tasks_file:
            self.command_router.set_tasks_file(tasks_file)

        self._command_queue: queue.Queue = queue.Queue()
        self._timer = self.create_timer(0.1, self._process_command_queue)

        # ---- FastAPI 应用 ----
        app = create_app(self.nav2_interface, self.command_router, robot_id)
        app.state.cmd_queue = self._command_queue
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
            f'robot_http_api 启动完毕: http://{http_host}:{http_port}'
        )

    def _run_fastapi(self, host: str, port: int):
        uvicorn.run(self._app, host=host, port=port, log_level="info")

    def _process_command_queue(self):
        """10Hz: 消费命令队列 (向后兼容旧 POST /api/robot/command 格式)。"""
        while not self._command_queue.empty():
            try:
                cmd = self._command_queue.get_nowait()
            except queue.Empty:
                break
            try:
                result = self.command_router.dispatch(cmd.command, cmd.payload)
                resp = {'code': 0, 'message': 'success', 'data': result}
            except Exception as e:
                self.get_logger().error(f'命令执行失败: {e}')
                resp = {'code': 500, 'message': str(e), 'data': {}}
            try:
                cmd.response_queue.put_nowait(resp)
            except Exception:
                self.get_logger().warning('HTTP 客户端可能已超时断开')

    def destroy_node(self):
        self.get_logger().info('正在关闭 robot_http_api ...')
        super().destroy_node()
