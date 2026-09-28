"""http_gateway_node.py —— ROS2 节点主入口 + HTTP 服务线程。

架构：
  HTTP 线程 (HttpServerThread)
       │
       ▼  (仅放入 queue.Queue)
  command_queue (线程安全)
       │
       ▼  (仅由 Timer 回调取出)
  ROS2 主线程 (GatewayNode.process_command_queue)
       │
       ▼
  Nav2Interface (所有 ROS2 调用在此完成)
"""

import json
import queue
import threading
import http.server
from dataclasses import dataclass

import rclpy
from rclpy.node import Node

from .nav2_interface import Nav2Interface
from .qr_uploader import QrUploader


@dataclass
class Command:
    command: str
    payload: dict
    response_queue: queue.Queue


class HttpRequestHandler(http.server.BaseHTTPRequestHandler):
    command_queue: queue.Queue = None
    robot_id: str = ''
    logger = None

    ALLOWED_COMMANDS = frozenset({
        'GET_STATUS',
        'START_INSPECTION',
        'STOP_INSPECTION',
        'START_NAVIGATION',
        'STOP_NAVIGATION',
    })

    def log_message(self, format, *args):
        if self.logger is not None:
            self.logger.debug(f'HTTP {self.client_address[0]}: {format % args}')

    def do_POST(self):
        if self.path != '/api/robot/command':
            self._send_error(404, 'not found')
            return
        try:
            content_length = int(self.headers.get('Content-Length', 0))
            if content_length <= 0:
                self._send_error(400, 'empty body')
                return
            body = self.rfile.read(content_length)
            data = json.loads(body.decode('utf-8'))
        except (json.JSONDecodeError, UnicodeDecodeError):
            self._send_error(400, 'invalid JSON')
            return

        if data.get('robot_id', '') != self.robot_id:
            self._send_error(403, f'robot_id mismatch, expected {self.robot_id}')
            return

        cmd_name = data.get('command', '')
        if cmd_name not in self.ALLOWED_COMMANDS:
            self._send_error(400,
                f'invalid command: {cmd_name}, allowed: {sorted(self.ALLOWED_COMMANDS)}')
            return

        payload = data.get('payload', {})
        if not isinstance(payload, dict):
            self._send_error(400, 'payload must be a JSON object')
            return

        resp_queue = queue.Queue()
        cmd = Command(command=cmd_name, payload=payload, response_queue=resp_queue)
        self.command_queue.put(cmd)

        try:
            result = resp_queue.get(timeout=8.0)
            self._send_json(result)
        except queue.Empty:
            self.logger.warning('命令处理超时')
            self._send_error(504, 'command timeout')

    def do_GET(self):
        self._send_error(405, 'only POST allowed.')

    def _send_json(self, data: dict, http_code: int = 200):
        body = json.dumps(data, ensure_ascii=False).encode('utf-8')
        self.send_response(http_code)
        self.send_header('Content-Type', 'application/json; charset=utf-8')
        self.send_header('Content-Length', str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def _send_error(self, http_code: int, message: str):
        self._send_json({'code': http_code, 'message': message, 'data': {}}, http_code=http_code)


class HttpServerThread(threading.Thread):
    def __init__(self, host, port, command_queue, robot_id, logger):
        super().__init__(daemon=True, name='HttpServerThread')
        self._host = host
        self._port = port
        self._command_queue = command_queue
        self._robot_id = robot_id
        self._logger = logger
        self._httpd = None

    def run(self):
        HttpRequestHandler.command_queue = self._command_queue
        HttpRequestHandler.robot_id = self._robot_id
        HttpRequestHandler.logger = self._logger
        self._httpd = http.server.HTTPServer((self._host, self._port), HttpRequestHandler)
        self._logger.info(f'HTTP 网关已启动: http://{self._host}:{self._port}/api/robot/command')
        self._httpd.serve_forever()

    def shutdown(self):
        if self._httpd is not None:
            self._httpd.shutdown()
            self._logger.info('HTTP 网关已关闭')


class GatewayNode(Node):
    def __init__(self):
        super().__init__('robot_http_gateway')

        self.declare_parameter('robot_id', 'R001')
        self.declare_parameter('http_host', '0.0.0.0')
        self.declare_parameter('http_port', 9099)
        self.declare_parameter('nav2_action_name', '/navigate_to_pose')
        self.declare_parameter('inspection_trigger_topic', '/inspection/start_cmd')

        robot_id = self.get_parameter('robot_id').get_parameter_value().string_value
        http_host = self.get_parameter('http_host').get_parameter_value().string_value
        http_port = self.get_parameter('http_port').get_parameter_value().integer_value

        self.nav2_interface = Nav2Interface(self)
        self.qr_uploader = QrUploader(self)
        self._command_queue: queue.Queue = queue.Queue()
        self._timer = self.create_timer(0.1, self.process_command_queue)

        self._http_thread = HttpServerThread(
            host=http_host, port=http_port,
            command_queue=self._command_queue,
            robot_id=robot_id,
            logger=self.get_logger(),
        )
        self._http_thread.start()
        self.get_logger().info('GatewayNode 启动完毕')

    def process_command_queue(self):
        while not self._command_queue.empty():
            try:
                cmd = self._command_queue.get_nowait()
            except queue.Empty:
                break
            try:
                result = self._dispatch(cmd)
                resp = {'code': 0, 'message': 'success', 'data': result}
            except Exception as e:
                self.get_logger().error(f'命令执行失败: {e}')
                resp = {'code': 500, 'message': str(e), 'data': {}}
            try:
                cmd.response_queue.put_nowait(resp)
            except Exception:
                self.get_logger().warning('HTTP 客户端可能已超时断开')

    def _dispatch(self, cmd):
        if cmd.command == 'GET_STATUS':
            return self.nav2_interface.get_status()
        elif cmd.command == 'START_INSPECTION':
            return self.nav2_interface.start_inspection()
        elif cmd.command == 'STOP_INSPECTION':
            return self.nav2_interface.stop_inspection()
        elif cmd.command == 'START_NAVIGATION':
            return self.nav2_interface.start_navigation(cmd.payload)
        elif cmd.command == 'STOP_NAVIGATION':
            return self.nav2_interface.stop_navigation()
        else:
            raise ValueError(f'未识别的命令: {cmd.command}')

    def destroy_node(self):
        self.get_logger().info('正在关闭 GatewayNode...')
        if self._http_thread is not None:
            self._http_thread.shutdown()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = GatewayNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
