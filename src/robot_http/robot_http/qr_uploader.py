"""qr_uploader.py —— 订阅 /qr_detected 话题，读取 JSON 并上传到 Web 后端。

职责：收到二维码识别结果（JSON 文件路径）后，读取文件内容并通过 HTTP POST 上传。
所有 ROS2 调用通过传入的 Node 句柄完成，不自己创建 Node。
"""

import urllib.request
import urllib.error
import json

from std_msgs.msg import String


class QrUploader:
    """订阅 qr_detected 话题，自动上传二维码 JSON 到 Web 后端。"""

    def __init__(self, node):
        """初始化上传器。

        Args:
            node: rclpy.node.Node 实例，由 GatewayNode 传入。
        """
        self._node = node
        self._logger = node.get_logger()

        # ---- 参数读取 ----
        node.declare_parameter('web_ip', '192.168.110.100')
        node.declare_parameter('web_port', 5000)
        node.declare_parameter('qr_upload_topic', '/qr_detected')

        web_ip = node.get_parameter('web_ip').get_parameter_value().string_value
        web_port = node.get_parameter('web_port').get_parameter_value().integer_value
        qr_topic = node.get_parameter('qr_upload_topic').get_parameter_value().string_value

        self._endpoint = f'http://{web_ip}:{web_port}/api/robot/qr_scan'
        self._fail_endpoint = f'http://{web_ip}:{web_port}/api/robot/qr_scan_failed'

        # ---- 订阅二维码检测结果 ----
        self._sub = node.create_subscription(
            String,
            qr_topic,
            self._on_qr_detected,
            10,
        )

        self._logger.info(
            f'QrUploader 已启动: topic={qr_topic}, endpoint={self._endpoint}'
        )

    def _on_qr_detected(self, msg):
        """收到二维码检测结果回调。

        msg.data 是 JSON 文件的绝对路径，如 /home/sunrise/qr_results/qr_xxx.json

        Args:
            msg: std_msgs.msg.String，内容是 JSON 文件路径。
        """
        path = msg.data.strip()
        if not path:
            self._logger.warning('收到空的 QR 文件路径，忽略')
            return

        self._logger.info(f'收到 QR 文件路径: {path}')

        # 1. 读取 JSON 文件
        try:
            with open(path, 'r', encoding='utf-8') as f:
                content = f.read()
        except FileNotFoundError:
            self._logger.error(f'QR JSON 文件不存在: {path}')
            self._send_failure_alert(f'QR JSON 文件不存在: {path}')
            return
        except Exception as e:
            self._logger.error(f'读取 QR JSON 文件失败 [{path}]: {e}')
            self._send_failure_alert(f'读取 QR JSON 文件失败: {e}')
            return

        # 验证是否为有效 JSON
        try:
            json.loads(content)
        except json.JSONDecodeError as e:
            self._logger.error(f'QR 文件不是有效 JSON [{path}]: {e}')
            self._send_failure_alert(f'QR 文件不是有效 JSON: {e}')
            return

        # 2. 上传到 Web 后端
        self._upload(path, content)

    def _upload(self, path, content):
        """通过 HTTP POST 上传 JSON 数据。

        Args:
            path: 原始文件路径（仅用于日志）。
            content: JSON 字符串。
        """
        try:
            data = content.encode('utf-8')
            req = urllib.request.Request(
                self._endpoint,
                data=data,
                headers={'Content-Type': 'application/json; charset=utf-8'},
                method='POST',
            )
            with urllib.request.urlopen(req, timeout=10.0) as resp:
                status = resp.status
                if 200 <= status < 300:
                    self._logger.info(
                        f'QR 上传成功 [HTTP {status}]: {path}'
                    )
                else:
                    self._logger.warning(
                        f'QR 上传返回非成功状态 [HTTP {status}]: {path}'
                    )
        except urllib.error.HTTPError as e:
            self._logger.warning(
                f'QR 上传 HTTP 错误 [{e.code}]: {path} — {e.reason}'
            )
        except urllib.error.URLError as e:
            self._logger.warning(
                f'QR 上传连接失败: {path} — {e.reason}'
            )
        except Exception as e:
            self._logger.error(f'QR 上传未知异常: {path} — {e}')

    def _send_failure_alert(self, message: str):
        """向 Web 后端 POST 二维码识别/解析失败告警。"""
        try:
            data = json.dumps({
                "robot_id": "R001",
                "point_id": "",
                "message": message
            }).encode('utf-8')
            req = urllib.request.Request(
                self._fail_endpoint,
                data=data,
                headers={'Content-Type': 'application/json; charset=utf-8'},
                method='POST',
            )
            with urllib.request.urlopen(req, timeout=5.0) as resp:
                self._logger.info(f'QR 失败告警已上报 Web: HTTP {resp.status}')
        except Exception as e:
            self._logger.debug(f'QR 失败告警上报 Web 失败: {e}')
