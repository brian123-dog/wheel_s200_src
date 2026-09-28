"""nav2_interface.py —— Nav2 导航接口封装。

参照 robot_http_api/nav2_interface.py，为 huangshan_task_api 提供导航能力。
负责通过 NavigateToPose action 驱动机器人到指定航点。
"""

import math
import threading
import time
from enum import Enum
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from action_msgs.msg import GoalStatus
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
from std_msgs.msg import Float32


class NavState(Enum):
    IDLE = "IDLE"
    NAVIGATING = "NAVIGATING"
    SUCCEEDED = "SUCCEEDED"
    FAILED = "FAILED"
    CANCELLED = "CANCELLED"


class Nav2Interface:
    """封装 NavigateToPose Action Client，供 TaskManager 调用。"""

    def __init__(self, node: Node):
        self._node = node
        self._logger = node.get_logger()

        # ---- Action Client ----
        self._action_client = ActionClient(node, NavigateToPose, 'navigate_to_pose')

        # ---- Subscribers ----
        self._current_pose: Optional[PoseStamped] = None
        self._battery: Optional[float] = None

        self._pose_sub = node.create_subscription(
            PoseWithCovarianceStamped, '/amcl_pose', self._pose_callback, 10)
        self._battery_sub = node.create_subscription(
            Float32, '/PowerVoltage', self._battery_callback, 10)

        # ---- 状态 ----
        self._state = NavState.IDLE
        self._current_goal_handle = None
        self._current_task_id: Optional[str] = None
        self._task_result_event = threading.Event()
        self._task_result_status: Optional[str] = None
        self._lock = threading.Lock()
        self._cancelled = False

        self._logger.info('Nav2Interface 初始化完成 (navigate_to_pose)')

    # ==================== 料仓数据 ====================

    def get_bunker_status(self) -> dict:
        """返回料仓状态字典，若暂无真实数据源则返回模拟数据。"""
        return {
            "percent": 64,
            "remainingG": 3200,
            "capacityG": 5000,
            "level": "normal",
            "updatedAt": None,
        }

    # ==================== 回调 ====================

    def _pose_callback(self, msg: PoseWithCovarianceStamped):
        self._current_pose = msg.pose.pose

    def _battery_callback(self, msg: Float32):
        self._battery = msg.data

    # ==================== 位置获取 ====================

    def get_current_pose_dict(self) -> Optional[dict]:
        if self._current_pose is None:
            return None
        p = self._current_pose  # 已是 Pose 类型 (msg.pose.pose)
        siny_cosp = 2.0 * (p.orientation.w * p.orientation.z)
        cosy_cosp = 1.0 - 2.0 * (p.orientation.z * p.orientation.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        return {
            'x': p.position.x,
            'y': p.position.y,
            'yaw': yaw,
        }

    def get_battery_percent(self) -> Optional[str]:
        if self._battery is None:
            return None
        return round((self._battery - 20.0) / 5.0 * 100, 2)

    # ==================== 四元数工具 ====================

    @staticmethod
    def _yaw_to_quat(yaw: float) -> dict:
        half = yaw * 0.5
        return {'x': 0.0, 'y': 0.0, 'z': math.sin(half), 'w': math.cos(half)}

    # ==================== 导航 ====================

    def navigate_to_waypoint(self, task_id: str, x: float, y: float,
                              yaw: float = 0.0, frame_id: str = 'map',
                              timeout_sec: float = 120.0) -> str:
        """同步导航到指定航点。阻塞当前线程直到导航完成或超时。

        Returns:
            'SUCCEEDED' | 'FAILED' | 'CANCELLED' | 'TIMEOUT'
        """
        # 等待 Action Server
        if not self._action_client.wait_for_server(timeout_sec=5.0):
            self._logger.error('NavigateToPose Action Server 未就绪，请确认 Nav2 已启动')
            return 'FAILED'

        with self._lock:
            self._state = NavState.NAVIGATING
            self._current_task_id = task_id
            self._task_result_status = None
            self._task_result_event.clear()
            self._cancelled = False

        # 构造 goal
        goal_msg = NavigateToPose.Goal()
        pose_stamped = PoseStamped()
        pose_stamped.header.frame_id = frame_id
        pose_stamped.header.stamp = self._node.get_clock().now().to_msg()
        pose_stamped.pose.position.x = float(x)
        pose_stamped.pose.position.y = float(y)
        q = self._yaw_to_quat(yaw)
        pose_stamped.pose.orientation.z = q['z']
        pose_stamped.pose.orientation.w = q['w']
        goal_msg.pose = pose_stamped

        self._logger.info(f'[Nav2Interface] 开始导航 task={task_id} → ({x:.3f}, {y:.3f}, yaw={yaw:.3f})')

        # 发送 goal
        send_goal_future = self._action_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self._goal_response_callback)

        # 等待结果
        finished = self._task_result_event.wait(timeout=timeout_sec)
        if not finished:
            self._logger.warning(f'[Nav2Interface] 导航超时 task={task_id}')
            self.cancel_navigation()
            with self._lock:
                self._state = NavState.FAILED
            return 'TIMEOUT'

        result = self._task_result_status or 'FAILED'
        self._logger.info(f'[Nav2Interface] 导航结束 task={task_id} result={result}')
        return result

    def _goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self._logger.error('导航 goal 被拒绝')
            self._task_result_status = 'FAILED'
            self._task_result_event.set()
            return

        with self._lock:
            self._current_goal_handle = goal_handle

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._nav_result_callback)

    def _nav_result_callback(self, future):
        if self._cancelled:
            self._task_result_status = 'CANCELLED'
        else:
            result = future.result()
            if result.status == GoalStatus.STATUS_SUCCEEDED:
                self._task_result_status = 'SUCCEEDED'
                self._state = NavState.SUCCEEDED
            else:
                self._task_result_status = 'FAILED'
                self._state = NavState.FAILED
        self._task_result_event.set()

    def cancel_navigation(self):
        with self._lock:
            if self._current_goal_handle is not None:
                self._cancelled = True
                try:
                    self._action_client.cancel_goal_async(self._current_goal_handle)
                except Exception as e:
                    self._logger.warning(f'取消导航失败: {e}')
                self._current_goal_handle = None
            self._state = NavState.CANCELLED

    def is_navigating(self) -> bool:
        return self._state == NavState.NAVIGATING

    @property
    def state(self) -> NavState:
        return self._state
