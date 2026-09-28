"""nav2_interface.py —— 封装所有 ROS2 交互逻辑。

架构：
  - 所有 ROS2 Publisher/Subscriber/ActionClient 在此创建和管理
  - 非 ROS2 线程（WebSocket / FastAPI）通过锁保护的 push_data 读取缓存
  - 速度控制通过锁保护直接 publish，保证实时性
  - 导航/巡检等需要返回结果的操作通过 queue.Queue 由 ROS2 主线程消费
"""

import math
import threading
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from action_msgs.msg import GoalStatus
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
from std_msgs.msg import String, Float32
from nav_msgs.msg import Path
from std_srvs.srv import Trigger


class Nav2Interface:
    def __init__(self, node: Node):
        self._node = node
        self._logger = node.get_logger()

        # ---- 参数 ----
        nav2_action_name = (
            node.get_parameter('nav2_action_name')
            .get_parameter_value().string_value
        )
        inspection_topic = (
            node.get_parameter('inspection_trigger_topic')
            .get_parameter_value().string_value
        )

        # ---- Action Client: NavigateToPose ----
        self._action_client = ActionClient(node, NavigateToPose, nav2_action_name)

        # ---- Publishers ----
        self._inspection_pub = node.create_publisher(String, inspection_topic, 10)
        self._cmd_vel_pub = node.create_publisher(Twist, '/cmd_vel', 10)
        self._cmd_vel_lock = threading.Lock()

        # ---- Subscribers ----
        self._current_pose = None
        self._battery = None

        self._pose_sub = node.create_subscription(
            PoseWithCovarianceStamped, '/amcl_pose', self._pose_callback, 10)
        self._battery_sub = node.create_subscription(
            Float32, '/PowerVoltage', self._battery_callback, 10)

        # ---- P1: 全局路径订阅 ----
        self._current_path = None
        self._path_sub = node.create_subscription(
            Path, '/plan', self._path_callback, 10)

        # ---- P1: 路径点记录 Service 客户端 ----
        self._save_position_client = node.create_client(Trigger, '/save_position')

        # ---- P1: 机械臂 Action 客户端 ----
        try:
            from base_interfaces.action import NavArmControl
            arm_action_name = '/nav_arm_control'
            self._arm_action_client = ActionClient(node, NavArmControl, arm_action_name)
            self._arm_goal_handle = None
            self._arm_lock = threading.Lock()
            self._arm_available = True
        except Exception as e:
            self._logger.warning(f'NavArmControl Action 不可用: {e}')
            self._arm_available = False

        # ---- 内部状态 ----
        self._task_status = 'idle'
        self._current_goal_handle = None
        self._current_task_id = None
        self._task_id_counter = 0
        self._cancelled = False

        # ---- 共享数据 (给 WebSocket 推送用) ----
        self._push_data_lock = threading.Lock()
        self._push_data = {
            'pose': None,
            'battery': None,
            'fsm': 'idle',
            'current_task_id': None,
            'path': None,
        }
        self._push_timer = node.create_timer(0.1, self._update_push_data)

        self._logger.info('Nav2Interface 初始化完成')

    # ==================== 回调 ====================

    def _pose_callback(self, msg: PoseWithCovarianceStamped):
        self._current_pose = msg.pose.pose

    def _battery_callback(self, msg: Float32):
        self._battery = msg.data

    def _path_callback(self, msg: Path):
        self._current_path = msg

    # ==================== 推送数据更新 (10Hz) ====================

    def _update_push_data(self):
        pose = None
        if self._current_pose is not None:
            p = self._current_pose
            siny_cosp = 2.0 * (p.orientation.w * p.orientation.z)
            cosy_cosp = 1.0 - 2.0 * (p.orientation.z * p.orientation.z)
            yaw = math.atan2(siny_cosp, cosy_cosp)
            pose = {'x': p.position.x, 'y': p.position.y, 'yaw': yaw}

        path = None
        if self._current_path is not None:
            path = [{'x': p.pose.position.x, 'y': p.pose.position.y}
                    for p in self._current_path.poses]

        with self._push_data_lock:
            self._push_data = {
                'pose': pose,
                'battery': f"{round((self._battery - 20.0) / 5.0 * 100, 2)}%" if self._battery is not None else None,
                'fsm': self._task_status,
                'current_task_id': self._current_task_id,
                'path': path,
            }

    def get_push_data(self) -> dict:
        """线程安全: 非 ROS2 线程调用, 返回最新快照。"""
        with self._push_data_lock:
            return dict(self._push_data)

    # ==================== 四元数工具 ====================

    @staticmethod
    def _yaw_to_quat(yaw: float) -> dict:
        half = yaw * 0.5
        return {
            'x': 0.0, 'y': 0.0,
            'z': math.sin(half), 'w': math.cos(half),
        }

    # ==================== 速度控制 (线程安全, 可被 WebSocket 线程直接调用) ====================

    def set_velocity(self, linear: float, angular: float):
        twist = Twist()
        twist.linear.x = float(linear)
        twist.angular.z = float(angular)
        with self._cmd_vel_lock:
            self._cmd_vel_pub.publish(twist)

    def stop_velocity(self):
        self.set_velocity(0.0, 0.0)

    # ==================== 巡检 ====================

    def start_inspection(self) -> dict:
        self._task_id_counter += 1
        task_id = f'INS_{self._task_id_counter:03d}'
        self._current_task_id = task_id
        self._task_status = 'inspection_running'

        msg = String()
        msg.data = 'START_INSPECTION'
        self._inspection_pub.publish(msg)

        return {'task_id': task_id, 'status': 'started'}

    def stop_inspection(self) -> dict:
        msg = String()
        msg.data = 'STOP_INSPECTION'
        self._inspection_pub.publish(msg)
        self._task_status = 'idle'
        return {'message': 'inspection stopped'}

    # ==================== 导航 ====================

    def start_navigation(self, payload: dict) -> dict:
        x = float(payload['x'])
        y = float(payload['y'])
        yaw = float(payload.get('yaw', 0.0))
        frame_id = payload.get('frame_id', 'map')

        if not self._action_client.wait_for_server(timeout_sec=5.0):
            raise RuntimeError('NavigateToPose Action Server 未就绪, 请确认 Nav2 已启动')

        goal_msg = NavigateToPose.Goal()
        pose_stamped = PoseStamped()
        pose_stamped.header.frame_id = frame_id
        pose_stamped.header.stamp = self._node.get_clock().now().to_msg()
        pose_stamped.pose.position.x = x
        pose_stamped.pose.position.y = y

        q = self._yaw_to_quat(yaw)
        pose_stamped.pose.orientation.z = q['z']
        pose_stamped.pose.orientation.w = q['w']
        goal_msg.pose = pose_stamped

        self._task_id_counter += 1
        task_id = f'NAV_{self._task_id_counter:03d}'
        self._current_task_id = task_id
        self._task_status = 'nav_running'
        self._cancelled = False

        send_goal_future = self._action_client.send_goal_async(
            goal_msg, feedback_callback=self._nav_feedback_callback)
        send_goal_future.add_done_callback(
            lambda f: self._goal_response_callback(f, task_id))

        return {'task_id': task_id, 'status': 'navigating'}

    def _goal_response_callback(self, future, task_id: str):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self._task_status = 'failed'
            return
        self._current_goal_handle = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(
            lambda f: self._nav_result_callback(f, task_id))

    def _nav_feedback_callback(self, feedback_msg):
        dist = feedback_msg.feedback.distance_remaining
        self._logger.debug(f'导航剩余距离: {dist:.2f} m')

    def _nav_result_callback(self, future, task_id: str):
        if self._cancelled:
            return
        result = future.result()
        if result.status == GoalStatus.STATUS_SUCCEEDED:
            self._task_status = 'succeeded'
        else:
            self._task_status = 'failed'

    def stop_navigation(self) -> dict:
        if self._current_goal_handle is not None:
            self._cancelled = True
            try:
                self._action_client.cancel_goal_async(self._current_goal_handle)
            except Exception as e:
                self._logger.error(f'cancel_goal_async failed: {e}')
            self._task_status = 'idle'
            self._current_goal_handle = None
            print("直行到这里了！！！！！！！！！！！！")
            return {'message': 'navigation cancelled'}
        return {'message': 'no active navigation to cancel'}

    # ==================== 状态 ====================

    def get_status(self) -> dict:
        pose_data = None
        if self._current_pose is not None:
            p = self._current_pose
            siny_cosp = 2.0 * (p.orientation.w * p.orientation.z)
            cosy_cosp = 1.0 - 2.0 * (p.orientation.z * p.orientation.z)
            yaw = math.atan2(siny_cosp, cosy_cosp)
            pose_data = {'x': p.position.x, 'y': p.position.y, 'yaw': yaw}

        return {
            'robot_pose': pose_data,
            'battery': f"{round((self._battery - 20.0) / 5.0 * 100, 2)}%" if self._battery is not None else None,
            'task_status': self._task_status,
            'current_task_id': self._current_task_id,
        }

    def confirm_status(self) -> dict:
        if self._task_status in ('succeeded', 'failed', 'canceled'):
            self._task_status = 'idle'
        return {'message': 'status confirmed', 'task_status': self._task_status}

    # ==================== P1: 路径点记录 ====================

    def save_current_position(self) -> dict:
        if not self._save_position_client.wait_for_service(timeout_sec=3.0):
            raise RuntimeError('/save_position 服务未就绪, 请确认 position_recorder 已启动')

        req = Trigger.Request()
        future = self._save_position_client.call_async(req)
        rclpy.spin_until_future_complete(self._node, future, timeout_sec=5.0)

        if future.result() is None:
            raise RuntimeError('/save_position 调用超时')
        result = future.result()
        return {
            'success': result.success,
            'message': result.message,
        }

    # ==================== P1: 机械臂控制 ====================

    def start_arm_task(self, waypoint_index: int = 0, run_full_task: bool = False) -> dict:
        if not self._arm_available:
            raise RuntimeError('NavArmControl Action 不可用')

        if not self._arm_action_client.wait_for_server(timeout_sec=3.0):
            raise RuntimeError('NavArmControl Action Server 未就绪')

        goal_msg = self._arm_action_client.Goal()
        goal_msg.waypoint_index = waypoint_index

        send_goal_future = self._arm_action_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self._node, send_goal_future, timeout_sec=5.0)

        goal_handle = send_goal_future.result()
        if goal_handle is None or not goal_handle.accepted:
            raise RuntimeError('机械臂任务被拒绝')

        with self._arm_lock:
            self._arm_goal_handle = goal_handle

        return {'status': 'started', 'waypoint_index': waypoint_index}

    def stop_arm_task(self) -> dict:
        with self._arm_lock:
            if self._arm_goal_handle is not None:
                self._arm_action_client.cancel_goal_async(self._arm_goal_handle)
                self._arm_goal_handle = None
                return {'message': 'arm task cancelled'}
        return {'message': 'no active arm task'}

    # ==================== P2: 地图保存 ====================

    def save_map(self, name: str) -> dict:
        """通过 ros2 service call 调用 map_saver 保存地图。"""
        import subprocess
        result = subprocess.run(
            ['ros2', 'service', 'call', '/map_saver/save_map',
             'nav2_msgs/srv/SaveMap', f'{{map_url: "{name}"}}'],
            capture_output=True, text=True, timeout=15.0,
        )
        if result.returncode != 0:
            raise RuntimeError(f'map_save 失败: {result.stderr.strip()}')
        return {'name': name, 'message': result.stdout.strip()}
