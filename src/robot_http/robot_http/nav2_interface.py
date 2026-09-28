"""nav2_interface.py —— 封装所有 ROS2 交互逻辑。

HTTP 层不直接触碰任何 ROS2 API，所有 ROS2 调用统一由此模块代理，
确保线程安全：只有 GatewayNode 主线程调用本模块的方法。
"""

import math
from rclpy.node import Node
from rclpy.action import ActionClient

from action_msgs.msg import GoalStatus
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from std_msgs.msg import String, Float32


class Nav2Interface:
    """封装 Nav2 Action、话题发布/订阅，提供面向任务的接口。"""

    def __init__(self, node: Node):
        """初始化所有 ROS2 通信句柄。

        Args:
            node: ROS2 Node 实例，由 GatewayNode 传入。
        """
        self._node = node
        self._logger = node.get_logger()

        # ---- 参数读取（若 GatewayNode 已声明则复用，否则在此声明） ----
        try:
            node.declare_parameter('nav2_action_name', '/navigate_to_pose')
        except Exception:
            pass
        try:
            node.declare_parameter('inspection_trigger_topic', '/inspection/start_cmd')
        except Exception:
            pass

        nav2_action_name = node.get_parameter(
            'nav2_action_name'
        ).get_parameter_value().string_value
        inspection_topic = node.get_parameter(
            'inspection_trigger_topic'
        ).get_parameter_value().string_value

        # ---- Action Client: NavigateToPose ----
        self._action_client = ActionClient(
            node, NavigateToPose, nav2_action_name
        )

        # ---- Publishers ----
        self._inspection_pub = node.create_publisher(
            String, inspection_topic, 10
        )

        # ---- Subscribers ----
        self._current_pose = None
        self._battery = None

        self._pose_sub = node.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose',
            self._pose_callback,
            10
        )

        # 电池话题可选，不一定存在
        # 注意: create_subscription 在话题不存在时不会抛异常，仅静默收不到消息
        self._battery_sub = node.create_subscription(
            Float32, '/battery_status', self._battery_callback, 10
        )

        # ---- 内部状态 ----
        self._task_status = 'idle'
        self._current_goal_handle = None
        self._current_task_id = None
        self._task_id_counter = 0

        self._logger.info('Nav2Interface 初始化完成')

    # ==================== 回调 ====================

    def _pose_callback(self, msg: PoseWithCovarianceStamped):
        """缓存 AMCL 最新位姿。"""
        self._current_pose = msg.pose.pose

    def _battery_callback(self, msg: Float32):
        """缓存电池电压/电量。"""
        self._battery = msg.data

    # ==================== 四元数工具 ====================

    @staticmethod
    def yaw_to_quaternion(yaw: float) -> dict:
        """极简 yaw → 四元数转换（单位四元数）。

        使用公式: q = (cos(yaw/2), 0, 0, sin(yaw/2))
        不依赖 tf_transformations 等任何外部库。

        Args:
            yaw: 偏航角（弧度）。

        Returns:
            字典 {'x','y','z','w'} 对应 geometry_msgs/Quaternion。
        """
        half = yaw * 0.5
        return {
            'x': 0.0,
            'y': 0.0,
            'z': math.sin(half),
            'w': math.cos(half),
        }

    # ==================== 巡检 ====================

    def start_inspection(self) -> dict:
        """启动巡检：发布 START_INSPECTION 到 /inspection/start_cmd。

        Returns:
            {'task_id': str, 'status': 'started'}
        """
        self._task_id_counter += 1
        task_id = f'INS_{self._task_id_counter:03d}'
        self._current_task_id = task_id
        self._task_status = 'inspection_running'

        msg = String()
        msg.data = 'START_INSPECTION'
        self._inspection_pub.publish(msg)
        self._logger.info(f'[{task_id}] 巡检已启动')

        return {'task_id': task_id, 'status': 'started'}

    def stop_inspection(self) -> dict:
        """停止巡检：发布 STOP_INSPECTION，由 inspection_master 负责取消导航。

        注意：不再自行取消 Nav2 Goal，避免与 inspection_master 重复操作。
        """
        msg = String()
        msg.data = 'STOP_INSPECTION'
        self._inspection_pub.publish(msg)
        self._task_status = 'idle'
        self._logger.info('巡检停止指令已发布（由 inspection_master 处理）')

        return {'message': 'inspection stopped'}

    # ==================== 导航 ====================

    def start_navigation(self, payload: dict) -> dict:
        """发起导航目标（非阻塞）。

        payload 格式:
            {
                "x": float,
                "y": float,
                "yaw": float,
                "frame_id": "map"   # 可选，默认 "map"
            }

        Returns:
            {'task_id': str, 'status': 'navigating'}
            若 Action Server 不可用，返回错误信息。
        """
        # 校验 payload
        try:
            x = float(payload['x'])
            y = float(payload['y'])
            yaw = float(payload['yaw'])
            frame_id = payload.get('frame_id', 'map')
        except (KeyError, ValueError, TypeError) as e:
            raise ValueError(f'导航参数无效: {e}')

        # 等待 Action Server 就绪
        if not self._action_client.wait_for_server(timeout_sec=5.0):
            raise RuntimeError(
                'NavigateToPose Action Server 未就绪，'
                '请确认 Nav2 已启动'
            )

        # 构造目标位姿
        goal_msg = NavigateToPose.Goal()
        pose_stamped = PoseStamped()
        pose_stamped.header.frame_id = frame_id
        pose_stamped.header.stamp = self._node.get_clock().now().to_msg()
        pose_stamped.pose.position.x = x
        pose_stamped.pose.position.y = y

        q = self.yaw_to_quaternion(yaw)
        pose_stamped.pose.orientation.z = q['z']
        pose_stamped.pose.orientation.w = q['w']

        goal_msg.pose = pose_stamped

        self._task_id_counter += 1
        task_id = f'NAV_{self._task_id_counter:03d}'
        self._current_task_id = task_id
        self._task_status = 'nav_running'

        # 发送目标（非阻塞）
        send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self._nav_feedback_callback
        )
        send_goal_future.add_done_callback(
            lambda f: self._goal_response_callback(f, task_id)
        )

        self._logger.info(
            f'[{task_id}] 导航目标已发送: x={x:.2f}, y={y:.2f}, yaw={yaw:.2f}'
        )

        return {'task_id': task_id, 'status': 'navigating'}

    def _goal_response_callback(self, future, task_id: str):
        """处理 Action Server 对目标请求的响应。"""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self._logger.warning(f'[{task_id}] 导航目标被拒绝')
            self._task_status = 'failed'
            return

        self._current_goal_handle = goal_handle
        self._logger.info(f'[{task_id}] 导航目标已接受')

        # 等待结果
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(
            lambda f: self._nav_result_callback(f, task_id)
        )

    def _nav_feedback_callback(self, feedback_msg):
        """导航反馈回调：记录剩余距离。

        RDK S100 上 debug 级别日志默认不输出，无性能问题，
        故直接记录即可，无需节流。
        """
        dist = feedback_msg.feedback.distance_remaining
        self._logger.debug(f'导航剩余距离: {dist:.2f} m')

    def _nav_result_callback(self, future, task_id: str):
        """导航完成回调。"""
        result = future.result()
        if result.status == GoalStatus.STATUS_SUCCEEDED:
            self._logger.info(f'[{task_id}] 导航成功')
            self._task_status = 'idle'
        else:
            self._logger.warning(f'[{task_id}] 导航失败，状态码: {result.status}')
            self._task_status = 'failed'

    def stop_navigation(self) -> dict:
        """取消当前导航目标。

        Returns:
            {'message': str}
        """
        if self._current_goal_handle is not None:
            self._logger.info('取消导航目标')
            self._action_client.cancel_goal_async(self._current_goal_handle)
            self._task_status = 'idle'
            return {'message': 'navigation cancelled'}
        else:
            return {'message': 'no active navigation to cancel'}

    # ==================== 状态 ====================

    def get_status(self) -> dict:
        """获取机器人当前状态快照。

        Returns:
            {
                'robot_pose': {'x':.., 'y':.., 'yaw':..} | None,
                'battery': float | None,
                'task_status': str,
                'current_task_id': str | None
            }
        """
        pose_data = None
        if self._current_pose is not None:
            p = self._current_pose
            # 从四元数反算 yaw
            siny_cosp = 2.0 * (p.orientation.w * p.orientation.z)
            cosy_cosp = 1.0 - 2.0 * (p.orientation.z * p.orientation.z)
            yaw = math.atan2(siny_cosp, cosy_cosp)
            pose_data = {
                'x': p.position.x,
                'y': p.position.y,
                'yaw': yaw,
            }

        return {
            'robot_pose': pose_data,
            'battery': self._battery,
            'task_status': self._task_status,
            'current_task_id': self._current_task_id,
        }
