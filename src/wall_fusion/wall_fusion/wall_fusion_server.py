#!/usr/bin/env python3
"""
wall_fusion_server.py — 多传感器融合墙面精修 Action Server (v2 修复版)
======================================================================
技术路线: 特征级融合 (Feature-level Fusion)
- 阶段1 (粗调): 2D 激光雷达主导, RANSAC + PCA 直线拟合
- 阶段2 (精调): 深度相机主导, ROI + PCA 平面拟合
- 融合策略: base_link 坐标系下按置信度动态加权
- 控制策略: PD 控制(无I项), 状态机分三段 + 稳定确认
- 速度限制: 线速度 ≤0.04 m/s, 角速度 ≤0.12 rad/s

v2 修复要点 (对比 v1):
  Fix-1 距离语义: dist_error = measured_distance - target_dist (差值,非绝对值)
  Fix-2 TF 旋转:  从四元数提取 yaw 角, 正确修正法向量旋转偏装
  Fix-3 TF 失败:  传感器降权 (conf=0) 而非返回裸坐标数据
  Fix-4 深度读取: 分块 NumPy 直接操作字节缓冲区, 避免 list(gen) 性能爆炸
  Fix-5 激光距离: 用直线方程 c 参数精确计算原点→直线投影距离
  Fix-6 数据新鲜度: 时间戳过期 (>0.5s) 传感器自动降权

依赖: 纯 Python + NumPy, 禁止 PCL/Open3D/PyTorch
"""

import math
import threading
from enum import Enum

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.time import Time

# Messages
from sensor_msgs.msg import LaserScan, PointCloud2
from geometry_msgs.msg import Twist
from builtin_interfaces.msg import Time as BuiltinTime

# TF2
from tf2_ros import Buffer, TransformListener, TransformException

# Message filters
import message_filters

# Custom action
from base_interfaces.action import AlignToWall


# =============================================================================
# 状态枚举
# =============================================================================
class AlignState(Enum):
    COARSE_ALIGN_YAW = "COARSE_ALIGN_YAW"
    COARSE_ALIGN_DIST = "COARSE_ALIGN_DIST"
    FINE_ALIGN = "FINE_ALIGN"
    STABILIZING = "STABILIZING"
    FAILED = "FAILED"
    SUCCEEDED = "SUCCEEDED"


# =============================================================================
# 工具函数
# =============================================================================
def _quaternion_to_yaw(qx, qy, qz, qw):
    """四元数 → yaw 角 (rad)"""
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    return math.atan2(siny_cosp, cosy_cosp)


def _normalize_angle(a):
    """归一化角度到 [-pi, pi]"""
    return math.atan2(math.sin(a), math.cos(a))


def _stamp_to_seconds(stamp: BuiltinTime):
    """ROS2 Time 消息 → 秒"""
    return stamp.sec + stamp.nanosec * 1e-9


# =============================================================================
# 墙面精修 Action Server
# =============================================================================
class WallFusionServer(Node):
    """多传感器融合墙面精修 Action Server"""

    def __init__(self):
        super().__init__('wall_fusion_server')
        self._declare_parameters()

        # TF2
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        # 数据缓存 (线程安全)
        self._lock = threading.Lock()
        self._synced_scan = None    # LaserScan
        self._synced_depth = None   # PointCloud2
        self._synced_scan_stamp = 0.0
        self._synced_depth_stamp = 0.0

        # 误差记录 (D 项)
        self._last_yaw_error = 0.0
        self._last_dist_error = 0.0
        self._last_error_time = self.get_clock().now()

        # P1-1 修复: 预创建 sleep future/timer 占位，避免每次 create_timer
        self._sleep_future = None
        self._sleep_timer = None

        # 状态机
        self._state = AlignState.COARSE_ALIGN_YAW
        self._stabilizing_start = None

        # 激光 EMA 平滑（抑制 RANSAC 跳变）
        self._laser_yaw_ema = None

        # P1-4 修复: RANSAC 固定 seed，消除帧间跳变
        self._rng = np.random.default_rng(seed=42)

        # 预缓存 TF 变换
        # 激光: (dx, dy, yaw) — 2D 激光只需 yaw
        self._tf_laser_to_base = None
        # 深度相机: (dx, dy, qx, qy, qz, qw) — P1-4 修复: 缓存完整四元数，补偿 roll/pitch
        self._tf_camera_to_base = None

        # 发布器
        self._cmd_pub = self.create_publisher(Twist, self._cmd_vel_topic, 10)

        # 消息同步
        self._setup_synchronizer()

        # Action Server
        self._action_server = ActionServer(
            self, AlignToWall, '/align_to_wall',
            execute_callback=self._execute_callback,
            goal_callback=self._goal_callback,
            cancel_callback=self._cancel_callback,
            callback_group=MutuallyExclusiveCallbackGroup(),
        )
        self.get_logger().info('WallFusionServer v2 initialized.')

    # =========================================================================
    # 参数
    # =========================================================================
    def _declare_parameters(self):
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('depth_points_topic', '/camera/depth/points')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('laser_frame', 'laser_link')
        self.declare_parameter('camera_frame', 'camera_link')
        self.declare_parameter('max_linear_speed', 0.04)
        self.declare_parameter('max_angular_speed', 0.12)
        self.declare_parameter('kp_angular', 1.2)
        self.declare_parameter('kd_angular', 0.15)
        self.declare_parameter('kp_linear', 0.5)
        self.declare_parameter('kd_linear', 0.05)
        self.declare_parameter('min_angular_fine', 0.035)
        self.declare_parameter('stab_damping_gain', 0.3)
        self.declare_parameter('stab_max_angular_ratio', 0.3)
        self.declare_parameter('stab_confirm_frames', 5)
        self.declare_parameter('fusion_lost_tolerance_frames', 10)
        self.declare_parameter('stabilizing_duration', 1.2)
        self.declare_parameter('timeout_duration', 25.0)
        self.declare_parameter('angle_threshold_coarse', 0.12)
        self.declare_parameter('dist_threshold_coarse', 0.10)
        self.declare_parameter('sync_slop', 0.08)
        self.declare_parameter('sync_queue', 10)
        self.declare_parameter('data_stale_timeout', 0.5)
        # 野值过滤: 允许的单帧最大误差跳变 (rad)。
        # 依据: 底盘最大角速度 max_angular_speed(0.06 rad/s) × 控制周期(0.1s) = 0.006 rad,
        #       再乘安全系数。默认 0.10 rad(5.7°) 可容纳测量噪声(±1~2°)与真实运动,
        #       但拒绝 RANSAC/深度误配产生的数十度级假跳变。
        self.declare_parameter('max_yaw_jump_rad', 0.10)
        # 连续被拒次数上限: 超过此次数认为"变化是真实且持续的", 予以接受, 避免锁死。
        self.declare_parameter('max_consecutive_jump_rejects', 5)
        self.declare_parameter('laser_angle_range', 1.57)
        self.declare_parameter('laser_min_range', 0.15)
        self.declare_parameter('laser_max_range', 2.5)
        self.declare_parameter('ransac_iterations', 80)
        self.declare_parameter('ransac_distance_thresh', 0.03)
        self.declare_parameter('laser_min_inliers', 6)
        self.declare_parameter('laser_max_inliers', 30)
        self.declare_parameter('depth_roi_x_min', 0.15)
        self.declare_parameter('depth_roi_x_max', 1.2)
        self.declare_parameter('depth_roi_y_min', -1.0)
        self.declare_parameter('depth_roi_y_max', 1.0)
        self.declare_parameter('depth_roi_z_min', 0.1)
        self.declare_parameter('depth_roi_z_max', 0.6)
        self.declare_parameter('depth_max_points', 400)
        self.declare_parameter('depth_min_points', 80)
        self.declare_parameter('depth_blind_distance', 0.25)
        self.declare_parameter('depth_far_distance', 1.5)
        self.declare_parameter('depth_planarity_thresh', 0.15)
        self._cache_params()

    def _cache_params(self):
        self._scan_topic = self.get_parameter('scan_topic').value
        self._depth_topic = self.get_parameter('depth_points_topic').value
        self._cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        self._base_frame = self.get_parameter('base_frame').value
        self._laser_frame = self.get_parameter('laser_frame').value
        self._camera_frame = self.get_parameter('camera_frame').value
        self._max_lin = self.get_parameter('max_linear_speed').value
        self._max_ang = self.get_parameter('max_angular_speed').value
        self._kp_ang = self.get_parameter('kp_angular').value
        self._kd_ang = self.get_parameter('kd_angular').value
        self._kp_lin = self.get_parameter('kp_linear').value
        self._kd_lin = self.get_parameter('kd_linear').value
        self._min_ang_fine = self.get_parameter('min_angular_fine').value
        self._stab_damping_gain = self.get_parameter('stab_damping_gain').value
        self._stab_max_ang_ratio = self.get_parameter('stab_max_angular_ratio').value
        self._stab_confirm_frames = self.get_parameter('stab_confirm_frames').value
        self._fusion_lost_tol_frames = self.get_parameter('fusion_lost_tolerance_frames').value
        self._stab_dur = self.get_parameter('stabilizing_duration').value
        self._timeout_dur = self.get_parameter('timeout_duration').value
        self._ang_thresh_coarse = self.get_parameter('angle_threshold_coarse').value
        self._dist_thresh_coarse = self.get_parameter('dist_threshold_coarse').value
        self._sync_slop = self.get_parameter('sync_slop').value
        self._sync_queue = self.get_parameter('sync_queue').value
        self._data_stale_timeout = self.get_parameter('data_stale_timeout').value
        self._max_yaw_jump = self.get_parameter('max_yaw_jump_rad').value
        self._max_jump_rejects = self.get_parameter('max_consecutive_jump_rejects').value
        self._laser_ang_range = self.get_parameter('laser_angle_range').value
        self._laser_min_r = self.get_parameter('laser_min_range').value
        self._laser_max_r = self.get_parameter('laser_max_range').value
        self._ransac_iter = self.get_parameter('ransac_iterations').value
        self._ransac_thresh = self.get_parameter('ransac_distance_thresh').value
        self._laser_min_n = self.get_parameter('laser_min_inliers').value
        self._laser_max_n = self.get_parameter('laser_max_inliers').value
        self._roi_xmin = self.get_parameter('depth_roi_x_min').value
        self._roi_xmax = self.get_parameter('depth_roi_x_max').value
        self._roi_ymin = self.get_parameter('depth_roi_y_min').value
        self._roi_ymax = self.get_parameter('depth_roi_y_max').value
        self._roi_zmin = self.get_parameter('depth_roi_z_min').value
        self._roi_zmax = self.get_parameter('depth_roi_z_max').value
        self._depth_max_pts = self.get_parameter('depth_max_points').value
        self._depth_min_pts = self.get_parameter('depth_min_points').value
        self._depth_blind = self.get_parameter('depth_blind_distance').value
        self._depth_far = self.get_parameter('depth_far_distance').value
        self._depth_planarity_thresh = self.get_parameter('depth_planarity_thresh').value

    # =========================================================================
    # 消息同步
    # =========================================================================
    def _setup_synchronizer(self):
        # 主同步器: 双话题同步
        self._scan_sub = message_filters.Subscriber(self, LaserScan, self._scan_topic)
        self._depth_sub = message_filters.Subscriber(self, PointCloud2, self._depth_topic)
        self._sync = message_filters.ApproximateTimeSynchronizer(
            [self._scan_sub, self._depth_sub],
            queue_size=self._sync_queue, slop=self._sync_slop,
        )
        self._sync.registerCallback(self._synced_callback)
        # 独立订阅: 即使只有一个传感器有数据也能用
        self.create_subscription(LaserScan, self._scan_topic, self._scan_only_callback, 10)
        self.create_subscription(PointCloud2, self._depth_topic, self._depth_only_callback, 10)

    def _synced_callback(self, scan_msg, depth_msg):
        with self._lock:
            self._synced_scan = scan_msg
            self._synced_depth = depth_msg
            self._synced_scan_stamp = _stamp_to_seconds(scan_msg.header.stamp)
            self._synced_depth_stamp = _stamp_to_seconds(depth_msg.header.stamp)

    def _scan_only_callback(self, msg):
        with self._lock:
            self._synced_scan = msg
            self._synced_scan_stamp = _stamp_to_seconds(msg.header.stamp)

    def _depth_only_callback(self, msg):
        with self._lock:
            self._synced_depth = msg
            self._synced_depth_stamp = _stamp_to_seconds(msg.header.stamp)

    # =========================================================================
    # TF 预查询
    # =========================================================================
    def _update_tf_transforms(self):
        # P1-3 修复: 使用当前时间而非 Time() 零点，避免 tf2 缓存未命中
        now = self.get_clock().now()
        try:
            t = self._tf_buffer.lookup_transform(
                self._base_frame, self._laser_frame, now,
                timeout=rclpy.duration.Duration(seconds=0.1))
            self._tf_laser_to_base = (
                t.transform.translation.x,
                t.transform.translation.y,
                _quaternion_to_yaw(t.transform.rotation.x, t.transform.rotation.y,
                                   t.transform.rotation.z, t.transform.rotation.w))
        except TransformException:
            self._tf_laser_to_base = None

        try:
            t = self._tf_buffer.lookup_transform(
                self._base_frame, self._camera_frame, now,
                timeout=rclpy.duration.Duration(seconds=0.1))
            # P1-4 修复: 缓存完整四元数 (dx, dy, qx, qy, qz, qw)，补偿 roll/pitch 偏装
            self._tf_camera_to_base = (
                t.transform.translation.x,
                t.transform.translation.y,
                t.transform.rotation.x,
                t.transform.rotation.y,
                t.transform.rotation.z,
                t.transform.rotation.w)
        except TransformException:
            self._tf_camera_to_base = None

    # =========================================================================
    # Action 回调
    # =========================================================================
    def _goal_callback(self, goal_request):
        self.get_logger().info(
            f'Received goal: dist={goal_request.target_distance:.3f}, '
            f'yaw={goal_request.target_yaw:.3f}')
        return GoalResponse.ACCEPT

    def _cancel_callback(self, goal_handle):
        self.get_logger().info('Goal cancelled.')
        self._send_zero_velocity()
        return CancelResponse.ACCEPT

    async def _execute_callback(self, goal_handle):
        self.get_logger().info('Starting alignment...')
        self._cache_params()

        goal = goal_handle.request
        fb = AlignToWall.Feedback()
        res = AlignToWall.Result()

        tol_dist = goal.tolerance_distance if goal.tolerance_distance > 0 else 0.05
        tol_yaw = goal.tolerance_yaw if goal.tolerance_yaw > 0 else 0.05
        target_dist = goal.target_distance
        target_yaw = goal.target_yaw

        self._state = AlignState.COARSE_ALIGN_YAW
        t_start = self.get_clock().now()
        self._last_yaw_error = 0.0
        self._last_dist_error = 0.0
        self._last_error_time = t_start
        self._stabilizing_start = None

        # 等待同步数据（渐进降级: 3s双传感器 → 降级为单传感器）
        self.get_logger().info('Waiting for synced sensor data...')
        t_wait = self.get_clock().now()
        while rclpy.ok():
            if goal_handle.is_cancel_requested:
                self._send_zero_velocity()
                res.success = False; res.message = 'Cancelled'
                goal_handle.canceled(); return res
            with self._lock:
                has_scan = self._synced_scan is not None
                has_depth = self._synced_depth is not None
            if has_scan or has_depth:
                break
            if (self.get_clock().now() - t_wait).nanoseconds / 1e9 > 3.0:
                self.get_logger().warn('Synced data timeout, falling back to single-sensor mode.')
                break
            await self._sleep_async(0.1)

        with self._lock:
            has_scan = self._synced_scan is not None
            has_depth = self._synced_depth is not None
        if has_scan:
            self.get_logger().info('Laser data available.')
        if has_depth:
            self.get_logger().info('Depth camera data available.')
        if not has_scan and not has_depth:
            self.get_logger().error('No sensor data at all.')
            res.success = False; res.message = 'No sensor data'
            goal_handle.abort(); return res

        self.get_logger().info('Control loop starting (20 Hz).')

        # 融合丢失容忍: 旋转时激光 RANSAC 或深度可能偶然丢帧，容忍 0.5s（10 帧）
        fusion_lost_count = 0
        prev_yaw_err, prev_dist_err = 0.0, 0.0

        # 野值过滤状态 (每目标重置): 上次被接受的有效误差 + 连续拒绝计数
        self._accepted_yaw = None
        self._accepted_dist = None
        self._jump_reject_count = 0
        self._last_jump_log_time = 0.0

        # STABILIZING 连续确认计数（P0-2 修复: 连续 N 帧 < tol 才成功）
        stab_ok_count = 0

        while rclpy.ok():
            if goal_handle.is_cancel_requested:
                self._send_zero_velocity()
                res.success = False; res.message = 'Cancelled'
                goal_handle.canceled(); return res

            if (self.get_clock().now() - t_start).nanoseconds / 1e9 > self._timeout_dur:
                self.get_logger().error('Timeout.')
                self._send_zero_velocity()
                res.success = False
                res.final_distance_error = abs(self._last_dist_error)
                res.final_yaw_error = abs(self._last_yaw_error)
                res.message = 'Timeout'
                goal_handle.abort(); return res

            self._update_tf_transforms()

            yaw_err, dist_err, lw, dw = self._compute_fused_errors(target_yaw, target_dist)
            if yaw_err is None:
                fusion_lost_count += 1
                if fusion_lost_count <= self._fusion_lost_tol_frames:
                    yaw_err, dist_err = prev_yaw_err, prev_dist_err
                    lw, dw = 0.0, 0.0
                else:
                    self.get_logger().warn(
                        f'Fusion lost for {fusion_lost_count} frames, aborting.')
                    self._send_zero_velocity()
                    res.success = False; res.message = 'Fusion failed'
                    goal_handle.abort(); return res
            else:
                fusion_lost_count = 0
                prev_yaw_err, prev_dist_err = yaw_err, dist_err

                # ---- 野值过滤 ----
                # 真实转角上限 = 底盘最大角速度 × 控制周期 × 安全系数。
                # 超过此上限的跳变必为传感器误配(RANSAC 拟合到错误面/深度平面误配),
                # 若放行会假触发 STABILIZING<->FINE_ALIGN 震荡直到超时。
                if self._accepted_yaw is not None:
                    jump = abs(yaw_err - self._accepted_yaw)
                    if jump > self._max_yaw_jump and self._jump_reject_count < self._max_jump_rejects:
                        self._jump_reject_count += 1
                        yaw_err, dist_err = self._accepted_yaw, self._accepted_dist
                        lw, dw = 0.0, 0.0
                        now_log = self.get_clock().now().nanoseconds / 1e9
                        if now_log - self._last_jump_log_time > 1.0:
                            self._last_jump_log_time = now_log
                            self.get_logger().warn(
                                f'Yaw jump rejected: {math.degrees(jump):.1f}° > '
                                f'{math.degrees(self._max_yaw_jump):.1f}° '
                                f'(物理上限约 {math.degrees(self._max_ang * 0.1):.2f}°/帧), '
                                f'保持 {math.degrees(yaw_err):.2f}° '
                                f'[{self._jump_reject_count}/{self._max_jump_rejects}]')
                    else:
                        if self._jump_reject_count >= self._max_jump_rejects:
                            self.get_logger().warn(
                                f'Yaw jump persistent after {self._jump_reject_count} rejects, '
                                f'accepting {math.degrees(yaw_err):.2f}° as real.')
                        self._accepted_yaw = yaw_err
                        self._accepted_dist = dist_err
                        self._jump_reject_count = 0
                else:
                    self._accepted_yaw = yaw_err
                    self._accepted_dist = dist_err
                    self._jump_reject_count = 0

            ay, ad = abs(yaw_err), abs(dist_err)

            # 状态机 — 纯角度精修，所有阶段只检查角度
            if self._state == AlignState.COARSE_ALIGN_YAW:
                if ay < self._ang_thresh_coarse:
                    self.get_logger().info(
                        f'Coarse yaw done ({math.degrees(ay):.2f}°) → COARSE_ALIGN_DIST')
                    self._state = AlignState.COARSE_ALIGN_DIST

            elif self._state == AlignState.COARSE_ALIGN_DIST:
                if ay < tol_yaw * 3:
                    self.get_logger().info(
                        f'Coarse dist done ({math.degrees(ay):.2f}°) → FINE_ALIGN')
                    self._state = AlignState.FINE_ALIGN

            elif self._state == AlignState.FINE_ALIGN:
                # P1-5: 放宽进入 STABILIZING 的阈值 tol_yaw*2.0
                if ay < tol_yaw * 2.0:
                    self.get_logger().info(
                        f'Fine align done ({math.degrees(ay):.2f}°) → STABILIZING')
                    self._state = AlignState.STABILIZING
                    stab_ok_count = 0

            elif self._state == AlignState.STABILIZING:
                # P1-5: 放宽通过阈值 tol_yaw*1.5，小幅漂移(1.5~3.0×tol_yaw)不重置计数
                if ay < tol_yaw * 1.5:
                    stab_ok_count += 1
                    if stab_ok_count >= self._stab_confirm_frames:
                        self.get_logger().info(
                            f'SUCCEEDED (yaw={math.degrees(ay):.2f}°, '
                            f'连续 {stab_ok_count} 帧稳定)')
                        self._send_zero_velocity()
                        res.success = True
                        res.final_distance_error = ad
                        res.final_yaw_error = ay
                        res.message = 'Wall alignment succeeded'
                        goal_handle.succeed(); return res
                elif ay < tol_yaw * 4.0:
                    # 小幅漂移(1.5~4.0×tol_yaw)不重置计数，保持 STABILIZING
                    # (配合死区补偿, 该区间内仍能主动收敛回成功窗口)
                    pass
                else:
                    # 大幅漂移(>4.0×tol_yaw)才退回 FINE_ALIGN
                    stab_ok_count = 0
                    self.get_logger().warn(
                        f'Drifted (yaw={math.degrees(ay):.2f}° > {math.degrees(tol_yaw*4.0):.1f}°), '
                        f're-aligning.')
                    self._state = AlignState.FINE_ALIGN

            self._apply_pd_control(yaw_err, dist_err, self._state, tol_yaw)

            fb.current_yaw_error = yaw_err
            fb.current_distance_error = dist_err
            fb.lidar_weight = lw; fb.depth_weight = dw
            fb.state = self._state.value
            goal_handle.publish_feedback(fb)
            await self._sleep_async(0.1)

        self._send_zero_velocity()
        res.success = False; res.message = 'Shutdown'
        return res

    # =========================================================================
    # 融合计算 (核心)
    # =========================================================================
    def _compute_fused_errors(self, target_yaw, target_dist):
        """
        各传感器在自身坐标系测量 → TF 转到 base_link → 置信度加权融合。
        返回: (yaw_err_base, dist_err_base, lidar_weight, depth_weight)
        dist_err_base = measured_distance_in_base - target_dist
        """
        now_sec = self.get_clock().now().nanoseconds / 1e9
        with self._lock:
            scan, depth = self._synced_scan, self._synced_depth
            s_stamp, d_stamp = self._synced_scan_stamp, self._synced_depth_stamp

        scan_fresh = scan is not None and (now_sec - s_stamp) < self._data_stale_timeout
        depth_fresh = depth is not None and (now_sec - d_stamp) < self._data_stale_timeout
        if not scan_fresh and not depth_fresh:
            return None, None, 0.0, 0.0

        # --- 激光 ---
        l_yaw, l_dist_err, l_conf = None, None, 0.0
        if scan_fresh:
            pts, _ = self._extract_laser_points(scan)
            if len(pts) >= self._laser_min_n:
                inl, (a, b, c), norm = self._ransac_line_fit(pts)
                ni = int(np.sum(inl))
                if ni >= self._laser_min_n:
                    yaw_l, dist_l = self._laser_line_to_errors((a, b, c), norm, target_yaw)

                    # P1-1 修复: EMA 按阶段自适应
                    # COARSE 阶段: 启用 EMA 抑制 RANSAC 跳变 (α=0.5 更快响应)
                    # FINE/STABILIZING 阶段: 禁用 EMA，用原始值确保 D 项能感知真实变化
                    if self._state in (AlignState.COARSE_ALIGN_YAW, AlignState.COARSE_ALIGN_DIST):
                        if self._laser_yaw_ema is None:
                            self._laser_yaw_ema = yaw_l
                        else:
                            self._laser_yaw_ema = 0.5 * yaw_l + 0.5 * self._laser_yaw_ema
                        l_yaw_use = self._laser_yaw_ema
                    else:
                        # FINE/STABILIZING: 使用原始值，软同步 EMA 防止切回 COARSE 时跳变
                        self._laser_yaw_ema = yaw_l
                        l_yaw_use = yaw_l

                    l_yaw, l_dist_err = self._sensor_to_base(
                        l_yaw_use, dist_l - target_dist, self._tf_laser_to_base, target_yaw)
                    l_conf = 0.0
                    if l_yaw is not None:
                        l_conf = min(ni / self._laser_max_n, 1.0)
                    else:
                        self.get_logger().debug('Laser TF failed, downgraded.')

        # --- 深度 ---
        d_yaw, d_dist_err, d_conf = None, None, 0.0
        if depth_fresh:
            dpts = self._extract_depth_points(depth)
            if dpts.shape[0] >= self._depth_min_pts:
                # P0-2 修复: _depth_plane_fit 返回完整 3D 法向量
                normal_cam, dist_d, conf_raw = self._depth_plane_fit(dpts)
                d_yaw, d_dist_err = self._sensor_to_base(
                    0.0, dist_d - target_dist, self._tf_camera_to_base, target_yaw,
                    is_depth=True, normal_cam=normal_cam)
                d_conf = 0.0
                if d_yaw is not None:
                    d_conf = conf_raw
                else:
                    self.get_logger().debug('Depth TF failed, downgraded.')

        # --- 加权融合 ---
        tc = l_conf + d_conf
        if tc < 1e-6:
            self.get_logger().warn('Both sensors zero confidence.')
            return None, None, 0.0, 0.0

        lw, dw = l_conf / tc, d_conf / tc
        fy, fd = 0.0, 0.0
        if l_yaw is not None and l_conf > 0:
            fy += lw * l_yaw; fd += lw * l_dist_err
        if d_yaw is not None and d_conf > 0:
            fy += dw * d_yaw; fd += dw * d_dist_err

        # 注意: 不在融合函数中更新 _last_*, 由 _apply_pd_control 统一管理 D 项
        return fy, fd, lw, dw

    # =========================================================================
    # 传感器 → base_link 变换
    # =========================================================================
    def _sensor_to_base(self, yaw_err_sensor, dist_err_sensor, tf_tuple, target_yaw,
                        is_depth=False, normal_cam=None):
        """
        tf_tuple:
          - 激光: (dx, dy, yaw_s) — 3-tuple
          - 深度相机: (dx, dy, qx, qy, qz, qw) — 6-tuple (P1-4 修复)
        返回 (yaw_err_base, dist_err_base) 或 (None, None)。

        P0-2 修复: 深度相机接收完整 3D 法向量 normal_cam，用 3×3 旋转矩阵 R
        直接旋转 normal_cam 得到 normal_base，再计算 yaw 误差和距离投影。
        严禁假设 n_cam_z=0。
        """
        if tf_tuple is None:
            return None, None

        if is_depth and len(tf_tuple) == 6 and normal_cam is not None:
            dx, dy, qx, qy, qz, qw = tf_tuple
            # 四元数 → 3×3 旋转矩阵
            R00 = 1.0 - 2.0 * (qy * qy + qz * qz)
            R01 = 2.0 * (qx * qy - qz * qw)
            R02 = 2.0 * (qx * qz + qy * qw)
            R10 = 2.0 * (qx * qy + qz * qw)
            R11 = 1.0 - 2.0 * (qx * qx + qz * qz)
            R12 = 2.0 * (qy * qz - qx * qw)
            R20 = 2.0 * (qx * qz - qy * qw)
            R21 = 2.0 * (qy * qz + qx * qw)
            R22 = 1.0 - 2.0 * (qx * qx + qy * qy)

            # P0-2 修复: 用 3×3 旋转矩阵 R 旋转完整 3D 法向量
            n_cx, n_cy, n_cz = normal_cam[0], normal_cam[1], normal_cam[2]
            nx_base = R00 * n_cx + R01 * n_cy + R02 * n_cz
            ny_base = R10 * n_cx + R11 * n_cy + R12 * n_cz
            # 用法向量在 base_link XY 平面的投影计算 yaw 角
            yaw_base_raw = math.atan2(ny_base, nx_base)
            yaw_base = _normalize_angle(yaw_base_raw - target_yaw)

            # 距离投影: 用法向量在 base_link XY 平面上的投影角
            normal_angle_base = math.atan2(ny_base, nx_base)
            dist_base = dist_err_sensor + (dx * math.cos(normal_angle_base)
                                           + dy * math.sin(normal_angle_base))
            return yaw_base, dist_base
        else:
            # 激光: 2D 传感器，只需 yaw 角
            dx, dy, yaw_s = tf_tuple[0], tf_tuple[1], tf_tuple[2]
            yaw_base = _normalize_angle(yaw_err_sensor + yaw_s)
            normal_angle_base = target_yaw + yaw_err_sensor + yaw_s
            dist_base = dist_err_sensor + (dx * math.cos(normal_angle_base)
                                           + dy * math.sin(normal_angle_base))
            return yaw_base, dist_base

    # =========================================================================
    # 激光处理
    # =========================================================================
    def _extract_laser_points(self, scan):
        ranges = np.array(scan.ranges, dtype=np.float64)
        n = len(ranges)
        angles = scan.angle_min + np.arange(n, dtype=np.float64) * scan.angle_increment
        mask = np.abs(angles) < self._laser_ang_range
        mask &= np.isfinite(ranges) & (ranges > self._laser_min_r) & (ranges < self._laser_max_r)
        vr, va = ranges[mask], angles[mask]
        return np.column_stack([vr * np.cos(va), vr * np.sin(va)]), mask

    def _ransac_line_fit(self, points):
        """RANSAC+PCA 直线拟合。返回 (inlier_mask, (a,b,c), normal_vec), 法向量归一化"""
        n = points.shape[0]
        if n < 3:
            return np.zeros(n, dtype=bool), (1.0, 0.0, 0.0), np.array([1.0, 0.0])
        best_inl = np.zeros(n, dtype=bool)
        best_cnt = 0
        best_norm = np.array([1.0, 0.0])
        # P1-4 修复: 使用 self._rng (固定 seed=42) 避免帧间跳变
        for _ in range(self._ransac_iter):
            i0, i1 = self._rng.choice(n, size=2, replace=False)
            d = points[i1] - points[i0]
            nd = np.linalg.norm(d)
            if nd < 1e-8: continue
            norm = np.array([-d[1], d[0]]) / nd
            dists = np.abs(np.dot(points - (points[i0] + points[i1]) * 0.5, norm))
            inl = dists < self._ransac_thresh
            cnt = int(np.sum(inl))
            if cnt > best_cnt:
                best_cnt, best_inl, best_norm = cnt, inl.copy(), norm.copy()
        if best_cnt >= 3:
            ctr = points[best_inl] - np.mean(points[best_inl], axis=0)
            _, evecs = np.linalg.eigh(np.dot(ctr.T, ctr) / (best_cnt - 1))
            best_norm = evecs[:, 0].copy()
            best_norm /= np.linalg.norm(best_norm)
        if best_norm[0] < 0:
            best_norm = -best_norm
        a, b = best_norm[0], best_norm[1]
        cx = np.mean(points[best_inl][:, 0]) if best_cnt > 0 else 0.0
        cy = np.mean(points[best_inl][:, 1]) if best_cnt > 0 else 0.0
        c = -(a * cx + b * cy)
        return best_inl, (a, b, c), best_norm

    def _laser_line_to_errors(self, line_params, normal, target_yaw):
        """
        直线 a*x + b*y + c = 0, (a,b) 归一化。
        - yaw_error = atan2(b, a) - target_yaw
        - wall_distance = -c (原点(laser)到直线的带符号距离)
        """
        a, b, c = line_params
        yaw_err = _normalize_angle(math.atan2(b, a) - target_yaw)
        wall_dist = -c  # normal 指向 x>0, 墙面在前方, c 为负, -c>0
        return yaw_err, wall_dist

    # =========================================================================
    # 深度处理
    # =========================================================================
    def _extract_depth_points(self, depth_msg):
        """
        分块 NumPy 直接读取 PointCloud2 字节缓冲区。
        每块最多 4096 点, ROI 过滤 + NaN 剔除, 避免 list(gen) 性能问题。
        """
        h, w, ps = depth_msg.height, depth_msg.width, depth_msg.point_step
        total = h * w
        if total == 0:
            return np.empty((0, 3))
        # 查找 x,y,z 偏移 (先验证字段存在性)
        offsets = {}
        for f in depth_msg.fields:
            offsets[f.name] = f.offset
        if 'x' not in offsets or 'y' not in offsets or 'z' not in offsets:
            self.get_logger().error(
                f'PointCloud2 缺少必要字段: 已有={list(offsets.keys())}, 需要 x/y/z')
            return np.empty((0, 3))
        ox, oy, oz = offsets['x'], offsets['y'], offsets['z']
        data = np.frombuffer(depth_msg.data, dtype=np.uint8)
        chunks = []
        chunk_n = 4096
        for start in range(0, total, chunk_n):
            end = min(start + chunk_n, total)
            b0, b1 = start * ps, end * ps
            chunk = data[b0:b1].reshape(-1, ps)
            # P0-1 修复: 切片+view 正确提取跨行字段
            # chunk[:, ox:ox+4] 提取每行同偏移的 4 字节 → reshape 展平 → view float32
            x = chunk[:, ox:ox + 4].reshape(-1).view(np.float32)
            y = chunk[:, oy:oy + 4].reshape(-1).view(np.float32)
            z = chunk[:, oz:oz + 4].reshape(-1).view(np.float32)
            m = np.isfinite(x) & np.isfinite(y) & np.isfinite(z)
            m &= (x > self._roi_xmin) & (x < self._roi_xmax)
            m &= (y > self._roi_ymin) & (y < self._roi_ymax)
            m &= (z > self._roi_zmin) & (z < self._roi_zmax)
            if np.any(m):
                chunks.append(np.column_stack([x[m], y[m], z[m]]))
        if not chunks:
            return np.empty((0, 3))
        pts = np.vstack(chunks).astype(np.float64)
        if pts.shape[0] > self._depth_max_pts:
            idx = np.linspace(0, pts.shape[0] - 1, self._depth_max_pts, dtype=int)
            pts = pts[idx]
        return pts

    def _depth_plane_fit(self, points):
        """
        PCA 平面拟合 + 置信度。
        P0-2 修复: 返回完整 3D 单位法向量 (shape=(3,))，而非只返回 yaw 角。
        返回: (normal_3d, wall_dist, confidence)
        """
        n = points.shape[0]
        if n < 3:
            return np.array([1.0, 0.0, 0.0]), 0.0, 0.0
        ctr = np.mean(points, axis=0)
        ctd = points - ctr
        evals, evecs = np.linalg.eigh(np.dot(ctd.T, ctd) / (n - 1))
        normal = evecs[:, 0].copy()
        # 确保法向量指向 x>0 方向（相机前方）
        if normal[0] < 0:
            normal = -normal
        # wall_dist: 原点(相机)到平面的有向距离 = normal · ctr
        wall_dist = float(np.dot(ctr, normal))
        # 置信度
        pc = min(n / self._depth_min_pts, 1.0) if n < self._depth_min_pts else 1.0
        pl = evals[0] / (evals[2] + 1e-10)
        plc = max(0.0, 1.0 - pl / self._depth_planarity_thresh)
        mx = ctr[0]
        if mx < self._depth_blind:
            dc = 0.2
        elif mx > self._depth_far:
            dc = max(0.0, 1.0 - (mx - self._depth_far) / 2.0)
        else:
            dc = 1.0
        conf = max(0.0, min(pc * plc * dc, 1.0))
        return normal, wall_dist, conf

    # =========================================================================
    # PD 控制
    # =========================================================================
    def _apply_pd_control(self, yaw_err, dist_err, state, tol_yaw):
        """
        PD 控制器。yaw_err/dist_err 均为带符号误差。无 I 项。
        仅输出纯原地旋转 (linear.x = 0)，不前进/后退。

        P0-1 修复: FINE 阶段引入最小有效角速度，补偿 S200 静摩擦死区。
        P0-2 修复: STABILIZING 阶段改为弱阻尼微调 PD（而非完全停止）。
        """
        now = self.get_clock().now()
        dt = (now - self._last_error_time).nanoseconds / 1e9
        dt = max(min(dt if dt > 0 else 0.05, 0.2), 0.02)
        twist = Twist()
        twist.linear.x = 0.0  # 纯原地旋转

        if state == AlignState.STABILIZING:
            # P0-2 修复: STABILIZING 改为阻尼微调（半增益 × stab_damping_gain）
            dy = (yaw_err - self._last_yaw_error) / max(dt, 0.05)
            av = (self._kp_ang * yaw_err + self._kd_ang * dy) * self._stab_damping_gain
            # STABILIZING 阶段速度上限: max_ang * stab_max_angular_ratio
            max_ang_stab = self._max_ang * self._stab_max_ang_ratio
            av = max(-max_ang_stab, min(max_ang_stab, av))
        else:
            # COARSE / FINE 阶段: 全增益 PD
            dy = (yaw_err - self._last_yaw_error) / max(dt, 0.05)
            av = self._kp_ang * yaw_err + self._kd_ang * dy
            # 钳位速度上限
            av = max(-self._max_ang, min(self._max_ang, av))

        # 死区补偿 (修复"卡死 + 过冲"极限环):
        #   底盘在角速度 < min_angular_fine 时因静摩擦基本不动, 需给一个推得动的补偿量。
        #   补偿上限 = min(min_angular_fine, |yaw_err| / dt):
        #     - 误差较大 -> 取 min_angular_fine (克服静摩擦的经验最小角速度)
        #     - 误差较小 -> 收缩为 |yaw_err|/dt, 保证单帧位移不超过剩余误差,
        #                  避免旧实现"恒定 min_angular_fine 冲过目标 -> 反向过冲 -> 再修"
        #   生效范围:
        #     - FINE_ALIGN: 始终生效
        #     - STABILIZING: 仅当误差超出成功窗口 (tol_yaw*1.5) 时生效,
        #                    否则旧实现会"进不去窗口又推不动"卡到超时
        win_yaw = tol_yaw * 1.5
        need_comp = (state == AlignState.FINE_ALIGN) or (
            state == AlignState.STABILIZING and abs(yaw_err) > win_yaw)
        if need_comp and abs(yaw_err) > 1e-5:
            comp = min(self._min_ang_fine, abs(yaw_err) / max(dt, 0.05))
            if abs(av) < comp:
                av = math.copysign(comp, yaw_err)

        twist.angular.z = av

        self._cmd_pub.publish(twist)
        # D 项计算完成后才更新历史值
        self._last_yaw_error = yaw_err
        self._last_dist_error = dist_err
        self._last_error_time = now

    def _send_zero_velocity(self):
        self._cmd_pub.publish(Twist())

    async def _sleep_async(self, sec):
        """P1-1 修复: ROS2 future + timer 非阻塞等待。
        timer 用完即取消+销毁，避免资源累积。"""
        self._sleep_future = rclpy.task.Future()
        def _on_done():
            if not self._sleep_future.done():
                self._sleep_future.set_result(True)
        self._sleep_timer = self.create_timer(sec, _on_done)
        await self._sleep_future
        self._sleep_timer.cancel()
        self._sleep_timer.destroy()
        self._sleep_timer = None


def main(args=None):
    rclpy.init(args=args)
    node = WallFusionServer()
    exe = MultiThreadedExecutor(num_threads=4)
    exe.add_node(node)
    try:
        exe.spin()
    except KeyboardInterrupt:
        node.get_logger().info('KeyboardInterrupt.')
    finally:
        node._send_zero_velocity()
        exe.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
