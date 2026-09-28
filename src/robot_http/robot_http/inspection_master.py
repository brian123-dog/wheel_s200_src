#!/usr/bin/env python3
"""
inspection_master.py —— 巡检主控节点（robot_http 功能包）
====================================================
职责：统筹导航→墙面精修→二维码扫描→机械臂任务的全流程。
子线程 + callback/Event.wait 模式，禁止 spin_until_future_complete。
"""

import json
import math
import os
import subprocess
import threading
import time
import urllib.request
import urllib.error
import uuid
from enum import Enum

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, Quaternion
from nav2_msgs.action import NavigateToPose
from base_interfaces.action import AlignToWall
from std_srvs.srv import Trigger
from std_msgs.msg import String


class WaypointLoader:
    @staticmethod
    def load(json_path: str) -> list:
        if not os.path.exists(json_path):
            raise FileNotFoundError(f"路径点文件不存在: {json_path}")
        with open(json_path, 'r', encoding='utf-8') as f:
            data = json.load(f)
        waypoints = []
        if "points" in data and isinstance(data["points"], list):
            for i, pt in enumerate(data["points"]):
                pose = pt.get("pose", pt)
                wp = {"point_id": pt.get("point_id", f"P{i:03d}"), "x": pose["x"], "y": pose["y"],
                      "yaw": pose.get("yaw", 0.0), "execute_tasks": pt.get("execute_tasks", False), "tasks": pt.get("tasks", [])}
                waypoints.append(wp)
            return waypoints
        if isinstance(data, list):
            for i, pt in enumerate(data):
                wp = {"point_id": f"P{i:03d}", "x": pt["x"], "y": pt["y"],
                      "yaw": pt.get("yaw", 0.0), "execute_tasks": pt.get("execute_tasks", False), "tasks": pt.get("tasks", [])}
                waypoints.append(wp)
            return waypoints
        if "waypoints" in data and isinstance(data["waypoints"], list):
            for i, pt in enumerate(data["waypoints"]):
                wp = {"point_id": pt.get("name", f"waypoint_{i}"), "x": pt["x"], "y": pt["y"],
                      "yaw": pt.get("yaw", 0.0), "execute_tasks": pt.get("execute_tasks", False), "tasks": pt.get("tasks", [])}
                waypoints.append(wp)
            return waypoints
        if "position" in data:
            pos_data = data["position"]
            if isinstance(pos_data, dict): pos_data = [pos_data]
            for i, pt in enumerate(pos_data):
                wp = {"point_id": f"position_{i + 1}", "x": pt["x"], "y": pt["y"],
                      "yaw": pt.get("yaw", 0.0), "execute_tasks": pt.get("execute_tasks", False), "tasks": pt.get("tasks", [])}
                waypoints.append(wp)
            return waypoints
        raise ValueError(f"无法识别的 JSON 格式: {list(data.keys())}")


class PatrolState(Enum):
    IDLE = "IDLE"
    RUNNING = "RUNNING"
    CANCELLED = "CANCELLED"


class InspectionMaster(Node):
    def __init__(self):
        super().__init__('inspection_master')
        self.declare_parameter('waypoints_file', '/home/sunrise/wheel_S200_src/src/waypoint_action_publisher/src/current_position.json')
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('nav_timeout_sec', 120)
        self.declare_parameter('align_timeout_sec', 30)
        self.declare_parameter('arm_timeout_sec', 30)
        self.declare_parameter('cabinet_timeout_sec', 30)
        self.declare_parameter('qr_timeout_sec', 10)
        self.declare_parameter('web_ip', '192.168.110.100')
        self.declare_parameter('web_port', 5000)
        self.declare_parameter('align_target_distance', 0.30)
        self.declare_parameter('align_target_yaw', 0.0)
        self.declare_parameter('align_tolerance_distance', 0.02)
        self.declare_parameter('align_tolerance_yaw', 0.017)

        self._waypoints_file = self.get_parameter('waypoints_file').value
        self._frame_id = self.get_parameter('frame_id').value
        self._nav_timeout = self.get_parameter('nav_timeout_sec').value
        self._align_timeout = self.get_parameter('align_timeout_sec').value
        self._arm_timeout = self.get_parameter('arm_timeout_sec').value
        self._qr_timeout = self.get_parameter('qr_timeout_sec').value
        self._align_target_dist = self.get_parameter('align_target_distance').value
        self._align_target_yaw = self.get_parameter('align_target_yaw').value
        self._align_tol_dist = self.get_parameter('align_tolerance_distance').value
        self._align_tol_yaw = self.get_parameter('align_tolerance_yaw').value

        self._nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self._align_client = ActionClient(self, AlignToWall, '/align_to_wall')
        # 到站服务：柜体任务 + 机械臂，与 C++ 节点 triggerCabinetTask 行为一致
        self._cabinet_client = self.create_client(Trigger, '/rg_cabinet/run_sequence_from_qr')
        self._arm_client = self.create_client(Trigger, '/rm_group/arm_task_execute')
        self._status_pub = self.create_publisher(String, '/robot_status', 10)
        self._cmd_sub = self.create_subscription(String, '/inspection/start_cmd', self._cmd_callback, 10,
                                                  callback_group=MutuallyExclusiveCallbackGroup())
        self._qr_result = None
        self._qr_event = threading.Event()
        self._qr_sub = self.create_subscription(String, '/qr_detected', self._qr_callback, 10)

        # ---- Web 后端配置 ----
        self._qr_fail_endpoint = f'http://{self.get_parameter("web_ip").get_parameter_value().string_value}:{self.get_parameter("web_port").get_parameter_value().integer_value}/api/robot/qr_scan_failed'
        self._qr_success_endpoint = f'http://{self.get_parameter("web_ip").get_parameter_value().string_value}:{self.get_parameter("web_port").get_parameter_value().integer_value}/api/robot/qr_scan_completed'

        self._state = PatrolState.IDLE
        self._state_lock = threading.Lock()
        self._mission_id = "default"
        self._current_point = ""
        self._last_pub_state = ""
        self._last_pub_point = ""
        self._patrol_thread = None
        self._thread_running = threading.Event()
        self._cancel_timer = None
        self._goal_handle = None

        # ---- 常驻扫码子进程（节点初始化时启动一次，销毁时终止） ----
        self._qr_proc = None
        self._qr_log = None
        try:
            unique_name = f'qr_node_{uuid.uuid4().hex[:8]}'
            os.makedirs('/home/sunrise/log', exist_ok=True)
            log_path = f'/home/sunrise/log/qr_{time.strftime("%Y%m%d_%H%M%S")}.log'
            self._qr_log = open(log_path, 'w')
            self.get_logger().info(f'启动常驻扫码子进程, 日志: {log_path}')
            self._qr_proc = subprocess.Popen(
                ['ros2', 'run', 'qr_detector', 'test_qr_main',
                 '--ros-args', '-r', f'__node:={unique_name}'],
                stdout=self._qr_log, stderr=subprocess.STDOUT,
            )
        except Exception as e:
            self.get_logger().error(f'无法启动常驻扫码子进程: {e}')

        self.get_logger().info('InspectionMaster 已启动')

    # === 状态发布 ===
    def _publish_status(self, state: str, point: str = ""):
        if state == self._last_pub_state and point == self._last_pub_point:
            return
        self._last_pub_state = state
        self._last_pub_point = point
        msg = String()
        msg.data = json.dumps({"state": state, "current_point": point, "mission_id": self._mission_id, "timestamp": time.time()}, ensure_ascii=False)
        self._status_pub.publish(msg)
        self.get_logger().info(f"[ROBOT_STATUS] {msg.data}")

    # === Web 指令 ===
    def _cmd_callback(self, msg: String):
        cmd = msg.data.strip().upper()
        self.get_logger().info(f"收到指令: {cmd}")
        if cmd == "START_INSPECTION" and self._state == PatrolState.IDLE:
            self._start_patrol()
        elif cmd == "STOP_INSPECTION" and self._state == PatrolState.RUNNING:
            self._cancel_patrol()

    def _start_patrol(self):
        if self._patrol_thread and self._patrol_thread.is_alive():
            self.get_logger().warning("巡检线程已在运行，忽略重复启动")
            return
        self._state = PatrolState.RUNNING
        self._thread_running.set()
        self._patrol_thread = threading.Thread(target=self._patrol_loop, daemon=True)
        self._patrol_thread.start()
        self._publish_status("RUNNING")
        self.get_logger().info("========== 巡检任务开始 ==========")

    def _cancel_patrol(self):
        self.get_logger().info("收到取消指令")
        self._state = PatrolState.CANCELLED
        self._thread_running.clear()
        self._publish_status("CANCELLED")
        if self._goal_handle is not None:
            try:
                self._goal_handle.cancel_goal_async()
            except Exception as e:
                self.get_logger().warning(f"取消导航异常: {e}")
            self._goal_handle = None
        try:
            if self._cancel_timer is not None:
                self.destroy_timer(self._cancel_timer)
                self._cancel_timer = None
        except Exception:
            pass
        self._cancel_timer = self.create_timer(2.0, self._reset_to_idle, callback_group=MutuallyExclusiveCallbackGroup())

    def _reset_to_idle(self):
        if self._state == PatrolState.CANCELLED:
            self._state = PatrolState.IDLE
            self._publish_status("IDLE")
            self.get_logger().info("状态已重置为 IDLE")
        try:
            if self._cancel_timer is not None:
                self.destroy_timer(self._cancel_timer)
                self._cancel_timer = None
        except Exception as e:
            self.get_logger().debug(f"销毁timer异常: {e}")

    def _qr_callback(self, msg: String):
        self._qr_result = msg.data
        self._qr_event.set()
        self.get_logger().info(f"收到二维码: {msg.data}")

    # === 主巡检循环（子线程） ===
    def _patrol_loop(self):
        try:
            waypoints = WaypointLoader.load(self._waypoints_file)
        except Exception as e:
            self.get_logger().error(f"加载路径点失败: {e}")
            self._state = PatrolState.IDLE; self._publish_status("IDLE"); return
        if not waypoints:
            self._state = PatrolState.IDLE; self._publish_status("IDLE"); return
        try:
            with open(self._waypoints_file, 'r') as f:
                self._mission_id = json.load(f).get("mission_id", "default")
        except Exception: pass
        total = len(waypoints)
        self.get_logger().info(f"加载 {total} 个路径点, mission_id={self._mission_id}")
        if not self._nav_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error("Nav2 不可用"); self._state = PatrolState.IDLE; self._publish_status("IDLE"); return

        for idx, wp in enumerate(waypoints):
            if not self._thread_running.is_set():
                self.get_logger().info("巡检已取消"); return
            point_id = wp["point_id"]
            self._current_point = point_id
            self._publish_status("RUNNING", point_id)
            self.get_logger().info(f"[{(idx+1)*100//total}%] 导航至 {point_id} ({idx+1}/{total}) execute_tasks={wp['execute_tasks']}")
            if not self._navigate_to(wp):
                self.get_logger().error(f"导航至 {point_id} 失败"); break
            if not self._thread_running.is_set(): return
            if wp["execute_tasks"]:
                self.get_logger().info(f"--- {point_id}: 开始执行到站任务 ---")
                self._call_align_to_wall()
                qr = self._wait_for_qr()
                if qr:
                    self.get_logger().info(f"二维码: {qr}")
                    # 扫码成功 → 通知 Web 后端翻标记，前端获取真实 position.json 值
                    self._send_qr_success_notification(point_id)
                else:
                    self.get_logger().warning(f"二维码扫描失败（超时），上报 Web 告警")
                    self._send_qr_failure_alert(point_id)
                # 步骤 A: 柜体任务（与 C++ 节点 triggerCabinetTask 行为一致）
                self._call_cabinet_task()
                # 步骤 B: 机械臂任务
                self._call_arm_service()
                self.get_logger().info(f"--- {point_id}: 到站任务完成 ---")
        self._state = PatrolState.IDLE; self._publish_status("IDLE")
        self.get_logger().info("========== 巡检任务结束 ==========")

    # === 导航（callback+Event.wait） ===
    def _navigate_to(self, wp: dict) -> bool:
        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = self._frame_id
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = wp["x"]; goal.pose.pose.position.y = wp["y"]; goal.pose.pose.position.z = 0.0
        goal.pose.pose.orientation = self._yaw_to_quat(wp["yaw"])

        accepted_event = threading.Event(); result_event = threading.Event()
        self._goal_handle = None; nav_result = None

        def _on_goal_accepted(future):
            nonlocal accepted_event
            try: self._goal_handle = future.result()
            except Exception as e: self.get_logger().error(f"导航接受回调异常: {e}")
            finally: accepted_event.set()

        send_future = self._nav_client.send_goal_async(goal)
        send_future.add_done_callback(_on_goal_accepted)
        if not accepted_event.wait(timeout=10.0):
            self.get_logger().error("导航目标发送超时"); return False
        if self._goal_handle is None or not self._goal_handle.accepted:
            self.get_logger().error("导航目标被拒绝"); return False

        def _on_result(future):
            nonlocal result_event, nav_result
            try: nav_result = future.result()
            except Exception as e: self.get_logger().error(f"导航结果回调异常: {e}")
            finally: result_event.set()

        result_future = self._goal_handle.get_result_async()
        result_future.add_done_callback(_on_result)
        if not result_event.wait(timeout=self._nav_timeout):
            self.get_logger().error(f"导航结果等待超时 ({self._nav_timeout}s)")
            if self._goal_handle is not None:
                try: self._goal_handle.cancel_goal_async()
                except Exception as e: self.get_logger().warning(f"取消导航异常: {e}")
            self._goal_handle = None; return False
        self._goal_handle = None
        if nav_result is None or nav_result.status != GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().error(f"导航失败, status={nav_result.status if nav_result else 'None'}")
            return False
        self.get_logger().info(f"导航到达 {wp['point_id']} 成功")
        return True

    # === 墙面精修（callback+Event.wait） ===
    def _call_align_to_wall(self) -> bool:
        if not self._align_client.wait_for_server(timeout_sec=3.0):
            self.get_logger().warning("/align_to_wall 不可用，跳过"); return False
        goal = AlignToWall.Goal()
        goal.target_distance = self._align_target_dist; goal.target_yaw = self._align_target_yaw
        goal.tolerance_distance = self._align_tol_dist; goal.tolerance_yaw = self._align_tol_yaw
        goal.laser_frame = "laser_link"; goal.camera_frame = "camera_link"

        accepted_event = threading.Event(); result_event = threading.Event()
        align_handle = None; align_result = None

        def _on_goal_accepted(future):
            nonlocal accepted_event, align_handle
            try: align_handle = future.result()
            except Exception as e: self.get_logger().error(f"精修接受回调异常: {e}")
            finally: accepted_event.set()

        send_future = self._align_client.send_goal_async(goal)
        send_future.add_done_callback(_on_goal_accepted)
        if not accepted_event.wait(timeout=5.0):
            self.get_logger().warning("精修目标发送超时"); return False
        if align_handle is None or not align_handle.accepted:
            self.get_logger().warning("精修目标被拒绝"); return False

        def _on_result(future):
            nonlocal result_event, align_result
            try: align_result = future.result()
            except Exception as e: self.get_logger().error(f"精修结果回调异常: {e}")
            finally: result_event.set()

        result_future = align_handle.get_result_async()
        result_future.add_done_callback(_on_result)
        if not result_event.wait(timeout=self._align_timeout):
            self.get_logger().warning("精修结果等待超时")
            if align_handle is not None:
                try: align_handle.cancel_goal_async()
                except Exception as e: self.get_logger().warning(f"取消精修异常: {e}")
            return False
        if align_result is None or not align_result.result.success:
            self.get_logger().warning("墙面精修失败"); return False
        self.get_logger().info(f"精修完成: dist_err={align_result.result.final_distance_error:.4f}")
        return True

    # === 等二维码（常驻子进程，仅清空 event 等待） ===
    def _wait_for_qr(self) -> str:
        self._qr_result = None
        self._qr_event.clear()
        # 给常驻扫码子进程一点初始化时间（订阅话题、打开相机等）
        time.sleep(0.5)
        self.get_logger().info(f'等待二维码扫描结果（超时 {self._qr_timeout}s）...')
        received = self._qr_event.wait(timeout=self._qr_timeout)
        return self._qr_result if received and self._qr_result else ''

    # === 上报二维码扫描失败告警到 Web 后端 ===
    def _send_qr_failure_alert(self, point_id: str):
        """向 Web 后端 POST 二维码扫描失败消息。"""
        try:
            data = json.dumps({
                "robot_id": "R001",
                "point_id": point_id,
                "message": f"点位 {point_id} 二维码扫描超时或未识别到有效二维码"
            }).encode('utf-8')
            req = urllib.request.Request(
                self._qr_fail_endpoint,
                data=data,
                headers={'Content-Type': 'application/json; charset=utf-8'},
                method='POST',
            )
            with urllib.request.urlopen(req, timeout=5.0) as resp:
                body = resp.read().decode('utf-8')
                self.get_logger().info(f'QR 失败告警已上报 Web: {body[:200]}')
        except urllib.error.URLError as e:
            self.get_logger().warning(f'QR 失败告警上报 Web 失败（网络不可达）: {e}')
        except Exception as e:
            self.get_logger().error(f'QR 失败告警上报异常: {e}')

    # === 上报二维码扫描成功通知到 Web 后端 ===
    def _send_qr_success_notification(self, point_id: str):
        """向 Web 后端 POST 二维码扫描成功消息，触发 QR_SCAN_COMPLETED=True。"""
        try:
            data = json.dumps({
                "robot_id": "R001",
                "point_id": point_id,
                "message": f"点位 {point_id} 二维码扫描成功，位姿数据已就绪"
            }).encode('utf-8')
            req = urllib.request.Request(
                self._qr_success_endpoint,
                data=data,
                headers={'Content-Type': 'application/json; charset=utf-8'},
                method='POST',
            )
            with urllib.request.urlopen(req, timeout=5.0) as resp:
                body = resp.read().decode('utf-8')
                self.get_logger().info(f'QR 成功通知已上报 Web: {body[:200]}')
        except urllib.error.URLError as e:
            self.get_logger().warning(f'QR 成功通知上报 Web 失败（网络不可达）: {e}')
        except Exception as e:
            self.get_logger().error(f'QR 成功通知上报异常: {e}')

    # === 柜体任务（与 C++ 节点 triggerCabinetTask 行为一致） ===
    def _call_cabinet_task(self) -> bool:
        if not self._cabinet_client.wait_for_service(timeout_sec=3.0):
            self.get_logger().warning("柜体任务服务不可用，跳过"); return False
        req = Trigger.Request()
        result_event = threading.Event(); cab_result = None

        def _on_response(future):
            nonlocal result_event, cab_result
            try: cab_result = future.result()
            except Exception as e: self.get_logger().error(f"柜体任务回调异常: {e}")
            finally: result_event.set()

        future = self._cabinet_client.call_async(req)
        future.add_done_callback(_on_response)
        if not result_event.wait(timeout=30.0):
            self.get_logger().error("柜体任务调用超时"); return False
        if cab_result is not None and cab_result.success:
            self.get_logger().info(f"柜体任务完成: {cab_result.message}"); return True
        # TF 树断开属于设计预期（底盘与机械臂独立），降级为 INFO
        msg = cab_result.message if cab_result else '无响应'
        if 'Tf has two or more unconnected trees' in msg:
            self.get_logger().info(f"柜体任务跳过（TF 树断开，设计如此）: {msg[:80]}...")
        else:
            self.get_logger().warning(f"柜体任务失败: {msg}")
        return False

    # === 机械臂（callback+Event.wait） ===
    def _call_arm_service(self) -> bool:
        if not self._arm_client.wait_for_service(timeout_sec=3.0):
            self.get_logger().warning("机械臂服务不可用，跳过"); return False
        req = Trigger.Request()
        result_event = threading.Event(); arm_result = None

        def _on_response(future):
            nonlocal result_event, arm_result
            try: arm_result = future.result()
            except Exception as e: self.get_logger().error(f"机械臂回调异常: {e}")
            finally: result_event.set()

        future = self._arm_client.call_async(req)
        future.add_done_callback(_on_response)
        if not result_event.wait(timeout=self._arm_timeout):
            self.get_logger().error("机械臂调用超时"); return False
        if arm_result is not None and arm_result.success:
            self.get_logger().info(f"机械臂完成: {arm_result.message}"); return True
        self.get_logger().warning(f"机械臂失败: {arm_result.message if arm_result else '无响应'}")
        return False

    @staticmethod
    def _yaw_to_quat(yaw: float) -> Quaternion:
        q = Quaternion(); q.w = math.cos(yaw / 2.0); q.z = math.sin(yaw / 2.0)
        return q


def main(args=None):
    rclpy.init(args=args)
    node = InspectionMaster()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # 终止常驻扫码子进程 + 关闭日志文件
        if node._qr_proc is not None:
            try:
                node.get_logger().info('正在终止常驻扫码子进程...')
                node._qr_proc.terminate()
                node._qr_proc.wait(timeout=5.0)
            except subprocess.TimeoutExpired:
                node._qr_proc.kill()
                node._qr_proc.wait()
            except Exception as e:
                node.get_logger().warning(f'终止扫码子进程异常: {e}')
        if node._qr_log is not None:
            try:
                node._qr_log.close()
            except Exception:
                pass
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
