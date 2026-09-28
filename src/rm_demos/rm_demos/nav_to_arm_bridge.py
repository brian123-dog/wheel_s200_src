#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
nav_to_arm_bridge.py —— "导航完成 → 机械臂执行" 桥接节点（样子货）。

功能说明：
  本节点为概念示意节点，用于在 ROS2 节点拓扑图中清晰地表达
  "导航到目标点 → 触发机械臂执行任务" 的信息流。

  核心逻辑：
    1. 订阅 /navigate_to_pose 的 action 状态话题，监听导航是否成功
    2. 导航成功后，作为 Service Client 调用 /arm_task_service_node
       提供的 /rm_group/arm_task_execute_detailed 服务
    3. 在终端打印日志，方便演示和理解数据流

  图中连线效果：
    /navigate_to_pose ─(goal_status topic)─▶ /nav_to_arm_bridge
                                              │
                                              └──(ArmTaskExecute service)──▶ /arm_task_service_node

使用方式：
  ros2 run rm_demos nav_to_arm_bridge
"""

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup

# 导航 action 状态消息类型
from action_msgs.msg import GoalStatusArray

# 标准消息类型
from std_msgs.msg import Int32

# 机械臂任务执行服务类型
from base_interfaces.srv import ArmTaskExecute


class NavToArmBridge(Node):
    """
    导航到机械臂桥接节点。

    职责：
      - 监听 NavigateToPose 的 action 状态
      - 一旦检测到导航成功（status == 4），调用机械臂服务
    """

    def __init__(self):
        super().__init__('nav_to_arm_bridge')

        # 使用可重入回调组，避免 service 调用阻塞
        self._cb_group = ReentrantCallbackGroup()

        # ---- 订阅导航 Action 状态（Topic 连线：/navigate_to_pose → /nav_to_arm_bridge） ----
        self._nav_status_sub = self.create_subscription(
            GoalStatusArray,
            '/navigate_to_pose/_action/status',
            self._nav_status_callback,
            10,
            callback_group=self._cb_group,
        )
        self.get_logger().info('已订阅导航状态话题: /navigate_to_pose/_action/status')

        # ---- 订阅机械臂状态话题（Topic 连线：/arm_task_service_node → /nav_to_arm_bridge） ----
        # 作用：使节点图中出现 /arm_task_service_node 指向 /nav_to_arm_bridge 的连线
        # /rm_group/arm_status 是 arm_task_service_node 发布的机械臂状态话题
        self._arm_status_sub = self.create_subscription(
            Int32,
            '/rm_group/arm_status',
            self._arm_status_callback,
            10,
            callback_group=self._cb_group,
        )
        self.get_logger().info('已订阅机械臂状态话题: /rm_group/cup_delete（仅用于拓扑连线）')

        # ---- 机械臂服务客户端 ----
        self._arm_cli = self.create_client(
            ArmTaskExecute,
            '/rm_group/arm_task_execute_detailed',
            callback_group=self._cb_group,
        )
        self.get_logger().info('等待机械臂服务 /rm_group/arm_task_execute_detailed ...')

        # 防重入：避免同一次导航成功触发多次机械臂调用
        self._last_triggered_goal_id = None

    # ------------------------------------------------------------------
    # 机械臂状态回调（样子货，仅用于建立节点图连线）
    # ------------------------------------------------------------------
    def _arm_status_callback(self, msg: Int32):
        """接收 arm_task_service_node 发布的 /rm_group/cup_delete 消息。
        此回调不执行业务逻辑，仅用于在节点图中维持话题连线。"""
        pass

    # ------------------------------------------------------------------
    # 导航状态回调
    # ------------------------------------------------------------------
    def _nav_status_callback(self, msg: GoalStatusArray):
        """
        解析导航 action 的状态列表。

        当检测到任一 Goal 的状态为 STATUS_SUCCEEDED (4) 时，
        触发机械臂服务调用。
        """
        for status_msg in msg.status_list:
            goal_id_str = str(status_msg.goal_info.goal_id.uuid)

            # 只处理成功状态，且防止重复触发
            if status_msg.status == 4:  # 4 = STATUS_SUCCEEDED
                if goal_id_str != self._last_triggered_goal_id:
                    self._last_triggered_goal_id = goal_id_str
                    self.get_logger().info(
                        f'✅ 检测到导航成功 (goal_id={goal_id_str[:8]}...)，'
                        f'即将触发机械臂任务'
                    )
                    self._call_arm_service(waypoint_index=1)

    # ------------------------------------------------------------------
    # 机械臂服务调用
    # ------------------------------------------------------------------
    def _call_arm_service(self, waypoint_index: int = 1):
        """
        向机械臂任务服务节点发送请求。

        Args:
            waypoint_index: 航点编号，传递给机械臂服务
        """
        if not self._arm_cli.wait_for_service(timeout_sec=2.0):
            self.get_logger().warn(
                '⚠️ 机械臂服务 /rm_group/arm_task_execute_detailed 未就绪，跳过本次调用'
            )
            return

        # 构造请求
        req = ArmTaskExecute.Request()
        req.waypoint_index = waypoint_index
        req.run_full_task = True

        self.get_logger().info(
            f'📤 发送机械臂任务请求: waypoint_index={waypoint_index}'
        )

        # 异步调用，不阻塞回调
        future = self._arm_cli.call_async(req)
        future.add_done_callback(self._arm_response_callback)

    # ------------------------------------------------------------------
    # 机械臂服务响应回调
    # ------------------------------------------------------------------
    def _arm_response_callback(self, future):
        """
        处理机械臂服务的响应。
        """
        try:
            response = future.result()
            if response.success:
                self.get_logger().info(
                    f'✅ 机械臂任务执行成功! '
                    f'task_id={response.task_id}, '
                    f'message="{response.message}"'
                )
            else:
                self.get_logger().warn(
                    f'❌ 机械臂任务执行失败: '
                    f'failed_stage={response.failed_stage}, '
                    f'message="{response.message}"'
                )
        except Exception as e:
            self.get_logger().error(f'机械臂服务调用异常: {e}')


# =========================================================================
# 入口
# =========================================================================
def main(args=None):
    rclpy.init(args=args)
    node = NavToArmBridge()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('节点被用户中断')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
