#!/usr/bin/env python3
"""
align_to_wall_client.py — 墙面精修 Action Client (方案 B 集成用)
===============================================================
独立可执行节点, 在 Nav2 导航到达目标后调用 /align_to_wall 进行精修。
可被 vlm_nav2_interface / waypoint_action_publisher 等上层任务节点调用，
也可独立运行进行测试。

用法:
  ros2 run wall_fusion align_to_wall_client              # 使用默认参数
  ros2 run wall_fusion align_to_wall_client --ros-args \
    -p target_distance:=0.30 -p target_yaw:=0.0
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from base_interfaces.action import AlignToWall


class AlignToWallClient(Node):
    """墙面精修 Action Client"""

    def __init__(self):
        super().__init__('align_to_wall_client')

        self.declare_parameter('target_distance', 0.30)
        self.declare_parameter('target_yaw', 0.0)
        self.declare_parameter('tolerance_distance', 0.02)
        self.declare_parameter('tolerance_yaw', 0.017)
        self.declare_parameter('laser_frame', 'laser_link')
        self.declare_parameter('camera_frame', 'camera_link')

        self._action_client = ActionClient(self, AlignToWall, '/align_to_wall')
        self.get_logger().info('AlignToWall client ready. Waiting for server...')

    def send_goal_sync(self):
        """同步发送 goal 并等待结果"""
        if not self._action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('/align_to_wall server not available!')
            return False

        goal = AlignToWall.Goal()
        goal.target_distance = self.get_parameter('target_distance').value
        goal.target_yaw = self.get_parameter('target_yaw').value
        goal.tolerance_distance = self.get_parameter('tolerance_distance').value
        goal.tolerance_yaw = self.get_parameter('tolerance_yaw').value
        goal.laser_frame = self.get_parameter('laser_frame').value
        goal.camera_frame = self.get_parameter('camera_frame').value

        self.get_logger().info(
            f'Sending AlignToWall goal: '
            f'dist={goal.target_distance:.2f}, yaw={goal.target_yaw:.2f}'
        )

        send_goal_future = self._action_client.send_goal_async(
            goal, feedback_callback=self._feedback_cb
        )
        rclpy.spin_until_future_complete(self, send_goal_future)
        goal_handle = send_goal_future.result()

        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected by server.')
            return False

        self.get_logger().info('Goal accepted. Waiting for result...')
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result = result_future.result().result

        self.get_logger().info(
            f'AlignToWall result: success={result.success}, '
            f'final_dist_err={result.final_distance_error:.4f}m, '
            f'final_yaw_err={result.final_yaw_error:.4f}rad, '
            f'message={result.message}'
        )
        return result.success

    def _feedback_cb(self, feedback_msg):
        fb = feedback_msg.feedback
        self.get_logger().info(
            f'[Feedback] state={fb.state}, '
            f'yaw_err={fb.current_yaw_error:.4f}, '
            f'dist_err={fb.current_distance_error:.4f}, '
            f'lidar_w={fb.lidar_weight:.2f}, depth_w={fb.depth_weight:.2f}'
        )


def main(args=None):
    rclpy.init(args=args)
    node = AlignToWallClient()
    success = node.send_goal_sync()
    node.destroy_node()
    rclpy.shutdown()
    return 0 if success else 1


if __name__ == '__main__':
    exit(main())
