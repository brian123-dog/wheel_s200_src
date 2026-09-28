#!/usr/bin/env python3
"""
debug_plot.py — 墙面精修调试可视化工具
===========================================
订阅 /align_to_wall/_action/feedback，实时打印并可选保存 CSV。
用法:
  ros2 run wall_fusion debug_plot                    # 仅终端打印
  ros2 run wall_fusion debug_plot --ros-args -p save_csv:=True  # 同时保存 CSV
"""

import os
import csv
import rclpy
from rclpy.node import Node
from base_interfaces.action import AlignToWall


class DebugPlot(Node):
    """订阅 Action Feedback 并实时打印/记录"""

    def __init__(self):
        super().__init__('debug_plot')

        self.declare_parameter('save_csv', False)
        self.declare_parameter('csv_path', '/tmp/wall_fusion_debug.csv')
        self._save_csv = self.get_parameter('save_csv').value
        self._csv_path = self.get_parameter('csv_path').value

        self._csv_file = None
        self._csv_writer = None

        if self._save_csv:
            os.makedirs(os.path.dirname(self._csv_path) or '.', exist_ok=True)
            self._csv_file = open(self._csv_path, 'w', newline='')
            self._csv_writer = csv.writer(self._csv_file)
            self._csv_writer.writerow([
                'timestamp', 'yaw_error', 'dist_error',
                'lidar_weight', 'depth_weight', 'state'
            ])
            self.get_logger().info(f'Saving debug CSV to: {self._csv_path}')

        # 订阅 action feedback
        self._sub = self.create_subscription(
            AlignToWall.Impl.FeedbackMessage,
            '/align_to_wall/_action/feedback',
            self._feedback_callback,
            10
        )

        self.get_logger().info(
            'DebugPlot ready. Waiting for /align_to_wall feedback...'
        )
        self._last_state = None

    def _feedback_callback(self, msg):
        feedback = msg.feedback
        now = self.get_clock().now().nanoseconds / 1e9
        state = feedback.state

        # 状态变化时高亮打印
        if state != self._last_state:
            self.get_logger().info(
                f'━━━ STATE: {self._last_state} → {state} ━━━'
            )
            self._last_state = state

        # 格式化输出
        line = (
            f'[t={now:.3f}] '
            f'yaw_err={feedback.current_yaw_error:+.4f} rad '
            f'dist_err={feedback.current_distance_error:+.4f} m '
            f'lidar_w={feedback.lidar_weight:.2f} '
            f'depth_w={feedback.depth_weight:.2f} '
            f'state={state}'
        )
        self.get_logger().info(line)

        # CSV 写入
        if self._csv_writer:
            self._csv_writer.writerow([
                f'{now:.3f}',
                f'{feedback.current_yaw_error:.6f}',
                f'{feedback.current_distance_error:.6f}',
                f'{feedback.lidar_weight:.4f}',
                f'{feedback.depth_weight:.4f}',
                state,
            ])
            self._csv_file.flush()

    def destroy_node(self):
        if self._csv_file:
            self._csv_file.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = DebugPlot()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
