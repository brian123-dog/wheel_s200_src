#!/usr/bin/env python3
"""
tf_repair.py —— S100 TF 树修复节点
====================================
修复 CrossMaps 语义建图所需的完整 TF 链：
  1. 静态 TF: base_footprint → base_link (z=0.05)
  2. 动态 TF: odom_combined → base_footprint (订阅 /odom_combined 转 TF)
  3. 静态 TF: camera_link → camera_dcw2_depth_optical_frame  (光学旋转)
  4. 静态 TF: camera_link → camera_dcw2_color_optical_frame  (光学旋转)

用法:
  python3 tf_repair.py
  或通过 launch 文件启动
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster


class TfRepairNode(Node):
    def __init__(self):
        super().__init__('tf_repair')

        # ---- 1. 静态 TF: base_footprint → base_link ----
        static_bc = StaticTransformBroadcaster(self)
        t = TransformStamped()
        t.header.frame_id = 'base_footprint'
        t.child_frame_id = 'base_link'
        t.transform.translation.z = 0.05
        t.transform.rotation.w = 1.0
        t.header.stamp = self.get_clock().now().to_msg()
        static_bc.sendTransform(t)
        self.get_logger().info('[tf_repair] Published static TF: base_footprint -> base_link')

        # ---- 2. 静态 TF: camera_link → DCW2 光学帧 ----
        # 奥比中光 optical 帧旋转: rpy = [-π/2, 0, -π/2] → xyzw = [-0.5, 0.5, -0.5, 0.5]
        for child in [
            'camera_dcw2_color_optical_frame',
            'camera_dcw2_depth_optical_frame',
        ]:
            t2 = TransformStamped()
            t2.header.frame_id = 'camera_link'
            t2.child_frame_id = child
            t2.transform.rotation.x = -0.5
            t2.transform.rotation.y = 0.5
            t2.transform.rotation.z = -0.5
            t2.transform.rotation.w = 0.5
            t2.header.stamp = self.get_clock().now().to_msg()
            static_bc.sendTransform(t2)
            self.get_logger().info(f'[tf_repair] Published static TF: camera_link -> {child}')

        # ---- 3. 动态 TF: odom_combined → base_footprint ----
        self._bc = TransformBroadcaster(self)
        self._sub = self.create_subscription(
            Odometry, '/odom_combined', self._odom_cb, 10)
        self.get_logger().info('[tf_repair] Listening on /odom_combined for dynamic TF...')
        self.get_logger().info('[tf_repair] All TFs published. TF repair running.')

    def _odom_cb(self, msg: Odometry):
        t = TransformStamped()
        t.header.stamp = msg.header.stamp
        t.header.frame_id = msg.header.frame_id   # 'odom_combined'
        t.child_frame_id = msg.child_frame_id      # 'base_footprint'
        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = msg.pose.pose.position.z
        t.transform.rotation = msg.pose.pose.orientation
        self._bc.sendTransform(t)


def main():
    rclpy.init()
    node = TfRepairNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
