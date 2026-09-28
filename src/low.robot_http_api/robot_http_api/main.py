"""main.py —— robot_http_api 统一入口。

启动顺序:
  1. rclpy.init()
  2. 创建 GatewayNode (内含 Nav2Interface + FastAPI 线程)
  3. rclpy.spin() 驱动 ROS2 事件循环
"""

import rclpy
from .gateway_node import GatewayNode


def main(args=None):
    rclpy.init(args=args)
    node = GatewayNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
