"""robot_http.launch.py —— 极简 HTTP 网关启动文件。"""

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    """生成 LaunchDescription，仅启动 robot_http_gateway 节点。"""
    pkg_share = get_package_share_directory('robot_http')
    params_file = os.path.join(pkg_share, 'config', 'params.yaml')

    gateway_node = Node(
        package='robot_http',
        executable='gateway_node',
        name='robot_http_gateway',
        output='screen',
        parameters=[params_file],
    )

    return LaunchDescription([gateway_node])
