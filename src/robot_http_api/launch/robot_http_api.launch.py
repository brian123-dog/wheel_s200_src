"""robot_http_api.launch.py —— FastAPI 网关启动文件。"""

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_share = get_package_share_directory('robot_http_api')
    params_file = os.path.join(pkg_share, 'config', 'params.yaml')

    gateway_node = Node(
        package='robot_http_api',
        executable='gateway_node',
        name='robot_http_api',
        output='screen',
        parameters=[params_file],
    )

    return LaunchDescription([gateway_node])
