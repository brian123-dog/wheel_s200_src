"""huangshan_task_api.launch.py —— Huangshan Task API 启动文件。

可接受参数:
  waypoints_file:=/path/to/waypoints.json  航点配置文件
  http_port:=6080                          HTTP 端口
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_share = get_package_share_directory('huangshan_task_api')
    params_file = os.path.join(pkg_share, 'config', 'params.yaml')

    waypoints_file = LaunchConfiguration('waypoints_file', default='')
    http_port = LaunchConfiguration('http_port', default='6080')

    node = Node(
        package='huangshan_task_api',
        executable='huangshan_task_api_node',
        name='huangshan_task_api',
        output='screen',
        parameters=[params_file, {
            'waypoints_file': waypoints_file,
            'http_port': http_port,
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument('waypoints_file', default_value=''),
        DeclareLaunchArgument('http_port', default_value='6080'),
        node,
    ])
