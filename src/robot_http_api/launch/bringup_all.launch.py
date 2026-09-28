"""bringup_all.launch.py —— 一键启动所有节点。

用法:
  ros2 launch robot_http_api bringup_all.launch.py

  可选参数:
    map:=/path/to/map.yaml       指定地图文件
    use_api:=true                 启动 HTTP API (默认 true)
    use_inspection:=true          启动巡检节点
    use_arm:=false                启动机械臂 (默认关)
    use_waypoint_tools:=false     启动路径点记录工具 (默认关)

参照: turtlesim 的 MinimalLaunchDescription 模式
     每个 IncludeLaunchDescription 对应一个功能模块
"""

import os
import yaml
from pathlib import Path
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
    GroupAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node


def load_yaml(file_path: Path) -> dict:
    with open(file_path, 'r') as f:
        return yaml.safe_load(f)


def generate_launch_description():
    # ====== 参数 ======
    use_api = LaunchConfiguration('use_api', default='true')
    use_inspection = LaunchConfiguration('use_inspection', default='true')
    use_arm = LaunchConfiguration('use_arm', default='false')
    use_waypoint_tools = LaunchConfiguration('use_waypoint_tools', default='false')
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    # ====== 路径 ======
    turn_on_dir = get_package_share_directory('turn_on_wheeltec_robot')
    turn_on_launch = os.path.join(turn_on_dir, 'launch')

    nav2_dir = get_package_share_directory('wheeltec_nav2')
    nav2_launch = os.path.join(nav2_dir, 'launch')

    api_dir = get_package_share_directory('robot_http_api')
    api_launch = os.path.join(api_dir, 'launch')

    # 地图 + 导航参数
    cfg_params = load_yaml(
        os.path.join(turn_on_dir, 'config', 'wheeltec_param.yaml'))
    car_mode = cfg_params.get('car_mode', '0')
    map_dir = os.path.join(nav2_dir, 'map')
    param_dir = os.path.join(nav2_dir, 'param', 'wheeltec_params')

    map_file = LaunchConfiguration(
        'map', default=os.path.join(map_dir, 'WHEELTEC.yaml'))
    param_file = LaunchConfiguration(
        'params', default=os.path.join(param_dir, f'param_{car_mode}.yaml'))

    return LaunchDescription([
        # ====== 声明参数 ======
        DeclareLaunchArgument('map', default_value=map_file),
        DeclareLaunchArgument('params', default_value=param_file),
        DeclareLaunchArgument('use_api', default_value='true'),
        DeclareLaunchArgument('use_inspection', default_value='true'),
        DeclareLaunchArgument('use_arm', default_value='false'),
        DeclareLaunchArgument('use_waypoint_tools', default_value='false'),

        LogInfo(msg=f'car_mode: {car_mode}'),

        # ====== 1. 底盘 + 串口 + EKF (必须) ======
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [turn_on_launch, '/turn_on_wheeltec_robot.launch.py']),
        ),

        # ====== 2. 激光雷达 (必须) ======
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [turn_on_launch, '/wheeltec_lidar.launch.py']),
        ),

        # ====== 3. Nav2 导航全家桶 (必须) ======
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [nav2_launch, '/bringup_launch.py']),
            launch_arguments={
                'map': map_file,
                'use_sim_time': use_sim_time,
                'params_file': param_file,
            }.items(),
        ),

        # ====== 4. HTTP API 服务 (可选, 默认开) ======
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [api_launch, '/robot_http_api.launch.py']),
            condition=IfCondition(use_api),
        ),

        # ====== 5. 巡检主控 (可选, 默认开) ======
        Node(
            package='robot_http',
            executable='inspection_master',
            name='inspection_master',
            output='screen',
            condition=IfCondition(use_inspection),
        ),

        # ====== 6. 路径点记录 (可选, 默认关) ======
        Node(
            package='waypoint_action_publisher',
            executable='position_recorder',
            name='position_recorder',
            output='screen',
            condition=IfCondition(use_waypoint_tools),
        ),

        # ====== 7. 机械臂 Action Server (可选, 默认关) ======
        Node(
            package='multi_action_node',
            executable='nav_arm_action_server',
            name='nav_arm_action_server',
            output='screen',
            condition=IfCondition(use_arm),
        ),
    ])
