"""all.launch.py —— 一键启动所有节点（底盘 + 雷达 + 导航 + API + 可选模块）。

用法:
  ros2 launch huangshan_task_api all.launch.py

  可选参数:
    map:=/path/to/map.yaml          指定地图文件
    use_api:=true                   启动 HTTP API (默认 true)
    use_inspection:=true            启动巡检节点 (默认 true)
    use_qr_detector:=false           启动二维码检测节点 (默认关)
    use_meter_reader:=false          启动仪表读取节点 (默认关)
    use_arm:=false                  启动机械臂 + 手眼标定 (默认关)
    use_waypoint_tools:=false       启动路径点记录工具 (默认关)
    use_rm_bringup:=false           启动睿尔曼机械臂驱动 (默认关)

参照: robot_http_api/launch/bringup_all.launch.py
"""

import os
import yaml
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node


def load_yaml(file_path: str) -> dict:
    with open(file_path, 'r') as f:
        return yaml.safe_load(f)


def generate_launch_description():
    # ====== 可选参数 ======
    use_api = LaunchConfiguration('use_api', default='true')
    use_inspection = LaunchConfiguration('use_inspection', default='true')
    use_qr_detector = LaunchConfiguration('use_qr_detector', default='false')
    use_meter_reader = LaunchConfiguration('use_meter_reader', default='false')
    use_arm = LaunchConfiguration('use_arm', default='false')
    use_rm_bringup = LaunchConfiguration('use_rm_bringup', default='false')
    use_waypoint_tools = LaunchConfiguration('use_waypoint_tools', default='false')
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    waypoints_file = LaunchConfiguration('waypoints_file', default='')

    # ====== 路径 ======
    turn_on_dir = get_package_share_directory('turn_on_wheeltec_robot')
    turn_on_launch = os.path.join(turn_on_dir, 'launch')

    nav2_dir = get_package_share_directory('wheeltec_nav2')
    nav2_launch = os.path.join(nav2_dir, 'launch')

    api_dir = get_package_share_directory('huangshan_task_api')
    api_launch = os.path.join(api_dir, 'launch')

    # 地图 + 导航参数
    cfg_params = load_yaml(
        os.path.join(turn_on_dir, 'config', 'wheeltec_param.yaml'))
    car_mode = cfg_params.get('car_mode', '0')
    map_dir = os.path.join(nav2_dir, 'map')
    param_dir = os.path.join(nav2_dir, 'param', 'wheeltec_params')
    print(f'car_mode: {car_mode}, map_dir: {map_dir}, param_dir: {param_dir}')

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
        DeclareLaunchArgument('use_qr_detector', default_value='false'),
        DeclareLaunchArgument('use_meter_reader', default_value='false'),
        DeclareLaunchArgument('use_arm', default_value='false'),
        DeclareLaunchArgument('use_rm_bringup', default_value='false'),
        DeclareLaunchArgument('use_waypoint_tools', default_value='false'),
        DeclareLaunchArgument('waypoints_file', default_value=''),

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

        # ====== 4. Huangshan Task API 服务 (可选, 默认开) ======
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [api_launch, '/huangshan_task_api.launch.py']),
            launch_arguments={
                'waypoints_file': waypoints_file,
            }.items(),
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

        # ====== 6. 二维码检测 (可选, 默认关) ======
        Node(
            package='qr_detector',
            executable='qr_detector_node',
            name='qr_detector',
            output='screen',
            condition=IfCondition(use_qr_detector),
        ),

        # ====== 7. 仪表读取 (可选, 默认关) ======
        Node(
            package='meter_reader',
            executable='meter_reader_node',
            name='meter_reader',
            output='screen',
            condition=IfCondition(use_meter_reader),
        ),

        # ====== 8. 睿尔曼机械臂驱动 (可选, 默认关) ======
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [get_package_share_directory('rm_bringup'),
                 '/launch', '/rm_75_6fb.launch.py']),
            condition=IfCondition(use_rm_bringup),
        ),

        # ====== 9. 机械臂 Action Server (可选, 默认关) ======
        Node(
            package='multi_action_node',
            executable='nav_arm_action_server',
            name='nav_arm_action_server',
            output='screen',
            condition=IfCondition(use_arm),
        ),

        # ====== 10. 路径点记录 (可选, 默认关) ======
        Node(
            package='waypoint_action_publisher',
            executable='position_recorder',
            name='position_recorder',
            output='screen',
            condition=IfCondition(use_waypoint_tools),
        ),
    ])
