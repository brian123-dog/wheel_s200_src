#!/usr/bin/env python3
"""
wall_fusion_nav2.launch.py — 集成启动: Nav2 + 墙面精修
=======================================================
启动顺序: 底盘 → 激光雷达 → 相机 → Nav2 → WallFusion
用法:
  ros2 launch wall_fusion wall_fusion_nav2.launch.py
  或追加到 wheeltec_nav2.launch.py 中。
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    pkg_wheeltec_nav = get_package_share_directory('wheeltec_nav2')
    pkg_wall_fusion = get_package_share_directory('wall_fusion')

    # Nav2 参数
    nav_params = os.path.join(
        pkg_wheeltec_nav, 'param', 'wheeltec_params', 'param_S200_diff.yaml'
    )

    # Wall Fusion 参数
    wf_config = os.path.join(pkg_wall_fusion, 'config', 'wall_fusion.yaml')

    # 地图
    map_file = os.path.join(pkg_wheeltec_nav, 'map', 'WHEELTEC.yaml')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),

        # =====================================================================
        # Nav2 Bringup (不含底盘和激光, 假设 robot.service 已启动)
        # =====================================================================
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                pkg_wheeltec_nav, '/launch', '/bringup_launch.py'
            ]),
            launch_arguments={
                'map': map_file,
                'use_sim_time': use_sim_time,
                'params_file': nav_params,
            }.items(),
        ),

        # =====================================================================
        # Wall Fusion Action Server
        # =====================================================================
        Node(
            package='wall_fusion',
            executable='wall_fusion_server',
            name='wall_fusion_server',
            output='screen',
            parameters=[wf_config, {'use_sim_time': use_sim_time}],
        ),
    ])
