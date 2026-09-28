#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ros2_graph_filter 启动文件。

功能：
  - 加载 YAML 配置文件
  - 运行 ros2_graph_filter 节点生成 SVG 拓扑图
  - 程序生成 SVG 后自动退出

使用方式：
  ros2 launch ros2_graph_filter graph_filter.launch.py
  ros2 launch ros2_graph_filter graph_filter.launch.py config_path:=/path/to/custom.yaml output_path:=/path/to/output.svg
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration


def launch_setup(context, *args, **kwargs):
    """
    Launch 启动回调函数。

    读取启动参数，构建并执行 ros2_graph_filter 命令行。
    """
    config_path = LaunchConfiguration('config_path').perform(context)
    output_path = LaunchConfiguration('output_path').perform(context)

    # 构建命令行
    cmd = [
        'ros2', 'run', 'ros2_graph_filter', 'ros2_graph_filter',
        config_path, output_path,
    ]

    # 创建进程执行动作
    process = ExecuteProcess(
        cmd=cmd,
        output='screen',
        name='ros2_graph_filter_generator',
    )

    return [process]


def generate_launch_description():
    """
    生成 LaunchDescription。

    声明可配置的启动参数：
      - config_path: YAML 配置文件路径
      - output_path: SVG 输出文件路径
    """
    # 获取包的 share 目录
    pkg_share = get_package_share_directory('ros2_graph_filter')

    # 默认配置文件路径
    default_config = os.path.join(pkg_share, 'config', 'filter.yaml')

    # 默认输出路径（当前工作目录）
    default_output = os.path.join(os.getcwd(), 'ros2_topology.svg')

    # 声明启动参数
    config_arg = DeclareLaunchArgument(
        'config_path',
        default_value=default_config,
        description='YAML 过滤器配置文件路径',
    )

    output_arg = DeclareLaunchArgument(
        'output_path',
        default_value=default_output,
        description='SVG 拓扑图输出路径',
    )

    # 构建 LaunchDescription
    ld = LaunchDescription([
        config_arg,
        output_arg,
        OpaqueFunction(function=launch_setup),
    ])

    return ld
