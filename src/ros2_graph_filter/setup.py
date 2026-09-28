#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ros2_graph_filter 包的 setup.py 文件。
用于 ament_python 构建系统。
"""

from setuptools import find_packages, setup

package_name = 'ros2_graph_filter'

setup(
    name=package_name,
    version='0.1.0',
    # 自动查找 ros2_graph_filter 目录下的 Python 包
    packages=find_packages(exclude=['test']),
    # 需要与包名同名的空标记文件，ament_python 要求
    data_files=[
        # 安装 resource 标记文件到 share 目录
        ('share/' + package_name, ['resource/ros2_graph_filter']),
        # 安装 launch 文件
        ('share/' + package_name + '/launch', ['launch/graph_filter.launch.py']),
        # 安装配置文件
        ('share/' + package_name + '/config', ['config/filter.yaml']),
        # 安装 package.xml
        ('share/ament_index/resource_index/packages',
            ['resource/ros2_graph_filter']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@example.com',
    description='ROS2 节点拓扑图生成工具，支持 YAML 过滤与聚合规则',
    license='Apache-2.0',
    tests_require=['pytest'],
    # 注册可执行入口点：命令行 ros2_graph_filter
    entry_points={
        'console_scripts': [
            'ros2_graph_filter = ros2_graph_filter.graph_filter_node:main',
        ],
    },
)
