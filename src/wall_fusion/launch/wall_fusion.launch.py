#!/usr/bin/env python3
"""
wall_fusion.launch.py — 启动墙面精修节点
说明：laser_link 和 camera_link 的 TF 已由 wheeltec_robot_urdf 通过
      robot_state_publisher 发布，此处不再重复发布静态 TF。
      如 URDF 中未定义，可取消下方注释来发布静态 TF。
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():

    pkg_wall_fusion = get_package_share_directory('wall_fusion')
    config_file = os.path.join(pkg_wall_fusion, 'config', 'wall_fusion.yaml')

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='false',
        description='Use simulation (Gazebo) clock if true')

    # =========================================================================
    # 主节点：多传感器融合墙面精修 Action Server
    # =========================================================================
    wall_fusion_server_node = Node(
        package='wall_fusion',
        executable='wall_fusion_server',
        name='wall_fusion_server',
        output='screen',
        parameters=[config_file, {'use_sim_time': use_sim_time}],
        # 如需 namespace 隔离，取消下行注释:
        # namespace='',
    )

    # =========================================================================
    # 以下静态 TF 仅在没有 URDF 定义时启用（模板）
    # 当前 S200_diff_robot.urdf 已定义 laser_link 和 camera_link 的 joint：
    #   laser_joint:  x=0.240, y=0.0001, z=0.290
    #   camera_joint: x=0.292, y=0.0003, z=0.213
    # 因此默认不发布静态 TF，避免与 robot_state_publisher 冲突。
    # 如果实际安装位置与 URDF 不同，请修改 URDF 或取消下方注释。
    # =========================================================================
    #
    # from launch_ros.actions import Node as StaticTFNode
    # static_tf_laser = Node(
    #     package='tf2_ros',
    #     executable='static_transform_publisher',
    #     name='static_tf_base_to_laser',
    #     arguments=['--x', '0.18', '--y', '0.0', '--z', '0.12',
    #                '--roll', '0', '--pitch', '0', '--yaw', '0',
    #                'base_link', 'laser_link'],
    # )
    # static_tf_camera = Node(
    #     package='tf2_ros',
    #     executable='static_transform_publisher',
    #     name='static_tf_base_to_camera',
    #     arguments=['--x', '0.15', '--y', '0.0', '--z', '0.35',
    #                '--roll', '0', '--pitch', '0', '--yaw', '0',
    #                'base_link', 'camera_link'],
    # )

    return LaunchDescription([
        declare_use_sim_time,
        wall_fusion_server_node,
        # static_tf_laser,
        # static_tf_camera,
    ])
