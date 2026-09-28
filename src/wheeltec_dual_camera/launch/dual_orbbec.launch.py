"""
双奥比中光相机（DaBai DCW2 + Gemini 2）独立进程启动。

硬件：
  - DaBai DCW2 (车载, SN: CH82C5203DP)  →  camera_dcw2
  - Gemini 2     (机械臂, SN: AY3B1450038) →  camera_gemini

架构：
  - 两个相机各自运行在独立的 ComposableNodeContainer，禁止同一进程。
  - 通过 serial_number 硬绑定设备。
  - 保持与原来 orbbec_camera launch 完全一致的 namespace 结构：
    PushRosNamespace('camera_dcw2') + name='camera' → topic: /camera_dcw2/camera/...
"""

import os
import yaml
from launch import LaunchDescription
from launch.actions import GroupAction, TimerAction
from launch_ros.actions import ComposableNodeContainer, Node, PushRosNamespace
from launch_ros.descriptions import ComposableNode
from ament_index_python.packages import get_package_share_directory


def load_yaml_param_dict(path):
    """读取 ROS2 参数 YAML，提取 ros__parameters 字典"""
    with open(path, 'r') as f:
        raw = yaml.safe_load(f)
    first_node = next(iter(raw.values()))
    return first_node.get('ros__parameters', {})


def generate_launch_description():
    pkg_share = get_package_share_directory('wheeltec_dual_camera')

    dcw2_yaml   = os.path.join(pkg_share, 'config', 'camera_dcw2_params.yaml')
    gemini_yaml = os.path.join(pkg_share, 'config', 'camera_gemini_params.yaml')

    dcw2_params   = load_yaml_param_dict(dcw2_yaml)
    gemini_params = load_yaml_param_dict(gemini_yaml)

    # ===================== DaBai DCW2 车载相机 =====================
    dcw2_composable = ComposableNode(
        package='orbbec_camera',
        plugin='orbbec_camera::OBCameraNodeDriver',
        name='camera',
        namespace='',
        parameters=[dcw2_params],
    )

    dcw2_container = ComposableNodeContainer(
        name='dcw2_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[dcw2_composable],
        output='screen',
    )

    # base_link → camera_dcw2_link (车载相机位姿)
    dcw2_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_camera_dcw2',
        arguments=[
            '0.291921528759723',
            '0.000328321716947771',
            '0.21270400000001',
            '0.0', '0.0', '0.0',
            'base_link',
            'camera_dcw2_link',
        ],
        output='screen',
    )

    # ===================== Gemini 2 机械臂相机 =====================
    gemini_composable = ComposableNode(
        package='orbbec_camera',
        plugin='orbbec_camera::OBCameraNodeDriver',
        name='camera',
        namespace='',
        parameters=[gemini_params],
    )

    gemini_container = ComposableNodeContainer(
        name='gemini_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[gemini_composable],
        output='screen',
    )

    # link6 → camera_gemini_link (机械臂相机手眼标定位姿)
    gemini_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='link6_to_camera_gemini',
        arguments=[
            '0.06195', '-0.01163', '0.00658',
            '1.56007', '-1.54664', '1.58747',
            'link6',
            'camera_gemini_link',
        ],
        output='screen',
    )

    return LaunchDescription([
        GroupAction([PushRosNamespace('camera_dcw2'), dcw2_container]),
        dcw2_tf,
        # Gemini 延迟 3 秒启动，避免设备枚举冲突
        TimerAction(
            period=3.0,
            actions=[
                GroupAction([PushRosNamespace('camera_gemini'), gemini_container]),
                gemini_tf,
            ],
        ),
    ])
