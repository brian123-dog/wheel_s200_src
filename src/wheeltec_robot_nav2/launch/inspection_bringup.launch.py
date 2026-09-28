#!/usr/bin/env python3
"""
inspection_bringup.launch.py —— 导航 + 到点精修 测试用一键启动
====================================================
启动顺序 (按延时依次拉起):
  1. realman_gemini_cabinet → 机械臂控制 (direct driver)
  2. wheeltec_dual_camera   → 奥比中光双相机 (DCW2 + Gemini2)
  3. wheeltec_nav2          → 底盘 + 激光雷达 + Nav2 导航
  4. wall_fusion            → 到点精修服务 (/align_to_wall)

用法:
  ros2 launch wheeltec_nav2 inspection_bringup.launch.py

启动后另开终端做导航测试:
  ros2 run waypoint_action_publisher json_waypoint_navigator

可选参数 (按需覆盖):
  task_config:=...          柜体任务配置（默认独立双旋钮模式）
  connect_arm:=false        不启动机械臂相关节点
  start_camera:=false       跳过相机
  start_nav:=false          跳过导航
  start_wall_fusion:=false  跳过到点精修
  depth_points_topic:=...   深度点云话题 (默认 /camera_dcw2/depth/points)
  arm_delay / camera_delay / nav_delay / wall_fusion_delay  各阶段延时秒数
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
    RegisterEventHandler,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():

    # ==================== 参数声明 ====================
    connect_arm = LaunchConfiguration("connect_arm", default="true")
    use_sim_time = LaunchConfiguration("use_sim_time", default="false")

    task_config = LaunchConfiguration("task_config")

    # 各阶段独立开关
    start_arm = LaunchConfiguration("start_arm", default="true")
    start_camera = LaunchConfiguration("start_camera", default="true")
    start_nav = LaunchConfiguration("start_nav", default="true")
    start_wall_fusion = LaunchConfiguration("start_wall_fusion", default="false")

    # 各阶段延时 (秒) —— 保证按 机械臂→相机→导航→精修 顺序稳定拉起
    arm_delay = LaunchConfiguration("arm_delay", default="0.0")
    camera_delay = LaunchConfiguration("camera_delay", default="6.0")
    nav_delay = LaunchConfiguration("nav_delay", default="12.0")
    wall_fusion_delay = LaunchConfiguration("wall_fusion_delay", default="18.0")

    # 深度点云话题 (DCW2 车载相机, namespace=camera_dcw2)
    # 如需临时切换可用 launch 参数覆盖，无需重新编译
    depth_points_topic = LaunchConfiguration(
        "depth_points_topic", default="/camera_dcw2/depth/points")

    # ==================== 包路径 ====================
    pkg_wheeltec_nav = get_package_share_directory("wheeltec_nav2")
    pkg_dual_camera = get_package_share_directory("wheeltec_dual_camera")
    pkg_gemini = get_package_share_directory("realman_gemini_cabinet")
    pkg_wall_fusion = get_package_share_directory("wall_fusion")

    # ==================== 配置文件路径 ====================
    map_file = os.path.join(pkg_wheeltec_nav, "map", "WHEELTEC.yaml")
    wf_config = os.path.join(pkg_wall_fusion, "config", "wall_fusion.yaml")

    return LaunchDescription(
        [
            # -------------------- 参数 --------------------
            DeclareLaunchArgument(
                "task_config",
                default_value=os.path.join(
                    
            # ///切换单选钮模式 --------------------
                    
                    # pkg_gemini, "config", "ros2_qr_panel_layout_movel_reroute.yaml"),
                    
            
            # ///切换双旋钮模式 --------------------
            # ///切换双旋钮模式 --------------------
            # ///切换双旋钮模式 --------------------
            
                    pkg_gemini, "config", "ros2_qr_panel_layout_two_knobs.yaml"),

                    
                    
                description="柜体任务配置；总启动默认双旋钮，原机械臂独立启动默认不变"),
            DeclareLaunchArgument("connect_arm", default_value="true",
                                  description="是否启动机械臂相关节点"),
            DeclareLaunchArgument("use_sim_time", default_value="false",
                                  description="使用仿真时间"),
            DeclareLaunchArgument("start_arm", default_value="true",
                                  description="是否启动机械臂阶段"),
            DeclareLaunchArgument("start_camera", default_value="true",
                                  description="是否启动相机阶段"),
            DeclareLaunchArgument("start_nav", default_value="true",
                                  description="是否启动导航阶段"),
            DeclareLaunchArgument("start_wall_fusion", default_value="false",
                                  description="是否启动到点精修阶段"),
            DeclareLaunchArgument("arm_delay", default_value="0.0",
                                  description="机械臂阶段延时(秒)"),
            DeclareLaunchArgument("camera_delay", default_value="6.0",
                                  description="相机阶段延时(秒)"),
            DeclareLaunchArgument("nav_delay", default_value="12.0",
                                  description="导航阶段延时(秒)"),
            DeclareLaunchArgument("wall_fusion_delay", default_value="18.0",
                                  description="到点精修阶段延时(秒)"),
            DeclareLaunchArgument(
                "depth_points_topic", default_value="/camera_dcw2/depth/points",
                description="深度点云话题 (wall_fusion 订阅)"),

            LogInfo(msg="=============================================="),
            LogInfo(msg="  导航 + 到点精修 测试用一键启动"),
            LogInfo(msg="  顺序: 机械臂 -> 相机 -> 导航 -> 到点精修"),
            LogInfo(msg="=============================================="),

            # =================================================================
            # 阶段 1. 机械臂 (realman_gemini_cabinet direct driver)
            # =================================================================
            TimerAction(
                period=arm_delay,
                actions=[
                    LogInfo(msg="[1/4] 启动机械臂 (direct_driver_bringup)..."),
                    LogInfo(msg=["[柜体任务] 配置: ", task_config]),
                    IncludeLaunchDescription(
                        PythonLaunchDescriptionSource(
                            [pkg_gemini, "/launch", "/direct_driver_bringup.launch.py"]
                        ),
                        launch_arguments={"task_config": task_config}.items(),
                        condition=IfCondition(start_arm),
                    ),
                ],
            ),

            # =================================================================
            # 阶段 2. 奥比中光双相机 (DCW2 车载 + Gemini2 机械臂)
            # =================================================================
            TimerAction(
                period=camera_delay,
                actions=[
                    LogInfo(msg="[2/4] 启动双相机 (dual_orbbec)..."),
                    IncludeLaunchDescription(
                        PythonLaunchDescriptionSource(
                            [pkg_dual_camera, "/launch", "/dual_orbbec.launch.py"]
                        ),
                        condition=IfCondition(start_camera),
                    ),
                ],
            ),

            # =================================================================
            # 阶段 3. Nav2 导航 (底盘 + 激光雷达 + 地图 + AMCL + 规划器)
            # =================================================================
            TimerAction(
                period=nav_delay,
                actions=[
                    LogInfo(msg="[3/4] 启动 Nav2 导航..."),
                    IncludeLaunchDescription(
                        PythonLaunchDescriptionSource(
                            [pkg_wheeltec_nav, "/launch", "/wheeltec_nav2.launch.py"]
                        ),
                        launch_arguments={
                            "map": map_file,
                            "use_sim_time": use_sim_time,
                            "connect_arm": connect_arm,
                        }.items(),
                        condition=IfCondition(start_nav),
                    ),
                ],
            ),

            # =================================================================
            # 阶段 4. 到点精修 Action Server (wall_fusion)
            # =================================================================
            TimerAction(
                period=wall_fusion_delay,
                actions=[
                    LogInfo(msg="[4/4] 启动到点精修服务 (wall_fusion_server)..."),
                    Node(
                        package="wall_fusion",
                        executable="wall_fusion_server",
                        name="wall_fusion_server",
                        output="screen",
                        parameters=[
                            wf_config,
                            {
                                "use_sim_time": use_sim_time,
                                # 覆盖 yaml 中的话题，确保指向 DCW2 车载相机
                                "depth_points_topic": depth_points_topic,
                            },
                        ],
                        condition=IfCondition(start_wall_fusion),
                    ),
                ],
            ),

            LogInfo(msg="=============================================="),
            LogInfo(msg="  全部阶段已拉起。另开终端开始导航测试:"),
            LogInfo(msg="  ros2 run waypoint_action_publisher json_waypoint_navigator"),
            LogInfo(msg="  单独测试到点精修:"),
            LogInfo(msg="  ros2 run wall_fusion align_to_wall_client"),
            LogInfo(msg="=============================================="),
        ]
    )
