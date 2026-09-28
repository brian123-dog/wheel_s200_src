from launch_ros.actions import Node 
from launch import LaunchDescription 
import os
from ament_index_python.packages import get_package_share_directory
from launch.actions import (IncludeLaunchDescription,
                            ExecuteProcess,TimerAction,
                            RegisterEventHandler,
                            LogInfo)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.event_handlers import OnProcessStart, OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():

    """生成launch描述的主函数"""
    robot_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory('rm_bringup'), 'launch', 'robot_with_gripper_bringup.launch.py'
                )]            
        )
    )
    realsense_publish = Node(
        package='rm_demos',
        executable='realsense_publisher',
        name='realsense_publisher',
        output='screen',)
    cup_spawn = Node(
        package='rm_demos',
        executable='fivecups_spawn',
        name='fivecups_spawn',
        output='screen',)
    delayed_realsense = TimerAction(
        period=5.0,
        actions=[realsense_publish]
    )
    delayed_cup_spawn = TimerAction(
        period=10.0,
        actions=[cup_spawn]
    )
    on_robot_start = LogInfo(msg="机械臂系统已启动，10秒后启动Realsense发布器...")
    on_realsense_start = LogInfo(msg="Realsense发布器已启动，10秒后生成杯子模型...")
    on_all_started = LogInfo(msg="所有节点已启动，准备执行任务...")
    return LaunchDescription([
        LogInfo(msg="正在启动机械臂系统..."),      
        robot_bringup,
        on_robot_start,
        delayed_realsense,
        on_realsense_start,
        delayed_cup_spawn,
        on_all_started,
        # RegisterEventHandler
    ])