import os
from  ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

from launch.conditions import IfCondition, UnlessCondition
from launch.actions import (DeclareLaunchArgument, GroupAction,
                            IncludeLaunchDescription, SetEnvironmentVariable)
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():

    # 添加连接机械臂的参数
    connect_arm_arg = DeclareLaunchArgument(
        'connect_arm',
        default_value='true',
        description='Whether to connect to the real arm (true) or not (false)'
    )

    connect_arm = LaunchConfiguration('connect_arm')

    rm_63_driver = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(get_package_share_directory(('rm_driver')),'launch', 'rm_63_driver.launch.py')),
            condition=IfCondition(connect_arm)
    )

    rm_63_description = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('rm_description'), 'launch', 'rm_63_display.launch.py')),
    )



    # 修改控制模块，传递connect_arm参数
    rm_63_control = Node(
        package='rm_control',
        executable='rm_control',
        parameters=[
            {'follow': False},
            {'arm_type': 632},
            {'connect_arm': connect_arm}
        ],
        output='screen',
    )

    rm_63_moveit_config = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(get_package_share_directory(('rm_63_config')),'launch', 'real_moveit_demo.launch.py'))
    )

    return LaunchDescription([
        connect_arm_arg,
        rm_63_description,
        rm_63_control,
        rm_63_moveit_config,
        # 只有在connect_arm为true时才启动驱动
        rm_63_driver,
    ])


