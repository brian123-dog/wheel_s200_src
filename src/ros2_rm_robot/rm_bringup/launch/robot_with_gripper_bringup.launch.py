import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable, DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition
from launch.actions import IncludeLaunchDescription, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():

    # [新增] 声明 use_rviz 参数，默认 false 以节省 CPU
    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',  # 默认关闭 RViz，减少 CPU/GPU 负载
        description='Whether to start RViz2 visualization interface (set to true to enable)'
    )
    
    use_rviz = LaunchConfiguration('use_rviz')

    rm_63_driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('rm_driver'), 'launch', 'rm_63_driver.launch.py')
        )
    )

    rm_63_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('rm_description'), 'launch', 'robot_with_gripper.launch.py')
        ),
    )

    rm_63_control = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('rm_control'), 'launch', 'rm_63_control.launch.py')
        )
    )

    # [修改] 将 use_rviz 参数传递给 MoveIt launch 文件
    # 注意：这要求 real_moveit_demo_III.launch.py 也支持 use_rviz 参数（见下文说明）
    rm_63_moveit_config = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('robot_with_gripper_config'), 'launch', 'real_moveit_demo_III.launch.py')
        ),
        launch_arguments={
            'use_rviz': use_rviz,  # 传递参数给子 launch 文件
        }.items()
    )

    return LaunchDescription([
        # 必须将参数声明放在最前面
        use_rviz_arg,
        
        # 其他组件
        rm_63_driver,
        rm_63_description,
        rm_63_control,
        rm_63_moveit_config,
    ])
