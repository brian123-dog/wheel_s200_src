import os,yaml
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition

def load_yaml(file_path: Path) -> dict:
    with open(file_path, 'r') as f:
        return yaml.safe_load(f)

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    start_base = LaunchConfiguration('start_base', default='true')
    start_lidar = LaunchConfiguration('start_lidar', default='true')
    connect_arm = LaunchConfiguration('connect_arm', default='true')

    wheeltec_robot_dir = get_package_share_directory('turn_on_wheeltec_robot')
    wheeltec_launch_dir = os.path.join(wheeltec_robot_dir, 'launch')
        
    wheeltec_nav_dir = get_package_share_directory('wheeltec_nav2')
    wheeltec_nav_launchr = os.path.join(wheeltec_nav_dir, 'launch')
    cfg_params = load_yaml(os.path.join(get_package_share_directory('turn_on_wheeltec_robot'),'config','wheeltec_param.yaml'))
    car_mode = cfg_params['car_mode']
    print(f"car_mode:{car_mode}")

    map_dir = os.path.join(wheeltec_nav_dir, 'map')
    map_file = LaunchConfiguration('map', default=os.path.join(
        map_dir, 'WHEELTEC.yaml'))

    param_dir = os.path.join(wheeltec_nav_dir, 'param','wheeltec_params')
    param_file = LaunchConfiguration('params', default=os.path.join(
        param_dir, f'param_{car_mode}.yaml'))
    print(os.path.join(param_dir, f'param_{car_mode}.yaml'))

    return LaunchDescription([
        DeclareLaunchArgument(
            'map',
            default_value=map_file,
            description='Full path to map file to load'),

        DeclareLaunchArgument(
            'params',
            default_value=param_file,
            description='Full path to param file to load'),
        LogInfo(msg=['[wheeltec_nav2] Using params file: ', param_file]),
        DeclareLaunchArgument(
            'start_base',
            default_value='true',
            description='Whether to launch turn_on_wheeltec_robot stack (set false if robot.service already runs it)'),
        DeclareLaunchArgument(
            'start_lidar',
            default_value='true',
            description='Whether to launch wheeltec_lidar driver (set false if already running)'),
        DeclareLaunchArgument(
            'connect_arm',
            default_value='true',
            description='Whether to connect and launch arm service node (set false to run without arm)'),
        Node(
            name='waypoint_cycle',
            package='nav2_waypoint_cycle',
            executable='nav2_waypoint_cycle',
        ),
        Node(
            name='arm_task_service_node',
            package='rm_demos',
            executable='arm_task_service',
            output='screen',
            condition=IfCondition(connect_arm),
        ),
        Node(
            name='arm_task_dummy_service_node',
            package='rm_demos',
            executable='arm_task_dummy_service',
            output='screen',
            condition=UnlessCondition(connect_arm),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [wheeltec_launch_dir, '/turn_on_wheeltec_robot.launch.py']),
            condition=IfCondition(start_base),
        ),
       IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [wheeltec_launch_dir, '/wheeltec_lidar.launch.py']),
            condition=IfCondition(start_lidar),
        ),        
        # 等底盘 EKF + 激光 3 秒稳定后再启动 Nav2 bringup
        # 避免 AMCL 激活时 TF buffer 为空导致激光全丢帧
        TimerAction(
            period=3.0,
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        [wheeltec_nav_launchr, '/bringup_launch.py']),
                    launch_arguments={
                        'map': map_file,
                        'use_sim_time': use_sim_time,
                        'params_file': param_file}.items(),
                ),
            ],
        ),
    ])
