import os
import yaml
from pathlib import Path

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, IncludeLaunchDescription,
    GroupAction, OpaqueFunction
)
from launch_ros.actions import LifecycleNode,Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory


def load_yaml(path: Path) -> dict:
    with open(path, 'r') as f:
        return yaml.safe_load(f)


def _is_disabled_angle(value) -> bool:
    if value is None:
        return True
    if isinstance(value, (list, tuple)):
        return len(value) == 0 or all(v == 0 or v == 0.0 for v in value)
    return value == 0 or value == 0.0


def _coerce_angle_value(value, template_value):
    """Coerce cfg angle values to match template type (scalar vs list)."""
    if isinstance(value, (list, tuple)):
        if isinstance(template_value, (list, tuple)):
            return list(value)
        # template expects scalar
        return float(value[0]) if len(value) > 0 else float(0.0)
    # cfg is scalar
    if isinstance(template_value, (list, tuple)):
        return [value]
    return float(value)


def include_lidar_launch(context, *args, **kwargs):
    # 1. 读取参数文件
    yaml_path = LaunchConfiguration('lidar_type_yaml').perform(context)
    cfg = load_yaml(Path(yaml_path))

    # 2. 取 lidar_type（优先命令行 > yaml 默认）
    lidar_type = LaunchConfiguration('lidar_type').perform(context) or cfg['lidar_type']
    lidar_type = str(lidar_type).strip()
    print(f'lidar_type:{lidar_type}')
    # 3. 根据型号决定包路径
    actions = []
    if lidar_type.startswith('ls'):            # LS 系列
        launch_dir = os.path.join(get_package_share_directory('lslidar_driver'), 'launch')
        if lidar_type == 'lscx':
            template_yaml = Path(
                get_package_share_directory('lslidar_driver'),
                'config', 'lslidar_cx.yaml'
                )
            cx_cfg = yaml.safe_load(template_yaml.read_text())['cx']['lslidar_driver_node']['ros__parameters']
            if cfg['lscx']['angle_disable_min'] != 0 and cfg['lscx']['angle_disable_max'] != 0 :
                cx_cfg['angle_disable_min'] = cfg['lscx']['angle_disable_min']
                cx_cfg['angle_disable_max'] = cfg['lscx']['angle_disable_max']
            
            lidar_launch = GroupAction(
                actions=[
                    LifecycleNode(package='lslidar_driver',
                                    executable='lslidar_driver_node',
                                    name='lslidar_driver_node',
                                    namespace='cx', # 与对应yaml文件中命名空间一致
                                    parameters=[cx_cfg],
                                    output='screen'
                                    ),
                    IncludeLaunchDescription(PythonLaunchDescriptionSource(
                        os.path.join(get_package_share_directory('pointcloud_to_laserscan'),
                            'launch', 'pointcloud_to_laserscan_launch.py')),
                        )
                    ]
                )
        else:
            template_yaml = Path(
                get_package_share_directory('lslidar_driver'),
                'config', 'lslidar_x10.yaml'
                )
            if not template_yaml.is_file():
                fallback_yaml = Path(
                    get_package_share_directory('turn_on_wheeltec_robot'),
                    'config', 'lslidar_x10.yaml'
                )
                if fallback_yaml.is_file():
                    template_yaml = fallback_yaml
                else:
                    raise FileNotFoundError(
                        f'Cannot find lslidar_x10.yaml. Tried: {template_yaml} and {fallback_yaml}'
                    )
            lidar_port = str(cfg['x10']['lidar_port']).strip()
            x10_cfg = yaml.safe_load(template_yaml.read_text())['x10']['lslidar_driver_node']['ros__parameters']
            
            if lidar_type.endswith('net'):
                x10_cfg['serial_port'] = ''
            elif lidar_type.endswith('uart'):
                x10_cfg['serial_port'] = lidar_port
            if lidar_type.startswith('ls_M10'):
                if lidar_type.startswith('ls_M10P'):
                    x10_cfg['lidar_model'] = 'M10P'
                else:
                    x10_cfg['lidar_model'] = 'M10'
            if lidar_type.startswith('ls_N10'):
                if lidar_type.startswith('ls_N10Plus'):
                    x10_cfg['lidar_model'] = 'N10Plus'
                else:
                    x10_cfg['lidar_model'] = 'N10'
                    
            angle_min = cfg.get('x10', {}).get('angle_disable_min')
            angle_max = cfg.get('x10', {}).get('angle_disable_max')
            if (not _is_disabled_angle(angle_min)) and (not _is_disabled_angle(angle_max)):
                x10_cfg['angle_disable_min'] = _coerce_angle_value(
                    angle_min,
                    x10_cfg.get('angle_disable_min')
                )
                x10_cfg['angle_disable_max'] = _coerce_angle_value(
                    angle_max,
                    x10_cfg.get('angle_disable_max')
                )
                
            lidar_launch = LifecycleNode(package='lslidar_driver',
                                    executable='lslidar_driver_node',
                                    name='lslidar_driver_node',
                                    namespace='x10', # 与对应yaml文件中命名空间一致
                                    parameters=[x10_cfg],
                                    output='screen'
                                    )
        
    elif lidar_type == 'ld14':
        lidar_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('ldlidar_sl_ros2'), 'launch','ld14.launch.py')),)
    elif lidar_type == 'ld06':
        lidar_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('ldlidar_stl_ros2'), 'launch','ld06.launch.py')),)
    elif lidar_type == 'ldstl06n':
        lidar_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('ldlidar'), 'launch', 'stl06n.launch.py')),)
    elif lidar_type == 'rplidar_c1':
        lidar_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('rplidar_ros'), 'launch', 'rplidar_c1_launch.py')),)
    else:
        raise ValueError(f'Unsupported lidar: {lidar_type}')
    # 4. 返回动作列表
    actions.append(lidar_launch)  
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'lidar_type_yaml',
            default_value=os.path.join(
                get_package_share_directory('turn_on_wheeltec_robot'),
                'config', 'wheeltec_param.yaml'),
            description='Path to lidar_type.yaml'),
        DeclareLaunchArgument(
            'lidar_type',
            default_value='',
            description='Which lidar model to launch'),
        OpaqueFunction(function=include_lidar_launch),
    ])
