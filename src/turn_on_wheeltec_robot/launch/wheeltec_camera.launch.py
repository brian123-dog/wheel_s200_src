import os
import yaml
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def load_yaml(file_path: Path) -> dict:
    with open(file_path, 'r') as f:
        return yaml.safe_load(f)


def spawn_camera_nodes(context, *args, **kwargs):
    param_file = LaunchConfiguration('camera_modes_yaml').perform(context)
    cfg = load_yaml(Path(param_file))

    camera_mode_ = LaunchConfiguration('camera_mode').perform(context)
    if not camera_mode_:
        camera_mode_ = cfg.get('camera_mode', '')

    print(f'[wheeltec_camera] camera_mode: {camera_mode_}')

    orbbec_dir = get_package_share_directory('orbbec_camera')
    orbbec_launch_dir = os.path.join(orbbec_dir, 'launch')

    # camera_mode 与 OrbbecSDK launch 文件的映射表
    camera_launch_map = {
        'astra':            'astra.launch.py',
        'astra_pro':        'astra_pro_plus.launch.py',
        'astra_pro_plus':   'astra_pro_plus.launch.py',
        'astra2':           'astra2.launch.py',
        'dabai':            'dabai.launch.py',
        'dabai_u3':         'dabai.launch.py',
        'dabai_d1':         'dabai_d1.launch.py',
        'dabai_dcl':        'dabai_dcl.py',
        'dabai_dcw':        'dabai_dcw.launch.py',
        'dabai_dcw2':       'dabai_dcw2.launch.py',
        'dabai_dw':         'dabai_dw.launch.py',
        'dabai_dw2':        'dabai_dw2.launch.py',
        'dabai_pro':        'dabai_pro.launch.py',
        'dabai_max':        'dabai_max.launch.py',
        'dabai_max_pro':    'dabai_max_pro.launch.py',
        'deeya':            'deeya.launch.py',
        'deeyea':           'deeya.launch.py',
        'gemini':           'dabai.launch.py',
        'gemini2':          'astra2.launch.py',
        'femto':            'femto.launch.py',
        'femto_bolt':       'femto_bolt.launch.py',
    }

    file_name = camera_launch_map.get(camera_mode_)
    if not file_name:
        available = ', '.join(sorted(camera_launch_map.keys()))
        raise RuntimeError(
            f"[wheeltec_camera] 未知的 camera_mode: '{camera_mode_}'\n"
            f"请在 wheeltec_param.yaml 中设置为以下之一:\n{available}"
        )

    launch_path = os.path.join(orbbec_launch_dir, file_name)
    if not os.path.exists(launch_path):
        raise RuntimeError(
            f"[wheeltec_camera] launch 文件不存在: {launch_path}\n"
            f"请确认 orbbec_camera 包已正确编译安装。"
        )

    camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(launch_path),
    )

    return [camera_launch]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'camera_modes_yaml',
            default_value=os.path.join(
                get_package_share_directory('turn_on_wheeltec_robot'),
                'config', 'wheeltec_param.yaml'),
            description='Path to wheeltec_param.yaml'),
        DeclareLaunchArgument(
            'camera_mode',
            default_value='',
            description='Which camera mode to launch (empty to use yaml value)'),
        OpaqueFunction(function=spawn_camera_nodes),
    ])
