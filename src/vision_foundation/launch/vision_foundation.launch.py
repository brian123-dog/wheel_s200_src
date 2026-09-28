"""Launch vision_foundation node.

The node needs the zhizhen venv's Python for torch/ultralytics.
Activate the venv before launching, or set the VENV_PYTHON env var.
"""
import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_share = get_package_share_directory('vision_foundation')
    default_model = os.path.join(pkg_share, 'models', 'yoloe-11s-seg.pt')

    venv_python = os.path.expanduser(
        os.environ.get(
            'VENV_PYTHON',
            '/home/sunrise/wheel_S200_src/src/zhizhen/venv/bin/python3'))

    node_script = os.path.join(pkg_share, '..', '..', 'lib',
                               'vision_foundation', 'detector_node')

    detector_node_cmd = ExecuteProcess(
        cmd=[venv_python, '-u', node_script,
             '--ros-args',
             '-p', f'model_path:={default_model}',
             '-p', 'device:=cpu',
             '-p', 'image_topic:=/camera_dcw2/color/image_raw',
             '-p', 'default_confidence:=0.25',
             '-p', 'publish_debug_image:=true',
             ],
        output='screen',
        name='vision_foundation',
    )

    return LaunchDescription([detector_node_cmd])
