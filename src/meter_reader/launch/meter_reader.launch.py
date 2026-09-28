"""Launch meter_reader node.

The node needs the zhizhen venv's Python for torch/ultralytics.
Activate the venv before launching, or set the VENV_PYTHON env var.
"""
import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_share = get_package_share_directory('meter_reader')
    default_model = os.path.join(pkg_share, 'models', 'best.pt')
    results_dir = os.path.join(os.getcwd(), 'meter_results')

    venv_python = os.path.expanduser(
        os.environ.get(
            'VENV_PYTHON',
            '/home/sunrise/wheel_S200_src/src/zhizhen/venv/bin/python3'))

    node_script = os.path.join(pkg_share, '..', '..', 'lib',
                               'meter_reader', 'meter_reader_node')

    meter_reader_cmd = ExecuteProcess(
        cmd=[venv_python, '-u', node_script,
             '--ros-args',
             '-p', f'results_dir:={results_dir}',
             '-p', f'model_path:={default_model}',
             '-p', 'box_confidence:=0.01',
             '-p', 'kpts_confidence_threshold:=0.01',
             '-p', 'save_debug_images:=true',
             ],
        output='screen',
        name='meter_reader',
    )

    return LaunchDescription([meter_reader_cmd])
