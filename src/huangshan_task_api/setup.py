from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'huangshan_task_api'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
        # 示例航点文件
        (os.path.join('share', package_name, 'config'),
            glob('config/*.json')),
    ],
    install_requires=['setuptools', 'fastapi', 'uvicorn', 'pydantic', 'apscheduler'],
    zip_safe=True,
    maintainer='sunrise',
    maintainer_email='2738921330@qq.com',
    description='Huangshan task & storage management REST API (FastAPI + ROS2)',
    license='MIT',
    entry_points={
        'console_scripts': [
            'huangshan_task_api_node = huangshan_task_api.main:main',
        ],
    },
)
