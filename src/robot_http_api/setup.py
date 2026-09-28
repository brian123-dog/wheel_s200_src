from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'robot_http_api'

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
    ],
    install_requires=['setuptools', 'fastapi', 'uvicorn', 'pydantic'],
    zip_safe=True,
    maintainer='sunrise',
    maintainer_email='2738921330@qq.com',
    description='REST API + WebSocket gateway for robot control (FastAPI)',
    license='MIT',
    entry_points={
        'console_scripts': [
            'gateway_node = robot_http_api.main:main',
        ],
    },
)
