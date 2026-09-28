from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'robot_http'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # 安装 config 目录下所有文件
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')),
        # 安装 launch 目录下所有 launch 文件
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
    ],
    install_requires=[
        'setuptools',
        'fastapi',
        'uvicorn',
        'apscheduler',
        'pydantic',
    ],
    zip_safe=True,
    maintainer='sunrise',
    maintainer_email='2738921330@qq.com',
    description='HTTP gateway for web dashboard to trigger robot navigation and inspection',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'gateway_node = robot_http.http_gateway_node:main',
            'inspection_master = robot_http.inspection_master:main',
        ],
    },
)
