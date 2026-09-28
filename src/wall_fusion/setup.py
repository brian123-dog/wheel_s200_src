from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'wall_fusion'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sunrise',
    maintainer_email='2738921330@qq.com',
    description='Multi-sensor fusion wall alignment refinement',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'wall_fusion_server = wall_fusion.wall_fusion_server:main',
            'debug_plot = wall_fusion.debug_plot:main',
            'align_to_wall_client = wall_fusion.align_to_wall_client:main',
        ],
    },
)
