from setuptools import find_packages, setup
import os
from glob import glob
package_name = 'rm_demos'

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
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jxs',
    maintainer_email='jxs@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'real_robot_grasp = rm_demos.real_robot_grasp:main',
            'cup_gripper_moveit = rm_demos.cup_gripper_moveit:main',
            'arc_move = rm_demos.arc_move:main',
            'delete_collision_cupid = rm_demos.delete_collision_CUPID:main',
            'from_gazebo_copy = rm_demos.from_gazebo_copy:main',
            'gazebo_copy = rm_demos.gazebo_copy:main',
            'real_move_demo = rm_demos.real_move_demo:main',
            'realsense_publisher = rm_demos.real_sense_publisher:main',
            'ros2_gripper_control = rm_demos.ros2_gripper_control:main',
            'time_less = rm_demos.time_less:main',
            'with_cup_collision = rm_demos.with_cup_collision:main',
            'zhixing = rm_demos.zhixing:main',
            'zhixing_modbus_topic = rm_demos.zhixing_modbus_topic:main',
            'zhixing_socket_node = rm_demos.zhixing_socket_node:main',
            'fivecups_spawn = rm_demos.fivecups_spawn:main',
            'five_demos = rm_demos.five_demos:main',
            'arm_task_service = rm_demos.arm_task_service_node:main',
            'arm_task_dummy_service = rm_demos.arm_task_dummy_service_node:main',
            'nav_to_arm_bridge = rm_demos.nav_to_arm_bridge:main',
        ],
    },
)
