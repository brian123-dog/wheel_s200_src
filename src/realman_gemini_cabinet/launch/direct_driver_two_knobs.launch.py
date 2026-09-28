"""Optional two-knob sequence. The original bringup default remains unchanged."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    share = FindPackageShare('realman_gemini_cabinet')
    return LaunchDescription([
        DeclareLaunchArgument('task_config', default_value=PathJoinSubstitution([
            share, 'config', 'ros2_qr_panel_layout_two_knobs.yaml'])),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                share, 'launch', 'direct_driver_bringup.launch.py'])),
            launch_arguments={
                'task_config': LaunchConfiguration('task_config')
            }.items(),
        )
    ])
