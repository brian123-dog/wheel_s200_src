from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("backend", default_value="rm_driver"),
            DeclareLaunchArgument(
                "task_config",
                default_value=PathJoinSubstitution(
                    [
                        FindPackageShare("realman_gemini_cabinet"),
                        "config",
                        "ros2_qr_named_panel_demo.yaml",
                    ]
                ),
            ),
            DeclareLaunchArgument("include_camera", default_value="false"),
            DeclareLaunchArgument("include_rm_bringup", default_value="true"),
            DeclareLaunchArgument("publish_gripper_joint_states", default_value="true"),
            DeclareLaunchArgument(
                "rm_bringup_launch",
                default_value="robot_with_gripper_bringup.launch.py",
            ),
            DeclareLaunchArgument(
                "camera_color_topic",
                default_value="/camera_gemini/color/image_raw",
            ),
            DeclareLaunchArgument(
                "camera_depth_topic",
                default_value="/camera_gemini/depth/image_raw",
            ),
            DeclareLaunchArgument(
                "camera_info_topic",
                default_value="/camera_gemini/color/camera_info",
            ),
            DeclareLaunchArgument(
                "camera_frame",
                default_value="camera_gemini_color_optical_frame",
            ),
            DeclareLaunchArgument("vision_enabled", default_value="true"),
            DeclareLaunchArgument("vision_tf_timeout_s", default_value="1.0"),
            DeclareLaunchArgument("qr_result_file", default_value=""),
            DeclareLaunchArgument("qr_results_dir", default_value=""),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [
                            FindPackageShare("realman_gemini_cabinet"),
                            "launch",
                            "mock_demo.launch.py",
                        ]
                    )
                ),
                launch_arguments={
                    "task_config": LaunchConfiguration("task_config"),
                    "backend": LaunchConfiguration("backend"),
                    "include_camera": LaunchConfiguration("include_camera"),
                    "include_rm_bringup": LaunchConfiguration("include_rm_bringup"),
                    "publish_gripper_joint_states": LaunchConfiguration(
                        "publish_gripper_joint_states"
                    ),
                    "rm_bringup_launch": LaunchConfiguration("rm_bringup_launch"),
                    "camera_color_topic": LaunchConfiguration("camera_color_topic"),
                    "camera_depth_topic": LaunchConfiguration("camera_depth_topic"),
                    "camera_info_topic": LaunchConfiguration("camera_info_topic"),
                    "camera_frame": LaunchConfiguration("camera_frame"),
                    "vision_enabled": LaunchConfiguration("vision_enabled"),
                    "vision_tf_timeout_s": LaunchConfiguration("vision_tf_timeout_s"),
                    "qr_result_file": LaunchConfiguration("qr_result_file"),
                    "qr_results_dir": LaunchConfiguration("qr_results_dir"),
                }.items(),
            ),
        ]
    )
