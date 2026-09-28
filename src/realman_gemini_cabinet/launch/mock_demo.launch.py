from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    task_config = LaunchConfiguration("task_config")
    backend = LaunchConfiguration("backend")
    include_camera = LaunchConfiguration("include_camera")
    include_rm_bringup = LaunchConfiguration("include_rm_bringup")
    publish_gripper_joint_states = LaunchConfiguration("publish_gripper_joint_states")
    rm_bringup_launch = LaunchConfiguration("rm_bringup_launch")
    vision_enabled = LaunchConfiguration("vision_enabled")
    camera_frame = LaunchConfiguration("camera_frame")
    vision_tf_timeout_s = LaunchConfiguration("vision_tf_timeout_s")
    qr_result_file = LaunchConfiguration("qr_result_file")
    qr_results_dir = LaunchConfiguration("qr_results_dir")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "task_config",
                default_value=PathJoinSubstitution(
                    [
                        FindPackageShare("realman_gemini_cabinet"),
                        "config",
                        "ros2_mock_demo.yaml",
                    ]
                ),
            ),
            DeclareLaunchArgument("backend", default_value="mock"),
            DeclareLaunchArgument("include_camera", default_value="false"),
            DeclareLaunchArgument("include_rm_bringup", default_value="false"),
            DeclareLaunchArgument("publish_gripper_joint_states", default_value="false"),
            DeclareLaunchArgument(
                "rm_bringup_launch",
                default_value="robot_with_gripper_bringup.launch.py",
            ),
            DeclareLaunchArgument("camera_color_topic", default_value="/camera/color/image_raw"),
            DeclareLaunchArgument("camera_depth_topic", default_value="/camera/depth/image_raw"),
            DeclareLaunchArgument("camera_info_topic", default_value="/camera/color/camera_info"),
            DeclareLaunchArgument("camera_frame", default_value="camera_color_optical_frame"),
            DeclareLaunchArgument("vision_enabled", default_value="true"),
            DeclareLaunchArgument("vision_tf_timeout_s", default_value="1.0"),
            DeclareLaunchArgument("qr_result_file", default_value=""),
            DeclareLaunchArgument("qr_results_dir", default_value=""),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [
                            FindPackageShare("orbbec_camera"),
                            "launch",
                            "gemini2.launch.py",
                        ]
                    )
                ),
                condition=IfCondition(include_camera),
                launch_arguments={"camera_name": "camera"}.items(),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [
                            FindPackageShare("rm_bringup"),
                            "launch",
                            rm_bringup_launch,
                        ]
                    )
                ),
                condition=IfCondition(include_rm_bringup),
            ),
            Node(
                package="realman_gemini_cabinet",
                executable="gripper_joint_state_node",
                name="gripper_joint_state_node",
                output="screen",
                condition=IfCondition(publish_gripper_joint_states),
            ),
            Node(
                package="realman_gemini_cabinet",
                executable="cabinet_task_node",
                name="cabinet_task_node",
                output="screen",
                parameters=[
                    {
                        "task_config": task_config,
                        "backend": backend,
                        "camera_color_topic": LaunchConfiguration("camera_color_topic"),
                        "camera_depth_topic": LaunchConfiguration("camera_depth_topic"),
                        "camera_info_topic": LaunchConfiguration("camera_info_topic"),
                        "camera_frame": camera_frame,
                        "vision_enabled": ParameterValue(vision_enabled, value_type=bool),
                        "vision_tf_timeout_s": ParameterValue(
                            vision_tf_timeout_s,
                            value_type=float,
                        ),
                        "qr_result_file": qr_result_file,
                        "qr_results_dir": qr_results_dir,
                    }
                ],
            ),
        ]
    )
