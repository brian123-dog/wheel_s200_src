from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    task_config = LaunchConfiguration("task_config")
    backend = LaunchConfiguration("backend")
    include_rm_driver = LaunchConfiguration("include_rm_driver")
    include_robot_description = LaunchConfiguration("include_robot_description")
    include_static_camera_tf = LaunchConfiguration("include_static_camera_tf")
    publish_gripper_joint_states = LaunchConfiguration("publish_gripper_joint_states")
    rm_driver_launch = LaunchConfiguration("rm_driver_launch")
    robot_description_launch = LaunchConfiguration("robot_description_launch")
    camera_color_topic = LaunchConfiguration("camera_color_topic")
    camera_depth_topic = LaunchConfiguration("camera_depth_topic")
    camera_info_topic = LaunchConfiguration("camera_info_topic")
    camera_frame = LaunchConfiguration("camera_frame")
    vision_enabled = LaunchConfiguration("vision_enabled")
    vision_tf_timeout_s = LaunchConfiguration("vision_tf_timeout_s")
    qr_result_file = LaunchConfiguration("qr_result_file")
    qr_results_dir = LaunchConfiguration("qr_results_dir")
    base_frame = LaunchConfiguration("base_frame")
    gripper_backend = LaunchConfiguration("gripper_backend")
    gripper_ip = LaunchConfiguration("gripper_ip")
    gripper_port = LaunchConfiguration("gripper_port")
    gripper_device_id = LaunchConfiguration("gripper_device_id")
    gripper_pos_open = LaunchConfiguration("gripper_pos_open")
    gripper_pos_close = LaunchConfiguration("gripper_pos_close")
    gripper_command_timeout_s = LaunchConfiguration("gripper_command_timeout_s")
    gripper_modbus_port = LaunchConfiguration("gripper_modbus_port")
    gripper_baudrate = LaunchConfiguration("gripper_baudrate")
    gripper_modbus_timeout_s = LaunchConfiguration("gripper_modbus_timeout_s")
    gripper_voltage_type = LaunchConfiguration("gripper_voltage_type")
    gripper_default_speed = LaunchConfiguration("gripper_default_speed")
    gripper_default_force = LaunchConfiguration("gripper_default_force")
    gripper_acceleration = LaunchConfiguration("gripper_acceleration")
    gripper_deacceleration = LaunchConfiguration("gripper_deacceleration")
    gripper_init_on_connect = LaunchConfiguration("gripper_init_on_connect")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "task_config",
                default_value=PathJoinSubstitution(
                    [
                        FindPackageShare("realman_gemini_cabinet"),
                        "config",
                        "ros2_qr_panel_layout_movel_reroute.yaml",
                    ]
                ),
            ),
            DeclareLaunchArgument("backend", default_value="rm_driver"),
            DeclareLaunchArgument("include_rm_driver", default_value="true"),
            DeclareLaunchArgument("include_robot_description", default_value="true"),
            DeclareLaunchArgument("include_static_camera_tf", default_value="false"),
            DeclareLaunchArgument("publish_gripper_joint_states", default_value="false"),
            DeclareLaunchArgument("rm_driver_launch", default_value="rm_63_driver.launch.py"),
            DeclareLaunchArgument(
                "robot_description_launch",
                default_value="robot_with_gripper.launch.py",
            ),
            DeclareLaunchArgument("camera_color_topic", default_value="/camera_gemini/color/image_raw"),
            DeclareLaunchArgument("camera_depth_topic", default_value="/camera_gemini/depth/image_raw"),
            DeclareLaunchArgument("camera_info_topic", default_value="/camera_gemini/color/camera_info"),
            DeclareLaunchArgument("camera_frame", default_value="camera_gemini_color_optical_frame"),
            DeclareLaunchArgument("vision_enabled", default_value="true"),
            DeclareLaunchArgument("vision_tf_timeout_s", default_value="1.0"),
            DeclareLaunchArgument("qr_result_file", default_value=""),
            DeclareLaunchArgument("qr_results_dir", default_value=""),
            DeclareLaunchArgument("base_frame", default_value="base_link"),
            DeclareLaunchArgument("gripper_backend", default_value="socket_json"),
            DeclareLaunchArgument("gripper_ip", default_value="192.168.1.18"),
            DeclareLaunchArgument("gripper_port", default_value="8080"),
            DeclareLaunchArgument("gripper_device_id", default_value="1"),
            DeclareLaunchArgument("gripper_pos_open", default_value="0"),
            DeclareLaunchArgument("gripper_pos_close", default_value="9000"),
            DeclareLaunchArgument("gripper_command_timeout_s", default_value="2.0"),
            DeclareLaunchArgument("gripper_modbus_port", default_value="1"),
            DeclareLaunchArgument("gripper_baudrate", default_value="115200"),
            DeclareLaunchArgument("gripper_modbus_timeout_s", default_value="2"),
            DeclareLaunchArgument("gripper_voltage_type", default_value="3"),
            DeclareLaunchArgument("gripper_default_speed", default_value="100"),
            DeclareLaunchArgument("gripper_default_force", default_value="50"),
            DeclareLaunchArgument("gripper_acceleration", default_value="1000"),
            DeclareLaunchArgument("gripper_deacceleration", default_value="1000"),
            DeclareLaunchArgument("gripper_init_on_connect", default_value="true"),
            DeclareLaunchArgument("camera_tf_x", default_value="0.0"),
            DeclareLaunchArgument("camera_tf_y", default_value="0.0"),
            DeclareLaunchArgument("camera_tf_z", default_value="0.0"),
            DeclareLaunchArgument("camera_tf_roll", default_value="0.0"),
            DeclareLaunchArgument("camera_tf_pitch", default_value="0.0"),
            DeclareLaunchArgument("camera_tf_yaw", default_value="0.0"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [
                            FindPackageShare("rm_driver"),
                            "launch",
                            rm_driver_launch,
                        ]
                    )
                ),
                condition=IfCondition(include_rm_driver),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [
                            FindPackageShare("rm_description"),
                            "launch",
                            robot_description_launch,
                        ]
                    )
                ),
                condition=IfCondition(include_robot_description),
            ),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="cabinet_camera_static_tf",
                output="screen",
                arguments=[
                    "--x",
                    LaunchConfiguration("camera_tf_x"),
                    "--y",
                    LaunchConfiguration("camera_tf_y"),
                    "--z",
                    LaunchConfiguration("camera_tf_z"),
                    "--roll",
                    LaunchConfiguration("camera_tf_roll"),
                    "--pitch",
                    LaunchConfiguration("camera_tf_pitch"),
                    "--yaw",
                    LaunchConfiguration("camera_tf_yaw"),
                    "--frame-id",
                    base_frame,
                    "--child-frame-id",
                    camera_frame,
                ],
                condition=IfCondition(include_static_camera_tf),
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
                        "camera_color_topic": camera_color_topic,
                        "camera_depth_topic": camera_depth_topic,
                        "camera_info_topic": camera_info_topic,
                        "camera_frame": camera_frame,
                        "vision_enabled": ParameterValue(vision_enabled, value_type=bool),
                        "vision_tf_timeout_s": ParameterValue(
                            vision_tf_timeout_s,
                            value_type=float,
                        ),
                        "qr_result_file": qr_result_file,
                        "qr_results_dir": qr_results_dir,
                        "base_frame": base_frame,
                        "gripper_backend": gripper_backend,
                        "gripper_ip": gripper_ip,
                        "gripper_port": ParameterValue(gripper_port, value_type=int),
                        "gripper_device_id": ParameterValue(
                            gripper_device_id,
                            value_type=int,
                        ),
                        "gripper_pos_open": ParameterValue(
                            gripper_pos_open,
                            value_type=int,
                        ),
                        "gripper_pos_close": ParameterValue(
                            gripper_pos_close,
                            value_type=int,
                        ),
                        "gripper_command_timeout_s": ParameterValue(
                            gripper_command_timeout_s,
                            value_type=float,
                        ),
                        "gripper_modbus_port": ParameterValue(
                            gripper_modbus_port,
                            value_type=int,
                        ),
                        "gripper_baudrate": ParameterValue(
                            gripper_baudrate,
                            value_type=int,
                        ),
                        "gripper_modbus_timeout_s": ParameterValue(
                            gripper_modbus_timeout_s,
                            value_type=int,
                        ),
                        "gripper_voltage_type": ParameterValue(
                            gripper_voltage_type,
                            value_type=int,
                        ),
                        "gripper_default_speed": ParameterValue(
                            gripper_default_speed,
                            value_type=int,
                        ),
                        "gripper_default_force": ParameterValue(
                            gripper_default_force,
                            value_type=int,
                        ),
                        "gripper_acceleration": ParameterValue(
                            gripper_acceleration,
                            value_type=int,
                        ),
                        "gripper_deacceleration": ParameterValue(
                            gripper_deacceleration,
                            value_type=int,
                        ),
                        "gripper_init_on_connect": ParameterValue(
                            gripper_init_on_connect,
                            value_type=bool,
                        ),
                    }
                ],
            ),
        ]
    )
