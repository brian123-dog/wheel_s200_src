import unittest
import json
import math
import tempfile
from pathlib import Path
from types import SimpleNamespace


try:
    import rclpy
    from rclpy.parameter import Parameter
    from std_msgs.msg import Float64
    from std_srvs.srv import SetBool, Trigger

    from rg_cabinet.adapters.mock import MockRobotAdapter
    from rg_cabinet.geometry import Pose
    from rg_cabinet.perception import ElementTarget
    from rg_cabinet_ros2.backends import quaternion_pose_to_ros_pose
    from rg_cabinet_ros2.task_node import CabinetTaskNode
except ImportError:
    rclpy = None
    Parameter = None
    Float64 = None
    SetBool = None
    Trigger = None
    MockRobotAdapter = None
    Pose = None
    ElementTarget = None
    quaternion_pose_to_ros_pose = None
    CabinetTaskNode = None


def make_rgb_image():
    return SimpleNamespace(
        width=2,
        height=1,
        encoding="rgb8",
        step=6,
        data=bytes([255, 0, 0, 0, 255, 0]),
    )


@unittest.skipIf(rclpy is None, "ROS2 Python packages are not available")
class Ros2TaskNodeTests(unittest.TestCase):
    def setUp(self):
        if not rclpy.ok():
            rclpy.init(args=None)

    def tearDown(self):
        if rclpy.ok():
            rclpy.shutdown()

    def test_trigger_callback_executes_mock_sequence(self):
        config_path = (
            Path(__file__).resolve().parents[1]
            / "config"
            / "ros2_mock_demo.yaml"
        )
        node = CabinetTaskNode(
            parameter_overrides=[
                Parameter("task_config", value=str(config_path)),
                Parameter("backend", value="mock"),
            ]
        )
        try:
            response = node._handle_run_sequence(Trigger.Request(), Trigger.Response())
        finally:
            node.destroy_node()

        self.assertTrue(response.success)
        self.assertIn("sequence_complete", response.message)
        self.assertIn("recorded_commands", response.message)

    def test_manual_gripper_services_and_topic_only_set_gripper(self):
        config_path = (
            Path(__file__).resolve().parents[1]
            / "config"
            / "ros2_mock_demo.yaml"
        )
        robot = MockRobotAdapter()
        node = CabinetTaskNode(
            parameter_overrides=[
                Parameter("task_config", value=str(config_path)),
                Parameter("backend", value="mock"),
            ]
        )
        node._make_robot_adapter = lambda backend_name: robot
        try:
            close_request = SetBool.Request()
            close_request.data = True
            close_response = node._handle_set_gripper(
                close_request,
                SetBool.Response(),
            )
            open_response = node._handle_open_gripper(
                Trigger.Request(),
                Trigger.Response(),
            )
            msg = Float64()
            msg.data = 0.35
            node._gripper_position_cmd_cb(msg)
        finally:
            node.destroy_node()

        self.assertTrue(close_response.success)
        close_payload = json.loads(close_response.message)
        self.assertEqual(close_payload["backend"], "mock")
        self.assertEqual(close_payload["gripper_backend"], "socket_json")
        self.assertEqual(close_payload["gripper_pos_open"], 0)
        self.assertEqual(close_payload["gripper_pos_close"], 9000)
        self.assertEqual(close_payload["gripper_ip"], "192.168.1.18")
        self.assertEqual(close_payload["gripper_device_id"], 1)
        self.assertEqual(close_payload["gripper_modbus_port"], 1)
        self.assertIsNone(close_payload["gripper_debug"])
        self.assertTrue(open_response.success)
        gripper_commands = [
            command for command in robot.commands if command.name == "set_gripper"
        ]
        self.assertEqual(len(gripper_commands), 3)
        self.assertEqual(gripper_commands[0].payload["position"], 1.0)
        self.assertEqual(gripper_commands[1].payload["position"], 0.0)
        self.assertEqual(gripper_commands[2].payload["position"], 0.35)
        self.assertTrue(all(command.name == "set_gripper" for command in robot.commands))

    def test_manual_gripper_service_reports_adapter_failure(self):
        class FailingRobot:
            def set_gripper(self, position):
                raise RuntimeError("socket failed")

        config_path = (
            Path(__file__).resolve().parents[1]
            / "config"
            / "ros2_mock_demo.yaml"
        )
        node = CabinetTaskNode(
            parameter_overrides=[
                Parameter("task_config", value=str(config_path)),
                Parameter("backend", value="mock"),
            ]
        )
        node._make_robot_adapter = lambda backend_name: FailingRobot()
        try:
            response = node._handle_close_gripper(
                Trigger.Request(),
                Trigger.Response(),
            )
        finally:
            node.destroy_node()

        self.assertFalse(response.success)
        payload = json.loads(response.message)
        self.assertEqual(payload["event"], "manual_gripper_failed")
        self.assertEqual(payload["backend"], "mock")
        self.assertEqual(payload["gripper_backend"], "socket_json")
        self.assertEqual(payload["position"], 1.0)
        self.assertIn("socket failed", payload["error"])

    def test_default_camera_topics_use_arm_gemini_camera(self):
        config_path = (
            Path(__file__).resolve().parents[1]
            / "config"
            / "ros2_mock_demo.yaml"
        )
        node = CabinetTaskNode(
            parameter_overrides=[
                Parameter("task_config", value=str(config_path)),
                Parameter("backend", value="mock"),
            ]
        )
        try:
            self.assertEqual(
                node.get_parameter("camera_color_topic").value,
                "/camera_gemini/color/image_raw",
            )
            self.assertEqual(
                node.get_parameter("camera_depth_topic").value,
                "/camera_gemini/depth/image_raw",
            )
            self.assertEqual(
                node.get_parameter("camera_info_topic").value,
                "/camera_gemini/color/camera_info",
            )
            self.assertEqual(
                node.get_parameter("camera_frame").value,
                "camera_gemini_color_optical_frame",
            )
        finally:
            node.destroy_node()

    def test_trigger_callback_executes_mock_sequence_from_qr_observation_poses(self):
        config_path = (
            Path(__file__).resolve().parents[1]
            / "config"
            / "ros2_qr_vision_cabinet_demo.yaml"
        )
        positions = [
            {"x": 0.10, "y": 0.20, "z": 0.30, "roll": 0.0, "pitch": 0.0, "yaw": 0.0},
            {"x": 0.11, "y": 0.21, "z": 0.31, "roll": 0.0, "pitch": 0.0, "yaw": 0.1},
            {"x": 0.12, "y": 0.22, "z": 0.32, "roll": 0.0, "pitch": 0.0, "yaw": 0.2},
            {"x": 0.13, "y": 0.23, "z": 0.33, "roll": 0.0, "pitch": 0.0, "yaw": 0.3},
        ]

        with tempfile.TemporaryDirectory() as tmp:
            qr_path = Path(tmp) / "qr_mock.json"
            qr_path.write_text(
                json.dumps({"qr_data": json.dumps({"position": positions})}),
                encoding="utf-8",
            )
            robot = MockRobotAdapter(contact_after_servo_calls=2)
            node = CabinetTaskNode(
                parameter_overrides=[
                    Parameter("task_config", value=str(config_path)),
                    Parameter("backend", value="mock"),
                    Parameter("qr_result_file", value=str(qr_path)),
                    Parameter("qr_observation_settle_s", value=0.0),
                    Parameter(
                        "meter_results_dir",
                        value=str(Path(tmp) / "meter_results"),
                    ),
                ]
            )
            node.camera_cache.color = make_rgb_image()
            node.camera_cache.depth = object()
            node.camera_cache.camera_info = object()
            node._make_robot_adapter = lambda backend_name: robot
            resolver_observed_poses = []

            def resolve(config, step):
                del config
                resolver_observed_poses.append(robot.current_pose.as_list())
                details = {}
                if step["name"] == "middle_black_knob":
                    details = {
                        "pixel": [320.0, 215.0],
                        "desired_pixel": [320.0, 215.0],
                        "bbox": [310, 205, 325, 220],
                        "pixel_count": 125,
                        "pixel_error_base_m": [0.0, 0.0, 0.0],
                    }
                return ElementTarget(
                    step["name"],
                    step["target"]["kind"],
                    Pose(0.3, 0.0, 0.4, 3.1416, 0.0, 0.0, frame="base_link"),
                    details=details,
                )

            node.vision_target_resolver.resolve = resolve
            try:
                response = node._handle_run_sequence_from_qr(
                    Trigger.Request(),
                    Trigger.Response(),
                )
            finally:
                node.destroy_node()

        self.assertTrue(response.success)
        payload = json.loads(response.message)
        self.assertEqual(payload["event"], "qr_sequence_complete")
        self.assertEqual(payload["qr_target_count"], 4)
        self.assertEqual(
            resolver_observed_poses,
            [
                [item["x"], item["y"], item["z"], item["roll"], item["pitch"], item["yaw"]]
                for item in positions[:3]
            ],
        )
        self.assertEqual(
            [item["skill"] for item in payload["results"]],
            ["button", "toggle", "knob", "read_meter"],
        )
        self.assertEqual(len(payload["meter_readings"]), 1)
        self.assertTrue(Path(payload["meter_readings"][0]["path"]).is_file())
        self.assertEqual(
            payload["results"][3]["details"]["meter_reading"]["path"],
            payload["meter_readings"][0]["path"],
        )
        self.assertTrue(all(item["success"] for item in payload["results"]))

    def test_trigger_callback_executes_mock_sequence_from_qr_quaternion_observation_poses(self):
        config_path = (
            Path(__file__).resolve().parents[1]
            / "config"
            / "ros2_qr_vision_cabinet_demo.yaml"
        )
        yaw = 0.4
        qz = math.sin(yaw / 2.0)
        qw = math.cos(yaw / 2.0)
        positions = [
            {
                "position": {"x": 0.10, "y": 0.20, "z": 0.30},
                "orientation": {"x": 0.0, "y": 0.0, "z": qz, "w": qw},
            },
            {
                "position": {"x": 0.11, "y": 0.21, "z": 0.31},
                "orientation": {"x": 0.0, "y": 0.0, "z": qz, "w": qw},
            },
            {
                "position": {"x": 0.12, "y": 0.22, "z": 0.32},
                "orientation": {"x": 0.0, "y": 0.0, "z": qz, "w": qw},
            },
            {
                "position": {"x": 0.13, "y": 0.23, "z": 0.33},
                "orientation": {"x": 0.0, "y": 0.0, "z": qz, "w": qw},
            },
        ]

        with tempfile.TemporaryDirectory() as tmp:
            qr_path = Path(tmp) / "qr_mock_quat.json"
            qr_path.write_text(
                json.dumps({"qr_data": json.dumps({"position": positions})}),
                encoding="utf-8",
            )
            robot = MockRobotAdapter(contact_after_servo_calls=2)
            node = CabinetTaskNode(
                parameter_overrides=[
                    Parameter("task_config", value=str(config_path)),
                    Parameter("backend", value="mock"),
                    Parameter("qr_result_file", value=str(qr_path)),
                    Parameter("qr_observation_settle_s", value=0.0),
                    Parameter("meter_results_dir", value=str(Path(tmp) / "meter_results")),
                ]
            )
            node.camera_cache.color = make_rgb_image()
            node.camera_cache.depth = object()
            node.camera_cache.camera_info = object()
            node._make_robot_adapter = lambda backend_name: robot
            resolver_observed_poses = []

            def resolve(config, step):
                del config
                resolver_observed_poses.append(robot.current_pose.as_list())
                details = {}
                if step["name"] == "middle_black_knob":
                    details = {
                        "pixel": [320.0, 215.0],
                        "desired_pixel": [320.0, 215.0],
                        "bbox": [310, 205, 325, 220],
                        "pixel_count": 125,
                        "pixel_error_base_m": [0.0, 0.0, 0.0],
                    }
                return ElementTarget(
                    step["name"],
                    step["target"]["kind"],
                    Pose(0.3, 0.0, 0.4, 3.1416, 0.0, 0.0, frame="base_link"),
                    details=details,
                )

            node.vision_target_resolver.resolve = resolve
            try:
                response = node._handle_run_sequence_from_qr(
                    Trigger.Request(),
                    Trigger.Response(),
                )
            finally:
                node.destroy_node()

        self.assertTrue(response.success)
        payload = json.loads(response.message)
        self.assertEqual(payload["qr_target_count"], 4)
        self.assertEqual(robot.commands[0].name, "move_joint_pose")
        self.assertEqual(
            [command.name for command in robot.commands].count("move_joint_pose"),
            4,
        )
        self.assertEqual(len(resolver_observed_poses), 3)
        for observed, item in zip(resolver_observed_poses, positions[:3]):
            self.assertAlmostEqual(observed[0], item["position"]["x"])
            self.assertAlmostEqual(observed[1], item["position"]["y"])
            self.assertAlmostEqual(observed[2], item["position"]["z"])
            self.assertAlmostEqual(observed[5], yaw)
        self.assertEqual(payload["results"][3]["skill"], "read_meter")
        self.assertEqual(len(payload["meter_readings"]), 1)

    def test_trigger_callback_executes_named_qr_sequence_by_yaml_order(self):
        config_path = (
            Path(__file__).resolve().parents[1]
            / "config"
            / "ros2_qr_named_panel_demo.yaml"
        )
        positions = [
            {
                "name": "upper_meters",
                "x": 0.40,
                "y": 0.50,
                "z": 0.60,
                "roll": 0.0,
                "pitch": 0.0,
                "yaw": 0.3,
            },
            {
                "name": "rightmost_green_rocker",
                "x": 0.30,
                "y": 0.40,
                "z": 0.50,
                "roll": 0.0,
                "pitch": 0.0,
                "yaw": 0.2,
            },
            {
                "name": "lower_left_red_button",
                "x": 0.10,
                "y": 0.20,
                "z": 0.30,
                "roll": 0.0,
                "pitch": 0.0,
                "yaw": 0.0,
            },
            {
                "name": "middle_black_knob",
                "x": 0.20,
                "y": 0.30,
                "z": 0.40,
                "roll": 0.0,
                "pitch": 0.0,
                "yaw": 0.1,
            },
            {
                "name": "unused_extra_pose",
                "x": 9.0,
                "y": 9.0,
                "z": 9.0,
                "roll": 0.0,
                "pitch": 0.0,
                "yaw": 0.0,
            },
        ]

        with tempfile.TemporaryDirectory() as tmp:
            qr_path = Path(tmp) / "qr_named.json"
            qr_path.write_text(
                json.dumps({"qr_data": json.dumps({"position": positions})}),
                encoding="utf-8",
            )
            robot = MockRobotAdapter(contact_after_servo_calls=2)
            node = CabinetTaskNode(
                parameter_overrides=[
                    Parameter("task_config", value=str(config_path)),
                    Parameter("backend", value="mock"),
                    Parameter("qr_result_file", value=str(qr_path)),
                    Parameter("qr_observation_settle_s", value=0.0),
                    Parameter("meter_results_dir", value=str(Path(tmp) / "meter_results")),
                ]
            )
            node.camera_cache.color = make_rgb_image()
            node.camera_cache.depth = object()
            node.camera_cache.camera_info = object()
            node._make_robot_adapter = lambda backend_name: robot
            resolver_observed_poses = []

            def resolve(config, step):
                del config
                resolver_observed_poses.append(robot.current_pose.as_list())
                details = {}
                if step["name"] == "middle_black_knob":
                    details = {
                        "pixel": [320.0, 215.0],
                        "desired_pixel": [320.0, 215.0],
                        "bbox": [310, 205, 325, 220],
                        "pixel_count": 125,
                        "pixel_error_base_m": [0.0, 0.0, 0.0],
                    }
                return ElementTarget(
                    step["name"],
                    step["target"]["kind"],
                    Pose(0.3, 0.0, 0.4, 3.1416, 0.0, 0.0, frame="base_link"),
                    details=details,
                )

            node.vision_target_resolver.resolve = resolve
            try:
                response = node._handle_run_sequence_from_qr(
                    Trigger.Request(),
                    Trigger.Response(),
                )
            finally:
                node.destroy_node()

        self.assertTrue(response.success)
        payload = json.loads(response.message)
        self.assertEqual(payload["qr_target_count"], 5)
        self.assertEqual(
            [item["skill"] for item in payload["results"]],
            ["read_meter", "knob_camera_servo", "toggle", "button"],
        )
        self.assertEqual(
            resolver_observed_poses,
            [
                [0.20, 0.30, 0.40, 0.0, 0.0, 0.1],
                [0.30, 0.40, 0.50, 0.0, 0.0, 0.2],
                [0.10, 0.20, 0.30, 0.0, 0.0, 0.0],
            ],
        )
        self.assertEqual(len(payload["meter_readings"]), 1)
        self.assertTrue(Path(payload["meter_readings"][0]["path"]).is_file())

    def test_quaternion_pose_to_ros_pose_preserves_quaternion_orientation(self):
        msg = quaternion_pose_to_ros_pose(
            {"x": 0.1, "y": 0.2, "z": 0.3},
            {"x": 0.0, "y": 2.0, "z": 0.0, "w": 0.0},
        )

        self.assertAlmostEqual(msg.position.x, 0.1)
        self.assertAlmostEqual(msg.position.y, 0.2)
        self.assertAlmostEqual(msg.position.z, 0.3)
        self.assertAlmostEqual(msg.orientation.x, 0.0)
        self.assertAlmostEqual(msg.orientation.y, 1.0)
        self.assertAlmostEqual(msg.orientation.z, 0.0)
        self.assertAlmostEqual(msg.orientation.w, 0.0)

    def test_qr_sequence_fails_when_camera_cache_not_ready(self):
        config_path = (
            Path(__file__).resolve().parents[1]
            / "config"
            / "ros2_qr_vision_cabinet_demo.yaml"
        )
        positions = [
            {"x": 0.10, "y": 0.20, "z": 0.30, "roll": 0.0, "pitch": 0.0, "yaw": 0.0},
            {"x": 0.11, "y": 0.21, "z": 0.31, "roll": 0.0, "pitch": 0.0, "yaw": 0.1},
            {"x": 0.12, "y": 0.22, "z": 0.32, "roll": 0.0, "pitch": 0.0, "yaw": 0.2},
            {"x": 0.13, "y": 0.23, "z": 0.33, "roll": 0.0, "pitch": 0.0, "yaw": 0.3},
        ]

        with tempfile.TemporaryDirectory() as tmp:
            qr_path = Path(tmp) / "qr_mock.json"
            qr_path.write_text(
                json.dumps({"qr_data": json.dumps({"position": positions})}),
                encoding="utf-8",
            )
            robot = MockRobotAdapter(contact_after_servo_calls=2)
            node = CabinetTaskNode(
                parameter_overrides=[
                    Parameter("task_config", value=str(config_path)),
                    Parameter("backend", value="mock"),
                    Parameter("qr_result_file", value=str(qr_path)),
                    Parameter("qr_observation_settle_s", value=0.0),
                ]
            )
            node._make_robot_adapter = lambda backend_name: robot
            try:
                response = node._handle_run_sequence_from_qr(
                    Trigger.Request(),
                    Trigger.Response(),
                )
            finally:
                node.destroy_node()

        self.assertFalse(response.success)
        payload = json.loads(response.message)
        self.assertEqual(payload["event"], "qr_sequence_failed")
        self.assertIn("color/depth/camera_info are not ready", payload["error"])
        self.assertEqual(
            [command.name for command in robot.commands],
            ["move_pose"],
        )

    def test_qr_sequence_fails_before_skill_when_vision_target_not_found(self):
        config_path = (
            Path(__file__).resolve().parents[1]
            / "config"
            / "ros2_qr_vision_cabinet_demo.yaml"
        )
        positions = [
            {"x": 0.10, "y": 0.20, "z": 0.30, "roll": 0.0, "pitch": 0.0, "yaw": 0.0},
            {"x": 0.11, "y": 0.21, "z": 0.31, "roll": 0.0, "pitch": 0.0, "yaw": 0.1},
            {"x": 0.12, "y": 0.22, "z": 0.32, "roll": 0.0, "pitch": 0.0, "yaw": 0.2},
            {"x": 0.13, "y": 0.23, "z": 0.33, "roll": 0.0, "pitch": 0.0, "yaw": 0.3},
        ]

        with tempfile.TemporaryDirectory() as tmp:
            qr_path = Path(tmp) / "qr_mock.json"
            qr_path.write_text(
                json.dumps({"qr_data": json.dumps({"position": positions})}),
                encoding="utf-8",
            )
            robot = MockRobotAdapter(contact_after_servo_calls=2)
            node = CabinetTaskNode(
                parameter_overrides=[
                    Parameter("task_config", value=str(config_path)),
                    Parameter("backend", value="mock"),
                    Parameter("qr_result_file", value=str(qr_path)),
                    Parameter("qr_observation_settle_s", value=0.0),
                ]
            )
            node.camera_cache.color = make_rgb_image()
            node.camera_cache.depth = object()
            node.camera_cache.camera_info = object()
            node._make_robot_adapter = lambda backend_name: robot
            node.vision_target_resolver.resolve = lambda config, step: (_ for _ in ()).throw(
                RuntimeError("target not found")
            )
            try:
                response = node._handle_run_sequence_from_qr(
                    Trigger.Request(),
                    Trigger.Response(),
                )
            finally:
                node.destroy_node()

        self.assertFalse(response.success)
        payload = json.loads(response.message)
        self.assertEqual(payload["event"], "qr_sequence_failed")
        self.assertIn("target not found", payload["error"])
        self.assertEqual(
            [command.name for command in robot.commands],
            ["move_pose"],
        )

    def test_resolve_targets_reports_camera_not_ready(self):
        config_path = (
            Path(__file__).resolve().parents[1]
            / "config"
            / "ros2_vision_button_demo.yaml"
        )
        node = CabinetTaskNode(
            parameter_overrides=[
                Parameter("task_config", value=str(config_path)),
                Parameter("backend", value="mock"),
            ]
        )
        try:
            response = node._handle_resolve_targets(
                Trigger.Request(),
                Trigger.Response(),
            )
        finally:
            node.destroy_node()

        self.assertFalse(response.success)
        payload = json.loads(response.message)
        self.assertEqual(payload["event"], "target_resolution_failed")
        self.assertIn("color/depth/camera_info are not ready", payload["error"])
        self.assertNotIn("bool", payload["error"])


if __name__ == "__main__":
    unittest.main()
