from __future__ import annotations

import json
import time
from contextlib import contextmanager
from datetime import datetime
from pathlib import Path
from typing import Any

import rclpy
from geometry_msgs.msg import Pose as RosPose
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Empty, Float64, String
from std_srvs.srv import SetBool, Trigger
from tf2_ros import Buffer, TransformListener

from rg_cabinet.adapters.mock import MockRobotAdapter
from rg_cabinet.adapters.robot import RobotAdapter
from rg_cabinet.config import load_task_config
from rg_cabinet.geometry import Pose
from rg_cabinet.perception.element_detector import ElementTarget
from rg_cabinet.qr_targets import load_qr_target_set, overlay_qr_observation_poses
from rg_cabinet.task_runner import run_sequence
from rg_cabinet.verification import MockStateVerifier
from rg_cabinet_ros2.backends import (
    MoveIt2PlaceholderRobotAdapter,
    RmDriverRobotAdapter,
    quaternion_pose_to_ros_pose,
    ros_pose_to_cabinet_pose,
)
from rg_cabinet_ros2.task_reporting import status_summary, save_diagnostic
from rg_cabinet_ros2.camera_cache import CameraFrameCache
from rg_cabinet_ros2.image_tools import color_image_extension, write_color_image
from rg_cabinet_ros2.vision_target_resolver import VisionTargetResolver


def default_task_config_path() -> str:
    try:
        from ament_index_python.packages import get_package_share_directory

        return str(
            Path(get_package_share_directory("realman_gemini_cabinet"))
            / "config"
            / "ros2_mock_demo.yaml"
        )
    except Exception:
        return str(Path(__file__).resolve().parents[2] / "config" / "ros2_mock_demo.yaml")


def default_qr_results_dir() -> str:
    source_tree_dir = Path(__file__).resolve().parents[3] / "qr_detector" / "qr_results"
    if source_tree_dir.is_dir():
        return str(source_tree_dir)
    cwd_dir = Path.cwd() / "src" / "qr_detector" / "qr_results"
    if cwd_dir.is_dir():
        return str(cwd_dir)
    return str(source_tree_dir)


class CabinetTaskNode(Node):
    def __init__(self, **kwargs: Any) -> None:
        super().__init__("cabinet_task_node", **kwargs)
        self._declare_parameters()

        self.camera_cache = CameraFrameCache()
        self.latest_force: list[float] | None = None
        self.latest_joint_current: list[float] | None = None
        self.latest_tcp_pose: Pose | None = None
        self.io_callback_group = ReentrantCallbackGroup()
        self.rm_driver_callback_group = ReentrantCallbackGroup()
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.vision_target_resolver = VisionTargetResolver(
            self,
            self.camera_cache,
            self.tf_buffer,
        )
        self._manual_robot_adapter: RobotAdapter | None = None
        self._active_sequence_step_debug: dict[str, Any] | None = None
        self.last_motion_command_debug: dict[str, Any] | None = None
        self.last_alignment_preflight_debug: dict[str, Any] | None = None
        self._active_task_stage = "prepare"

        self.status_pub = self.create_publisher(String, "/rg_cabinet/status", 10)
        self.move_stop_pub = self.create_publisher(
            Empty, self.get_parameter("move_stop_topic").value, 10
        )

        self.create_subscription(
            Image,
            self.get_parameter("camera_color_topic").value,
            self._color_cb,
            10,
            callback_group=self.io_callback_group,
        )
        self.create_subscription(
            Image,
            self.get_parameter("camera_depth_topic").value,
            self._depth_cb,
            10,
            callback_group=self.io_callback_group,
        )
        self.create_subscription(
            CameraInfo,
            self.get_parameter("camera_info_topic").value,
            self._camera_info_cb,
            10,
            callback_group=self.io_callback_group,
        )
        self.create_subscription(
            RosPose,
            self.get_parameter("tcp_pose_topic").value,
            self._tcp_pose_cb,
            10,
            callback_group=self.io_callback_group,
        )
        self.create_subscription(
            Float64,
            "/rg_cabinet/gripper_position_cmd",
            self._gripper_position_cmd_cb,
            10,
            callback_group=self.io_callback_group,
        )
        self._create_optional_rm_state_subscriptions()

        self.create_service(
            Trigger,
            "/rg_cabinet/run_sequence",
            self._handle_run_sequence,
        )
        self.create_service(
            Trigger,
            "/rg_cabinet/run_sequence_from_qr",
            self._handle_run_sequence_from_qr,
        )
        self.create_service(
            Trigger,
            "/rg_cabinet/resolve_targets",
            self._handle_resolve_targets,
        )
        self.create_service(
            SetBool,
            "/rg_cabinet/set_gripper",
            self._handle_set_gripper,
        )
        self.create_service(
            Trigger,
            "/rg_cabinet/open_gripper",
            self._handle_open_gripper,
        )
        self.create_service(
            Trigger,
            "/rg_cabinet/close_gripper",
            self._handle_close_gripper,
        )
        self._publish_status(
            {
                "event": "node_ready",
                "backend": self.get_parameter("backend").value,
                "gripper_backend": self.get_parameter("gripper_backend").value,
                "task_config": self.get_parameter("task_config").value,
                "camera_color_topic": self.get_parameter("camera_color_topic").value,
                "camera_depth_topic": self.get_parameter("camera_depth_topic").value,
                "camera_info_topic": self.get_parameter("camera_info_topic").value,
                "camera_frame": self.get_parameter("camera_frame").value,
            }
        )

    def _declare_parameters(self) -> None:
        defaults = {
            "task_config": default_task_config_path(),
            "backend": "rm_driver",
            "mock_contact_after_servo_calls": 2,
            "qr_result_file": "",
            "qr_results_dir": default_qr_results_dir(),
            "qr_observation_settle_s": 0.5,
            "meter_results_dir": str(Path.cwd() / "meter_results"),
            "task_log_dir": str(Path.cwd() / "task_logs"),
            "camera_color_topic": "/camera_gemini/color/image_raw",
            "camera_depth_topic": "/camera_gemini/depth/image_raw",
            "camera_info_topic": "/camera_gemini/color/camera_info",
            "camera_frame": "camera_gemini_color_optical_frame",
            "vision_enabled": True,
            "vision_tf_timeout_s": 1.0,
            "force_topic": "/rm_driver/udp_six_force",
            "joint_current_topic": "/rm_driver/udp_joint_current",
            "tcp_pose_topic": "/rm_driver/udp_arm_position",
            "move_stop_topic": "/rm_driver/move_stop_cmd",
            "planning_group": "rm_group",
            "trajectory_action": "/rm_group_controller/follow_joint_trajectory",
            "base_frame": "base_link",
            "tool_frame": "Link6",
            "rm_driver_topic_prefix": "/rm_driver",
            "rm_driver_dof": 6,
            "rm_driver_default_speed_percent": 5,
            "rm_driver_max_speed_percent": 20,
            "rm_driver_linear_speed_scale_mps": 0.25,
            "rm_driver_angular_speed_scale_radps": 1.0,
            "rm_driver_servo_motion_mode": "movel",
            "rm_driver_result_timeout_s": 20.0,
            "rm_driver_motion_timeout_margin_s": 10.0,
            "rm_driver_motion_timeout_scale": 2.0,
            "rm_driver_motion_timeout_max_s": 120.0,
            "rm_driver_stop_on_timeout": True,
            "rm_driver_discovery_timeout_s": 5.0,
            "rm_driver_wait_for_subscribers": True,
            "rm_driver_gripper_timeout_ms": 3000,
            "rm_driver_gripper_min_position": 1,
            "rm_driver_gripper_max_position": 1000,
            "gripper_backend": "socket_json",
            "gripper_ip": "192.168.1.18",
            "gripper_port": 8080,
            "gripper_device_id": 1,
            "gripper_pos_open": 0,
            "gripper_pos_close": 9000,
            "gripper_command_timeout_s": 2.0,
            "gripper_modbus_port": 1,
            "gripper_baudrate": 115200,
            "gripper_modbus_timeout_s": 2,
            "gripper_voltage_type": 3,
            "gripper_default_speed": 100,
            "gripper_default_force": 50,
            "gripper_acceleration": 1000,
            "gripper_deacceleration": 1000,
            "gripper_init_on_connect": True,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)

    def _create_optional_rm_state_subscriptions(self) -> None:
        try:
            from rm_ros_interfaces.msg import Jointcurrent, Sixforce
        except Exception as exc:
            self.get_logger().warn(
                "rm_ros_interfaces is not available; force/current safety "
                f"subscriptions are disabled: {exc}"
            )
            return

        self.create_subscription(
            Sixforce,
            self.get_parameter("force_topic").value,
            self._force_cb,
            10,
            callback_group=self.io_callback_group,
        )
        self.create_subscription(
            Jointcurrent,
            self.get_parameter("joint_current_topic").value,
            self._joint_current_cb,
            10,
            callback_group=self.io_callback_group,
        )

    def _color_cb(self, msg: Image) -> None:
        self.camera_cache.color = msg

    def _depth_cb(self, msg: Image) -> None:
        self.camera_cache.depth = msg

    def _camera_info_cb(self, msg: CameraInfo) -> None:
        self.camera_cache.camera_info = msg

    def _tcp_pose_cb(self, msg: RosPose) -> None:
        self.latest_tcp_pose = ros_pose_to_cabinet_pose(
            msg,
            frame=self.get_parameter("base_frame").value,
        )

    def _force_cb(self, msg: Any) -> None:
        self.latest_force = [
            float(msg.force_fx),
            float(msg.force_fy),
            float(msg.force_fz),
            float(msg.force_mx),
            float(msg.force_my),
            float(msg.force_mz),
        ]

    def _joint_current_cb(self, msg: Any) -> None:
        self.latest_joint_current = [float(v) for v in msg.joint_current]

    def _gripper_position_cmd_cb(self, msg: Float64) -> None:
        position = min(1.0, max(0.0, float(msg.data)))
        try:
            self.set_gripper_position(position)
        except Exception as exc:
            self._publish_status(
                {
                    "event": "manual_gripper_failed",
                    "success": False,
                    "backend": self.get_parameter("backend").value,
                    "gripper_backend": self.get_parameter("gripper_backend").value,
                    "position": position,
                    "error": str(exc),
                }
            )

    def publish_motion_stop(self) -> None:
        self.move_stop_pub.publish(Empty())

    def set_gripper_position(self, position: float) -> dict[str, Any]:
        normalized = min(1.0, max(0.0, float(position)))
        robot = self._manual_robot()
        robot.set_gripper(normalized)
        payload: dict[str, Any] = {
            "event": "manual_gripper_set",
            "success": True,
            "backend": self.get_parameter("backend").value,
            "gripper_backend": self.get_parameter("gripper_backend").value,
            "position": normalized,
            "gripper_pos_open": self.get_parameter("gripper_pos_open").value,
            "gripper_pos_close": self.get_parameter("gripper_pos_close").value,
            "gripper_ip": self.get_parameter("gripper_ip").value,
            "gripper_device_id": self.get_parameter("gripper_device_id").value,
            "gripper_modbus_port": self.get_parameter("gripper_modbus_port").value,
            "gripper_debug": getattr(robot, "last_gripper_debug", None),
        }
        self._publish_status(payload)
        return payload

    def resolve_configured_targets(self) -> tuple[bool, dict[str, Any]]:
        config_path = self.get_parameter("task_config").value
        config = load_task_config(config_path)
        self.vision_target_resolver.reset()
        resolved = []
        for step in config.get("sequence", []):
            target = self.vision_target_resolver.resolve(config, step)
            if target is not None:
                resolved.append(
                    {
                        "name": target.name,
                        "kind": target.kind,
                        "pose": target.pose.as_list(),
                        "frame": target.pose.frame,
                        "confidence": target.confidence,
                    }
                )
        payload: dict[str, Any] = {
            "event": "targets_resolved",
            "success": True,
            "task_config": config_path,
            "camera": self.camera_cache.summary(),
            "targets": resolved,
            "vision_targets": list(self.vision_target_resolver.resolved_targets),
        }
        self._publish_status(payload)
        return True, payload

    def execute_configured_sequence(self) -> tuple[bool, dict[str, Any]]:
        self.last_alignment_preflight_debug = None
        self._active_sequence_step_debug = None
        self.last_motion_command_debug = None
        config_path = self.get_parameter("task_config").value
        backend_name = self.get_parameter("backend").value
        config = load_task_config(config_path)
        verifier = MockStateVerifier(config.get("mock_states", {}))
        with self._sequence_robot_adapter(backend_name) as robot:
            self.vision_target_resolver.reset()
            results = run_sequence(
                config,
                robot,
                verifier=verifier,
                target_resolver=lambda step: self.vision_target_resolver.resolve(config, step),
                before_step=self._set_active_sequence_step,
                camera_cache=self.camera_cache,
            )
            success = all(item["success"] for item in results)
            payload: dict[str, Any] = {
                "event": "sequence_complete",
                "success": success,
                "backend": backend_name,
                "task_config": config_path,
                "camera": self.camera_cache.summary(),
                "results": results,
            }
            if self.vision_target_resolver.resolved_targets:
                payload["vision_targets"] = list(self.vision_target_resolver.resolved_targets)
            if isinstance(robot, MockRobotAdapter):
                payload["recorded_commands"] = len(robot.commands)
            self._publish_status(payload)
            return success, payload

    def execute_sequence_from_qr(self) -> tuple[bool, dict[str, Any]]:
        self.last_alignment_preflight_debug = None
        self._active_sequence_step_debug = None
        self.last_motion_command_debug = None
        config_path = self.get_parameter("task_config").value
        backend_name = self.get_parameter("backend").value
        qr_result_file = self.get_parameter("qr_result_file").value
        qr_results_dir = self.get_parameter("qr_results_dir").value
        base_frame = self.get_parameter("base_frame").value
        if not qr_result_file and not qr_results_dir:
            qr_results_dir = default_qr_results_dir()

        config = load_task_config(config_path)
        targets = load_qr_target_set(
            qr_result_file=qr_result_file or None,
            qr_results_dir=qr_results_dir or None,
            frame=base_frame,
        )
        qr_config = overlay_qr_observation_poses(config, targets)
        verifier = MockStateVerifier(qr_config.get("mock_states", {}))
        with self._sequence_robot_adapter(backend_name) as robot:
            self.vision_target_resolver.reset()
            meter_readings: list[dict[str, Any]] = []
            results = run_sequence(
                qr_config,
                robot,
                verifier=verifier,
                before_step=lambda step: self._before_qr_step(step, robot, meter_readings, qr_config),
                target_resolver=lambda step: self._resolve_qr_vision_target(qr_config, step),
                camera_cache=self.camera_cache,
            )
            self._attach_meter_readings_to_results(qr_config, results)
            success = all(item["success"] for item in results)
            payload: dict[str, Any] = {
                "event": "qr_sequence_complete",
                "success": success,
                "backend": backend_name,
                "task_config": config_path,
                "qr_source_file": targets.source_file,
                "qr_target_count": len(targets.poses),
                "camera": self.camera_cache.summary(),
                "results": results,
            }
            if meter_readings:
                payload["meter_readings"] = meter_readings
            if self.vision_target_resolver.resolved_targets:
                payload["vision_targets"] = list(self.vision_target_resolver.resolved_targets)
            if isinstance(robot, MockRobotAdapter):
                payload["recorded_commands"] = len(robot.commands)
            self._publish_status(payload)
            return success, payload

    def _before_qr_step(
        self,
        step: dict[str, Any],
        robot: RobotAdapter,
        meter_readings: list[dict[str, Any]],
        config: dict[str, Any] | None = None,
    ) -> None:
        self._set_active_sequence_step(step)
        is_meter_step = str(step.get("skill", "")) == "read_meter"
        # Joint-space steps carry an absolute joint target and are executed
        # directly by their skill; they have no QR observation pose to visit.
        if self._step_skips_observation_pose(step):
            return
        self._move_to_qr_observation_pose(
            step,
            robot,
            require_full_camera=not is_meter_step,
            fresh_sampling=(not is_meter_step and config is not None
                            and self.vision_target_resolver.fresh_sampling_settings(config, step) is not None),
        )
        if is_meter_step:
            reading = self._capture_meter_image(step)
            step["_meter_reading"] = reading
            meter_readings.append(reading)

    @staticmethod
    def _step_skips_observation_pose(step: dict[str, Any]) -> bool:
        """True for steps that must not be driven to a QR observation pose.

        A joint-space step (for example the post-sequence safe pose) carries its
        own absolute joint target, so there is no observation pose to visit and
        no QR entry to consume.
        """
        observation_pose = step.get("observation_pose")
        if isinstance(observation_pose, dict) and observation_pose.get("skip"):
            return True
        if observation_pose is not None and not isinstance(observation_pose, dict):
            return False
        return str(step.get("skill", "")) == "move_joint"

    def _move_to_qr_observation_pose(
        self,
        step: dict[str, Any],
        robot: RobotAdapter,
        require_full_camera: bool = True,
        fresh_sampling: bool = False,
    ) -> None:
        self._report_task_stage("observation")
        observation = step.get("observation_pose")
        if not isinstance(observation, dict):
            raise RuntimeError(
                f"QR sequence step {step.get('name', '<unnamed>')} is missing "
                "observation_pose"
            )
        frame = str(observation.get("frame", self.get_parameter("base_frame").value))
        move_speed = float(observation.get("move_speed_mps", 0.05))

        if "position" in observation or "orientation" in observation:
            position, orientation = self._read_observation_quaternion_pose(
                observation,
                step,
            )
            move_joint_pose = getattr(robot, "move_joint_pose", None)
            move_quaternion_pose = getattr(robot, "move_quaternion_pose", None)
            if callable(move_joint_pose):
                move_joint_pose(
                    position,
                    orientation,
                    speed=move_speed,
                    wait=True,
                )
            elif callable(move_quaternion_pose):
                move_quaternion_pose(
                    position,
                    orientation,
                    speed=move_speed,
                    wait=True,
                )
            else:
                pose = ros_pose_to_cabinet_pose(
                    quaternion_pose_to_ros_pose(position, orientation),
                    frame=frame,
                )
                robot.move_pose(pose, speed=move_speed, wait=True)
        elif "pose" in observation:
            pose = Pose.from_sequence(observation["pose"], frame=frame)
            robot.move_pose(pose, speed=move_speed, wait=True)
        else:
            raise RuntimeError(
                f"QR sequence step {step.get('name', '<unnamed>')} "
                "observation_pose is missing pose or position/orientation"
            )

        settle_s = float(
            observation.get(
                "settle_s",
                self.get_parameter("qr_observation_settle_s").value,
            )
        )
        if settle_s > 0.0:
            time.sleep(settle_s)

        if fresh_sampling:
            # The resolver owns the bounded wait, diagnostics and retries. Its
            # timestamp boundary is set after this motion and settling period.
            return

        # ====== [当前已有代码] 通用相机帧等待 ======
        camera_deadline = time.time() + self.get_parameter("vision_tf_timeout_s").value
        if require_full_camera:
            while not self.camera_cache.ready:
                if time.time() > camera_deadline:
                    raise RuntimeError(
                        "QR observation reached but color/depth/camera_info are not ready "
                        f"after {self.get_parameter('vision_tf_timeout_s').value:.1f}s; "
                        f"missing: {[k for k, v in self.camera_cache.summary().items() if not v]}"
                    )
                time.sleep(0.1)
        elif self.camera_cache.color is None:
            while self.camera_cache.color is None:
                if time.time() > camera_deadline:
                    raise RuntimeError(
                        "QR observation reached but color image is not ready "
                        f"after {self.get_parameter('vision_tf_timeout_s').value:.1f}s"
                    )
                time.sleep(0.1)

        # ====== [新增] 等一帧新帧，确保拿到臂到位后的画面 ======
        self.camera_cache.mark_fresh()
        if not self.camera_cache.wait_fresh(timeout_s=2.0):
            self.get_logger().warning(
                f"等待新帧超时 ({2.0}s)，使用缓存帧"
            )

    def _capture_meter_image(self, step: dict[str, Any]) -> dict[str, Any]:
        image = self.camera_cache.color
        if image is None:
            raise RuntimeError("Meter reading requested but color image is not ready")
        params = step.get("params", {})
        save_dir = self._meter_save_dir(params)
        prefix = _safe_filename(str(params.get("image_prefix", step.get("name", "panel_meter"))))
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
        extension = color_image_extension(image)
        path = save_dir / f"{prefix}_{timestamp}{extension}"
        reading = write_color_image(image, path)
        reading.update(
            {
                "step": str(step.get("name", "panel_meter")),
                "timestamp": timestamp,
            }
        )
        self._publish_status(
            {
                "event": "meter_image_saved",
                "success": True,
                "meter_reading": reading,
            }
        )
        return reading

    def _meter_save_dir(self, params: dict[str, Any]) -> Path:
        raw = str(params.get("save_dir") or self.get_parameter("meter_results_dir").value)
        path = Path(raw).expanduser()
        if not path.is_absolute():
            path = Path.cwd() / path
        return path

    def _attach_meter_readings_to_results(
        self,
        config: dict[str, Any],
        results: list[dict[str, Any]],
    ) -> None:
        for step, result in zip(config.get("sequence", []), results):
            reading = step.get("_meter_reading")
            if reading:
                result.setdefault("details", {})["meter_reading"] = reading

    def _read_observation_quaternion_pose(
        self,
        observation: dict[str, Any],
        step: dict[str, Any],
    ) -> tuple[dict[str, float], dict[str, float]]:
        step_name = step.get("name", "<unnamed>")
        position = observation.get("position")
        orientation = observation.get("orientation")
        if not isinstance(position, dict) or not isinstance(orientation, dict):
            raise RuntimeError(
                f"QR sequence step {step_name} observation_pose must contain "
                "position and orientation objects"
            )
        try:
            position_values = {
                "x": float(position["x"]),
                "y": float(position["y"]),
                "z": float(position["z"]),
            }
            orientation_values = {
                "x": float(orientation["x"]),
                "y": float(orientation["y"]),
                "z": float(orientation["z"]),
                "w": float(orientation["w"]),
            }
        except (KeyError, TypeError, ValueError) as exc:
            raise RuntimeError(
                f"QR sequence step {step_name} observation_pose contains an "
                "invalid quaternion pose"
            ) from exc
        return position_values, orientation_values

    def _resolve_qr_vision_target(
        self,
        config: dict[str, Any],
        step: dict[str, Any],
    ) -> ElementTarget:
        if self._active_task_stage in ("prepare", "observation"):
            self._report_task_stage("vision")
        target = self.vision_target_resolver.resolve(config, step)
        if target is None:
            raise RuntimeError(
                f"QR sequence step {step.get('name', '<unnamed>')} must use "
                "target.source camera/vision/gemini2"
            )
        return target

    @contextmanager
    def _sequence_robot_adapter(self, backend_name: str):
        """Release per-sequence ROS entities on both success and failure."""
        robot = self._make_robot_adapter(backend_name)
        try:
            yield robot
        finally:
            if isinstance(robot, RmDriverRobotAdapter):
                robot.close()

    def _make_robot_adapter(self, backend_name: str):
        if backend_name == "mock":
            return MockRobotAdapter(
                contact_after_servo_calls=int(
                    self.get_parameter("mock_contact_after_servo_calls").value
                )
            )
        if backend_name == "moveit2":
            return MoveIt2PlaceholderRobotAdapter(self)
        if backend_name == "rm_driver":
            return RmDriverRobotAdapter(self)
        raise ValueError(f"Unsupported backend: {backend_name}")

    def _manual_robot(self) -> RobotAdapter:
        backend_name = self.get_parameter("backend").value
        if self._manual_robot_adapter is None:
            self._manual_robot_adapter = self._make_robot_adapter(backend_name)
        return self._manual_robot_adapter

    def _handle_run_sequence(
        self,
        request: Trigger.Request,
        response: Trigger.Response,
    ) -> Trigger.Response:
        del request
        try:
            success, payload = self.execute_configured_sequence()
            response.success = success
            response.message = json.dumps(payload, ensure_ascii=False)
        except Exception as exc:
            payload = {
                "event": "sequence_failed",
                "success": False,
                "error": str(exc),
                "backend": self.get_parameter("backend").value,
                "task_config": self.get_parameter("task_config").value,
            }
            self._attach_execution_debug(payload)
            self._publish_status(payload)
            response.success = False
            response.message = json.dumps(payload, ensure_ascii=False)
        return response

    def _handle_run_sequence_from_qr(
        self,
        request: Trigger.Request,
        response: Trigger.Response,
    ) -> Trigger.Response:
        del request
        try:
            success, payload = self.execute_sequence_from_qr()
            response.success = success
            response.message = json.dumps(payload, ensure_ascii=False)
        except Exception as exc:
            payload = {
                "event": "qr_sequence_failed",
                "success": False,
                "error": str(exc),
                "backend": self.get_parameter("backend").value,
                "task_config": self.get_parameter("task_config").value,
                "qr_result_file": self.get_parameter("qr_result_file").value,
                "qr_results_dir": self.get_parameter("qr_results_dir").value,
            }
            self._attach_execution_debug(payload)
            self._publish_status(payload)
            response.success = False
            response.message = json.dumps(payload, ensure_ascii=False)
        return response

    def _handle_set_gripper(
        self,
        request: SetBool.Request,
        response: SetBool.Response,
    ) -> SetBool.Response:
        position = 1.0 if request.data else 0.0
        try:
            payload = self.set_gripper_position(position)
            response.success = True
            response.message = json.dumps(payload, ensure_ascii=False)
        except Exception as exc:
            payload = {
                "event": "manual_gripper_failed",
                "success": False,
                "backend": self.get_parameter("backend").value,
                "gripper_backend": self.get_parameter("gripper_backend").value,
                "position": position,
                "error": str(exc),
            }
            self._publish_status(payload)
            response.success = False
            response.message = json.dumps(payload, ensure_ascii=False)
        return response

    def _handle_open_gripper(
        self,
        request: Trigger.Request,
        response: Trigger.Response,
    ) -> Trigger.Response:
        del request
        try:
            payload = self.set_gripper_position(0.0)
            response.success = True
            response.message = json.dumps(payload, ensure_ascii=False)
        except Exception as exc:
            payload = {
                "event": "manual_gripper_failed",
                "success": False,
                "backend": self.get_parameter("backend").value,
                "gripper_backend": self.get_parameter("gripper_backend").value,
                "position": 0.0,
                "error": str(exc),
            }
            self._publish_status(payload)
            response.success = False
            response.message = json.dumps(payload, ensure_ascii=False)
        return response

    def _handle_close_gripper(
        self,
        request: Trigger.Request,
        response: Trigger.Response,
    ) -> Trigger.Response:
        del request
        try:
            payload = self.set_gripper_position(1.0)
            response.success = True
            response.message = json.dumps(payload, ensure_ascii=False)
        except Exception as exc:
            payload = {
                "event": "manual_gripper_failed",
                "success": False,
                "backend": self.get_parameter("backend").value,
                "gripper_backend": self.get_parameter("gripper_backend").value,
                "position": 1.0,
                "error": str(exc),
            }
            self._publish_status(payload)
            response.success = False
            response.message = json.dumps(payload, ensure_ascii=False)
        return response

    def _handle_resolve_targets(
        self,
        request: Trigger.Request,
        response: Trigger.Response,
    ) -> Trigger.Response:
        del request
        try:
            success, payload = self.resolve_configured_targets()
            response.success = success
            response.message = json.dumps(payload, ensure_ascii=False)
        except Exception as exc:
            payload = {
                "event": "target_resolution_failed",
                "success": False,
                "error": str(exc),
                "task_config": self.get_parameter("task_config").value,
            }
            self._attach_vision_debug(payload)
            self._publish_status(payload)
            response.success = False
            response.message = json.dumps(payload, ensure_ascii=False)
        return response

    def _set_active_sequence_step(self, step: dict[str, Any]) -> None:
        target = step.get("target")
        target_debug: dict[str, Any] = {}
        if isinstance(target, dict):
            for key in ("source", "detection", "kind"):
                if key in target:
                    target_debug[key] = target[key]
        debug: dict[str, Any] = {
            "name": str(step.get("name", "<unnamed>")),
            "skill": str(step.get("skill", "")),
        }
        if target_debug:
            debug["target"] = target_debug
        self._active_sequence_step_debug = debug
        self.last_motion_command_debug = None
        self.last_alignment_preflight_debug = None
        self._report_task_stage("prepare")

    def _attach_execution_debug(self, payload: dict[str, Any]) -> None:
        preflight = getattr(self, "last_alignment_preflight_debug", None)
        if preflight is not None:
            payload["alignment_preflight"] = dict(preflight)
        payload["stage"] = getattr(self, "_active_task_stage", "prepare")
        self._attach_sequence_step_debug(payload)
        self._attach_motion_debug(payload)
        self._attach_vision_debug(payload)

    def _attach_sequence_step_debug(self, payload: dict[str, Any]) -> None:
        debug = self._active_sequence_step_debug
        if debug is None:
            return
        payload["step"] = debug.get("name")
        payload["skill"] = debug.get("skill")
        payload["active_step"] = dict(debug)

    def _attach_motion_debug(self, payload: dict[str, Any]) -> None:
        debug = self.last_motion_command_debug
        if debug is not None:
            payload["motion_command_debug"] = dict(debug)

    def _attach_vision_debug(self, payload: dict[str, Any]) -> None:
        debug = self.vision_target_resolver.last_target_debug
        if debug is not None:
            payload["vision_target_debug"] = debug
        if self.vision_target_resolver.resolved_targets:
            payload["vision_targets"] = list(self.vision_target_resolver.resolved_targets)

    def _report_task_stage(self, stage: str) -> None:
        self._active_task_stage = stage
        step = self._active_sequence_step_debug or {}
        self._publish_status({"event": "task_stage", "step": step.get("name"), "stage": stage})

    def _publish_status(self, payload: dict[str, Any]) -> None:
        event = str(payload.get("event", ""))
        is_motion = event.startswith("motion_command")
        if is_motion or event == "alignment_preflight":
            payload.setdefault("stage", getattr(self, "_active_task_stage", "prepare"))
            self._attach_sequence_step_debug(payload)
        payload["summary"] = status_summary(payload)
        # Keep status-topic and service JSON compatible; only console output is compact.
        if not is_motion and event not in ("node_ready", "task_stage"):
            save_diagnostic(payload, self.get_parameter("task_log_dir").value)
            payload["summary"] = status_summary(payload)
        msg = String()
        msg.data = json.dumps(payload, ensure_ascii=False)
        self.status_pub.publish(msg)
        motion = payload.get("motion_command_debug") or {}
        failed = (payload.get("success") is False or motion.get("result") in (False, "timeout")
                  or payload.get("status") == "REJECTED")
        logger = self.get_logger()
        # Humble caches severity per call site: use separate calls for each level.
        if failed:
            logger.error(payload["summary"])
        elif is_motion:
            logger.debug(payload["summary"])
        else:
            logger.info(payload["summary"])
        if payload.get("diagnostic_file"):
            logger.info("诊断文件: " + payload["diagnostic_file"])


def _safe_filename(value: str) -> str:
    cleaned = "".join(ch if ch.isalnum() or ch in ("-", "_") else "_" for ch in value)
    return cleaned.strip("_") or "meter"


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = CabinetTaskNode()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
