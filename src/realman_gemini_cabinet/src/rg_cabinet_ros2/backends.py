from __future__ import annotations

import math
import threading
import time
from typing import Any, Sequence

from rg_cabinet.adapters.robot import ContactThreshold, RobotAdapter
from rg_cabinet.geometry import Pose, Twist, rotate_rpy
from rg_cabinet_ros2.gripper_socket import (
    SocketJsonGripperClient,
    SocketJsonGripperConfig,
)


class MoveIt2PlaceholderRobotAdapter(RobotAdapter):
    """Safe placeholder for the future RealMan MoveIt2 backend.

    This class deliberately refuses motion commands. It lets the ROS2 node load
    and validate MoveIt2-related parameters without accidentally moving hardware.
    """

    def __init__(self, node: Any) -> None:
        self.node = node
        self.planning_group = node.get_parameter("planning_group").value
        self.trajectory_action = node.get_parameter("trajectory_action").value
        self.base_frame = node.get_parameter("base_frame").value
        self.tool_frame = node.get_parameter("tool_frame").value

    def move_pose(
        self,
        pose: Pose,
        speed: float = 0.05,
        acceleration: float = 0.1,
        wait: bool = True,
    ) -> None:
        raise RuntimeError(
            "MoveIt2 backend is a safe placeholder in this package version; "
            "use backend:=mock or implement MoveGroup/FollowJointTrajectory wiring."
        )

    def move_joint(
        self,
        joints: Sequence[float],
        speed: float = 0.2,
        acceleration: float = 0.4,
        wait: bool = True,
    ) -> None:
        raise RuntimeError(
            "MoveIt2 backend is a safe placeholder in this package version."
        )

    def servo_twist(self, twist: Twist, duration_s: float, frame: str = "tool") -> None:
        raise RuntimeError(
            "MoveIt2 backend does not yet provide cartesian servoing in this package."
        )

    def stop(self) -> None:
        self.node.publish_motion_stop()

    def set_gripper(self, position: float, speed: float | None = None) -> None:
        raise RuntimeError(
            "MoveIt2 backend does not yet provide gripper wiring in this package."
        )

    def get_tcp_pose(self) -> Pose:
        raise RuntimeError(
            "MoveIt2 backend does not yet provide TCP state wiring in this package."
        )

    def force_or_current_limit(self, threshold: ContactThreshold) -> bool:
        force = self.node.latest_force
        currents = self.node.latest_joint_current
        if threshold.max_force_n is not None and force is not None:
            if max(abs(v) for v in force[:3]) >= threshold.max_force_n:
                return True
        if threshold.max_current_a is not None and currents is not None:
            if max(abs(v) for v in currents) >= threshold.max_current_a:
                return True
        return False


class RmDriverRobotAdapter(RobotAdapter):
    """RealMan hardware backend using rm_driver command/result topics."""

    def execute_movel_alignment_route(self, moves, settings):
        if self.servo_motion_mode not in ("movel", "move_l", "absolute"):
            raise RuntimeError("Alignment rerouting requires absolute MoveL mode")
        from rg_cabinet_ros2.alignment_preflight_runtime import AlignmentPreflight
        checker = getattr(self.node, "_alignment_preflight_checker", None)
        if checker is None:
            checker = AlignmentPreflight(self.node)
            self.node._alignment_preflight_checker = checker
        return checker.execute_route(self, moves, settings)

    def preflight_movel_alignment(self, moves, settings):
        if self.servo_motion_mode not in ("movel", "move_l", "absolute"):
            raise RuntimeError("Alignment preflight requires the existing absolute MoveL mode")
        from rg_cabinet_ros2.alignment_preflight_runtime import AlignmentPreflight
        checker = getattr(self.node, "_alignment_preflight_checker", None)
        if checker is None:
            checker = AlignmentPreflight(self.node)
            self.node._alignment_preflight_checker = checker
        return checker.check(moves, settings)

    def __init__(self, node: Any) -> None:
        from rm_ros_interfaces.msg import Gripperset, Movej, Movejp, Movel, Moveloffset
        from std_msgs.msg import Bool

        self.node = node
        self.prefix = str(node.get_parameter("rm_driver_topic_prefix").value).rstrip("/")
        self.dof = int(node.get_parameter("rm_driver_dof").value)
        self.default_speed_percent = int(
            node.get_parameter("rm_driver_default_speed_percent").value
        )
        self.max_speed_percent = int(node.get_parameter("rm_driver_max_speed_percent").value)
        self.linear_speed_scale_mps = float(
            node.get_parameter("rm_driver_linear_speed_scale_mps").value
        )
        self.angular_speed_scale_radps = float(
            node.get_parameter("rm_driver_angular_speed_scale_radps").value
        )
        self.result_timeout_s = float(
            node.get_parameter("rm_driver_result_timeout_s").value
        )
        self.servo_motion_mode = str(
            _parameter_value(node, "rm_driver_servo_motion_mode", "movel")
        ).strip().lower()
        self.motion_timeout_margin_s = float(
            _parameter_value(node, "rm_driver_motion_timeout_margin_s", 10.0)
        )
        self.motion_timeout_scale = float(
            _parameter_value(node, "rm_driver_motion_timeout_scale", 2.0)
        )
        self.motion_timeout_max_s = float(
            _parameter_value(node, "rm_driver_motion_timeout_max_s", 120.0)
        )
        self.stop_on_timeout = _bool_parameter_value(
            node,
            "rm_driver_stop_on_timeout",
            True,
        )
        self.discovery_timeout_s = float(
            node.get_parameter("rm_driver_discovery_timeout_s").value
        )
        self.wait_for_subscribers = bool(
            node.get_parameter("rm_driver_wait_for_subscribers").value
        )
        self.gripper_timeout_ms = int(
            node.get_parameter("rm_driver_gripper_timeout_ms").value
        )
        self.gripper_min_position = int(
            node.get_parameter("rm_driver_gripper_min_position").value
        )
        self.gripper_max_position = int(
            node.get_parameter("rm_driver_gripper_max_position").value
        )
        self.gripper_backend = str(
            _parameter_value(node, "gripper_backend", "socket_json")
        ).strip().lower()
        self.socket_gripper_config = SocketJsonGripperConfig(
            ip=str(_parameter_value(node, "gripper_ip", "192.168.1.18")),
            port=int(_parameter_value(node, "gripper_port", 8080)),
            device_id=int(_parameter_value(node, "gripper_device_id", 1)),
            pos_open=int(_parameter_value(node, "gripper_pos_open", 0)),
            pos_close=int(_parameter_value(node, "gripper_pos_close", 9000)),
            timeout_s=float(_parameter_value(node, "gripper_command_timeout_s", 2.0)),
            modbus_port=int(_parameter_value(node, "gripper_modbus_port", 1)),
            baudrate=int(_parameter_value(node, "gripper_baudrate", 115200)),
            modbus_timeout_s=int(_parameter_value(node, "gripper_modbus_timeout_s", 2)),
            voltage_type=int(_parameter_value(node, "gripper_voltage_type", 3)),
            speed=int(_parameter_value(node, "gripper_default_speed", 100)),
            force=int(_parameter_value(node, "gripper_default_force", 50)),
            acceleration=int(_parameter_value(node, "gripper_acceleration", 1000)),
            deacceleration=int(_parameter_value(node, "gripper_deacceleration", 1000)),
            init_on_connect=_bool_parameter_value(
                node,
                "gripper_init_on_connect",
                True,
            ),
        )
        self._socket_gripper: SocketJsonGripperClient | None = None
        self.last_gripper_debug: dict[str, Any] | None = None
        self._last_pose: Pose | None = None
        self._last_command_debug: dict[str, Any] | None = None

        self._Movej = Movej
        self._Movejp = Movejp
        self._Movel = Movel
        self._Moveloffset = Moveloffset
        self._Gripperset = Gripperset

        self._movej_pub = node.create_publisher(Movej, self._topic("movej_cmd"), 10)
        self._movej_p_pub = node.create_publisher(
            Movejp,
            self._topic("movej_p_cmd"),
            10,
        )
        self._movel_pub = node.create_publisher(Movel, self._topic("movel_cmd"), 10)
        self._movel_offset_pub = node.create_publisher(
            Moveloffset,
            self._topic("movel_offset_cmd"),
            10,
        )
        self._gripper_pub = node.create_publisher(
            Gripperset,
            self._topic("set_gripper_position_cmd"),
            10,
        )

        self._result_events = {
            "movej": threading.Event(),
            "movej_p": threading.Event(),
            "movel": threading.Event(),
            "movel_offset": threading.Event(),
            "gripper": threading.Event(),
            "stop": threading.Event(),
        }
        self._result_values: dict[str, bool | None] = {
            name: None for name in self._result_events
        }
        self._result_subscriptions: dict[str, Any] = {}
        self._subscriptions = [
            self._create_result_subscription(Bool, "movej", "movej_result"),
            self._create_result_subscription(Bool, "movej_p", "movej_p_result"),
            self._create_result_subscription(Bool, "movel", "movel_result"),
            self._create_result_subscription(Bool, "movel_offset", "movel_offset_result"),
            self._create_result_subscription(
                Bool,
                "gripper",
                "set_gripper_position_result",
            ),
            self._create_result_subscription(Bool, "stop", "move_stop_result"),
        ]

    def close(self) -> None:
        """Release owned ROS entities/socket; never issue a hardware command.

        Call only after this adapter's sequence has finished. Node-level state
        subscriptions and the shared alignment checker remain alive.
        """
        for subscription in list(self._subscriptions):
            try:
                self.node.destroy_subscription(subscription)
            except Exception as exc:
                self.node.get_logger().warning(f"Driver subscription cleanup failed: {exc}")
            else:
                self._subscriptions.remove(subscription)
                for name, owned in list(self._result_subscriptions.items()):
                    if owned is subscription:
                        del self._result_subscriptions[name]
        for name in ("_movej_pub", "_movej_p_pub", "_movel_pub", "_movel_offset_pub", "_gripper_pub"):
            publisher = getattr(self, name, None)
            if publisher is not None:
                try:
                    self.node.destroy_publisher(publisher)
                except Exception as exc:
                    self.node.get_logger().warning(f"Driver publisher cleanup failed: {exc}")
                else:
                    setattr(self, name, None)
        if self._socket_gripper is not None:
            try:
                self._socket_gripper.close()
            except Exception as exc:
                self.node.get_logger().warning(f"Gripper socket cleanup failed: {exc}")
            else:
                self._socket_gripper = None

    def move_pose(
        self,
        pose: Pose,
        speed: float = 0.05,
        acceleration: float = 0.1,
        wait: bool = True,
    ) -> None:
        del acceleration
        msg = self._Movel()
        msg.pose = cabinet_pose_to_ros_pose(pose)
        msg.speed = self._speed_percent(speed)
        msg.trajectory_connect = 0
        msg.block = bool(wait)
        start_pose = self._current_pose_or_none()
        distance_m = self._pose_distance(start_pose, pose)
        timeout_s = self._motion_timeout(distance_m=distance_m, speed_mps=speed)
        self._record_command_debug(
            {
                "command": "move_pose",
                "topic": self._topic("movel_cmd"),
                "result_name": "movel",
                "pose": pose.as_list(),
                "frame": pose.frame,
                "start_pose": start_pose.as_list() if start_pose is not None else None,
                "start_frame": start_pose.frame if start_pose is not None else None,
                "distance_m": distance_m,
                "speed_mps": float(speed),
                "speed_percent": int(msg.speed),
                "timeout_s": timeout_s,
                "wait": bool(wait),
            }
        )
        self._publish_and_wait(
            self._movel_pub,
            msg,
            "movel",
            wait=wait,
            timeout_s=timeout_s,
            stop_on_timeout=self.stop_on_timeout,
        )
        self._last_pose = pose

    def move_quaternion_pose(
        self,
        position: dict[str, float],
        orientation: dict[str, float],
        speed: float = 0.05,
        acceleration: float = 0.1,
        wait: bool = True,
    ) -> None:
        del acceleration
        msg = self._Movel()
        msg.pose = quaternion_pose_to_ros_pose(position, orientation)
        msg.speed = self._speed_percent(speed)
        msg.trajectory_connect = 0
        msg.block = bool(wait)
        target_pose = ros_pose_to_cabinet_pose(
            msg.pose,
            frame=self.node.get_parameter("base_frame").value,
        )
        start_pose = self._current_pose_or_none()
        distance_m = self._pose_distance(start_pose, target_pose)
        timeout_s = self._motion_timeout(distance_m=distance_m, speed_mps=speed)
        self._record_command_debug(
            {
                "command": "move_quaternion_pose",
                "topic": self._topic("movel_cmd"),
                "result_name": "movel",
                "position": {key: float(value) for key, value in position.items()},
                "orientation": {
                    key: float(value) for key, value in orientation.items()
                },
                "start_pose": start_pose.as_list() if start_pose is not None else None,
                "start_frame": start_pose.frame if start_pose is not None else None,
                "target_pose": target_pose.as_list(),
                "target_frame": target_pose.frame,
                "distance_m": distance_m,
                "speed_mps": float(speed),
                "speed_percent": int(msg.speed),
                "timeout_s": timeout_s,
                "wait": bool(wait),
            }
        )
        self._publish_and_wait(
            self._movel_pub,
            msg,
            "movel",
            wait=wait,
            timeout_s=timeout_s,
            stop_on_timeout=self.stop_on_timeout,
        )
        self._last_pose = target_pose

    def move_joint_pose(
        self,
        position: dict[str, float],
        orientation: dict[str, float],
        speed: float = 0.05,
        acceleration: float = 0.1,
        wait: bool = True,
    ) -> None:
        del acceleration
        msg = self._Movejp()
        msg.pose = quaternion_pose_to_ros_pose(position, orientation)
        msg.speed = self._speed_percent(speed)
        msg.trajectory_connect = 0
        msg.block = bool(wait)
        target_pose = ros_pose_to_cabinet_pose(
            msg.pose,
            frame=self.node.get_parameter("base_frame").value,
        )
        start_pose = self._current_pose_or_none()
        distance_m = self._pose_distance(start_pose, target_pose)
        timeout_s = self._motion_timeout(distance_m=distance_m, speed_mps=speed)
        self._record_command_debug(
            {
                "command": "move_joint_pose",
                "topic": self._topic("movej_p_cmd"),
                "result_name": "movej_p",
                "position": {key: float(value) for key, value in position.items()},
                "orientation": {
                    key: float(value) for key, value in orientation.items()
                },
                "start_pose": start_pose.as_list() if start_pose is not None else None,
                "start_frame": start_pose.frame if start_pose is not None else None,
                "target_pose": target_pose.as_list(),
                "target_frame": target_pose.frame,
                "distance_m": distance_m,
                "speed_mps": float(speed),
                "speed_percent": int(msg.speed),
                "timeout_s": timeout_s,
                "wait": bool(wait),
            }
        )
        self._publish_and_wait(
            self._movej_p_pub,
            msg,
            "movej_p",
            wait=wait,
            timeout_s=timeout_s,
            stop_on_timeout=self.stop_on_timeout,
        )
        self._last_pose = target_pose

    def move_joint(
        self,
        joints: Sequence[float],
        speed: float = 0.2,
        acceleration: float = 0.4,
        wait: bool = True,
    ) -> None:
        del acceleration
        msg = self._Movej()
        msg.joint = [float(v) for v in joints]
        msg.speed = self._speed_percent(speed)
        msg.block = bool(wait)
        msg.trajectory_connect = 0
        msg.dof = self.dof
        self._record_command_debug(
            {
                "command": "move_joint",
                "topic": self._topic("movej_cmd"),
                "result_name": "movej",
                "joints": list(msg.joint),
                "speed": float(speed),
                "speed_percent": int(msg.speed),
                "wait": bool(wait),
                "dof": int(msg.dof),
            }
        )
        self._publish_and_wait(self._movej_pub, msg, "movej", wait=wait)

    def servo_twist(self, twist: Twist, duration_s: float, frame: str = "tool") -> None:
        if self.servo_motion_mode in ("movel", "move_l", "absolute"):
            self._servo_twist_with_movel(twist, duration_s, frame)
            return
        if self.servo_motion_mode not in ("movel_offset", "offset"):
            raise RuntimeError(
                "Unsupported rm_driver_servo_motion_mode: "
                f"{self.servo_motion_mode}"
            )
        self._servo_twist_with_movel_offset(twist, duration_s, frame)

    def _servo_twist_with_movel(
        self,
        twist: Twist,
        duration_s: float,
        frame: str,
    ) -> None:
        start_pose = self._current_pose_or_none()
        if start_pose is None:
            raise RuntimeError("No TCP pose is available for MoveL servo fallback")
        target_pose = self._target_pose_from_twist(start_pose, twist, duration_s, frame)
        msg = self._Movel()
        msg.pose = cabinet_pose_to_ros_pose(target_pose)
        msg.speed = self._twist_speed_percent(twist)
        msg.trajectory_connect = 0
        msg.block = True
        distance_m = self._twist_linear_distance(twist, duration_s)
        timeout_s = self._motion_timeout(duration_s=duration_s)
        self._record_command_debug(
            {
                "command": "servo_twist",
                "servo_motion_mode": "movel",
                "topic": self._topic("movel_cmd"),
                "result_name": "movel",
                "twist": twist.as_list(),
                "duration_s": float(duration_s),
                "frame": frame,
                "start_pose": start_pose.as_list(),
                "start_frame": start_pose.frame,
                "target_pose": target_pose.as_list(),
                "target_frame": target_pose.frame,
                "distance_m": distance_m,
                "speed_percent": int(msg.speed),
                "timeout_s": timeout_s,
                "wait": True,
            }
        )
        self._publish_and_wait(
            self._movel_pub,
            msg,
            "movel",
            timeout_s=timeout_s,
            stop_on_timeout=self.stop_on_timeout,
        )
        self._last_pose = target_pose

    def _servo_twist_with_movel_offset(
        self,
        twist: Twist,
        duration_s: float,
        frame: str,
    ) -> None:
        msg = self._Moveloffset()
        msg.pose = cabinet_pose_to_ros_pose(
            Pose(
                x=twist.linear_x * duration_s,
                y=twist.linear_y * duration_s,
                z=twist.linear_z * duration_s,
                rx=twist.angular_x * duration_s,
                ry=twist.angular_y * duration_s,
                rz=twist.angular_z * duration_s,
                frame=frame,
            )
        )
        msg.speed = self._twist_speed_percent(twist)
        msg.r = 0
        msg.trajectory_connect = False
        msg.frame_type = frame == "tool"
        msg.block = True
        start_pose = self._current_pose_or_none()
        distance_m = self._twist_linear_distance(twist, duration_s)
        timeout_s = self._motion_timeout(duration_s=duration_s)
        self._record_command_debug(
            {
                "command": "servo_twist",
                "servo_motion_mode": "movel_offset",
                "topic": self._topic("movel_offset_cmd"),
                "result_name": "movel_offset",
                "twist": twist.as_list(),
                "duration_s": float(duration_s),
                "frame": frame,
                "start_pose": start_pose.as_list() if start_pose is not None else None,
                "start_frame": start_pose.frame if start_pose is not None else None,
                "distance_m": distance_m,
                "frame_type_tool": bool(msg.frame_type),
                "speed_percent": int(msg.speed),
                "timeout_s": timeout_s,
                "wait": True,
            }
        )
        self._publish_and_wait(
            self._movel_offset_pub,
            msg,
            "movel_offset",
            timeout_s=timeout_s,
            stop_on_timeout=self.stop_on_timeout,
        )
        self._update_last_pose_from_offset(twist, duration_s, frame)

    def stop(self) -> None:
        self._result_values["stop"] = None
        self._result_events["stop"].clear()
        self.node.publish_motion_stop()
        self._result_events["stop"].wait(timeout=min(1.0, self.result_timeout_s))

    def set_gripper(self, position: float, speed: float | None = None) -> None:
        del speed
        self.last_gripper_debug = None
        if self.gripper_backend in ("socket_json", "socket", "controller_socket"):
            self.last_gripper_debug = self._socket_gripper_client().set_normalized_position(
                position
            )
            return
        if self.gripper_backend not in ("rm_driver", "topic"):
            raise RuntimeError(f"Unsupported gripper_backend: {self.gripper_backend}")
        msg = self._Gripperset()
        msg.position = self._gripper_position(position)
        msg.block = True
        msg.timeout = self.gripper_timeout_ms
        self._publish_and_wait(self._gripper_pub, msg, "gripper")

    def get_tcp_pose(self) -> Pose:
        if self.node.latest_tcp_pose is not None:
            return self.node.latest_tcp_pose
        if self._last_pose is not None:
            return self._last_pose
        raise RuntimeError("No TCP pose has been received from /rm_driver/udp_arm_position")

    def force_or_current_limit(self, threshold: ContactThreshold) -> bool:
        force = self.node.latest_force
        currents = self.node.latest_joint_current
        if threshold.max_force_n is not None and force is not None:
            if max(abs(v) for v in force[:3]) >= threshold.max_force_n:
                return True
        if threshold.max_current_a is not None and currents is not None:
            if max(abs(v) for v in currents) >= threshold.max_current_a:
                return True
        return False

    def _topic(self, name: str) -> str:
        return f"{self.prefix}/{name}"

    def _socket_gripper_client(self) -> SocketJsonGripperClient:
        if self._socket_gripper is None:
            self._socket_gripper = SocketJsonGripperClient(self.socket_gripper_config)
        return self._socket_gripper

    def _create_result_subscription(
        self,
        msg_type: Any,
        result_name: str,
        topic_name: str,
    ) -> Any:
        subscription = self.node.create_subscription(
            msg_type,
            self._topic(topic_name),
            lambda msg, name=result_name: self._record_result(name, msg.data),
            10,
            callback_group=getattr(self.node, "rm_driver_callback_group", None),
        )
        self._result_subscriptions[result_name] = subscription
        return subscription

    def _record_result(self, name: str, value: bool) -> None:
        self._result_values[name] = bool(value)
        self._result_events[name].set()

    def _publish_and_wait(
        self,
        publisher: Any,
        msg: Any,
        result_name: str,
        wait: bool = True,
        timeout_s: float | None = None,
        stop_on_timeout: bool = False,
    ) -> None:
        timeout = float(timeout_s if timeout_s is not None else self.result_timeout_s)
        if self.wait_for_subscribers:
            self._wait_for_subscriber(publisher)
            if wait:
                self._wait_for_result_publisher(result_name)
        self._mark_command_timeout(result_name, timeout)
        self._publish_motion_status("motion_command_started", result_name)
        self._result_values[result_name] = None
        self._result_events[result_name].clear()
        start_time = time.monotonic()
        publisher.publish(msg)
        if not wait:
            self._mark_command_result(
                result_name,
                "not_waited",
                elapsed_s=time.monotonic() - start_time,
            )
            self._publish_motion_status("motion_command_completed", result_name)
            return
        if not self._result_events[result_name].wait(timeout=timeout):
            elapsed_s = time.monotonic() - start_time
            self._mark_command_result(result_name, "timeout", elapsed_s=elapsed_s)
            if stop_on_timeout:
                self._publish_stop_after_timeout()
            self._publish_motion_status("motion_command_completed", result_name)
            raise RuntimeError(
                f"Timed out waiting {timeout:.3f}s for rm_driver {result_name} result. "
                "Check that rm_driver is connected to the arm."
            )
        if self._result_values[result_name] is False:
            self._mark_command_result(
                result_name,
                False,
                elapsed_s=time.monotonic() - start_time,
            )
            self._publish_motion_status("motion_command_completed", result_name)
            raise RuntimeError(f"rm_driver reported {result_name} failure")
        self._mark_command_result(
            result_name,
            True,
            elapsed_s=time.monotonic() - start_time,
        )
        self._publish_motion_status("motion_command_completed", result_name)

    def _record_command_debug(self, payload: dict[str, Any]) -> None:
        debug = dict(payload)
        debug["backend"] = "rm_driver"
        debug["timestamp_s"] = time.time()
        self._last_command_debug = debug
        setattr(self.node, "last_motion_command_debug", debug)

    def _mark_command_timeout(self, result_name: str, timeout_s: float) -> None:
        debug = self._last_command_debug
        if debug is None or debug.get("result_name") != result_name:
            return
        debug["timeout_s"] = float(timeout_s)
        setattr(self.node, "last_motion_command_debug", debug)

    def _mark_command_result(
        self,
        result_name: str,
        value: bool | str,
        elapsed_s: float | None = None,
    ) -> None:
        debug = self._last_command_debug
        if debug is None or debug.get("result_name") != result_name:
            return
        debug["result"] = value
        if elapsed_s is not None:
            debug["elapsed_s"] = float(elapsed_s)
        setattr(self.node, "last_motion_command_debug", debug)

    def report_task_stage(self, stage: str) -> None:
        report = getattr(self.node, "_report_task_stage", None)
        if callable(report):
            report(stage)

    def _publish_motion_status(self, event: str, result_name: str) -> None:
        debug = self._last_command_debug
        if debug is None or debug.get("result_name") != result_name:
            return
        payload = {"event": event, "motion_command_debug": dict(debug)}
        publish_status = getattr(self.node, "_publish_status", None)
        if callable(publish_status):
            publish_status(payload)
            return
        logger = getattr(self.node, "get_logger", lambda: None)()
        if logger is not None and hasattr(logger, "info"):
            logger.info(str(payload))

    def _publish_stop_after_timeout(self) -> None:
        debug = self._last_command_debug
        if debug is not None:
            debug["stop_sent"] = True
            setattr(self.node, "last_motion_command_debug", debug)
        self._result_values["stop"] = None
        self._result_events["stop"].clear()
        self.node.publish_motion_stop()
        self._result_events["stop"].wait(timeout=min(1.0, self.result_timeout_s))

    def _current_pose_or_none(self) -> Pose | None:
        pose = getattr(self.node, "latest_tcp_pose", None)
        if pose is not None:
            return pose
        return self._last_pose

    def _pose_distance(self, start_pose: Pose | None, target_pose: Pose) -> float | None:
        if start_pose is None:
            return None
        return start_pose.distance_to(target_pose)

    def _motion_timeout(
        self,
        distance_m: float | None = None,
        speed_mps: float | None = None,
        duration_s: float | None = None,
    ) -> float:
        timeout = self.result_timeout_s
        if distance_m is not None and speed_mps is not None and speed_mps > 0.0:
            timeout = max(
                timeout,
                distance_m / speed_mps * self.motion_timeout_scale
                + self.motion_timeout_margin_s,
            )
        if duration_s is not None:
            timeout = max(timeout, float(duration_s) + self.motion_timeout_margin_s)
        max_timeout = max(self.result_timeout_s, self.motion_timeout_max_s)
        return min(max(timeout, self.result_timeout_s), max_timeout)

    def _twist_linear_distance(self, twist: Twist, duration_s: float) -> float:
        linear = math.sqrt(
            twist.linear_x * twist.linear_x
            + twist.linear_y * twist.linear_y
            + twist.linear_z * twist.linear_z
        )
        return linear * float(duration_s)

    def _wait_for_subscriber(self, publisher: Any) -> None:
        deadline = time.monotonic() + self.discovery_timeout_s
        while publisher.get_subscription_count() <= 0:
            if time.monotonic() >= deadline:
                raise RuntimeError(
                    "rm_driver command subscriber was not discovered. "
                    "Start the RealMan bringup and check ROS_DOMAIN_ID/network."
                )
            time.sleep(0.05)

    def _wait_for_result_publisher(self, result_name: str) -> None:
        subscription = self._result_subscriptions.get(result_name)
        if subscription is None or not hasattr(subscription, "get_publisher_count"):
            return
        deadline = time.monotonic() + self.discovery_timeout_s
        while subscription.get_publisher_count() <= 0:
            if time.monotonic() >= deadline:
                raise RuntimeError(
                    f"rm_driver {result_name} result publisher was not discovered. "
                    "Start the RealMan bringup and check ROS_DOMAIN_ID/network."
                )
            time.sleep(0.05)

    def _speed_percent(self, speed_mps: float) -> int:
        if speed_mps <= 0.0:
            return self.default_speed_percent
        percent = round(100.0 * speed_mps / max(self.linear_speed_scale_mps, 1e-6))
        return _clamp_int(percent, 1, self.max_speed_percent)

    def _twist_speed_percent(self, twist: Twist) -> int:
        linear = max(abs(twist.linear_x), abs(twist.linear_y), abs(twist.linear_z))
        angular = max(abs(twist.angular_x), abs(twist.angular_y), abs(twist.angular_z))
        linear_percent = 0
        angular_percent = 0
        if linear > 0.0:
            linear_percent = round(100.0 * linear / max(self.linear_speed_scale_mps, 1e-6))
        if angular > 0.0:
            angular_percent = round(
                100.0 * angular / max(self.angular_speed_scale_radps, 1e-6)
            )
        return _clamp_int(
            max(linear_percent, angular_percent, self.default_speed_percent),
            1,
            self.max_speed_percent,
        )

    def _gripper_position(self, normalized_closed: float) -> int:
        closed = min(1.0, max(0.0, float(normalized_closed)))
        span = self.gripper_max_position - self.gripper_min_position
        return int(round(self.gripper_max_position - closed * span))

    def _update_last_pose_from_offset(
        self,
        twist: Twist,
        duration_s: float,
        frame: str,
    ) -> None:
        if self._last_pose is None:
            return
        self._last_pose = self._target_pose_from_twist(
            self._last_pose,
            twist,
            duration_s,
            frame=frame,
        )

    def _target_pose_from_twist(
        self,
        start_pose: Pose,
        twist: Twist,
        duration_s: float,
        frame: str,
    ) -> Pose:
        dx = twist.linear_x * duration_s
        dy = twist.linear_y * duration_s
        dz = twist.linear_z * duration_s
        drx = twist.angular_x * duration_s
        dry = twist.angular_y * duration_s
        drz = twist.angular_z * duration_s
        if frame in ("base", start_pose.frame):
            world_delta = (dx, dy, dz)
        elif frame in ("tool", "target", "local"):
            world_delta = rotate_rpy(
                (dx, dy, dz),
                start_pose.rx,
                start_pose.ry,
                start_pose.rz,
            )
        else:
            raise ValueError(f"Unsupported servo frame: {frame}")
        rx, ry, rz = _compose_rpy_offset(
            start_pose.rx,
            start_pose.ry,
            start_pose.rz,
            drx,
            dry,
            drz,
            local=frame in ("tool", "target", "local"),
        )
        return start_pose.translated(
            dx=world_delta[0],
            dy=world_delta[1],
            dz=world_delta[2],
        ).rotated(
            drx=rx - start_pose.rx,
            dry=ry - start_pose.ry,
            drz=rz - start_pose.rz,
        )


def cabinet_pose_to_ros_pose(pose: Pose) -> Any:
    from geometry_msgs.msg import Pose as RosPose

    msg = RosPose()
    msg.position.x = float(pose.x)
    msg.position.y = float(pose.y)
    msg.position.z = float(pose.z)
    qx, qy, qz, qw = _rpy_to_quaternion(pose.rx, pose.ry, pose.rz)
    msg.orientation.x = qx
    msg.orientation.y = qy
    msg.orientation.z = qz
    msg.orientation.w = qw
    return msg


def quaternion_pose_to_ros_pose(
    position: dict[str, float],
    orientation: dict[str, float],
) -> Any:
    from geometry_msgs.msg import Pose as RosPose

    qx, qy, qz, qw = normalize_quaternion(
        orientation["x"],
        orientation["y"],
        orientation["z"],
        orientation["w"],
    )
    msg = RosPose()
    msg.position.x = float(position["x"])
    msg.position.y = float(position["y"])
    msg.position.z = float(position["z"])
    msg.orientation.x = qx
    msg.orientation.y = qy
    msg.orientation.z = qz
    msg.orientation.w = qw
    return msg


def ros_pose_to_cabinet_pose(msg: Any, frame: str = "base_link") -> Pose:
    rx, ry, rz = _quaternion_to_rpy(
        msg.orientation.x,
        msg.orientation.y,
        msg.orientation.z,
        msg.orientation.w,
    )
    return Pose(
        x=float(msg.position.x),
        y=float(msg.position.y),
        z=float(msg.position.z),
        rx=rx,
        ry=ry,
        rz=rz,
        frame=frame,
    )


def normalize_quaternion(
    x_value: float,
    y_value: float,
    z_value: float,
    w_value: float,
) -> tuple[float, float, float, float]:
    x = float(x_value)
    y = float(y_value)
    z = float(z_value)
    w = float(w_value)
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if norm <= 1e-12:
        raise ValueError("Quaternion must not be zero")
    return x / norm, y / norm, z / norm, w / norm


def _rpy_to_quaternion(roll: float, pitch: float, yaw: float) -> tuple[float, float, float, float]:
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    qw = cr * cp * cy + sr * sp * sy
    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy
    return qx, qy, qz, qw


def _compose_rpy_offset(
    roll: float,
    pitch: float,
    yaw: float,
    delta_roll: float,
    delta_pitch: float,
    delta_yaw: float,
    local: bool,
) -> tuple[float, float, float]:
    if (
        abs(delta_roll) <= 1e-12
        and abs(delta_pitch) <= 1e-12
        and abs(delta_yaw) <= 1e-12
    ):
        return roll, pitch, yaw
    base_q = _rpy_to_quaternion(roll, pitch, yaw)
    delta_q = _rpy_to_quaternion(delta_roll, delta_pitch, delta_yaw)
    if local:
        qx, qy, qz, qw = _quaternion_multiply(base_q, delta_q)
    else:
        qx, qy, qz, qw = _quaternion_multiply(delta_q, base_q)
    return _quaternion_to_rpy(*normalize_quaternion(qx, qy, qz, qw))


def _quaternion_multiply(
    left: tuple[float, float, float, float],
    right: tuple[float, float, float, float],
) -> tuple[float, float, float, float]:
    lx, ly, lz, lw = left
    rx, ry, rz, rw = right
    return (
        lw * rx + lx * rw + ly * rz - lz * ry,
        lw * ry - lx * rz + ly * rw + lz * rx,
        lw * rz + lx * ry - ly * rx + lz * rw,
        lw * rw - lx * rx - ly * ry - lz * rz,
    )


def _quaternion_to_rpy(x: float, y: float, z: float, w: float) -> tuple[float, float, float]:
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (w * y - z * x)
    if abs(sinp) >= 1.0:
        pitch = math.copysign(math.pi / 2.0, sinp)
    else:
        pitch = math.asin(sinp)

    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return roll, pitch, yaw


def _clamp_int(value: int, lower: int, upper: int) -> int:
    return min(upper, max(lower, int(value)))


def _parameter_value(node: Any, name: str, default: Any) -> Any:
    try:
        return node.get_parameter(name).value
    except Exception:
        return default


def _bool_parameter_value(node: Any, name: str, default: bool) -> bool:
    value = _parameter_value(node, name, default)
    if isinstance(value, str):
        return value.strip().lower() in ("1", "true", "yes", "on")
    return bool(value)
