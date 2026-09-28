"""ROS feedback wrapper for the optional alignment-only kinematic check."""
from __future__ import annotations

import math
import threading
import time

from rg_cabinet_ros2.movel_alignment_check import (
    RealmanOfflineKinematics, check_alignment_path, orientation_error_deg,
)

_SDK_LOCK = threading.Lock()

# Runtime position/orientation arrival tolerances.
# The orientation tolerance must absorb the RPY round-trip jitter produced by
# the pitch=-90 deg gimbal lock of actuation_rpy_base (measured drift ~0.23 deg
# across the alignment route), which a 0.2 deg limit would reject as a failure.
_POSE_TOL_M = 0.002
_ORIENT_TOL_DEG = 0.5


class AlignmentPreflight:
    def __init__(self, node):
        from sensor_msgs.msg import JointState
        from geometry_msgs.msg import Pose
        self.node = node
        self._lock = threading.Lock()
        self._q = None
        self._stable_q = None
        self._pose = None
        self._q_time = self._pose_time = self._stable_since = 0.0
        self._joint_stamp = None
        self._model = None
        self._library = None
        group = node.rm_driver_callback_group
        self.subscriptions = [
            node.create_subscription(JointState, "/joint_states", self._joints_cb, 1,
                                     callback_group=group),
            node.create_subscription(Pose, node.get_parameter("tcp_pose_topic").value,
                                     self._pose_cb, 1, callback_group=group),
        ]

    def _joints_cb(self, msg):
        now = time.monotonic()
        names = [f"joint{i}" for i in range(1, 7)]
        if len(msg.name) != len(msg.position) or len(set(msg.name)) != len(msg.name):
            return
        positions = dict(zip(msg.name, msg.position))
        if not all(name in positions for name in names):
            return  # A separate gripper-only JointState must not erase arm feedback.
        q = [math.degrees(positions[name]) for name in names]
        if not all(math.isfinite(v) for v in q):
            return
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        with self._lock:
            if self._stable_q is None or max(abs(a-b) for a, b in zip(q, self._stable_q)) > 0.05:
                self._stable_since = now
                self._stable_q = q
            self._q, self._q_time, self._joint_stamp = q, now, stamp

    def _pose_cb(self, msg):
        from rg_cabinet_ros2.backends import ros_pose_to_cabinet_pose
        pose = ros_pose_to_cabinet_pose(msg, frame=self.node.get_parameter("base_frame").value)
        if not all(math.isfinite(v) for v in pose.as_list()):
            return
        with self._lock:
            self._pose, self._pose_time = pose, time.monotonic()

    def _snapshot(self, timeout=2.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            now = time.monotonic()
            ros_now = self.node.get_clock().now().nanoseconds * 1e-9
            with self._lock:
                ready = (self._q is not None and self._pose is not None
                         and now - self._q_time <= 0.3 and now - self._pose_time <= 0.3
                         and abs(self._q_time - self._pose_time) <= 0.1
                         and now - self._stable_since >= 0.1
                         and self._joint_stamp is not None
                         and -0.05 <= ros_now - self._joint_stamp <= 0.5)
                if ready:
                    return self._pose, list(self._q)
            time.sleep(0.01)
        raise RuntimeError("No fresh stationary arm joint/pose feedback for alignment preflight")

    def check(self, moves, settings, *, reroute=False, self_collision=False):
        report = {"status": "REJECTED", "scope": "alignment_movel_kinematics_only",
                  "collision_checked": False, "dynamics_checked": False}
        try:
            if self.node.get_parameter("base_frame").value != "base_link":
                raise RuntimeError("Offline alignment model requires base_frame=base_link")
            if int(self.node.get_parameter("rm_driver_dof").value) != 6:
                raise RuntimeError("Alignment preflight currently supports RM63 III six-axis only")
            library = settings.alignment_preflight_sdk_library
            with _SDK_LOCK:
                if self._model is None or self._library != library:
                    self._model = RealmanOfflineKinematics(library)
                    self._library = library
                start, q = self._snapshot()
                options = dict(step_m=settings.alignment_preflight_step_m,
                               min_margin_deg=settings.alignment_preflight_min_margin_deg,
                               max_joint_step_deg=settings.alignment_preflight_max_joint_step_deg)
                if reroute:
                    from rg_cabinet_ros2.movel_alignment_routes import search_alignment_route
                    drop = float(settings.alignment_reroute_drop_clearance_m)
                    if not math.isfinite(drop) or not .005 <= drop <= .20:
                        raise ValueError("Set the verified downward clearance (0.005 <= value <= 0.20 m)")
                    end_z = start.z + sum(d for axis, d in moves if axis == "z")
                    bounds = (min(start.z-drop, end_z-.002), max(start.z, end_z)+.002)
                    # Reserve 2 mm below the detour for feedback/control tolerance.
                    offsets = [-min(.025*i, drop-.002) for i in range(1, math.ceil(drop/.025)+1)]
                    report = search_alignment_route(self._model, start, q, moves,
                                                    z_offsets=offsets, z_bounds=bounds, **options)
                    report["start_pose"] = start.as_list()
                    report["start_joints_deg"] = q
                    report["base_frame"] = start.frame
                else:
                    report = check_alignment_path(
                        self._model, start, q, moves,
                        state_validator=self._model.self_collision_free if self_collision else None,
                        **options)
                if report["status"] == "PASSED":
                    after, after_q = self._snapshot()
                    if (max(abs(a-b) for a, b in zip(q, after_q)) > 0.2
                            or start.distance_to(after) > 0.001
                            or orientation_error_deg(start, after) > _ORIENT_TOL_DEG):
                        report.update(status="REJECTED", reason="STATE_CHANGED_DURING_CHECK")
        except Exception as exc:
            report.update(status="REJECTED", reason="PREFLIGHT_UNAVAILABLE", error=str(exc))
        self.node.last_alignment_preflight_debug = report
        self.node._publish_status({"event": "alignment_preflight", **report})
        if report["status"] != "PASSED":
            raise RuntimeError("MoveL alignment preflight rejected: " + report["reason"])
        return report

    def _wait_at(self, expected_pose, expected_joints, timeout=2.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            pose, q = self._snapshot(timeout=max(.05, deadline-time.monotonic()))
            if (pose.distance_to(expected_pose) <= .002
                    and orientation_error_deg(pose, expected_pose) <= _ORIENT_TOL_DEG
                    and max(abs(a-b) for a, b in zip(q, expected_joints)) <= 1.0):
                return pose, q
            time.sleep(.02)
        raise RuntimeError("Alignment route feedback differs from planned endpoint/branch")

    def execute_route(self, robot, moves, settings):
        from rg_cabinet.geometry import Pose
        from rg_cabinet.skills.base import servo_distance
        from rg_cabinet_ros2.movel_alignment_routes import route_in_z_bounds
        plan = self.check(moves, settings, reroute=True)
        route = plan["selected"]["moves"]
        expected_pose = Pose.from_sequence(plan["start_pose"], plan["base_frame"])
        expected_q = plan["start_joints_deg"]
        final_pose = expected_pose
        for axis, distance in moves:
            final_pose = final_pose.translated(**{"d"+axis: distance})
        plan["execution_segments"] = []
        plan["execution_status"] = "STARTING"
        checkpoints = []
        checkpoint = expected_pose
        for axis, distance in route:
            checkpoint = checkpoint.translated(**{"d"+axis: distance})
            checkpoints.append(checkpoint)
        try:
            for index, (axis, distance) in enumerate(route):
                actual, _ = self._wait_at(expected_pose, expected_q)
                # Correct the commanded axis to its original absolute checkpoint,
                # avoiding accumulation of endpoint errors across the detour.
                adjusted = []
                simulated = actual
                for (remaining_axis, _), target in zip(route[index:], checkpoints[index:]):
                    d = getattr(target, remaining_axis)-getattr(simulated, remaining_axis)
                    adjusted.append((remaining_axis, d))
                    simulated = simulated.translated(**{"d"+remaining_axis: d})
                # Recheck the entire remainder from current feedback, not only
                # the endpoint or an arbitrary alternate IK solution.
                remainder = self.check(adjusted, settings, self_collision=True)
                actual = Pose.from_sequence(remainder["start_pose"], plan["base_frame"])
                if not route_in_z_bounds(actual.z, adjusted, plan["surveyed_z_bounds_m"]):
                    raise RuntimeError("Feedback drift puts remaining route outside verified clearance")
                if (robot.get_tcp_pose().distance_to(actual) > .001
                        or orientation_error_deg(robot.get_tcp_pose(), actual) > _ORIENT_TOL_DEG):
                    raise RuntimeError("Driver pose and preflight pose do not agree")
                distance = adjusted[0][1]
                expected_pose = actual.translated(**{"d"+axis: distance})
                expected_q = remainder["segments"][0]["end_joints_deg"]
                servo_distance(robot, axis=axis, distance_m=distance,
                               speed_mps=settings.align_speed_mps, frame="base")
                reached, _ = self._wait_at(expected_pose, expected_q)
                plan["execution_segments"].append({"axis": axis, "distance_m": distance,
                                                   "reached_pose": reached.as_list()})
                self.node.last_alignment_preflight_debug = plan
                self.node._publish_status({"event": "alignment_route_segment_completed", **plan})
            reached, _ = self._snapshot()
            if reached.distance_to(final_pose) > _POSE_TOL_M or orientation_error_deg(reached, final_pose) > _ORIENT_TOL_DEG:
                raise RuntimeError("Rerouted alignment did not reach the original alignment goal")
            plan["execution_status"] = "COMPLETED"
        except Exception as exc:
            robot.stop()
            plan.update(execution_status="FAILED", execution_error=str(exc))
            raise
        finally:
            self.node.last_alignment_preflight_debug = plan
            self.node._publish_status({"event": "alignment_route_execution", **plan})
        return plan
