"""Read-only, sampled kinematic preflight for the existing base-axis MoveL alignment.

No controller connection, motion commands, alternate seeds, or collision claims.
The SDK model uses RM63 III factory parameters; mismatched feedback fails closed.
"""
from __future__ import annotations

import ctypes as C
import math
from pathlib import Path
from typing import Sequence

from rg_cabinet.geometry import Pose


class _XYZ(C.Structure):
    _fields_ = [(name, C.c_float) for name in ("x", "y", "z")]


class _Quat(C.Structure):
    _fields_ = [(name, C.c_float) for name in ("w", "x", "y", "z")]


class _SDKPose(C.Structure):
    _fields_ = [("position", _XYZ), ("quaternion", _Quat), ("euler", _XYZ)]


class _IK(C.Structure):
    _fields_ = [("q_in", C.c_float * 7), ("q_pose", _SDKPose), ("flag", C.c_uint8)]


def sdk_library_path(explicit: str = "") -> Path:
    if explicit:
        path = Path(explicit).expanduser().resolve()
    else:
        from ament_index_python.packages import get_package_prefix
        prefix = Path(get_package_prefix("rm_driver")).resolve()
        path = prefix.parent.parent / "src/ros2_rm_robot/rm_driver/lib/libapi_cpp.so"
    if not path.is_file():
        raise RuntimeError(f"Alignment preflight SDK not found: {path}; set alignment_preflight_sdk_library")
    return path


class RealmanOfflineKinematics:
    """C ABI verified against the SDK shipped with this workspace (ARM_DOF=7).

    Instances share SDK global model data. The runtime wrapper serializes access.
    All solver handles are NULL; this class cannot command or connect to hardware.
    """
    model_source = "RM63_III_standard_factory_defaults_not_controller_readback"

    def __init__(self, library: str):
        self.library = str(sdk_library_path(library))
        self.api = C.CDLL(self.library)
        pointer = C.POINTER(C.c_float)
        signatures = {
            "rm_algo_init_sys_data": ([C.c_int, C.c_int], None),
            "rm_algo_get_joint_min_limit": ([pointer], None),
            "rm_algo_get_joint_max_limit": ([pointer], None),
            "rm_algo_forward_kinematics": ([C.c_void_p, pointer], _SDKPose),
            "rm_algo_inverse_kinematics": ([C.c_void_p, _IK, pointer], C.c_int),
        }
        for name, (args, result) in signatures.items():
            function = getattr(self.api, name)
            function.argtypes, function.restype = args, result
        self.api.rm_algo_init_sys_data(4, 0)  # RM_MODEL_RM_63_III_E, RM_MODEL_RM_B_E
        low, high = (C.c_float * 7)(), (C.c_float * 7)()
        self.api.rm_algo_get_joint_min_limit(low)
        self.api.rm_algo_get_joint_max_limit(high)
        self.limits = [(float(low[i]), float(high[i])) for i in range(6)]
        if any(not math.isfinite(a + b) or a >= b for a, b in self.limits):
            raise RuntimeError("SDK returned invalid joint limits")

    def forward(self, joints_deg: Sequence[float], frame: str) -> Pose:
        values = (C.c_float * 7)(*joints_deg, 0.0)
        result = self.api.rm_algo_forward_kinematics(None, values)
        return Pose(result.position.x, result.position.y, result.position.z,
                    result.euler.x, result.euler.y, result.euler.z, frame)

    def inverse(self, pose: Pose, seed_deg: Sequence[float]):
        request = _IK()
        request.q_in = (C.c_float * 7)(*seed_deg, 0.0)
        request.q_pose.position = _XYZ(pose.x, pose.y, pose.z)
        request.q_pose.euler = _XYZ(pose.rx, pose.ry, pose.rz)
        request.flag = 1
        result = (C.c_float * 7)()
        code = self.api.rm_algo_inverse_kinematics(None, request, result)
        return code, [float(result[i]) for i in range(6)]

    def self_collision_free(self, joints_deg: Sequence[float]) -> bool:
        check = self.api.rm_algo_safety_robot_self_collision_detection
        check.argtypes = [C.POINTER(C.c_float)]
        check.restype = C.c_int
        return check((C.c_float * 7)(*joints_deg, 0.0)) == 0


def orientation_error_deg(a: Pose, b: Pose) -> float:
    # Quaternion comparison avoids Euler wrap and pitch=+-pi/2 artifacts.
    def quat(p):
        cr, cp, cy = (math.cos(v / 2) for v in (p.rx, p.ry, p.rz))
        sr, sp, sy = (math.sin(v / 2) for v in (p.rx, p.ry, p.rz))
        return (cr*cp*cy + sr*sp*sy, sr*cp*cy - cr*sp*sy,
                cr*sp*cy + sr*cp*sy, cr*cp*sy - sr*sp*cy)
    dot = abs(sum(x*y for x, y in zip(quat(a), quat(b))))
    return math.degrees(2 * math.acos(min(1.0, dot)))


def check_alignment_path(kinematics, start: Pose, joints_deg: Sequence[float],
                         moves: Sequence[tuple[str, float]], *, step_m=0.001,
                         min_margin_deg=5.0, max_joint_step_deg=5.0,
                         state_validator=None) -> dict:
    """Check ALL alignment segments from the measured branch before the first MoveL.

    Bounds are not wrapped modulo 360; an SDK branch switch is a rejection.
    This samples geometry only, not the controller's proprietary interpolation or
    dynamic feasibility. Factory-model limits are explicitly reported.
    """
    report = {"status": "REJECTED", "scope": "alignment_movel_kinematics_only",
              "collision_checked": False, "dynamics_checked": False,
              "self_collision_checked": state_validator is not None,
              "model_source": kinematics.model_source,
              "sample_step_m": step_m, "required_margin_deg": min_margin_deg,
              "max_joint_step_deg": max_joint_step_deg, "checked_samples": 0,
              "start_pose": start.as_list(), "start_joints_deg": list(joints_deg),
              "limits_deg": [list(pair) for pair in kinematics.limits], "segments": []}

    def reject(reason, **details):
        report.update(reason=reason, **details)
        return report

    values = list(joints_deg) + start.as_list() + [step_m, min_margin_deg, max_joint_step_deg]
    if len(joints_deg) != 6 or not all(math.isfinite(v) for v in values):
        return reject("INVALID_INPUT")
    if not (0 < step_m <= 0.005 and 0 <= min_margin_deg < 90 and 0 < max_joint_step_deg <= 30):
        return reject("INVALID_CHECK_SETTINGS")
    if len(kinematics.limits) != 6:
        return reject("INVALID_MODEL_LIMITS")
    for axis, distance in moves:
        if axis not in ("x", "y", "z") or not math.isfinite(distance):
            return reject("INVALID_ALIGNMENT_SEGMENT")
    if sum(math.ceil(abs(d)/step_m) for _, d in moves) > 5000:
        return reject("TOO_MANY_SAMPLES")

    def margins(q):
        return [min(v - lo, hi - v) for v, (lo, hi) in zip(q, kinematics.limits)]

    margin = margins(joints_deg)
    report["minimum_margin_deg"] = min(margin)
    report["limiting_joint"] = margin.index(min(margin)) + 1
    if min(margin) < 0:
        return reject("START_OUTSIDE_MODEL_LIMITS")
    if min(margin) < min_margin_deg:
        return reject("START_MARGIN_TOO_SMALL")
    if state_validator is not None and not state_validator(joints_deg):
        return reject("START_SELF_COLLISION")
    fk = kinematics.forward(joints_deg, start.frame)
    if not all(math.isfinite(v) for v in fk.as_list()):
        return reject("INVALID_FORWARD_KINEMATICS")
    report["start_fk_position_error_m"] = start.distance_to(fk)
    report["start_fk_orientation_error_deg"] = orientation_error_deg(start, fk)
    if start.distance_to(fk) > 0.003 or orientation_error_deg(start, fk) > 0.5:
        return reject("MODEL_OR_TCP_FRAME_MISMATCH")

    seed = list(joints_deg)
    origin = start
    for axis, distance in moves:
        segment = {"axis": axis, "distance_m": distance, "checked_samples": 0}
        report["segments"].append(segment)
        count = max(1, math.ceil(abs(distance) / step_m))
        for i in range(1, count + 1):
            offset = distance * i / count
            pose = origin.translated(**{"d" + axis: offset})
            code, q = kinematics.inverse(pose, seed)
            location = {"failed_axis": axis, "failed_distance_m": offset,
                        "failed_pose": pose.as_list(), "last_joints_deg": seed}
            if code:
                return reject("IK_FAILED", sdk_code=int(code), **location)
            if len(q) != 6 or not all(math.isfinite(v) for v in q):
                return reject("INVALID_IK_RESULT", **location)
            jump = [abs(a-b) for a, b in zip(q, seed)]
            if max(jump) > max_joint_step_deg:
                return reject("JOINT_BRANCH_JUMP", jump_joint=jump.index(max(jump))+1,
                              jump_deg=max(jump), proposed_joints_deg=q, **location)
            margin = margins(q)
            if min(margin) < report["minimum_margin_deg"]:
                report["minimum_margin_deg"] = min(margin)
                report["limiting_joint"] = margin.index(min(margin)) + 1
            if min(margin) < min_margin_deg:
                return reject("JOINT_LIMIT" if min(margin) < 0 else "MARGIN_TOO_SMALL",
                              proposed_joints_deg=q, **location)
            if state_validator is not None and not state_validator(q):
                return reject("SELF_COLLISION", proposed_joints_deg=q, **location)
            solved = kinematics.forward(q, start.frame)
            if not all(math.isfinite(v) for v in solved.as_list()) or pose.distance_to(solved) > 0.003 or orientation_error_deg(pose, solved) > 0.5:
                return reject("IK_POSE_MISMATCH", **location)
            seed = q
            report["checked_samples"] += 1
            segment["checked_samples"] += 1
        origin = origin.translated(**{"d" + axis: distance})
        segment["end_joints_deg"] = seed
    report.update(status="PASSED", reason="SAMPLED_ALIGNMENT_PATH_ACCEPTED",
                  end_joints_deg=seed, end_pose=origin.as_list())
    return report
