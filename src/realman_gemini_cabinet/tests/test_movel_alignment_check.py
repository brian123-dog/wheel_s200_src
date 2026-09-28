import math
from dataclasses import replace

import pytest

from rg_cabinet.geometry import Pose
from rg_cabinet_ros2.movel_alignment_check import check_alignment_path, orientation_error_deg
from rg_cabinet.skills.knob_camera_servo import KnobCameraServoSkill, KnobCameraServoConfig


class Model:
    model_source = "test_model"
    limits = [(-180.0, 180.0)] * 6

    def __init__(self):
        self.seeds = []
        self.failure = None

    def forward(self, q, frame):
        return Pose(q[0]/100, q[1]/100, q[2]/100, frame=frame)

    def inverse(self, pose, seed):
        self.seeds.append(list(seed))
        q = [pose.x*100, pose.y*100, pose.z*100, 0.0, 0.0, 0.0]
        if self.failure:
            return self.failure(pose, q)
        return 0, q


def check(model=None, **kwargs):
    return check_alignment_path(model or Model(), Pose(0, 0, 0), [0.0]*6,
                                [("y", .010), ("z", .005)], **kwargs)


def test_both_segments_use_previous_solution_and_preserve_order():
    model = Model()
    result = check(model)
    assert result["status"] == "PASSED"
    assert result["checked_samples"] == 15
    assert result["end_pose"][:3] == [0, .01, .005]
    assert model.seeds[10][1] == 1.0
    assert result["end_joints_deg"][:3] == [0, 1, .5]
    assert not result["collision_checked"]
    assert not result["dynamics_checked"]


def test_later_axis_failure_is_reported_by_full_precheck():
    model = Model()
    model.failure = lambda p, q: (1 if p.z > .003 else 0, q)
    result = check(model)
    assert result["reason"] == "IK_FAILED"
    assert result["failed_axis"] == "z"


def test_branch_jump_not_treated_as_a_valid_solution():
    model = Model()
    model.failure = lambda p, q: (0, q[:3] + [178.0, 0.0, 0.0])
    result = check(model)
    assert result["reason"] == "JOINT_BRANCH_JUMP"
    assert result["jump_joint"] == 4


def test_margin_is_checked_at_intermediate_samples():
    model = Model()
    model.limits = [(-180, 180), (-10, 5.5)] + [(-180, 180)]*4
    result = check(model)
    assert result["reason"] == "MARGIN_TOO_SMALL"
    assert result["failed_axis"] == "y"
    assert result["limiting_joint"] == 2


def test_actual_start_is_not_clamped_or_replaced():
    model = Model()
    result = check_alignment_path(model, Pose(0, 0, 0), [0,0,0,-181,0,0], [("y", .01)])
    assert result["reason"] == "START_OUTSIDE_MODEL_LIMITS"
    assert not model.seeds


def test_start_fk_mismatch_refuses_different_tool_or_model():
    result = check_alignment_path(Model(), Pose(.1, 0, 0), [0]*6, [("y", .01)])
    assert result["reason"] == "MODEL_OR_TCP_FRAME_MISMATCH"


def test_sdk_valid_flag_with_wrong_pose_is_rejected():
    model = Model()
    model.failure = lambda p, q: (0, [0.0]*6)
    assert check(model)["reason"] == "IK_POSE_MISMATCH"


@pytest.mark.parametrize("settings", [{"step_m": 0}, {"step_m": .1},
                                    {"min_margin_deg": -1}, {"max_joint_step_deg": 180}])
def test_invalid_settings_fail_closed(settings):
    assert check(**settings)["reason"] == "INVALID_CHECK_SETTINGS"


def test_invalid_solver_numbers_fail_closed():
    model = Model()
    model.failure = lambda p, q: (0, [float("nan")]*6)
    assert check(model)["reason"] == "INVALID_IK_RESULT"


def test_euler_wrap_is_not_a_false_model_mismatch():
    assert orientation_error_deg(Pose(0,0,0,0,0,math.pi), Pose(0,0,0,0,0,-math.pi)) < 1e-5


class Robot:
    def __init__(self, fail=False):
        self.commands = []
        self.checks = []
        self.fail = fail

    def preflight_movel_alignment(self, moves, settings):
        self.checks.append(moves)
        if self.fail:
            raise RuntimeError("alignment rejected")

    def servo_twist(self, twist, duration_s, frame):
        self.commands.append((twist.as_list(), duration_s, frame))

    def stop(self):
        pass


def test_rejected_alignment_does_not_publish_any_axis_motion():
    robot = Robot(fail=True)
    skill = KnobCameraServoSkill(KnobCameraServoConfig(alignment_preflight=True))
    with pytest.raises(RuntimeError, match="alignment rejected"):
        skill._servo_alignment(robot, {"x":0, "y":.015, "z":.02})
    assert not robot.commands
    assert robot.checks == [[("y", .015), ("z", .02)]]


def test_enabled_check_keeps_identical_movel_commands_and_rechecks_each_axis():
    cfg = KnobCameraServoConfig()
    old, checked = Robot(), Robot()
    delta = {"x":0, "y":.015, "z":.02}
    KnobCameraServoSkill(cfg)._servo_alignment(old, delta)
    KnobCameraServoSkill(replace(cfg, alignment_preflight=True))._servo_alignment(checked, delta)
    assert old.commands == checked.commands
    assert old.checks == []
    assert checked.checks == [[("y", .015), ("z", .02)], [("y", .015)], [("z", .02)]]


def test_tiny_skipped_moves_do_not_require_a_check():
    robot = Robot(fail=True)
    KnobCameraServoSkill(KnobCameraServoConfig(alignment_preflight=True))._servo_alignment(
        robot, {"x":0, "y":.0001, "z":0})
    assert not robot.checks
    assert not robot.commands
