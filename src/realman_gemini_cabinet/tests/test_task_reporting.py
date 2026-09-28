import json

from rg_cabinet_ros2.task_reporting import save_diagnostic, status_summary


def test_motion_failure_is_short_and_does_not_guess_root_cause():
    payload = {
        "event": "qr_sequence_failed", "success": False, "step": "middle_black_knob",
        "stage": "orientation", "error": "rm_driver reported movel failure",
        "motion_command_debug": {"result": False, "result_name": "movel",
                                 "elapsed_s": .01988547, "distance_m": .000023946},
        "vision_targets": [{"matrix": list(range(10000))}],
    }
    text = status_summary(payload)
    assert "调整操作姿态" in text and "0.020 s" in text and "0.024 mm" in text
    assert len(text) < 250
    assert "欠压" not in text and "零位移" not in text and "matrix" not in text


def test_successful_previous_motion_is_not_blamed_for_vision_failure():
    text = status_summary({"success": False, "error": "Missing or ambiguous central hub cap",
                           "stage": "vision", "motion_command_debug": {
                               "result": True, "result_name": "movej_p", "elapsed_s": 1.5}})
    assert "轴心" in text and "movej_p" not in text and "耗时" not in text


def test_full_diagnostic_is_saved_including_large_arrays(tmp_path):
    payload = {"success": False, "error": "原始错误", "vision_targets": list(range(2000))}
    save_diagnostic(payload, tmp_path)
    saved = json.loads(open(payload["diagnostic_file"], encoding="utf8").read())
    assert saved == payload
    second = dict(payload)
    save_diagnostic(second, tmp_path)
    assert second["diagnostic_file"] != payload["diagnostic_file"]


def test_disk_failure_preserves_original_error(tmp_path):
    path = tmp_path / "not_a_directory"
    path.write_text("occupied")
    payload = {"success": False, "error": "original motion failure"}
    save_diagnostic(payload, path)
    assert payload["error"] == "original motion failure"
    assert "diagnostic_write_error" in payload and "diagnostic_file" not in payload
    assert "保存失败" in status_summary(payload)


def test_timeout_and_result_failure_are_distinguished():
    text = status_summary({"success": False, "error": "movel timeout", "motion_command_debug": {
        "result": "timeout", "result_name": "movel", "elapsed_s": 20}})
    assert "等待超时" in text and "20.000 s" in text
    text = status_summary({"event": "qr_sequence_complete", "success": False,
                           "results": [{"success": False}]})
    assert "失败" in text


def test_prepare_and_completion_are_readable():
    assert "调整操作姿态" in status_summary({"event": "task_stage", "stage": "orientation"})
    assert "2/2" in status_summary({"event": "qr_sequence_complete", "success": True,
                                   "results": [{"success": True}, {"success": True}]})


def test_reroute_summary_contains_moves_not_path_samples():
    text = status_summary({"event": "alignment_preflight", "status": "PASSED", "selected": {
        "moves": [["z", -.098], ["y", .135]], "minimum_margin_deg": 23.39}})
    assert "Z -98.0 mm → Y 135.0 mm" in text and "23.4°" in text
    assert "未通过" in status_summary({"event": "alignment_preflight", "status": "REJECTED",
                                      "reason": "NO_VALID_ROUTE_IN_SEARCH_SET"})


def test_real_node_can_switch_log_levels_without_affecting_payload(tmp_path):
    import pytest
    rclpy = pytest.importorskip("rclpy")
    from rclpy.parameter import Parameter
    from rg_cabinet_ros2.task_node import CabinetTaskNode
    rclpy.init()
    node = CabinetTaskNode(parameter_overrides=[Parameter("backend", value="mock"),
                                               Parameter("task_log_dir", value=str(tmp_path))])
    try:
        node._set_active_sequence_step({"name": "knob", "skill": "knob_camera_servo"})
        node._report_task_stage("orientation")
        for value in (True, False, True):
            node._publish_status({"event": "motion_command_completed",
                                  "motion_command_debug": {"result": value, "result_name": "movel"}})
        payload = {"event": "qr_sequence_failed", "success": False, "error": "original error"}
        node._attach_execution_debug(payload)
        node._publish_status(payload)
        assert payload["stage"] == "orientation"
        assert payload["error"] == "original error"
        assert json.loads(open(payload["diagnostic_file"]).read())["stage"] == "orientation"
    finally:
        node.destroy_node()
        rclpy.shutdown()
