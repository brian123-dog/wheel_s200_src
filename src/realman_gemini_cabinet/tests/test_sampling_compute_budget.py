"""Slow accepted detections still require fresh, consistent frames within budget."""
from dataclasses import replace
from pathlib import Path
from unittest.mock import Mock, patch
import pytest
import yaml
from test_fresh_vision import Stream, target
from rg_cabinet_ros2.fresh_vision import SamplingPolicy, VisionSamplingError, sample_target
from rg_cabinet_ros2.vision_target_resolver import VisionTargetResolver


def slow_sampling(budget, duration=4.5, second_x=81):
    clock=[0.]
    count=[0]
    def detect(frame):
        count[0]+=1
        clock[0]+=duration
        return target(80 if count[0]==1 else second_x)
    with patch('rg_cabinet_ros2.fresh_vision.time.monotonic',side_effect=lambda:clock[0]):
        return sample_target(Stream(),replace(SamplingPolicy(),total_timeout_s=budget),
                             lambda:100.,detect,Mock())


def test_two_slow_valid_frames_fail_at_six_seconds_with_clear_timing():
    with pytest.raises(VisionSamplingError) as caught:
        slow_sampling(6)
    report=caught.value.report
    assert report['elapsed_s']==9 and report['total_timeout_s']==6
    assert report['confirmations']==0
    assert [v['detection_s'] for v in report['history']]==[4.5,4.5]
    assert 'detection accepted' in str(caught.value)
    assert 'budget=6.00s' in str(caught.value)


def test_two_slow_valid_frames_succeed_with_right_knob_budget():
    result,report=slow_sampling(12)
    assert report['confirmations']==2 and report['attempts']==2
    assert report['elapsed_s']==9 and result.details['axis_center_pixel']==[81,80]


def test_twelve_seconds_is_still_a_deadline_for_accepted_detection():
    with pytest.raises(VisionSamplingError):
        slow_sampling(12,duration=7)


def test_larger_budget_does_not_accept_conflicting_second_frame():
    with pytest.raises(VisionSamplingError):
        slow_sampling(12,second_x=100)


def test_only_right_knob_budget_changes_and_freshness_checks_are_preserved():
    root=Path(__file__).resolve().parents[1]
    cfg=yaml.safe_load((root/'config/ros2_qr_panel_layout_two_knobs.yaml').read_text())
    resolver=VisionTargetResolver.__new__(VisionTargetResolver)
    policies={name:resolver.fresh_sampling_settings(cfg,{'target':{'source':'camera','detection':name}})
              for name in ['middle_black_knob','upper_row_right_black_knob']}
    middle=SamplingPolicy.from_config(policies['middle_black_knob'])
    right=SamplingPolicy.from_config(policies['upper_row_right_black_knob'])
    assert middle.total_timeout_s==right.total_timeout_s==12
    assert right.confirmation_frames==1 and middle.confirmation_frames==2
    assert replace(right,confirmation_frames=2)==middle
