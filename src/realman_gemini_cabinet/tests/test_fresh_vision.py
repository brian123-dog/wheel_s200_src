from dataclasses import replace
from types import SimpleNamespace
from unittest.mock import Mock, patch
import json
import threading
import time

import pytest

from rg_cabinet.geometry import Pose
from rg_cabinet.perception.element_detector import ElementTarget
from rg_cabinet_ros2.camera_cache import CameraFrameCache, FrameSnapshot, FreshFrameTimeout
from rg_cabinet_ros2.fresh_vision import (PanelLayoutRejected, SamplingPolicy,
    VisionSamplingError, sample_target)


def msg(stamp):
    sec = int(stamp)
    return SimpleNamespace(header=SimpleNamespace(stamp=SimpleNamespace(
        sec=sec, nanosec=int((stamp-sec)*1e9)), frame_id='camera'),
        width=2, height=2, step=6, encoding='rgb8', data=bytes([100]*12))


def frame(c=100.1, d=100.1):
    return FrameSnapshot(msg(c), msg(d), object())


def target(x=80, depth=.45):
    return ElementTarget(name='knob', kind='knob', pose=Pose(0, 0, 0, frame='base_link'),
                         confidence=1., details={'axis_center_pixel': [x, 80],
                         'depth_m': depth, 'camera_frame': 'camera', 'image_size': [1280, 720]})


class Stream:
    def __init__(self, events=None):
        self.events = list(events or []); self.index = 0; self.inputs = []
    def snapshot(self):
        return frame(99., 99.)
    def wait_snapshot(self, **kwargs):
        self.inputs.append(kwargs)
        self.index += 1
        if self.events:
            event = self.events.pop(0)
            if isinstance(event, Exception):
                raise event
        f = frame(100+self.index*.1, 100+self.index*.1)
        return f, {'color_stamp_s': f.stamps[0], 'depth_stamp_s': f.stamps[1], 'problems': []}


def run(stream, outcomes, policy=None):
    detects = Mock(side_effect=outcomes)
    failed = Mock()
    result = sample_target(stream, policy or SamplingPolicy(), lambda: 100., detects, failed)
    return result, detects, failed


def test_reject_then_two_consistent_fresh_frames_recovers():
    stream = Stream()
    (result, report), detects, failed = run(stream, [PanelLayoutRejected('weak'), target(), target(81)])
    assert report['attempts'] == 3 and report['confirmations'] == 2
    assert failed.call_count == 1
    assert stream.inputs[0]['after'] == (99., 99.)
    assert stream.inputs[1]['after'] == pytest.approx((100.1, 100.1))
    assert result.details['axis_center_pixel'] == [81, 80]


def test_conflicting_axis_must_get_a_second_consistent_confirmation():
    (result, report), _, failed = run(Stream(), [target(), target(100), target(101)])
    assert report['attempts'] == 3 and failed.call_count == 1
    assert 'disagree' in report['history'][1]['reason']


def test_depth_conflict_is_also_rejected():
    (_, report), _, failed = run(Stream(), [target(depth=.4), target(depth=.48), target(depth=.481)])
    assert report['attempts'] == 3 and failed.call_count == 1


def test_rejection_resets_consecutive_confirmation():
    (_, report), _, _ = run(Stream(), [target(), PanelLayoutRejected('bad'), target(), target()])
    assert report['attempts'] == 4


def test_five_bad_frames_stop_with_history():
    detect = Mock(side_effect=PanelLayoutRejected('weak match')); failed = Mock()
    with pytest.raises(VisionSamplingError) as caught:
        sample_target(Stream(), SamplingPolicy(), lambda: 100., detect, failed)
    assert detect.call_count == failed.call_count == 5
    assert len(caught.value.report['history']) == 5


def test_frame_timeout_can_recover_without_using_stale_image():
    stale = frame(99, 99)
    timeout = FreshFrameTimeout(stale, {'problems': ['depth: no new frame']})
    (_, report), detect, failed = run(Stream([timeout]), [target(), target()])
    assert report['attempts'] == 3 and detect.call_count == 2
    assert failed.call_args[0][0]['frame_timeout']


def test_fatal_workspace_or_tf_error_is_never_retried():
    detect = Mock(side_effect=RuntimeError('outside workspace'))
    with pytest.raises(RuntimeError, match='outside workspace'):
        sample_target(Stream(), SamplingPolicy(), lambda: 100., detect, Mock())
    assert detect.call_count == 1


@pytest.mark.parametrize('c,d,previous,start,now,reason', [
    (100., 100., (100.,100.), 99.,100.1,'no new frame'),
    (100.1, 99., (99.,99.), 100.,100.2,'depth:'),
    (99.5,99.5,(98.,98.),99.,101.,'stale frame'),
    (101.,100.5,(99.,99.),100.,101.1,'skew'),
    (110.,110.,(99.,99.),100.,100.1,'future'),
    (99.9,99.9,(99.,99.),100.,100.1,'after sampling started'),
])
def test_frame_quality_rejects_bad_pairs(c,d,previous,start,now,reason):
    diag=frame(c,d).diagnostics(now,previous,start,.75,.15)
    assert any(reason in p for p in diag['problems'])


def test_fresh_pair_accepted_and_snapshot_keeps_original_messages():
    cache=CameraFrameCache(color=msg(100.1), depth=msg(100.12), camera_info=object())
    frozen,diag=cache.wait_snapshot(after=(99.,99.),not_before_s=100.,timeout_s=.01,
        now_s=lambda:100.2,max_age_s=.75,max_skew_s=.15)
    cache.color=msg(101.)
    assert not diag['problems'] and frozen.stamps[0] == pytest.approx(100.1)


def test_real_wait_is_bounded_and_records_which_stream_did_not_update():
    cache=CameraFrameCache(color=msg(100.1),depth=msg(99),camera_info=object())
    start=time.monotonic()
    with pytest.raises(FreshFrameTimeout) as caught:
        cache.wait_snapshot(after=(99.,99.),not_before_s=100.,timeout_s=.03,
            now_s=lambda:100.2,max_age_s=.75,max_skew_s=.15)
    assert time.monotonic()-start < .3
    assert any(p.startswith('depth:') for p in caught.value.diagnostics['problems'])


def test_budget_stops_retry_when_camera_is_frozen():
    cache=CameraFrameCache(color=msg(99),depth=msg(99),camera_info=object())
    policy=replace(SamplingPolicy(),total_timeout_s=.05,frame_wait_timeout_s=.02)
    detect=Mock();start=time.monotonic()
    with pytest.raises(VisionSamplingError) as caught:
        sample_target(cache,policy,lambda:100.,detect,Mock())
    assert time.monotonic()-start < .3 and not detect.called
    assert caught.value.report['attempts'] <= 3


def test_failure_artifacts_saved_without_success_image_setting(tmp_path):
    from rg_cabinet_ros2.vision_failure_artifacts import save_failure_artifacts
    paths=save_failure_artifacts(frame(),{'error':'weak match'},
        {'save_debug_images':False,'vision_debug_dir':str(tmp_path)},'knob')
    assert 'error' not in paths
    assert json.loads(open(paths['metadata_path']).read())['error']=='weak match'
    assert __import__('pathlib').Path(paths['color_path']).exists()


def test_disk_failure_remains_diagnostic_not_detection_success(tmp_path):
    from rg_cabinet_ros2.vision_failure_artifacts import save_failure_artifacts
    blocked=tmp_path/'file';blocked.write_text('x')
    paths=save_failure_artifacts(frame(),{}, {'vision_debug_dir':str(blocked)},'knob')
    assert 'error' in paths and 'metadata_error' in paths


def test_legacy_resolver_bypasses_new_sampling():
    from rg_cabinet_ros2.vision_target_resolver import VisionTargetResolver
    resolver=VisionTargetResolver.__new__(VisionTargetResolver)
    resolver._resolve_once=Mock(return_value=target())
    with patch('rg_cabinet_ros2.vision_target_resolver.sample_target') as sampling:
        resolver.resolve({'vision':{'fresh_sampling':{'enabled':True}}},
            {'name':'knob','skill':'knob_camera_servo','target':{'source':'camera','locator':'gray_plate'}})
    assert not sampling.called and resolver._resolve_once.call_count == 1


def test_resolver_only_publishes_confirmed_target(tmp_path):
    from rg_cabinet_ros2.vision_target_resolver import VisionTargetResolver
    resolver=VisionTargetResolver.__new__(VisionTargetResolver)
    resolver.camera_cache=Stream();resolver.resolved_targets=[];resolver.last_target_debug=None
    resolver._panel_layout_detectors={};resolver._publish_target_pose=Mock()
    resolver.node=Mock();resolver.node.get_parameter.return_value.value=True
    resolver.node.get_clock.return_value.now.return_value.nanoseconds=100_000_000_000
    resolver._resolve_once=Mock(side_effect=[PanelLayoutRejected('weak'), target(),target()])
    result=resolver.resolve({'vision':{'fresh_sampling':{'enabled':True},'save_debug_images':False,
        'vision_debug_dir':str(tmp_path),'targets':{'knob':{'locator':'panel_layout'}}}},
        {'name':'knob','skill':'knob_camera_servo','target':{'source':'camera','detection':'knob'}})
    assert resolver._publish_target_pose.call_count == 1
    assert len(resolver.resolved_targets)==1
    assert result.details['fresh_sampling']['attempts']==3
    assert all(call.kwargs['publish'] is False for call in resolver._resolve_once.call_args_list)
    assert list(tmp_path.glob('*_failed_color.png'))


def test_confirmation_does_not_allow_gradual_drift():
    policy=replace(SamplingPolicy(),confirmation_frames=3,max_attempts=3)
    with pytest.raises(VisionSamplingError):
        run(Stream(),[target(80),target(84),target(88)],policy)


def test_new_sampling_owns_alignment_wait_without_changing_servo_delta():
    from rg_cabinet.skills.knob_camera_servo import KnobCameraServoSkill
    t=target();t.details.update(fresh_sampling={'enabled':True}, alignment_error_base_m=[0.,.02,.01])
    context=SimpleNamespace(robot=Mock(),camera_cache=Mock(),target_refresher=Mock(return_value=t))
    context.camera_cache.wait_fresh.return_value=False
    skill=KnobCameraServoSkill();skill._servo_alignment=Mock()
    result=skill._align_to_target(context,t,{'x':0.,'y':.02,'z':.01})
    assert context.target_refresher.call_count == 2
    context.camera_cache.wait_fresh.assert_not_called()
    skill._servo_alignment.assert_called_once_with(context.robot,{'x':0.,'y':.02,'z':.01})
    assert result[-1]=='median_alignment_done'


def test_legacy_alignment_keeps_existing_frame_wait():
    from rg_cabinet.skills.knob_camera_servo import KnobCameraServoSkill
    t=target();t.details['alignment_error_base_m']=[0.,.02,.01]
    context=SimpleNamespace(robot=Mock(),camera_cache=Mock(),target_refresher=Mock(return_value=t))
    context.camera_cache.wait_fresh.return_value=False
    skill=KnobCameraServoSkill();skill._servo_alignment=Mock()
    with pytest.raises(RuntimeError,match='camera frames not refreshed'):
        skill._align_to_target(context,t,{'x':0.,'y':.02,'z':.01})
    skill._servo_alignment.assert_not_called()


def test_observation_new_mode_defers_frame_wait_until_after_motion():
    from rg_cabinet_ros2.task_node import CabinetTaskNode
    node=SimpleNamespace(_report_task_stage=Mock(),camera_cache=Mock(),get_parameter=Mock())
    node.get_parameter.return_value.value='base_link'
    robot=Mock()
    CabinetTaskNode._move_to_qr_observation_pose(node,
        {'name':'knob','observation_pose':{'pose':[0,0,0,0,0,0],'settle_s':0}},
        robot,fresh_sampling=True)
    robot.move_pose.assert_called_once()
    node.camera_cache.mark_fresh.assert_not_called()
    node.camera_cache.wait_fresh.assert_not_called()


def test_one_confirmed_frame_can_locate_far_target_and_bad_frame_still_retries():
    policy=SamplingPolicy.from_config({'confirmation_frames':1})
    (result,report),detect,failed=run(Stream(),[target()],policy)
    assert report['attempts']==report['confirmations']==1
    assert result.details['axis_center_pixel']==[80,80]
    assert detect.call_count==1 and failed.call_count==0
    (result,report),detect,failed=run(Stream(),[PanelLayoutRejected('bad image'),target()],policy)
    assert report['attempts']==2 and report['confirmations']==1
    assert failed.call_count==1 and detect.call_count==2


def test_zero_confirmation_is_rejected():
    with pytest.raises(ValueError,match='confirmation_frames'):
        SamplingPolicy.from_config({'confirmation_frames':0})
