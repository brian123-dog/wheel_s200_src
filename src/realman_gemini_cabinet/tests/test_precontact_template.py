from pathlib import Path
from dataclasses import replace
import cv2
import numpy as np
import pytest
import yaml
from rg_cabinet_ros2.panel_layout import PanelLayoutDetector, project
from rg_cabinet.skills.base import SkillContext
from rg_cabinet.task_runner import _step_for_refresh
from test_precontact_centering import Robot, make_case, run

ROOT=Path(__file__).resolve().parents[1]

@pytest.fixture(scope='module')
def detector():
    return PanelLayoutDetector.from_template(ROOT/'config/panel_layout_right_knob/precontact/template.json')


def test_four_heldout_failed_frames_now_match_consistently(detector):
    centers=[]
    for i in range(1,5):
        r=detector.process(cv2.imread(str(ROOT/f'tests/data/right_knob_near_holdout_{i}.png')))
        assert r['status']=='ACCEPTED_2D',r['reason']
        assert r['registration']['inliers']>=20
        centers.append(r['axis_center_px'])
    centers=np.array(centers)
    assert np.linalg.norm(centers-[637,441],axis=1).max()<3
    assert np.linalg.norm(centers[:,None]-centers[None,:],axis=2).max()<2


@pytest.mark.parametrize('case',['missing','occluded','yellow'])
def test_near_template_rejects_bad_target_even_with_lower_knob_visible(detector,case):
    im=detector.reference.copy()
    if case=='missing':cv2.fillPoly(im,[np.int32(detector.search)],(145,145,145))
    elif case=='occluded':cv2.rectangle(im,(595,400),(638,493),(145,145,145),-1)
    else:cv2.circle(im,(637,441),28,(0,210,220),-1)
    r=detector.process(im)
    assert r['status']=='REJECTED'


@pytest.mark.parametrize('angle,scale,dx,dy',[(-4,.9,15,-10),(4,1.1,-15,10),(0,1.,25,20)])
def test_small_view_changes_preserve_target_identity(detector,angle,scale,dx,dy):
    M=cv2.getRotationMatrix2D((637,441),angle,scale);M[:,2]+=[dx,dy]
    H=np.vstack([M,[0,0,1]])
    im=cv2.warpPerspective(detector.reference,H,(1280,720))
    r=detector.process(im)
    assert r['status']=='ACCEPTED_2D',r['reason']
    assert np.linalg.norm(np.array(r['axis_center_px'])-project([[637,441]],H)[0])<3


def test_only_near_refresh_switches_reference_bank():
    cfg=yaml.safe_load((ROOT/'config/ros2_qr_panel_layout_two_knobs.yaml').read_text())
    step=next(s for s in cfg['sequence'] if s['name']=='upper_row_right_black_knob')
    context=SkillContext(Robot())
    assert _step_for_refresh(step,context) is step
    context.precontact_centering=True
    near=_step_for_refresh(step,context)
    assert near['target']['panel_layout_templates']==[
        'panel_layout_right_knob/precontact/template.json',
        'panel_layout_right_knob/precontact/template_1401.json',
        'panel_layout_right_knob/precontact/template_1417.json']
    assert len(cfg['vision']['targets'][step['name']]['panel_layout_templates'])==8
    assert 'panel_layout_templates' not in step['target']
    assert near['target']['save_debug_images']


@pytest.mark.parametrize('paths',['bad/path.json',[None],['']])
def test_invalid_near_template_list_is_rejected_before_motion(paths):
    robot,context,skill=make_case([],precontact_panel_layout_templates=paths)
    with pytest.raises(ValueError):run(skill,context)
    assert robot.commands==[]


def test_right_far_and_near_sampling_policies_are_separate():
    from rg_cabinet_ros2.vision_target_resolver import VisionTargetResolver
    from rg_cabinet_ros2.fresh_vision import SamplingPolicy
    cfg=yaml.safe_load((ROOT/'config/ros2_qr_panel_layout_two_knobs.yaml').read_text())
    step=next(s for s in cfg['sequence'] if s['name']=='upper_row_right_black_knob')
    resolver=VisionTargetResolver.__new__(VisionTargetResolver)
    far=SamplingPolicy.from_config(resolver.fresh_sampling_settings(cfg,step))
    assert far.confirmation_frames==1
    assert step['params']['median_samples']==1
    context=SkillContext(Robot(),precontact_centering=True)
    near_step=_step_for_refresh(step,context)
    near=SamplingPolicy.from_config(resolver.fresh_sampling_settings(cfg,near_step))
    assert near.confirmation_frames==2
    assert near.max_attempts==far.max_attempts==5
    assert near.max_frame_age_s==far.max_frame_age_s==.75
    assert near.max_rgbd_skew_s==far.max_rgbd_skew_s==.15
    assert near.total_timeout_s==far.total_timeout_s==12
    assert near_step['target']['panel_layout_templates']==step['params']['precontact_panel_layout_templates']
    assert 'fresh_sampling' not in step['target']
