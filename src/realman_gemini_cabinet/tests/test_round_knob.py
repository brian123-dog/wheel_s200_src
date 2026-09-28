from pathlib import Path
import copy
import math
import cv2
import numpy as np
import pytest
import yaml
from rg_cabinet_ros2.panel_layout import PanelLayoutDetector
from rg_cabinet_ros2.round_knob import refine_round_knob
from rg_cabinet.qr_targets import QrTargetSet, overlay_qr_observation_poses
from rg_cabinet.geometry import Pose

ROOT=Path(__file__).resolve().parents[1]
NAME='upper_row_right_black_knob'

@pytest.fixture
def detector():
    return PanelLayoutDetector.from_template(ROOT/'config/panel_layout_right_knob/template_upper_row_right.json')

def test_real_reference_selects_upper_row_not_highlight_or_lower_row(detector):
    r=detector.process(detector.reference)
    assert r['status']=='ACCEPTED_2D'
    assert np.linalg.norm(np.array(r['axis_center_px'])-[660,486]) < 1.5
    assert r['plate_corners'] is None
    assert len(r['knob_outline_px'])==72

@pytest.mark.parametrize('case',['upper_missing','half_occluded','blank','yellow_body'])
def test_rejects_missing_or_occluded_target_even_with_lower_knob_visible(detector,case):
    image=detector.reference.copy()
    if case=='blank': image[:]=160
    elif case=='upper_missing': cv2.rectangle(image,(633,459),(687,522),(160,160,160),-1)
    elif case=='half_occluded': cv2.rectangle(image,(633,459),(660,522),(160,160,160),-1)
    else: cv2.circle(image,(660,486),17,(0,220,230),-1)
    assert detector.process(image)['status']=='REJECTED'

@pytest.mark.parametrize('angle',[0,45,90,180,270])
def test_axis_is_independent_of_handle_direction_and_highlight(detector,angle):
    image=detector.reference.copy()
    cv2.rectangle(image,(634,459),(687,523),(180,180,180),-1)
    cv2.circle(image,(660,486),15,(25,25,25),-1)
    end=tuple(np.rint(np.array([660,486])+25*np.array([np.cos(np.deg2rad(angle)),np.sin(np.deg2rad(angle))])).astype(int))
    cv2.line(image,(660,486),end,(25,25,25),5)
    cv2.circle(image,(658,480),2,(240,240,240),-1)
    r,_,error=refine_round_knob(image,detector.reference.shape,detector.search,detector.config,np.eye(3))
    assert error is None
    assert np.linalg.norm(r['center']-[660,486])<1.5

def test_round_geometry_uses_existing_depth_and_tf_resolver(detector):
    from test_panel_layout_mode import resolver_for
    cfg={'vision':{'targets':{NAME:{'locator':'panel_layout','kind':'knob',
        'panel_layout_template':'panel_layout_right_knob/template_upper_row_right.json',
        'panel_layout_target_id':NAME,'min_depth_m':.1,'max_depth_m':1.2}}}}
    step={'name':NAME,'skill':'knob_camera_servo','target':{'source':'camera','detection':NAME,'kind':'knob'}}
    target=resolver_for(detector.reference).resolve(cfg,step)
    assert target.details['local_geometry']=='round_knob'
    assert target.details['candidate_count']==1
    assert target.details['plate_corners'] is None
    assert target.details['depth_m']==pytest.approx(.4)
    assert target.details['depth_source']=='panel_layout_bbox_ring'

def config():
    return yaml.safe_load((ROOT/'config/ros2_qr_panel_layout_two_knobs.yaml').read_text())

def test_fixed_observation_survives_qr_overlay_and_preserves_original_steps():
    cfg=config()
    old=yaml.safe_load((ROOT/'config/ros2_qr_panel_layout_movel_reroute.yaml').read_text())
    assert [s for s in cfg['sequence'] if s['name'] != NAME]==old['sequence']
    middle=cfg['vision']['targets']['middle_black_knob']
    original=old['vision']['targets']['middle_black_knob']
    # Each mode may carry its operator's TCP calibration and compute budget.
    # All other middle-knob detection settings and motion steps are preserved.
    mode_specific={'fresh_sampling','tcp_offset_m'}
    assert {k:v for k,v in middle.items() if k not in mode_specific} == {
        k:v for k,v in original.items() if k not in mode_specific}
    assert middle['fresh_sampling']=={**cfg['vision']['fresh_sampling'],'total_timeout_s':12.0}
    qr=QrTargetSet('test', [Pose(1,2,3),Pose(4,5,6)], names=['upper_meters','middle_black_knob'])
    updated=overlay_qr_observation_poses(cfg,qr)
    assert updated['vision']['targets']['middle_black_knob']['tcp_offset_m']==middle['tcp_offset_m']
    new=updated['sequence'][2]
    assert new['observation_pose']==cfg['sequence'][2]['observation_pose']
    assert new['observation_pose']['position']['x']==-0.3334310054779053
    assert new['params']['turn_angle_rad']==pytest.approx(math.pi/3)
    assert cfg['sequence'][1]['params']['turn_angle_rad']==1.5708
    assert new['params']['release_after_turn']
    assert not __import__('rg_cabinet_ros2.task_node',fromlist=['CabinetTaskNode']).CabinetTaskNode._step_skips_observation_pose(new)

@pytest.mark.parametrize('bad',['frame','zero_quaternion','nan','missing_pose','qr_name'])
def test_invalid_fixed_pose_rejected_before_sequence(bad):
    cfg=config();obs=cfg['sequence'][2]['observation_pose']
    if bad=='frame':obs['frame']='map'
    elif bad=='zero_quaternion':obs['orientation']=dict(x=0,y=0,z=0,w=0)
    elif bad=='nan':obs['position']['x']=float('nan')
    elif bad=='missing_pose':obs.pop('orientation')
    else:obs['qr_name']='unused'
    qr=QrTargetSet('test',[Pose(1,2,3),Pose(4,5,6)],names=['upper_meters','middle_black_knob'])
    with pytest.raises(ValueError):overlay_qr_observation_poses(cfg,qr)

@pytest.mark.parametrize('suffix',['right','smallest'])
def test_additional_view_templates(suffix):
    d=PanelLayoutDetector.from_template(ROOT/f'config/panel_layout_right_knob/template_upper_row_right_{suffix}.json')
    r=d.process(d.reference)
    assert r['status']=='ACCEPTED_2D'
    assert abs(r['axis_center_px'][1]-d.config['axis_center_reference_px'][1]) < 3
    missing=d.reference.copy()
    cv2.fillPoly(missing,[np.int32(d.search)],(160,160,160))
    assert d.process(missing)['status']=='REJECTED'

def test_fixed_observation_is_sent_to_driver_with_original_quaternion():
    from types import SimpleNamespace
    from unittest.mock import Mock, patch
    from rg_cabinet_ros2.task_node import CabinetTaskNode
    step=config()['sequence'][2]
    node=SimpleNamespace(_report_task_stage=Mock(),get_parameter=lambda n:SimpleNamespace(value='base_link' if n=='base_frame' else 0))
    node._read_observation_quaternion_pose=lambda observation,step:CabinetTaskNode._read_observation_quaternion_pose(node,observation,step)
    robot=SimpleNamespace(move_joint_pose=Mock())
    with patch('rg_cabinet_ros2.task_node.time.sleep'):
        CabinetTaskNode._move_to_qr_observation_pose(node,step,robot,fresh_sampling=True)
    robot.move_joint_pose.assert_called_once_with(step['observation_pose']['position'],step['observation_pose']['orientation'],speed=.05,wait=True)

@pytest.mark.parametrize('suffix',['','_right','_smallest'])
def test_full_round_template_bank_rejects_missing_upper_knob(suffix):
    from rg_cabinet_ros2.panel_template_bank import locate_with_templates
    paths=list((ROOT/'config/panel_layout_right_knob').glob('template_*.json'))
    detectors={str(p):PanelLayoutDetector.from_template(p) for p in paths}
    d=PanelLayoutDetector.from_template(ROOT/f'config/panel_layout_right_knob/template_upper_row_right{suffix}.json')
    image=d.reference.copy()
    cv2.fillPoly(image,[np.int32(d.search)],(160,160,160))
    _,r,_=locate_with_templates(image,detectors)
    assert r['status']=='REJECTED'
