"""Replay held-out images from the 14:01 failed task; no robot commands."""
from pathlib import Path
import cv2
import numpy as np
import pytest
from rg_cabinet_ros2.panel_layout import PanelLayoutDetector
from rg_cabinet_ros2.panel_template_bank import locate_with_templates
ROOT=Path(__file__).resolve().parents[1]

@pytest.fixture(scope='module')
def bank():
    folder=ROOT/'config/panel_layout_right_knob/precontact'
    return {name:PanelLayoutDetector.from_template(folder/name)
            for name in ['template.json','template_1401.json']}

def test_heldout_sequence_has_consecutive_accepted_centers(bank):
    results=[]
    for i in range(1,5):
        im=cv2.imread(str(ROOT/f'tests/data/right_knob_near_1401_holdout_{i}.png'))
        _,r,_=locate_with_templates(im,bank);results.append(r)
    # Frame 2 remains rejected: evidence checks must not be bypassed.
    assert [r['status'] for r in results]==['ACCEPTED_2D','REJECTED','ACCEPTED_2D','ACCEPTED_2D']
    for r in [results[0],results[2],results[3]]:
        assert r['registration']['inliers']>=20
        assert min(r['registration']['anchor_span_fraction'])>=.35
        assert np.linalg.norm(np.array(r['axis_center_px'])-[636,440])<3
    assert np.linalg.norm(np.array(results[2]['axis_center_px'])-results[3]['axis_center_px'])<1

def test_old_heldout_views_still_pass_with_expanded_bank(bank):
    for i in range(1,5):
        _,r,_=locate_with_templates(cv2.imread(str(ROOT/f'tests/data/right_knob_near_holdout_{i}.png')),bank)
        assert r['status']=='ACCEPTED_2D',r['reason']
        assert np.linalg.norm(np.array(r['axis_center_px'])-[637,441])<3

@pytest.mark.parametrize('reference',['template.json','template_1401.json'])
@pytest.mark.parametrize('case',['missing','occluded','yellow'])
def test_both_templates_reject_damaged_target(bank,reference,case):
    detector=bank[reference];im=detector.reference.copy()
    if case=='missing':cv2.fillPoly(im,[np.int32(detector.search)],(145,145,145))
    elif case=='occluded':cv2.rectangle(im,(595,400),(638,493),(145,145,145),-1)
    else:cv2.circle(im,(637,441),28,(0,210,220),-1)
    _,r,_=locate_with_templates(im,bank)
    assert r['status']=='REJECTED',r
