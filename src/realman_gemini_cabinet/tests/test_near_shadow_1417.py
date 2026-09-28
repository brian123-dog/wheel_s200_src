"""Measured failure replay and negative controls for reference-local shadow segmentation."""
from pathlib import Path
import json
import cv2
import numpy as np
import pytest
from rg_cabinet_ros2.panel_layout import PanelLayoutDetector
from rg_cabinet_ros2.panel_template_bank import locate_with_templates
from rg_cabinet_ros2.round_knob import refine_round_knob
ROOT=Path(__file__).resolve().parents[1]
FOLDER=ROOT/'config/panel_layout_right_knob/precontact'

@pytest.fixture(scope='module')
def bank():
    return {p.name:PanelLayoutDetector.from_template(p) for p in sorted(FOLDER.glob('template*.json'))}

def test_failed_task_replay_consistent_centers_and_strict_circle_evidence(bank):
    centers=[]
    for i in range(1,5):
        im=cv2.imread(str(ROOT/f'tests/data/right_knob_near_1417_replay_{i}.png'))
        _,r,_=locate_with_templates(im,bank)
        assert r['status']=='ACCEPTED_2D',r['reason']
        assert r['registration']['inliers']>=20
        q=r['refinement'];assert q['boundary_coverage']>=.78
        assert q['max_missing_arc_fraction']<=.20
        assert q['dark_support']>=.8 and q['contrast_support']>=.72
        centers.append(r['axis_center_px'])
    centers=np.array(centers)
    assert np.linalg.norm(centers-[637,442],axis=1).max()<2
    assert np.linalg.norm(centers[:,None]-centers[None,:],axis=2).max()<2

def test_new_template_passes_its_reference_and_previous_views(bank):
    d=bank['template_1417.json'];r=d.process(d.reference)
    assert r['status']=='ACCEPTED_2D',r['reason']
    for pattern in ['right_knob_near_holdout_*.png','right_knob_near_1401_holdout_*.png']:
        passed=[]
        for p in sorted((ROOT/'tests/data').glob(pattern)):
            _,r,_=locate_with_templates(cv2.imread(str(p)),bank)
            if r['status']=='ACCEPTED_2D':
                assert np.linalg.norm(np.array(r['axis_center_px'])-[636,441])<4
                passed.append(True)
            else:passed.append(False)
        assert passed[2:]==[True,True]
        if pattern=='right_knob_near_holdout_*.png':assert all(passed)

@pytest.mark.parametrize('reference',['template.json','template_1401.json','template_1417.json'])
@pytest.mark.parametrize('case',['missing','occluded','yellow','rim_only'])
def test_expanded_bank_rejects_missing_or_wrong_body(bank,reference,case):
    d=bank[reference];im=d.reference.copy()
    if case=='missing':cv2.fillPoly(im,[np.int32(d.search)],(145,145,145))
    elif case=='occluded':cv2.rectangle(im,(595,400),(638,493),(145,145,145),-1)
    elif case=='yellow':cv2.circle(im,(637,441),29,(0,210,220),-1)
    else:
        cv2.fillPoly(im,[np.int32(d.search)],(145,145,145))
        cv2.circle(im,(637,442),28,(90,90,90),3)
    _,r,_=locate_with_templates(im,bank)
    assert r['status']=='REJECTED',r

@pytest.mark.parametrize('bad',[0,.49,1.01,float('nan'),float('inf'),True,'0.8'])
def test_bad_segmentation_setting_rejected(bank,bad):
    d=bank['template_1417.json']
    with pytest.raises(ValueError,match='body_threshold_scale'):
        PanelLayoutDetector(d.reference,{**d.config,'body_threshold_scale':bad})

def test_default_preserves_otsu_and_stricter_setting_rejects_cast_shadow(bank):
    d=bank['template_1417.json']
    old={k:v for k,v in d.config.items() if k!='body_threshold_scale'}
    r,stats,error=refine_round_knob(d.reference,d.reference.shape,d.search,old,np.eye(3))
    assert r is None and error is not None
    assert stats['body_segmentation_threshold']==stats['body_otsu_threshold']
    r,stats,error=refine_round_knob(d.reference,d.reference.shape,d.search,d.config,np.eye(3))
    assert error is None
    assert np.linalg.norm(r['center']-[637,442])<2
    assert stats['body_segmentation_threshold']==pytest.approx(.8*stats['body_otsu_threshold'])
