"""Regression: silver bezel and black body must not share fitting evidence."""
from pathlib import Path
import cv2
import numpy as np
import pytest
from rg_cabinet_ros2.panel_layout import PanelLayoutDetector
from rg_cabinet_ros2.panel_template_bank import locate_with_templates
from rg_cabinet_ros2.round_knob import refine_round_knob

ROOT=Path(__file__).resolve().parents[1]

@pytest.fixture(scope='module')
def detectors():
    return {str(p):PanelLayoutDetector.from_template(p)
            for p in (ROOT/'config/panel_layout_right_knob').glob('template_*.json')}

def test_two_real_ambiguous_frames_recover_consistent_black_body_centers(detectors):
    centers=[]
    for n in [1,2]:
        image=cv2.imread(str(ROOT/f'tests/data/right_knob_bezel_ambiguity_{n}.png'))
        assert image is not None
        _,result,_=locate_with_templates(image,detectors)
        assert result['status']=='ACCEPTED_2D',result['reason']
        assert result['refinement']['edge_source']=='dark_body_silhouette'
        # Image annotation is on the black circular face, not its offset rim.
        assert np.linalg.norm(np.array(result['axis_center_px'])-[865,352]) < 2
        centers.append(result['axis_center_px'])
    assert np.linalg.norm(np.array(centers[0])-centers[1]) < 2

@pytest.mark.parametrize('bezel_value',[120,155])
@pytest.mark.parametrize('body_present',[True,False])
def test_offset_silver_bezel_is_not_used_as_body(detectors,bezel_value,body_present):
    d=next(v for v in detectors.values() if v.config['reference_image']=='reference_left.png')
    image=d.reference.copy()
    cv2.rectangle(image,(634,459),(687,523),(195,195,195),-1)
    cv2.circle(image,(656,486),19,(bezel_value,)*3,-1)
    if body_present:
        cv2.circle(image,(660,486),15,(25,)*3,-1)
        cv2.line(image,(660,486),(660,513),(25,)*3,5)
        cv2.circle(image,(658,480),2,(240,)*3,-1)
    result,_,error=refine_round_knob(image,d.reference.shape,d.search,d.config,np.eye(3))
    if body_present:
        assert error is None
        assert np.linalg.norm(result['center']-[660,486]) < 1.5
    else:
        assert error is not None

@pytest.mark.parametrize('view',[1,2,3,4])
def test_user_reference_annotations_are_excluded_from_matching(view):
    d=PanelLayoutDetector.from_template(ROOT/f'config/panel_layout_right_knob/template_user_front_{view}.json')
    clean=d.reference.copy()
    for points in d.config['anchor_exclusion_polygons']:
        cv2.fillPoly(clean,[np.int32(points)],(180,180,180))
    a=d.process(d.reference);b=d.process(clean)
    assert a['status']==b['status']=='ACCEPTED_2D'
    assert np.linalg.norm(np.array(a['axis_center_px'])-b['axis_center_px']) < 1.5

def test_right_and_middle_template_banks_have_separate_paths_and_target_ids():
    import yaml,json
    cfg=yaml.safe_load((ROOT/'config/ros2_qr_panel_layout_two_knobs.yaml').read_text())
    for name,directory in [('middle_black_knob','panel_layout'),('upper_row_right_black_knob','panel_layout_right_knob')]:
        for value in cfg['vision']['targets'][name]['panel_layout_templates']:
            path=Path(value)
            assert path.parts[0]==directory
            assert json.loads((ROOT/'config'/path).read_text())['target_id']==name
