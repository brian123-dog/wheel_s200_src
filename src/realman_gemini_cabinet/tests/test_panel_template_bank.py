from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock
import json
import numpy as np
import pytest

from rg_cabinet_ros2.panel_layout import PanelLayoutDetector
from rg_cabinet_ros2.panel_template_bank import locate_with_templates


def result(x=80,inliers=50,status='ACCEPTED_2D'):
    return {'status':status,'reason':'ok' if status=='ACCEPTED_2D' else 'weak',
        'axis_center_px':[x,80],'plate_corners':[[60,60],[100,60],[100,100],[60,100]],
        'prediction_polygon':[[50,50],[110,50],[110,110],[50,110]],
        'registration':{'inliers':inliers,'inlier_ratio':.8,'median_reprojection_px':.3}}


def bank(*results):
    return {str(i):SimpleNamespace(sift=Mock(),process=Mock(return_value=r)) for i,r in enumerate(results)}


def test_multiple_templates_share_one_scene_feature_extraction():
    detectors=bank(result(),result(81,70))
    key,chosen,summary=locate_with_templates(np.zeros((32,32,3),np.uint8),detectors)
    assert key=='1' and chosen['status']=='ACCEPTED_2D'
    assert detectors['0'].sift.detectAndCompute.call_count==1
    assert detectors['1'].sift.detectAndCompute.call_count==0
    assert detectors['0'].process.call_args.kwargs['features'] is detectors['1'].process.call_args.kwargs['features']


def test_template_disagreement_rejected_despite_high_inlier_count():
    key,chosen,_=locate_with_templates(np.zeros((32,32,3),np.uint8),bank(result(80,200),result(95,100)))
    assert chosen['status']=='REJECTED'
    assert chosen['reason']=='Conflicting template localizations'


def test_additional_template_can_recover_weak_original():
    _,chosen,summary=locate_with_templates(np.zeros((32,32,3),np.uint8),
        bank(result(inliers=9,status='REJECTED'),result(inliers=60)))
    assert summary[0]['status']=='REJECTED' and chosen['status']=='ACCEPTED_2D'


def test_all_rejected_templates_remain_rejected():
    _,chosen,_=locate_with_templates(np.zeros((32,32,3),np.uint8),
        bank(result(inliers=9,status='REJECTED'),result(inliers=10,status='REJECTED')))
    assert chosen['status']=='REJECTED'


@pytest.mark.parametrize('name',['largest','smallest','left','right'])
def test_added_reference_is_valid_and_keeps_original_quality_thresholds(name):
    root=Path(__file__).resolve().parents[1]/'config/panel_layout'
    original=json.loads((root/'template.json').read_text())
    detector=PanelLayoutDetector.from_template(root/f'template_{name}.json')
    for key in ['target_id','min_inliers','min_inlier_ratio','min_anchor_span_fraction','max_median_reprojection_px']:
        assert detector.config[key]==original[key]
    assert detector.process(detector.reference)['status']=='ACCEPTED_2D'


@pytest.mark.parametrize('occluded',[False,True])
def test_bank_rejects_blank_scene_or_missing_knob(occluded):
    import cv2
    root=Path(__file__).resolve().parents[1]/'config/panel_layout'
    paths=[root/'template.json']+[root/f'template_{name}.json' for name in ['largest','smallest','left','right']]
    detectors={str(p):PanelLayoutDetector.from_template(p) for p in paths}
    ref=detectors[str(paths[0])]
    image=ref.reference.copy() if occluded else np.zeros_like(ref.reference)
    if occluded:
        cv2.fillPoly(image,[np.int32(ref.search)],(160,160,160))
    _,result,_=locate_with_templates(image,detectors)
    assert result['status']=='REJECTED'
