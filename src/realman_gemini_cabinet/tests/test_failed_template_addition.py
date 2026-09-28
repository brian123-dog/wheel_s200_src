from pathlib import Path
import json
import cv2
import numpy as np
import yaml
from rg_cabinet_ros2.panel_layout import PanelLayoutDetector
from rg_cabinet_ros2.panel_template_bank import locate_with_templates


def configured_bank():
    root=Path(__file__).resolve().parents[1]
    config=root/'config/ros2_qr_panel_layout_movel_reroute.yaml'
    paths=yaml.safe_load(config.read_text())['vision']['targets']['middle_black_knob']['panel_layout_templates']
    return {str(p):PanelLayoutDetector.from_template(config.parent/p) for p in paths}


def test_manually_annotated_distant_reference_is_usable_without_lowering_thresholds():
    detectors=configured_bank()
    selected=detectors['panel_layout/template_distant_panel.json']
    original=detectors['panel_layout/template.json']
    for key in ['target_id','min_inliers','min_inlier_ratio','min_anchor_span_fraction','max_median_reprojection_px']:
        assert selected.config[key]==original.config[key]
    _,r,_=locate_with_templates(selected.reference,detectors)
    assert r['status']=='ACCEPTED_2D'
    assert np.linalg.norm(np.asarray(r['axis_center_px'])-[385,369])<2


def test_real_failed_frame_without_cabinet_stays_rejected():
    image=cv2.imread(str(Path(__file__).parent/'data/panel_no_cabinet_20260923_153003.png'))
    assert image is not None
    _,r,_=locate_with_templates(image,configured_bank())
    assert r['status']=='REJECTED'
