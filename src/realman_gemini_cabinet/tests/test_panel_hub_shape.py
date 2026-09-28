from pathlib import Path

import cv2
import numpy as np

from rg_cabinet_ros2.panel_layout import PanelLayoutDetector


PACKAGE = Path(__file__).resolve().parents[1]


def detector():
    return PanelLayoutDetector.from_template(PACKAGE/'config/panel_layout/template.json')


def test_real_handle_highlight_is_not_a_second_shaft_cap():
    image=cv2.imread(str(PACKAGE/'tests/data/panel_layout_hub_highlight.png'))
    assert image is not None
    result=detector().process(image)
    assert result['status']=='ACCEPTED_2D'
    np.testing.assert_allclose(result['axis_center_px'],[641.46,393.82],atol=2.)
    metrics=result['refinement']
    assert metrics['hub_cap_candidates']==1
    assert any(item['circularity']<.5 for item in metrics['hub_cap_shape_rejections'])


def synthetic_hub(shapes):
    d=detector()
    image=d.reference.copy()
    patch=np.full((160,160,3),155,dtype=np.uint8)
    cv2.rectangle(patch,(1,1),(158,158),(15,15,15),4)
    cv2.circle(patch,(80,80),52,(15,15,15),-1)
    for center,axes in shapes:
        cv2.ellipse(patch,center,axes,0,0,360,(230,230,230),-1)
    transform=cv2.getPerspectiveTransform(np.float32([[0,0],[159,0],[159,159],[0,159]]),d.plate)
    painted=cv2.warpPerspective(patch,transform,(image.shape[1],image.shape[0]))
    mask=cv2.warpPerspective(np.full((160,160),255,np.uint8),transform,(image.shape[1],image.shape[0]))
    image[mask>0]=painted[mask>0]
    return d,image


def test_two_round_caps_remain_ambiguous():
    d,image=synthetic_hub([((64,68),(7,7)),((96,100),(7,7))])
    refined,metrics,error=d.refine(image,np.eye(3))
    assert metrics['hub_cap_candidates']==2
    assert error=='Missing or ambiguous central hub cap'
    assert 'center' not in refined


def test_only_an_elongated_highlight_cannot_supply_axis_center():
    d,image=synthetic_hub([((80,80),(14,3))])
    refined,metrics,error=d.refine(image,np.eye(3))
    assert metrics['hub_cap_candidates']==0
    assert error=='Missing or ambiguous central hub cap'
    assert 'center' not in refined
