import copy
import json
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch

import cv2
import numpy as np
import yaml

from rg_cabinet_ros2.camera_cache import CameraFrameCache
from rg_cabinet_ros2.panel_layout import PanelLayoutDetector
from rg_cabinet_ros2.vision_target_resolver import VisionTargetResolver
from test_vision_target_resolver import _FakeNode, _FakeTfBuffer


PACKAGE = Path(__file__).resolve().parents[1]
TEMPLATE = PACKAGE / 'config/panel_layout/template.json'
REFERENCE = PACKAGE / 'config/panel_layout/reference.png'


class Node(_FakeNode):
    def get_parameter(self, name):
        if name == 'task_config':
            return SimpleNamespace(value=str(PACKAGE / 'config/ros2_qr_panel_layout_demo.yaml'))
        return super().get_parameter(name)


def resolver_for(bgr, depth_mm=400):
    h, w = bgr.shape[:2]
    # Exercise rgb8 conversion used by the actual camera.
    color = SimpleNamespace(width=w, height=h, step=w*3, encoding='rgb8',
                            data=cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB).tobytes(),
                            header=SimpleNamespace(frame_id='camera_color_optical_frame'))
    depth = SimpleNamespace(width=w, height=h, step=w*2, encoding='16UC1', is_bigendian=0,
                            data=np.full((h, w), depth_mm, dtype='<u2').tobytes())
    info = SimpleNamespace(k=[600., 0., w/2, 0., 600., h/2, 0., 0., 1.])
    resolver = VisionTargetResolver(Node(), CameraFrameCache(color, depth, info), _FakeTfBuffer())
    resolver._publish_target_pose = lambda _target: None
    return resolver


class PanelLayoutModeTests(unittest.TestCase):
    def setUp(self):
        self.image = cv2.imread(str(REFERENCE))
        self.config = {'vision': {'save_debug_images': False, 'targets': {
            'middle_black_knob': {'locator': 'panel_layout', 'kind': 'knob',
                                 'panel_layout_template': 'panel_layout/template.json',
                                 'min_depth_m': .1, 'max_depth_m': 1.2,
                                 'desired_pixel': [640, 322]}}}}
        self.step = {'name': 'turn_knob', 'skill': 'knob_camera_servo',
                     'target': {'source': 'camera', 'detection': 'middle_black_knob', 'kind': 'knob'}}

    def test_new_mode_full_resolve_with_relative_template_and_depth(self):
        resolver = resolver_for(self.image)
        target = resolver.resolve(self.config, self.step)
        self.assertEqual(target.details['locator'], 'panel_layout')
        self.assertAlmostEqual(target.details['pixel'][0], 597.54, delta=2.)
        self.assertAlmostEqual(target.details['pixel'][1], 405.99, delta=2.)
        self.assertEqual(target.details['depth_source'], 'panel_layout_bbox_ring')
        self.assertAlmostEqual(target.details['depth_m'], .4)
        self.assertIn('pixel_error_base_m', target.details)
        self.assertEqual(len(resolver.resolved_targets), 1)
        self.assertEqual(len(resolver._panel_layout_detectors), 1)
        resolver.reset()
        self.assertEqual(resolver._panel_layout_detectors, {})

    def test_override_can_select_new_mode_without_mutating_named_legacy_config(self):
        self.config['vision']['targets']['middle_black_knob']['locator'] = 'gray_plate'
        self.config['vision']['targets']['middle_black_knob'].pop('panel_layout_template')
        before = copy.deepcopy(self.config)
        self.step['target'].update(locator='panel_layout', panel_layout_template=str(TEMPLATE))
        target = resolver_for(self.image).resolve(self.config, self.step)
        self.assertEqual(target.details['locator'], 'panel_layout')
        self.assertEqual(self.config, before)

    def test_rejection_never_uses_color_blob_fallback(self):
        cv2.rectangle(self.image, (540, 345), (660, 465), (150, 150, 150), -1)
        resolver = resolver_for(self.image)
        with patch('rg_cabinet_ros2.vision_target_resolver.find_color_blobs', side_effect=AssertionError('legacy detector used')):
            with self.assertRaisesRegex(RuntimeError, 'Panel-layout target rejected: No complete plate'):
                resolver.resolve(self.config, self.step)
        self.assertEqual(resolver.last_target_debug['locator'], 'panel_layout')
        self.assertEqual(resolver.resolved_targets, [])

    def test_hidden_cap_does_not_return_plate_center_as_axis(self):
        cv2.rectangle(self.image, (586, 394), (609, 418), (30, 30, 30), -1)
        with self.assertRaisesRegex(RuntimeError, 'Missing or ambiguous central hub cap'):
            resolver_for(self.image).resolve(self.config, self.step)

    def test_real_thin_border_failure_is_recovered_without_changing_target(self):
        image = cv2.imread(str(PACKAGE / 'tests/data/panel_layout_border_gap.png'))
        self.assertIsNotNone(image)
        target = resolver_for(image).resolve(self.config, self.step)
        self.assertEqual(target.details['locator'], 'panel_layout')
        self.assertAlmostEqual(target.details['pixel'][0], 611.06, delta=2.)
        self.assertAlmostEqual(target.details['pixel'][1], 388.12, delta=2.)
        metrics = target.details['panel_layout']['refinement']
        self.assertGreaterEqual(min(metrics['plate_side_support']), .8)

    def test_long_missing_border_still_rejected(self):
        # Remove a substantial part of the left edge while keeping the hub
        # visible. The short-gap repair must not invent a complete plate.
        cv2.rectangle(self.image, (555, 381), (572, 425), (150, 150, 150), -1)
        with self.assertRaisesRegex(RuntimeError, 'No complete plate contour'):
            resolver_for(self.image).resolve(self.config, self.step)

    def test_existing_depth_and_workspace_rejections_still_apply(self):
        cfg = self.config['vision']['targets']['middle_black_knob']
        cfg['max_depth_m'] = .3
        with self.assertRaisesRegex(RuntimeError, 'farther than max_depth_m'):
            resolver_for(self.image).resolve(self.config, self.step)
        cfg['max_depth_m'] = 1.2
        cfg['workspace_bounds'] = {'z': [.7, .9]}
        with self.assertRaisesRegex(RuntimeError, 'outside workspace'):
            resolver_for(self.image).resolve(self.config, self.step)

    def test_wrong_template_identity_is_rejected(self):
        self.step['target']['panel_layout_target_id'] = 'another_knob'
        with self.assertRaisesRegex(RuntimeError, 'target_id does not match'):
            resolver_for(self.image).resolve(self.config, self.step)

    def test_missing_template_is_an_error_without_legacy_fallback(self):
        self.step['target']['panel_layout_template'] = str(TEMPLATE.parent / 'missing.json')
        with self.assertRaises(FileNotFoundError):
            resolver_for(self.image).resolve(self.config, self.step)

    def test_rejected_frame_still_saves_overlay(self):
        resolver = resolver_for(np.full_like(self.image, 150))
        with tempfile.TemporaryDirectory() as directory:
            self.config['vision'].update(save_debug_images=True, vision_debug_dir=directory)
            with self.assertRaisesRegex(RuntimeError, 'Insufficient image features'):
                resolver.resolve(self.config, self.step)
            path = resolver.last_target_debug['debug_images']['overlay_path']
            self.assertIsNotNone(cv2.imread(path))
            images = resolver.last_target_debug['debug_images']
            raw = cv2.imread(images['color_path'])
            np.testing.assert_array_equal(raw, np.full_like(self.image, 150))
            metadata = json.loads(Path(images['metadata_path']).read_text())
            self.assertEqual(metadata['panel_layout']['status'], 'REJECTED')

    def test_old_gray_plate_mode_keeps_dispatch_and_depth_path(self):
        resolver = resolver_for(self.image)
        self.config['vision']['targets']['middle_black_knob']['locator'] = 'gray_plate'
        with patch.object(resolver, '_resolve_gray_plate_target', return_value=(600., 405., .4, 'gray_plate_bbox_ring', {'locator': 'gray_plate'})) as legacy:
            with patch.object(resolver, '_resolve_panel_layout_target', side_effect=AssertionError('new detector used')):
                target = resolver.resolve(self.config, self.step)
        legacy.assert_called_once()
        self.assertEqual(target.details['locator'], 'gray_plate')

    def test_new_task_configuration_changes_only_detection_selection(self):
        old = yaml.safe_load((PACKAGE / 'config/ros2_qr_named_panel_demo.yaml').read_text())
        new = yaml.safe_load((PACKAGE / 'config/ros2_qr_panel_layout_demo.yaml').read_text())
        target = new['vision']['targets']['middle_black_knob']
        self.assertEqual(target.pop('panel_layout_template'), 'panel_layout/template.json')
        self.assertEqual(target.pop('panel_layout_target_id'), 'middle_black_knob')
        self.assertEqual(target['locator'], 'panel_layout')
        target['locator'] = 'gray_plate'
        self.assertEqual(new, old)

    def test_template_out_of_bounds_rejected_early(self):
        config = json.loads(TEMPLATE.read_text())
        config['search_polygon'][0][0] = -10
        with self.assertRaisesRegex(ValueError, 'outside the reference image'):
            PanelLayoutDetector(self.image, config)


if __name__ == '__main__':
    unittest.main()
