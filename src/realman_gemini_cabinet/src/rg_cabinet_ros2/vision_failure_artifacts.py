"""Persist failed samples without making image I/O a task failure."""
from datetime import datetime
from pathlib import Path
import json
import re


def save_failure_artifacts(frame, debug, vision_cfg, name, detector=None):
    paths = {}
    directory = Path(str(vision_cfg.get('vision_debug_dir', 'vision_debug')))
    stem = re.sub(r'[^a-zA-Z0-9_-]', '_', name) + '_' + datetime.now().strftime('%Y%m%d_%H%M%S_%f') + '_failed'
    metadata = directory / (stem + '.json')
    try:
        directory.mkdir(parents=True, exist_ok=True)
        paths['metadata_path'] = str(metadata.resolve())
        if frame is not None and frame.color is not None:
            import cv2
            import numpy as np
            from rg_cabinet_ros2.image_tools import _image_to_rgb_array
            from rg_cabinet_ros2.panel_layout import draw_panel_layout_result
            bgr = cv2.cvtColor(_image_to_rgb_array(frame.color, np), cv2.COLOR_RGB2BGR)
            raw = directory / (stem + '_color.png')
            if not cv2.imwrite(str(raw), bgr):
                raise RuntimeError('Could not write raw failure image')
            paths['color_path'] = str(raw.resolve())
            result = debug.get('panel_layout')
            if result:
                overlay = draw_panel_layout_result(bgr, {**result, 'file': name})
                path = directory / (stem + '_overlay.png')
                if not cv2.imwrite(str(path), overlay):
                    raise RuntimeError('Could not write failure overlay')
                paths['overlay_path'] = str(path.resolve())
                match_data = getattr(detector, 'last_match_data', None)
                if match_data is not None:
                    keypoints, matches, inliers = match_data
                    match_image = cv2.drawMatches(detector.reference, detector.keypoints,
                        bgr, keypoints, matches, None, matchColor=(0, 180, 255),
                        flags=cv2.DrawMatchesFlags_NOT_DRAW_SINGLE_POINTS)
                    path = directory / (stem + '_matches.png')
                    if not cv2.imwrite(str(path), match_image):
                        raise RuntimeError('Could not write match evidence')
                    paths['matches_path'] = str(path.resolve())
                    debug = {**debug, 'match_evidence': [
                        {'reference_px': list(detector.keypoints[m.queryIdx].pt),
                         'image_px': list(keypoints[m.trainIdx].pt),
                         'inlier': bool(inliers[i]) if inliers is not None else None}
                        for i, m in enumerate(matches)]}
    except Exception as exc:
        paths['error'] = str(exc)
    try:
        metadata.write_text(json.dumps({**debug, 'debug_images': paths}, ensure_ascii=False, indent=2), encoding='utf-8')
    except Exception as exc:
        paths['metadata_error'] = str(exc)
    return paths
