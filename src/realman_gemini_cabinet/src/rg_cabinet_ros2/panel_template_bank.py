"""All templates keep their own strict checks; conflicting results are rejected."""
import math
import cv2
import numpy as np


def locate_with_templates(image, detectors, max_center_delta_px=5.0):
    if not detectors:
        raise ValueError('Panel template bank is empty')
    first = next(iter(detectors.values()))
    features = first.sift.detectAndCompute(cv2.cvtColor(image, cv2.COLOR_BGR2GRAY), None)
    results = [(key, detector.process(image, features=features)) for key, detector in detectors.items()]
    accepted = [(key, r) for key, r in results if r['status'] == 'ACCEPTED_2D']

    def score(item):
        r = item[1]; q = r.get('registration', {})
        return (r.get('plate_corners') is not None, r.get('prediction_polygon') is not None,
                q.get('inliers', 0), q.get('inlier_ratio', 0), -q.get('median_reprojection_px', math.inf))

    key, chosen = max(accepted or results, key=score)
    if accepted:
        for i, (_, a) in enumerate(accepted):
            for _, b in accepted[i+1:]:
                delta = np.linalg.norm(np.asarray(a['axis_center_px']) - np.asarray(b['axis_center_px']))
                if delta > max_center_delta_px:
                    chosen = {**chosen, 'status': 'REJECTED',
                              'reason': 'Conflicting template localizations',
                              'template_conflict_delta_px': float(delta)}
                    break
            if chosen['status'] == 'REJECTED':
                break
    summaries = [{'template': name, 'status': r['status'], 'reason': r['reason'],
                  'registration': r.get('registration', {}),
                  'axis_center_px': r.get('axis_center_px')} for name, r in results]
    return key, chosen, summaries
