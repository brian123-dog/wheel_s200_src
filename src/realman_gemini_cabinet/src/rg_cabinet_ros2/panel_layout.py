"""Optional panel-layout locator shared by ROS and the offline demo.

Only this module requires OpenCV/SIFT; legacy resolver paths import it lazily.
ACCEPTED_2D is an image-geometry decision, not calibrated robot confidence.
"""
from __future__ import annotations

import json
from pathlib import Path

import cv2
import numpy as np


def project(points, matrix):
    return cv2.perspectiveTransform(np.float32(points).reshape(1, -1, 2), matrix)[0]


def order_quad(points):
    points = np.float32(points).reshape(4, 2)
    center = points.mean(axis=0)
    points = points[np.argsort(np.arctan2(points[:, 1] - center[1], points[:, 0] - center[0]))]
    return np.roll(points, -np.argmin(points.sum(axis=1)), axis=0)


def polygon(image, points, color, thickness=2):
    cv2.polylines(image, [np.rint(points).astype(np.int32)], True, color, thickness, cv2.LINE_AA)


def plate_side_support(quad, evidence):
    """Check each proposed edge against unclosed segmentation, not filled-in gaps."""
    nearby = cv2.dilate(evidence, np.ones((5, 5), np.uint8))
    height, width = nearby.shape
    coverage, gaps = [], []
    for start, end in zip(quad, np.roll(quad, -1, axis=0)):
        count = max(2, int(np.ceil(np.linalg.norm(end - start))) + 1)
        samples = np.rint(np.linspace(start, end, count)).astype(int)
        valid = ((samples[:, 0] >= 0) & (samples[:, 0] < width)
                 & (samples[:, 1] >= 0) & (samples[:, 1] < height))
        supported = np.zeros(count, dtype=bool)
        supported[valid] = nearby[samples[valid, 1], samples[valid, 0]] > 0
        longest, run = 0, 0
        for present in supported:
            run = 0 if present else run + 1
            longest = max(longest, run)
        coverage.append(float(supported.mean()))
        gaps.append(longest / count)
    return coverage, gaps


class PanelLayoutDetector:
    @classmethod
    def from_template(cls, path):
        path = Path(path).expanduser().resolve()
        config = json.loads(path.read_text(encoding="utf-8"))
        reference_path = (path.parent / config["reference_image"]).resolve()
        reference = cv2.imread(str(reference_path))
        if reference is None:
            raise ValueError(f"Panel-layout reference image unreadable: {reference_path}")
        return cls(reference, config)

    def __init__(self, reference, config):
        if reference is None or reference.ndim != 3 or reference.shape[2] != 3:
            raise ValueError("Panel-layout reference must be a BGR image")
        height, width = reference.shape[:2]
        self.local_geometry = config.get('local_geometry', 'square_plate')
        if self.local_geometry not in ('square_plate', 'round_knob'):
            raise ValueError('Unsupported panel local_geometry')
        names = ['anchor_polygon', 'search_polygon']
        if self.local_geometry == 'square_plate':
            names.append('plate_corners')
        else:
            center = np.asarray(config['axis_center_reference_px'], dtype=float)
            radius = float(config['body_radius_reference_px'])
            if center.shape != (2,) or not np.isfinite(center).all() or not np.isfinite(radius) or radius <= 0:
                raise ValueError('Round knob needs a finite center and positive radius')
            threshold_scale = config.get('body_threshold_scale', 1.0)
            if (isinstance(threshold_scale, bool)
                    or not isinstance(threshold_scale, (int, float))
                    or not np.isfinite(threshold_scale)
                    or not .5 <= threshold_scale <= 1.0):
                raise ValueError('body_threshold_scale must be a finite number in [0.5, 1.0]')
            search = np.float32(config['search_polygon'])
            if cv2.pointPolygonTest(search, tuple(center), True) < 1.4 * radius:
                raise ValueError('Round body must fit inside its search region')
        for name in names:
            points = np.float32(config[name])
            if points.shape != (4, 2) or not np.isfinite(points).all():
                raise ValueError(f"{name} must contain four finite [x, y] points")
            if (points < 0).any() or (points[:, 0] >= width).any() or (points[:, 1] >= height).any():
                raise ValueError(f"{name} is outside the reference image")
            if not cv2.isContourConvex(points) or cv2.contourArea(points, oriented=True) <= 0:
                raise ValueError(f"{name} must be a convex TL/TR/BR/BL polygon")
        self.reference = reference
        self.config = config
        self.search = np.float32(config['search_polygon'])
        self.plate = order_quad(config['plate_corners']) if self.local_geometry == 'square_plate' else None
        self.sift = cv2.SIFT_create(nfeatures=3500)
        gray = cv2.cvtColor(reference, cv2.COLOR_BGR2GRAY)
        mask = np.zeros_like(gray)
        cv2.fillPoly(mask, [np.int32(config['anchor_polygon'])], 255)
        cv2.fillPoly(mask, [np.int32(self.search)], 0)
        exclusions = config.get('anchor_exclusion_polygons', [])
        excluded = np.zeros_like(gray)
        for points in exclusions:
            q = np.float32(points)
            if (q.shape != (4, 2) or not np.isfinite(q).all()
                    or not cv2.isContourConvex(q) or cv2.contourArea(q, oriented=True) <= 0
                    or (q < 0).any() or (q[:, 0] >= width).any() or (q[:, 1] >= height).any()):
                raise ValueError('Invalid reference anchor exclusion polygon')
            cv2.fillPoly(excluded, [np.rint(q).astype(np.int32)], 255)
        mask[excluded > 0] = 0
        self.keypoints, self.descriptors = self.sift.detectAndCompute(gray, mask)
        if exclusions and self.descriptors is not None:
            # Mask the descriptor footprint as well as its center. Otherwise
            # SIFT near a screenshot annotation can still encode its strokes.
            distance = cv2.distanceTransform(255 - excluded, cv2.DIST_L2, 5)
            keep = [i for i, kp in enumerate(self.keypoints)
                    if distance[int(round(kp.pt[1])), int(round(kp.pt[0]))] > 6 * kp.size]
            self.keypoints = [self.keypoints[i] for i in keep]
            self.descriptors = self.descriptors[keep] if keep else None
        if self.descriptors is None:
            raise ValueError('Reference has no usable anchor features')

    def register(self, image, features=None):
        self.last_match_data = None
        kps, des = (self.sift.detectAndCompute(cv2.cvtColor(image, cv2.COLOR_BGR2GRAY), None)
                    if features is None else features)
        quality = {'reference_keypoints': len(self.keypoints)}
        if des is None or len(des) < 2:
            return None, quality, 'Insufficient image features'
        pairs = cv2.BFMatcher().knnMatch(self.descriptors, des, k=2)
        matches = [pair[0] for pair in pairs if len(pair) == 2 and pair[0].distance < .70 * pair[1].distance]
        # A scene feature must not serve as several independent correspondences.
        unique = {}
        for m in sorted(matches, key=lambda m: m.distance):
            unique.setdefault(m.trainIdx, m)
        matches = list(unique.values())
        quality['matches'] = len(matches)
        self.last_match_data = (kps, matches, None)
        if len(matches) < 4:
            return None, quality, 'Insufficient unique matches'
        src = np.float32([self.keypoints[m.queryIdx].pt for m in matches])
        dst = np.float32([kps[m.trainIdx].pt for m in matches])
        cv2.setRNGSeed(0)
        H, mask = cv2.findHomography(src, dst, cv2.RANSAC, 3.0)
        if H is None or mask is None or not np.isfinite(H).all():
            return None, quality, 'Homography estimation failed'
        inside = mask.ravel().astype(bool)
        self.last_match_data = (kps, matches, inside)
        count = int(inside.sum())
        if count < 4:
            return None, quality, 'Too few homography inliers'
        errors = np.linalg.norm(project(src[inside], H) - dst[inside], axis=1)
        anchor = np.float32(self.config['anchor_polygon'])
        span = np.ptp(src[inside], axis=0) / np.maximum(np.ptp(anchor, axis=0), 1)
        quality.update(inliers=count, inlier_ratio=count / len(matches),
                       median_reprojection_px=float(np.median(errors)), anchor_span_fraction=span.tolist())
        if count < self.config['min_inliers'] or quality['inlier_ratio'] < self.config['min_inlier_ratio']:
            return None, quality, 'Weak global match: insufficient inlier support'
        if np.any(span < np.float32(self.config['min_anchor_span_fraction'])):
            return None, quality, 'Matches concentrated in too small an anchor region'
        if quality['median_reprojection_px'] > self.config['max_median_reprojection_px']:
            return None, quality, 'Reprojection error too large'
        # Reject singular transforms and quadrilaterals crossing the projective horizon.
        if abs(np.linalg.det(H)) < 1e-10:
            return None, quality, 'Singular homography'
        denom = np.c_[self.search, np.ones(4)] @ H[2]
        if np.any(np.abs(denom) < 1e-6) or np.any(denom * denom[0] <= 0):
            return None, quality, 'Invalid projective region'
        predicted = project(self.search, H)
        h, w = image.shape[:2]
        if not cv2.isContourConvex(predicted) or cv2.contourArea(predicted, oriented=True) <= 0:
            return None, quality, 'Invalid projected search polygon'
        if np.any(predicted < 2) or np.any(predicted[:, 0] >= w - 2) or np.any(predicted[:, 1] >= h - 2):
            return None, quality, 'Predicted target region clipped by image edge'
        return H, quality, None

    def refine(self, image, H):
        # Normalize the local search to the reference image, allowing translation,
        # rotation and scale changes. Global H is only a coarse prediction.
        rh, rw = self.reference.shape[:2]
        rectified = cv2.warpPerspective(image, np.linalg.inv(H), (rw, rh))
        x0, y0 = np.floor(self.search.min(axis=0)).astype(int)
        x1, y1 = np.ceil(self.search.max(axis=0)).astype(int)
        crop = rectified[y0:y1, x0:x1]
        gray = cv2.cvtColor(crop, cv2.COLOR_BGR2GRAY)
        threshold, mask = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY_INV | cv2.THRESH_OTSU)
        adaptive = cv2.adaptiveThreshold(
            gray, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY_INV, 21, 7
        )
        # Narrow dark borders can split at a few bright/noisy pixels. Close only
        # short gaps in the normalized local view; do not form a convex hull
        # across a missing side or broaden the global search region.
        masks = (
            ('otsu', mask),
            ('otsu_close5', cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((5, 5), np.uint8))),
            ('adaptive', adaptive),
            ('adaptive_close3', cv2.morphologyEx(adaptive, cv2.MORPH_CLOSE, np.ones((3, 3), np.uint8))),
        )
        evidence = cv2.bitwise_or(mask, adaptive)
        candidates = []
        expected_area = cv2.contourArea(self.plate)
        stats = {'plate_otsu_threshold': threshold, 'plate_contours': 0,
                 'plate_quads': 0, 'plate_geometry_rejections': 0, 'plate_edge_rejections': 0}
        for method, segmented in masks:
            contours, _ = cv2.findContours(segmented, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
            stats['plate_contours'] += len(contours)
            for contour in contours:
                q = cv2.approxPolyDP(contour, .025 * cv2.arcLength(contour, True), True)
                if len(q) != 4 or not cv2.isContourConvex(q):
                    continue
                stats['plate_quads'] += 1
                local_quad = order_quad(q)
                q = local_quad + [x0, y0]
                area_ratio = cv2.contourArea(np.float32(q)) / expected_area
                corner_rms = float(np.sqrt(np.mean(np.sum((q - self.plate) ** 2, axis=1))))
                sides = np.linalg.norm(q - np.roll(q, 1, axis=0), axis=1)
                if not .65 < area_ratio < 1.4 or corner_rms > 13 or max(sides) / max(min(sides), 1e-6) > 1.5:
                    stats['plate_geometry_rejections'] += 1
                    continue
                support, gaps = plate_side_support(local_quad, evidence)
                if min(support) < .8 or max(gaps) > .15:
                    stats['plate_edge_rejections'] += 1
                    continue
                candidates.append((corner_rms, q, area_ratio, method, support))
        stats['plate_candidates'] = len(candidates)
        if not candidates:
            return None, stats, 'No complete plate contour in predicted region'
        candidates.sort(key=lambda item: item[0])
        # Inner/outer contours of the same physical frame are not separate targets.
        best = candidates[0]
        for candidate in candidates[1:]:
            if np.linalg.norm(candidate[1].mean(0) - best[1].mean(0)) > 12:
                return None, stats, 'Ambiguous plate candidates'
        _, quad, area_ratio, method, support = best
        plate_in_image = project(quad, H)
        unit = np.float32([[0, 0], [159, 0], [159, 159], [0, 159]])
        to_plate = cv2.getPerspectiveTransform(np.float32(plate_in_image), unit)
        patch = cv2.warpPerspective(image, to_plate, (160, 160))
        gray = cv2.cvtColor(patch, cv2.COLOR_BGR2GRAY)
        _, dark = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY_INV | cv2.THRESH_OTSU)
        contours, hierarchy = cv2.findContours(dark, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        holes = []
        cap_shape_rejections = []
        if hierarchy is not None:
            for i, c in enumerate(contours):
                # This specific knob has a light cap at the rotation center,
                # enclosed by its dark hub. Do not substitute a blob centroid.
                if hierarchy[0, i, 3] < 0:
                    continue
                area = cv2.contourArea(c)
                if not 45 < area < 900:
                    continue
                moments = cv2.moments(c)
                if moments['m00'] <= 0:
                    continue
                center = np.array([moments['m10'], moments['m01']]) / moments['m00']
                if not (48 < center[0] < 112 and 48 < center[1] < 112):
                    continue
                perimeter = cv2.arcLength(c, True)
                circularity = 4 * np.pi * area / max(perimeter ** 2, 1)
                # The shaft cap is round in the rectified plate. Thin highlights
                # on the dark handle can also be enclosed light regions; the old
                # .45 cutoff admitted those as a second cap (real failure: .484).
                if circularity < .65:
                    cap_shape_rejections.append({
                        'center_normalized_px': center.tolist(),
                        'area_normalized_px': float(area),
                        'circularity': float(circularity),
                        'reason': 'insufficient_circularity',
                    })
                    continue
                radius = np.sqrt(area / np.pi)
                yy, xx = np.indices(dark.shape)
                distance = np.hypot(xx - center[0], yy - center[1])
                ring = (distance > radius * 1.5) & (distance < radius * 2.5)
                coverage = float(np.mean(dark[ring] > 0)) if ring.any() else 0
                if coverage < .8:
                    continue
                holes.append((center, area, circularity, coverage))
        metrics = {**stats, 'plate_corner_rms_reference_px': best[0],
                   'plate_area_ratio': area_ratio, 'plate_segmentation': method,
                   'plate_side_support': support, 'hub_cap_candidates': len(holes),
                   'hub_cap_min_circularity': .65,
                   'hub_cap_shape_rejections': cap_shape_rejections,
                   'hub_cap_candidate_details': [
                       {'center_normalized_px': center.tolist(),
                        'area_normalized_px': float(area),
                        'circularity': float(circularity),
                        'dark_ring_coverage': float(coverage)}
                       for center, area, circularity, coverage in holes]}
        if len(holes) != 1:
            return {'plate': plate_in_image}, metrics, 'Missing or ambiguous central hub cap'
        center, area, circularity, coverage = holes[0]
        center_image = project([center], np.linalg.inv(to_plate))[0]
        metrics.update(hub_cap_area_normalized_px=area, hub_cap_circularity=circularity,
                       hub_dark_ring_coverage=coverage)
        return {'plate': plate_in_image, 'center': center_image}, metrics, None

    def process(self, image, features=None):
        result = {'target_id': self.config['target_id'], 'status': 'REJECTED',
                  'prediction_polygon': None, 'plate_corners': None, 'axis_center_px': None}
        H, quality, error = self.register(image) if features is None else self.register(image, features)
        result['registration'] = quality
        if error:
            result['reason'] = error
            return result
        result['prediction_polygon'] = project(self.search, H).tolist()
        result['reference_to_image_h'] = H.tolist()
        if self.local_geometry == 'round_knob':
            from .round_knob import refine_round_knob
            refined, quality, error = refine_round_knob(
                image, self.reference.shape, self.search, self.config, H)
            result['local_geometry'] = 'round_knob'
            result['refinement'] = quality
            if error:
                result['reason'] = error
                return result
            result.update(status='ACCEPTED_2D', reason='Dark circular body supported by image',
                          axis_center_px=refined['center'].tolist(),
                          local_bbox_corners=refined['bbox'].tolist(),
                          knob_outline_px=refined['outline'].tolist())
            return result
        refined, quality, error = self.refine(image, H)
        result['refinement'] = quality
        if refined is not None:
            result['plate_corners'] = refined['plate'].tolist()
        if error:
            result['reason'] = error
            return result
        result.update(status='ACCEPTED_2D', reason='Plate and central hub cap supported by image',
                      axis_center_px=refined['center'].tolist())
        return result


def draw_panel_layout_result(image, result, labels=True):
    out = image.copy()
    if result['prediction_polygon'] is not None:
        polygon(out, result['prediction_polygon'], (0, 215, 255))
    if result['plate_corners'] is not None:
        polygon(out, result['plate_corners'], (0, 255, 0))
    if result.get('knob_outline_px') is not None:
        polygon(out, result['knob_outline_px'], (0, 255, 0))
    if result['axis_center_px'] is not None:
        p = tuple(np.rint(result['axis_center_px']).astype(int))
        cv2.drawMarker(out, p, (255, 0, 255), cv2.MARKER_CROSS, 17, 2)
        cv2.circle(out, p, 5, (255, 0, 255), 1, cv2.LINE_AA)
        if labels:
            cv2.putText(out, f'axis ({p[0]}, {p[1]})', (p[0]+14, p[1]-12), cv2.FONT_HERSHEY_SIMPLEX, .5, (255, 0, 255), 1, cv2.LINE_AA)
    if not labels:
        return out
    cv2.rectangle(out, (0, 0), (out.shape[1], 78), (25, 25, 25), -1)
    color = (100, 255, 100) if result['status'] == 'ACCEPTED_2D' else (80, 80, 255)
    cv2.putText(out, result['status']+' | '+result['file'], (12, 23), cv2.FONT_HERSHEY_SIMPLEX, .55, color, 1, cv2.LINE_AA)
    cv2.putText(out, 'YELLOW: predicted search | GREEN: measured base | MAGENTA: hub center estimate', (12, 47), cv2.FONT_HERSHEY_SIMPLEX, .52, (230, 230, 230), 1, cv2.LINE_AA)
    cv2.putText(out, result['reason'], (12, 69), cv2.FONT_HERSHEY_SIMPLEX, .48, color, 1, cv2.LINE_AA)
    return out
