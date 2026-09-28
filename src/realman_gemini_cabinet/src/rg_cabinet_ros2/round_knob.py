"""Local circular-base refinement; global registration only predicts the ROI.

Fit the dark circular body, not the rotating handle or its specular highlight.
Reject incomplete/ambiguous boundaries. This provides 2D evidence, not a
calibration of the physical shaft or a collision-free robot trajectory.
"""
import cv2
import numpy as np


def refine_round_knob(image, reference_shape, search, config, H):
    from .panel_layout import project
    center = np.asarray(config['axis_center_reference_px'], dtype=float)
    radius = float(config['body_radius_reference_px'])
    rh, rw = reference_shape[:2]
    rectified = cv2.warpPerspective(image, np.linalg.inv(H), (rw, rh))
    x0, y0 = np.floor(search.min(axis=0)).astype(int)
    x1, y1 = np.ceil(search.max(axis=0)).astype(int)
    crop = rectified[y0:y1, x0:x1]
    origin = np.array([x0, y0])
    expected = center - origin
    gray = cv2.cvtColor(crop, cv2.COLOR_BGR2GRAY)
    gray = cv2.GaussianBlur(gray, (3, 3), .6)
    # Restrict evidence to the dark body silhouette. Canny on the entire
    # crop also sees the offset silver bezel and its shadow; mixing those arcs
    # yields several high-scoring circles with different centers. External
    # contours exclude enclosed highlights; RANSAC still removes the handle.
    threshold, dark_mask = cv2.threshold(
        gray, 0, 255, cv2.THRESH_BINARY_INV | cv2.THRESH_OTSU)
    # A reference may use stricter dark-pixel segmentation when the gray
    # bezel shadow merges with the black body under Otsu. This changes the
    # extracted contour, never the circle evidence/ambiguity acceptance gates.
    # Existing references retain the original segmentation exactly.
    threshold_scale = float(config.get('body_threshold_scale', 1.0))
    segmentation_threshold = threshold * threshold_scale
    if threshold_scale != 1.0:
        _, dark_mask = cv2.threshold(gray, segmentation_threshold, 255, cv2.THRESH_BINARY_INV)
    contours, _ = cv2.findContours(dark_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    bodies = [c for c in contours
              if cv2.pointPolygonTest(c, tuple(expected), False) >= 0
              and .55 * np.pi * radius**2 <= cv2.contourArea(c) <= 1.8 * np.pi * radius**2]
    stats = {'geometry': 'round_knob', 'edge_source': 'dark_body_silhouette',
             'body_otsu_threshold': float(threshold),
             'body_segmentation_threshold': float(segmentation_threshold),
             'body_threshold_scale': threshold_scale,
             'body_components': len(bodies), 'round_candidates': 0}
    if len(bodies) != 1:
        return None, stats, 'Missing or ambiguous dark body in predicted region'
    points = bodies[0].reshape(-1, 2).astype(float)
    dist = np.linalg.norm(points - expected, axis=1)
    points = points[(dist > radius * .65) & (dist < radius * 1.35)]
    stats['edge_points'] = len(points)
    if len(points) < 24:
        return None, stats, 'Insufficient circular-body edge evidence'
    # Seeded RANSAC rejects the handle protrusion and highlight edges.
    rng = np.random.RandomState(0)
    hypotheses = [(expected, radius)]
    for _ in range(250):
        a, b, c = points[rng.choice(len(points), 3, replace=False)]
        A = 2 * np.array([b-a, c-a])
        if abs(np.linalg.det(A)) < 1:
            continue
        cc = np.linalg.solve(A, np.array([b@b-a@a, c@c-a@a]))
        rr = np.linalg.norm(a-cc)
        if np.linalg.norm(cc-expected) <= radius*.30 and .78*radius <= rr <= 1.22*radius:
            hypotheses.append((cc, rr))
    angles = np.linspace(0, 2*np.pi, 72, endpoint=False)
    unit = np.column_stack([np.cos(angles), np.sin(angles)])
    hsv = cv2.cvtColor(crop, cv2.COLOR_BGR2HSV)
    def sample(im, coords):
        coords = np.float32(coords)
        return cv2.remap(im, coords[:, 0].reshape(1,-1), coords[:, 1].reshape(1,-1),
                         cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT).ravel()
    candidates = []
    # A local memo only: identical inlier sets yield identical least-squares
    # fits. Preserve every hypothesis and all quality/ambiguity thresholds.
    fit_cache = {}
    circle_cache = {}
    for cc, rr in hypotheses:
        # Refine only supported radial edges; never use blob centroid.
        for _ in range(2):
            residual = np.abs(np.linalg.norm(points-cc, axis=1)-rr)
            inliers = residual < 1.25
            pts = points[inliers]
            if len(pts) < 20:
                break
            fit_key = inliers.tobytes()
            fitted = fit_cache.get(fit_key)
            if fitted is None:
                A = np.column_stack([2*pts, np.ones(len(pts))])
                fitted = np.linalg.lstsq(A, (pts*pts).sum(axis=1), rcond=None)[0]
                fit_cache[fit_key] = fitted
            cc = fitted[:2]
            rr = np.sqrt(max(0., fitted[2]+cc@cc))
        if np.linalg.norm(cc-expected) > radius*.30 or not .78*radius <= rr <= 1.22*radius:
            continue
        # Exact floating-point identity only, never merge nearby centers.
        circle_key = (cc.tobytes(), float(rr))
        if circle_key in circle_cache:
            if circle_cache[circle_key] is not None:
                candidates.append(circle_cache[circle_key])
            continue
        circle_cache[circle_key] = None
        boundary = cc + rr*unit
        nearest = np.min(np.linalg.norm(boundary[:,None,:]-points[None,:,:],axis=2),axis=1)
        supported = nearest < 1.8
        coverage = float(supported.mean())
        run = longest = 0
        for supported_angle in np.tile(supported, 2):
            run = 0 if supported_angle else run+1
            longest = max(longest, run)
        gap = min(longest, len(unit))/len(unit)
        inside = sample(gray, cc + .65*rr*unit)
        outside = sample(gray, cc + 1.30*rr*unit)
        contrast = outside.astype(float)-inside.astype(float)
        contrast_support = float((contrast > 22).mean())
        saturation = sample(hsv[:,:,1], cc + .65*rr*unit)
        dark_support = float(((inside < 135) & (saturation < 100)).mean())
        if coverage < .78 or gap > .20 or contrast_support < .72 or dark_support < .80:
            continue
        residual = float(np.median(nearest))
        score = coverage + contrast_support - .05*residual - .01*np.linalg.norm(cc-expected)
        candidate = (score, cc, rr, coverage, gap, contrast_support, dark_support)
        candidates.append(candidate)
        circle_cache[circle_key] = candidate
    stats['round_candidates'] = len(candidates)
    if not candidates:
        return None, stats, 'No complete dark circular body in predicted upper-row region'
    candidates.sort(key=lambda v:v[0], reverse=True)
    _, cc, rr, coverage, gap, contrast, dark = candidates[0]
    stats['best_circle_hypotheses'] = [dict(score=float(v[0]), center=(v[1]+origin).tolist(), radius=float(v[2])) for v in candidates[:5]]
    if any(np.linalg.norm(other[1]-cc) > radius*.20 for other in candidates[1:]
           if other[0] >= candidates[0][0] - .08):
        return None, stats, 'Ambiguous circular-body centers'
    center_ref = cc+origin
    outline = project(center_ref + rr*unit, H)
    image_center = project([center_ref], H)[0]
    apparent_radius = np.median(np.linalg.norm(outline-image_center,axis=1))
    stats.update(body_radius_reference_px=float(rr), boundary_coverage=coverage,
                 max_missing_arc_fraction=gap, contrast_support=contrast,
                 dark_support=dark, center_shift_reference_px=float(np.linalg.norm(cc-expected)),
                 apparent_radius_px=float(apparent_radius))
    if apparent_radius < 8:
        return None, stats, 'Circular body too small for reliable axis refinement'
    lo, hi = outline.min(axis=0), outline.max(axis=0)
    bbox = np.array([lo, [hi[0],lo[1]], hi, [lo[0],hi[1]]])
    return {'center':image_center, 'outline':outline, 'bbox':bbox}, stats, None
