from __future__ import annotations

import math
import struct
from dataclasses import dataclass
from pathlib import Path
from statistics import median
from typing import Any, Iterable

from rg_cabinet.geometry import Intrinsics


@dataclass(frozen=True)
class ColorBlob:
    u: float
    v: float
    pixel_count: int
    bbox: tuple[int, int, int, int]


@dataclass(frozen=True)
class SquareFrameDetection:
    u: float
    v: float
    pixel_count: int
    bbox: tuple[int, int, int, int]
    size_px: float
    aspect_ratio: float
    edge_score: float
    side_coverage: float
    fill_ratio: float
    side_coverages: tuple[float, float, float, float] = (0.0, 0.0, 0.0, 0.0)
    center_distance_px: float | None = None


@dataclass(frozen=True)
class SquareFrameCandidate:
    u: float
    v: float
    pixel_count: int
    bbox: tuple[int, int, int, int]
    size_px: float
    aspect_ratio: float
    edge_score: float
    side_coverage: float
    fill_ratio: float
    side_coverages: tuple[float, float, float, float]
    center_distance_px: float | None
    passed: bool
    error: str


@dataclass(frozen=True)
class SquareFrameSearchResult:
    detection: SquareFrameDetection | None
    candidates: list[SquareFrameCandidate]
    matching_pixels: set[tuple[int, int]]
    roi_bounds: tuple[int, int, int, int]
    error: str = ""


@dataclass(frozen=True)
class GrayPlateDetection:
    u: float
    v: float
    bbox: tuple[int, int, int, int]
    corners: tuple[tuple[float, float], tuple[float, float], tuple[float, float], tuple[float, float]]
    width_px: float
    height_px: float
    angle_deg: float
    area_px: float
    score: float
    rectangularity: float
    mask_coverage: float
    aspect_ratio: float
    center_distance_px: float | None = None


@dataclass(frozen=True)
class GrayPlateCandidate:
    u: float
    v: float
    bbox: tuple[int, int, int, int]
    corners: tuple[tuple[float, float], tuple[float, float], tuple[float, float], tuple[float, float]]
    width_px: float
    height_px: float
    angle_deg: float
    area_px: float
    score: float
    rectangularity: float
    mask_coverage: float
    aspect_ratio: float
    center_distance_px: float | None
    passed: bool
    error: str


@dataclass(frozen=True)
class GrayPlateSearchResult:
    detection: GrayPlateDetection | None
    candidates: list[GrayPlateCandidate]
    roi_bounds: tuple[int, int, int, int]
    mask: Any
    edges: Any
    error: str = ""


def camera_info_to_intrinsics(camera_info: Any, depth_scale: float = 1.0) -> Intrinsics:
    k = camera_info.k
    return Intrinsics(
        fx=float(k[0]),
        fy=float(k[4]),
        cx=float(k[2]),
        cy=float(k[5]),
        depth_scale=depth_scale,
    )


def find_color_blob(
    image: Any,
    rgb_min: Iterable[int],
    rgb_max: Iterable[int],
    min_pixels: int = 20,
    roi: Iterable[int] | None = None,
    stride: int = 1,
) -> ColorBlob:
    blobs = find_color_blobs(
        image,
        rgb_min=rgb_min,
        rgb_max=rgb_max,
        min_pixels=min_pixels,
        roi=roi,
        stride=stride,
    )
    return blobs[0]


def find_color_blobs(
    image: Any,
    rgb_min: Iterable[int],
    rgb_max: Iterable[int],
    min_pixels: int = 20,
    roi: Iterable[int] | None = None,
    stride: int = 1,
) -> list[ColorBlob]:
    lower = tuple(int(v) for v in rgb_min)
    upper = tuple(int(v) for v in rgb_max)
    if len(lower) != 3 or len(upper) != 3:
        raise ValueError("rgb_min and rgb_max must contain three values")
    if stride <= 0:
        raise ValueError("stride must be positive")

    matching = _matching_rgb_pixels(image, lower, upper, roi, stride)
    blobs = [_blob_from_pixels(component) for component in _connected_pixel_components(matching, stride)]
    valid = [blob for blob in blobs if blob.pixel_count >= min_pixels]
    if not valid:
        largest = blobs[0].pixel_count if blobs else 0
        raise ValueError(
            f"Color blob has {largest} pixels, fewer than {min_pixels}"
        )
    return sorted(valid, key=lambda blob: blob.pixel_count, reverse=True)


def find_square_frame(
    image: Any,
    rgb_min: Iterable[int],
    rgb_max: Iterable[int],
    min_pixels: int = 20,
    roi: Iterable[int] | None = None,
    stride: int = 1,
    min_size_px: float = 12.0,
    max_size_px: float | None = None,
    aspect_tolerance: float = 0.25,
    edge_tolerance_px: int = 2,
    min_edge_score: float = 0.55,
    min_side_coverage: float = 0.45,
    max_fill_ratio: float = 0.70,
    desired_pixel: Iterable[float] | None = None,
    scan_step_px: int = 1,
) -> SquareFrameDetection:
    result = find_square_frame_with_debug(
        image,
        rgb_min=rgb_min,
        rgb_max=rgb_max,
        min_pixels=min_pixels,
        roi=roi,
        stride=stride,
        min_size_px=min_size_px,
        max_size_px=max_size_px,
        aspect_tolerance=aspect_tolerance,
        edge_tolerance_px=edge_tolerance_px,
        min_edge_score=min_edge_score,
        min_side_coverage=min_side_coverage,
        max_fill_ratio=max_fill_ratio,
        desired_pixel=desired_pixel,
        scan_step_px=scan_step_px,
    )
    if result.detection is None:
        raise ValueError(result.error or "No square frame candidate passed validation")
    return result.detection


def find_square_frame_with_debug(
    image: Any,
    rgb_min: Iterable[int],
    rgb_max: Iterable[int],
    min_pixels: int = 20,
    roi: Iterable[int] | None = None,
    stride: int = 1,
    min_size_px: float = 12.0,
    max_size_px: float | None = None,
    aspect_tolerance: float = 0.25,
    edge_tolerance_px: int = 2,
    min_edge_score: float = 0.55,
    min_side_coverage: float = 0.45,
    max_fill_ratio: float = 0.70,
    desired_pixel: Iterable[float] | None = None,
    scan_step_px: int = 1,
) -> SquareFrameSearchResult:
    lower = tuple(int(v) for v in rgb_min)
    upper = tuple(int(v) for v in rgb_max)
    if len(lower) != 3 or len(upper) != 3:
        raise ValueError("rgb_min and rgb_max must contain three values")
    if stride <= 0:
        raise ValueError("stride must be positive")
    if scan_step_px <= 0:
        raise ValueError("scan_step_px must be positive")

    roi_bounds = _roi_bounds(image.width, image.height, roi)
    matching = _matching_rgb_pixels(image, lower, upper, roi, stride)
    window_candidates = _score_square_frame_windows(
        matching,
        roi_bounds=roi_bounds,
        min_pixels=min_pixels,
        min_size_px=min_size_px,
        max_size_px=max_size_px,
        aspect_tolerance=aspect_tolerance,
        edge_tolerance_px=edge_tolerance_px,
        min_edge_score=min_edge_score,
        min_side_coverage=min_side_coverage,
        max_fill_ratio=max_fill_ratio,
        desired_pixel=desired_pixel,
        scan_step_px=scan_step_px,
    )
    passed = [candidate for candidate in window_candidates if candidate.passed]
    if passed:
        selected = _select_square_frame_candidate(passed, desired_pixel)
        return SquareFrameSearchResult(
            detection=_candidate_to_detection(selected),
            candidates=window_candidates,
            matching_pixels=matching,
            roi_bounds=roi_bounds,
        )

    components = _connected_pixel_components(matching, stride)
    candidates: list[SquareFrameDetection] = []
    errors: list[str] = []
    for idx, component in enumerate(components):
        try:
            detection = _square_frame_from_component(
                component,
                min_pixels=min_pixels,
                min_size_px=min_size_px,
                max_size_px=max_size_px,
                aspect_tolerance=aspect_tolerance,
                edge_tolerance_px=edge_tolerance_px,
                min_edge_score=min_edge_score,
                min_side_coverage=min_side_coverage,
                max_fill_ratio=max_fill_ratio,
            )
            candidates.append(detection)
        except Exception as exc:
            blob = _blob_from_pixels(component)
            errors.append(
                f"candidate {idx}: pixels={blob.pixel_count}, "
                f"bbox={list(blob.bbox)}, error={exc}"
            )

    if candidates:
        component_candidates = [
            _detection_to_candidate(candidate, desired_pixel, passed=True, error="")
            for candidate in candidates
        ]
        selected_component = _select_square_frame_candidate(component_candidates, desired_pixel)
        merged_candidates = window_candidates + component_candidates
        return SquareFrameSearchResult(
            detection=_candidate_to_detection(selected_component),
            candidates=merged_candidates,
            matching_pixels=matching,
            roi_bounds=roi_bounds,
        )

    top_windows = [
        _format_square_frame_candidate(candidate, idx)
        for idx, candidate in enumerate(_top_square_frame_candidates(window_candidates, limit=5))
    ]
    if top_windows:
        error = (
            "No square frame candidate passed validation; "
            + "; ".join(top_windows)
            + ("; connected components: " + "; ".join(errors[:5]) if errors else "")
        )
    else:
        error = "No square frame candidate passed validation"
        if errors:
            error += "; " + "; ".join(errors[:5])
    return SquareFrameSearchResult(
        detection=None,
        candidates=window_candidates,
        matching_pixels=matching,
        roi_bounds=roi_bounds,
        error=error,
    )


def find_gray_plate(
    image: Any,
    roi: Iterable[int] | None = None,
    desired_pixel: Iterable[float] | None = None,
    saturation_max: int = 80,
    value_min: int = 70,
    value_max: int = 235,
    min_area_px: float = 700.0,
    max_area_px: float | None = 6000.0,
    min_width_px: float = 25.0,
    max_width_px: float | None = 100.0,
    min_height_px: float = 25.0,
    max_height_px: float | None = 100.0,
    min_aspect_ratio: float = 0.55,
    max_aspect_ratio: float = 1.80,
    close_kernel_px: int = 9,
    close_iterations: int = 2,
    min_rectangularity: float = 0.55,
    min_mask_coverage: float = 0.45,
) -> GrayPlateDetection:
    result = find_gray_plate_with_debug(
        image,
        roi=roi,
        desired_pixel=desired_pixel,
        saturation_max=saturation_max,
        value_min=value_min,
        value_max=value_max,
        min_area_px=min_area_px,
        max_area_px=max_area_px,
        min_width_px=min_width_px,
        max_width_px=max_width_px,
        min_height_px=min_height_px,
        max_height_px=max_height_px,
        min_aspect_ratio=min_aspect_ratio,
        max_aspect_ratio=max_aspect_ratio,
        close_kernel_px=close_kernel_px,
        close_iterations=close_iterations,
        min_rectangularity=min_rectangularity,
        min_mask_coverage=min_mask_coverage,
    )
    if result.detection is None:
        raise ValueError(result.error or "No gray plate candidate passed validation")
    return result.detection


def find_gray_plate_with_debug(
    image: Any,
    roi: Iterable[int] | None = None,
    desired_pixel: Iterable[float] | None = None,
    saturation_max: int = 80,
    value_min: int = 70,
    value_max: int = 235,
    min_area_px: float = 700.0,
    max_area_px: float | None = 6000.0,
    min_width_px: float = 25.0,
    max_width_px: float | None = 100.0,
    min_height_px: float = 25.0,
    max_height_px: float | None = 100.0,
    min_aspect_ratio: float = 0.55,
    max_aspect_ratio: float = 1.80,
    close_kernel_px: int = 9,
    close_iterations: int = 2,
    min_rectangularity: float = 0.55,
    min_mask_coverage: float = 0.45,
) -> GrayPlateSearchResult:
    cv2, np = _require_cv2()
    rgb = _image_to_rgb_array(image, np)
    roi_bounds = _roi_bounds(image.width, image.height, roi)
    x0, y0, x1, y1 = roi_bounds
    roi_rgb = rgb[y0:y1, x0:x1]
    if roi_rgb.size == 0:
        return GrayPlateSearchResult(
            detection=None,
            candidates=[],
            roi_bounds=roi_bounds,
            mask=np.zeros((int(image.height), int(image.width)), dtype=np.uint8),
            edges=np.zeros((int(image.height), int(image.width)), dtype=np.uint8),
            error="Gray plate ROI is empty",
        )

    hsv = cv2.cvtColor(roi_rgb, cv2.COLOR_RGB2HSV)
    sat = hsv[:, :, 1]
    val = hsv[:, :, 2]
    roi_mask = (
        (sat <= int(saturation_max))
        & (val >= int(value_min))
        & (val <= int(value_max))
    ).astype(np.uint8) * 255

    kernel_size = max(1, int(close_kernel_px))
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (kernel_size, kernel_size))
    roi_mask = cv2.morphologyEx(
        roi_mask,
        cv2.MORPH_CLOSE,
        kernel,
        iterations=max(0, int(close_iterations)),
    )
    edges_roi = cv2.Canny(roi_mask, 50, 150)
    contours, _ = cv2.findContours(roi_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    desired = _pair2(desired_pixel, "desired_pixel") if desired_pixel is not None else None
    candidates = [
        _gray_plate_candidate_from_contour(
            contour,
            roi_mask=roi_mask,
            roi_offset=(x0, y0),
            desired=desired,
            min_area_px=min_area_px,
            max_area_px=max_area_px,
            min_width_px=min_width_px,
            max_width_px=max_width_px,
            min_height_px=min_height_px,
            max_height_px=max_height_px,
            min_aspect_ratio=min_aspect_ratio,
            max_aspect_ratio=max_aspect_ratio,
            min_rectangularity=min_rectangularity,
            min_mask_coverage=min_mask_coverage,
            roi_diag_px=math.hypot(max(1, x1 - x0), max(1, y1 - y0)),
        )
        for contour in contours
    ]
    candidates = sorted(candidates, key=_gray_plate_candidate_rank, reverse=True)
    passed = [candidate for candidate in candidates if candidate.passed]

    full_mask = np.zeros((int(image.height), int(image.width)), dtype=np.uint8)
    full_edges = np.zeros((int(image.height), int(image.width)), dtype=np.uint8)
    full_mask[y0:y1, x0:x1] = roi_mask
    full_edges[y0:y1, x0:x1] = edges_roi

    if passed:
        selected = passed[0]
        return GrayPlateSearchResult(
            detection=_gray_plate_candidate_to_detection(selected),
            candidates=candidates,
            roi_bounds=roi_bounds,
            mask=full_mask,
            edges=full_edges,
        )

    error = "No gray plate candidate passed validation"
    if candidates:
        error += "; " + "; ".join(
            _format_gray_plate_candidate(candidate, idx)
            for idx, candidate in enumerate(candidates[:5])
        )
    return GrayPlateSearchResult(
        detection=None,
        candidates=candidates,
        roi_bounds=roi_bounds,
        mask=full_mask,
        edges=full_edges,
        error=error,
    )


def read_rgb(image: Any, u: int, v: int) -> tuple[int, int, int]:
    encoding = image.encoding.lower()
    offset = int(v) * int(image.step) + int(u) * _color_bytes_per_pixel(encoding)
    data = image.data

    if encoding in ("rgb8", "8uc3"):
        return int(data[offset]), int(data[offset + 1]), int(data[offset + 2])
    if encoding == "bgr8":
        return int(data[offset + 2]), int(data[offset + 1]), int(data[offset])
    if encoding == "rgba8":
        return int(data[offset]), int(data[offset + 1]), int(data[offset + 2])
    if encoding == "bgra8":
        return int(data[offset + 2]), int(data[offset + 1]), int(data[offset])
    if encoding in ("mono8", "8uc1"):
        value = int(data[offset])
        return value, value, value
    raise ValueError(f"Unsupported color image encoding: {image.encoding}")


def color_image_extension(image: Any) -> str:
    encoding = image.encoding.lower()
    if encoding in ("rgb8", "bgr8"):
        return ".ppm"
    if encoding == "mono8":
        return ".pgm"
    raise ValueError(f"Unsupported meter image encoding: {image.encoding}")


def write_color_image(image: Any, path: str | Path) -> dict[str, Any]:
    encoding = image.encoding.lower()
    width = int(image.width)
    height = int(image.height)
    step = int(image.step)
    if width <= 0 or height <= 0:
        raise ValueError("Color image has invalid dimensions")

    output_path = Path(path)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    if encoding == "rgb8":
        _write_rgb8_ppm(image.data, output_path, width, height, step)
        image_format = "ppm"
    elif encoding == "bgr8":
        _write_bgr8_ppm(image.data, output_path, width, height, step)
        image_format = "ppm"
    elif encoding == "mono8":
        _write_mono8_pgm(image.data, output_path, width, height, step)
        image_format = "pgm"
    else:
        raise ValueError(f"Unsupported meter image encoding: {image.encoding}")

    return {
        "path": str(output_path),
        "width": width,
        "height": height,
        "encoding": image.encoding,
        "format": image_format,
    }


def write_square_frame_debug_images(
    image: Any,
    path_prefix: str | Path,
    matching_pixels: set[tuple[int, int]],
    roi_bounds: tuple[int, int, int, int],
    selected_frame: SquareFrameDetection | None = None,
    candidates: list[SquareFrameCandidate] | None = None,
    desired_pixel: Iterable[float] | None = None,
    blob_center: Iterable[float] | None = None,
) -> dict[str, Any]:
    prefix = Path(path_prefix)
    prefix.parent.mkdir(parents=True, exist_ok=True)

    color_path = prefix.with_name(prefix.name + "_color.ppm")
    mask_path = prefix.with_name(prefix.name + "_mask.pgm")
    overlay_path = prefix.with_name(prefix.name + "_overlay.ppm")

    write_color_image(image, color_path)
    _write_mask_pgm(matching_pixels, mask_path, int(image.width), int(image.height))
    _write_square_frame_overlay_ppm(
        image,
        overlay_path,
        roi_bounds=roi_bounds,
        selected_frame=selected_frame,
        candidates=candidates or [],
        desired_pixel=desired_pixel,
        blob_center=blob_center,
    )
    return {
        "color_path": str(color_path),
        "mask_path": str(mask_path),
        "overlay_path": str(overlay_path),
    }


def write_gray_plate_debug_images(
    image: Any,
    path_prefix: str | Path,
    mask: Any,
    edges: Any,
    roi_bounds: tuple[int, int, int, int],
    selected_plate: GrayPlateDetection | None = None,
    candidates: list[GrayPlateCandidate] | None = None,
    desired_pixel: Iterable[float] | None = None,
    alignment_reference_pixel: Iterable[float] | None = None,
    blob_center: Iterable[float] | None = None,
) -> dict[str, Any]:
    cv2, np = _require_cv2()
    prefix = Path(path_prefix)
    prefix.parent.mkdir(parents=True, exist_ok=True)

    color_path = prefix.with_name(prefix.name + "_color.ppm")
    gray_mask_path = prefix.with_name(prefix.name + "_gray_mask.pgm")
    edges_path = prefix.with_name(prefix.name + "_edges.pgm")
    overlay_path = prefix.with_name(prefix.name + "_overlay.ppm")

    write_color_image(image, color_path)
    cv2.imwrite(str(gray_mask_path), np.asarray(mask, dtype=np.uint8))
    cv2.imwrite(str(edges_path), np.asarray(edges, dtype=np.uint8))
    _write_gray_plate_overlay_ppm(
        image,
        overlay_path,
        roi_bounds=roi_bounds,
        selected_plate=selected_plate,
        candidates=candidates or [],
        desired_pixel=desired_pixel,
        alignment_reference_pixel=alignment_reference_pixel,
        blob_center=blob_center,
    )
    return {
        "color_path": str(color_path),
        "gray_mask_path": str(gray_mask_path),
        "edges_path": str(edges_path),
        "overlay_path": str(overlay_path),
    }


def median_depth_meters(
    depth_image: Any,
    color_u: float,
    color_v: float,
    color_width: int,
    color_height: int,
    window_px: int = 5,
    depth_scale_m: float | None = None,
) -> float:
    if depth_image.width <= 0 or depth_image.height <= 0:
        raise ValueError("Depth image has invalid dimensions")

    scale_u = depth_image.width / max(float(color_width), 1.0)
    scale_v = depth_image.height / max(float(color_height), 1.0)
    center_u = int(round(float(color_u) * scale_u))
    center_v = int(round(float(color_v) * scale_v))
    half = max(0, int(window_px) // 2)
    scale = _depth_scale(depth_image.encoding, depth_scale_m)
    values: list[float] = []

    for v in range(max(0, center_v - half), min(depth_image.height, center_v + half + 1)):
        for u in range(max(0, center_u - half), min(depth_image.width, center_u + half + 1)):
            raw = read_depth_raw(depth_image, u, v)
            if raw is None:
                continue
            depth_m = raw * scale
            if math.isfinite(depth_m) and depth_m > 0.0:
                values.append(depth_m)

    if not values:
        raise ValueError("No positive depth values around detected target")
    return float(median(values))


def median_bbox_ring_depth_meters(
    depth_image: Any,
    color_bbox: Iterable[int],
    color_width: int,
    color_height: int,
    margin_px: int = 30,
    exclude_padding_px: int = 4,
    min_samples: int = 10,
    depth_scale_m: float | None = None,
) -> float:
    if depth_image.width <= 0 or depth_image.height <= 0:
        raise ValueError("Depth image has invalid dimensions")
    bbox = [int(value) for value in color_bbox]
    if len(bbox) != 4:
        raise ValueError("color_bbox must be [min_u, min_v, max_u, max_v]")

    color_min_u, color_min_v, color_max_u, color_max_v = bbox
    if color_min_u > color_max_u or color_min_v > color_max_v:
        raise ValueError("color_bbox min values must be <= max values")

    scale_u = depth_image.width / max(float(color_width), 1.0)
    scale_v = depth_image.height / max(float(color_height), 1.0)
    margin_u = max(0, int(round(float(margin_px) * scale_u)))
    margin_v = max(0, int(round(float(margin_px) * scale_v)))
    padding_u = max(0, int(round(float(exclude_padding_px) * scale_u)))
    padding_v = max(0, int(round(float(exclude_padding_px) * scale_v)))

    min_u = int(math.floor(color_min_u * scale_u))
    min_v = int(math.floor(color_min_v * scale_v))
    max_u = int(math.ceil(color_max_u * scale_u))
    max_v = int(math.ceil(color_max_v * scale_v))
    outer_min_u = max(0, min_u - margin_u)
    outer_min_v = max(0, min_v - margin_v)
    outer_max_u = min(int(depth_image.width) - 1, max_u + margin_u)
    outer_max_v = min(int(depth_image.height) - 1, max_v + margin_v)
    inner_min_u = max(0, min_u - padding_u)
    inner_min_v = max(0, min_v - padding_v)
    inner_max_u = min(int(depth_image.width) - 1, max_u + padding_u)
    inner_max_v = min(int(depth_image.height) - 1, max_v + padding_v)

    scale = _depth_scale(depth_image.encoding, depth_scale_m)
    values: list[float] = []
    for v in range(outer_min_v, outer_max_v + 1):
        for u in range(outer_min_u, outer_max_u + 1):
            if inner_min_u <= u <= inner_max_u and inner_min_v <= v <= inner_max_v:
                continue
            raw = read_depth_raw(depth_image, u, v)
            if raw is None:
                continue
            depth_m = raw * scale
            if math.isfinite(depth_m) and depth_m > 0.0:
                values.append(depth_m)

    required_samples = max(1, int(min_samples))
    if len(values) < required_samples:
        raise ValueError(
            f"Only {len(values)} positive fallback depth samples around detected bbox, "
            f"fewer than {required_samples}"
        )
    return float(median(values))


def _write_rgb8_ppm(data: Any, path: Path, width: int, height: int, step: int) -> None:
    row_bytes = width * 3
    with path.open("wb") as handle:
        handle.write(f"P6\n{width} {height}\n255\n".encode("ascii"))
        for y in range(height):
            offset = y * step
            handle.write(bytes(data[offset : offset + row_bytes]))


def _write_bgr8_ppm(data: Any, path: Path, width: int, height: int, step: int) -> None:
    with path.open("wb") as handle:
        handle.write(f"P6\n{width} {height}\n255\n".encode("ascii"))
        for y in range(height):
            row_offset = y * step
            row = bytearray()
            for x in range(width):
                offset = row_offset + x * 3
                row.extend((data[offset + 2], data[offset + 1], data[offset]))
            handle.write(row)


def _write_mono8_pgm(data: Any, path: Path, width: int, height: int, step: int) -> None:
    with path.open("wb") as handle:
        handle.write(f"P5\n{width} {height}\n255\n".encode("ascii"))
        for y in range(height):
            offset = y * step
            handle.write(bytes(data[offset : offset + width]))


def _write_mask_pgm(
    matching_pixels: set[tuple[int, int]],
    path: Path,
    width: int,
    height: int,
) -> None:
    with path.open("wb") as handle:
        handle.write(f"P5\n{width} {height}\n255\n".encode("ascii"))
        for v in range(height):
            row = bytearray()
            for u in range(width):
                row.append(255 if (u, v) in matching_pixels else 0)
            handle.write(row)


def _write_square_frame_overlay_ppm(
    image: Any,
    path: Path,
    roi_bounds: tuple[int, int, int, int],
    selected_frame: SquareFrameDetection | None,
    candidates: list[SquareFrameCandidate],
    desired_pixel: Iterable[float] | None,
    blob_center: Iterable[float] | None,
) -> None:
    width = int(image.width)
    height = int(image.height)
    pixels = [
        [list(read_rgb(image, u, v)) for u in range(width)]
        for v in range(height)
    ]

    x0, y0, x1, y1 = roi_bounds
    _draw_rect(pixels, (x0, y0, max(x0, x1 - 1), max(y0, y1 - 1)), (255, 220, 0))
    for candidate in _top_square_frame_candidates(candidates, limit=5):
        color = (0, 180, 255) if candidate.passed else (255, 80, 80)
        _draw_rect(pixels, candidate.bbox, color)
    if selected_frame is not None:
        _draw_rect(pixels, selected_frame.bbox, (0, 255, 0))
        _draw_cross(pixels, selected_frame.u, selected_frame.v, (0, 255, 0), radius=5)
    if desired_pixel is not None:
        desired_u, desired_v = _pair2(desired_pixel, "desired_pixel")
        _draw_cross(pixels, desired_u, desired_v, (40, 100, 255), radius=6)
    if blob_center is not None:
        blob_u, blob_v = _pair2(blob_center, "blob_center")
        _draw_cross(pixels, blob_u, blob_v, (255, 0, 255), radius=5)

    with path.open("wb") as handle:
        handle.write(f"P6\n{width} {height}\n255\n".encode("ascii"))
        for row in pixels:
            handle.write(bytes(channel for pixel in row for channel in pixel))


def _write_gray_plate_overlay_ppm(
    image: Any,
    path: Path,
    roi_bounds: tuple[int, int, int, int],
    selected_plate: GrayPlateDetection | None,
    candidates: list[GrayPlateCandidate],
    desired_pixel: Iterable[float] | None,
    alignment_reference_pixel: Iterable[float] | None,
    blob_center: Iterable[float] | None,
) -> None:
    width = int(image.width)
    height = int(image.height)
    pixels = [
        [list(read_rgb(image, u, v)) for u in range(width)]
        for v in range(height)
    ]

    x0, y0, x1, y1 = roi_bounds
    _draw_rect(pixels, (x0, y0, max(x0, x1 - 1), max(y0, y1 - 1)), (255, 220, 0))
    for candidate in candidates[:5]:
        color = (0, 180, 255) if candidate.passed else (255, 80, 80)
        _draw_polygon(pixels, candidate.corners, color)
    if selected_plate is not None:
        _draw_polygon(pixels, selected_plate.corners, (0, 255, 0))
        _draw_cross(pixels, selected_plate.u, selected_plate.v, (0, 255, 0), radius=5)
    if desired_pixel is not None:
        desired_u, desired_v = _pair2(desired_pixel, "desired_pixel")
        _draw_cross(pixels, desired_u, desired_v, (40, 100, 255), radius=6)
    if alignment_reference_pixel is not None:
        ref_u, ref_v = _pair2(alignment_reference_pixel, "alignment_reference_pixel")
        _draw_cross(pixels, ref_u, ref_v, (0, 255, 255), radius=7)
    if blob_center is not None:
        blob_u, blob_v = _pair2(blob_center, "blob_center")
        _draw_cross(pixels, blob_u, blob_v, (255, 0, 255), radius=5)

    with path.open("wb") as handle:
        handle.write(f"P6\n{width} {height}\n255\n".encode("ascii"))
        for row in pixels:
            handle.write(bytes(channel for pixel in row for channel in pixel))


def _draw_rect(
    pixels: list[list[list[int]]],
    bbox: tuple[int, int, int, int],
    color: tuple[int, int, int],
) -> None:
    height = len(pixels)
    width = len(pixels[0]) if height else 0
    min_u, min_v, max_u, max_v = bbox
    min_u = max(0, min(width - 1, int(round(min_u))))
    max_u = max(0, min(width - 1, int(round(max_u))))
    min_v = max(0, min(height - 1, int(round(min_v))))
    max_v = max(0, min(height - 1, int(round(max_v))))
    if min_u > max_u or min_v > max_v:
        return
    for u in range(min_u, max_u + 1):
        pixels[min_v][u] = list(color)
        pixels[max_v][u] = list(color)
    for v in range(min_v, max_v + 1):
        pixels[v][min_u] = list(color)
        pixels[v][max_u] = list(color)


def _draw_polygon(
    pixels: list[list[list[int]]],
    points: Iterable[Iterable[float]],
    color: tuple[int, int, int],
) -> None:
    pts = [(float(point[0]), float(point[1])) for point in points]
    if len(pts) < 2:
        return
    for idx, start in enumerate(pts):
        end = pts[(idx + 1) % len(pts)]
        _draw_line(pixels, start[0], start[1], end[0], end[1], color)


def _draw_line(
    pixels: list[list[list[int]]],
    u0: float,
    v0: float,
    u1: float,
    v1: float,
    color: tuple[int, int, int],
) -> None:
    height = len(pixels)
    width = len(pixels[0]) if height else 0
    steps = max(1, int(round(max(abs(u1 - u0), abs(v1 - v0)))))
    for idx in range(steps + 1):
        alpha = idx / float(steps)
        u = int(round(u0 + (u1 - u0) * alpha))
        v = int(round(v0 + (v1 - v0) * alpha))
        if 0 <= u < width and 0 <= v < height:
            pixels[v][u] = list(color)


def _draw_cross(
    pixels: list[list[list[int]]],
    u: float,
    v: float,
    color: tuple[int, int, int],
    radius: int,
) -> None:
    height = len(pixels)
    width = len(pixels[0]) if height else 0
    center_u = int(round(float(u)))
    center_v = int(round(float(v)))
    for du in range(-radius, radius + 1):
        px = center_u + du
        if 0 <= px < width and 0 <= center_v < height:
            pixels[center_v][px] = list(color)
    for dv in range(-radius, radius + 1):
        py = center_v + dv
        if 0 <= center_u < width and 0 <= py < height:
            pixels[py][center_u] = list(color)


def read_depth_raw(depth_image: Any, u: int, v: int) -> float | None:
    encoding = depth_image.encoding.lower()
    offset = int(v) * int(depth_image.step) + int(u) * _depth_bytes_per_pixel(encoding)
    data = depth_image.data
    endian = ">" if getattr(depth_image, "is_bigendian", 0) else "<"

    if encoding in ("16uc1", "mono16"):
        return float(int.from_bytes(data[offset : offset + 2], endian == ">" and "big" or "little"))
    if encoding == "32fc1":
        return float(struct.unpack_from(f"{endian}f", data, offset)[0])
    if encoding == "32sc1":
        return float(struct.unpack_from(f"{endian}i", data, offset)[0])
    raise ValueError(f"Unsupported depth image encoding: {depth_image.encoding}")


def _color_bytes_per_pixel(encoding: str) -> int:
    if encoding in ("rgb8", "bgr8", "8uc3"):
        return 3
    if encoding in ("rgba8", "bgra8"):
        return 4
    if encoding in ("mono8", "8uc1"):
        return 1
    raise ValueError(f"Unsupported color image encoding: {encoding}")


def _depth_bytes_per_pixel(encoding: str) -> int:
    if encoding in ("16uc1", "mono16"):
        return 2
    if encoding in ("32fc1", "32sc1"):
        return 4
    raise ValueError(f"Unsupported depth image encoding: {encoding}")


def _depth_scale(encoding: str, explicit_scale: float | None) -> float:
    if explicit_scale is not None:
        return float(explicit_scale)
    if encoding.lower() in ("16uc1", "mono16", "32sc1"):
        return 0.001
    return 1.0


def _roi_bounds(
    width: int,
    height: int,
    roi: Iterable[int] | None,
) -> tuple[int, int, int, int]:
    if roi is None:
        return 0, 0, int(width), int(height)
    values = [int(v) for v in roi]
    if len(values) != 4:
        raise ValueError("roi must be [x, y, width, height]")
    x, y, w, h = values
    x0 = max(0, min(int(width), x))
    y0 = max(0, min(int(height), y))
    x1 = max(x0, min(int(width), x + max(0, w)))
    y1 = max(y0, min(int(height), y + max(0, h)))
    return x0, y0, x1, y1


def _rgb_in_range(
    rgb: tuple[int, int, int],
    lower: tuple[int, int, int],
    upper: tuple[int, int, int],
) -> bool:
    return all(lower[idx] <= rgb[idx] <= upper[idx] for idx in range(3))


def _matching_rgb_pixels(
    image: Any,
    lower: tuple[int, int, int],
    upper: tuple[int, int, int],
    roi: Iterable[int] | None,
    stride: int,
) -> set[tuple[int, int]]:
    x0, y0, x1, y1 = _roi_bounds(image.width, image.height, roi)
    matching: set[tuple[int, int]] = set()
    for v in range(y0, y1, stride):
        for u in range(x0, x1, stride):
            if _rgb_in_range(read_rgb(image, u, v), lower, upper):
                matching.add((u, v))
    return matching


def _require_cv2() -> tuple[Any, Any]:
    try:
        import cv2  # type: ignore[import-not-found]
        import numpy as np  # type: ignore[import-not-found]
    except Exception as exc:
        raise RuntimeError(
            "gray_plate locator requires OpenCV; install ROS dependency python3-opencv"
        ) from exc
    return cv2, np


def _image_to_rgb_array(image: Any, np: Any) -> Any:
    width = int(image.width)
    height = int(image.height)
    step = int(image.step)
    encoding = image.encoding.lower()
    data = np.frombuffer(bytes(image.data), dtype=np.uint8)
    bpp = _color_bytes_per_pixel(encoding)
    rows = data.reshape((height, step))
    active = rows[:, : width * bpp]
    if encoding in ("rgb8", "8uc3"):
        return active.reshape((height, width, 3)).copy()
    if encoding == "bgr8":
        return active.reshape((height, width, 3))[:, :, ::-1].copy()
    if encoding == "rgba8":
        return active.reshape((height, width, 4))[:, :, :3].copy()
    if encoding == "bgra8":
        return active.reshape((height, width, 4))[:, :, 2::-1].copy()
    if encoding in ("mono8", "8uc1"):
        mono = active.reshape((height, width))
        return np.repeat(mono[:, :, None], 3, axis=2)
    raise ValueError(f"Unsupported color image encoding: {image.encoding}")


def _gray_plate_candidate_from_contour(
    contour: Any,
    roi_mask: Any,
    roi_offset: tuple[int, int],
    desired: tuple[float, float] | None,
    min_area_px: float,
    max_area_px: float | None,
    min_width_px: float,
    max_width_px: float | None,
    min_height_px: float,
    max_height_px: float | None,
    min_aspect_ratio: float,
    max_aspect_ratio: float,
    min_rectangularity: float,
    min_mask_coverage: float,
    roi_diag_px: float,
) -> GrayPlateCandidate:
    cv2, np = _require_cv2()
    rect = cv2.minAreaRect(contour)
    (center_x, center_y), (rect_w, rect_h), angle_deg = rect
    width_px = float(max(rect_w, rect_h))
    height_px = float(min(rect_w, rect_h))
    area_px = float(width_px * height_px)
    aspect_ratio = width_px / max(height_px, 1.0)
    contour_area = float(cv2.contourArea(contour))
    rectangularity = contour_area / max(area_px, 1.0)

    filled = np.zeros_like(roi_mask)
    cv2.drawContours(filled, [contour], -1, 255, thickness=-1)
    filled_count = int((filled > 0).sum())
    mask_count = int(((roi_mask > 0) & (filled > 0)).sum())
    mask_coverage = mask_count / max(float(filled_count), 1.0)

    box = cv2.boxPoints(rect)
    x_offset, y_offset = roi_offset
    corners = tuple((float(x + x_offset), float(y + y_offset)) for x, y in box)
    xs = [point[0] for point in corners]
    ys = [point[1] for point in corners]
    bbox = (
        int(math.floor(min(xs))),
        int(math.floor(min(ys))),
        int(math.ceil(max(xs))),
        int(math.ceil(max(ys))),
    )
    center_u = float(center_x + x_offset)
    center_v = float(center_y + y_offset)
    center_distance_px = None
    distance_score = 1.0
    if desired is not None:
        center_distance_px = math.hypot(center_u - desired[0], center_v - desired[1])
        distance_score = max(0.0, 1.0 - center_distance_px / max(roi_diag_px, 1.0))

    size_score = min(1.0, area_px / max(float(min_area_px), 1.0))
    score = (
        0.35 * max(0.0, min(1.0, rectangularity))
        + 0.25 * max(0.0, min(1.0, mask_coverage))
        + 0.25 * distance_score
        + 0.15 * size_score
    )

    errors: list[str] = []
    if area_px < float(min_area_px):
        errors.append(f"plate area {area_px:.1f}px is smaller than min_area_px {float(min_area_px):.1f}")
    if max_area_px is not None and area_px > float(max_area_px):
        errors.append(f"plate area {area_px:.1f}px is larger than max_area_px {float(max_area_px):.1f}")
    if width_px < float(min_width_px):
        errors.append(f"plate width {width_px:.1f}px is smaller than min_width_px {float(min_width_px):.1f}")
    if max_width_px is not None and width_px > float(max_width_px):
        errors.append(f"plate width {width_px:.1f}px is larger than max_width_px {float(max_width_px):.1f}")
    if height_px < float(min_height_px):
        errors.append(f"plate height {height_px:.1f}px is smaller than min_height_px {float(min_height_px):.1f}")
    if max_height_px is not None and height_px > float(max_height_px):
        errors.append(f"plate height {height_px:.1f}px is larger than max_height_px {float(max_height_px):.1f}")
    if aspect_ratio < float(min_aspect_ratio) or aspect_ratio > float(max_aspect_ratio):
        errors.append(
            f"plate aspect_ratio {aspect_ratio:.3f} outside "
            f"[{float(min_aspect_ratio):.3f}, {float(max_aspect_ratio):.3f}]"
        )
    if rectangularity < float(min_rectangularity):
        errors.append(
            f"plate rectangularity {rectangularity:.3f} is below min_rectangularity {float(min_rectangularity):.3f}"
        )
    if mask_coverage < float(min_mask_coverage):
        errors.append(
            f"plate mask_coverage {mask_coverage:.3f} is below min_mask_coverage {float(min_mask_coverage):.3f}"
        )

    return GrayPlateCandidate(
        u=center_u,
        v=center_v,
        bbox=bbox,
        corners=corners,  # type: ignore[arg-type]
        width_px=width_px,
        height_px=height_px,
        angle_deg=float(angle_deg),
        area_px=area_px,
        score=score,
        rectangularity=rectangularity,
        mask_coverage=mask_coverage,
        aspect_ratio=aspect_ratio,
        center_distance_px=center_distance_px,
        passed=not errors,
        error="; ".join(errors),
    )


def _gray_plate_candidate_rank(candidate: GrayPlateCandidate) -> tuple[float, float, float]:
    distance_rank = 0.0
    if candidate.center_distance_px is not None:
        distance_rank = -float(candidate.center_distance_px)
    return (1.0 if candidate.passed else 0.0, float(candidate.score), distance_rank)


def _gray_plate_candidate_to_detection(candidate: GrayPlateCandidate) -> GrayPlateDetection:
    return GrayPlateDetection(
        u=candidate.u,
        v=candidate.v,
        bbox=candidate.bbox,
        corners=candidate.corners,
        width_px=candidate.width_px,
        height_px=candidate.height_px,
        angle_deg=candidate.angle_deg,
        area_px=candidate.area_px,
        score=candidate.score,
        rectangularity=candidate.rectangularity,
        mask_coverage=candidate.mask_coverage,
        aspect_ratio=candidate.aspect_ratio,
        center_distance_px=candidate.center_distance_px,
    )


def _format_gray_plate_candidate(candidate: GrayPlateCandidate, idx: int) -> str:
    distance = "none" if candidate.center_distance_px is None else f"{candidate.center_distance_px:.1f}"
    error = candidate.error or "passed"
    return (
        f"candidate {idx}: bbox={list(candidate.bbox)}, "
        f"center=[{candidate.u:.1f}, {candidate.v:.1f}], "
        f"size=[{candidate.width_px:.1f}, {candidate.height_px:.1f}], "
        f"area={candidate.area_px:.1f}, score={candidate.score:.3f}, "
        f"rectangularity={candidate.rectangularity:.3f}, "
        f"mask_coverage={candidate.mask_coverage:.3f}, "
        f"center_distance_px={distance}, error={error}"
    )


def _connected_pixel_components(
    matching: set[tuple[int, int]],
    stride: int,
) -> list[set[tuple[int, int]]]:
    components: list[set[tuple[int, int]]] = []
    step = max(1, int(stride))
    remaining = set(matching)

    while remaining:
        seed = remaining.pop()
        stack = [seed]
        component = {seed}

        while stack:
            u, v = stack.pop()

            for neighbor in (
                (u - step, v),
                (u + step, v),
                (u, v - step),
                (u, v + step),
            ):
                if neighbor in remaining:
                    remaining.remove(neighbor)
                    component.add(neighbor)
                    stack.append(neighbor)

        components.append(component)

    return sorted(components, key=len, reverse=True)


def _score_square_frame_windows(
    matching: set[tuple[int, int]],
    roi_bounds: tuple[int, int, int, int],
    min_pixels: int,
    min_size_px: float,
    max_size_px: float | None,
    aspect_tolerance: float,
    edge_tolerance_px: int,
    min_edge_score: float,
    min_side_coverage: float,
    max_fill_ratio: float,
    desired_pixel: Iterable[float] | None,
    scan_step_px: int,
) -> list[SquareFrameCandidate]:
    x0, y0, x1, y1 = roi_bounds
    roi_width = max(0, x1 - x0)
    roi_height = max(0, y1 - y0)
    max_side = min(roi_width, roi_height)
    if max_side <= 0:
        return []

    min_side = max(2, int(math.ceil(float(min_size_px))))
    if max_size_px is not None:
        max_side = min(max_side, int(math.floor(float(max_size_px))))
    if min_side > max_side:
        return []

    desired: tuple[float, float] | None = None
    if desired_pixel is not None:
        desired = _pair2(desired_pixel, "desired_pixel")
    prefix = _matching_prefix(matching, roi_bounds)
    step = max(1, int(scan_step_px))
    candidates: list[SquareFrameCandidate] = []
    top_rejected: list[SquareFrameCandidate] = []

    for side in range(min_side, max_side + 1, step):
        for min_v in range(y0, y1 - side + 1, step):
            max_v = min_v + side - 1
            for min_u in range(x0, x1 - side + 1, step):
                max_u = min_u + side - 1
                candidate = _score_square_frame_window(
                    prefix=prefix,
                    roi_bounds=roi_bounds,
                    bbox=(min_u, min_v, max_u, max_v),
                    min_pixels=min_pixels,
                    aspect_tolerance=aspect_tolerance,
                    edge_tolerance_px=edge_tolerance_px,
                    min_edge_score=min_edge_score,
                    min_side_coverage=min_side_coverage,
                    max_fill_ratio=max_fill_ratio,
                    desired=desired,
                )
                if candidate.passed:
                    candidates.append(candidate)
                elif candidate.pixel_count >= max(1, int(min_pixels) // 2):
                    _append_top_square_frame_candidate(top_rejected, candidate, limit=30)

    return candidates + _top_square_frame_candidates(top_rejected, limit=30)


def _score_square_frame_window(
    prefix: list[list[int]],
    roi_bounds: tuple[int, int, int, int],
    bbox: tuple[int, int, int, int],
    min_pixels: int,
    aspect_tolerance: float,
    edge_tolerance_px: int,
    min_edge_score: float,
    min_side_coverage: float,
    max_fill_ratio: float,
    desired: tuple[float, float] | None,
) -> SquareFrameCandidate:
    min_u, min_v, max_u, max_v = bbox
    width = max_u - min_u + 1
    height = max_v - min_v + 1
    size_px = (float(width) + float(height)) / 2.0
    pixel_count = _rect_count(prefix, roi_bounds, min_u, min_v, max_u, max_v)
    fill_ratio = pixel_count / max(float(width * height), 1.0)
    aspect_ratio = float(width) / max(float(height), 1.0)
    center_u = (min_u + max_u) / 2.0
    center_v = (min_v + max_v) / 2.0
    center_distance_px = None
    if desired is not None:
        center_distance_px = math.hypot(center_u - desired[0], center_v - desired[1])

    tol = max(0, int(edge_tolerance_px))
    top_count = _rect_count(prefix, roi_bounds, min_u, min_v, max_u, min_v)
    bottom_count = _rect_count(prefix, roi_bounds, min_u, max_v, max_u, max_v)
    left_count = _rect_count(prefix, roi_bounds, min_u, min_v, min_u, max_v)
    right_count = _rect_count(prefix, roi_bounds, max_u, min_v, max_u, max_v)
    side_coverages = (
        min(1.0, top_count / max(float(width), 1.0)),
        min(1.0, bottom_count / max(float(width), 1.0)),
        min(1.0, left_count / max(float(height), 1.0)),
        min(1.0, right_count / max(float(height), 1.0)),
    )
    side_coverage = min(side_coverages)

    inner_min_u = min_u + tol + 1
    inner_min_v = min_v + tol + 1
    inner_max_u = max_u - tol - 1
    inner_max_v = max_v - tol - 1
    if inner_min_u <= inner_max_u and inner_min_v <= inner_max_v:
        inner_count = _rect_count(prefix, roi_bounds, inner_min_u, inner_min_v, inner_max_u, inner_max_v)
        edge_pixels = pixel_count - inner_count
    else:
        edge_pixels = pixel_count
    expected_edge_pixels = max(1.0, float(2 * width + 2 * height - 4))
    edge_score = min(1.0, edge_pixels / expected_edge_pixels)

    errors: list[str] = []
    if pixel_count < int(min_pixels):
        errors.append(f"frame has {pixel_count} pixels, fewer than {int(min_pixels)}")
    if abs(aspect_ratio - 1.0) > float(aspect_tolerance):
        errors.append(
            f"frame aspect_ratio {aspect_ratio:.3f} exceeds tolerance {float(aspect_tolerance):.3f}"
        )
    if fill_ratio > float(max_fill_ratio):
        errors.append(
            f"frame fill_ratio {fill_ratio:.3f} exceeds max_fill_ratio {float(max_fill_ratio):.3f}"
        )
    if edge_score < float(min_edge_score):
        errors.append(
            f"frame edge_score {edge_score:.3f} is below min_edge_score {float(min_edge_score):.3f}"
        )
    if side_coverage < float(min_side_coverage):
        errors.append(
            f"frame side_coverage {side_coverage:.3f} is below "
            f"min_side_coverage {float(min_side_coverage):.3f}"
        )

    return SquareFrameCandidate(
        u=center_u,
        v=center_v,
        pixel_count=pixel_count,
        bbox=bbox,
        size_px=size_px,
        aspect_ratio=aspect_ratio,
        edge_score=edge_score,
        side_coverage=side_coverage,
        fill_ratio=fill_ratio,
        side_coverages=side_coverages,
        center_distance_px=center_distance_px,
        passed=not errors,
        error="; ".join(errors),
    )


def _matching_prefix(
    matching: set[tuple[int, int]],
    roi_bounds: tuple[int, int, int, int],
) -> list[list[int]]:
    x0, y0, x1, y1 = roi_bounds
    width = max(0, x1 - x0)
    height = max(0, y1 - y0)
    grid = [[0] * (width + 1) for _ in range(height + 1)]
    for u, v in matching:
        if x0 <= u < x1 and y0 <= v < y1:
            grid[v - y0 + 1][u - x0 + 1] = 1
    for row_idx in range(1, height + 1):
        row = grid[row_idx]
        prev = grid[row_idx - 1]
        running = 0
        for col_idx in range(1, width + 1):
            running += row[col_idx]
            row[col_idx] = running + prev[col_idx]
    return grid


def _rect_count(
    prefix: list[list[int]],
    roi_bounds: tuple[int, int, int, int],
    min_u: int,
    min_v: int,
    max_u: int,
    max_v: int,
) -> int:
    x0, y0, x1, y1 = roi_bounds
    min_u = max(x0, min(x1 - 1, int(min_u)))
    max_u = max(x0, min(x1 - 1, int(max_u)))
    min_v = max(y0, min(y1 - 1, int(min_v)))
    max_v = max(y0, min(y1 - 1, int(max_v)))
    if min_u > max_u or min_v > max_v:
        return 0
    left = min_u - x0
    right = max_u - x0 + 1
    top = min_v - y0
    bottom = max_v - y0 + 1
    return (
        prefix[bottom][right]
        - prefix[top][right]
        - prefix[bottom][left]
        + prefix[top][left]
    )


def _append_top_square_frame_candidate(
    candidates: list[SquareFrameCandidate],
    candidate: SquareFrameCandidate,
    limit: int,
) -> None:
    candidates.append(candidate)
    candidates.sort(key=_square_frame_candidate_rank, reverse=True)
    del candidates[max(0, int(limit)) :]


def _top_square_frame_candidates(
    candidates: list[SquareFrameCandidate],
    limit: int,
) -> list[SquareFrameCandidate]:
    return sorted(candidates, key=_square_frame_candidate_rank, reverse=True)[: max(0, int(limit))]


def _square_frame_candidate_rank(candidate: SquareFrameCandidate) -> tuple[float, float, float, float]:
    distance_score = 0.0
    if candidate.center_distance_px is not None:
        distance_score = -float(candidate.center_distance_px)
    return (
        1.0 if candidate.passed else 0.0,
        float(candidate.edge_score),
        float(candidate.side_coverage),
        distance_score,
    )


def _select_square_frame_candidate(
    candidates: list[SquareFrameCandidate],
    desired_pixel: Iterable[float] | None,
) -> SquareFrameCandidate:
    if desired_pixel is not None:
        return min(
            candidates,
            key=lambda candidate: (
                float("inf") if candidate.center_distance_px is None else candidate.center_distance_px,
                -candidate.edge_score,
                -candidate.side_coverage,
                candidate.fill_ratio,
            ),
        )
    return max(candidates, key=lambda candidate: (candidate.edge_score, candidate.side_coverage, -candidate.fill_ratio))


def _candidate_to_detection(candidate: SquareFrameCandidate) -> SquareFrameDetection:
    return SquareFrameDetection(
        u=candidate.u,
        v=candidate.v,
        pixel_count=candidate.pixel_count,
        bbox=candidate.bbox,
        size_px=candidate.size_px,
        aspect_ratio=candidate.aspect_ratio,
        edge_score=candidate.edge_score,
        side_coverage=candidate.side_coverage,
        fill_ratio=candidate.fill_ratio,
        side_coverages=candidate.side_coverages,
        center_distance_px=candidate.center_distance_px,
    )


def _detection_to_candidate(
    detection: SquareFrameDetection,
    desired_pixel: Iterable[float] | None,
    passed: bool,
    error: str,
) -> SquareFrameCandidate:
    center_distance_px = detection.center_distance_px
    if desired_pixel is not None and center_distance_px is None:
        desired_u, desired_v = _pair2(desired_pixel, "desired_pixel")
        center_distance_px = math.hypot(detection.u - desired_u, detection.v - desired_v)
    return SquareFrameCandidate(
        u=detection.u,
        v=detection.v,
        pixel_count=detection.pixel_count,
        bbox=detection.bbox,
        size_px=detection.size_px,
        aspect_ratio=detection.aspect_ratio,
        edge_score=detection.edge_score,
        side_coverage=detection.side_coverage,
        fill_ratio=detection.fill_ratio,
        side_coverages=detection.side_coverages,
        center_distance_px=center_distance_px,
        passed=passed,
        error=error,
    )


def _format_square_frame_candidate(candidate: SquareFrameCandidate, idx: int) -> str:
    sides = ",".join(f"{value:.2f}" for value in candidate.side_coverages)
    distance = "none" if candidate.center_distance_px is None else f"{candidate.center_distance_px:.1f}"
    error = candidate.error or "passed"
    return (
        f"window {idx}: pixels={candidate.pixel_count}, bbox={list(candidate.bbox)}, "
        f"size={candidate.size_px:.1f}, edge_score={candidate.edge_score:.3f}, "
        f"side_coverage={candidate.side_coverage:.3f}, side_coverages=[{sides}], "
        f"fill_ratio={candidate.fill_ratio:.3f}, center_distance_px={distance}, error={error}"
    )


def _blob_from_pixels(pixels: set[tuple[int, int]]) -> ColorBlob:
    if not pixels:
        raise ValueError("Cannot build blob from empty pixel set")
    count = len(pixels)
    sum_u = sum(u for u, _ in pixels)
    sum_v = sum(v for _, v in pixels)
    min_u = min(u for u, _ in pixels)
    min_v = min(v for _, v in pixels)
    max_u = max(u for u, _ in pixels)
    max_v = max(v for _, v in pixels)
    return ColorBlob(
        u=sum_u / count,
        v=sum_v / count,
        pixel_count=count,
        bbox=(min_u, min_v, max_u, max_v),
    )


def _square_frame_from_component(
    pixels: set[tuple[int, int]],
    min_pixels: int,
    min_size_px: float,
    max_size_px: float | None,
    aspect_tolerance: float,
    edge_tolerance_px: int,
    min_edge_score: float,
    min_side_coverage: float,
    max_fill_ratio: float,
) -> SquareFrameDetection:
    blob = _blob_from_pixels(pixels)
    if blob.pixel_count < min_pixels:
        raise ValueError(f"frame has {blob.pixel_count} pixels, fewer than {min_pixels}")
    min_u, min_v, max_u, max_v = blob.bbox
    width = max_u - min_u + 1
    height = max_v - min_v + 1
    size_px = (float(width) + float(height)) / 2.0
    if size_px < float(min_size_px):
        raise ValueError(f"frame size {size_px:.1f}px is smaller than min_size_px {min_size_px:.1f}")
    if max_size_px is not None and size_px > float(max_size_px):
        raise ValueError(f"frame size {size_px:.1f}px is larger than max_size_px {float(max_size_px):.1f}")

    aspect_ratio = float(width) / max(float(height), 1.0)
    if abs(aspect_ratio - 1.0) > float(aspect_tolerance):
        raise ValueError(
            f"frame aspect_ratio {aspect_ratio:.3f} exceeds tolerance {float(aspect_tolerance):.3f}"
        )

    area = max(width * height, 1)
    fill_ratio = blob.pixel_count / float(area)
    if fill_ratio > float(max_fill_ratio):
        raise ValueError(
            f"frame fill_ratio {fill_ratio:.3f} exceeds max_fill_ratio {float(max_fill_ratio):.3f}"
        )

    tol = max(0, int(edge_tolerance_px))
    edge_pixels = 0
    top_columns: set[int] = set()
    bottom_columns: set[int] = set()
    left_rows: set[int] = set()
    right_rows: set[int] = set()
    for u, v in pixels:
        on_top = abs(v - min_v) <= tol
        on_bottom = abs(v - max_v) <= tol
        on_left = abs(u - min_u) <= tol
        on_right = abs(u - max_u) <= tol
        if on_top or on_bottom or on_left or on_right:
            edge_pixels += 1
        if on_top:
            top_columns.add(u)
        if on_bottom:
            bottom_columns.add(u)
        if on_left:
            left_rows.add(v)
        if on_right:
            right_rows.add(v)

    edge_score = edge_pixels / max(float(blob.pixel_count), 1.0)
    if edge_score < float(min_edge_score):
        raise ValueError(
            f"frame edge_score {edge_score:.3f} is below min_edge_score {float(min_edge_score):.3f}"
        )

    side_coverage = min(
        len(top_columns) / max(float(width), 1.0),
        len(bottom_columns) / max(float(width), 1.0),
        len(left_rows) / max(float(height), 1.0),
        len(right_rows) / max(float(height), 1.0),
    )
    side_coverages = (
        len(top_columns) / max(float(width), 1.0),
        len(bottom_columns) / max(float(width), 1.0),
        len(left_rows) / max(float(height), 1.0),
        len(right_rows) / max(float(height), 1.0),
    )
    if side_coverage < float(min_side_coverage):
        raise ValueError(
            f"frame side_coverage {side_coverage:.3f} is below "
            f"min_side_coverage {float(min_side_coverage):.3f}"
        )

    return SquareFrameDetection(
        u=(min_u + max_u) / 2.0,
        v=(min_v + max_v) / 2.0,
        pixel_count=blob.pixel_count,
        bbox=blob.bbox,
        size_px=size_px,
        aspect_ratio=aspect_ratio,
        edge_score=edge_score,
        side_coverage=side_coverage,
        fill_ratio=fill_ratio,
        side_coverages=side_coverages,
    )


def _pair2(raw: Iterable[float], name: str) -> tuple[float, float]:
    values = [float(value) for value in raw]
    if len(values) != 2:
        raise ValueError(f"{name} must contain [u, v]")
    return values[0], values[1]
