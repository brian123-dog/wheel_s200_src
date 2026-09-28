from __future__ import annotations

import math
import time
from datetime import datetime
from pathlib import Path
from typing import Any

from geometry_msgs.msg import PoseStamped
from rclpy.duration import Duration
from rclpy.time import Time

from rg_cabinet.geometry import Intrinsics, Pose, backproject_pixel
from rg_cabinet.perception.element_detector import ElementTarget
from rg_cabinet_ros2.camera_cache import CameraFrameCache
from rg_cabinet_ros2.fresh_vision import (
    PanelLayoutRejected, SamplingPolicy, VisionSamplingError, sample_target,
)
from rg_cabinet_ros2.image_tools import (
    camera_info_to_intrinsics,
    find_color_blobs,
    find_gray_plate_with_debug,
    find_square_frame_with_debug,
    median_bbox_ring_depth_meters,
    median_depth_meters,
    write_gray_plate_debug_images,
    write_square_frame_debug_images,
)


class VisionTargetResolver:
    def __init__(
        self,
        node: Any,
        camera_cache: CameraFrameCache,
        tf_buffer: Any,
    ) -> None:
        self.node = node
        self.camera_cache = camera_cache
        self.tf_buffer = tf_buffer
        self.target_pub = node.create_publisher(PoseStamped, "/rg_cabinet/target_pose", 10)
        self.resolved_targets: list[dict[str, Any]] = []
        self.last_target_debug: dict[str, Any] | None = None
        self._panel_layout_detectors: dict[str, Any] = {}

    def reset(self) -> None:
        self.resolved_targets.clear()
        self.last_target_debug = None
        self._panel_layout_detectors.clear()

    def fresh_sampling_settings(self, config, step):
        target = step.get("target", {})
        if str(target.get("source", target.get("from", ""))).lower() not in ("camera", "vision", "gemini2"):
            return None
        vision = config.get("vision", {})
        name = str(target.get("detection", step.get("name", "target")))
        detection = self._merged_detection_config(vision, target, name)
        settings = detection.get("fresh_sampling", vision.get("fresh_sampling", {}))
        if str(detection.get("locator", target.get("locator", ""))).strip().lower() != "panel_layout":
            return None
        return settings if _as_bool(settings.get("enabled", False)) else None

    def resolve(self, config: dict[str, Any], step: dict[str, Any]) -> ElementTarget | None:
        settings = self.fresh_sampling_settings(config, step)
        if settings is None:
            return self._resolve_once(config, step)
        policy = SamplingPolicy.from_config(settings)
        if not _as_bool(self.node.get_parameter("vision_enabled").value):
            raise RuntimeError("Vision target requested but vision_enabled is false")
        name = str(step.get("name", "target"))

        def failed(item, frame):
            if item.get("frame_timeout") or self.last_target_debug is None:
                self.last_target_debug = {"name": name, "locator": "panel_layout"}
            debug = self.last_target_debug
            debug["error"] = item["reason"]
            debug["sampling_attempt"] = dict(item)
            from rg_cabinet_ros2.vision_failure_artifacts import save_failure_artifacts
            detector = self._panel_layout_detectors.get(debug.get("panel_layout_template"))
            paths = save_failure_artifacts(frame, debug, config.get("vision", {}), name, detector)
            debug["debug_images"] = paths
            item["debug_images"] = paths
            reason = item["reason"]
            if "insufficient inlier" in reason:
                reason = "面板全局匹配不足"
            elif "time budget exhausted during detection" in reason:
                reason = (f"已检出目标，但两帧确认超时（预算 {policy.total_timeout_s:.1f}s，"
                          f"本帧检测 {item.get('detection_s', 0):.2f}s）")
            elif item.get("frame_timeout"):
                issues = item.get("frames", {}).get("problems", [])
                reason = "等待新帧超时：" + "; ".join(issues)
            self.node.get_logger().warning(
                f"[视觉重采] {name} {item['attempt']}/{policy.max_attempts} | {reason}"
            )

        def detect(frame):
            self.last_target_debug = None
            try:
                return self._resolve_once(config, step, frames=frame, publish=False)
            except PanelLayoutRejected:
                raise
            except Exception as exc:
                # TF, configuration, depth and workspace failures are not retried.
                failed({"attempt": 0, "status": "fatal", "reason": str(exc)}, frame)
                raise

        try:
            target, report = sample_target(
                self.camera_cache, policy,
                lambda: self.node.get_clock().now().nanoseconds * 1e-9, detect, failed)
        except VisionSamplingError as exc:
            if self.last_target_debug is None:
                self.last_target_debug = {"name": name, "locator": "panel_layout"}
            self.last_target_debug["fresh_sampling"] = exc.report
            self.last_target_debug["error"] = str(exc)
            raise
        target.details["fresh_sampling"] = report
        self.last_target_debug = target.details
        self.resolved_targets.append(target.details)
        self._publish_target_pose(target)
        self.node.get_logger().info(
            f"[视觉采样] {name} | {report['confirmations']} 帧定位一致 | "
            f"尝试 {report['attempts']} 次，耗时 {report['elapsed_s']:.2f}s"
        )
        return target

    def _resolve_once(self, config: dict[str, Any], step: dict[str, Any],
                      *, frames=None, publish=True) -> ElementTarget | None:
        target_cfg = step.get("target", {})
        source = str(target_cfg.get("source", target_cfg.get("from", ""))).lower()
        if source not in ("camera", "vision", "gemini2"):
            return None
        if not _as_bool(self.node.get_parameter("vision_enabled").value):
            raise RuntimeError("Vision target requested but vision_enabled is false")
        if frames is None and not self.camera_cache.ready:
            raise RuntimeError("Vision target requested but color/depth/camera_info are not ready")

        vision_cfg = config.get("vision", {})
        detection_name = str(target_cfg.get("detection", step.get("name", "target")))
        detection_cfg = self._merged_detection_config(vision_cfg, target_cfg, detection_name)
        target_name = str(step.get("name", target_cfg.get("name", detection_name)))
        target_kind = str(target_cfg.get("kind", detection_cfg.get("kind", step["skill"])))
        confidence = float(detection_cfg.get("confidence", target_cfg.get("confidence", 1.0)))

        color = frames.color if frames is not None else self.camera_cache.color
        depth = frames.depth if frames is not None else self.camera_cache.depth
        camera_info = frames.camera_info if frames is not None else self.camera_cache.camera_info
        if color is None or depth is None or camera_info is None:
            raise RuntimeError("Camera cache became incomplete during target resolution")

        intrinsics = camera_info_to_intrinsics(camera_info, depth_scale=1.0)
        camera_frame = str(
            detection_cfg.get(
                "camera_frame",
                getattr(color.header, "frame_id", "") or self.node.get_parameter("camera_frame").value,
            )
        )
        if not camera_frame:
            raise RuntimeError("Camera frame is empty; set target.camera_frame or camera_frame parameter")
        base_frame = str(detection_cfg.get("base_frame", self.node.get_parameter("base_frame").value))
        alignment_reference = self._alignment_reference(
            detection_cfg,
            base_frame,
            camera_frame,
            intrinsics,
        )

        locator = str(detection_cfg.get("locator", target_cfg.get("locator", ""))).strip().lower()
        locator_debug: dict[str, Any] = {}
        if locator == "panel_layout":
            if target_kind != "knob":
                raise RuntimeError("panel_layout currently supports knob targets only")
            target_u, target_v, depth_m, depth_source, locator_debug = (
                self._resolve_panel_layout_target(
                    color, depth, detection_cfg, vision_cfg, target_name, detection_name
                )
            )
        elif locator == "gray_plate" and target_kind == "knob":
            target_u, target_v, depth_m, depth_source, locator_debug = (
                self._resolve_gray_plate_target(
                    color,
                    depth,
                    detection_cfg,
                    vision_cfg,
                    target_name,
                    alignment_reference_pixel=(
                        alignment_reference.get("alignment_reference_pixel")
                        if alignment_reference is not None
                        else None
                    ),
                )
            )
        elif locator == "square_frame" and target_kind == "knob":
            target_u, target_v, depth_m, depth_source, locator_debug = (
                self._resolve_square_frame_target(
                    color,
                    depth,
                    detection_cfg,
                    vision_cfg,
                    target_name,
                )
            )
        else:
            blobs = find_color_blobs(
                color,
                rgb_min=_required_rgb(detection_cfg, "rgb_min"),
                rgb_max=_required_rgb(detection_cfg, "rgb_max"),
                min_pixels=int(detection_cfg.get("min_pixels", vision_cfg.get("min_pixels", 20))),
                roi=detection_cfg.get("roi", vision_cfg.get("roi")),
                stride=int(detection_cfg.get("stride", vision_cfg.get("stride", 1))),
            )
            blob, depth_m, depth_source, candidate_errors = self._select_depth_valid_blob(
                blobs,
                depth,
                color.width,
                color.height,
                detection_cfg,
                vision_cfg,
            )
            target_u = float(blob.u)
            target_v = float(blob.v)
            locator_debug = {
                "pixel_count": blob.pixel_count,
                "candidate_count": len(blobs),
                "bbox": list(blob.bbox),
            }
        point_camera = backproject_pixel(target_u, target_v, depth_m, intrinsics, frame=camera_frame)
        desired_pixel = detection_cfg.get("desired_pixel", detection_cfg.get("servo_desired_pixel"))
        pixel_error_base_m: list[float] | None = None
        if desired_pixel is not None:
            desired_u, desired_v = _pixel2(desired_pixel, "desired_pixel")
            desired_camera = backproject_pixel(
                desired_u,
                desired_v,
                depth_m,
                intrinsics,
                frame=camera_frame,
            )
            error_camera = (
                point_camera.x - desired_camera.x,
                point_camera.y - desired_camera.y,
                point_camera.z - desired_camera.z,
            )
        else:
            error_camera = None
        point_base = self._transform_camera_point(point_camera, base_frame, camera_frame)
        if error_camera is not None:
            pixel_error_base_m = list(
                self._rotate_camera_vector(error_camera, base_frame, camera_frame)
            )
        alignment_error_base_m: list[float] | None = None
        if alignment_reference is not None:
            tcp_base = alignment_reference["tcp_center_base_m"]
            alignment_error_base_m = [
                float(point_base[0]) - float(tcp_base[0]),
                float(point_base[1]) - float(tcp_base[1]),
                float(point_base[2]) - float(tcp_base[2]),
            ]
        offset = _vector3(detection_cfg.get("offset_base_m", [0.0, 0.0, 0.0]), "offset_base_m")
        rpy = _vector3(
            detection_cfg.get(
                "rpy_base",
                target_cfg.get("rpy", vision_cfg.get("default_rpy_base", [0.0, 0.0, 0.0])),
            ),
            "rpy_base",
        )
        pose = Pose(
            x=point_base[0] + offset[0],
            y=point_base[1] + offset[1],
            z=point_base[2] + offset[2],
            rx=rpy[0],
            ry=rpy[1],
            rz=rpy[2],
            frame=base_frame,
        )
        debug = {
            "name": target_name,
            "kind": target_kind,
            "source": source,
            "camera_frame": camera_frame,
            "base_frame": base_frame,
            "pixel": [round(target_u, 3), round(target_v, 3)],
            "image_size": [int(color.width), int(color.height)],
            "depth_m": round(depth_m, 6),
            "depth_source": depth_source,
            "pose": pose.as_list(),
        }
        debug.update(locator_debug)
        if desired_pixel is not None:
            desired_u, desired_v = _pixel2(desired_pixel, "desired_pixel")
            debug["desired_pixel"] = [
                round(desired_u, 3),
                round(desired_v, 3),
            ]
        if pixel_error_base_m is not None:
            debug["pixel_error_base_m"] = [
                round(float(value), 6) for value in pixel_error_base_m
            ]
        if alignment_reference is not None:
            debug.update(alignment_reference)
            debug["plate_center_base_m"] = [
                round(float(value), 6) for value in point_base
            ]
        if alignment_error_base_m is not None:
            debug["alignment_error_base_m"] = [
                round(float(value), 6) for value in alignment_error_base_m
            ]
        self.last_target_debug = debug
        _validate_workspace_bounds(pose, detection_cfg, vision_cfg)
        target = ElementTarget(
            name=target_name,
            kind=target_kind,
            pose=pose,
            confidence=confidence,
            details=debug,
        )
        if publish:
            self.resolved_targets.append(debug)
            self._publish_target_pose(target)
        return target

    def _resolve_panel_layout_target(
        self,
        color: Any,
        depth: Any,
        detection_cfg: dict[str, Any],
        vision_cfg: dict[str, Any],
        target_name: str,
        detection_name: str,
    ) -> tuple[float, float, float, str, dict[str, Any]]:
        # Lazy import keeps all legacy modes independent of SIFT availability.
        import cv2
        import json
        import numpy as np
        from rg_cabinet_ros2.image_tools import _image_to_rgb_array
        from rg_cabinet_ros2.panel_layout import PanelLayoutDetector, draw_panel_layout_result

        raw_templates = detection_cfg.get("panel_layout_templates")
        if raw_templates is None:
            raw_templates = [detection_cfg.get("panel_layout_template")]
        if not isinstance(raw_templates, list) or not 1 <= len(raw_templates) <= 8 or not all(raw_templates):
            raise RuntimeError("panel_layout requires one to eight template paths")
        expected_id = str(detection_cfg.get("panel_layout_target_id", detection_name))
        detectors = {}
        template_load_start = time.monotonic()
        loaded_templates = 0
        for raw_template in raw_templates:
            template_path = Path(str(raw_template)).expanduser()
            if not template_path.is_absolute():
                config_path = self._parameter_value("task_config", "")
                if not config_path:
                    raise RuntimeError("Relative panel_layout_template requires task_config path")
                template_path = Path(str(config_path)).expanduser().resolve().parent / template_path
            key = str(template_path.resolve())
            detector = self._panel_layout_detectors.get(key)
            if detector is None:
                detector = PanelLayoutDetector.from_template(Path(key))
                loaded_templates += 1
                self._panel_layout_detectors[key] = detector
            if str(detector.config["target_id"]) != expected_id:
                raise RuntimeError(f"Panel-layout template target_id does not match {expected_id}")
            detectors[key] = detector

        template_load_s = time.monotonic() - template_load_start
        detection_start = time.monotonic()
        rgb = _image_to_rgb_array(color, np)
        bgr = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
        template_summaries = []
        if len(detectors) == 1:
            key, detector = next(iter(detectors.items()))
            result = detector.process(bgr)
        else:
            from rg_cabinet_ros2.panel_template_bank import locate_with_templates
            key, result, template_summaries = locate_with_templates(bgr, detectors)
        debug = {
            "name": target_name,
            "locator": "panel_layout",
            "panel_layout_template": key,
            "panel_layout": result,
            "panel_layout_templates": template_summaries,
            "panel_layout_timing": {
                "template_load_s": template_load_s,
                "templates_loaded": loaded_templates,
                "template_count": len(detectors),
                "detection_s": time.monotonic() - detection_start,
            },
        }
        if _as_bool(detection_cfg.get("save_debug_images", vision_cfg.get("save_debug_images", True))):
            debug_dir = Path(str(detection_cfg.get("vision_debug_dir", vision_cfg.get("vision_debug_dir", "vision_debug"))))
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
            overlay_path = debug_dir / f"{_safe_debug_name(target_name)}_{timestamp}_panel_layout.png"
            color_path = overlay_path.with_name(overlay_path.stem + "_color.png")
            metadata_path = overlay_path.with_suffix(".json")
            debug["debug_images"] = {}
            try:
                debug_dir.mkdir(parents=True, exist_ok=True)
                if not cv2.imwrite(str(color_path), bgr):
                    raise RuntimeError("OpenCV could not save panel-layout raw image")
                debug["debug_images"]["color_path"] = str(color_path.resolve())
                overlay = draw_panel_layout_result(bgr, {**result, "file": target_name})
                if not cv2.imwrite(str(overlay_path), overlay):
                    raise RuntimeError("OpenCV could not save panel-layout overlay")
                debug["debug_images"]["overlay_path"] = str(overlay_path.resolve())
                debug["debug_images"]["metadata_path"] = str(metadata_path.resolve())
                metadata_path.write_text(json.dumps(debug, ensure_ascii=False, indent=2), encoding="utf-8")
            except Exception as exc:
                debug["debug_images"]["error"] = str(exc)
        self.last_target_debug = debug
        if result["status"] != "ACCEPTED_2D":
            debug["error"] = result["reason"]
            # Explicit mode selection: never fall back to a legacy color blob.
            raise PanelLayoutRejected(f"Panel-layout target rejected: {result['reason']}")

        target_u, target_v = (float(v) for v in result["axis_center_px"])
        corners = np.float32(result.get("local_bbox_corners", result["plate_corners"]))
        lo = np.floor(corners.min(axis=0)).astype(int)
        hi = np.ceil(corners.max(axis=0)).astype(int)
        bbox = (int(lo[0]), int(lo[1]), int(hi[0]), int(hi[1]))
        offset_u, offset_v = _pixel2(detection_cfg.get("knob_center_offset_px", [0.0, 0.0]), "knob_center_offset_px")
        target_u += offset_u
        target_v += offset_v
        if not (0 <= target_u < int(color.width) and 0 <= target_v < int(color.height)):
            raise RuntimeError("Panel-layout target offset leaves image bounds")
        # Reuse existing depth limits, ring/center sampling and downstream TF,
        # TCP alignment and workspace checks. No new motion behavior is enabled.
        depth_m, source = self._depth_for_gray_plate(
            bbox, target_u, target_v, depth, int(color.width), int(color.height), detection_cfg, vision_cfg
        )
        depth_source = source.replace("gray_plate_", "panel_layout_", 1)
        debug.update({"bbox": list(bbox), "plate_bbox": list(bbox),
                      "plate_corners": result["plate_corners"],
                      "axis_center_pixel": result["axis_center_px"],
                      "local_geometry": result.get("local_geometry", "square_plate"),
                      "knob_outline_px": result.get("knob_outline_px"),
                      "candidate_count": (1 if result.get("local_geometry") == "round_knob"
                                          else result["refinement"]["hub_cap_candidates"])})
        return target_u, target_v, depth_m, depth_source, debug

    def _resolve_square_frame_target(
        self,
        color: Any,
        depth: Any,
        detection_cfg: dict[str, Any],
        vision_cfg: dict[str, Any],
        target_name: str,
    ) -> tuple[float, float, float, str, dict[str, Any]]:
        desired_pixel = detection_cfg.get("desired_pixel", detection_cfg.get("servo_desired_pixel"))
        search = find_square_frame_with_debug(
            color,
            rgb_min=_rgb_or_fallback(detection_cfg, "frame_rgb_min", "rgb_min"),
            rgb_max=_rgb_or_fallback(detection_cfg, "frame_rgb_max", "rgb_max"),
            min_pixels=int(detection_cfg.get("frame_min_pixels", detection_cfg.get("min_pixels", 20))),
            roi=detection_cfg.get("roi", vision_cfg.get("roi")),
            stride=int(detection_cfg.get("frame_stride", detection_cfg.get("stride", 1))),
            min_size_px=float(detection_cfg.get("frame_min_size_px", 12.0)),
            max_size_px=_optional_float(detection_cfg.get("frame_max_size_px")),
            aspect_tolerance=float(detection_cfg.get("frame_aspect_tolerance", 0.25)),
            edge_tolerance_px=int(detection_cfg.get("frame_edge_tolerance_px", 2)),
            min_edge_score=float(detection_cfg.get("frame_min_edge_score", 0.55)),
            min_side_coverage=float(detection_cfg.get("frame_min_side_coverage", 0.45)),
            max_fill_ratio=float(detection_cfg.get("frame_max_fill_ratio", 0.70)),
            desired_pixel=desired_pixel,
            scan_step_px=int(detection_cfg.get("frame_scan_step_px", vision_cfg.get("frame_scan_step_px", 1))),
        )
        blob_debug = self._black_blob_debug(color, detection_cfg, vision_cfg)
        if search.detection is None:
            debug_images = self._save_square_frame_debug_images(
                color,
                detection_cfg,
                vision_cfg,
                target_name,
                search,
                selected_frame=None,
                blob_debug=blob_debug,
                desired_pixel=desired_pixel,
            )
            debug = {
                "name": target_name,
                "locator": "square_frame",
                "error": search.error,
                "frame_candidates": _square_frame_candidates_debug(search.candidates),
                "frame_candidate_count": len(search.candidates),
                "roi": list(search.roi_bounds),
            }
            debug.update(blob_debug)
            if debug_images:
                debug["debug_images"] = debug_images
            self.last_target_debug = debug
            image_hint = ""
            if debug_images:
                image_hint = f"; debug_images={debug_images}"
            raise RuntimeError(f"{search.error}{image_hint}")

        frame = search.detection
        offset_u, offset_v = _pixel2(
            detection_cfg.get("knob_center_offset_px", [0.0, 0.0]),
            "knob_center_offset_px",
        )
        target_u = float(frame.u) + offset_u
        target_v = float(frame.v) + offset_v
        depth_m, depth_source = self._depth_for_square_frame(
            frame.bbox,
            target_u,
            target_v,
            depth,
            int(color.width),
            int(color.height),
            detection_cfg,
            vision_cfg,
        )
        debug_images = self._save_square_frame_debug_images(
            color,
            detection_cfg,
            vision_cfg,
            target_name,
            search,
            selected_frame=frame,
            blob_debug=blob_debug,
            desired_pixel=desired_pixel,
        )
        debug = {
            "locator": "square_frame",
            "pixel_count": frame.pixel_count,
            "candidate_count": len(search.candidates),
            "bbox": list(frame.bbox),
            "frame_bbox": list(frame.bbox),
            "frame_center_pixel": [round(float(frame.u), 3), round(float(frame.v), 3)],
            "knob_center_offset_px": [round(offset_u, 3), round(offset_v, 3)],
            "frame_size_px": round(float(frame.size_px), 3),
            "frame_aspect_ratio": round(float(frame.aspect_ratio), 6),
            "frame_edge_score": round(float(frame.edge_score), 6),
            "frame_side_coverage": round(float(frame.side_coverage), 6),
            "frame_side_coverages": [round(float(value), 6) for value in frame.side_coverages],
            "frame_fill_ratio": round(float(frame.fill_ratio), 6),
            "frame_candidates": _square_frame_candidates_debug(search.candidates),
        }
        if debug_images:
            debug["debug_images"] = debug_images
        debug.update(blob_debug)
        return target_u, target_v, depth_m, depth_source, debug

    def _resolve_gray_plate_target(
        self,
        color: Any,
        depth: Any,
        detection_cfg: dict[str, Any],
        vision_cfg: dict[str, Any],
        target_name: str,
        alignment_reference_pixel: Any = None,
    ) -> tuple[float, float, float, str, dict[str, Any]]:
        desired_pixel = detection_cfg.get("desired_pixel", detection_cfg.get("servo_desired_pixel"))
        search = find_gray_plate_with_debug(
            color,
            roi=detection_cfg.get("roi", vision_cfg.get("roi")),
            desired_pixel=desired_pixel,
            saturation_max=int(detection_cfg.get("plate_saturation_max", vision_cfg.get("plate_saturation_max", 80))),
            value_min=int(detection_cfg.get("plate_value_min", vision_cfg.get("plate_value_min", 70))),
            value_max=int(detection_cfg.get("plate_value_max", vision_cfg.get("plate_value_max", 235))),
            min_area_px=float(detection_cfg.get("plate_min_area_px", 700.0)),
            max_area_px=_optional_float(detection_cfg.get("plate_max_area_px", 6000.0)),
            min_width_px=float(detection_cfg.get("plate_min_width_px", 25.0)),
            max_width_px=_optional_float(detection_cfg.get("plate_max_width_px", 100.0)),
            min_height_px=float(detection_cfg.get("plate_min_height_px", 25.0)),
            max_height_px=_optional_float(detection_cfg.get("plate_max_height_px", 100.0)),
            min_aspect_ratio=float(detection_cfg.get("plate_min_aspect_ratio", 0.55)),
            max_aspect_ratio=float(detection_cfg.get("plate_max_aspect_ratio", 1.80)),
            close_kernel_px=int(detection_cfg.get("plate_close_kernel_px", 9)),
            close_iterations=int(detection_cfg.get("plate_close_iterations", 2)),
            min_rectangularity=float(detection_cfg.get("plate_min_rectangularity", 0.55)),
            min_mask_coverage=float(detection_cfg.get("plate_min_mask_coverage", 0.45)),
        )
        blob_debug = self._black_blob_debug(color, detection_cfg, vision_cfg)
        if search.detection is None:
            # Fallback: when gray_plate fails (e.g. full-image relaxed search),
            # try using the black blob detector result directly as the knob target.
            blob_center = blob_debug.get("blob_centroid_pixel")
            blob_bbox = blob_debug.get("blob_bbox")
            if blob_center is not None and blob_bbox is not None:
                try:
                    fallback_u = float(blob_center[0])
                    fallback_v = float(blob_center[1])
                    fallback_depth_m, fallback_depth_source = self._depth_for_blob_fallback(
                        tuple(blob_bbox),
                        fallback_u,
                        fallback_v,
                        depth,
                        int(color.width),
                        int(color.height),
                        detection_cfg,
                        vision_cfg,
                    )
                    _validate_depth_m(fallback_depth_m, detection_cfg, vision_cfg)
                    offset_u, offset_v = _pixel2(
                        detection_cfg.get("knob_center_offset_px", [0.0, 0.0]),
                        "knob_center_offset_px",
                    )
                    target_u = fallback_u + offset_u
                    target_v = fallback_v + offset_v
                    depth_m = fallback_depth_m
                    depth_source = fallback_depth_source
                    locator_debug_out = {
                        "locator": "gray_plate_blob_fallback",
                        "pixel_count": int(blob_debug.get("blob_pixel_count", 0)),
                        "candidate_count": int(blob_debug.get("blob_candidate_count", 0)),
                        "bbox": list(blob_bbox),
                        "blob_centroid_pixel": [round(float(blob_center[0]), 3), round(float(blob_center[1]), 3)],
                        "roi": list(search.roi_bounds),
                        "fallback_reason": search.error,
                    }
                    debug_images = self._save_gray_plate_debug_images(
                        color,
                        detection_cfg,
                        vision_cfg,
                        target_name,
                        search,
                        selected_plate=None,
                        blob_debug=blob_debug,
                        desired_pixel=desired_pixel,
                        alignment_reference_pixel=alignment_reference_pixel,
                    )
                    if debug_images:
                        locator_debug_out["debug_images"] = debug_images
                    self.last_target_debug = locator_debug_out
                    self.resolved_targets.append(locator_debug_out)
                    return target_u, target_v, depth_m, depth_source, locator_debug_out
                except Exception:
                    pass  # fallback also failed; continue to raise the original error

            debug_images = self._save_gray_plate_debug_images(
                color,
                detection_cfg,
                vision_cfg,
                target_name,
                search,
                selected_plate=None,
                blob_debug=blob_debug,
                desired_pixel=desired_pixel,
                alignment_reference_pixel=alignment_reference_pixel,
            )
            debug = {
                "name": target_name,
                "locator": "gray_plate",
                "error": search.error,
                "plate_candidates": _gray_plate_candidates_debug(search.candidates),
                "plate_candidate_count": len(search.candidates),
                "roi": list(search.roi_bounds),
            }
            debug.update(blob_debug)
            if debug_images:
                debug["debug_images"] = debug_images
            self.last_target_debug = debug
            image_hint = ""
            if debug_images:
                image_hint = f"; debug_images={debug_images}"
            raise RuntimeError(f"{search.error}{image_hint}")

        plate = search.detection
        offset_u, offset_v = _pixel2(
            detection_cfg.get("knob_center_offset_px", [0.0, 0.0]),
            "knob_center_offset_px",
        )
        target_u = float(plate.u) + offset_u
        target_v = float(plate.v) + offset_v
        depth_m, depth_source = self._depth_for_gray_plate(
            plate.bbox,
            target_u,
            target_v,
            depth,
            int(color.width),
            int(color.height),
            detection_cfg,
            vision_cfg,
        )
        debug_images = self._save_gray_plate_debug_images(
            color,
            detection_cfg,
            vision_cfg,
            target_name,
            search,
            selected_plate=plate,
            blob_debug=blob_debug,
            desired_pixel=desired_pixel,
            alignment_reference_pixel=alignment_reference_pixel,
        )
        debug = {
            "locator": "gray_plate",
            "pixel_count": int(round(plate.area_px)),
            "candidate_count": len(search.candidates),
            "bbox": list(plate.bbox),
            "plate_bbox": list(plate.bbox),
            "plate_corners": [
                [round(float(u), 3), round(float(v), 3)]
                for u, v in plate.corners
            ],
            "plate_center_pixel": [round(float(plate.u), 3), round(float(plate.v), 3)],
            "knob_center_offset_px": [round(offset_u, 3), round(offset_v, 3)],
            "plate_width_px": round(float(plate.width_px), 3),
            "plate_height_px": round(float(plate.height_px), 3),
            "plate_angle_deg": round(float(plate.angle_deg), 3),
            "plate_area_px": round(float(plate.area_px), 3),
            "plate_score": round(float(plate.score), 6),
            "plate_rectangularity": round(float(plate.rectangularity), 6),
            "plate_mask_coverage": round(float(plate.mask_coverage), 6),
            "plate_aspect_ratio": round(float(plate.aspect_ratio), 6),
            "plate_candidates": _gray_plate_candidates_debug(search.candidates),
        }
        if debug_images:
            debug["debug_images"] = debug_images
        debug.update(blob_debug)
        return target_u, target_v, depth_m, depth_source, debug

    def _save_gray_plate_debug_images(
        self,
        color: Any,
        detection_cfg: dict[str, Any],
        vision_cfg: dict[str, Any],
        target_name: str,
        search: Any,
        selected_plate: Any,
        blob_debug: dict[str, Any],
        desired_pixel: Any,
        alignment_reference_pixel: Any = None,
    ) -> dict[str, Any]:
        if not _as_bool(detection_cfg.get("save_debug_images", vision_cfg.get("save_debug_images", True))):
            return {}
        raw_dir = str(detection_cfg.get("vision_debug_dir", vision_cfg.get("vision_debug_dir", "vision_debug")))
        debug_dir = Path(raw_dir)
        if not debug_dir.is_absolute():
            debug_dir = Path.cwd() / debug_dir
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
        prefix = debug_dir / f"{_safe_debug_name(target_name)}_{timestamp}"
        blob_center = blob_debug.get("blob_centroid_pixel")
        try:
            return write_gray_plate_debug_images(
                color,
                prefix,
                mask=search.mask,
                edges=search.edges,
                roi_bounds=search.roi_bounds,
                selected_plate=selected_plate,
                candidates=search.candidates,
                desired_pixel=desired_pixel,
                alignment_reference_pixel=alignment_reference_pixel,
                blob_center=blob_center,
            )
        except Exception as exc:
            return {"error": str(exc)}

    def _save_square_frame_debug_images(
        self,
        color: Any,
        detection_cfg: dict[str, Any],
        vision_cfg: dict[str, Any],
        target_name: str,
        search: Any,
        selected_frame: Any,
        blob_debug: dict[str, Any],
        desired_pixel: Any,
    ) -> dict[str, Any]:
        if not _as_bool(detection_cfg.get("save_debug_images", vision_cfg.get("save_debug_images", True))):
            return {}
        raw_dir = str(detection_cfg.get("vision_debug_dir", vision_cfg.get("vision_debug_dir", "vision_debug")))
        debug_dir = Path(raw_dir)
        if not debug_dir.is_absolute():
            debug_dir = Path.cwd() / debug_dir
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
        prefix = debug_dir / f"{_safe_debug_name(target_name)}_{timestamp}"
        blob_center = blob_debug.get("blob_centroid_pixel")
        try:
            return write_square_frame_debug_images(
                color,
                prefix,
                matching_pixels=search.matching_pixels,
                roi_bounds=search.roi_bounds,
                selected_frame=selected_frame,
                candidates=search.candidates,
                desired_pixel=desired_pixel,
                blob_center=blob_center,
            )
        except Exception as exc:
            return {"error": str(exc)}

    def _depth_for_square_frame(
        self,
        frame_bbox: tuple[int, int, int, int],
        target_u: float,
        target_v: float,
        depth: Any,
        color_width: int,
        color_height: int,
        detection_cfg: dict[str, Any],
        vision_cfg: dict[str, Any],
    ) -> tuple[float, str]:
        depth_scale_m = detection_cfg.get("depth_scale_m", vision_cfg.get("depth_scale_m"))
        try:
            val = median_bbox_ring_depth_meters(
                depth,
                frame_bbox,
                color_width,
                color_height,
                margin_px=int(detection_cfg.get("frame_depth_margin_px", detection_cfg.get("depth_fallback_margin_px", 30))),
                exclude_padding_px=int(
                    detection_cfg.get(
                        "frame_depth_exclude_padding_px",
                        detection_cfg.get("depth_fallback_exclude_padding_px", 4),
                    )
                ),
                min_samples=int(detection_cfg.get("frame_depth_min_samples", detection_cfg.get("depth_fallback_min_samples", 10))),
                depth_scale_m=depth_scale_m,
            )
            _validate_depth_m(val, detection_cfg, vision_cfg)
            return val, "square_frame_bbox_ring"
        except Exception as ring_exc:
            try:
                val = median_depth_meters(
                    depth,
                    target_u,
                    target_v,
                    color_width,
                    color_height,
                    window_px=int(detection_cfg.get("depth_window_px", vision_cfg.get("depth_window_px", 7))),
                    depth_scale_m=depth_scale_m,
                )
                _validate_depth_m(val, detection_cfg, vision_cfg)
                return val, "square_frame_center"
            except Exception as center_exc:
                raise RuntimeError(
                    f"square frame ring depth failed: {ring_exc}; "
                    f"center depth failed: {center_exc}"
                ) from center_exc

    def _depth_for_blob_fallback(
        self,
        blob_bbox: tuple[int, int, int, int],
        target_u: float,
        target_v: float,
        depth: Any,
        color_width: int,
        color_height: int,
        detection_cfg: dict[str, Any],
        vision_cfg: dict[str, Any],
    ) -> tuple[float, str]:
        """Depth for blob-only fallback when gray_plate locator fails."""
        depth_scale_m = detection_cfg.get("depth_scale_m", vision_cfg.get("depth_scale_m"))
        try:
            val = median_bbox_ring_depth_meters(
                depth,
                blob_bbox,
                color_width,
                color_height,
                margin_px=int(detection_cfg.get("depth_fallback_margin_px", 30)),
                exclude_padding_px=int(detection_cfg.get("depth_fallback_exclude_padding_px", 4)),
                min_samples=int(detection_cfg.get("depth_fallback_min_samples", 10)),
                depth_scale_m=depth_scale_m,
            )
            return val, "blob_fallback_bbox_ring"
        except Exception:
            val = median_depth_meters(
                depth,
                target_u,
                target_v,
                color_width,
                color_height,
                window_px=int(detection_cfg.get("depth_window_px", vision_cfg.get("depth_window_px", 7))),
                depth_scale_m=depth_scale_m,
            )
            return val, "blob_fallback_center"

    def _depth_for_gray_plate(
        self,
        plate_bbox: tuple[int, int, int, int],
        target_u: float,
        target_v: float,
        depth: Any,
        color_width: int,
        color_height: int,
        detection_cfg: dict[str, Any],
        vision_cfg: dict[str, Any],
    ) -> tuple[float, str]:
        depth_scale_m = detection_cfg.get("depth_scale_m", vision_cfg.get("depth_scale_m"))
        try:
            val = median_bbox_ring_depth_meters(
                depth,
                plate_bbox,
                color_width,
                color_height,
                margin_px=int(
                    detection_cfg.get(
                        "plate_depth_margin_px",
                        detection_cfg.get("depth_fallback_margin_px", 30),
                    )
                ),
                exclude_padding_px=int(
                    detection_cfg.get(
                        "plate_depth_exclude_padding_px",
                        detection_cfg.get("depth_fallback_exclude_padding_px", 4),
                    )
                ),
                min_samples=int(
                    detection_cfg.get(
                        "plate_depth_min_samples",
                        detection_cfg.get("depth_fallback_min_samples", 10),
                    )
                ),
                depth_scale_m=depth_scale_m,
            )
            _validate_depth_m(val, detection_cfg, vision_cfg)
            return val, "gray_plate_bbox_ring"
        except Exception as ring_exc:
            try:
                val = median_depth_meters(
                    depth,
                    target_u,
                    target_v,
                    color_width,
                    color_height,
                    window_px=int(detection_cfg.get("depth_window_px", vision_cfg.get("depth_window_px", 7))),
                    depth_scale_m=depth_scale_m,
                )
                _validate_depth_m(val, detection_cfg, vision_cfg)
                return val, "gray_plate_center"
            except Exception as center_exc:
                raise RuntimeError(
                    f"gray plate ring depth failed: {ring_exc}; "
                    f"center depth failed: {center_exc}"
                ) from center_exc

    def _black_blob_debug(
        self,
        color: Any,
        detection_cfg: dict[str, Any],
        vision_cfg: dict[str, Any],
    ) -> dict[str, Any]:
        try:
            blobs = find_color_blobs(
                color,
                rgb_min=_required_rgb(detection_cfg, "rgb_min"),
                rgb_max=_required_rgb(detection_cfg, "rgb_max"),
                min_pixels=int(detection_cfg.get("min_pixels", vision_cfg.get("min_pixels", 20))),
                roi=detection_cfg.get("roi", vision_cfg.get("roi")),
                stride=int(detection_cfg.get("stride", vision_cfg.get("stride", 1))),
            )
            ordered = _ordered_blobs(blobs, detection_cfg)
            blob = ordered[0]
            return {
                "blob_centroid_pixel": [round(float(blob.u), 3), round(float(blob.v), 3)],
                "blob_bbox": list(blob.bbox),
                "blob_pixel_count": int(blob.pixel_count),
                "blob_candidate_count": len(blobs),
            }
        except Exception as exc:
            return {"blob_debug_error": str(exc)}

    def _select_depth_valid_blob(
        self,
        blobs: list[Any],
        depth: Any,
        color_width: int,
        color_height: int,
        detection_cfg: dict[str, Any],
        vision_cfg: dict[str, Any],
    ) -> tuple[Any, float, str, list[str]]:
        errors: list[str] = []
        for idx, blob in enumerate(_ordered_blobs(blobs, detection_cfg)):
            try:
                _validate_blob_geometry(blob, detection_cfg, vision_cfg)
                depth_m, depth_source = _depth_for_blob(
                    blob,
                    depth,
                    color_width,
                    color_height,
                    detection_cfg,
                    vision_cfg,
                )
                _validate_depth_m(depth_m, detection_cfg, vision_cfg)
                return blob, depth_m, depth_source, errors
            except Exception as exc:
                errors.append(
                    f"candidate {idx}: pixels={blob.pixel_count}, "
                    f"bbox={list(blob.bbox)}, error={exc}"
                )
        raise RuntimeError(
            "No color blob candidate passed depth validation; "
            + "; ".join(errors[:5])
        )

    def _merged_detection_config(
        self,
        vision_cfg: dict[str, Any],
        target_cfg: dict[str, Any],
        detection_name: str,
    ) -> dict[str, Any]:
        targets = vision_cfg.get("targets", {})
        named = targets.get(detection_name, {}) if isinstance(targets, dict) else {}
        merged = dict(named)
        if isinstance(target_cfg.get("detection_config"), dict):
            merged.update(target_cfg["detection_config"])
        for key in (
            "fresh_sampling",
            "panel_layout_templates",
            "rgb_min",
            "rgb_max",
            "roi",
            "stride",
            "min_pixels",
            "max_pixels",
            "min_bbox_width_px",
            "max_bbox_width_px",
            "min_bbox_height_px",
            "max_bbox_height_px",
            "min_roi_margin_px",
            "desired_pixel",
            "servo_desired_pixel",
            "depth_window_px",
            "depth_scale_m",
            "depth_fallback",
            "depth_fallback_margin_px",
            "depth_fallback_exclude_padding_px",
            "depth_fallback_min_samples",
            "min_depth_m",
            "max_depth_m",
            "camera_frame",
            "base_frame",
            "alignment_reference",
            "tcp_frame",
            "tcp_offset_m",
            "offset_base_m",
            "rpy_base",
            "workspace_bounds",
            "kind",
            "confidence",
            "locator",
            "panel_layout_template",
            "panel_layout_target_id",
            "frame_rgb_min",
            "frame_rgb_max",
            "frame_min_pixels",
            "frame_min_size_px",
            "frame_max_size_px",
            "frame_aspect_tolerance",
            "frame_edge_tolerance_px",
            "frame_min_edge_score",
            "frame_min_side_coverage",
            "frame_max_fill_ratio",
            "frame_stride",
            "frame_scan_step_px",
            "frame_depth_margin_px",
            "frame_depth_exclude_padding_px",
            "frame_depth_min_samples",
            "knob_center_offset_px",
            "save_debug_images",
            "vision_debug_dir",
            "plate_saturation_max",
            "plate_value_min",
            "plate_value_max",
            "plate_min_area_px",
            "plate_max_area_px",
            "plate_min_width_px",
            "plate_max_width_px",
            "plate_min_height_px",
            "plate_max_height_px",
            "plate_min_aspect_ratio",
            "plate_max_aspect_ratio",
            "plate_close_kernel_px",
            "plate_close_iterations",
            "plate_min_rectangularity",
            "plate_min_mask_coverage",
            "plate_depth_margin_px",
            "plate_depth_exclude_padding_px",
            "plate_depth_min_samples",
        ):
            if key in target_cfg:
                merged[key] = target_cfg[key]
        return merged

    def _alignment_reference(
        self,
        detection_cfg: dict[str, Any],
        base_frame: str,
        camera_frame: str,
        intrinsics: Intrinsics,
    ) -> dict[str, Any] | None:
        mode = str(detection_cfg.get("alignment_reference", "desired_pixel")).strip().lower()
        if mode in ("", "desired_pixel", "servo_desired_pixel", "pixel"):
            return None
        if mode not in ("tcp", "tcp_frame", "tool_frame"):
            raise RuntimeError(f"Unsupported alignment_reference: {mode}")

        tcp_frame = str(
            detection_cfg.get(
                "tcp_frame",
                self._parameter_value("tool_frame", "Link6"),
            )
        )
        if not tcp_frame:
            raise RuntimeError("alignment_reference=tcp_frame requires non-empty tcp_frame")

        tcp_offset_m = _vector3(detection_cfg.get("tcp_offset_m", [0.0, 0.0, 0.0]), "tcp_offset_m")
        tcp_camera = self._lookup_frame_point(camera_frame, tcp_frame, tcp_offset_m)
        tcp_base = self._lookup_frame_point(base_frame, tcp_frame, tcp_offset_m)
        tcp_u, tcp_v = _project_camera_point(tcp_camera, intrinsics, tcp_frame, camera_frame)
        tcp_pixel = [round(tcp_u, 3), round(tcp_v, 3)]
        return {
            "alignment_reference": "tcp_frame",
            "tcp_frame": tcp_frame,
            "tcp_offset_m": [round(float(value), 6) for value in tcp_offset_m],
            "alignment_reference_pixel": tcp_pixel,
            "tcp_center_pixel": tcp_pixel,
            "tcp_center_camera_m": [round(float(value), 6) for value in tcp_camera],
            "tcp_center_base_m": [round(float(value), 6) for value in tcp_base],
        }

    def _parameter_value(self, name: str, default: Any) -> Any:
        try:
            return self.node.get_parameter(name).value
        except Exception:
            return default

    def _lookup_frame_origin(
        self,
        target_frame: str,
        source_frame: str,
    ) -> tuple[float, float, float]:
        return self._lookup_frame_point(target_frame, source_frame, (0.0, 0.0, 0.0))

    def _lookup_frame_point(
        self,
        target_frame: str,
        source_frame: str,
        point_source: tuple[float, float, float],
    ) -> tuple[float, float, float]:
        timeout_s = float(self.node.get_parameter("vision_tf_timeout_s").value)
        transform = self.tf_buffer.lookup_transform(
            target_frame,
            source_frame,
            Time(),
            timeout=Duration(seconds=timeout_s),
        )
        translation = transform.transform.translation
        rotation = transform.transform.rotation
        rx, ry, rz = _rotate_quaternion(
            point_source,
            (rotation.x, rotation.y, rotation.z, rotation.w),
        )
        return (
            float(translation.x) + rx,
            float(translation.y) + ry,
            float(translation.z) + rz,
        )

    def _transform_camera_point(
        self,
        point_camera: Pose,
        base_frame: str,
        camera_frame: str,
    ) -> tuple[float, float, float]:
        timeout_s = float(self.node.get_parameter("vision_tf_timeout_s").value)
        transform = self.tf_buffer.lookup_transform(
            base_frame,
            camera_frame,
            Time(),
            timeout=Duration(seconds=timeout_s),
        )
        translation = transform.transform.translation
        rotation = transform.transform.rotation
        rx, ry, rz = _rotate_quaternion(
            (point_camera.x, point_camera.y, point_camera.z),
            (rotation.x, rotation.y, rotation.z, rotation.w),
        )
        return translation.x + rx, translation.y + ry, translation.z + rz

    def _rotate_camera_vector(
        self,
        vector_camera: tuple[float, float, float],
        base_frame: str,
        camera_frame: str,
    ) -> tuple[float, float, float]:
        timeout_s = float(self.node.get_parameter("vision_tf_timeout_s").value)
        transform = self.tf_buffer.lookup_transform(
            base_frame,
            camera_frame,
            Time(),
            timeout=Duration(seconds=timeout_s),
        )
        rotation = transform.transform.rotation
        return _rotate_quaternion(
            vector_camera,
            (rotation.x, rotation.y, rotation.z, rotation.w),
        )

    def _publish_target_pose(self, target: ElementTarget) -> None:
        msg = PoseStamped()
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.header.frame_id = target.pose.frame
        msg.pose.position.x = target.pose.x
        msg.pose.position.y = target.pose.y
        msg.pose.position.z = target.pose.z
        qx, qy, qz, qw = _rpy_to_quaternion(target.pose.rx, target.pose.ry, target.pose.rz)
        msg.pose.orientation.x = qx
        msg.pose.orientation.y = qy
        msg.pose.orientation.z = qz
        msg.pose.orientation.w = qw
        self.target_pub.publish(msg)


def _ordered_blobs(blobs: list[Any], detection_cfg: dict[str, Any]) -> list[Any]:
    desired_pixel = detection_cfg.get("desired_pixel", detection_cfg.get("servo_desired_pixel"))
    if desired_pixel is None:
        return list(blobs)
    desired_u, desired_v = _pixel2(desired_pixel, "desired_pixel")
    return sorted(
        blobs,
        key=lambda blob: (
            (float(blob.u) - desired_u) * (float(blob.u) - desired_u)
            + (float(blob.v) - desired_v) * (float(blob.v) - desired_v)
        ),
    )


def _project_camera_point(
    point_camera: tuple[float, float, float],
    intrinsics: Intrinsics,
    source_frame: str,
    camera_frame: str,
) -> tuple[float, float]:
    x, y, z = (float(point_camera[0]), float(point_camera[1]), float(point_camera[2]))
    if z <= 0.0:
        raise RuntimeError(
            f"Cannot project {source_frame} into {camera_frame}: z={z:.6f}m is not in front of camera"
        )
    u = intrinsics.fx * x / z + intrinsics.cx
    v = intrinsics.fy * y / z + intrinsics.cy
    return u, v


def _square_frame_candidates_debug(candidates: list[Any], limit: int = 5) -> list[dict[str, Any]]:
    ordered = sorted(
        candidates,
        key=lambda candidate: (
            bool(getattr(candidate, "passed", False)),
            float(getattr(candidate, "edge_score", 0.0)),
            float(getattr(candidate, "side_coverage", 0.0)),
            -float(getattr(candidate, "center_distance_px", 0.0) or 0.0),
        ),
        reverse=True,
    )
    debug: list[dict[str, Any]] = []
    for candidate in ordered[: max(0, int(limit))]:
        center_distance = getattr(candidate, "center_distance_px", None)
        item = {
            "bbox": list(getattr(candidate, "bbox")),
            "center_pixel": [
                round(float(getattr(candidate, "u")), 3),
                round(float(getattr(candidate, "v")), 3),
            ],
            "pixel_count": int(getattr(candidate, "pixel_count")),
            "size_px": round(float(getattr(candidate, "size_px")), 3),
            "edge_score": round(float(getattr(candidate, "edge_score")), 6),
            "side_coverage": round(float(getattr(candidate, "side_coverage")), 6),
            "side_coverages": [
                round(float(value), 6)
                for value in getattr(candidate, "side_coverages", (0.0, 0.0, 0.0, 0.0))
            ],
            "fill_ratio": round(float(getattr(candidate, "fill_ratio")), 6),
            "passed": bool(getattr(candidate, "passed", False)),
            "error": str(getattr(candidate, "error", "")),
        }
        if center_distance is not None:
            item["center_distance_px"] = round(float(center_distance), 3)
        debug.append(item)
    return debug


def _gray_plate_candidates_debug(candidates: list[Any], limit: int = 5) -> list[dict[str, Any]]:
    debug: list[dict[str, Any]] = []
    for candidate in candidates[: max(0, int(limit))]:
        center_distance = getattr(candidate, "center_distance_px", None)
        item = {
            "bbox": list(getattr(candidate, "bbox")),
            "center_pixel": [
                round(float(getattr(candidate, "u")), 3),
                round(float(getattr(candidate, "v")), 3),
            ],
            "corners": [
                [round(float(u), 3), round(float(v), 3)]
                for u, v in getattr(candidate, "corners", [])
            ],
            "width_px": round(float(getattr(candidate, "width_px")), 3),
            "height_px": round(float(getattr(candidate, "height_px")), 3),
            "angle_deg": round(float(getattr(candidate, "angle_deg")), 3),
            "area_px": round(float(getattr(candidate, "area_px")), 3),
            "score": round(float(getattr(candidate, "score")), 6),
            "rectangularity": round(float(getattr(candidate, "rectangularity")), 6),
            "mask_coverage": round(float(getattr(candidate, "mask_coverage")), 6),
            "aspect_ratio": round(float(getattr(candidate, "aspect_ratio")), 6),
            "passed": bool(getattr(candidate, "passed", False)),
            "error": str(getattr(candidate, "error", "")),
        }
        if center_distance is not None:
            item["center_distance_px"] = round(float(center_distance), 3)
        debug.append(item)
    return debug


def _safe_debug_name(value: str) -> str:
    cleaned = []
    for char in value:
        if char.isalnum() or char in ("-", "_"):
            cleaned.append(char)
        else:
            cleaned.append("_")
    return "".join(cleaned) or "target"


def _depth_for_blob(
    blob: Any,
    depth: Any,
    color_width: int,
    color_height: int,
    detection_cfg: dict[str, Any],
    vision_cfg: dict[str, Any],
) -> tuple[float, str]:
    depth_scale_m = detection_cfg.get("depth_scale_m", vision_cfg.get("depth_scale_m"))
    try:
        val = median_depth_meters(
            depth,
            blob.u,
            blob.v,
            color_width,
            color_height,
            window_px=int(
                detection_cfg.get(
                    "depth_window_px",
                    vision_cfg.get("depth_window_px", 7),
                )
            ),
            depth_scale_m=depth_scale_m,
        )
        _validate_depth_m(val, detection_cfg, vision_cfg)
        return (
            val,
            "center",
        )
    except Exception as center_exc:
        fallback = str(detection_cfg.get("depth_fallback", "")).strip().lower()
        if fallback != "bbox_ring":
            raise
        try:
            return (
                median_bbox_ring_depth_meters(
                    depth,
                    blob.bbox,
                    color_width,
                    color_height,
                    margin_px=int(detection_cfg.get("depth_fallback_margin_px", 30)),
                    exclude_padding_px=int(
                        detection_cfg.get("depth_fallback_exclude_padding_px", 4)
                    ),
                    min_samples=int(detection_cfg.get("depth_fallback_min_samples", 10)),
                    depth_scale_m=depth_scale_m,
                ),
                "bbox_ring",
            )
        except Exception as fallback_exc:
            raise RuntimeError(
                f"{center_exc}; fallback bbox_ring failed: {fallback_exc}"
            ) from fallback_exc


def _required_rgb(config: dict[str, Any], key: str) -> list[int]:
    if key not in config:
        raise RuntimeError(f"Vision detection config is missing {key}")
    values = [int(v) for v in config[key]]
    if len(values) != 3:
        raise RuntimeError(f"{key} must contain three RGB values")
    return values


def _rgb_or_fallback(config: dict[str, Any], key: str, fallback_key: str) -> list[int]:
    return _required_rgb(config, key if key in config else fallback_key)


def _validate_depth_m(
    depth_m: float,
    detection_cfg: dict[str, Any],
    vision_cfg: dict[str, Any],
) -> None:
    min_depth = _optional_float(detection_cfg.get("min_depth_m", vision_cfg.get("min_depth_m")))
    max_depth = _optional_float(detection_cfg.get("max_depth_m", vision_cfg.get("max_depth_m")))
    if min_depth is not None and depth_m < min_depth:
        raise RuntimeError(
            f"Detected target depth {depth_m:.3f}m is closer than min_depth_m {min_depth:.3f}m"
        )
    if max_depth is not None and depth_m > max_depth:
        raise RuntimeError(
            f"Detected target depth {depth_m:.3f}m is farther than max_depth_m {max_depth:.3f}m"
        )


def _validate_blob_geometry(
    blob: Any,
    detection_cfg: dict[str, Any],
    vision_cfg: dict[str, Any],
) -> None:
    max_pixels = _optional_float(detection_cfg.get("max_pixels", vision_cfg.get("max_pixels")))
    if max_pixels is not None and blob.pixel_count > max_pixels:
        raise RuntimeError(
            f"Detected blob has {blob.pixel_count} pixels, more than max_pixels {int(max_pixels)}"
        )

    min_u, min_v, max_u, max_v = blob.bbox
    width = int(max_u) - int(min_u) + 1
    height = int(max_v) - int(min_v) + 1
    roi_margin = _optional_float(
        detection_cfg.get("min_roi_margin_px", vision_cfg.get("min_roi_margin_px"))
    )
    roi = detection_cfg.get("roi", vision_cfg.get("roi"))
    if roi_margin is not None and roi is not None:
        roi_values = [int(v) for v in roi]
        if len(roi_values) != 4:
            raise RuntimeError("roi must be [x, y, width, height]")
        roi_x, roi_y, roi_w, roi_h = roi_values
        if (
            min_u - roi_x < roi_margin
            or min_v - roi_y < roi_margin
            or roi_x + roi_w - 1 - max_u < roi_margin
            or roi_y + roi_h - 1 - max_v < roi_margin
        ):
            raise RuntimeError(
                f"Detected blob bbox {list(blob.bbox)} is closer than "
                f"min_roi_margin_px {int(roi_margin)} to ROI edge"
            )
    _validate_bbox_dimension(
        "width",
        width,
        detection_cfg.get("min_bbox_width_px", vision_cfg.get("min_bbox_width_px")),
        detection_cfg.get("max_bbox_width_px", vision_cfg.get("max_bbox_width_px")),
    )
    _validate_bbox_dimension(
        "height",
        height,
        detection_cfg.get("min_bbox_height_px", vision_cfg.get("min_bbox_height_px")),
        detection_cfg.get("max_bbox_height_px", vision_cfg.get("max_bbox_height_px")),
    )


def _validate_bbox_dimension(
    name: str,
    value: int,
    min_value: Any,
    max_value: Any,
) -> None:
    lower = _optional_float(min_value)
    upper = _optional_float(max_value)
    if lower is not None and value < lower:
        raise RuntimeError(
            f"Detected blob bbox {name} {value}px is smaller than min_bbox_{name}_px {int(lower)}"
        )
    if upper is not None and value > upper:
        raise RuntimeError(
            f"Detected blob bbox {name} {value}px is larger than max_bbox_{name}_px {int(upper)}"
        )


def _validate_workspace_bounds(
    pose: Pose,
    detection_cfg: dict[str, Any],
    vision_cfg: dict[str, Any],
) -> None:
    bounds = detection_cfg.get("workspace_bounds", vision_cfg.get("workspace_bounds"))
    if bounds is None:
        return
    if not isinstance(bounds, dict):
        raise RuntimeError("workspace_bounds must be a mapping with x/y/z ranges")
    for axis, value in (("x", pose.x), ("y", pose.y), ("z", pose.z)):
        if axis not in bounds:
            continue
        lower, upper = _range2(bounds[axis], f"workspace_bounds.{axis}")
        if value < lower or value > upper:
            raise RuntimeError(
                f"Detected target {axis}={value:.3f}m is outside workspace "
                f"[{lower:.3f}, {upper:.3f}] in {pose.frame}"
            )


def _optional_float(value: Any) -> float | None:
    if value is None:
        return None
    return float(value)


def _pixel2(value: Any, name: str) -> tuple[float, float]:
    values = [float(v) for v in value]
    if len(values) != 2:
        raise RuntimeError(f"{name} must contain [u, v]")
    return values[0], values[1]


def _range2(value: Any, name: str) -> tuple[float, float]:
    values = [float(v) for v in value]
    if len(values) != 2:
        raise RuntimeError(f"{name} must contain [min, max]")
    if values[0] > values[1]:
        raise RuntimeError(f"{name} min must be <= max")
    return values[0], values[1]


def _as_bool(value: Any) -> bool:
    if isinstance(value, str):
        return value.strip().lower() in ("1", "true", "yes", "on")
    return bool(value)


def _vector3(value: Any, name: str) -> tuple[float, float, float]:
    values = [float(v) for v in value]
    if len(values) != 3:
        raise RuntimeError(f"{name} must contain three numbers")
    return values[0], values[1], values[2]


def _rotate_quaternion(
    vector: tuple[float, float, float],
    quat: tuple[float, float, float, float],
) -> tuple[float, float, float]:
    qx, qy, qz, qw = quat
    norm = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
    if norm <= 0.0:
        raise RuntimeError("Cannot transform point with zero-length quaternion")
    qx, qy, qz, qw = qx / norm, qy / norm, qz / norm, qw / norm
    vx, vy, vz = vector

    tx = 2.0 * (qy * vz - qz * vy)
    ty = 2.0 * (qz * vx - qx * vz)
    tz = 2.0 * (qx * vy - qy * vx)

    rx = vx + qw * tx + (qy * tz - qz * ty)
    ry = vy + qw * ty + (qz * tx - qx * tz)
    rz = vz + qw * tz + (qx * ty - qy * tx)
    return rx, ry, rz


def _rpy_to_quaternion(
    roll: float,
    pitch: float,
    yaw: float,
) -> tuple[float, float, float, float]:
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )
