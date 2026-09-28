import unittest
import tempfile
from pathlib import Path
from types import SimpleNamespace

from rg_cabinet.geometry import Pose
from rg_cabinet_ros2.camera_cache import CameraFrameCache
from rg_cabinet_ros2.vision_target_resolver import (
    VisionTargetResolver,
    _validate_blob_geometry,
    _validate_depth_m,
    _validate_workspace_bounds,
)


class VisionTargetResolverTests(unittest.TestCase):
    def test_depth_range_rejects_far_saturated_depth(self):
        with self.assertRaisesRegex(RuntimeError, "farther than max_depth_m"):
            _validate_depth_m(5.0, {}, {"max_depth_m": 1.5})

    def test_workspace_bounds_rejects_impossible_base_pose(self):
        pose = Pose(-5.49, 0.34, 0.0, frame="base_link")

        with self.assertRaisesRegex(RuntimeError, "outside workspace"):
            _validate_workspace_bounds(
                pose,
                {},
                {"workspace_bounds": {"x": [-0.2, 1.0]}},
            )

    def test_valid_depth_and_workspace_pass(self):
        _validate_depth_m(0.6, {}, {"min_depth_m": 0.1, "max_depth_m": 1.5})
        _validate_blob_geometry(
            SimpleNamespace(pixel_count=125, bbox=(10, 20, 24, 34)),
            {
                "min_pixels": 50,
                "max_pixels": 6000,
                "min_bbox_width_px": 5,
                "max_bbox_width_px": 120,
                "min_bbox_height_px": 5,
                "max_bbox_height_px": 140,
            },
            {},
        )
        _validate_blob_geometry(
            SimpleNamespace(pixel_count=500, bbox=(10, 20, 49, 79)),
            {
                "min_pixels": 20,
                "max_pixels": 1000,
                "min_bbox_width_px": 20,
                "max_bbox_width_px": 80,
                "min_bbox_height_px": 20,
                "max_bbox_height_px": 80,
            },
            {},
        )
        _validate_workspace_bounds(
            Pose(0.45, 0.1, 0.4, frame="base_link"),
            {},
            {
                "workspace_bounds": {
                    "x": [-0.2, 1.0],
                    "y": [-0.8, 0.8],
                    "z": [-0.2, 1.2],
                }
            },
        )

    def test_blob_geometry_rejects_large_candidate(self):
        blob = SimpleNamespace(pixel_count=15171, bbox=(472, 417, 729, 489))

        with self.assertRaisesRegex(RuntimeError, "more than max_pixels"):
            _validate_blob_geometry(blob, {"max_pixels": 6000}, {})

        with self.assertRaisesRegex(RuntimeError, "bbox width 258px"):
            _validate_blob_geometry(blob, {"max_bbox_width_px": 120}, {})

    def test_select_depth_valid_blob_skips_far_candidate(self):
        resolver = VisionTargetResolver.__new__(VisionTargetResolver)
        near = SimpleNamespace(u=0.0, v=0.0, pixel_count=12, bbox=(0, 0, 0, 0))
        far = SimpleNamespace(u=1.0, v=0.0, pixel_count=20, bbox=(1, 0, 1, 0))
        values_mm = [600, 5000]
        depth = SimpleNamespace(
            width=2,
            height=1,
            encoding="16UC1",
            step=4,
            is_bigendian=0,
            data=b"".join(value.to_bytes(2, "little") for value in values_mm),
        )

        blob, depth_m, depth_source, errors = resolver._select_depth_valid_blob(
            [far, near],
            depth,
            color_width=2,
            color_height=1,
            detection_cfg={"depth_window_px": 1},
            vision_cfg={"max_depth_m": 1.5},
        )

        self.assertIs(blob, near)
        self.assertAlmostEqual(depth_m, 0.6)
        self.assertEqual(depth_source, "center")
        self.assertEqual(len(errors), 1)

    def test_select_depth_valid_blob_skips_geometry_rejected_candidate(self):
        resolver = VisionTargetResolver.__new__(VisionTargetResolver)
        valid = SimpleNamespace(u=0.0, v=0.0, pixel_count=500, bbox=(0, 0, 39, 39))
        large = SimpleNamespace(u=1.0, v=0.0, pixel_count=15171, bbox=(472, 417, 729, 489))
        values_mm = [300, 300]
        depth = SimpleNamespace(
            width=2,
            height=1,
            encoding="16UC1",
            step=4,
            is_bigendian=0,
            data=b"".join(value.to_bytes(2, "little") for value in values_mm),
        )

        blob, depth_m, depth_source, errors = resolver._select_depth_valid_blob(
            [large, valid],
            depth,
            color_width=2,
            color_height=1,
            detection_cfg={"depth_window_px": 1, "max_bbox_width_px": 120},
            vision_cfg={"max_depth_m": 1.5},
        )

        self.assertIs(blob, valid)
        self.assertAlmostEqual(depth_m, 0.3)
        self.assertEqual(depth_source, "center")
        self.assertIn("bbox width 258px", errors[0])

    def test_depth_fallback_disabled_keeps_center_depth_failure(self):
        resolver = VisionTargetResolver.__new__(VisionTargetResolver)
        blob = SimpleNamespace(u=3.0, v=3.0, pixel_count=9, bbox=(2, 2, 4, 4))
        depth = _depth_image_with_zero_bbox(width=7, height=7, bbox=(2, 2, 4, 4))

        with self.assertRaisesRegex(RuntimeError, "No positive depth values"):
            resolver._select_depth_valid_blob(
                [blob],
                depth,
                color_width=7,
                color_height=7,
                detection_cfg={"depth_window_px": 1},
                vision_cfg={},
            )

    def test_depth_fallback_uses_bbox_ring_depth(self):
        resolver = VisionTargetResolver.__new__(VisionTargetResolver)
        blob = SimpleNamespace(u=3.0, v=3.0, pixel_count=9, bbox=(2, 2, 4, 4))
        depth = _depth_image_with_zero_bbox(width=7, height=7, bbox=(2, 2, 4, 4))

        selected, depth_m, depth_source, errors = resolver._select_depth_valid_blob(
            [blob],
            depth,
            color_width=7,
            color_height=7,
            detection_cfg={
                "depth_window_px": 1,
                "depth_fallback": "bbox_ring",
                "depth_fallback_margin_px": 1,
                "depth_fallback_exclude_padding_px": 0,
                "depth_fallback_min_samples": 8,
            },
            vision_cfg={},
        )

        self.assertIs(selected, blob)
        self.assertAlmostEqual(depth_m, 0.8)
        self.assertEqual(depth_source, "bbox_ring")
        self.assertEqual(errors, [])

    def test_select_depth_valid_blob_prefers_desired_pixel_candidate(self):
        resolver = VisionTargetResolver.__new__(VisionTargetResolver)
        near_desired = SimpleNamespace(u=5.0, v=5.0, pixel_count=10, bbox=(5, 5, 5, 5))
        larger_far = SimpleNamespace(u=0.0, v=0.0, pixel_count=100, bbox=(0, 0, 0, 0))
        values_mm = [800] * 36
        depth = SimpleNamespace(
            width=6,
            height=6,
            encoding="16UC1",
            step=12,
            is_bigendian=0,
            data=b"".join(value.to_bytes(2, "little") for value in values_mm),
        )

        blob, depth_m, depth_source, errors = resolver._select_depth_valid_blob(
            [larger_far, near_desired],
            depth,
            color_width=6,
            color_height=6,
            detection_cfg={"depth_window_px": 1, "desired_pixel": [5, 5]},
            vision_cfg={},
        )

        self.assertIs(blob, near_desired)
        self.assertAlmostEqual(depth_m, 0.8)
        self.assertEqual(depth_source, "center")
        self.assertEqual(errors, [])

    def test_resolve_records_bbox_ring_depth_source(self):
        color = _rgb_image_with_black_bbox(width=7, height=7, bbox=(2, 2, 4, 4))
        depth = _depth_image_with_zero_bbox(width=7, height=7, bbox=(2, 2, 4, 4))
        camera_info = SimpleNamespace(k=[100.0, 0.0, 3.0, 0.0, 100.0, 3.0, 0.0, 0.0, 1.0])
        resolver = VisionTargetResolver(
            _FakeNode(),
            CameraFrameCache(color=color, depth=depth, camera_info=camera_info),
            _FakeTfBuffer(),
        )
        resolver._publish_target_pose = lambda _target: None

        target = resolver.resolve(
            {
                "vision": {
                    "targets": {
                        "middle_black_knob": {
                            "rgb_min": [0, 0, 0],
                            "rgb_max": [10, 10, 10],
                            "min_pixels": 1,
                            "depth_window_px": 1,
                            "depth_fallback": "bbox_ring",
                            "depth_fallback_margin_px": 1,
                            "depth_fallback_exclude_padding_px": 0,
                            "depth_fallback_min_samples": 8,
                            "desired_pixel": [3, 3],
                        }
                    }
                }
            },
            {
                "name": "middle_black_knob",
                "skill": "knob_camera_servo",
                "target": {
                    "source": "camera",
                    "detection": "middle_black_knob",
                    "kind": "knob",
                },
            },
        )

        self.assertEqual(target.details["depth_source"], "bbox_ring")
        self.assertEqual(target.details["pixel"], [3.0, 3.0])

    def test_resolve_square_frame_locator_uses_frame_center_offset(self):
        color = _rgb_image_with_square_frame(
            width=11,
            height=11,
            bbox=(2, 2, 8, 8),
            inner_knob=(5, 5, 5, 5),
        )
        depth = _depth_image_with_zero_bbox(width=11, height=11, bbox=(2, 2, 8, 8))
        camera_info = SimpleNamespace(k=[100.0, 0.0, 5.0, 0.0, 100.0, 5.0, 0.0, 0.0, 1.0])
        resolver = VisionTargetResolver(
            _FakeNode(),
            CameraFrameCache(color=color, depth=depth, camera_info=camera_info),
            _FakeTfBuffer(),
        )
        resolver._publish_target_pose = lambda _target: None

        target = resolver.resolve(
            {
                "vision": {
                    "targets": {
                        "middle_black_knob": {
                            "kind": "knob",
                            "locator": "square_frame",
                            "rgb_min": [0, 0, 0],
                            "rgb_max": [10, 10, 10],
                            "frame_rgb_min": [0, 0, 0],
                            "frame_rgb_max": [10, 10, 10],
                            "min_pixels": 1,
                            "frame_min_pixels": 10,
                            "frame_min_size_px": 5,
                            "frame_edge_tolerance_px": 0,
                            "frame_min_edge_score": 0.95,
                            "frame_min_side_coverage": 0.95,
                            "frame_depth_margin_px": 1,
                            "frame_depth_exclude_padding_px": 0,
                            "frame_depth_min_samples": 20,
                            "desired_pixel": [5, 5],
                            "knob_center_offset_px": [1, -1],
                            "save_debug_images": False,
                        }
                    }
                }
            },
            {
                "name": "middle_black_knob",
                "skill": "knob_camera_servo",
                "target": {
                    "source": "camera",
                    "detection": "middle_black_knob",
                    "kind": "knob",
                },
            },
        )

        self.assertEqual(target.details["locator"], "square_frame")
        self.assertEqual(target.details["pixel"], [6.0, 4.0])
        self.assertEqual(target.details["frame_center_pixel"], [5.0, 5.0])
        self.assertEqual(target.details["frame_bbox"], [2, 2, 8, 8])
        self.assertEqual(target.details["depth_source"], "square_frame_bbox_ring")
        self.assertAlmostEqual(target.details["frame_edge_score"], 1.0)
        self.assertEqual(target.details["blob_centroid_pixel"], [5.0, 5.0])
        self.assertEqual(target.details["pixel_error_base_m"], [0.008, -0.008, 0.0])

    def test_square_frame_failure_records_candidates_and_debug_images(self):
        color = _rgb_image_with_lines(width=30, height=30)
        depth = _depth_image_with_zero_bbox(width=30, height=30, bbox=(10, 4, 10, 25))
        camera_info = SimpleNamespace(k=[100.0, 0.0, 15.0, 0.0, 100.0, 15.0, 0.0, 0.0, 1.0])
        with tempfile.TemporaryDirectory() as tmp:
            resolver = VisionTargetResolver(
                _FakeNode(),
                CameraFrameCache(color=color, depth=depth, camera_info=camera_info),
                _FakeTfBuffer(),
            )
            resolver._publish_target_pose = lambda _target: None

            with self.assertRaisesRegex(RuntimeError, "No square frame candidate"):
                resolver.resolve(
                    {
                        "vision": {
                            "vision_debug_dir": tmp,
                            "save_debug_images": True,
                            "targets": {
                                "middle_black_knob": {
                                    "kind": "knob",
                                    "locator": "square_frame",
                                    "rgb_min": [0, 0, 0],
                                    "rgb_max": [10, 10, 10],
                                    "frame_rgb_min": [0, 0, 0],
                                    "frame_rgb_max": [10, 10, 10],
                                    "min_pixels": 1,
                                    "frame_min_pixels": 10,
                                    "frame_min_size_px": 8,
                                    "frame_max_size_px": 20,
                                    "frame_min_edge_score": 0.45,
                                    "frame_min_side_coverage": 0.30,
                                    "desired_pixel": [15, 15],
                                }
                            },
                        }
                    },
                    {
                        "name": "middle_black_knob",
                        "skill": "knob_camera_servo",
                        "target": {
                            "source": "camera",
                            "detection": "middle_black_knob",
                            "kind": "knob",
                        },
                    },
                )

            debug = resolver.last_target_debug
            self.assertIsNotNone(debug)
            self.assertIn("frame_candidates", debug)
            self.assertIn("debug_images", debug)
            for key in ("color_path", "mask_path", "overlay_path"):
                self.assertTrue(Path(debug["debug_images"][key]).exists())

    def test_resolve_gray_plate_locator_uses_plate_center_offset(self):
        color = _rgb_image_with_gray_plate(
            width=80,
            height=70,
            bbox=(25, 18, 56, 51),
            knob=(38, 28, 44, 42),
        )
        depth = _depth_image_with_zero_bbox(width=80, height=70, bbox=(25, 18, 56, 51))
        camera_info = SimpleNamespace(k=[100.0, 0.0, 40.0, 0.0, 100.0, 35.0, 0.0, 0.0, 1.0])
        resolver = VisionTargetResolver(
            _FakeNode(),
            CameraFrameCache(color=color, depth=depth, camera_info=camera_info),
            _FakeTfBuffer(),
        )
        resolver._publish_target_pose = lambda _target: None

        target = resolver.resolve(
            {
                "vision": {
                    "targets": {
                        "middle_black_knob": {
                            "kind": "knob",
                            "locator": "gray_plate",
                            "rgb_min": [0, 0, 0],
                            "rgb_max": [80, 80, 80],
                            "roi": [10, 8, 60, 54],
                            "plate_min_area_px": 700,
                            "plate_depth_margin_px": 1,
                            "plate_depth_exclude_padding_px": 0,
                            "plate_depth_min_samples": 20,
                            "desired_pixel": [40, 35],
                            "knob_center_offset_px": [1, -1],
                            "save_debug_images": False,
                        }
                    }
                }
            },
            {
                "name": "middle_black_knob",
                "skill": "knob_camera_servo",
                "target": {
                    "source": "camera",
                    "detection": "middle_black_knob",
                    "kind": "knob",
                },
            },
        )

        self.assertEqual(target.details["locator"], "gray_plate")
        self.assertAlmostEqual(target.details["pixel"][0], 41.5, delta=1.0)
        self.assertAlmostEqual(target.details["pixel"][1], 33.5, delta=1.0)
        self.assertAlmostEqual(target.details["plate_center_pixel"][0], 40.5, delta=1.0)
        self.assertAlmostEqual(target.details["plate_center_pixel"][1], 34.5, delta=1.0)
        self.assertEqual(target.details["depth_source"], "gray_plate_bbox_ring")
        self.assertIn("plate_score", target.details)
        self.assertIn("blob_centroid_pixel", target.details)

    def test_resolve_gray_plate_tcp_reference_uses_tf_alignment_error(self):
        color = _rgb_image_with_gray_plate(
            width=80,
            height=70,
            bbox=(25, 18, 56, 51),
            knob=(38, 28, 44, 42),
        )
        depth = _depth_image_with_zero_bbox(width=80, height=70, bbox=(25, 18, 56, 51))
        camera_info = SimpleNamespace(k=[100.0, 0.0, 40.0, 0.0, 100.0, 35.0, 0.0, 0.0, 1.0])
        resolver = VisionTargetResolver(
            _FakeNode(),
            CameraFrameCache(color=color, depth=depth, camera_info=camera_info),
            _FakeTfBuffer(
                {
                    ("camera_color_optical_frame", "gripper_base_link"): (0.016, -0.008, 0.8),
                    ("base_link", "gripper_base_link"): (0.001, -0.010, 0.700),
                }
            ),
        )
        resolver._publish_target_pose = lambda _target: None

        target = resolver.resolve(
            {
                "vision": {
                    "targets": {
                        "middle_black_knob": {
                            "kind": "knob",
                            "locator": "gray_plate",
                            "alignment_reference": "tcp_frame",
                            "tcp_frame": "gripper_base_link",
                            "tcp_offset_m": [0.0, 0.0, 0.02],
                            "rgb_min": [0, 0, 0],
                            "rgb_max": [80, 80, 80],
                            "roi": [10, 8, 60, 54],
                            "plate_min_area_px": 700,
                            "plate_depth_margin_px": 1,
                            "plate_depth_exclude_padding_px": 0,
                            "plate_depth_min_samples": 20,
                            "desired_pixel": [40, 35],
                            "save_debug_images": False,
                        }
                    }
                }
            },
            {
                "name": "middle_black_knob",
                "skill": "knob_camera_servo",
                "target": {
                    "source": "camera",
                    "detection": "middle_black_knob",
                    "kind": "knob",
                },
            },
        )

        self.assertEqual(target.details["alignment_reference"], "tcp_frame")
        self.assertEqual(target.details["tcp_frame"], "gripper_base_link")
        self.assertEqual(target.details["tcp_offset_m"], [0.0, 0.0, 0.02])
        self.assertEqual(target.details["tcp_center_pixel"], [41.951, 34.024])
        self.assertEqual(target.details["alignment_reference_pixel"], [41.951, 34.024])
        self.assertAlmostEqual(target.details["plate_center_base_m"][0], 0.004, delta=0.012)
        self.assertAlmostEqual(target.details["plate_center_base_m"][1], -0.004, delta=0.012)
        self.assertAlmostEqual(target.details["alignment_error_base_m"][0], 0.003, delta=0.012)
        self.assertAlmostEqual(target.details["alignment_error_base_m"][1], 0.006, delta=0.012)
        self.assertIn("pixel_error_base_m", target.details)

    def test_resolve_tcp_reference_fails_when_tf_missing(self):
        color = _rgb_image_with_gray_plate(
            width=80,
            height=70,
            bbox=(25, 18, 56, 51),
            knob=(38, 28, 44, 42),
        )
        depth = _depth_image_with_zero_bbox(width=80, height=70, bbox=(25, 18, 56, 51))
        camera_info = SimpleNamespace(k=[100.0, 0.0, 40.0, 0.0, 100.0, 35.0, 0.0, 0.0, 1.0])
        resolver = VisionTargetResolver(
            _FakeNode(),
            CameraFrameCache(color=color, depth=depth, camera_info=camera_info),
            _FakeTfBuffer(error=RuntimeError("tf unavailable")),
        )

        with self.assertRaisesRegex(RuntimeError, "tf unavailable"):
            resolver.resolve(
                {
                    "vision": {
                        "targets": {
                            "middle_black_knob": {
                                "kind": "knob",
                                "locator": "gray_plate",
                                "alignment_reference": "tcp_frame",
                                "tcp_frame": "gripper_base_link",
                                "rgb_min": [0, 0, 0],
                                "rgb_max": [80, 80, 80],
                                "roi": [10, 8, 60, 54],
                                "plate_min_area_px": 700,
                                "save_debug_images": False,
                            }
                        }
                    }
                },
                {
                    "name": "middle_black_knob",
                    "skill": "knob_camera_servo",
                    "target": {
                        "source": "camera",
                        "detection": "middle_black_knob",
                        "kind": "knob",
                    },
                },
            )

    def test_gray_plate_failure_records_candidates_and_debug_images(self):
        color = _rgb_image_with_colored_buttons(width=90, height=60)
        depth = _depth_image_with_zero_bbox(width=90, height=60, bbox=(0, 0, 0, 0))
        camera_info = SimpleNamespace(k=[100.0, 0.0, 45.0, 0.0, 100.0, 30.0, 0.0, 0.0, 1.0])
        with tempfile.TemporaryDirectory() as tmp:
            resolver = VisionTargetResolver(
                _FakeNode(),
                CameraFrameCache(color=color, depth=depth, camera_info=camera_info),
                _FakeTfBuffer(),
            )
            resolver._publish_target_pose = lambda _target: None

            with self.assertRaisesRegex(RuntimeError, "No gray plate candidate"):
                resolver.resolve(
                    {
                        "vision": {
                            "vision_debug_dir": tmp,
                            "save_debug_images": True,
                            "targets": {
                                "middle_black_knob": {
                                    "kind": "knob",
                                    "locator": "gray_plate",
                                    "rgb_min": [0, 0, 0],
                                    "rgb_max": [80, 80, 80],
                                    "roi": [0, 0, 90, 60],
                                    "plate_min_area_px": 500,
                                    "desired_pixel": [45, 30],
                                }
                            },
                        }
                    },
                    {
                        "name": "middle_black_knob",
                        "skill": "knob_camera_servo",
                        "target": {
                            "source": "camera",
                            "detection": "middle_black_knob",
                            "kind": "knob",
                        },
                    },
                )

            debug = resolver.last_target_debug
            self.assertIsNotNone(debug)
            self.assertIn("plate_candidates", debug)
            self.assertIn("debug_images", debug)
            for key in ("color_path", "gray_mask_path", "edges_path", "overlay_path"):
                self.assertTrue(Path(debug["debug_images"][key]).exists())


class _FakeNode:
    def create_publisher(self, *_args):
        return SimpleNamespace(publish=lambda _msg: None)

    def get_parameter(self, name):
        values = {
            "vision_enabled": True,
            "camera_frame": "camera_color_optical_frame",
            "base_frame": "base_link",
            "vision_tf_timeout_s": 0.1,
            "tool_frame": "Link6",
        }
        return SimpleNamespace(value=values[name])

    def get_clock(self):
        return SimpleNamespace(now=lambda: SimpleNamespace(to_msg=lambda: SimpleNamespace()))


class _FakeTfBuffer:
    def __init__(self, transforms=None, error=None):
        self.transforms = transforms or {}
        self.error = error

    def lookup_transform(self, *_args, **_kwargs):
        if self.error is not None:
            raise self.error
        target_frame = _args[0]
        source_frame = _args[1]
        translation = self.transforms.get((target_frame, source_frame), (0.0, 0.0, 0.0))
        return SimpleNamespace(
            transform=SimpleNamespace(
                translation=SimpleNamespace(x=translation[0], y=translation[1], z=translation[2]),
                rotation=SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0),
            )
        )


def _rgb_image_with_black_bbox(width, height, bbox):
    min_u, min_v, max_u, max_v = bbox
    pixels = []
    for v in range(height):
        for u in range(width):
            if min_u <= u <= max_u and min_v <= v <= max_v:
                pixels.append((0, 0, 0))
            else:
                pixels.append((255, 255, 255))
    return SimpleNamespace(
        width=width,
        height=height,
        encoding="rgb8",
        step=width * 3,
        data=bytes(channel for pixel in pixels for channel in pixel),
        header=SimpleNamespace(frame_id="camera_color_optical_frame"),
    )


def _rgb_image_with_square_frame(width, height, bbox, inner_knob=None):
    min_u, min_v, max_u, max_v = bbox
    pixels = []
    for v in range(height):
        for u in range(width):
            on_frame = (
                (v in (min_v, max_v) and min_u <= u <= max_u)
                or (u in (min_u, max_u) and min_v <= v <= max_v)
            )
            on_knob = (
                inner_knob is not None
                and inner_knob[0] <= u <= inner_knob[2]
                and inner_knob[1] <= v <= inner_knob[3]
            )
            pixels.append((0, 0, 0) if on_frame or on_knob else (255, 255, 255))
    return SimpleNamespace(
        width=width,
        height=height,
        encoding="rgb8",
        step=width * 3,
        data=bytes(channel for pixel in pixels for channel in pixel),
        header=SimpleNamespace(frame_id="camera_color_optical_frame"),
    )


def _rgb_image_with_lines(width, height):
    pixels = []
    for v in range(height):
        for u in range(width):
            on_line = (u == 10 and 4 <= v <= 25) or (v == 20 and 16 <= u <= 27)
            pixels.append((0, 0, 0) if on_line else (255, 255, 255))
    return SimpleNamespace(
        width=width,
        height=height,
        encoding="rgb8",
        step=width * 3,
        data=bytes(channel for pixel in pixels for channel in pixel),
        header=SimpleNamespace(frame_id="camera_color_optical_frame"),
    )


def _rgb_image_with_gray_plate(width, height, bbox, knob=None):
    min_u, min_v, max_u, max_v = bbox
    pixels = []
    for v in range(height):
        for u in range(width):
            in_plate = min_u <= u <= max_u and min_v <= v <= max_v
            on_knob = knob is not None and knob[0] <= u <= knob[2] and knob[1] <= v <= knob[3]
            if on_knob:
                pixels.append((35, 35, 35))
            elif in_plate:
                shade = 178 + ((u + v) % 9)
                pixels.append((shade, shade, shade - 6))
            else:
                pixels.append((255, 255, 255))
    return SimpleNamespace(
        width=width,
        height=height,
        encoding="rgb8",
        step=width * 3,
        data=bytes(channel for pixel in pixels for channel in pixel),
        header=SimpleNamespace(frame_id="camera_color_optical_frame"),
    )


def _rgb_image_with_colored_buttons(width, height):
    import cv2
    import numpy as np

    image = np.full((height, width, 3), 255, dtype=np.uint8)
    cv2.circle(image, (25, 30), 10, (210, 20, 20), -1)
    cv2.circle(image, (45, 30), 10, (240, 180, 20), -1)
    cv2.circle(image, (65, 30), 10, (20, 150, 40), -1)
    cv2.circle(image, (45, 12), 7, (20, 20, 20), -1)
    return SimpleNamespace(
        width=width,
        height=height,
        encoding="rgb8",
        step=width * 3,
        data=bytes(int(channel) for v in range(height) for u in range(width) for channel in image[v, u]),
        header=SimpleNamespace(frame_id="camera_color_optical_frame"),
    )


def _depth_image_with_zero_bbox(width, height, bbox):
    min_u, min_v, max_u, max_v = bbox
    values_mm = []
    for v in range(height):
        for u in range(width):
            values_mm.append(0 if min_u <= u <= max_u and min_v <= v <= max_v else 800)
    return SimpleNamespace(
        width=width,
        height=height,
        encoding="16UC1",
        step=width * 2,
        is_bigendian=0,
        data=b"".join(value.to_bytes(2, "little") for value in values_mm),
    )


if __name__ == "__main__":
    unittest.main()
