import unittest
import tempfile
from pathlib import Path
from types import SimpleNamespace

from rg_cabinet_ros2.image_tools import (
    find_color_blob,
    find_color_blobs,
    find_gray_plate,
    find_gray_plate_with_debug,
    find_square_frame,
    find_square_frame_with_debug,
    median_bbox_ring_depth_meters,
    median_depth_meters,
    write_color_image,
    write_gray_plate_debug_images,
    write_square_frame_debug_images,
)


class ImageToolsTests(unittest.TestCase):
    def test_find_color_blob_in_rgb8_image(self):
        image = SimpleNamespace(
            width=3,
            height=3,
            encoding="rgb8",
            step=9,
            data=bytes(
                [
                    0,
                    0,
                    0,
                    0,
                    0,
                    0,
                    0,
                    0,
                    0,
                    0,
                    0,
                    0,
                    255,
                    0,
                    0,
                    255,
                    0,
                    0,
                    0,
                    0,
                    0,
                    255,
                    0,
                    0,
                    255,
                    0,
                    0,
                ]
            ),
        )

        blob = find_color_blob(image, [200, 0, 0], [255, 80, 80], min_pixels=4)

        self.assertEqual(blob.pixel_count, 4)
        self.assertAlmostEqual(blob.u, 1.5)
        self.assertAlmostEqual(blob.v, 1.5)

    def test_find_color_blobs_returns_connected_components(self):
        pixels = [
            (255, 0, 0), (255, 0, 0), (0, 0, 0), (255, 0, 0),
            (255, 0, 0), (255, 0, 0), (0, 0, 0), (0, 0, 0),
            (0, 0, 0), (0, 0, 0), (0, 0, 0), (255, 0, 0),
        ]
        data = bytes(channel for pixel in pixels for channel in pixel)
        image = SimpleNamespace(
            width=4,
            height=3,
            encoding="rgb8",
            step=12,
            data=data,
        )

        blobs = find_color_blobs(image, [200, 0, 0], [255, 80, 80], min_pixels=1)

        self.assertEqual([blob.pixel_count for blob in blobs], [4, 1, 1])
        self.assertEqual(blobs[0].bbox, (0, 0, 1, 1))

    def test_find_square_frame_detects_frame_center(self):
        image = _rgb_image_with_square_frame(
            width=20,
            height=20,
            bbox=(4, 4, 15, 15),
            inner_knob=(8, 8, 11, 11),
        )

        frame = find_square_frame(
            image,
            [0, 0, 0],
            [20, 20, 20],
            min_pixels=20,
            min_size_px=10,
            edge_tolerance_px=1,
            min_edge_score=0.80,
            min_side_coverage=0.90,
            desired_pixel=[10, 10],
        )

        self.assertEqual(frame.bbox, (4, 4, 15, 15))
        self.assertAlmostEqual(frame.u, 9.5)
        self.assertAlmostEqual(frame.v, 9.5)
        self.assertGreaterEqual(frame.edge_score, 0.9)

    def test_find_square_frame_rejects_solid_square(self):
        image = _rgb_image_with_black_bbox(width=20, height=20, bbox=(4, 4, 15, 15))

        with self.assertRaisesRegex(ValueError, "No square frame candidate"):
            find_square_frame(
                image,
                [0, 0, 0],
                [20, 20, 20],
                min_pixels=20,
                min_size_px=10,
                max_fill_ratio=0.70,
            )

    def test_find_square_frame_rejects_missing_side(self):
        image = _rgb_image_with_square_frame(
            width=20,
            height=20,
            bbox=(4, 4, 15, 15),
            missing_side="bottom",
        )

        with self.assertRaisesRegex(ValueError, "side_coverage"):
            find_square_frame(
                image,
                [0, 0, 0],
                [20, 20, 20],
                min_pixels=20,
                min_size_px=10,
                edge_tolerance_px=1,
                min_side_coverage=0.90,
            )

    def test_find_square_frame_accepts_broken_edges(self):
        image = _rgb_image_with_broken_square_frame(width=24, height=24, bbox=(5, 5, 18, 18))

        frame = find_square_frame(
            image,
            [0, 0, 0],
            [20, 20, 20],
            min_pixels=20,
            min_size_px=10,
            edge_tolerance_px=1,
            min_edge_score=0.45,
            min_side_coverage=0.30,
            desired_pixel=[12, 12],
        )

        self.assertEqual(frame.bbox, (5, 5, 18, 18))
        self.assertAlmostEqual(frame.u, 11.5)
        self.assertAlmostEqual(frame.v, 11.5)

    def test_find_square_frame_rejects_lines(self):
        image = _rgb_image_with_lines(width=30, height=30)

        with self.assertRaisesRegex(ValueError, "No square frame candidate"):
            find_square_frame(
                image,
                [0, 0, 0],
                [20, 20, 20],
                min_pixels=20,
                min_size_px=10,
                min_edge_score=0.45,
                min_side_coverage=0.30,
            )

    def test_write_square_frame_debug_images_saves_color_mask_and_overlay(self):
        image = _rgb_image_with_square_frame(width=20, height=20, bbox=(4, 4, 15, 15))
        search = find_square_frame_with_debug(
            image,
            [0, 0, 0],
            [20, 20, 20],
            min_pixels=20,
            min_size_px=10,
            desired_pixel=[10, 10],
        )
        self.assertIsNotNone(search.detection)

        with tempfile.TemporaryDirectory() as tmp:
            paths = write_square_frame_debug_images(
                image,
                Path(tmp) / "knob",
                matching_pixels=search.matching_pixels,
                roi_bounds=search.roi_bounds,
                selected_frame=search.detection,
                candidates=search.candidates,
                desired_pixel=[10, 10],
                blob_center=[9.5, 9.5],
            )
            color_data = Path(paths["color_path"]).read_bytes()
            mask_data = Path(paths["mask_path"]).read_bytes()
            overlay_data = Path(paths["overlay_path"]).read_bytes()

        self.assertTrue(color_data.startswith(b"P6\n20 20\n255\n"))
        self.assertTrue(mask_data.startswith(b"P5\n20 20\n255\n"))
        self.assertTrue(overlay_data.startswith(b"P6\n20 20\n255\n"))

    def test_find_gray_plate_detects_plate_center_with_knob(self):
        image = _rgb_image_with_gray_plate(width=80, height=70, bbox=(25, 18, 56, 51), knob=(38, 28, 44, 42))

        plate = find_gray_plate(
            image,
            roi=[10, 8, 60, 54],
            desired_pixel=[40, 35],
            min_area_px=700,
        )

        self.assertAlmostEqual(plate.u, 40.5, delta=1.0)
        self.assertAlmostEqual(plate.v, 34.5, delta=1.0)
        self.assertGreaterEqual(plate.width_px, 30.0)
        self.assertGreaterEqual(plate.height_px, 30.0)

    def test_find_gray_plate_detects_rotated_plate(self):
        image = _rgb_image_with_rotated_gray_plate(width=90, height=80, center=(45, 40), size=(34, 30), angle_deg=-12)

        plate = find_gray_plate(
            image,
            roi=[15, 12, 60, 56],
            desired_pixel=[45, 40],
            min_area_px=650,
            min_mask_coverage=0.40,
        )

        self.assertAlmostEqual(plate.u, 45.0, delta=2.0)
        self.assertAlmostEqual(plate.v, 40.0, delta=2.0)
        self.assertGreater(plate.area_px, 650)

    def test_find_gray_plate_rejects_colored_buttons_and_black_knob(self):
        image = _rgb_image_with_colored_buttons(width=90, height=60)

        with self.assertRaisesRegex(ValueError, "No gray plate candidate"):
            find_gray_plate(
                image,
                roi=[0, 0, 90, 60],
                desired_pixel=[45, 30],
                min_area_px=500,
            )

    def test_write_gray_plate_debug_images_saves_outputs(self):
        image = _rgb_image_with_gray_plate(width=80, height=70, bbox=(25, 18, 56, 51), knob=(38, 28, 44, 42))
        search = find_gray_plate_with_debug(
            image,
            roi=[10, 8, 60, 54],
            desired_pixel=[40, 35],
            min_area_px=700,
        )
        self.assertIsNotNone(search.detection)

        with tempfile.TemporaryDirectory() as tmp:
            paths = write_gray_plate_debug_images(
                image,
                Path(tmp) / "knob",
                mask=search.mask,
                edges=search.edges,
                roi_bounds=search.roi_bounds,
                selected_plate=search.detection,
                candidates=search.candidates,
                desired_pixel=[40, 35],
                blob_center=[41, 35],
            )
            color_data = Path(paths["color_path"]).read_bytes()
            mask_data = Path(paths["gray_mask_path"]).read_bytes()
            edges_data = Path(paths["edges_path"]).read_bytes()
            overlay_data = Path(paths["overlay_path"]).read_bytes()

        self.assertTrue(color_data.startswith(b"P6\n80 70\n255\n"))
        self.assertTrue(mask_data.startswith(b"P5\n80 70\n255\n"))
        self.assertTrue(edges_data.startswith(b"P5\n80 70\n255\n"))
        self.assertTrue(overlay_data.startswith(b"P6\n80 70\n255\n"))

    def test_median_depth_reads_16uc1_in_meters(self):
        values_mm = [1000, 1000, 1000, 1000, 1200, 1000, 1000, 1000, 1000]
        data = b"".join(value.to_bytes(2, "little") for value in values_mm)
        depth = SimpleNamespace(
            width=3,
            height=3,
            encoding="16UC1",
            step=6,
            is_bigendian=0,
            data=data,
        )

        depth_m = median_depth_meters(
            depth,
            color_u=1.0,
            color_v=1.0,
            color_width=3,
            color_height=3,
            window_px=3,
        )

        self.assertAlmostEqual(depth_m, 1.0)

    def test_median_bbox_ring_depth_uses_surrounding_valid_depth(self):
        values_mm = []
        for v in range(7):
            for u in range(7):
                if 2 <= u <= 4 and 2 <= v <= 4:
                    values_mm.append(0)
                else:
                    values_mm.append(800)
        depth = SimpleNamespace(
            width=7,
            height=7,
            encoding="16UC1",
            step=14,
            is_bigendian=0,
            data=b"".join(value.to_bytes(2, "little") for value in values_mm),
        )

        depth_m = median_bbox_ring_depth_meters(
            depth,
            color_bbox=(2, 2, 4, 4),
            color_width=7,
            color_height=7,
            margin_px=1,
            exclude_padding_px=0,
            min_samples=8,
        )

        self.assertAlmostEqual(depth_m, 0.8)

    def test_write_color_image_saves_rgb8_ppm(self):
        image = SimpleNamespace(
            width=2,
            height=1,
            encoding="rgb8",
            step=6,
            data=bytes([255, 0, 0, 0, 255, 0]),
        )
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "meter.ppm"

            info = write_color_image(image, path)
            data = path.read_bytes()

        self.assertEqual(info["format"], "ppm")
        self.assertEqual(data, b"P6\n2 1\n255\n" + bytes([255, 0, 0, 0, 255, 0]))

    def test_write_color_image_saves_bgr8_as_rgb_ppm(self):
        image = SimpleNamespace(
            width=1,
            height=1,
            encoding="bgr8",
            step=3,
            data=bytes([1, 2, 3]),
        )
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "meter.ppm"

            write_color_image(image, path)
            data = path.read_bytes()

        self.assertEqual(data, b"P6\n1 1\n255\n" + bytes([3, 2, 1]))

    def test_write_color_image_saves_mono8_pgm(self):
        image = SimpleNamespace(
            width=2,
            height=1,
            encoding="mono8",
            step=2,
            data=bytes([10, 20]),
        )
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "meter.pgm"

            info = write_color_image(image, path)
            data = path.read_bytes()

        self.assertEqual(info["format"], "pgm")
        self.assertEqual(data, b"P5\n2 1\n255\n" + bytes([10, 20]))

    def test_write_color_image_rejects_unsupported_encoding(self):
        image = SimpleNamespace(
            width=1,
            height=1,
            encoding="rgba8",
            step=4,
            data=bytes([0, 0, 0, 255]),
        )
        with tempfile.TemporaryDirectory() as tmp:
            with self.assertRaisesRegex(ValueError, "Unsupported meter image encoding"):
                write_color_image(image, Path(tmp) / "meter.ppm")


def _rgb_image_with_black_bbox(width, height, bbox):
    min_u, min_v, max_u, max_v = bbox
    pixels = []
    for v in range(height):
        for u in range(width):
            if min_u <= u <= max_u and min_v <= v <= max_v:
                pixels.append((0, 0, 0))
            else:
                pixels.append((255, 255, 255))
    return _rgb_image(width, height, pixels)


def _rgb_image_with_square_frame(
    width,
    height,
    bbox,
    inner_knob=None,
    missing_side=None,
):
    min_u, min_v, max_u, max_v = bbox
    knob = inner_knob
    pixels = []
    for v in range(height):
        for u in range(width):
            on_top = v == min_v and min_u <= u <= max_u and missing_side != "top"
            on_bottom = v == max_v and min_u <= u <= max_u and missing_side != "bottom"
            on_left = u == min_u and min_v <= v <= max_v and missing_side != "left"
            on_right = u == max_u and min_v <= v <= max_v and missing_side != "right"
            in_knob = (
                knob is not None
                and knob[0] <= u <= knob[2]
                and knob[1] <= v <= knob[3]
            )
            is_frame = on_top or on_bottom or on_left or on_right
            pixels.append((0, 0, 0) if is_frame or in_knob else (255, 255, 255))
    return _rgb_image(width, height, pixels)


def _rgb_image_with_broken_square_frame(width, height, bbox):
    min_u, min_v, max_u, max_v = bbox
    pixels = []
    for v in range(height):
        for u in range(width):
            on_top = v == min_v and min_u <= u <= max_u and (u - min_u) % 3 != 1
            on_bottom = v == max_v and min_u <= u <= max_u and (u - min_u) % 3 != 2
            on_left = u == min_u and min_v <= v <= max_v and (v - min_v) % 3 != 1
            on_right = u == max_u and min_v <= v <= max_v and (v - min_v) % 3 != 2
            pixels.append((0, 0, 0) if on_top or on_bottom or on_left or on_right else (255, 255, 255))
    return _rgb_image(width, height, pixels)


def _rgb_image_with_lines(width, height):
    pixels = []
    for v in range(height):
        for u in range(width):
            on_line = (u == 10 and 4 <= v <= 25) or (v == 20 and 16 <= u <= 27)
            pixels.append((0, 0, 0) if on_line else (255, 255, 255))
    return _rgb_image(width, height, pixels)


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
    return _rgb_image(width, height, pixels)


def _rgb_image_with_rotated_gray_plate(width, height, center, size, angle_deg):
    import cv2
    import numpy as np

    image = np.full((height, width, 3), 255, dtype=np.uint8)
    rect = (tuple(float(v) for v in center), tuple(float(v) for v in size), float(angle_deg))
    box = cv2.boxPoints(rect).astype(np.int32)
    cv2.fillConvexPoly(image, box, (180, 180, 174))
    cv2.circle(image, (int(center[0]), int(center[1])), 5, (35, 35, 35), -1)
    return _rgb_image(width, height, [tuple(int(c) for c in image[v, u]) for v in range(height) for u in range(width)])


def _rgb_image_with_colored_buttons(width, height):
    import cv2
    import numpy as np

    image = np.full((height, width, 3), 255, dtype=np.uint8)
    cv2.circle(image, (25, 30), 10, (210, 20, 20), -1)
    cv2.circle(image, (45, 30), 10, (240, 180, 20), -1)
    cv2.circle(image, (65, 30), 10, (20, 150, 40), -1)
    cv2.circle(image, (45, 12), 7, (20, 20, 20), -1)
    return _rgb_image(width, height, [tuple(int(c) for c in image[v, u]) for v in range(height) for u in range(width)])


def _rgb_image(width, height, pixels):
    return SimpleNamespace(
        width=width,
        height=height,
        encoding="rgb8",
        step=width * 3,
        data=bytes(channel for pixel in pixels for channel in pixel),
    )


if __name__ == "__main__":
    unittest.main()
