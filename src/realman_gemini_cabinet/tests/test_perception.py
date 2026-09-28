import unittest

from rg_cabinet.adapters.camera import RGBDFrame
from rg_cabinet.geometry import Intrinsics, Pose
from rg_cabinet.perception import (
    ColorBlobElementDetector,
    ColorRange,
    DetectionSpec,
    ElementNotFound,
)


class PerceptionTests(unittest.TestCase):
    def test_color_blob_backprojects_depth(self):
        color = [
            [(0, 0, 0), (0, 0, 0), (0, 0, 0)],
            [(0, 0, 0), (255, 0, 0), (255, 0, 0)],
            [(0, 0, 0), (255, 0, 0), (255, 0, 0)],
        ]
        depth = [
            [1.0, 1.0, 1.0],
            [1.0, 1.0, 1.0],
            [1.0, 1.0, 1.0],
        ]
        frame = RGBDFrame(
            color=color,
            depth=depth,
            intrinsics=Intrinsics(fx=100.0, fy=100.0, cx=1.0, cy=1.0),
            camera_pose_in_base=Pose(0.5, 0.0, 0.2),
        )
        spec = DetectionSpec(
            name="red_button",
            kind="button",
            color_range=ColorRange((200, 0, 0), (255, 80, 80)),
            min_pixels=4,
        )

        target = ColorBlobElementDetector().detect(frame, spec)

        self.assertEqual(target.name, "red_button")
        self.assertAlmostEqual(target.pose.x, 0.505)
        self.assertAlmostEqual(target.pose.y, 0.005)
        self.assertAlmostEqual(target.pose.z, 1.2)

    def test_missing_blob_raises(self):
        frame = RGBDFrame(
            color=[[(0, 0, 0)]],
            depth=[[1.0]],
            intrinsics=Intrinsics(fx=100.0, fy=100.0, cx=0.0, cy=0.0),
        )
        spec = DetectionSpec(
            name="blue_button",
            kind="button",
            color_range=ColorRange((0, 0, 200), (80, 80, 255)),
        )
        with self.assertRaises(ElementNotFound):
            ColorBlobElementDetector().detect(frame, spec)


if __name__ == "__main__":
    unittest.main()
