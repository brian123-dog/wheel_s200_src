import math
import unittest

from rg_cabinet.geometry import Pose


class GeometryTests(unittest.TestCase):
    def test_translated_local_without_rotation_matches_base_axis(self):
        pose = Pose(1.0, 2.0, 3.0)

        shifted = pose.translated_local("z", 0.1)

        self.assertAlmostEqual(shifted.x, 1.0)
        self.assertAlmostEqual(shifted.y, 2.0)
        self.assertAlmostEqual(shifted.z, 3.1)

    def test_translated_local_uses_pose_orientation(self):
        pose = Pose(0.0, 0.0, 0.0, rx=math.pi)

        shifted = pose.translated_local("z", 0.1)

        self.assertAlmostEqual(shifted.x, 0.0, places=6)
        self.assertAlmostEqual(shifted.y, 0.0, places=6)
        self.assertAlmostEqual(shifted.z, -0.1, places=6)


if __name__ == "__main__":
    unittest.main()
