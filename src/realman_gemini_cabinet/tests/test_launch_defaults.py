import unittest
from pathlib import Path


class LaunchDefaultsTests(unittest.TestCase):
    def test_hardware_bringup_defaults_to_external_arm_gemini_camera_topics(self):
        launch_path = (
            Path(__file__).resolve().parents[1]
            / "launch"
            / "hardware_bringup.launch.py"
        )
        contents = launch_path.read_text(encoding="utf-8")

        self.assertIn('DeclareLaunchArgument("include_camera", default_value="false")', contents)
        self.assertIn("/camera_gemini/color/image_raw", contents)
        self.assertIn("/camera_gemini/depth/image_raw", contents)
        self.assertIn("/camera_gemini/color/camera_info", contents)
        self.assertIn("camera_gemini_color_optical_frame", contents)


if __name__ == "__main__":
    unittest.main()
