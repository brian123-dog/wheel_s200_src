import unittest
from pathlib import Path

from rg_cabinet.adapters.mock import MockRobotAdapter
from rg_cabinet.config import load_task_config
from rg_cabinet.geometry import Pose
from rg_cabinet.perception import ElementTarget
from rg_cabinet.task_runner import run_sequence
from rg_cabinet.verification import MockStateVerifier


class Ros2ConfigTests(unittest.TestCase):
    def test_ros2_mock_demo_config_runs_with_core_runner(self):
        config_path = (
            Path(__file__).resolve().parents[1]
            / "config"
            / "ros2_mock_demo.yaml"
        )
        config = load_task_config(config_path)
        robot = MockRobotAdapter(contact_after_servo_calls=2)
        verifier = MockStateVerifier(config.get("mock_states", {}))

        results = run_sequence(config, robot, verifier=verifier)

        self.assertEqual(len(results), 4)
        self.assertTrue(all(item["success"] for item in results))
        self.assertGreater(len(robot.commands), 0)

    def test_run_sequence_uses_target_resolver_when_requested(self):
        config = {
            "sequence": [
                {
                    "name": "camera_button",
                    "skill": "button",
                    "target": {"source": "camera", "kind": "button"},
                    "params": {"contact": {"max_contact_travel_m": 0.001}},
                }
            ]
        }
        robot = MockRobotAdapter(contact_after_servo_calls=1)

        def resolver(step):
            from rg_cabinet.geometry import Pose
            from rg_cabinet.perception import ElementTarget

            self.assertEqual(step["name"], "camera_button")
            return ElementTarget("camera_button", "button", Pose(0.1, 0.2, 0.3))

        results = run_sequence(config, robot, target_resolver=resolver)

        self.assertTrue(results[0]["success"])
        self.assertEqual(results[0]["name"], "camera_button")

    def test_run_sequence_passes_target_refresher_to_knob_camera_servo(self):
        config = {
            "sequence": [
                {
                    "name": "middle_black_knob",
                    "skill": "knob_camera_servo",
                    "target": {"source": "camera", "kind": "knob"},
                    "params": {
                        "actuation_rpy_base": None,
                        "approach_distance_m": 0.0,
                        "retract_m": 0.0,
                        "turn_angle_rad": 0.0,
                        "align_settle_s": 0.0,
                        "contact": {"max_contact_travel_m": 0.0},
                    },
                }
            ]
        }
        robot = MockRobotAdapter(contact_after_servo_calls=1)
        resolver_calls = []

        def resolver(step):
            resolver_calls.append(step["name"])
            if len(resolver_calls) == 1:
                details = {
                    "pixel": [600.0, 330.0],
                    "desired_pixel": [320.0, 215.0],
                    "pixel_error_base_m": [0.0, 0.02, 0.0],
                }
            else:
                details = {
                    "pixel": [320.0, 215.0],
                    "desired_pixel": [320.0, 215.0],
                    "pixel_error_base_m": [0.0, 0.0, 0.0],
                }
            return ElementTarget(
                step["name"],
                step["target"]["kind"],
                Pose(0.3, 0.0, 0.4, frame="base_link"),
                details=details,
            )

        results = run_sequence(config, robot, target_resolver=resolver)

        self.assertTrue(results[0]["success"])
        self.assertEqual(resolver_calls, ["middle_black_knob", "middle_black_knob"])
        self.assertEqual(len(results[0]["details"]["alignment_iterations"]), 1)

    def test_ros2_qr_vision_demo_config_uses_camera_targets(self):
        config_path = (
            Path(__file__).resolve().parents[1]
            / "config"
            / "ros2_qr_vision_cabinet_demo.yaml"
        )
        config = load_task_config(config_path)
        self.assertEqual(
            [step["skill"] for step in config["sequence"]],
            ["button", "toggle", "knob", "read_meter"],
        )
        self.assertTrue(
            all(step["target"]["source"] == "camera" for step in config["sequence"][:3])
        )
        self.assertEqual(config["sequence"][3]["name"], "panel_meter")
        self.assertNotIn("target", config["sequence"][3])

        robot = MockRobotAdapter(contact_after_servo_calls=2)
        verifier = MockStateVerifier(config.get("mock_states", {}))
        resolved_steps = []

        def resolver(step):
            resolved_steps.append(step["name"])
            return ElementTarget(
                step["name"],
                step["target"]["kind"],
                Pose(0.3, 0.0, 0.4, 3.1416, 0.0, 0.0, frame="base_link"),
            )

        results = run_sequence(config, robot, verifier=verifier, target_resolver=resolver)

        self.assertEqual(len(results), 4)
        self.assertEqual(resolved_steps, ["start_button", "mode_toggle", "selector_knob"])
        self.assertEqual(results[3]["name"], "panel_meter")
        self.assertEqual(results[3]["skill"], "read_meter")
        self.assertTrue(all(item["success"] for item in results))

    def test_ros2_qr_named_panel_demo_config_uses_named_panel_targets(self):
        config_path = (
            Path(__file__).resolve().parents[1]
            / "config"
            / "ros2_qr_named_panel_demo.yaml"
        )
        config = load_task_config(config_path)

        self.assertEqual(
            [step["name"] for step in config["sequence"]],
            [
                "upper_meters",
                "middle_black_knob",
                "rightmost_green_rocker",
                "lower_left_red_button",
            ],
        )
        self.assertEqual(
            [step["skill"] for step in config["sequence"]],
            ["read_meter", "knob_camera_servo", "toggle", "button"],
        )
        self.assertEqual(
            [
                step["observation_pose"]["qr_name"]
                for step in config["sequence"]
            ],
            [
                "upper_meters",
                "middle_black_knob",
                "rightmost_green_rocker",
                "lower_left_red_button",
            ],
        )
        knob_cfg = config["vision"]["targets"]["middle_black_knob"]
        knob_step = config["sequence"][1]
        self.assertEqual(knob_cfg["roi"], [220, 120, 200, 140])
        self.assertEqual(knob_cfg["locator"], "gray_plate")
        self.assertEqual(knob_cfg["alignment_reference"], "tcp_frame")
        self.assertEqual(knob_cfg["tcp_frame"], "gripper_base_link")
        self.assertEqual(knob_cfg["tcp_offset_m"], [0.0, 0.0, 0.12])
        self.assertEqual(knob_cfg["desired_pixel"], [320, 215])
        self.assertEqual(knob_cfg["knob_center_offset_px"], [0.0, 0.0])
        self.assertEqual(knob_cfg["plate_saturation_max"], 80)
        self.assertEqual(knob_cfg["plate_value_min"], 70)
        self.assertEqual(knob_cfg["plate_value_max"], 235)
        self.assertEqual(knob_cfg["plate_min_area_px"], 700)
        self.assertEqual(knob_cfg["plate_max_area_px"], 6000)
        self.assertEqual(knob_cfg["plate_min_width_px"], 25)
        self.assertEqual(knob_cfg["plate_max_width_px"], 100)
        self.assertEqual(knob_cfg["plate_min_height_px"], 25)
        self.assertEqual(knob_cfg["plate_max_height_px"], 100)
        self.assertEqual(knob_cfg["plate_min_rectangularity"], 0.55)
        self.assertEqual(knob_cfg["plate_min_mask_coverage"], 0.45)
        self.assertEqual(knob_cfg["frame_min_size_px"], 18)
        self.assertEqual(knob_cfg["frame_max_size_px"], 120)
        self.assertEqual(knob_cfg["frame_edge_tolerance_px"], 2)
        self.assertEqual(knob_cfg["frame_min_side_coverage"], 0.30)
        self.assertEqual(knob_cfg["frame_scan_step_px"], 1)
        self.assertTrue(config["vision"]["save_debug_images"])
        self.assertEqual(config["vision"]["vision_debug_dir"], "vision_debug")
        self.assertLessEqual(knob_cfg["workspace_bounds"]["x"][0], -1.01)
        self.assertEqual(knob_cfg["depth_fallback"], "bbox_ring")
        self.assertEqual(knob_cfg["depth_fallback_margin_px"], 30)
        self.assertEqual(knob_step["params"]["max_align_iterations"], 6)
        self.assertLessEqual(knob_step["params"]["max_align_step_m"], 0.015)
        self.assertEqual(knob_step["params"]["align_tolerance_px"], 20.0)
        self.assertEqual(knob_step["params"]["proceed_tolerance_px"], 20.0)
        self.assertEqual(knob_step["params"]["approach_distance_m"], 0.060)
        self.assertEqual(knob_step["params"]["open_loop_advance_m"], 0.085)
        self.assertEqual(
            knob_step["params"]["open_loop_advance_speed_mps"],
            0.004,
        )
        self.assertEqual(knob_step["params"]["retract_m"], 0.060)
        self.assertFalse(knob_step["params"]["require_contact"])
        self.assertEqual(knob_step["params"]["contact_mode"], "open_loop")
        self.assertNotIn("contact", knob_step["params"])
        self.assertAlmostEqual(
            knob_step["params"]["approach_distance_m"]
            + knob_step["params"]["open_loop_advance_m"],
            0.145,
        )

        robot = MockRobotAdapter(contact_after_servo_calls=2)
        resolved_steps = []

        def resolver(step):
            resolved_steps.append(step["name"])
            details = {}
            if step["name"] == "middle_black_knob":
                details = {
                    "pixel": [320.0, 215.0],
                    "desired_pixel": [320.0, 215.0],
                    "bbox": [310, 205, 325, 220],
                    "pixel_count": 125,
                    "pixel_error_base_m": [0.0, 0.0, 0.0],
                }
            return ElementTarget(
                step["name"],
                step["target"]["kind"],
                Pose(0.3, 0.0, 0.4, 3.1416, 0.0, 0.0, frame="base_link"),
                details=details,
            )

        results = run_sequence(config, robot, target_resolver=resolver)

        self.assertEqual(
            resolved_steps,
            ["middle_black_knob", "rightmost_green_rocker", "lower_left_red_button"],
        )
        self.assertEqual(results[0]["name"], "upper_meters")
        self.assertEqual(results[0]["skill"], "read_meter")
        self.assertTrue(all(item["success"] for item in results))

    def test_run_sequence_executes_joint_safe_pose_without_vision(self):
        config = {
            "sequence": [
                {
                    "name": "start_button",
                    "skill": "button",
                    "target": {"kind": "button", "pose": [0.45, 0.10, 0.20, 3.1416, 0.0, 0.0]},
                    "params": {"contact": {"max_contact_travel_m": 0.001}},
                },
                {
                    "name": "safe_pose",
                    "skill": "move_joint",
                    "params": {"joint_positions": [0.1, -0.2, 0.3, 0.0, 0.5, 0.0]},
                },
            ]
        }
        robot = MockRobotAdapter(contact_after_servo_calls=1)
        resolver_calls = []

        def resolver(step):
            resolver_calls.append(step["name"])
            return ElementTarget(
                "start_button", "button", Pose(0.45, 0.10, 0.20, 3.1416, 0.0, 0.0)
            )

        results = run_sequence(config, robot, target_resolver=resolver)

        # The joint-space step must not ask the vision resolver for a target.
        self.assertEqual(resolver_calls, ["start_button"])
        self.assertTrue(all(item["success"] for item in results))
        self.assertEqual(results[1]["skill"], "move_joint")
        self.assertEqual(
            results[1]["details"]["joint_positions"],
            [0.1, -0.2, 0.3, 0.0, 0.5, 0.0],
        )
        joint_commands = [
            command for command in robot.commands if command.name == "move_joint"
        ]
        self.assertEqual(len(joint_commands), 1)

    def test_joint_safe_pose_runs_without_target_block(self):
        config = {
            "sequence": [
                {
                    "name": "safe_pose",
                    "skill": "move_joint",
                    "params": {"joint_positions": [0.0] * 6},
                }
            ]
        }
        robot = MockRobotAdapter()

        results = run_sequence(config, robot)

        self.assertTrue(results[0]["success"])
        self.assertEqual(results[0]["name"], "safe_pose")


if __name__ == "__main__":
    unittest.main()
