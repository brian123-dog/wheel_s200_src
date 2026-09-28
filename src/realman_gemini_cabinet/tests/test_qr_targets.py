import json
import os
import tempfile
import unittest
from pathlib import Path

from rg_cabinet.geometry import Pose
from rg_cabinet.qr_targets import (
    QrTargetSet,
    Quaternion,
    load_qr_target_set,
    overlay_qr_observation_poses,
)


def write_qr_result(path, positions):
    payload = {"position": positions}
    outer = {"qr_data": json.dumps(payload)}
    path.write_text(json.dumps(outer), encoding="utf-8")


class QrTargetsTests(unittest.TestCase):
    def test_loads_current_sample_qr_data_string(self):
        sample_path = (
            Path(__file__).resolve().parents[2]
            / "qr_detector"
            / "qr_results"
            / "qr_20260523_153714_465.json"
        )
        if not sample_path.is_file():
            self.skipTest(f"sample QR result is not present: {sample_path}")

        targets = load_qr_target_set(sample_path, frame="base_link")

        self.assertEqual(targets.source_file, str(sample_path))
        self.assertEqual(len(targets.poses), 4)
        self.assertEqual(targets.poses[0].frame, "base_link")
        self.assertAlmostEqual(targets.poses[0].x, -0.33078500628471375)
        self.assertAlmostEqual(targets.poses[0].z, 0.5782570242881775)
        self.assertIsNotNone(targets.quaternion_at(0))

    def test_selects_position_json_from_directory_first(self):
        with tempfile.TemporaryDirectory() as tmp:
            directory = Path(tmp)
            position = directory / "position.json"
            newest_qr = directory / "qr_newer.json"
            write_qr_result(
                position,
                [
                    {
                        "x": 4,
                        "y": 5,
                        "z": 6,
                        "roll": 0.4,
                        "pitch": 0.5,
                        "yaw": 0.6,
                    }
                ],
            )
            write_qr_result(
                newest_qr,
                [
                    {
                        "x": 1,
                        "y": 2,
                        "z": 3,
                        "roll": 0.1,
                        "pitch": 0.2,
                        "yaw": 0.3,
                    }
                ],
            )
            os.utime(position, ns=(1_000_000_000, 1_000_000_000))
            os.utime(newest_qr, ns=(2_000_000_000, 2_000_000_000))

            targets = load_qr_target_set(qr_results_dir=directory)

        self.assertEqual(Path(targets.source_file).name, "position.json")
        self.assertEqual(targets.poses[0].as_list(), [4.0, 5.0, 6.0, 0.4, 0.5, 0.6])

    def test_selects_latest_qr_file_from_directory(self):
        with tempfile.TemporaryDirectory() as tmp:
            directory = Path(tmp)
            older = directory / "qr_older.json"
            newer = directory / "qr_newer.json"
            write_qr_result(
                older,
                [
                    {
                        "x": 0,
                        "y": 0,
                        "z": 0,
                        "roll": 0,
                        "pitch": 0,
                        "yaw": 0,
                    }
                ],
            )
            write_qr_result(
                newer,
                [
                    {
                        "x": 1,
                        "y": 2,
                        "z": 3,
                        "roll": 0.1,
                        "pitch": 0.2,
                        "yaw": 0.3,
                    }
                ],
            )
            os.utime(older, ns=(1_000_000_000, 1_000_000_000))
            os.utime(newer, ns=(2_000_000_000, 2_000_000_000))

            targets = load_qr_target_set(qr_results_dir=directory)

        self.assertEqual(Path(targets.source_file).name, "qr_newer.json")
        self.assertEqual(targets.poses[0].as_list(), [1.0, 2.0, 3.0, 0.1, 0.2, 0.3])

    def test_invalid_qr_data_fails(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "qr_bad.json"
            path.write_text(json.dumps({"qr_data": "{bad"}), encoding="utf-8")

            with self.assertRaisesRegex(ValueError, "qr_data is not valid JSON"):
                load_qr_target_set(path)

    def test_loads_nested_quaternion_pose(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "qr_quat.json"
            write_qr_result(
                path,
                [
                    {
                        "position": {"x": 0.1, "y": 0.2, "z": 0.3},
                        "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 2.0},
                    }
                ],
            )

            targets = load_qr_target_set(path, frame="base_link")

        self.assertEqual(targets.poses[0].as_list(), [0.1, 0.2, 0.3, 0.0, 0.0, 0.0])
        self.assertEqual(
            targets.quaternion_at(0),
            Quaternion(0.0, 0.0, 0.0, 1.0),
        )

    def test_loads_named_target(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "qr_named.json"
            write_qr_result(
                path,
                [
                    {
                        "name": "lower_left_red_button",
                        "position": {"x": 0.1, "y": 0.2, "z": 0.3},
                        "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0},
                    }
                ],
            )

            targets = load_qr_target_set(path, frame="base_link")

        self.assertEqual(targets.name_at(0), "lower_left_red_button")
        self.assertTrue(targets.has_named_targets())
        self.assertEqual(targets.poses[0].as_list(), [0.1, 0.2, 0.3, 0.0, 0.0, 0.0])

    def test_loads_flat_quaternion_pose(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "qr_flat_quat.json"
            write_qr_result(
                path,
                [
                    {
                        "x": 0.1,
                        "y": 0.2,
                        "z": 0.3,
                        "qx": 0.0,
                        "qy": 0.0,
                        "qz": 0.0,
                        "qw": 1.0,
                    }
                ],
            )

            targets = load_qr_target_set(path, frame="base_link")

        self.assertEqual(targets.poses[0].as_list(), [0.1, 0.2, 0.3, 0.0, 0.0, 0.0])
        self.assertEqual(
            targets.quaternion_at(0),
            Quaternion(0.0, 0.0, 0.0, 1.0),
        )

    def test_zero_quaternion_fails(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "qr_zero_quat.json"
            write_qr_result(
                path,
                [
                    {
                        "position": {"x": 0.1, "y": 0.2, "z": 0.3},
                        "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 0.0},
                    }
                ],
            )

            with self.assertRaisesRegex(ValueError, "quaternion must not be zero"):
                load_qr_target_set(path)

    def test_missing_position_field_fails(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "qr_missing_field.json"
            write_qr_result(
                path,
                [
                    {
                        "x": 1,
                        "y": 2,
                        "z": 3,
                        "roll": 0,
                        "pitch": 0,
                    }
                ],
            )

            with self.assertRaisesRegex(ValueError, "missing fields"):
                load_qr_target_set(path)

    def test_overlay_adds_observation_poses_and_requires_matching_count(self):
        config = {
            "sequence": [
                {
                    "name": "start_button",
                    "skill": "button",
                    "target": {
                        "source": "camera",
                        "detection": "start_button",
                        "kind": "button",
                        "pose": [0, 0, 0, 0, 0, 0],
                    },
                    "params": {"pre_offset_m": 0.04},
                    "verify": {"sensor": "start_button", "expected": "pressed"},
                },
                {
                    "name": "mode_toggle",
                    "skill": "toggle",
                    "target": {"kind": "toggle_switch", "pose": [0, 0, 0, 0, 0, 0]},
                    "params": {"axis": "y"},
                },
            ]
        }
        targets = QrTargetSet(
            source_file="/tmp/qr.json",
            poses=[
                Pose(0.1, 0.2, 0.3, 0.4, 0.5, 0.6, frame="base_link"),
                Pose(1.1, 1.2, 1.3, 1.4, 1.5, 1.6, frame="base_link"),
            ],
        )

        updated = overlay_qr_observation_poses(config, targets)

        self.assertEqual(updated["qr_source_file"], "/tmp/qr.json")
        self.assertEqual(
            updated["sequence"][0]["observation_pose"]["pose"],
            [0.1, 0.2, 0.3, 0.4, 0.5, 0.6],
        )
        self.assertEqual(updated["sequence"][0]["observation_pose"]["frame"], "base_link")
        self.assertEqual(updated["sequence"][0]["target"]["kind"], "button")
        self.assertEqual(updated["sequence"][0]["target"]["source"], "camera")
        self.assertEqual(updated["sequence"][0]["target"]["detection"], "start_button")
        self.assertEqual(updated["sequence"][0]["target"]["pose"], [0, 0, 0, 0, 0, 0])
        self.assertEqual(updated["sequence"][0]["params"], {"pre_offset_m": 0.04})
        self.assertEqual(
            updated["sequence"][0]["verify"],
            {"sensor": "start_button", "expected": "pressed"},
        )
        self.assertEqual(config["sequence"][0]["target"]["source"], "camera")
        self.assertNotIn("observation_pose", config["sequence"][0])

        with self.assertRaisesRegex(ValueError, "does not match"):
            overlay_qr_observation_poses(
                config,
                QrTargetSet("/tmp/one.json", targets.poses[:1]),
            )

    def test_overlay_named_targets_by_step_name(self):
        config = {
            "sequence": [
                {"name": "lower_left_red_button", "skill": "button"},
                {"name": "middle_black_knob", "skill": "knob"},
            ]
        }
        targets = QrTargetSet(
            source_file="/tmp/qr.json",
            poses=[
                Pose(1.0, 1.1, 1.2, frame="base_link"),
                Pose(9.0, 9.1, 9.2, frame="base_link"),
                Pose(0.1, 0.2, 0.3, frame="base_link"),
            ],
            names=[
                "middle_black_knob",
                "unused_extra_pose",
                "lower_left_red_button",
            ],
        )

        updated = overlay_qr_observation_poses(config, targets)

        self.assertEqual(
            updated["sequence"][0]["observation_pose"]["pose"],
            [0.1, 0.2, 0.3, 0.0, 0.0, 0.0],
        )
        self.assertEqual(
            updated["sequence"][1]["observation_pose"]["pose"],
            [1.0, 1.1, 1.2, 0.0, 0.0, 0.0],
        )

    def test_overlay_named_targets_by_explicit_qr_name(self):
        config = {
            "sequence": [
                {
                    "name": "press_first",
                    "skill": "button",
                    "observation_pose": {
                        "qr_name": "lower_left_red_button",
                        "move_speed_mps": 0.02,
                    },
                }
            ]
        }
        targets = QrTargetSet(
            source_file="/tmp/qr.json",
            poses=[Pose(0.1, 0.2, 0.3, frame="base_link")],
            names=["lower_left_red_button"],
        )

        updated = overlay_qr_observation_poses(config, targets)
        observation = updated["sequence"][0]["observation_pose"]

        self.assertEqual(observation["pose"], [0.1, 0.2, 0.3, 0.0, 0.0, 0.0])
        self.assertEqual(observation["qr_name"], "lower_left_red_button")
        self.assertEqual(observation["move_speed_mps"], 0.02)

    def test_overlay_named_targets_reports_missing_name(self):
        config = {
            "sequence": [
                {
                    "name": "lower_left_red_button",
                    "skill": "button",
                    "observation_pose": {"qr_name": "missing_button"},
                }
            ]
        }
        targets = QrTargetSet(
            source_file="/tmp/qr.json",
            poses=[Pose(0.1, 0.2, 0.3, frame="base_link")],
            names=["lower_left_red_button"],
        )

        with self.assertRaisesRegex(ValueError, "Missing QR target name 'missing_button'"):
            overlay_qr_observation_poses(config, targets)

    def test_overlay_named_targets_rejects_duplicate_names(self):
        config = {
            "sequence": [
                {"name": "lower_left_red_button", "skill": "button"},
            ]
        }
        targets = QrTargetSet(
            source_file="/tmp/qr.json",
            poses=[
                Pose(0.1, 0.2, 0.3, frame="base_link"),
                Pose(0.4, 0.5, 0.6, frame="base_link"),
            ],
            names=["lower_left_red_button", "lower_left_red_button"],
        )

        with self.assertRaisesRegex(
            ValueError,
            "Duplicate QR target name 'lower_left_red_button'",
        ):
            overlay_qr_observation_poses(config, targets)

    def test_overlay_keeps_quaternion_observation_pose(self):
        config = {
            "sequence": [
                {
                    "name": "start_button",
                    "skill": "button",
                    "observation_pose": {"move_speed_mps": 0.05, "pose": [9, 9, 9]},
                    "target": {
                        "source": "camera",
                        "detection": "start_button",
                        "kind": "button",
                    },
                }
            ]
        }
        targets = QrTargetSet(
            source_file="/tmp/qr.json",
            poses=[Pose(0.1, 0.2, 0.3, frame="base_link")],
            quaternions=[Quaternion(0.0, 0.0, 0.0, 1.0)],
        )

        updated = overlay_qr_observation_poses(config, targets)
        observation = updated["sequence"][0]["observation_pose"]

        self.assertNotIn("pose", observation)
        self.assertEqual(observation["frame"], "base_link")
        self.assertEqual(observation["position"], {"x": 0.1, "y": 0.2, "z": 0.3})
        self.assertEqual(
            observation["orientation"],
            {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0},
        )
        self.assertEqual(observation["move_speed_mps"], 0.05)

    def test_overlay_skips_joint_space_safe_pose_step(self):
        config = {
            "sequence": [
                {"name": "start_button", "skill": "button"},
                {
                    "name": "safe_pose",
                    "skill": "move_joint",
                    "params": {"joint_positions": [0.0] * 6},
                },
            ]
        }
        targets = QrTargetSet(
            source_file="/tmp/qr.json",
            poses=[Pose(0.1, 0.2, 0.3, frame="base_link")],
        )

        updated = overlay_qr_observation_poses(config, targets)

        self.assertEqual(
            updated["sequence"][0]["observation_pose"]["pose"],
            [0.1, 0.2, 0.3, 0.0, 0.0, 0.0],
        )
        self.assertNotIn("observation_pose", updated["sequence"][1])

    def test_overlay_skips_step_marked_skip_in_observation_pose(self):
        config = {
            "sequence": [
                {"name": "start_button", "skill": "button"},
                {
                    "name": "safe_pose",
                    "skill": "button",
                    "observation_pose": {"skip": True},
                },
            ]
        }
        targets = QrTargetSet(
            source_file="/tmp/qr.json",
            poses=[Pose(0.4, 0.5, 0.6, frame="base_link")],
        )

        updated = overlay_qr_observation_poses(config, targets)

        self.assertEqual(
            updated["sequence"][0]["observation_pose"]["pose"],
            [0.4, 0.5, 0.6, 0.0, 0.0, 0.0],
        )
        self.assertEqual(updated["sequence"][1]["observation_pose"], {"skip": True})

    def test_overlay_counts_only_observing_steps_against_targets(self):
        config = {
            "sequence": [
                {
                    "name": "safe_pose_first",
                    "skill": "move_joint",
                    "params": {"joint_positions": [0.0] * 6},
                },
                {"name": "start_button", "skill": "button"},
            ]
        }
        targets = QrTargetSet(
            source_file="/tmp/qr.json",
            poses=[
                Pose(0.1, 0.2, 0.3, frame="base_link"),
                Pose(9.9, 9.9, 9.9, frame="base_link"),
            ],
        )

        # Two QR poses but only one observing step must be rejected.
        with self.assertRaisesRegex(ValueError, "does not match"):
            overlay_qr_observation_poses(config, targets)


if __name__ == "__main__":
    unittest.main()
