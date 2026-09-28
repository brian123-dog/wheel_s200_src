import unittest

from rg_cabinet.adapters.mock import MockRobotAdapter
from rg_cabinet.geometry import Pose
from rg_cabinet.perception import ElementTarget
from rg_cabinet.skills import (
    ButtonPressConfig,
    ButtonPressSkill,
    ContactMotionConfig,
    DoorHandleSkill,
    KnobCameraServoConfig,
    KnobCameraServoSkill,
    KnobTurnConfig,
    KnobTurnSkill,
    ReadMeterSkill,
    SkillContext,
    ToggleSwitchConfig,
    ToggleSwitchSkill,
)


class SkillTests(unittest.TestCase):
    def setUp(self):
        self.target = ElementTarget(
            name="target",
            kind="fixture",
            pose=Pose(0.4, 0.0, 0.2, 3.1416, 0.0, 0.0),
        )

    def command_names(self, robot):
        return [command.name for command in robot.commands]

    def move_poses(self, robot):
        return [
            command.payload["pose"]
            for command in robot.commands
            if command.name == "move_pose"
        ]

    def servo_commands(self, robot):
        return [
            command
            for command in robot.commands
            if command.name == "servo_twist"
        ]

    def test_button_press_uses_guarded_contact_and_retract(self):
        robot = MockRobotAdapter(contact_after_servo_calls=1)
        result = ButtonPressSkill().execute(SkillContext(robot), self.target)

        self.assertTrue(result.success)
        self.assertTrue(result.details["contact_detected"])
        self.assertIn("force_or_current_limit", self.command_names(robot))
        self.assertEqual(self.command_names(robot)[-1], "stop")

    def test_toggle_switch_slides_after_contact(self):
        robot = MockRobotAdapter(contact_after_servo_calls=1)
        result = ToggleSwitchSkill().execute(SkillContext(robot), self.target)

        self.assertTrue(result.success)
        servo_count = self.command_names(robot).count("servo_twist")
        self.assertGreaterEqual(servo_count, 3)

    def test_knob_turn_closes_gripper_and_rotates(self):
        robot = MockRobotAdapter(contact_after_servo_calls=1)
        result = KnobTurnSkill().execute(SkillContext(robot), self.target)

        self.assertTrue(result.success)
        self.assertGreaterEqual(self.command_names(robot).count("set_gripper"), 3)
        angular_commands = [
            command
            for command in robot.commands
            if command.name == "servo_twist"
            and abs(command.payload["twist"].angular_z) > 0.0
        ]
        self.assertEqual(len(angular_commands), 1)

    def test_knob_camera_servo_uses_local_alignment_sequence(self):
        robot = MockRobotAdapter(contact_after_servo_calls=1)
        target = ElementTarget(
            name="middle_black_knob",
            kind="knob",
            pose=self.target.pose,
            details={
                "pixel": [640.0, 430.0],
                "bbox": [620, 410, 650, 440],
                "pixel_count": 125,
                "pixel_error_base_m": [0.0, 0.02, -0.01],
            },
        )
        skill = KnobCameraServoSkill(
            KnobCameraServoConfig(
                approach_distance_m=0.01,
                retract_m=0.01,
                turn_angle_rad=1.5708,
                actuation_rpy_base=None,
            )
        )

        result = skill.execute(SkillContext(robot), target)

        self.assertTrue(result.success)
        names = self.command_names(robot)
        self.assertEqual(names[0], "get_tcp_pose")
        self.assertEqual(names[1], "set_gripper")
        self.assertEqual(robot.commands[1].payload["position"], 0.4)
        close_index = next(
            index
            for index, command in enumerate(robot.commands)
            if command.name == "set_gripper"
            and command.payload["position"] == 1.0
        )
        angular_index = next(
            index
            for index, command in enumerate(robot.commands)
            if command.name == "servo_twist"
            and abs(command.payload["twist"].angular_z) > 0.0
        )
        self.assertLess(close_index, angular_index)
        linear_base_commands = [
            command
            for command in robot.commands
            if command.name == "servo_twist"
            and command.payload["frame"] == "base"
            and abs(command.payload["twist"].angular_z) == 0.0
        ]
        self.assertGreaterEqual(len(linear_base_commands), 4)
        self.assertAlmostEqual(result.details["align_delta_base_m"]["y"], 0.02)
        self.assertAlmostEqual(result.details["align_delta_base_m"]["z"], -0.01)
        self.assertEqual(len(result.details["alignment_iterations"]), 1)
        self.assertAlmostEqual(
            result.details["alignment_iterations"][0]["move_delta_base_m"]["y"],
            0.02,
        )
        self.assertEqual(result.details["pixel_count"], 125)

    def test_knob_camera_servo_allows_rotation_without_contact_by_default(self):
        robot = MockRobotAdapter(contact_after_servo_calls=None)
        target = ElementTarget(
            name="middle_black_knob",
            kind="knob",
            pose=self.target.pose,
            details={"pixel_error_base_m": [0.0, 0.0, 0.0]},
        )
        skill = KnobCameraServoSkill(
            KnobCameraServoConfig(
                approach_distance_m=0.0,
                retract_m=0.0,
                turn_angle_rad=1.0,
                actuation_rpy_base=None,
                contact=ContactMotionConfig(max_contact_travel_m=0.001),
            )
        )

        result = skill.execute(SkillContext(robot), target)

        self.assertTrue(result.success)
        self.assertFalse(result.details["contact_detected"])
        angular_commands = [
            command
            for command in robot.commands
            if command.name == "servo_twist"
            and abs(command.payload["twist"].angular_z) > 0.0
        ]
        self.assertEqual(len(angular_commands), 1)

    def test_knob_camera_servo_requires_contact_before_rotation_and_retracts(self):
        robot = MockRobotAdapter(contact_after_servo_calls=None)
        target = ElementTarget(
            name="middle_black_knob",
            kind="knob",
            pose=self.target.pose,
            details={"pixel_error_base_m": [0.0, 0.0, 0.0]},
        )
        skill = KnobCameraServoSkill(
            KnobCameraServoConfig(
                approach_distance_m=0.0,
                retract_m=0.01,
                turn_angle_rad=1.0,
                actuation_rpy_base=None,
                require_contact=True,
                contact=ContactMotionConfig(
                    approach_speed_mps=0.005,
                    max_contact_travel_m=0.001,
                    timeout_s=1.0,
                ),
            )
        )

        with self.assertRaisesRegex(RuntimeError, "did not detect contact"):
            skill.execute(SkillContext(robot), target)

        angular_commands = [
            command
            for command in robot.commands
            if command.name == "servo_twist"
            and abs(command.payload["twist"].angular_z) > 0.0
        ]
        self.assertEqual(angular_commands, [])
        closed_gripper_commands = [
            command
            for command in robot.commands
            if command.name == "set_gripper"
            and command.payload["position"] == 1.0
        ]
        self.assertEqual(closed_gripper_commands, [])
        linear_base_commands = [
            command
            for command in robot.commands
            if command.name == "servo_twist"
            and command.payload["frame"] == "base"
            and command.payload["twist"].angular_x == 0.0
            and command.payload["twist"].angular_y == 0.0
            and command.payload["twist"].angular_z == 0.0
        ]
        self.assertGreaterEqual(len(linear_base_commands), 2)
        self.assertLess(linear_base_commands[0].payload["twist"].linear_x, 0.0)
        self.assertGreater(linear_base_commands[-1].payload["twist"].linear_x, 0.0)

    def test_knob_camera_servo_open_loop_advance_rotates_without_contact_check(self):
        robot = MockRobotAdapter(contact_after_servo_calls=None)
        target = ElementTarget(
            name="middle_black_knob",
            kind="knob",
            pose=self.target.pose,
            details={"pixel_error_base_m": [0.0, 0.0, 0.0]},
        )
        skill = KnobCameraServoSkill(
            KnobCameraServoConfig(
                approach_distance_m=0.060,
                open_loop_advance_m=0.085,
                open_loop_advance_speed_mps=0.004,
                retract_m=0.060,
                turn_angle_rad=1.0,
                actuation_rpy_base=None,
                contact_mode="open_loop",
                contact=ContactMotionConfig(max_contact_travel_m=0.500),
            )
        )

        result = skill.execute(SkillContext(robot), target)

        self.assertTrue(result.success)
        self.assertIsNone(result.details["contact_detected"])
        self.assertEqual(result.details["contact_mode"], "open_loop")
        self.assertAlmostEqual(result.details["planned_approach_m"], 0.145)
        self.assertNotIn("force_or_current_limit", self.command_names(robot))
        angular_commands = [
            command
            for command in robot.commands
            if command.name == "servo_twist"
            and abs(command.payload["twist"].angular_z) > 0.0
        ]
        self.assertEqual(len(angular_commands), 1)
        forward_distance_m = -sum(
            command.payload["twist"].linear_x * command.payload["duration_s"]
            for command in robot.commands
            if command.name == "servo_twist"
            and command.payload["frame"] == "base"
            and command.payload["twist"].linear_x < 0.0
        )
        self.assertAlmostEqual(forward_distance_m, 0.145)
        self.assertTrue(result.details["stages"]["approach_distance"])
        self.assertTrue(result.details["stages"]["open_loop_advance"])
        self.assertTrue(result.details["stages"]["turn"])

    def test_knob_camera_servo_refreshes_until_pixel_tolerance(self):
        robot = MockRobotAdapter(contact_after_servo_calls=1)

        def make_target(pixel, pixel_error_base_m):
            return ElementTarget(
                name="middle_black_knob",
                kind="knob",
                pose=self.target.pose,
                details={
                    "pixel": pixel,
                    "desired_pixel": [640.0, 430.0],
                    "bbox": [620, 410, 650, 440],
                    "pixel_count": 125,
                    "pixel_error_base_m": pixel_error_base_m,
                },
            )

        refreshed_targets = [
            make_target([628.0, 420.0], [0.0, 0.008, 0.006]),
            make_target([635.0, 425.0], [0.0, 0.003, 0.003]),
        ]

        def refresh_target():
            return refreshed_targets.pop(0)

        skill = KnobCameraServoSkill(
            KnobCameraServoConfig(
                approach_distance_m=0.0,
                retract_m=0.0,
                turn_angle_rad=0.0,
                actuation_rpy_base=None,
                align_settle_s=0.0,
                contact=ContactMotionConfig(max_contact_travel_m=0.0),
            )
        )

        result = skill.execute(
            SkillContext(
                robot,
                target_refresher=refresh_target,
            ),
            make_target([600.0, 330.0], [0.0, 0.040, 0.040]),
        )

        self.assertTrue(result.success)
        self.assertEqual(refreshed_targets, [])
        self.assertLessEqual(result.details["final_pixel_error_px"], 12.0)
        self.assertEqual(result.details["final_pixel"], [635.0, 425.0])
        iterations = result.details["alignment_iterations"]
        self.assertEqual(len(iterations), 2)
        self.assertAlmostEqual(iterations[0]["move_delta_base_m"]["y"], 0.025)
        self.assertAlmostEqual(iterations[0]["move_delta_base_m"]["z"], 0.025)
        self.assertAlmostEqual(iterations[1]["move_delta_base_m"]["y"], 0.0056)
        self.assertAlmostEqual(iterations[1]["move_delta_base_m"]["z"], 0.0042)
        self.assertEqual(result.details["alignment_stop_reason"], "pixel_tolerance_met")

    def test_knob_camera_servo_skips_closed_loop_when_initial_pixel_is_aligned(self):
        robot = MockRobotAdapter(contact_after_servo_calls=1)
        refresh_calls = []
        target = ElementTarget(
            name="middle_black_knob",
            kind="knob",
            pose=self.target.pose,
            details={
                "pixel": [638.0, 427.0],
                "desired_pixel": [640.0, 430.0],
                "pixel_error_base_m": [0.0, 0.020, 0.020],
            },
        )
        skill = KnobCameraServoSkill(
            KnobCameraServoConfig(
                approach_distance_m=0.0,
                retract_m=0.0,
                turn_angle_rad=0.0,
                actuation_rpy_base=None,
                align_settle_s=0.0,
                contact=ContactMotionConfig(max_contact_travel_m=0.0),
            )
        )

        result = skill.execute(
            SkillContext(
                robot,
                target_refresher=lambda: refresh_calls.append("called"),
            ),
            target,
        )

        self.assertTrue(result.success)
        self.assertEqual(refresh_calls, [])
        self.assertEqual(result.details["alignment_iterations"], [])
        self.assertEqual(result.details["alignment_stop_reason"], "pixel_tolerance_met")

    def test_knob_camera_servo_uses_proceed_tolerance_before_approach(self):
        robot = MockRobotAdapter(contact_after_servo_calls=None)
        refresh_calls = []
        target = ElementTarget(
            name="middle_black_knob",
            kind="knob",
            pose=self.target.pose,
            details={
                "pixel": [338.0, 213.5],
                "desired_pixel": [320.0, 215.0],
                "pixel_error_base_m": [0.0, 0.021, 0.002],
            },
        )
        skill = KnobCameraServoSkill(
            KnobCameraServoConfig(
                align_tolerance_px=5.0,
                proceed_tolerance_px=20.0,
                approach_distance_m=0.060,
                open_loop_advance_m=0.085,
                open_loop_advance_speed_mps=0.004,
                retract_m=0.0,
                turn_angle_rad=0.0,
                actuation_rpy_base=None,
                align_settle_s=0.0,
                contact_mode="open_loop",
            )
        )

        result = skill.execute(
            SkillContext(robot, target_refresher=lambda: refresh_calls.append("called")),
            target,
        )

        self.assertTrue(result.success)
        self.assertEqual(refresh_calls, [])
        self.assertEqual(result.details["alignment_iterations"], [])
        self.assertEqual(result.details["alignment_stop_reason"], "proceed_tolerance_met")
        self.assertTrue(result.details["stages"]["approach_distance"])
        self.assertTrue(result.details["stages"]["open_loop_advance"])
        forward_distance_m = -sum(
            command.payload["twist"].linear_x * command.payload["duration_s"]
            for command in robot.commands
            if command.name == "servo_twist"
            and command.payload["frame"] == "base"
            and command.payload["twist"].linear_x < 0.0
        )
        self.assertAlmostEqual(forward_distance_m, 0.145)

    def test_knob_camera_servo_wraps_alignment_driver_failure_with_stage(self):
        robot = _FailingAlignmentRobot()
        target = ElementTarget(
            name="middle_black_knob",
            kind="knob",
            pose=self.target.pose,
            details={
                "pixel": [360.0, 215.0],
                "desired_pixel": [320.0, 215.0],
                "pixel_error_base_m": [0.0, 0.040, 0.0],
            },
        )
        skill = KnobCameraServoSkill(
            KnobCameraServoConfig(
                align_tolerance_px=5.0,
                proceed_tolerance_px=20.0,
                approach_distance_m=0.0,
                retract_m=0.0,
                turn_angle_rad=0.0,
                actuation_rpy_base=None,
                align_settle_s=0.0,
            )
        )

        with self.assertRaisesRegex(RuntimeError, "stage=visual_alignment"):
            skill.execute(SkillContext(robot, target_refresher=lambda: target), target)

    def test_knob_camera_servo_can_proceed_after_alignment_driver_failure(self):
        robot = _FailingAlignmentRobot()
        target = ElementTarget(
            name="middle_black_knob",
            kind="knob",
            pose=self.target.pose,
            details={
                "pixel": [338.0, 213.5],
                "desired_pixel": [320.0, 215.0],
                "pixel_error_base_m": [0.0, 0.021, 0.0],
            },
        )
        skill = KnobCameraServoSkill(
            KnobCameraServoConfig(
                align_tolerance_px=5.0,
                proceed_tolerance_px=20.0,
                approach_distance_m=0.010,
                open_loop_advance_m=0.010,
                retract_m=0.0,
                turn_angle_rad=0.0,
                actuation_rpy_base=None,
                contact_mode="open_loop",
                align_settle_s=0.0,
            )
        )

        result = skill.execute(SkillContext(robot, target_refresher=None), target)

        self.assertTrue(result.success)
        self.assertEqual(result.details["alignment_stop_reason"], "driver_rejected_alignment_move")
        self.assertTrue(result.details["stages"]["approach_distance"])
        self.assertTrue(result.details["stages"]["open_loop_advance"])

    def test_knob_camera_servo_rejects_unsafe_local_alignment_without_motion(self):
        robot = MockRobotAdapter(contact_after_servo_calls=1)
        target = ElementTarget(
            name="middle_black_knob",
            kind="knob",
            pose=self.target.pose,
            details={"pixel_error_base_m": [0.0, 0.20, 0.0]},
        )
        skill = KnobCameraServoSkill(
            KnobCameraServoConfig(max_align_axis_m=0.05, max_align_total_m=0.10)
        )

        with self.assertRaisesRegex(RuntimeError, "exceeds max_align_axis_m"):
            skill.execute(SkillContext(robot), target)

        self.assertEqual(robot.commands, [])

    def test_knob_camera_servo_prefers_tcp_alignment_error(self):
        robot = MockRobotAdapter(contact_after_servo_calls=1)
        target = ElementTarget(
            name="middle_black_knob",
            kind="knob",
            pose=self.target.pose,
            details={
                "pixel": [42.0, 34.0],
                "desired_pixel": [40.0, 35.0],
                "alignment_reference_pixel": [42.0, 34.0],
                "pixel_error_base_m": [0.0, 0.050, 0.050],
                "alignment_error_base_m": [0.0, 0.010, -0.020],
            },
        )
        skill = KnobCameraServoSkill(
            KnobCameraServoConfig(
                approach_distance_m=0.0,
                retract_m=0.0,
                turn_angle_rad=0.0,
                actuation_rpy_base=None,
                contact=ContactMotionConfig(max_contact_travel_m=0.0),
            )
        )

        result = skill.execute(SkillContext(robot), target)

        self.assertTrue(result.success)
        self.assertEqual(result.details["alignment_error_source"], "alignment_error_base_m")
        self.assertAlmostEqual(result.details["initial_align_delta_base_m"]["y"], 0.010)
        self.assertAlmostEqual(result.details["initial_align_delta_base_m"]["z"], -0.020)
        self.assertEqual(result.details["final_pixel_error_px"], 0.0)
        self.assertEqual(result.details["final_alignment_error_base_m"], [0.0, 0.010, -0.020])

    def test_knob_camera_servo_requires_pixel_error_before_motion(self):
        robot = MockRobotAdapter(contact_after_servo_calls=1)

        with self.assertRaisesRegex(RuntimeError, "requires target.details.alignment_error_base_m"):
            KnobCameraServoSkill().execute(SkillContext(robot), self.target)

        self.assertEqual(robot.commands, [])

    def test_contact_skills_use_base_x_approach_poses(self):
        cases = [
            (
                ButtonPressSkill(
                    ButtonPressConfig(
                        approach_axis="x",
                        approach_offset_frame="base",
                    )
                ),
                0.04,
                0.008,
            ),
            (
                KnobTurnSkill(
                    KnobTurnConfig(
                        approach_axis="x",
                        approach_offset_frame="base",
                    )
                ),
                0.045,
                0.008,
            ),
            (
                ToggleSwitchSkill(
                    ToggleSwitchConfig(
                        approach_axis="x",
                        approach_offset_frame="base",
                    )
                ),
                0.035,
                0.006,
            ),
        ]

        for skill, pre_offset, start_offset in cases:
            with self.subTest(skill=skill.__class__.__name__):
                robot = MockRobotAdapter(contact_after_servo_calls=1)
                skill.execute(SkillContext(robot), self.target)
                move_poses = self.move_poses(robot)

                self.assertGreaterEqual(len(move_poses), 2)
                self.assertAlmostEqual(move_poses[0].x, self.target.pose.x + pre_offset)
                self.assertAlmostEqual(move_poses[1].x, self.target.pose.x + start_offset)
                self.assertAlmostEqual(move_poses[0].y, self.target.pose.y)
                self.assertAlmostEqual(move_poses[1].y, self.target.pose.y)
                self.assertAlmostEqual(move_poses[0].z, self.target.pose.z)
                self.assertAlmostEqual(move_poses[1].z, self.target.pose.z)
                self.assertEqual(move_poses[0].frame, "base")
                self.assertEqual(move_poses[1].frame, "base")

    def test_contact_and_retract_servo_use_base_frame(self):
        cases = [
            ButtonPressSkill(
                ButtonPressConfig(approach_axis="x", approach_offset_frame="base")
            ),
            KnobTurnSkill(
                KnobTurnConfig(approach_axis="x", approach_offset_frame="base")
            ),
            ToggleSwitchSkill(
                ToggleSwitchConfig(approach_axis="x", approach_offset_frame="base")
            ),
        ]

        for skill in cases:
            with self.subTest(skill=skill.__class__.__name__):
                robot = MockRobotAdapter(contact_after_servo_calls=1)
                skill.execute(SkillContext(robot), self.target)
                linear_servo_commands = [
                    command
                    for command in self.servo_commands(robot)
                    if command.payload["twist"].angular_x == 0.0
                    and command.payload["twist"].angular_y == 0.0
                    and command.payload["twist"].angular_z == 0.0
                ]

                self.assertGreaterEqual(len(linear_servo_commands), 2)
                self.assertEqual(linear_servo_commands[0].payload["frame"], "base")
                self.assertLess(linear_servo_commands[0].payload["twist"].linear_x, 0.0)
                self.assertEqual(linear_servo_commands[-1].payload["frame"], "base")
                self.assertGreater(linear_servo_commands[-1].payload["twist"].linear_x, 0.0)

    def test_door_handle_generates_pull_waypoints(self):
        robot = MockRobotAdapter(contact_after_servo_calls=1)
        result = DoorHandleSkill().execute(SkillContext(robot), self.target)

        self.assertTrue(result.success)
        move_pose_count = self.command_names(robot).count("move_pose")
        self.assertGreaterEqual(move_pose_count, 10)

    def test_read_meter_is_non_contact(self):
        robot = MockRobotAdapter(contact_after_servo_calls=1)
        result = ReadMeterSkill().execute(SkillContext(robot), self.target)

        self.assertTrue(result.success)
        self.assertTrue(result.details["non_contact"])
        self.assertEqual(self.command_names(robot), [])


class _FailingAlignmentRobot(MockRobotAdapter):
    def servo_twist(self, twist, duration_s: float, frame: str = "tool") -> None:
        if frame == "base" and (abs(twist.linear_y) > 0.0 or abs(twist.linear_z) > 0.0):
            raise RuntimeError("rm_driver reported movel failure")
        super().servo_twist(twist, duration_s, frame)


if __name__ == "__main__":
    unittest.main()
