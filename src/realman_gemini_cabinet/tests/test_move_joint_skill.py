import unittest

from rg_cabinet.adapters.mock import MockRobotAdapter
from rg_cabinet.geometry import Pose
from rg_cabinet.perception import ElementTarget
from rg_cabinet.skills import MoveJointConfig, MoveJointSkill, SkillContext


class MoveJointSkillTests(unittest.TestCase):
    def setUp(self):
        self.target = ElementTarget(
            name="safe_pose",
            kind="joint_pose",
            pose=Pose(0.0, 0.0, 0.0),
        )

    def test_move_joint_commands_absolute_joint_positions(self):
        robot = MockRobotAdapter()
        skill = MoveJointSkill(
            MoveJointConfig(joint_positions=[0.1, -0.2, 0.3, 0.0, 0.5, 0.0])
        )

        result = skill.execute(SkillContext(robot), self.target)

        self.assertTrue(result.success)
        names = [command.name for command in robot.commands]
        self.assertEqual(names[-1], "move_joint")
        move_command = next(
            command for command in robot.commands if command.name == "move_joint"
        )
        self.assertEqual(
            move_command.payload["joints"],
            [0.1, -0.2, 0.3, 0.0, 0.5, 0.0],
        )
        self.assertEqual(result.details["joint_positions"], [0.1, -0.2, 0.3, 0.0, 0.5, 0.0])

    def test_move_joint_can_open_gripper_before_moving(self):
        robot = MockRobotAdapter()
        skill = MoveJointSkill(
            MoveJointConfig(joint_positions=[0.0] * 6, gripper_position=1.0)
        )

        skill.execute(SkillContext(robot), self.target)

        names = [command.name for command in robot.commands]
        self.assertLess(names.index("set_gripper"), names.index("move_joint"))
        self.assertEqual(robot.commands[0].payload["position"], 1.0)

    def test_move_joint_requires_joint_positions(self):
        robot = MockRobotAdapter()
        skill = MoveJointSkill(MoveJointConfig(joint_positions=[]))

        with self.assertRaises(RuntimeError):
            skill.execute(SkillContext(robot), self.target)

    def test_move_joint_omits_gripper_when_not_configured(self):
        robot = MockRobotAdapter()
        skill = MoveJointSkill(MoveJointConfig(joint_positions=[0.0] * 6))

        skill.execute(SkillContext(robot), self.target)

        names = [command.name for command in robot.commands]
        self.assertNotIn("set_gripper", names)


if __name__ == "__main__":
    unittest.main()
