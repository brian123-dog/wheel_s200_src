import sys
import types
import unittest
from types import SimpleNamespace
from unittest.mock import patch


class FakeRosPose:
    def __init__(self):
        self.position = SimpleNamespace(x=0.0, y=0.0, z=0.0)
        self.orientation = SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0)


class FakeMovej:
    pass


class FakeMovejp:
    pass


class FakeMovel:
    pass


class FakeMoveloffset:
    pass


class FakeGripperset:
    pass


class FakeBool:
    def __init__(self, data=False):
        self.data = data


class FakePublisher:
    def __init__(self, node, msg_type, topic):
        self.node = node
        self.msg_type = msg_type
        self.topic = topic
        self.messages = []

    def publish(self, msg):
        self.messages.append(msg)
        if self.topic not in self.node.auto_results:
            return
        result_topic, value = self.node.auto_results[self.topic]
        callback = self.node.subscription_callbacks[result_topic]
        callback(FakeBool(value))

    def get_subscription_count(self):
        return 1


class FakeNode:
    def __init__(self):
        self.default_callback_group = object()
        self.rm_driver_callback_group = object()
        self.parameters = {
            "rm_driver_topic_prefix": "/rm_driver",
            "rm_driver_dof": 6,
            "rm_driver_default_speed_percent": 5,
            "rm_driver_max_speed_percent": 20,
            "rm_driver_linear_speed_scale_mps": 0.25,
            "rm_driver_angular_speed_scale_radps": 1.0,
            "rm_driver_servo_motion_mode": "movel",
            "rm_driver_result_timeout_s": 20.0,
            "rm_driver_discovery_timeout_s": 5.0,
            "rm_driver_wait_for_subscribers": False,
            "rm_driver_gripper_timeout_ms": 3000,
            "rm_driver_gripper_min_position": 1,
            "rm_driver_gripper_max_position": 1000,
            "base_frame": "base_link",
        }
        self.publishers = {}
        self.subscriptions = {}
        self.subscription_callbacks = {}
        self.auto_results = {}
        self.latest_tcp_pose = None
        self.latest_force = None
        self.latest_joint_current = None
        self.stop_count = 0
        self.status_payloads = []

    def get_parameter(self, name):
        return SimpleNamespace(value=self.parameters[name])

    def create_publisher(self, msg_type, topic, qos):
        del qos
        publisher = FakePublisher(self, msg_type, topic)
        self.publishers[topic] = publisher
        return publisher

    def create_subscription(self, msg_type, topic, callback, qos, callback_group=None):
        del msg_type, qos
        self.subscription_callbacks[topic] = callback
        subscription = SimpleNamespace(
            topic=topic,
            callback_group=callback_group or self.default_callback_group,
        )
        self.subscriptions[topic] = subscription
        return subscription

    def publish_motion_stop(self):
        self.stop_count += 1

    def _publish_status(self, payload):
        self.status_payloads.append(payload)


def fake_ros_modules():
    rm_msg = types.ModuleType("rm_ros_interfaces.msg")
    rm_msg.Gripperset = FakeGripperset
    rm_msg.Movej = FakeMovej
    rm_msg.Movejp = FakeMovejp
    rm_msg.Movel = FakeMovel
    rm_msg.Moveloffset = FakeMoveloffset
    rm_pkg = types.ModuleType("rm_ros_interfaces")
    rm_pkg.msg = rm_msg

    std_msg = types.ModuleType("std_msgs.msg")
    std_msg.Bool = FakeBool
    std_pkg = types.ModuleType("std_msgs")
    std_pkg.msg = std_msg

    geometry_msg = types.ModuleType("geometry_msgs.msg")
    geometry_msg.Pose = FakeRosPose
    geometry_pkg = types.ModuleType("geometry_msgs")
    geometry_pkg.msg = geometry_msg

    return {
        "rm_ros_interfaces": rm_pkg,
        "rm_ros_interfaces.msg": rm_msg,
        "std_msgs": std_pkg,
        "std_msgs.msg": std_msg,
        "geometry_msgs": geometry_pkg,
        "geometry_msgs.msg": geometry_msg,
    }


class RmDriverBackendTests(unittest.TestCase):
    def test_movej_p_topics_are_created(self):
        with patch.dict(sys.modules, fake_ros_modules()):
            from rg_cabinet_ros2.backends import RmDriverRobotAdapter

            node = FakeNode()
            RmDriverRobotAdapter(node)

        self.assertIn("/rm_driver/movej_p_cmd", node.publishers)
        self.assertIn("/rm_driver/movej_p_result", node.subscription_callbacks)

    def test_result_subscriptions_use_rm_driver_callback_group(self):
        with patch.dict(sys.modules, fake_ros_modules()):
            from rg_cabinet_ros2.backends import RmDriverRobotAdapter

            node = FakeNode()
            RmDriverRobotAdapter(node)

        for topic in (
            "/rm_driver/movej_result",
            "/rm_driver/movej_p_result",
            "/rm_driver/movel_result",
            "/rm_driver/movel_offset_result",
            "/rm_driver/set_gripper_position_result",
            "/rm_driver/move_stop_result",
        ):
            self.assertIs(
                node.subscriptions[topic].callback_group,
                node.rm_driver_callback_group,
            )
            self.assertIsNot(
                node.subscriptions[topic].callback_group,
                node.default_callback_group,
            )

    def test_move_joint_pose_publishes_movejp_fields(self):
        with patch.dict(sys.modules, fake_ros_modules()):
            from rg_cabinet_ros2.backends import RmDriverRobotAdapter

            node = FakeNode()
            adapter = RmDriverRobotAdapter(node)
            adapter.move_joint_pose(
                {"x": 0.1, "y": 0.2, "z": 0.3},
                {"x": 0.0, "y": 0.0, "z": 0.0, "w": 2.0},
                speed=0.05,
                wait=False,
            )

        published = node.publishers["/rm_driver/movej_p_cmd"].messages[-1]
        self.assertIsInstance(published, FakeMovejp)
        self.assertAlmostEqual(published.pose.position.x, 0.1)
        self.assertAlmostEqual(published.pose.position.y, 0.2)
        self.assertAlmostEqual(published.pose.position.z, 0.3)
        self.assertAlmostEqual(published.pose.orientation.x, 0.0)
        self.assertAlmostEqual(published.pose.orientation.y, 0.0)
        self.assertAlmostEqual(published.pose.orientation.z, 0.0)
        self.assertAlmostEqual(published.pose.orientation.w, 1.0)
        self.assertEqual(published.speed, 20)
        self.assertEqual(published.trajectory_connect, 0)
        self.assertIs(published.block, False)

    def test_move_joint_pose_waits_for_success_result(self):
        with patch.dict(sys.modules, fake_ros_modules()):
            from rg_cabinet_ros2.backends import RmDriverRobotAdapter

            node = FakeNode()
            node.auto_results["/rm_driver/movej_p_cmd"] = (
                "/rm_driver/movej_p_result",
                True,
            )
            adapter = RmDriverRobotAdapter(node)
            adapter.move_joint_pose(
                {"x": 0.1, "y": 0.2, "z": 0.3},
                {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0},
                speed=0.05,
                wait=True,
            )

        published = node.publishers["/rm_driver/movej_p_cmd"].messages[-1]
        self.assertIsInstance(published, FakeMovejp)

    def test_move_joint_pose_reports_movej_p_failure(self):
        with patch.dict(sys.modules, fake_ros_modules()):
            from rg_cabinet_ros2.backends import RmDriverRobotAdapter

            node = FakeNode()
            node.auto_results["/rm_driver/movej_p_cmd"] = (
                "/rm_driver/movej_p_result",
                False,
            )
            adapter = RmDriverRobotAdapter(node)
            with self.assertRaisesRegex(
                RuntimeError,
                "rm_driver reported movej_p failure",
            ):
                adapter.move_joint_pose(
                    {"x": 0.1, "y": 0.2, "z": 0.3},
                    {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0},
                    speed=0.05,
                    wait=True,
                )

    def test_move_pose_uses_adaptive_timeout_for_long_distance(self):
        with patch.dict(sys.modules, fake_ros_modules()):
            from rg_cabinet.geometry import Pose
            from rg_cabinet_ros2.backends import RmDriverRobotAdapter

            node = FakeNode()
            node.latest_tcp_pose = Pose(0.0, 0.0, 0.0, frame="base_link")
            node.auto_results["/rm_driver/movel_cmd"] = (
                "/rm_driver/movel_result",
                True,
            )
            adapter = RmDriverRobotAdapter(node)
            adapter.move_pose(
                Pose(0.5, 0.0, 0.0, frame="base_link"),
                speed=0.02,
                wait=True,
            )

        debug = node.last_motion_command_debug
        self.assertAlmostEqual(debug["distance_m"], 0.5)
        self.assertAlmostEqual(debug["timeout_s"], 60.0)
        self.assertAlmostEqual(debug["start_pose"][0], 0.0)
        self.assertTrue(debug["result"])
        self.assertIn("elapsed_s", debug)
        self.assertEqual(
            [payload["event"] for payload in node.status_payloads],
            ["motion_command_started", "motion_command_completed"],
        )

    def test_move_pose_timeout_never_below_base_result_timeout(self):
        with patch.dict(sys.modules, fake_ros_modules()):
            from rg_cabinet.geometry import Pose
            from rg_cabinet_ros2.backends import RmDriverRobotAdapter

            node = FakeNode()
            node.latest_tcp_pose = Pose(0.0, 0.0, 0.0, frame="base_link")
            node.auto_results["/rm_driver/movel_cmd"] = (
                "/rm_driver/movel_result",
                True,
            )
            adapter = RmDriverRobotAdapter(node)
            adapter.move_pose(
                Pose(0.01, 0.0, 0.0, frame="base_link"),
                speed=0.02,
                wait=True,
            )

        debug = node.last_motion_command_debug
        self.assertAlmostEqual(debug["distance_m"], 0.01)
        self.assertAlmostEqual(debug["timeout_s"], 20.0)
        self.assertTrue(debug["result"])

    def test_servo_twist_uses_movel_fallback_by_default(self):
        with patch.dict(sys.modules, fake_ros_modules()):
            from rg_cabinet.geometry import Pose, Twist
            from rg_cabinet_ros2.backends import RmDriverRobotAdapter

            node = FakeNode()
            node.latest_tcp_pose = Pose(0.0, 0.0, 0.0, frame="base_link")
            node.auto_results["/rm_driver/movel_cmd"] = (
                "/rm_driver/movel_result",
                True,
            )
            adapter = RmDriverRobotAdapter(node)
            adapter.servo_twist(Twist(linear_x=0.01), duration_s=25.0, frame="base")

        debug = node.last_motion_command_debug
        self.assertEqual(debug["command"], "servo_twist")
        self.assertEqual(debug["servo_motion_mode"], "movel")
        self.assertEqual(debug["topic"], "/rm_driver/movel_cmd")
        self.assertEqual(debug["result_name"], "movel")
        self.assertAlmostEqual(debug["distance_m"], 0.25)
        self.assertAlmostEqual(debug["timeout_s"], 35.0)
        self.assertAlmostEqual(debug["target_pose"][0], 0.25)
        self.assertTrue(debug["result"])
        self.assertEqual(node.publishers["/rm_driver/movel_offset_cmd"].messages, [])

    def test_servo_twist_can_use_movel_offset_mode(self):
        with patch.dict(sys.modules, fake_ros_modules()):
            from rg_cabinet.geometry import Pose, Twist
            from rg_cabinet_ros2.backends import RmDriverRobotAdapter

            node = FakeNode()
            node.parameters["rm_driver_servo_motion_mode"] = "movel_offset"
            node.latest_tcp_pose = Pose(0.0, 0.0, 0.0, frame="base_link")
            node.auto_results["/rm_driver/movel_offset_cmd"] = (
                "/rm_driver/movel_offset_result",
                True,
            )
            adapter = RmDriverRobotAdapter(node)
            adapter.servo_twist(Twist(linear_x=0.01), duration_s=25.0, frame="base")

        debug = node.last_motion_command_debug
        self.assertEqual(debug["command"], "servo_twist")
        self.assertEqual(debug["servo_motion_mode"], "movel_offset")
        self.assertEqual(debug["topic"], "/rm_driver/movel_offset_cmd")
        self.assertEqual(debug["result_name"], "movel_offset")
        self.assertAlmostEqual(debug["distance_m"], 0.25)
        self.assertAlmostEqual(debug["timeout_s"], 35.0)
        self.assertTrue(debug["result"])

    def test_tool_frame_servo_twist_maps_translation_to_current_pose(self):
        with patch.dict(sys.modules, fake_ros_modules()):
            from rg_cabinet.geometry import Pose, Twist
            from rg_cabinet_ros2.backends import RmDriverRobotAdapter

            node = FakeNode()
            node.latest_tcp_pose = Pose(
                0.0,
                0.0,
                0.0,
                0.0,
                -1.5708,
                0.0,
                frame="base_link",
            )
            node.auto_results["/rm_driver/movel_cmd"] = (
                "/rm_driver/movel_result",
                True,
            )
            adapter = RmDriverRobotAdapter(node)
            adapter.servo_twist(Twist(linear_z=0.01), duration_s=1.0, frame="tool")

        debug = node.last_motion_command_debug
        self.assertAlmostEqual(debug["target_pose"][0], -0.01, places=4)
        self.assertAlmostEqual(debug["target_pose"][1], 0.0, places=4)
        self.assertAlmostEqual(debug["target_pose"][2], 0.0, places=4)

    def test_timeout_records_debug_and_sends_stop(self):
        with patch.dict(sys.modules, fake_ros_modules()):
            from rg_cabinet.geometry import Pose
            from rg_cabinet_ros2.backends import RmDriverRobotAdapter

            node = FakeNode()
            node.parameters.update(
                {
                    "rm_driver_result_timeout_s": 0.01,
                    "rm_driver_motion_timeout_margin_s": 0.0,
                    "rm_driver_motion_timeout_scale": 1.0,
                    "rm_driver_motion_timeout_max_s": 0.01,
                    "rm_driver_stop_on_timeout": True,
                }
            )
            node.latest_tcp_pose = Pose(0.0, 0.0, 0.0, frame="base_link")
            adapter = RmDriverRobotAdapter(node)
            with self.assertRaisesRegex(
                RuntimeError,
                "Timed out waiting 0.010s for rm_driver movel result",
            ):
                adapter.move_pose(
                    Pose(0.0, 0.0, 0.0, frame="base_link"),
                    speed=0.02,
                    wait=True,
                )

        debug = node.last_motion_command_debug
        self.assertEqual(debug["result"], "timeout")
        self.assertAlmostEqual(debug["timeout_s"], 0.01)
        self.assertIn("elapsed_s", debug)
        self.assertAlmostEqual(debug["distance_m"], 0.0)
        self.assertTrue(debug["stop_sent"])
        self.assertEqual(node.stop_count, 1)

    def test_socket_json_gripper_backend_uses_controller_client(self):
        class FakeSocketGripperClient:
            instances = []

            def __init__(self, config):
                self.config = config
                self.positions = []
                self.instances.append(self)

            def set_normalized_position(self, position):
                self.positions.append(position)
                return {
                    "normalized_position": position,
                    "raw_target": 4500,
                    "pos_open": self.config.pos_open,
                    "pos_close": self.config.pos_close,
                }

        with patch.dict(sys.modules, fake_ros_modules()):
            from rg_cabinet_ros2.backends import RmDriverRobotAdapter

            node = FakeNode()
            node.parameters.update(
                {
                    "gripper_backend": "socket_json",
                    "gripper_ip": "10.0.0.5",
                    "gripper_port": 9090,
                    "gripper_device_id": 2,
                    "gripper_pos_open": 3000,
                    "gripper_pos_close": 5000,
                }
            )
            with patch(
                "rg_cabinet_ros2.backends.SocketJsonGripperClient",
                FakeSocketGripperClient,
            ):
                adapter = RmDriverRobotAdapter(node)
                adapter.set_gripper(0.75)

        client = FakeSocketGripperClient.instances[-1]
        self.assertEqual(client.positions, [0.75])
        self.assertEqual(client.config.ip, "10.0.0.5")
        self.assertEqual(client.config.port, 9090)
        self.assertEqual(client.config.device_id, 2)
        self.assertEqual(client.config.pos_open, 3000)
        self.assertEqual(client.config.pos_close, 5000)
        self.assertEqual(adapter.last_gripper_debug["normalized_position"], 0.75)
        self.assertEqual(adapter.last_gripper_debug["raw_target"], 4500)
        self.assertEqual(
            node.publishers["/rm_driver/set_gripper_position_cmd"].messages,
            [],
        )

    def test_rm_driver_gripper_backend_still_publishes_topic_command(self):
        with patch.dict(sys.modules, fake_ros_modules()):
            from rg_cabinet_ros2.backends import RmDriverRobotAdapter

            node = FakeNode()
            node.parameters["gripper_backend"] = "rm_driver"
            node.auto_results["/rm_driver/set_gripper_position_cmd"] = (
                "/rm_driver/set_gripper_position_result",
                True,
            )
            adapter = RmDriverRobotAdapter(node)
            adapter.set_gripper(1.0)

        published = node.publishers["/rm_driver/set_gripper_position_cmd"].messages[-1]
        self.assertIsInstance(published, FakeGripperset)
        self.assertEqual(published.position, 1)
        self.assertIs(published.block, True)
        self.assertEqual(published.timeout, 3000)


if __name__ == "__main__":
    unittest.main()
