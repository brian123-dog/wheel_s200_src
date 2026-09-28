"""Run with an isolated ROS_DOMAIN_ID; task execution is replaced with stubs."""
from types import SimpleNamespace
from unittest.mock import Mock, patch

import pytest

rclpy = pytest.importorskip('rclpy')
pytest.importorskip('rm_ros_interfaces.msg')
from rclpy.parameter import Parameter
from rg_cabinet_ros2 import task_node
from rg_cabinet_ros2.backends import RmDriverRobotAdapter


@pytest.fixture
def node():
    rclpy.init(args=[])
    n = task_node.CabinetTaskNode(parameter_overrides=[Parameter('backend', value='rm_driver')])
    n._publish_status = Mock()
    try:
        yield n
    finally:
        n.destroy_node()
        rclpy.shutdown()


def entities(node):
    return len(list(node.subscriptions)), len(list(node.publishers)), len(list(node.waitables))


@pytest.mark.parametrize('qr', [False, True])
@pytest.mark.parametrize('fails', [False, True])
def test_repeated_sequences_release_resources_on_success_and_failure(node, qr, fails):
    baseline = entities(node)
    seen = []
    def sequence(config, robot, **kwargs):
        assert isinstance(robot, RmDriverRobotAdapter)
        assert entities(node)[0] == baseline[0] + 6
        client = Mock()
        robot._socket_gripper = client
        seen.append(client)
        if fails:
            raise RuntimeError('simulated vision failure')
        return []
    targets = SimpleNamespace(source_file='fixture.json', poses={})
    with patch.object(task_node, 'load_task_config', return_value={}), \
         patch.object(task_node, 'load_qr_target_set', return_value=targets), \
         patch.object(task_node, 'overlay_qr_observation_poses', return_value={}), \
         patch.object(task_node, 'run_sequence', side_effect=sequence):
        execute = node.execute_sequence_from_qr if qr else node.execute_configured_sequence
        for _ in range(8):
            if fails:
                with pytest.raises(RuntimeError, match='simulated vision failure'):
                    execute()
            else:
                assert execute()[0]
            assert entities(node) == baseline
            seen[-1].close.assert_called_once_with()


def test_close_is_idempotent_and_keeps_node_feedback_subscriptions(node):
    baseline = entities(node)
    robot = RmDriverRobotAdapter(node)
    assert entities(node)[0] == baseline[0] + 6
    assert entities(node)[1] == baseline[1] + 5
    robot.close()
    robot.close()
    assert entities(node) == baseline


def test_task_cleanup_preserves_cached_manual_adapter(node):
    manual = node._manual_robot()
    baseline = entities(node)
    with node._sequence_robot_adapter('rm_driver') as robot:
        assert robot is not manual
    assert node._manual_robot() is manual
    assert entities(node) == baseline
    assert len(manual._subscriptions) == 6
    manual.close()


def test_initialization_after_adapter_creation_also_releases_resources(node):
    baseline = entities(node)
    with patch.object(task_node, 'load_task_config', return_value={}), \
         patch.object(node.vision_target_resolver, 'reset', side_effect=RuntimeError('reset failure')):
        with pytest.raises(RuntimeError, match='reset failure'):
            node.execute_configured_sequence()
    assert entities(node) == baseline
