from __future__ import annotations

from typing import Sequence

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


DEFAULT_GRIPPER_JOINT_NAMES = [
    "Left_1_Joint",
    "Left_Support_Joint",
    "Left_2_Joint",
    "Right_1_Joint",
    "Right_2_Joint",
    "Right_Support_Joint",
]

DEFAULT_GRIPPER_JOINT_POSITIONS = [
    0.0,
    0.0,
    -0.0,
    0.0,
    0.0,
    0.0,
]


class GripperJointStateNode(Node):
    """Publish default joint states for the passive gripper visualization model."""

    def __init__(self) -> None:
        super().__init__("gripper_joint_state_node")
        self.declare_parameter("joint_names", DEFAULT_GRIPPER_JOINT_NAMES)
        self.declare_parameter("joint_positions", DEFAULT_GRIPPER_JOINT_POSITIONS)
        self.declare_parameter("publish_rate_hz", 10.0)

        self.joint_names = [str(v) for v in self.get_parameter("joint_names").value]
        self.joint_positions = [
            float(v) for v in self.get_parameter("joint_positions").value
        ]
        self._validate_configuration(self.joint_names, self.joint_positions)

        rate_hz = float(self.get_parameter("publish_rate_hz").value)
        period_s = 1.0 / max(rate_hz, 0.1)
        self.publisher = self.create_publisher(JointState, "/joint_states", 10)
        self.timer = self.create_timer(period_s, self._publish_joint_state)
        self.get_logger().info(
            "Publishing passive gripper joint states for "
            f"{', '.join(self.joint_names)}"
        )

    def _publish_joint_state(self) -> None:
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(self.joint_names)
        msg.position = list(self.joint_positions)
        self.publisher.publish(msg)

    @staticmethod
    def _validate_configuration(
        joint_names: Sequence[str],
        joint_positions: Sequence[float],
    ) -> None:
        if len(joint_names) != len(joint_positions):
            raise ValueError(
                "joint_names and joint_positions must have the same length"
            )
        if not joint_names:
            raise ValueError("At least one gripper joint must be configured")


def main() -> None:
    rclpy.init()
    node = GripperJointStateNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
