#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from std_srvs.srv import Empty
from base_interfaces.srv import ArmTaskExecute


class ArmTaskDummyServiceNode(Node):
    """Dummy arm task services for navigation-only mode."""

    def __init__(self):
        super().__init__('arm_task_dummy_service_node')
        self._task_id = 0

        self._empty_srv = self.create_service(
            Empty,
            '/rm_group/arm_task_execute',
            self._handle_empty,
        )

        self._detailed_srv = self.create_service(
            ArmTaskExecute,
            '/rm_group/arm_task_execute_detailed',
            self._handle_detailed,
        )

        self.get_logger().info('Dummy arm services ready (no hardware mode)')

    def _handle_empty(self, request, response):
        del request
        self._task_id += 1
        self.get_logger().info(f'Dummy arm task accepted, task_id={self._task_id}')
        return response

    def _handle_detailed(self, request, response):
        self._task_id += 1
        response.task_id = self._task_id
        response.arm_task_start = True
        response.arm_task_done = True
        response.success = True
        response.failed_stage = 0
        response.message = f'dummy success (waypoint_index={request.waypoint_index})'
        self.get_logger().info(
            f'Dummy detailed arm task success, task_id={self._task_id}, waypoint_index={request.waypoint_index}'
        )
        return response


def main(args=None):
    rclpy.init(args=args)
    node = ArmTaskDummyServiceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
