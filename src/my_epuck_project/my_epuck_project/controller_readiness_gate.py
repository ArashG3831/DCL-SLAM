"""Two-robot controller-manager readiness gate."""

from __future__ import annotations

import time

import rclpy
from controller_manager_msgs.srv import ListControllers
from rclpy.node import Node


def active_required(states, required=('diffdrive_controller', 'joint_state_broadcaster')):
    return all(states.get(name) == 'active' for name in required)


class ControllerReadinessGate(Node):
    def __init__(self):
        super().__init__('controller_readiness_gate')
        self.declare_parameter('robot_ids', ['robot1', 'robot2'])
        self.declare_parameter('timeout_s', 120.0)
        self.robot_ids = list(self.get_parameter('robot_ids').value)
        self.timeout_s = float(self.get_parameter('timeout_s').value)
        self.service_clients = {
            robot: self.create_client(
                ListControllers, f'/{robot}/controller_manager/list_controllers')
            for robot in self.robot_ids}

    def run(self):
        deadline = time.monotonic() + self.timeout_s
        while time.monotonic() < deadline and rclpy.ok():
            ready = True
            observed = {}
            for robot, client in self.service_clients.items():
                if not client.wait_for_service(timeout_sec=0.5):
                    ready = False
                    observed[robot] = 'SERVICE_UNAVAILABLE'
                    continue
                future = client.call_async(ListControllers.Request())
                rclpy.spin_until_future_complete(self, future, timeout_sec=0.8)
                if not future.done() or future.result() is None:
                    ready = False
                    observed[robot] = 'NO_RESPONSE'
                    continue
                states = {item.name: item.state for item in future.result().controller}
                observed[robot] = states
                if not active_required(states):
                    ready = False
            if ready:
                self.get_logger().info(
                    f'Both robot controller managers READY: {observed}')
                return 0
            time.sleep(0.2)
        self.get_logger().error(
            f'Controller readiness timeout after {self.timeout_s}s: {observed}')
        return 1


def main(args=None):
    rclpy.init(args=args)
    node = ControllerReadinessGate()
    try:
        return node.run()
    except KeyboardInterrupt:
        return 130
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
