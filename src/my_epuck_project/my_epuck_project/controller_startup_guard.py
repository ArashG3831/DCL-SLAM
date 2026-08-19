"""Retrying, state-aware ros2_control controller bootstrap for Webots.

The Webots controller manager can briefly return an inconsistent loaded-state
answer while its hardware/plugin services are coming up.  This guard retries
the standard controller_manager spawner and only succeeds after the named
controller is observed ACTIVE.  It does not synthesize readiness.
"""

from __future__ import annotations

import subprocess
import sys
import time

import rclpy
from controller_manager_msgs.srv import (
    ConfigureController,
    ListControllers,
    SwitchController,
)
from rclpy.node import Node


def controller_states(response):
    return {item.name: item.state for item in response.controller}


def controller_action(state):
    if state is None:
        return 'LOAD'
    if state == 'active':
        return 'READY'
    if state == 'unconfigured':
        return 'CONFIGURE'
    if state in {'inactive', 'configured'}:
        return 'ACTIVATE'
    return 'RETRY'


class ControllerStartupGuard(Node):
    def __init__(self):
        super().__init__('controller_startup_guard')
        self.declare_parameter('controller_name', 'diffdrive_controller')
        self.declare_parameter('controller_manager', '')
        self.declare_parameter('controller_param_file', '')
        self.declare_parameter('controller_ros_args', '')
        self.declare_parameter('max_attempts', 6)
        self.declare_parameter('retry_delay_s', 1.0)
        self.declare_parameter('service_timeout_s', 3.0)
        self.controller_name = str(self.get_parameter('controller_name').value)
        self.manager = str(self.get_parameter('controller_manager').value)
        self.param_file = str(self.get_parameter('controller_param_file').value)
        self.controller_ros_args = str(
            self.get_parameter('controller_ros_args').value)
        self.max_attempts = max(1, int(self.get_parameter('max_attempts').value))
        self.retry_delay_s = max(0.1, float(self.get_parameter('retry_delay_s').value))
        self.service_timeout_s = max(0.2, float(
            self.get_parameter('service_timeout_s').value))
        if not self.manager.startswith('/'):
            raise ValueError('controller_manager must be absolute')
        self.list_client = self.create_client(
            ListControllers, self.manager + '/list_controllers')
        self.configure_client = self.create_client(
            ConfigureController, self.manager + '/configure_controller')
        self.switch_client = self.create_client(
            SwitchController, self.manager + '/switch_controller')

    def call(self, client, request):
        if not client.wait_for_service(timeout_sec=self.service_timeout_s):
            return None
        future = client.call_async(request)
        rclpy.spin_until_future_complete(
            self, future, timeout_sec=self.service_timeout_s)
        return future.result() if future.done() else None

    def states(self):
        response = self.call(self.list_client, ListControllers.Request())
        return controller_states(response) if response is not None else None

    def invoke_standard_spawner(self):
        command = [
            sys.executable, '-m', 'controller_manager.spawner',
            self.controller_name,
            '--controller-manager', self.manager,
            '--controller-manager-timeout', str(int(self.service_timeout_s)),
        ]
        if self.param_file:
            command.extend(['--param-file', self.param_file])
        if self.controller_ros_args:
            command.extend(['--controller-ros-args', self.controller_ros_args])
        self.get_logger().info(
            f'Loading {self.controller_name} through controller_manager '
            f'(attempted standard spawner): {self.manager}')
        completed = subprocess.run(command, check=False)
        return completed.returncode == 0

    def configure(self):
        request = ConfigureController.Request(name=self.controller_name)
        response = self.call(self.configure_client, request)
        return response is not None and bool(response.ok)

    def activate(self):
        request = SwitchController.Request(
            activate_controllers=[self.controller_name],
            deactivate_controllers=[],
            strictness=SwitchController.Request.BEST_EFFORT,
            activate_asap=True,
        )
        request.timeout.sec = 2
        response = self.call(self.switch_client, request)
        return response is not None and bool(response.ok)

    def ensure_active(self):
        for attempt in range(1, self.max_attempts + 1):
            states = self.states()
            state = None if states is None else states.get(self.controller_name)
            action = controller_action(state)
            self.get_logger().info(
                f'{self.manager}/{self.controller_name}: state={state!r}, '
                f'action={action}, attempt={attempt}/{self.max_attempts}')
            if action == 'READY':
                return True
            if action == 'LOAD':
                self.invoke_standard_spawner()
            elif action == 'CONFIGURE':
                self.configure()
            elif action == 'ACTIVATE':
                self.activate()
            time.sleep(self.retry_delay_s)
        states = self.states()
        if states is not None and controller_action(
                states.get(self.controller_name)) == 'READY':
            self.get_logger().info(
                f'Controller reached active after final attempt: '
                f'{self.manager}/{self.controller_name}')
            return True
        self.get_logger().error(
            f'Controller did not reach active: {self.manager}/'
            f'{self.controller_name}; states={states}')
        return False


def main(args=None):
    rclpy.init(args=args)
    node = ControllerStartupGuard()
    try:
        return 0 if node.ensure_active() else 1
    except KeyboardInterrupt:
        return 130
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
