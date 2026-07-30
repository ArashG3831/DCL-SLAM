"""Executor lifecycle regressions for the small project ROS nodes."""

import inspect

import rclpy
from rclpy.executors import SingleThreadedExecutor

from my_epuck_project.d500_scan_fix import D500ScanFix, shutdown_node as shutdown_scan_fix
from my_epuck_project.twist_stamper import TwistStamper, shutdown_node as shutdown_stamper


def _exercise_shutdown(node, shutdown):
    executor = SingleThreadedExecutor(context=node.context)
    executor.add_node(node)
    shutdown(node, executor)
    shutdown(node, executor)


def test_d500_scan_fix_uses_explicit_executor_and_idempotent_teardown():
    assert 'rclpy.spin(' not in inspect.getsource(__import__(
        'my_epuck_project.d500_scan_fix', fromlist=['main']))
    rclpy.init()
    node = D500ScanFix()
    _exercise_shutdown(node, shutdown_scan_fix)


def test_twist_stamper_uses_explicit_executor_and_idempotent_teardown():
    assert 'rclpy.spin(' not in inspect.getsource(__import__(
        'my_epuck_project.twist_stamper', fromlist=['main']))
    rclpy.init()
    node = TwistStamper()
    _exercise_shutdown(node, shutdown_stamper)


def test_nodes_tolerate_context_already_shutdown():
    rclpy.init()
    scan_node = D500ScanFix()
    rclpy.shutdown()
    shutdown_scan_fix(scan_node)

    rclpy.init()
    stamp_node = TwistStamper()
    rclpy.shutdown()
    shutdown_stamper(stamp_node)
