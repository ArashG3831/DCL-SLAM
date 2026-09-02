"""Executor lifecycle regressions for the small project ROS nodes."""

import inspect
import time
from types import SimpleNamespace

import rclpy
from rclpy.executors import SingleThreadedExecutor

from my_epuck_project.d500_scan_fix import (
    CORRECTED_SCAN_QOS,
    D500ScanFix,
    LATEST_SCAN_QOS,
    LATEST_NAV_SCAN_QOS,
    RAW_SCAN_BEST_EFFORT_QOS,
    RAW_SCAN_QOS,
    raw_scan_qos,
    shutdown_node as shutdown_scan_fix,
)
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


def test_d500_corrected_scan_uses_bounded_reliable_qos():
    assert CORRECTED_SCAN_QOS.depth == 100
    assert CORRECTED_SCAN_QOS.reliability.value == 1  # RELIABLE
    assert CORRECTED_SCAN_QOS.durability.value == 2  # VOLATILE
    assert LATEST_SCAN_QOS.depth == 1
    assert LATEST_SCAN_QOS.reliability.value == 1  # RELIABLE
    assert LATEST_NAV_SCAN_QOS.depth == 1
    assert LATEST_NAV_SCAN_QOS.reliability.value == 2  # BEST_EFFORT


def test_raw_scan_input_reliability_selects_the_requested_qos():
    assert raw_scan_qos('reliable') is RAW_SCAN_QOS
    assert raw_scan_qos('best_effort') is RAW_SCAN_BEST_EFFORT_QOS
    assert RAW_SCAN_BEST_EFFORT_QOS.depth == 1
    assert RAW_SCAN_BEST_EFFORT_QOS.reliability.value == 2  # BEST_EFFORT


def test_scan_fixer_drops_zero_simulation_timestamp_before_publish():
    rclpy.init()
    node = D500ScanFix()
    msg = SimpleNamespace(
        header=SimpleNamespace(
            stamp=SimpleNamespace(sec=0, nanosec=0), frame_id='diag_lidar'),
        ranges=[1.0, 1.0], intensities=[], time_increment=0.0,
        scan_time=0.1, range_min=0.02, range_max=12.0)
    node.callback(msg)
    assert node._received_count == 1
    assert node._published_count == 0
    assert node._zero_stamp_drops == 1
    shutdown_scan_fix(node)


def test_nav_relay_downsampling_is_explicitly_separate_from_slam_stream():
    source = inspect.getsource(D500ScanFix)
    assert 'output_sample_count' in source
    assert 'secondary_output_sample_count' in source
    assert 'secondary_pub' in source
    assert 'Uniformly retain the corrected angular support' in source


def test_twist_stamper_uses_explicit_executor_and_idempotent_teardown():
    assert 'rclpy.spin(' not in inspect.getsource(__import__(
        'my_epuck_project.twist_stamper', fromlist=['main']))
    rclpy.init()
    node = TwistStamper()
    _exercise_shutdown(node, shutdown_stamper)


def test_twist_stamper_clears_command_when_odom_is_stale(monkeypatch):
    """A stalled odometry stream cannot leave the last turn active forever."""
    rclpy.init()
    node = TwistStamper.__new__(TwistStamper)
    published = []
    node.telemetry_timeout_s = 2.0
    node._last_command_wall_s = 98.0
    node._last_odom_wall_s = 95.0
    node._command_active = True
    node._watchdog_stopped = False
    node.pub = SimpleNamespace(publish=published.append)
    node.get_clock = lambda: SimpleNamespace(
        now=lambda: SimpleNamespace(
            to_msg=lambda: SimpleNamespace(sec=1, nanosec=0)))
    node.get_logger = lambda: SimpleNamespace(error=lambda *args: None)
    monkeypatch.setattr(time, 'monotonic', lambda: 100.0)
    node.watchdog_callback()
    assert len(published) == 1
    assert published[0].twist.linear.x == 0.0
    assert published[0].twist.angular.z == 0.0
    assert node._command_active is False
    assert node._watchdog_stopped is True
    rclpy.shutdown()


def test_twist_stamper_odom_callback_refreshes_wall_liveness(monkeypatch):
    node = TwistStamper.__new__(TwistStamper)
    monkeypatch.setattr(time, 'monotonic', lambda: 123.0)
    node.odom_callback(SimpleNamespace())
    assert node._last_odom_wall_s == 123.0


def test_nodes_tolerate_context_already_shutdown():
    rclpy.init()
    scan_node = D500ScanFix()
    rclpy.shutdown()
    shutdown_scan_fix(scan_node)

    rclpy.init()
    stamp_node = TwistStamper()
    rclpy.shutdown()
    shutdown_stamper(stamp_node)


def test_slam_wrapper_owns_signal_shutdown_and_destroys_before_shutdown():
    """The C++ wrapper must avoid invalid-context teardown aborts."""
    from pathlib import Path

    source = (Path(__file__).parents[2] /
              'reliable_slam_toolbox_wrapper' / 'src' /
              'reliable_async_slam_toolbox_node.cpp').read_text(
                  encoding='utf-8')
    assert 'SignalHandlerOptions::None' in source
    assert 'executor.spin_some' in source
    assert 'node.reset();' in source
    assert source.index('node.reset();') < source.index('rclcpp::shutdown();')
