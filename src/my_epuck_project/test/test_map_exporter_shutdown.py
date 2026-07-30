"""Deterministic context-safe map exporter teardown tests."""

import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.parameter import Parameter

from my_epuck_project.map_exporter import MapExporter, shutdown_node


def exporter_node():
    return MapExporter(parameter_overrides=[
        Parameter('source_robot_id', Parameter.Type.STRING, 'robot1'),
    ])


def test_buffered_map_exports_once_before_context_shutdown():
    rclpy.init()
    node = exporter_node()
    try:
        node.latest_map = OccupancyGrid()
        assert node.finalize() is True
        assert node.revision == 1
        assert node.finalize() is False
    finally:
        shutdown_node(node)


def test_invalid_context_teardown_does_not_use_ros_entities():
    rclpy.init()
    node = exporter_node()
    rclpy.shutdown()
    shutdown_node(node)
    assert node._finalized is True
