"""Deterministic tests for the cooperative TF startup contract."""

from geometry_msgs.msg import TransformStamped
from rclpy.duration import Duration
from rclpy.time import Time
from tf2_ros import Buffer

from my_epuck_project.cooperative_regression import (
    TF_READINESS_TIMEOUT_S,
    tf_readiness_requirements,
)


def _transform(parent, child, seconds):
    message = TransformStamped()
    message.header.frame_id = parent
    message.child_frame_id = child
    message.header.stamp.sec = seconds
    message.transform.rotation.w = 1.0
    return message


def test_tf_readiness_contract_uses_namespaced_shared_map_chain():
    assert tf_readiness_requirements() == [
        ('robot1/base_footprint', 'robot1/odom', 'odom_to_base'),
        ('shared_map', 'robot1/base_footprint', 'shared_to_base'),
        ('robot2/base_footprint', 'robot2/odom', 'odom_to_base'),
        ('shared_map', 'robot2/base_footprint', 'shared_to_base'),
    ]
    assert TF_READINESS_TIMEOUT_S == 10.0


def test_future_dated_slam_transform_requires_temporal_overlap():
    """A fresh buffer must wait for overlapping map->odom and odom->base."""
    buffer = Buffer()
    buffer.set_transform_static(
        _transform('shared_map', 'robot1/map', 0), 'tf-readiness-fixture')
    buffer.set_transform(
        _transform('robot1/map', 'robot1/odom', 17), 'tf-readiness-fixture')
    buffer.set_transform(
        _transform('robot1/odom', 'robot1/base_footprint', 15),
        'tf-readiness-fixture')

    assert not buffer.can_transform(
        'shared_map', 'robot1/base_footprint', Time(),
        timeout=Duration(seconds=0.0))

    buffer.set_transform(
        _transform('robot1/map', 'robot1/odom', 16), 'tf-readiness-fixture')
    buffer.set_transform(
        _transform('robot1/odom', 'robot1/base_footprint', 16),
        'tf-readiness-fixture')

    assert buffer.can_transform(
        'shared_map', 'robot1/base_footprint', Time(),
        timeout=Duration(seconds=0.0))
