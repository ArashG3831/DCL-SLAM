"""Focused geometry and pending-queue tests for teammate scan filtering."""

import inspect
import math

import pytest
from sensor_msgs.msg import LaserScan

from my_epuck_project.teammate_scan_filter import (
    PendingScanQueue,
    Transform2D,
    circle_first_intersection,
    compose_transform,
    mask_teammate_returns,
)


def scan(ranges, angle_min=0.0, increment=0.0, intensities=None):
    message = LaserScan()
    message.header.stamp.sec = 12
    message.header.stamp.nanosec = 345
    message.header.frame_id = 'robot1/d500_lidar'
    message.angle_min = angle_min
    message.angle_max = angle_min + increment * max(0, len(ranges) - 1)
    message.angle_increment = increment
    message.time_increment = 0.0001
    message.scan_time = 0.1
    message.range_min = 0.03
    message.range_max = 12.0
    message.ranges = list(ranges)
    message.intensities = [] if intensities is None else list(intensities)
    return message


def assert_scan_preserved(source, output, masked=()):
    masked = set(masked)
    assert output.header == source.header
    assert output.angle_min == source.angle_min
    assert output.angle_max == source.angle_max
    assert output.angle_increment == source.angle_increment
    assert output.time_increment == source.time_increment
    assert output.scan_time == source.scan_time
    assert output.range_min == source.range_min
    assert output.range_max == source.range_max
    assert output.intensities == source.intensities
    for index, value in enumerate(source.ranges):
        if index in masked:
            assert math.isnan(output.ranges[index])
        elif math.isnan(value):
            assert math.isnan(output.ranges[index])
        else:
            assert output.ranges[index] == value


def filter_one(value, angle=0.0, tolerance=0.005):
    message = scan([value], angle_min=angle)
    output, indices = mask_teammate_returns(
        message, 1.0, 0.0, 0.1, tolerance)
    return message, output, indices


def test_finite_return_at_first_circle_intersection_is_masked():
    message, output, indices = filter_one(0.9)
    assert indices == [0]
    assert_scan_preserved(message, output, indices)


def test_finite_return_within_tolerance_is_masked():
    message, output, indices = filter_one(0.904)
    assert indices == [0]
    assert_scan_preserved(message, output, indices)


def test_wall_before_first_intersection_is_preserved():
    message, output, indices = filter_one(0.5)
    assert indices == []
    assert_scan_preserved(message, output)


def test_return_inside_teammate_envelope_is_masked():
    message, output, indices = filter_one(1.0, tolerance=0.01)
    assert indices == [0]
    assert_scan_preserved(message, output, indices)


def test_return_beyond_teammate_is_preserved():
    message, output, indices = filter_one(1.2)
    assert indices == []
    assert_scan_preserved(message, output)


def test_ray_missing_circle_is_preserved():
    message, output, indices = filter_one(1.0, angle=math.pi / 2.0)
    assert indices == []
    assert circle_first_intersection(1.0, 0.0, 0.1, math.pi / 2.0) is None
    assert_scan_preserved(message, output)


def test_tangent_return_is_masked_at_first_intersection():
    angle = math.asin(0.1)
    expected = circle_first_intersection(1.0, 0.0, 0.1, angle)
    assert expected is not None
    message, output, indices = filter_one(expected, angle=angle, tolerance=1e-8)
    assert indices == [0]
    assert_scan_preserved(message, output, indices)


def test_positive_infinity_crossing_circle_is_preserved_by_masking():
    message, output, indices = filter_one(math.inf)
    assert indices == []
    assert math.isinf(output.ranges[0]) and output.ranges[0] > 0.0


@pytest.mark.parametrize('value', [
    math.nan, -math.inf, 0.0, -1.0, 12.0, 0.02, 13.0,
])
def test_nonfinite_and_ineligible_finite_values_are_preserved(value):
    message, output, indices = filter_one(value)
    assert indices == []
    assert_scan_preserved(message, output)


def test_laserscan_metadata_and_intensities_are_unchanged():
    message = scan([0.9, 0.5], intensities=[7.0, 8.0])
    output, indices = mask_teammate_returns(
        message, 1.0, 0.0, 0.1, 0.005)
    assert indices == [0]
    assert_scan_preserved(message, output, indices)


def test_robot_behind_sensor_has_no_intersection():
    assert circle_first_intersection(-1.0, 0.0, 0.1, 0.0) is None


def test_negative_peer_coordinates_and_wrapped_beam_angle_work():
    center = -1.0
    angle = 5.0 * math.pi / 4.0
    expected = math.sqrt(2.0) - 0.1
    assert circle_first_intersection(center, center, 0.1, angle) == pytest.approx(
        expected)
    message = scan([expected], angle_min=angle)
    output, indices = mask_teammate_returns(
        message, center, center, 0.1, 0.005)
    assert indices == [0]
    assert math.isnan(output.ranges[0])


def stamped_scan(seconds, nanoseconds=0):
    message = scan([0.9])
    message.header.stamp.sec = seconds
    message.header.stamp.nanosec = nanoseconds
    return message


def test_queue_waits_for_transform_then_publishes_once():
    queue = PendingScanQueue(4)
    message = stamped_scan(20, 123)
    queue.enqueue(message, 1.0)
    assert queue.take(1.05, 0.2, False)[0] == 'wait'
    action, item, waited = queue.take(1.08, 0.2, True)
    assert action == 'publish'
    assert item[0] is message
    assert waited == pytest.approx(0.08)
    assert queue.take(1.09, 0.2, True)[0] == 'idle'


def test_queue_drops_after_transform_deadline():
    queue = PendingScanQueue(4)
    queue.enqueue(stamped_scan(1), 2.0)
    action, item, waited = queue.take(2.201, 0.2, False)
    assert action == 'expired'
    assert item[0].header.stamp.sec == 1
    assert waited > 0.2
    assert not queue.items


def test_queue_overflow_drops_oldest_scan():
    queue = PendingScanQueue(2)
    assert queue.enqueue(stamped_scan(1), 1.0) is None
    assert queue.enqueue(stamped_scan(2), 2.0) is None
    dropped = queue.enqueue(stamped_scan(3), 3.0)
    assert dropped[0].header.stamp.sec == 1
    assert [item[0].header.stamp.sec for item in queue.items] == [2, 3]


def test_queue_has_no_degraded_unmasked_publication_action():
    queue = PendingScanQueue(4)
    queue.enqueue(stamped_scan(1), 1.0)
    assert queue.take(1.1, 0.2, False)[0] == 'wait'
    assert tuple(inspect.signature(queue.take).parameters) == (
        'now', 'maximum_latency', 'pose_available')


def assert_transform_close(actual, expected, tolerance=1e-9):
    assert math.isclose(actual.x, expected.x, abs_tol=tolerance)
    assert math.isclose(actual.y, expected.y, abs_tol=tolerance)
    assert math.isclose(actual.yaw, expected.yaw, abs_tol=tolerance)


def test_independent_odometry_transform_composition_is_correct():
    lidar_from_own_odom = Transform2D(-0.02, 0.0, 0.0)
    own_odom_from_peer_odom = Transform2D(-0.30, 0.0, math.pi)
    peer_odom_from_peer_base = Transform2D(0.04, 0.0, 0.1)
    result = compose_transform(
        compose_transform(lidar_from_own_odom, own_odom_from_peer_odom),
        peer_odom_from_peer_base,
    )
    assert_transform_close(result, Transform2D(-0.36, 0.0, -math.pi + 0.1))
