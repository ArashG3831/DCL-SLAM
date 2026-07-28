import math

from sensor_msgs.msg import LaserScan

from my_epuck_project.teammate_scan_filter import (
    Transform2D, PoseSourceTransition, beam_directions, circle_ray_interval,
    compose_transform, filtered_scan, inverse_transform, selected_indices,
    selected_indices_cached,
)


def scan(ranges, angle_min=-1.0, increment=0.5, intensities=None):
    message = LaserScan()
    message.header.stamp.sec = 12
    message.header.stamp.nanosec = 345
    message.header.frame_id = 'robot1/d500_lidar'
    message.angle_min = angle_min
    message.angle_max = angle_min + increment * max(0, len(ranges)-1)
    message.angle_increment = increment
    message.time_increment = 0.0001
    message.scan_time = 0.1
    message.range_min = 0.03
    message.range_max = 12.0
    message.ranges = list(ranges)
    message.intensities = [] if intensities is None else list(intensities)
    return message


def assert_preserved(source, output, changed):
    assert output.header == source.header
    assert output.angle_min == source.angle_min
    assert output.angle_max == source.angle_max
    assert output.angle_increment == source.angle_increment
    assert output.time_increment == source.time_increment
    assert output.scan_time == source.scan_time
    assert output.range_min == source.range_min
    assert output.range_max == source.range_max
    assert len(output.ranges) == len(source.ranges)
    assert output.intensities == source.intensities
    for index, value in enumerate(source.ranges):
        if index in changed:
            assert math.isnan(output.ranges[index])
        elif math.isnan(value):
            assert math.isnan(output.ranges[index])
        else:
            assert output.ranges[index] == value


def test_ray_miss_and_no_angle_only_mask():
    message = scan([1.0], angle_min=math.pi/2, increment=1.0)
    output, indices, _ = filtered_scan(message, 1.0, 0.0, 0.1, 0.01)
    assert indices == []
    assert_preserved(message, output, set())


def test_peer_match_only():
    message = scan([0.95], angle_min=0.0, increment=1.0)
    output, indices, intervals = filtered_scan(message, 1.0, 0.0, 0.1, 0.01)
    assert indices == [0]
    assert intervals[0] == (0.9, 1.1)
    assert_preserved(message, output, {0})


def test_static_return_before_peer_is_preserved():
    message = scan([0.50], angle_min=0.0, increment=1.0)
    output, indices, _ = filtered_scan(message, 1.0, 0.0, 0.1, 0.01)
    assert indices == []
    assert_preserved(message, output, set())


def test_return_beyond_peer_is_preserved():
    message = scan([1.40], angle_min=0.0, increment=1.0)
    output, indices, _ = filtered_scan(message, 1.0, 0.0, 0.1, 0.01)
    assert indices == []
    assert_preserved(message, output, set())


def test_tangent_ray_interval_and_match():
    angle = math.asin(0.1)
    interval = circle_ray_interval(1.0, 0.0, 0.1, angle)
    assert interval is not None
    message = scan([interval[0]], angle_min=angle, increment=1.0)
    output, indices, _ = filtered_scan(message, 1.0, 0.0, 0.1, 1e-6)
    assert indices == [0]
    assert_preserved(message, output, {0})


def test_peer_partly_outside_scan_limits():
    message = scan([0.95, 0.95], angle_min=-0.2, increment=0.2)
    output, indices, _ = filtered_scan(message, 1.0, 0.0, 0.1, 0.01)
    assert indices == [1]
    assert_preserved(message, output, {1})


def test_peer_completely_outside_scan():
    message = scan([1.0, 1.0], angle_min=1.0, increment=0.2)
    output, indices, _ = filtered_scan(message, 1.0, 0.0, 0.1, 0.01)
    assert indices == []
    assert_preserved(message, output, set())


def test_nonfinite_and_out_of_range_inputs_unchanged():
    message = scan(
        [math.nan, math.inf, 0.02, 13.0, 0.95],
        angle_min=0.0,
        increment=0.0,
    )
    output, indices, _ = filtered_scan(message, 1.0, 0.0, 0.1, 0.01)
    assert indices == [4]
    assert_preserved(message, output, {4})


def test_range_tolerance_boundaries_inclusive():
    message = scan([0.88, 1.12], angle_min=0.0, increment=0.0)
    output, indices, _ = filtered_scan(message, 1.0, 0.0, 0.1, 0.02)
    assert indices == [0, 1]
    assert_preserved(message, output, {0, 1})


def test_static_and_dynamic_margin_boundaries():
    body_radius, static_margin, dynamic_margin = 0.037, 0.003, 0.007
    radius = body_radius + static_margin + dynamic_margin
    near = 1.0 - radius
    message = scan([near], angle_min=0.0, increment=1.0)
    output, indices, intervals = filtered_scan(message, 1.0, 0.0, radius, 0.0)
    assert math.isclose(intervals[0][0], near)
    assert indices == [0]
    assert_preserved(message, output, {0})


def test_rotated_circle_geometry_and_negative_coordinates():
    # A circle is rotation invariant; negative center coordinates still use rays.
    message = scan([0.95], angle_min=-math.pi, increment=1.0)
    output, indices, _ = filtered_scan(message, -1.0, 0.0, 0.1, 0.01)
    assert indices == [0]
    assert_preserved(message, output, {0})


def test_empty_and_populated_intensities_preserved():
    empty = scan([0.95], angle_min=0.0, increment=1.0)
    output, indices, _ = filtered_scan(empty, 1.0, 0.0, 0.1, 0.01)
    assert_preserved(empty, output, set(indices))
    populated = scan([0.95, 0.50], angle_min=0.0, increment=0.0,
                     intensities=[7.0, 8.0])
    output, indices, _ = filtered_scan(populated, 1.0, 0.0, 0.1, 0.01)
    assert_preserved(populated, output, set(indices))


# Pending queue/state-machine tests intentionally use synthetic scans and time.
from my_epuck_project.teammate_scan_filter import PendingScanQueue


def stamped_scan(seconds, nanoseconds=0):
    message = scan([0.95], angle_min=0.0, increment=1.0)
    message.header.stamp.sec = seconds
    message.header.stamp.nanosec = nanoseconds
    return message


def test_pending_transform_delayed_then_publishes_once_with_original_stamp():
    queue = PendingScanQueue(4)
    message = stamped_scan(20, 123)
    queue.enqueue(message, 1.0)
    action, _, _ = queue.take(1.05, 0.2, False)
    assert action == 'wait'
    assert len(queue.items) == 1
    action, item, _ = queue.take(1.08, 0.2, True)
    assert action == 'publish'
    assert item[0].header.stamp == message.header.stamp
    assert queue.take(1.09, 0.2, True)[0] == 'idle'


def test_pending_latency_expiry_drops_without_publication():
    queue = PendingScanQueue(4)
    queue.enqueue(stamped_scan(1), 2.0)
    action, item, waited = queue.take(2.201, 0.2, True)
    assert action == 'expired'
    assert item[0].header.stamp.sec == 1
    assert waited > 0.2
    assert queue.take(3.0, 0.2, True)[0] == 'idle'


def test_pending_overflow_drops_oldest_and_counts_separately():
    queue = PendingScanQueue(2)
    assert queue.enqueue(stamped_scan(1), 1.0) is None
    assert queue.enqueue(stamped_scan(2), 2.0) is None
    dropped = queue.enqueue(stamped_scan(3), 3.0)
    assert dropped[0].header.stamp.sec == 1
    assert queue.overflow_drops == 1
    assert [item[0].header.stamp.sec for item in queue.items] == [2, 3]


def test_pending_timestamp_order_and_no_overtake():
    queue = PendingScanQueue(4)
    queue.enqueue(stamped_scan(3), 3.0)
    queue.enqueue(stamped_scan(1), 1.0)
    queue.enqueue(stamped_scan(2), 2.0)
    action, item, _ = queue.take(3.05, 10.0, False)
    assert action == 'wait' and item[0].header.stamp.sec == 1
    published = []
    for now in (3.06, 3.07, 3.08):
        action, item, _ = queue.take(now, 10.0, True)
        assert action == 'publish'
        published.append(item[0].header.stamp.sec)
    assert published == [1, 2, 3]
    assert len(set(published)) == len(published)


def test_startup_policy_queues_without_unfiltered_passthrough():
    queue = PendingScanQueue(4)
    queue.enqueue(stamped_scan(4), 4.0)
    assert queue.take(4.05, 0.2, False)[0] == 'wait'
    action, item, _ = queue.take(4.10, 0.2, True)
    assert action == 'publish'
    assert item[0].header.stamp.sec == 4


def test_latest_transform_is_never_needed_by_queue_and_shutdown_is_safe():
    queue = PendingScanQueue(4)
    queue.enqueue(stamped_scan(5), 5.0)
    assert queue.take(5.01, 0.2, False)[0] == 'wait'
    queue.clear()
    queue.clear()
    assert queue.take(5.02, 0.2, True)[0] == 'idle'


def assert_transform_close(actual, expected, tolerance=1e-9):
    assert math.isclose(actual.x, expected.x, abs_tol=tolerance)
    assert math.isclose(actual.y, expected.y, abs_tol=tolerance)
    assert math.isclose(actual.yaw, expected.yaw, abs_tol=tolerance)


def test_known_initial_odom_transform_and_robot2_inverse():
    robot1_to_robot2 = Transform2D(
        -0.299999998712, -0.000027796077, -3.1415)
    robot2_to_robot1 = inverse_transform(robot1_to_robot2)
    assert_transform_close(
        robot2_to_robot1,
        Transform2D(-0.299999999999703, -0.000000000000102, 3.1415),
        2e-12,
    )
    identity = compose_transform(robot1_to_robot2, robot2_to_robot1)
    assert_transform_close(identity, Transform2D(0.0, 0.0, 0.0), 2e-12)


def test_internal_odom_composition_needs_no_map_transform():
    lidar_from_own_odom = Transform2D(-0.02, 0.0, 0.0)
    own_odom_from_peer_odom = Transform2D(-0.30, 0.0, math.pi)
    peer_odom_from_peer_base = Transform2D(0.04, 0.0, 0.1)
    result = compose_transform(
        compose_transform(lidar_from_own_odom, own_odom_from_peer_odom),
        peer_odom_from_peer_base,
    )
    expected = Transform2D(-0.36, 0.0, -math.pi + 0.1)
    assert_transform_close(result, expected)


def test_pose_source_one_way_transition_after_consecutive_matches():
    transition = PoseSourceTransition(3, 0.05, 0.15)
    transition.odom_available()
    assert transition.state == 'odom_bootstrap'
    odom = Transform2D(-0.3, 0.0, math.pi)
    shared = Transform2D(-0.301, 0.001, math.pi - 0.01)
    assert not transition.compare_shared(odom, shared)
    assert not transition.compare_shared(odom, shared)
    assert transition.compare_shared(odom, shared)
    assert transition.state == 'shared_map_tf_active'
    discontinuous = Transform2D(2.0, 2.0, 0.0)
    assert transition.compare_shared(odom, discontinuous)
    assert transition.state == 'shared_map_tf_active'


def test_pose_source_rejects_discontinuous_transition():
    transition = PoseSourceTransition(2, 0.05, 0.15)
    transition.odom_available()
    odom = Transform2D(-0.3, 0.0, math.pi)
    assert not transition.compare_shared(odom, Transform2D(-0.1, 0.0, 0.0))
    assert transition.state == 'odom_bootstrap'
    assert transition.consecutive_matches == 0
    assert transition.rejected_transitions == 1


def test_scan_ordering_is_preserved_across_pose_source_transition():
    queue = PendingScanQueue(4)
    for stamp in (10, 11, 12):
        queue.enqueue(stamped_scan(stamp), float(stamp))
    transition = PoseSourceTransition(1, 0.05, 0.15)
    transition.odom_available()
    published = []
    action, item, _ = queue.take(12.01, 5.0, True)
    published.append(item[0].header.stamp.sec)
    transition.compare_shared(Transform2D(0, 0, 0), Transform2D(0, 0, 0))
    while queue.items:
        action, item, _ = queue.take(12.02, 5.0, True)
        published.append(item[0].header.stamp.sec)
    assert published == [10, 11, 12]


def test_angular_window_optimization_matches_full_ray_selection():
    ranges = [0.4 + 0.001 * index for index in range(720)]
    message = scan(ranges, angle_min=-math.pi, increment=2 * math.pi / 720)
    directions = beam_directions(message)
    optimized, optimized_intervals = selected_indices_cached(
        message, -0.30, -0.02, 0.035, 0.012, directions)
    full = []
    full_intervals = {}
    for index, measured in enumerate(message.ranges):
        angle = message.angle_min + index * message.angle_increment
        interval = circle_ray_interval(-0.30, -0.02, 0.035, angle)
        if interval is None:
            continue
        full_intervals[index] = interval
        if interval[0] - 0.012 - 1e-7 <= measured <= interval[1] + 0.012 + 1e-7:
            full.append(index)
    assert optimized == full
    assert optimized_intervals.keys() == full_intervals.keys()
    assert selected_indices(message, -0.30, -0.02, 0.035, 0.012)[0] == full
