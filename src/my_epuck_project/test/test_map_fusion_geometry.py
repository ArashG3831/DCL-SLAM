"""Deterministic geometry checks for source-aware occupancy-grid fusion."""

import math
from types import SimpleNamespace

from builtin_interfaces.msg import Time as TimeMessage
from my_epuck_project.source_aware_map_fusion import SourceAwareMapFusion
from my_epuck_project.source_aware_map_fusion import (
    _changed_update_bounds,
    _snapshot_pose_age_s,
    _same_grid_content,
    _scalar_fused_data,
    _vectorized_fused_data,
)
from my_epuck_project.live_map_sanitizer import sanitize_shared_map
from nav_msgs.msg import OccupancyGrid
from rclpy.time import Time


def quaternion(yaw):
    return SimpleNamespace(
        x=0.0, y=0.0, z=math.sin(yaw / 2.0), w=math.cos(yaw / 2.0))


def grid(width, height, resolution, x, y, yaw):
    return SimpleNamespace(info=SimpleNamespace(
        width=width,
        height=height,
        resolution=resolution,
        origin=SimpleNamespace(
            position=SimpleNamespace(x=x, y=y, z=0.0),
            orientation=quaternion(yaw),
        ),
    ))


def test_rotated_negative_origin_and_dynamic_bounds_are_preserved():
    fusion = object.__new__(SourceAwareMapFusion)
    message = grid(4, 2, 0.03, -1.2, -0.7, math.pi / 2.0)
    transform = (0.4, -2.0, -math.pi / 2.0)
    corners = fusion.corners(message, transform)
    assert len(corners) == 4
    assert all(math.isfinite(value) for point in corners for value in point)
    expected = [
        fusion.point_in_output(message, transform, x, y)
        for x, y in ((0.0, 0.0), (0.12, 0.0), (0.0, 0.06), (0.12, 0.06))
    ]
    assert corners == expected
    assert min(point[0] for point in corners) < 0.0
    assert min(point[1] for point in corners) < 0.0


def test_profile_resolution_produces_metadata_sized_dynamic_grid():
    resolution = 0.03
    corners = [(-0.04, -0.07), (0.11, -0.07), (-0.04, 0.08), (0.11, 0.08)]
    minimum_x = math.floor(min(x for x, _ in corners) / resolution) * resolution
    minimum_y = math.floor(min(y for _, y in corners) / resolution) * resolution
    maximum_x = math.ceil(max(x for x, _ in corners) / resolution) * resolution
    maximum_y = math.ceil(max(y for _, y in corners) / resolution) * resolution
    width = int(round((maximum_x - minimum_x) / resolution))
    height = int(round((maximum_y - minimum_y) / resolution))
    assert (minimum_x, minimum_y) == (-0.06, -0.09)
    assert (width, height) == (6, 6)
    assert width * height == 36


def occupancy_grid(width, height, resolution, x, y, yaw, data):
    message = grid(width, height, resolution, x, y, yaw)
    message.header = SimpleNamespace(frame_id='robot/map')
    message.data = list(data)
    message.header.stamp = SimpleNamespace(sec=1, nanosec=0)
    return message


def test_vectorized_fusion_matches_scalar_reference():
    messages = [
        occupancy_grid(
            4, 3, 0.05, -0.2, 0.1, 0.1,
            [-1, 0, 100, -1, 50, 0, -1, 80, 0, -1, -1, 20]),
        occupancy_grid(
            3, 4, 0.05, 0.15, -0.1, -0.2,
            [0, 70, -1, 100, -1, 0, 30, -1, 90, 0, -1, -1]),
    ]
    transforms = [(0.2, -0.3, 0.4), (-0.1, 0.2, -0.5)]
    scalar = _scalar_fused_data(
        messages, transforms, -0.5, -0.5, 30, 30, 0.05)
    vector = _vectorized_fused_data(
        messages, transforms, -0.5, -0.5, 30, 30, 0.05, {})
    assert vector.tolist() == scalar.tolist()


def test_vectorized_fusion_matches_scalar_for_randomized_geometries():
    """Compare vectorized fusion with scalar fusion across varied transforms."""
    import random

    randomizer = random.Random(17)
    for _ in range(8):
        messages = []
        transforms = []
        for _ in range(2):
            width = randomizer.randint(2, 7)
            height = randomizer.randint(2, 7)
            data = [randomizer.choice((-1, 0, 35, 70, 100))
                    for _ in range(width * height)]
            messages.append(occupancy_grid(
                width, height, randomizer.choice((0.03, 0.05, 0.1)),
                randomizer.uniform(-0.4, 0.4),
                randomizer.uniform(-0.4, 0.4),
                randomizer.uniform(-math.pi, math.pi), data))
            transforms.append((
                randomizer.uniform(-0.4, 0.4),
                randomizer.uniform(-0.4, 0.4),
                randomizer.uniform(-math.pi, math.pi),
            ))
        scalar = _scalar_fused_data(
            messages, transforms, -1.0, -1.0, 40, 40, 0.05)
        vector = _vectorized_fused_data(
            messages, transforms, -1.0, -1.0, 40, 40, 0.05, {})
        assert vector.tolist() == scalar.tolist()


def test_vectorized_geometry_cache_is_bounded_across_map_revisions():
    """Map growth must not retain coordinate arrays for every old geometry."""
    cache = {}
    for revision in range(12):
        message = occupancy_grid(
            20, 20, 0.03, -float(revision), 0.0, 0.0,
            [0] * (20 * 20))
        _vectorized_fused_data(
            [message], [(0.0, 0.0, 0.0)], -2.0, -2.0, 200, 200,
            0.03, cache)
    assert len(cache) <= 4


def test_identical_map_content_ignores_timestamp_changes():
    first = occupancy_grid(2, 2, 0.1, 0.0, 0.0, 0.0, [0, 100, -1, 50])
    second = occupancy_grid(2, 2, 0.1, 0.0, 0.0, 0.0, [0, 100, -1, 50])
    second.header.stamp = SimpleNamespace(sec=99, nanosec=2)
    assert _same_grid_content(first, second)


def _fusion_clock(seconds):
    return SimpleNamespace(now=lambda: Time(nanoseconds=int(seconds * 1e9)))


def test_live_unchanged_map_uses_freshness_republish_without_revision_change():
    fusion = object.__new__(SourceAwareMapFusion)
    local = occupancy_grid(2, 2, 0.1, 0.0, 0.0, 0.0, [0, 100, -1, 50])
    remote = occupancy_grid(2, 2, 0.1, 0.0, 0.0, 0.0, [0, 100, -1, 50])
    local.header.stamp = TimeMessage(sec=10, nanosec=0)
    remote.header.stamp = TimeMessage(sec=9, nanosec=0)
    output = occupancy_grid(2, 2, 0.1, 0.0, 0.0, 0.0, [0, 100, -1, 50])
    output.header.frame_id = 'shared_map'
    output.header.stamp = TimeMessage(sec=8, nanosec=0)
    fusion.local_map = local
    fusion.remote_map = remote
    fusion.output_grid = output
    fusion.rebuild_period_s = 1.0
    fusion.source_freshness_max_age_s = 3.0
    fusion.last_output_publish_ros_s = 8.0
    fusion.map_revision = 4
    fusion.get_clock = lambda: _fusion_clock(10.0)
    published = []
    profiles = []
    fusion._publish_fused = lambda grid: published.append(grid)
    fusion._profile = lambda **kwargs: profiles.append(kwargs)

    assert fusion._maybe_republish_freshness(
        [local, remote], 0.0, 0.0)
    assert len(published) == 1
    assert output.header.stamp.sec == 10
    assert output.header.stamp.nanosec == 0
    assert fusion.map_revision == 4
    assert profiles[-1]['mode'] == 'FRESHNESS_REPUBLISH'


def test_stale_local_source_blocks_freshness_republish():
    fusion = object.__new__(SourceAwareMapFusion)
    local = occupancy_grid(2, 2, 0.1, 0.0, 0.0, 0.0, [0, 100, -1, 50])
    remote = occupancy_grid(2, 2, 0.1, 0.0, 0.0, 0.0, [0, 100, -1, 50])
    local.header.stamp = TimeMessage(sec=4, nanosec=0)
    remote.header.stamp = TimeMessage(sec=4, nanosec=0)
    fusion.local_map = local
    fusion.remote_map = remote
    fusion.output_grid = local
    fusion.rebuild_period_s = 1.0
    fusion.source_freshness_max_age_s = 3.0
    fusion.last_output_publish_ros_s = 4.0
    fusion.get_clock = lambda: _fusion_clock(10.0)
    published = []
    fusion._publish_fused = lambda grid: published.append(grid)
    fusion._profile = lambda **kwargs: None

    assert not fusion._maybe_republish_freshness(
        [local, remote], 0.0, 0.0)
    assert not published


def test_freshness_republish_respects_existing_cadence():
    fusion = object.__new__(SourceAwareMapFusion)
    local = occupancy_grid(2, 2, 0.1, 0.0, 0.0, 0.0, [0, 100, -1, 50])
    remote = occupancy_grid(2, 2, 0.1, 0.0, 0.0, 0.0, [0, 100, -1, 50])
    local.header.stamp = TimeMessage(sec=10, nanosec=0)
    remote.header.stamp = TimeMessage(sec=10, nanosec=0)
    fusion.local_map = local
    fusion.remote_map = remote
    fusion.output_grid = local
    fusion.rebuild_period_s = 1.0
    fusion.source_freshness_max_age_s = 3.0
    fusion.last_output_publish_ros_s = 9.5
    fusion.get_clock = lambda: _fusion_clock(10.0)
    published = []
    fusion._publish_fused = lambda grid: published.append(grid)
    fusion._profile = lambda **kwargs: None

    assert not fusion._maybe_republish_freshness(
        [local, remote], 0.0, 0.0)
    assert not published


def test_freshness_profile_diagnostic_is_serializable():
    fusion = object.__new__(SourceAwareMapFusion)
    fusion.profile_window = {
        'invocations': 0, 'full_rebuilds': 0, 'pose_updates': 0,
        'cells_inspected': 0, 'cells_copied': 0, 'cells_modified': 0,
        'publications': 0, 'visualization_bases': 0,
        'visualization_updates': 0, 'dirty_events': 0,
        'coalesced_events': 0, 'rebuild_skipped': 0,
        'wall_s': 0.0, 'cpu_s': 0.0, 'last_log_wall': 0.0,
    }
    fusion.last_freshness_source_age_s = 0.25
    fusion.last_freshness_previous_publication_age_s = 1.5
    fusion.last_freshness_output_stamp_s = 12.0
    messages = []
    fusion.get_logger = lambda: SimpleNamespace(
        info=lambda message: messages.append(message))

    fusion._profile(
        mode='FRESHNESS_REPUBLISH', map_changed=False,
        dimensions=(2, 2), cells_inspected=0, cells_copied=0,
        cells_modified=0, published=True, wall_s=0.01, cpu_s=0.01)

    assert messages
    assert 'mode=FRESHNESS_REPUBLISH' in messages[0]
    assert 'freshness_source_age_s=0.250' in messages[0]


def test_fusion_snapshot_time_is_the_newest_input_map_stamp():
    """Both peers must sanitize one map pair at the same TF snapshot time."""
    first = occupancy_grid(2, 2, 0.1, 0.0, 0.0, 0.0, [0, 100, -1, 50])
    second = occupancy_grid(2, 2, 0.1, 0.0, 0.0, 0.0, [0, 100, -1, 50])
    first.header.stamp = TimeMessage(sec=12, nanosec=100)
    second.header.stamp = TimeMessage(sec=11, nanosec=900)
    snapshot = SourceAwareMapFusion._common_snapshot_time([first, second])
    assert snapshot.nanoseconds == 12_000_000_100


def test_fusion_snapshot_time_is_independent_of_callback_order():
    first = occupancy_grid(2, 2, 0.1, 0.0, 0.0, 0.0, [0, 100, -1, 50])
    second = occupancy_grid(2, 2, 0.1, 0.0, 0.0, 0.0, [0, 100, -1, 50])
    first.header.stamp = TimeMessage(sec=4, nanosec=1)
    second.header.stamp = TimeMessage(sec=8, nanosec=2)
    left = SourceAwareMapFusion._common_snapshot_time([first, second])
    right = SourceAwareMapFusion._common_snapshot_time([second, first])
    assert left.nanoseconds == right.nanoseconds == 8_000_000_002


def _sanitizer_grid():
    result = OccupancyGrid()
    result.info.resolution = 0.01
    result.info.width = 30
    result.info.height = 30
    result.info.origin.position.x = -0.15
    result.info.origin.position.y = -0.15
    result.info.origin.orientation.w = 1.0
    result.data = [100] * (30 * 30)
    return result


def _snapshot_footprint_age(snapshot_ns, pose_ns):
    return _snapshot_pose_age_s(
        Time(nanoseconds=snapshot_ns),
        None if pose_ns is None else Time(nanoseconds=pose_ns),
    )


def test_delayed_snapshot_with_temporally_corresponding_pose_clears():
    """A four-second-old map is valid when TF is at that map timestamp."""
    age = _snapshot_footprint_age(4_000_000_000, 4_000_000_000)
    assert age == 0.0
    sanitized, details = sanitize_shared_map(
        _sanitizer_grid(),
        [{'x': 0.0, 'y': 0.0, 'radius_m': 0.037,
          'pose_age_s': age, 'role': 'peer'}],
        uncertainty_cells=0,
    )
    assert details['stale_pose_count'] == 0
    assert details['cleared_cell_count'] > 0
    assert sanitized.data[15 * 30 + 15] == 0


def test_pose_materially_mismatched_from_snapshot_is_rejected():
    age = _snapshot_footprint_age(4_000_000_000, 4_600_000_000)
    assert age == 0.6
    sanitized, details = sanitize_shared_map(
        _sanitizer_grid(),
        [{'x': 0.0, 'y': 0.0, 'radius_m': 0.037,
          'pose_age_s': age, 'role': 'peer'}],
        uncertainty_cells=0,
    )
    assert details['stale_pose_count'] == 1
    assert details['cleared_cell_count'] == 0
    assert list(sanitized.data) == [100] * (30 * 30)


def test_missing_tf_for_snapshot_is_rejected_safely():
    age = _snapshot_footprint_age(4_000_000_000, None)
    assert math.isinf(age)
    sanitized, details = sanitize_shared_map(
        _sanitizer_grid(),
        [{'x': 0.0, 'y': 0.0, 'radius_m': 0.037,
          'pose_age_s': age, 'role': 'peer'}],
        uncertainty_cells=0,
    )
    assert details['stale_pose_count'] == 1
    assert details['cleared_cell_count'] == 0
    assert list(sanitized.data) == [100] * (30 * 30)


def test_changed_map_content_is_detected_without_list_comparison():
    first = occupancy_grid(2, 2, 0.1, 0.0, 0.0, 0.0, [0, 100, -1, 50])
    second = occupancy_grid(2, 2, 0.1, 0.0, 0.0, 0.0, [0, 80, -1, 50])
    assert not _same_grid_content(first, second)


def test_visualization_update_bounds_are_minimal_and_empty_changes_are_skipped():
    previous = [[0, 0, 0, 0], [0, 0, 0, 0], [0, 0, 0, 0]]
    current = [[0, 0, 0, 0], [0, 0, 40, 0], [0, 0, 0, 80]]
    assert _changed_update_bounds(current, current) is None
    assert _changed_update_bounds(current, previous) == (2, 1, 2, 2)


def test_visualization_update_bounds_reject_geometry_shape_change():
    assert _changed_update_bounds([[0, 0]], [[0, 0, 0]]) is None
