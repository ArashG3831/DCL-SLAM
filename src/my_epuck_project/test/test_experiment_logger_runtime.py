import csv
import io
import json
import math
import signal
import subprocess
import threading
import time
from collections import deque
from bisect import bisect_right
from types import SimpleNamespace

import pytest
import rclpy
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from rcl_interfaces.msg import Log
from tf2_msgs.msg import TFMessage
from my_epuck_interfaces.msg import ExplorationEvent, ExplorationStatus
from tf2_ros import TransformException

from my_epuck_project.cooperative_experiment_logger import (
    CooperativeExperimentLogger, RollingTimestampIndex,
    RawTFSeriesIndex, SupervisorTimestampIndex, _compose_planar,
    _interpolate_planar, _invert_planar,
    create_logger_executor,
    is_shutdown_conversion_error, supervisor_robot_arguments)


@pytest.fixture
def observer(tmp_path):
    rclpy.init(
        args=[
            "--ros-args",
            "-p",
            f"output_root:={tmp_path}",
            "-p",
            "enable_console_status:=false",
            # These lifecycle/frontend tests do not provide local occupancy
            # snapshots.  Keep optional forensic capture off here so a bare
            # logger test exercises its own contract rather than failing on
            # deliberately absent map evidence.
            "-p",
            "enable_local_map_capture:=false",
        ]
    )
    node = CooperativeExperimentLogger()
    yield node
    if not node.finalized:
        node.finalize(False)
    node.destroy_node()
    rclpy.shutdown()


def read_json(path):
    return json.loads(path.read_text(encoding="utf-8"))


def read_events(observer):
    observer.events.flush()
    return [
        json.loads(line)
        for line in (observer.directory / "events.jsonl").read_text(
            encoding="utf-8"
        ).splitlines()
    ]


def test_opt_in_callback_timing_records_existing_logger_callbacks(observer):
    observer._callback_timing_enabled = True
    observer.safe_call('coverage', lambda: None)
    entry = observer._callback_timing['coverage']
    assert entry['calls'] == 1
    assert entry['total_wall_s'] >= 0.0
    assert entry['max_wall_s'] >= 0.0


def test_opt_in_odom_tf_profile_records_internal_stages(observer):
    observer._high_rate_profile_enabled = True

    odom = Odometry()
    odom.header.frame_id = 'robot1/odom'
    observer.odom('robot1', odom)

    transform = TransformStamped()
    transform.header.frame_id = 'robot1/map'
    transform.child_frame_id = 'robot1/odom'
    transform.transform.rotation.w = 1.0
    observer._direct_tf_message(TFMessage(transforms=[transform]))

    profile = observer._high_rate_profile
    expected = (
        'robot1.odom.live_state',
        'robot1.odom.index_maintenance',
        'robot1.odom.live_motion_state',
        'robot1.odom.direct_tf_lookup',
        'robot1.odom.tf2_lookup',
        'tf.tf2_buffer_update',
        'tf.direct_index',
    )
    for component in expected:
        assert profile[component]['calls'] == 1
        assert profile[component]['total_wall_s'] >= 0.0
        assert profile[component]['max_wall_s'] >= 0.0


def test_raw_evidence_odom_uses_existing_composed_tf_before_tf2(observer):
    """The raw-evidence path avoids tf2 for a complete chained TF sample."""
    observer.p['enable_scientific_raw_capture'] = True
    stamp_value = 1.0
    transforms = []
    for parent, child, x in (
            ('shared_map', 'robot1/local_world', 1.0),
            ('robot1/local_world', 'robot1/map', 2.0),
            ('robot1/map', 'robot1/odom', 3.0)):
        transform = TransformStamped()
        transform.header.frame_id = parent
        transform.child_frame_id = child
        transform.header.stamp.sec = int(stamp_value)
        transform.header.stamp.nanosec = 0
        transform.transform.translation.x = x
        transform.transform.rotation.w = 1.0
        transforms.append(transform)
    observer._direct_tf_message(TFMessage(transforms=transforms))

    class NoLookupBuffer:
        def lookup_transform(self, *args, **kwargs):
            raise AssertionError('tf2 lookup should not be needed')

    observer.tf_buffer = NoLookupBuffer()
    odom = Odometry()
    odom.header.frame_id = 'robot1/odom'
    odom.header.stamp.sec = int(stamp_value)
    odom.pose.pose.orientation.w = 1.0
    observer.odom('robot1', odom)

    assert observer.latest['robot1']['shared_pose_frame'] == 'shared_map'
    assert observer.latest['robot1']['shared_pose'][0] == pytest.approx(6.0)


def test_deferred_synchronized_frames_record_requests_then_reconstruct(
        tmp_path):
    """Derived sync rows are not built on the live timer path."""
    class FakeForensic:
        def __init__(self):
            self.rows = []

        def record_synchronized_map_frame(self, row):
            self.rows.append(row)

    node = CooperativeExperimentLogger.__new__(CooperativeExperimentLogger)
    node._defer_synchronized_map_frames = True
    node._deferred_sync_map_requests = []
    node.forensic = FakeForensic()
    node.forensic_sync_enabled = True
    node.robots = ['robot1']
    node.start = time.monotonic()
    node.latest_evidence = {'robot1': {'keyframe_id': 'k1'}}
    node._lookup_sync_tf_direct_only = lambda *args, **kwargs: None
    node._lookup_odom_pose = lambda *args, **kwargs: {
        'available': False, 'lookup_mode': 'exact_ros_time',
    }
    node.latest = {
        'robot1': {
            'map': type('Map', (), {
                'header': type('Header', (), {'frame_id': 'robot1/map'})(),
            })(),
            'odom': type('Odom', (), {
                'header': type('Header', (), {'frame_id': 'robot1/odom'})(),
                'child_frame_id': 'robot1/base_footprint',
            })(),
        },
    }
    node.ros_seconds = lambda: 12.5
    node._reconstruct_deferred_synchronized_map_frames = (
        node._reconstruct_deferred_synchronized_map_frames)

    node.forensic_synchronized_map_frame('periodic')

    assert len(node._deferred_sync_map_requests) == 1
    assert node.forensic.rows == []
    request = node._deferred_sync_map_requests[0]
    assert request['query_ros_time_s'] == pytest.approx(12.5)
    assert request['map_frame'] == 'robot1/map'
    assert request['base_frame'] == 'robot1/base_footprint'


def test_deferred_synchronized_reconstruction_preserves_request_schema(
        tmp_path):
    class FakeForensic:
        def __init__(self):
            self.rows = []

        def record_synchronized_map_frame(self, row):
            self.rows.append(row)

        def flush(self):
            pass

    node = CooperativeExperimentLogger.__new__(CooperativeExperimentLogger)
    node._defer_synchronized_map_frames = True
    node._deferred_sync_map_requests = [{
        'robot_id': 'robot1',
        'query_ros_time_s': 12.5,
        'query_wall_elapsed_s': 3.25,
        'sample_kind': 'periodic',
        'map_frame': 'robot1/map',
        'odom_frame': 'robot1/odom',
        'base_frame': 'robot1/base_footprint',
        'evidence': {'keyframe_id': 'k1'},
    }]
    node.forensic = FakeForensic()
    node.robots = ['robot1']
    node.directory = tmp_path
    (tmp_path / 'forensic').mkdir()
    (tmp_path / 'forensic' / 'raw_tf.csv').write_text(
        'topic,static,received_ros_time_s,received_wall_elapsed_s,'
        'transform_stamp,parent_frame,child_frame,translation_x,'
        'translation_y,translation_z,rotation_x,rotation_y,rotation_z,'
        'rotation_w\n', encoding='utf-8')
    node.flush = lambda: None
    node._sync_map_profile_enabled = False
    node._lookup_sync_tf = lambda *args, **kwargs: {
        'available': False,
        'synchronization_valid': False,
        'lookup_mode': 'raw_tf_unavailable',
        'path': ['robot1/base_footprint'],
        'error': '',
    }
    node._lookup_odom_pose = lambda *args, **kwargs: {
        'available': False,
        'lookup_mode': 'raw_tf_unavailable',
        'path': ['robot1/base_footprint'],
    }

    node._reconstruct_deferred_synchronized_map_frames()

    assert len(node.forensic.rows) == 1
    row = node.forensic.rows[0]
    assert row['schema_version'] == 'synchronized_map_frame_1.0'
    assert row['query_ros_time_s'] == pytest.approx(12.5)
    assert row['sample_kind'] == 'periodic'
    assert row['evidence'] == {'keyframe_id': 'k1'}


def legacy_nearest_row(rows, query_ros):
    values = sorted(rows, key=lambda item: item[0])
    return min(values, key=lambda item: abs(item[0] - float(query_ros)))


@pytest.mark.parametrize(
    "rows, queries",
    [
        ([(2.0, 'two'), (0.0, 'zero'), (1.0, 'one')],
         [0.0, 0.5, 1.0, 1.5, 2.0, -1.0, 3.0]),
        ([(2.0, 'first-two'), (0.0, 'zero'), (2.0, 'second-two'),
          (1.0, 'one')], [2.0, 1.5]),
    ],
)
def test_supervisor_timestamp_index_preserves_legacy_nearest_rows(rows, queries):
    index = SupervisorTimestampIndex(rows)
    for query in queries:
        assert index.nearest(query) == legacy_nearest_row(rows, query)


def test_supervisor_timestamp_index_matches_legacy_on_large_dataset():
    rows = [(float(index) * 0.02, index) for index in range(100_000)]
    rows = list(reversed(rows))
    index = SupervisorTimestampIndex(rows)
    for query in (-1.0, 0.0, 0.01, 999.99, 1000.0, 250.005, 1999.99):
        assert index.nearest(query) == legacy_nearest_row(rows, query)


def test_rolling_timestamp_index_preserves_sorted_interpolation_inputs():
    rows = [(2.0, 'two'), (0.0, 'zero'), (1.0, 'one'),
            (1.0, 'second-one')]
    index = RollingTimestampIndex(16)
    for timestamp, value in rows:
        index.append(timestamp, value)
    timestamps, values = index.snapshot()
    assert timestamps == (0.0, 1.0, 1.0, 2.0)
    assert values == ((0.0, 'zero'), (1.0, 'one'),
                      (1.0, 'second-one'), (2.0, 'two'))


def test_rolling_timestamp_index_matches_legacy_bounded_history():
    rows = [(float(index) * 0.02, index) for index in range(5000)]
    index = RollingTimestampIndex(4096)
    for timestamp, value in rows:
        index.append(timestamp, value)
    _, values = index.snapshot()
    expected = sorted(rows, key=lambda item: item[0])[-4096:]
    assert list(values) == expected
    for query in (18.08, 25.0, 99.98):
        right = bisect_right([item[0] for item in values], query)
        assert right == bisect_right([item[0] for item in expected], query)


def test_rolling_timestamp_index_lookup_bounds_matches_snapshot_brackets():
    rows = [(2.0, 'two'), (0.0, 'zero'), (1.0, 'one'),
            (1.0, 'second-one')]
    index = RollingTimestampIndex(16)
    for timestamp, value in rows:
        index.append(timestamp, value)
    timestamps, values = index.snapshot()
    for query in (-0.05, 0.0, 0.5, 1.0, 1.5, 2.0, 2.05):
        right = bisect_right(timestamps, query)
        if query < timestamps[0]:
            expected = ('before', values[0][0], values[0][1])
        elif right == len(values):
            expected = ('after', values[-1][0], values[-1][1])
        else:
            expected = ('between', values[right - 1][0], values[right - 1][1],
                        values[right][0], values[right][1])
        assert index.lookup_bounds(query) == expected


def _raw_tf_test_node(dynamic, static=(), maxlen=4096):
    node = CooperativeExperimentLogger.__new__(CooperativeExperimentLogger)
    node._direct_tf_samples = {}
    node._direct_tf_static = dict(static)
    node._direct_tf_sample_limit = maxlen
    node._direct_tf_graph_lock = threading.RLock()
    node._direct_tf_graph_keys = set()
    node._direct_tf_adjacency = {}
    node._direct_tf_adjacency_snapshot = None
    for key, values in dynamic.items():
        series = RawTFSeriesIndex(maxlen)
        for timestamp, transform in values:
            series.append(timestamp, transform)
        node._direct_tf_samples[key] = series
    for key in set(dynamic) | set(node._direct_tf_static):
        node._register_direct_tf_key(key)
    return node


def _legacy_tf_edge(dynamic, static, parent, child, query, allow_latest=False):
    key = (parent, child)
    if key in static:
        return static[key][1], {'lookup_mode': 'raw_tf_static'}
    values = list(dynamic.get(key, ()))
    if not values:
        return None
    values.sort(key=lambda item: item[0])
    query = float(query)
    if allow_latest:
        before = [item for item in values if item[0] <= query]
        if before:
            latest = before[-1]
            return latest[1], {'lookup_mode': 'raw_tf_latest_valid_before_query'}
    exact = min(values, key=lambda item: abs(item[0] - query))
    if abs(exact[0] - query) <= 1.0e-9:
        return exact[1], {'lookup_mode': 'raw_tf_exact'}
    if query < values[0][0] or query > values[-1][0]:
        if abs(exact[0] - query) > 0.10:
            return None
        return exact[1], {'lookup_mode': 'raw_tf_bounded_nearest'}
    right = next(index for index, item in enumerate(values) if item[0] > query)
    left_item, right_item = values[right - 1], values[right]
    span = right_item[0] - left_item[0]
    if span <= 0.0 or span > 0.10:
        return None
    fraction = (query - left_item[0]) / span
    return _interpolate_planar(
        left_item[1], right_item[1], fraction), {
            'lookup_mode': 'raw_tf_tightly_interpolated'}


def _legacy_tf_sync(dynamic, static, target, source, query):
    if target == source:
        return (0.0, 0.0, 0.0), {'lookup_mode': 'raw_tf_identity'}
    adjacency = {}
    for parent, child in set(dynamic) | set(static):
        adjacency.setdefault(child, []).append((parent, False))
        adjacency.setdefault(parent, []).append((child, True))
    queue = deque([(source, (0.0, 0.0, 0.0), [source])])
    visited = {source}
    while queue:
        current, accumulated, path = queue.popleft()
        for neighbour, inverse in sorted(adjacency.get(current, ())):
            if neighbour in visited:
                continue
            parent, child = ((neighbour, current) if not inverse else
                             (current, neighbour))
            edge = _legacy_tf_edge(dynamic, static, parent, child, query)
            if edge is None:
                continue
            transform, metadata = edge
            if inverse:
                transform = _invert_planar(transform)
            composed = _compose_planar(transform, accumulated)
            next_path = path + [neighbour]
            if neighbour == target:
                return composed, {
                    'lookup_mode': 'raw_tf_bounded_composed',
                    'path': next_path,
                }
            visited.add(neighbour)
            queue.append((neighbour, composed, next_path))
    return None, {'lookup_mode': 'raw_tf_unavailable', 'path': [source]}


def test_raw_tf_index_preserves_direct_and_chained_fallback_semantics():
    dynamic = {
        ('map', 'odom'): [
            (1.0, (1.0, 0.0, 0.0)),
            (0.0, (0.0, 0.0, 0.0)),
            (2.0, (2.0, 0.0, 0.0)),
        ],
        ('odom', 'base'): [
            (2.0, (0.0, 2.0, 0.0)),
            (1.0, (0.0, 1.0, 0.0)),
            (0.0, (0.0, 0.0, 0.0)),
            (1.0, (0.0, 1.5, 0.0)),
        ],
    }
    static = {('base', 'sensor'): (0.0, (0.2, 0.0, 0.0))}
    node = _raw_tf_test_node(dynamic, static)
    for target, source, query in (
            [('map', 'odom', 1.0), ('map', 'base', 1.05),
             ('base', 'map', 1.05), ('sensor', 'map', 1.05),
             ('missing', 'map', 1.05)]):
        expected = _legacy_tf_sync(dynamic, static, target, source, query)
        actual = node._direct_sync_tf(target, source, query)
        if expected[0] is None:
            assert actual[0] is None
        else:
            assert actual[0] == pytest.approx(expected[0])
        assert actual[1]['lookup_mode'] == expected[1]['lookup_mode']


def test_raw_tf_index_preserves_duplicate_ties_and_bounded_expiry():
    series = RawTFSeriesIndex(3)
    for item in ((2.0, 'late'), (1.0, 'first'), (1.0, 'second'),
                 (3.0, 'new'), (4.0, 'newest')):
        series.append(*item)
    timestamps, values = series.sorted_snapshot()
    assert timestamps == (1.0, 3.0, 4.0)
    assert values == ((1.0, 'second'), (3.0, 'new'), (4.0, 'newest'))
    node = _raw_tf_test_node({('map', 'odom'): [
        (2.0, (2.0, 0.0, 0.0)), (1.0, (1.0, 0.0, 0.0)),
        (1.0, (1.1, 0.0, 0.0)), (3.0, (3.0, 0.0, 0.0)),
        (4.0, (4.0, 0.0, 0.0))]}, maxlen=3)
    assert node._direct_tf_edge('map', 'odom', 3.0)[0] == pytest.approx(
        (3.0, 0.0, 0.0))
    # After arrival-bounded expiry, the remaining 3-to-4 gap is too large for
    # interpolation, so the legacy query is correctly unavailable.
    assert node._direct_tf_edge('map', 'odom', 3.5) is None


def test_raw_tf_monotonic_lookup_bounds_avoids_snapshot_copy_and_matches_legacy():
    series = RawTFSeriesIndex(32)
    rows = [(index * 0.02, (float(index), 0.0, 0.0))
            for index in range(100)]
    for row in rows:
        series.append(*row)
    for query in (-1.0, 0.0, 0.01, 1.0, 1.01, 1.98, 2.5):
        bounds = series.lookup_bounds(query)
        timestamps, values = series.sorted_snapshot()
        right = bisect_right(timestamps, query)
        if query < timestamps[0]:
            expected = ('before', values[0][0], values[0][1])
        elif right == len(values):
            expected = ('after', values[-1][0], values[-1][1])
        else:
            expected = ('between', values[right - 1][0], values[right - 1][1],
                        values[right][0], values[right][1])
        assert bounds == expected


def test_raw_tf_index_handles_large_out_of_order_trace():
    rows = [(float(index) * 0.02, (float(index), 0.0, 0.0))
            for index in range(2000)]
    dynamic = {('map', 'odom'): list(reversed(rows))}
    node = _raw_tf_test_node(dynamic)
    for query in (-1.0, 0.0, 0.01, 12.345, 39.98, 50.0):
        actual = node._direct_tf_edge('map', 'odom', query)
        expected = _legacy_tf_edge(dynamic, {}, 'map', 'odom', query)
        if expected is None:
            assert actual is None
        else:
            assert actual[0] == pytest.approx(expected[0])


def test_combined_tf_callback_feeds_buffer_and_retains_raw_evidence_path():
    class FakeBuffer:
        def __init__(self):
            self.dynamic = []
            self.static = []

        def set_transform(self, transform, authority):
            self.dynamic.append((transform, authority))

        def set_transform_static(self, transform, authority):
            self.static.append((transform, authority))

    node = CooperativeExperimentLogger.__new__(CooperativeExperimentLogger)
    node.tf_buffer = FakeBuffer()
    captured = []
    node._record_direct_tf_message = (
        lambda message, static=False, **kwargs:
        captured.append((message, static)))
    node.forensic = None
    transform = TransformStamped()
    transform.header.frame_id = 'map'
    transform.child_frame_id = 'odom'
    message = TFMessage(transforms=[transform])

    node._direct_tf_message(message)
    node._direct_tf_static_message(message)

    assert node.tf_buffer.dynamic == [(transform, 'default_authority')]
    assert node.tf_buffer.static == [(transform, 'default_authority')]
    assert captured == [(message, False), (message, True)]


def test_combined_tf_callback_skips_only_unqueried_dynamic_edges():
    class FakeBuffer:
        def __init__(self):
            self.dynamic = []

        def set_transform(self, transform, authority):
            self.dynamic.append((transform, authority))

    node = CooperativeExperimentLogger.__new__(CooperativeExperimentLogger)
    node.tf_buffer = FakeBuffer()
    node._tf_buffer_live_dynamic_edges = {
        ('robot1', 'robot1/odom'),
    }
    captured = []
    node._record_direct_tf_message = (
        lambda message, static=False, **kwargs:
        captured.append((message, static)))
    node.forensic = None

    relevant = TransformStamped()
    relevant.header.frame_id = 'robot1'
    relevant.child_frame_id = 'robot1/odom'
    irrelevant = TransformStamped()
    irrelevant.header.frame_id = 'robot1/base_link'
    irrelevant.child_frame_id = 'robot1/left wheel'
    message = TFMessage(transforms=[relevant, irrelevant])

    node._direct_tf_message(message)

    assert node.tf_buffer.dynamic == [(relevant, 'default_authority')]
    # The complete raw message still reaches the direct evidence path.
    assert captured == [(message, False)]


def test_repeated_direct_tf_edge_reuses_graph_registration():
    node = _raw_tf_test_node({})
    registrations = []
    original_register = node._register_direct_tf_key

    def record_registration(key):
        registrations.append(key)
        original_register(key)

    node._register_direct_tf_key = record_registration

    def message(stamp, x):
        return SimpleNamespace(transforms=[SimpleNamespace(
            header=SimpleNamespace(
                frame_id='map',
                stamp=SimpleNamespace(sec=int(stamp), nanosec=0),
            ),
            child_frame_id='odom',
            transform=SimpleNamespace(
                translation=SimpleNamespace(x=x, y=0.0, z=0.0),
                rotation=SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0),
            ),
        )])

    node._record_direct_tf_message(message(1.0, 1.0))
    node._record_direct_tf_message(message(2.0, 2.0))

    assert registrations == [('map', 'odom')]
    assert len(node._direct_tf_samples[('map', 'odom')].sorted_snapshot()[0]) == 2
    assert node._direct_tf_graph()['odom'] == (('map', False),)
    assert node._direct_tf_graph()['map'] == (('odom', True),)


def test_odom_uses_exact_indexed_raw_tf_before_tf2_lookup():
    node = _raw_tf_test_node({
        ('shared_map', 'robot1/odom'): [(1.0, (2.0, 3.0, 0.0))],
    })
    node.p = {
        'global_frame': 'shared_map',
        'enable_trajectory_overlap': True,
    }
    node.latest = {'robot1': {}}
    node._odom_samples = {'robot1': RollingTimestampIndex(32)}
    node.local_trajectory = type('Trajectory', (), {
        'add': lambda self, robot, x, y: None,
    })()
    node.trajectory = type('Trajectory', (), {
        'add': lambda self, robot, x, y: None,
    })()
    node.forensic = None
    node.mark = lambda *args, **kwargs: None

    class ForbiddenTF2:
        def lookup_transform(self, *args, **kwargs):
            raise AssertionError('exact raw TF fast path should avoid tf2')

    node.tf_buffer = ForbiddenTF2()
    message = Odometry()
    message.header.frame_id = 'robot1/odom'
    message.header.stamp.sec = 1
    message.pose.pose.position.x = 0.5
    message.pose.pose.position.y = 0.25
    node.odom('robot1', message)

    assert node.latest['robot1']['shared_pose'] == pytest.approx(
        (2.5, 3.25, 0.0))
    assert node.latest['robot1']['shared_pose_frame'] == 'shared_map'


def test_direct_tf_lookup_remains_authoritative_when_available():
    class FakeBuffer:
        def lookup_transform(self, *args, **kwargs):
            return object()

    node = CooperativeExperimentLogger.__new__(CooperativeExperimentLogger)
    node.tf_buffer = FakeBuffer()
    node._tf_observation = lambda transform, query: {
        'available': True,
        'returned_stamp_delta_s': 0.0,
    }
    result = node._lookup_sync_tf('map', 'base', 4.0)
    assert result['available'] is True
    assert result['synchronization_valid'] is True


def test_goal_decision_ledger_mirrors_navigation_decision_events(observer):
    observer.event(
        'NAV_GOAL_SENT', 'local goal dispatched', 'robot1',
        canonical_task_id='task-1', physical_task_signature='physical-1',
        path_length_m=2.5, map_revision=7,
    )
    observer.goal_decisions.flush()
    rows = [
        json.loads(line)
        for line in (observer.directory / 'goal_decision_ledger.jsonl').read_text(
            encoding='utf-8').splitlines()
    ]
    assert len(rows) == 1
    assert rows[0]['decision_stage'] == 'NAV_GOAL_SENT'
    assert rows[0]['robot_id'] == 'robot1'
    assert rows[0]['canonical_task_id'] == 'task-1'
    assert rows[0]['physical_task_signature'] == 'physical-1'
    assert rows[0]['path_length_m'] == 2.5


def test_goal_accounting_exposes_active_and_terminal_goals_per_robot(observer):
    fields = dict(round_id='round-1', canonical_task_id='task-1',
                  physical_task_signature='physical-1')
    observer.event('NAV_GOAL_SENT', 'sent', 'robot1', **fields)
    observer.event('NAV_GOAL_ACCEPTED', 'accepted', 'robot1', **fields)
    observer.event('NAVIGATION_SUCCEEDED', 'done', 'robot1', **fields)
    observer.event('NAV_GOAL_SENT', 'sent', 'robot1', round_id='round-2',
                   canonical_task_id='task-2', physical_task_signature='physical-2')
    observer.latest['robot1']['navigation_active'] = True
    accounting = observer.goal_accounting_summary()
    assert accounting['by_robot']['robot1']['dispatched'] == 2
    assert accounting['by_robot']['robot1']['categories'] == {
        'SUCCEEDED': 1, 'STILL_ACTIVE_AT_MISSION_END': 1}
    assert accounting['by_robot']['robot2']['dispatched'] == 0


def test_goal_accounting_handles_acceptance_before_send(observer):
    fields = dict(round_id='round-before-send', canonical_task_id='task-before-send',
                  physical_task_signature='physical-before-send')
    observer.event('NAV_GOAL_ACCEPTED', 'accepted before dispatch record',
                   'robot2', **fields)
    observer.event('NAV_GOAL_SENT', 'sent after acceptance record',
                   'robot2', **fields)
    record = observer.goal_accounting_summary()['records'][0]
    assert record['accepted'] is True
    assert record['terminal_category'] == 'UNKNOWN_OR_UNACCOUNTED'


def test_odom_motion_accounting_uses_local_odom_before_shared_tf(observer,
                                                                  monkeypatch):
    def missing_transform(*args, **kwargs):
        raise TransformException('shared frame unavailable before handoff')

    monkeypatch.setattr(observer.tf_buffer, 'lookup_transform',
                        missing_transform)
    first = Odometry()
    first.header.frame_id = 'robot1/odom'
    first.pose.pose.position.x = 0.0
    first.pose.pose.position.y = 0.0
    first.twist.twist.linear.x = 0.12
    first.twist.twist.angular.z = 0.20
    second = Odometry()
    second.header.frame_id = 'robot1/odom'
    second.pose.pose.position.x = 0.3
    second.pose.pose.position.y = 0.4
    second.twist.twist.linear.x = 0.08
    second.twist.twist.angular.z = -0.10
    observer.odom('robot1', first)
    observer.odom('robot1', second)
    assert observer.latest['robot1']['pose'][:2] == (0.3, 0.4)
    assert observer.latest['robot1']['speed'] == (0.08, -0.10)
    assert observer.latest['robot1']['pose_frame'] == 'robot1/odom'
    assert observer.local_trajectory.total_distance['robot1'] == pytest.approx(0.5)


def test_logger_uses_stable_executor_for_shutdown_pybind_regression():
    """The experimental EventsExecutor conversion crash is not used."""
    from rclpy.executors import SingleThreadedExecutor
    rclpy.init()
    try:
        executor = create_logger_executor()
        assert isinstance(executor, SingleThreadedExecutor)
        executor.shutdown()
    finally:
        if rclpy.ok():
            rclpy.shutdown()


def test_logger_executor_is_bound_to_explicit_context():
    from rclpy.context import Context
    context = Context()
    rclpy.init(context=context)
    try:
        executor = create_logger_executor(context)
        assert executor.context is context
        executor.shutdown()
    finally:
        if context.ok():
            context.shutdown()


def test_logger_conversion_signature_is_only_accepted_after_context_loss():
    error = RuntimeError('Unable to convert call argument')
    assert is_shutdown_conversion_error(error, True, False)
    assert not is_shutdown_conversion_error(error, False, False)
    assert not is_shutdown_conversion_error(error, True, True)
    assert not is_shutdown_conversion_error(
        RuntimeError('application callback failure'), True, False)


def test_forensic_child_keyboard_interrupt_does_not_abort_finalization(observer, monkeypatch):
    class Child:
        returncode = None

        def poll(self):
            return self.returncode

        def send_signal(self, value):
            assert value == signal.SIGINT
            self.returncode = 0

        def terminate(self):
            pass

        def kill(self):
            self.returncode = -9

        def wait(self, timeout=None):
            if self.returncode is None:
                raise subprocess.TimeoutExpired('observer', timeout)
            return self.returncode

    child = Child()
    observer.ground_truth_process = child
    observer.stop_forensic_ground_truth()
    assert child.returncode == 0
    assert (observer.directory / 'forensic' / 'runtime' /
            'shutdown.requested').is_file()


def test_forensic_child_naturally_exits_before_observer_signal(observer):
    """Webots shutdown releases Supervisor.step before observer signalling."""
    class Child:
        returncode = None
        signals = []

        def poll(self):
            return self.returncode

        def send_signal(self, value):
            self.signals.append(value)
            raise AssertionError('natural Webots shutdown should finish first')

        def wait(self, timeout=None):
            del timeout
            self.returncode = 0
            return 0

        def kill(self):
            self.returncode = -9

    child = Child()
    observer.ground_truth_process = child
    observer.stop_forensic_ground_truth()
    assert child.returncode == 0
    assert child.signals == []


def controller_error():
    message = Log()
    message.level = Log.ERROR
    message.name = "robot2.controller_server"
    message.msg = "Failed to make progress"
    return message


def test_controller_error_after_info_does_not_change_call_site_severity(observer):
    observer.event("STACK_READY", "prior info event", console=True)
    observer.rosout(controller_error())
    events = read_events(observer)
    assert events[-1]["event_type"] == "CONTROLLER_WARNING"
    assert events[-1]["message"] == "Failed to make progress"
    observer.nav2_diagnostics.flush()
    diagnostics = [
        json.loads(line)
        for line in (observer.directory / "nav2_diagnostics.jsonl").read_text(
            encoding="utf-8").splitlines()
    ]
    assert diagnostics[-1]["category"] == "CONTROLLER_OR_PLANNER_ERROR"
    assert diagnostics[-1]["message"] == "Failed to make progress"


def test_raw_mode_defers_nav2_diagnostic_serialization(observer):
    """Raw receipts, rather than a duplicate live file, own raw-mode replay."""
    observer.passive_bag_enabled = True
    observer.p['enable_scientific_raw_capture'] = True
    observer.rosout(controller_error())
    observer.nav2_diagnostics.flush()
    assert (observer.directory / 'nav2_diagnostics.jsonl').read_text(
        encoding='utf-8') == ''


def test_csv_timing_schema_accepts_wall_and_sim_elapsed_fields(observer):
    """Every CSV writer accepts the dual-clock row emitted by the logger."""
    observer.sample_telemetry()
    observer.sample_coverage()
    observer.sample_health()
    assert observer.internal_errors == {}
    observer.flush()
    for name in ("robot1_timeseries.csv", "robot2_timeseries.csv",
                 "coverage.csv", "topic_health.csv"):
        header = (observer.directory / name).read_text(
            encoding="utf-8").splitlines()[0].split(",")
        assert "elapsed_s" in header
        assert "wall_elapsed_s" in header


def test_scan_pipeline_records_nav_source_age_and_gap(observer):
    observer.p['initial_configuration_json'] = json.dumps({
        'slam_runtime_parameters': {'throttle_scans': 1},
    })
    values = observer.scan_pipeline['robot1']
    values['scan_d500_fixed_stamps'].extend([1.0, 2.0, 3.0])
    values['scan_d500_nav_stamps'].extend([1.0, 2.0, 4.0])
    values['scan_d500_nav_ages_s'].extend([0.2, 0.4, 2.1])
    observer.write_scan_pipeline_diagnostic()
    diagnostic = read_json(observer.directory / 'scan_pipeline_diagnostic.json')
    nav = diagnostic['robots']['robot1']
    assert nav['nav_scan_messages'] == 3
    assert nav['nav_scan_max_source_gap_s'] == 2.0
    assert nav['nav_scan_source_age_s']['p95'] == 2.1


def test_nonfatal_subsystem_error_is_counted_and_logging_continues(observer):
    def malformed():
        raise ValueError("malformed diagnostic")

    assert observer.safe_call("warning_normalization", malformed) is None
    observer.event("STACK_READY", "continued after malformed input")
    assert sum(observer.internal_errors.values()) == 1
    assert [event["event_type"] for event in read_events(observer)][-2:] == [
        "LOGGER_INTERNAL_ERROR",
        "STACK_READY",
    ]


def test_double_finalization_and_late_callback_are_safe(observer):
    assert observer.finalize(False)
    before = (observer.directory / "events.jsonl").stat().st_size
    called = []
    observer.safe_call("late_callback", lambda: called.append(True))
    assert observer.finalize(False) is False
    assert called == []
    assert (observer.directory / "events.jsonl").stat().st_size == before
    assert read_json(observer.directory / "summary.json")["run"]["clean_shutdown"] is False


def test_partial_and_terminal_robot_states_are_retained(observer):
    observer.latest["robot1"].update(
        claim_state="SUCCEEDED", claim_id=1, navigation_active=False
    )
    observer.latest["robot2"].update(
        claim_state="NAVIGATING", claim_id=2, navigation_active=True
    )
    summary = observer.summary(False)
    assert summary["robot_terminal_state"]["robot1"]["claim_state"] == "SUCCEEDED"
    assert summary["robot_terminal_state"]["robot2"]["navigation_active"] is True
    observer.latest["robot2"].update(claim_state="SUCCEEDED", navigation_active=False)
    summary = observer.summary(True)
    assert all(
        not state["navigation_active"]
        for state in summary["robot_terminal_state"].values()
    )


def test_missing_shared_map_is_explicitly_unavailable_not_zero(observer):
    """A no-handoff observer run must not encode missing coverage as zero."""
    summary = observer.summary(False)
    mapping = summary["mapping"]
    assert mapping["available"] is False
    assert mapping["initial_known_cells"] is None
    assert mapping["final_known_cells"] is None
    assert mapping["coverage_gain_cells"] is None
    assert mapping["coverage_gain_per_metre_travelled"] is None
    assert "unavailable" in mapping["reason"]


def test_single_robot_coverage_writes_samples_from_local_map(observer):
    """Condition A uses robot1/map without requiring a shared-map topic."""
    observer.robots = ['robot1']
    observer.latest = {'robot1': {}}
    observer.coverage_source = 'local_map'

    first = OccupancyGrid()
    first.header.frame_id = 'robot1/map'
    first.info.width = 4
    first.info.height = 2
    first.info.resolution = 0.1
    first.info.origin.orientation.w = 1.0
    first.data = [-1, 0, 0, 100, -1, -1, 0, 0]
    observer.mark('robot1', 'map', first)
    observer.sample_coverage()

    second = OccupancyGrid()
    second.header.frame_id = 'robot1/map'
    second.info.width = 4
    second.info.height = 2
    second.info.resolution = 0.1
    second.info.origin.orientation.w = 1.0
    second.data = [0, 0, 0, 100, 0, 0, 0, 0]
    observer.mark('robot1', 'map', second)
    observer.sample_coverage()
    observer.flush()

    with (observer.directory / 'coverage.csv').open(
            newline='', encoding='utf-8') as stream:
        rows = list(csv.DictReader(stream))
    assert len(rows) == 2
    assert int(rows[0]['robot1_shared_known']) == 5
    assert int(rows[1]['robot1_shared_known']) == 8
    assert int(rows[0]['total_known_union_cells']) == 5
    assert int(rows[1]['total_known_union_cells']) == 8
    assert float(rows[1]['known_area_m2']) == pytest.approx(0.08)
    assert observer.summary(False)['mapping']['source'] == 'local_map'


def test_two_robot_coverage_keeps_shared_map_source(observer):
    """C/D retain the established two-robot shared-map contract."""
    for robot in ('robot1', 'robot2'):
        message = OccupancyGrid()
        message.header.frame_id = 'shared_map'
        message.info.width = 2
        message.info.height = 2
        message.info.resolution = 0.1
        message.info.origin.orientation.w = 1.0
        message.data = [0, 0, -1, 100]
        observer.mark(robot, 'shared_map', message)
    observer.sample_coverage()
    observer.flush()
    with (observer.directory / 'coverage.csv').open(
            newline='', encoding='utf-8') as stream:
        rows = list(csv.DictReader(stream))
    assert len(rows) == 1
    assert rows[0]['shared_maps_equivalent'] == 'True'
    assert observer.summary(False)['mapping']['source'] == 'shared_map'


def test_raw_coverage_requests_preserve_legacy_map_readiness_boundary(observer):
    """Raw replay must not record timer ticks before selected maps exist."""
    observer.coverage_request_file = io.StringIO()
    observer.coverage_source = 'shared_map'
    observer.sample_coverage()
    assert observer.coverage_request_file.getvalue() == ''

    observer.latest['robot1']['shared_map'] = object()
    observer.latest['robot2']['shared_map'] = object()
    observer.sample_coverage()
    rows = [json.loads(line) for line in
            observer.coverage_request_file.getvalue().splitlines()]
    assert len(rows) == 1
    assert rows[0]['event_sequence'] >= 1


def test_two_robot_independent_coverage_unions_transformed_local_maps(observer):
    """B uses passive local-map union coverage, not a runtime shared map."""
    observer.coverage_source = 'local_map_union'
    observer.p['coverage_attribution_resolution'] = 0.1
    observer.p['known_relative_transform'] = [0.1, 0.0, 0.0]
    for robot in ('robot1', 'robot2'):
        message = OccupancyGrid()
        message.header.frame_id = f'{robot}/map'
        message.info.width = 2
        message.info.height = 1
        message.info.resolution = 0.1
        message.info.origin.orientation.w = 1.0
        message.data = [0, -1]
        observer.mark(robot, 'map', message)

    observer.sample_coverage()
    observer.flush()
    with (observer.directory / 'coverage.csv').open(
            newline='', encoding='utf-8') as stream:
        rows = list(csv.DictReader(stream))
    assert len(rows) == 1
    assert int(rows[0]['total_known_union_cells']) == 2
    assert float(rows[0]['known_area_m2']) == pytest.approx(0.02)
    assert observer.summary(False)['mapping']['source'] == 'local_map_union'


def test_clean_finalization_closes_files_after_internal_error(observer):
    observer.safe_call("optional_metric", lambda: 1 / 0)
    assert observer.finalize(True)
    assert observer.events.closed
    assert all(stream.closed for stream in observer.files)
    assert read_json(observer.directory / "run_manifest.json")["clean_shutdown"] is True
    summary = read_json(observer.directory / "summary.json")
    assert summary["system"]["internal_logger_error_count"] == 1


def test_finalization_writes_explicit_complete_artifact_contract(observer):
    assert observer.finalize(True)
    status = read_json(observer.directory / "artifact_finalization.json")
    assert status["complete"] is True
    assert status["missing"] == []
    assert read_json(observer.directory / "summary.json")[
        "artifact_finalization"]["complete"] is True


def test_summary_accepts_deferred_warning_mapping_records(observer):
    observer._deferred_warning_records = [{
        "node_name": "/planner",
        "severity": "WARN",
        "normalized_message": "planner warning",
        "occurrence_count": 3,
    }]

    summary = observer.summary(False)

    assert summary["anomalies"]["warning_occurrences"] == 3


def test_ready_deferred_nav2_replay_satisfies_finalization_gate(observer):
    observer._replay_navigation_actions = (
        lambda: {"status": "NO_SCIENTIFIC_RAW_BAG"})
    observer._replay_shared_trajectory_for_parity = (
        lambda: {"status": "NO_SCIENTIFIC_RAW_BAG"})
    observer._replay_coverage_for_parity = (
        lambda: {"status": "NO_SCIENTIFIC_RAW_BAG"})
    observer._replay_pair_decisions_for_parity = (
        lambda: {"status": "NO_SCIENTIFIC_RAW_BAG"})
    observer._replay_agreement_counters_for_parity = (
        lambda: {"status": "NO_SCIENTIFIC_RAW_BAG"})
    observer._replay_warnings_for_parity = (
        lambda: {"status": "NO_SCIENTIFIC_RAW_BAG"})
    observer._replay_nav2_diagnostics_for_parity = lambda: {
        "status": "DEFERRED_AUTHORITATIVE",
        "authority_ready": True,
        "deferred_records": [{"category": "PLANNER", "message": "warning"}],
    }
    observer._replay_local_trajectory_for_parity = (
        lambda: {"status": "NO_SCIENTIFIC_RAW_BAG"})
    observer.required_artifact_status = lambda include_campaign_files=True: {
        "complete": True, "required": [], "missing": [],
    }

    assert observer.finalize(True)
    assert getattr(observer, "_nav2_diagnostic_replay_failed", False) is False
    assert read_json(observer.directory / "artifact_finalization.json")[
        "complete"] is True


def test_missing_forensic_artifact_fails_closed(observer):
    class FakeForensic:
        def flush(self):
            pass

        def close(self):
            pass

        def manifest(self):
            return {"files": []}

    observer.forensic = FakeForensic()
    observer.forensic_snapshot = lambda force=False: None
    assert observer.finalize(True) is False
    status = read_json(observer.directory / "artifact_finalization.json")
    assert status["complete"] is False
    assert sorted(status["missing"]) == sorted([
        "forensic/transforms.csv",
        "forensic/maps/robot1_map_final.npz",
        "forensic/maps/robot2_map_final.npz",
    ])
    assert read_json(observer.directory / "mission_result.json")[
        "artifact_finalization"]["complete"] is False


def test_supervisor_robot_arguments_follow_active_robot_ids():
    assert supervisor_robot_arguments(['robot1']) == [
        '--robot-def', 'robot1']
    assert supervisor_robot_arguments(['robot1', 'robot2']) == [
        '--robot-def', 'robot1', '--robot-def', 'robot2']


def test_single_robot_finalization_requires_only_robot1_artifacts(observer):
    class FakeForensic:
        def flush(self):
            pass

        def close(self):
            pass

        def manifest(self):
            return {"files": []}

    observer.robots = ['robot1']
    observer.forensic = FakeForensic()
    observer.forensic_supervisor_enabled = True
    observer.forensic_sync_enabled = True
    observer.forensic_snapshot = lambda force=False: None
    root = observer.directory / 'forensic'
    for path in (
            root / 'transforms.csv',
            root / 'maps' / 'robot1_map_final.npz',
            root / 'supervisor_ground_truth.csv',
            root / 'synchronized_map_frame.jsonl'):
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text('', encoding='utf-8')
    assert observer.finalize(True)
    status = read_json(observer.directory / 'artifact_finalization.json')
    assert status['complete'] is True
    assert 'forensic/maps/robot2_map_final.npz' not in status['required']
    assert 'forensic/maps/robot2_map_final.npz' not in status['missing']
    result = read_json(observer.directory / 'mission_result.json')
    assert 'robot1_final_state' in result
    assert 'robot2_final_state' not in result


def test_requested_contact_capture_requires_finalized_contact_artifact(observer):
    class FakeForensic:
        def save_final_maps(self, *args, **kwargs):
            pass

        def record_transform(self, *args, **kwargs):
            pass

        def manifest(self):
            return {'files': []}

        def flush(self):
            pass

        def close(self):
            pass

    observer.robots = ['robot1', 'robot2']
    observer.forensic = FakeForensic()
    observer.forensic_supervisor_enabled = True
    observer.forensic_sync_enabled = False
    observer.contact_capture = True
    root = observer.directory / 'forensic'
    for path in (
            root / 'transforms.csv',
            root / 'maps' / 'robot1_map_final.npz',
            root / 'maps' / 'robot2_map_final.npz',
            root / 'supervisor_ground_truth.csv'):
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text('', encoding='utf-8')

    missing = observer.required_artifact_status(False)
    assert missing['complete'] is False
    assert 'forensic/contact_points.csv' in missing['missing']

    (root / 'contact_points.csv').write_text(
        'sim_time_s,robot_id,contact_count,point_x_m,point_y_m,point_z_m,\n',
        encoding='utf-8')
    complete = observer.required_artifact_status(False)
    assert complete['complete'] is True


def test_missing_required_single_robot_artifact_fails_closed(observer):
    class FakeForensic:
        def save_final_maps(self, *args, **kwargs):
            pass

        def flush(self):
            pass

        def close(self):
            pass

        def manifest(self):
            return {"files": []}

    observer.robots = ['robot1']
    observer.forensic = FakeForensic()
    observer.forensic_supervisor_enabled = True
    observer.forensic_sync_enabled = True
    observer.forensic_snapshot = lambda force=False: None
    status = observer.required_artifact_status(False)
    assert status['complete'] is False
    assert 'forensic/maps/robot1_map_final.npz' in status['missing']
    assert 'forensic/maps/robot2_map_final.npz' not in status['missing']


@pytest.mark.parametrize('condition', ['B', 'C', 'D'])
def test_two_robot_conditions_retain_two_robot_artifact_contract(observer,
                                                                 condition):
    del condition
    class FakeForensic:
        def save_final_maps(self, *args, **kwargs):
            pass

        def flush(self):
            pass

        def close(self):
            pass

        def manifest(self):
            return {"files": []}

    observer.robots = ['robot1', 'robot2']
    observer.forensic = FakeForensic()
    observer.forensic_supervisor_enabled = True
    observer.forensic_sync_enabled = True
    observer.forensic_snapshot = lambda force=False: None
    status = observer.required_artifact_status(False)
    assert 'forensic/maps/robot1_map_final.npz' in status['required']
    assert 'forensic/maps/robot2_map_final.npz' in status['required']
    assert 'forensic/supervisor_ground_truth.csv' in status['required']
    assert 'forensic/synchronized_map_frame.jsonl' in status['required']


def test_unknown_pose_requires_frontend_diagnostics_in_artifact_contract(observer):
    observer.p['initial_configuration_json'] = json.dumps({
        'unknown_initial_pose': True,
    })
    status = observer.required_artifact_status(False)
    assert status['complete'] is False
    assert sorted(status['missing']) == sorted([
        'frontend/robot1_unknown_pose_frontend.json',
        'frontend/robot1_consensus_diagnostics.jsonl',
        'frontend/robot1_physical_evidence_diagnostics.jsonl',
        'frontend/robot2_unknown_pose_frontend.json',
        'frontend/robot2_consensus_diagnostics.jsonl',
        'frontend/robot2_physical_evidence_diagnostics.jsonl',
    ])


def test_unknown_pose_artifacts_resolve_logger_run_id_collision(observer):
    """Frontend files may be created before logger collision suffixing."""
    observer.p['initial_configuration_json'] = json.dumps({
        'unknown_initial_pose': True,
    })
    sibling = observer.directory.parent / f'{observer.run_id}-01' / 'frontend'
    sibling.mkdir(parents=True)
    for robot in ('robot1', 'robot2'):
        for suffix in (
                'unknown_pose_frontend.json',
                'consensus_diagnostics.jsonl',
                'physical_evidence_diagnostics.jsonl'):
            (sibling / f'{robot}_{suffix}').write_text('{}\n')
    status = observer.required_artifact_status(False)
    assert status['complete'] is True
    assert status['missing'] == []
    assert status['frontend_directory'] == f'{observer.run_id}-01/frontend'


def test_unknown_pose_artifacts_resolve_unsuffixed_frontend_run(observer):
    """A suffixed logger finds the launcher's unsuffixed frontend sibling."""
    observer.p['initial_configuration_json'] = json.dumps({
        'unknown_initial_pose': True,
    })
    base_run_id = observer.run_id
    observer.run_id = f'{base_run_id}-01'
    sibling = observer.directory.parent / base_run_id / 'frontend'
    sibling.mkdir(parents=True)
    for robot in ('robot1', 'robot2'):
        for suffix in (
                'unknown_pose_frontend.json',
                'consensus_diagnostics.jsonl',
                'physical_evidence_diagnostics.jsonl'):
            (sibling / f'{robot}_{suffix}').write_text('{}\n')
    status = observer.required_artifact_status(False)
    assert status['complete'] is True
    assert status['missing'] == []
    assert status['frontend_directory'] == f'{base_run_id}/frontend'


def test_unknown_pose_artifacts_resolve_frontend_created_at_observer_root(observer):
    """The runner may create observer/frontend before logger run allocation."""
    observer.p['initial_configuration_json'] = json.dumps({
        'unknown_initial_pose': True,
    })
    frontend = observer.directory.parent / 'frontend'
    frontend.mkdir(parents=True)
    for robot in ('robot1', 'robot2'):
        for suffix in (
                'unknown_pose_frontend.json',
                'consensus_diagnostics.jsonl',
                'physical_evidence_diagnostics.jsonl'):
            (frontend / f'{robot}_{suffix}').write_text('{}\n')
    status = observer.required_artifact_status(False)
    assert status['complete'] is True
    assert status['missing'] == []
    assert status['frontend_directory'] == 'frontend'


def test_no_handoff_does_not_require_shared_map_exports(observer):
    """Shared exports are required only after a shared-map topic is seen."""
    class PassiveForensic:
        def save_final_maps(self, *args, **kwargs):
            pass

        def flush(self):
            pass

        def close(self):
            pass

        def manifest(self):
            return {'files': []}

        def record_transform(self, *args, **kwargs):
            pass

    observer.forensic = PassiveForensic()
    observer.p['initial_configuration_json'] = json.dumps({
        'unknown_initial_pose': True,
    })
    status = observer.required_artifact_status(False)
    assert 'forensic/maps/robot1_shared_map_final.npz' not in status['required']
    assert 'forensic/maps/robot2_shared_map_final.npz' not in status['required']
    observer.shared_map_seen.add('robot1')
    status = observer.required_artifact_status(False)
    assert 'forensic/maps/robot1_shared_map_final.npz' in status['required']
    assert 'forensic/maps/robot2_shared_map_final.npz' in status['required']


def test_large_occupancy_grid_is_converted_and_counted_once(observer):
    message = OccupancyGrid()
    message.info.width = 320
    message.info.height = 320
    message.info.resolution = 0.01
    message.data = [-1] * 10000 + [0] * 90000 + [100] * 2400
    observer.mark("robot1", "map", message)
    assert observer.map_counts("robot1", "map") == (92400, 90000, 2400)
    first = observer._map_cache[("robot1", "map")]
    assert observer.map_counts("robot1", "map") == (92400, 90000, 2400)
    assert observer._map_cache[("robot1", "map")] is first


def test_concurrent_updates_and_finalization_do_not_write_closed_files(observer):
    failures = []

    def update(index):
        try:
            for sequence in range(100):
                observer.safe_call(
                    f"thread_{index}",
                    observer.event,
                    "NAVIGATION_PROGRESS",
                    f"{index}:{sequence}",
                )
                observer.safe_call("rosout", observer.rosout, controller_error())
        except Exception as exc:  # Test must expose errors outside fault boundaries.
            failures.append(exc)

    threads = [threading.Thread(target=update, args=(index,)) for index in range(3)]
    for thread in threads:
        thread.start()
    observer.finalize(False)
    for thread in threads:
        thread.join()
    assert failures == []
    assert read_json(observer.directory / "summary.json")["run"]["clean_shutdown"] is False


def test_continuous_cycle_status_and_suppression_are_reconciled(observer):
    started = ExplorationEvent()
    started.header.frame_id = "shared_map"
    started.source_robot_id = "robot1"
    started.event_type = "EXPLORATION_CYCLE_STARTED"
    started.cycle_number = 1
    started.claim_id = 1
    started.frontier_id = 42
    observer.coordinator_event("robot1", started)
    observer.trajectory.total_distance["robot1"] = 0.4
    ended = ExplorationEvent()
    ended.header.frame_id = "shared_map"
    ended.source_robot_id = "robot1"
    ended.event_type = "EXPLORATION_CYCLE_ENDED"
    ended.cycle_number = 1
    ended.claim_id = 1
    ended.frontier_id = 42
    ended.terminal_result = "UNKNOWN_NAV2_FAILURE"
    observer.coordinator_event("robot1", ended)
    suppression = ExplorationEvent()
    suppression.event_type = "FAILURE_SUPPRESSION_CREATED"
    suppression.claim_id = 1
    suppression.frontier_id = 42
    suppression.reason = "UNKNOWN_NAV2_FAILURE"
    observer.coordinator_event("robot1", suppression)
    exhausted = ExplorationStatus()
    exhausted.state = ExplorationStatus.NO_ELIGIBLE_CANDIDATES
    exhausted.reason = "locally_exhausted"
    observer.status("robot1", exhausted)
    summary = observer.summary(False)
    robot = summary["continuous_exploration"]["robot1"]
    assert robot["exploration_cycles"] == 1
    assert robot["failed_goals"] == 1
    assert robot["maximum_equivalent_region_attempt_count"] == 1
    assert summary["events"]["FAILURE_SUPPRESSION_CREATED"] == 1
