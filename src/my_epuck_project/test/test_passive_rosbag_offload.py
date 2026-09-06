"""Lossless passive sensor offload and deterministic replay regressions."""

import json
from pathlib import Path

import pytest

import my_epuck_project.passive_rosbag as passive_rosbag
from my_epuck_project.passive_rosbag import (
    export_index_with_bounded_retry,
    export_required_topics,
    load_offloaded_timing,
    offloaded_sensor_topics,
    passive_topics,
    raw_bag_contract_status,
    recorder_command,
    recorded_topics,
    semantic_export_complete,
    scientific_raw_topics,
)
from my_epuck_project.raw_evidence_contract import (
    SCHEMA_VERSION, required_nonempty_topics, topic_spec,
)


def _write_rows(path, rows):
    with path.open('w', encoding='utf-8') as stream:
        for row in rows:
            stream.write(json.dumps(row, sort_keys=True) + '\n')


def test_offloaded_topic_set_is_lossless_and_disjoint():
    robots = ('robot1', 'robot2')
    base = set(passive_topics(robots))
    offloaded = offloaded_sensor_topics(robots)
    assert len(offloaded) == 9  # /clock plus four topics per robot
    assert base.isdisjoint(offloaded)
    selected = recorded_topics(robots, include_offloaded=True)
    assert len(selected) == len(set(selected))
    assert set(base) | set(offloaded) == set(selected)


def test_thin_contract_is_explicit_and_raw_only():
    robots = ('robot1', 'robot2')
    specs = topic_spec(robots)
    selected = scientific_raw_topics(robots)
    assert SCHEMA_VERSION.startswith('thin_raw_evidence_contract_')
    assert set(selected) == set(specs)
    assert '/clock' in required_nonempty_topics(robots)
    assert '/robot1/map' in required_nonempty_topics(robots)
    assert '/robot2/shared_map' in required_nonempty_topics(robots)
    assert '/robot1/task_snapshot' not in required_nonempty_topics(robots)
    assert '/cslam/unknown_pose/robot1/exploration_status' in selected
    assert '/cslam/unknown_pose/robot2/exploration_event' in selected
    assert len(selected) == len(set(selected))


def test_thin_recorder_topic_command_contains_custom_raw_contract(tmp_path):
    command, _ = recorder_command(
        tmp_path / 'thin' / 'passive_rosbag', ('robot1', 'robot2'),
        include_offloaded=False, include_scientific_raw=True)
    assert '/robot1/frontier_candidates' in command
    assert '/robot2/task_bids' in command
    assert '/robot1/map' in command
    assert '/tf_static' in command
    assert '/robot1/scan_d500_slam' not in command
    assert '/robot2/joint_states' not in command
    assert '/robot1/plan' in command
    assert '/robot1/navigate_to_pose/_action/feedback' in command
    assert '/rosout' not in command


def test_scientific_raw_recording_includes_action_path_evidence():
    selected = recorded_topics(
        ('robot1', 'robot2'), include_scientific_raw=True)
    assert '/robot1/navigate_to_pose/_action/status' in selected
    assert '/robot1/plan' in selected
    assert '/robot1/follow_path/_action/status' in selected
    assert '/robot1/compute_path_to_pose/_action/status' in selected


def test_if_published_topics_do_not_invalidate_complete_required_contract():
    required = ('/clock', '/robot1/odom')
    selected = required + ('/robot1/exploration_event',)
    status = raw_bag_contract_status(
        {'/clock': 'rosgraph_msgs/msg/Clock',
         '/robot1/odom': 'nav_msgs/msg/Odometry'},
        {'/clock': 3, '/robot1/odom': 4}, selected, required)
    assert status['complete'] is True
    assert status['missing_required_types'] == []
    assert status['missing_optional_topics'] == ['/robot1/exploration_event']


def test_scientific_export_required_set_excludes_if_published_topics():
    # The native exporter must apply the same required/non-required distinction
    # as the raw contract.  Optional topics may be absent from rosbag2's type
    # metadata when they never published, but required streams remain strict.
    required = export_required_topics(('robot1',), include_scientific_raw=True)
    assert '/robot1/exploration_event' not in required
    assert '/robot1/distributed_event' not in required
    assert '/robot1/odom' in required
    assert '/clock' in required


def test_missing_required_topic_type_still_invalidates_raw_contract():
    status = raw_bag_contract_status(
        {'/clock': 'rosgraph_msgs/msg/Clock'},
        {'/clock': 3}, ('/clock', '/robot1/odom'),
        ('/clock', '/robot1/odom'))
    assert status['complete'] is False
    assert status['missing_required_types'] == ['/robot1/odom']


def test_offloaded_replay_preserves_counts_timestamps_and_clock_mapping(tmp_path):
    export = tmp_path / 'export.jsonl'
    _write_rows(export, [
        {'topic': '/clock', 'bag_time_ns': 100, 'clock_s': 10.0},
        {'topic': '/robot1/scan_d500_slam', 'bag_time_ns': 110,
         'header_stamp_s': 9.9},
        {'topic': '/robot1/joint_states', 'bag_time_ns': 120,
         'header_stamp_s': 9.95},
        {'topic': '/clock', 'bag_time_ns': 200, 'clock_s': 11.0},
        {'topic': '/robot1/scan_d500_slam', 'bag_time_ns': 210,
         'header_stamp_s': 10.9},
    ])
    first = load_offloaded_timing(export)
    second = load_offloaded_timing(export)
    assert first == second
    assert [row['header_stamp_s'] for row in
            first['/robot1/scan_d500_slam']] == [9.9, 10.9]
    assert [row['received_sim_s'] for row in
            first['/robot1/scan_d500_slam']] == [10.0, 11.0]
    assert len(first['/robot1/joint_states']) == 1


def test_c_logger_removes_only_bag_owned_sensor_entities():
    source = (Path(__file__).parents[1] / 'my_epuck_project' /
              'cooperative_experiment_logger.py').read_text(encoding='utf-8')
    assert 'if not self.passive_sensor_offload_enabled:' in source
    assert "self.observe(Odometry" in source
    assert "self.observe(FrontierCandidateArray" in source
    assert "self.observe(TFMessage, '/tf'" in source
    assert 'include_offloaded=self.passive_sensor_offload_enabled' in source


def test_legacy_logger_exposes_opt_in_scientific_raw_capture():
    source = (Path(__file__).parents[1] / 'my_epuck_project' /
              'cooperative_experiment_logger.py').read_text(encoding='utf-8')
    assert "'enable_scientific_raw_capture': False" in source
    assert 'include_scientific_raw=bool(' in source
    assert "self.p.get('enable_scientific_raw_capture', False)" in source


def test_missing_clock_or_sensor_rows_remain_detectable():
    # The finalizer's required-topic contract includes /clock and every
    # offloaded stream; this guards against silently treating an empty export
    # as complete.
    assert '/clock' in offloaded_sensor_topics(('robot1',))
    assert '/robot1/scan_d500_fixed' in offloaded_sensor_topics(('robot1',))
    assert '/robot1/joint_states' in offloaded_sensor_topics(('robot1',))


def test_clock_qos_override_is_native_and_reaches_recorder_command(tmp_path):
    command, qos_path = recorder_command(
        tmp_path / 'new_run' / 'passive_rosbag', ('robot1', 'robot2'),
        include_offloaded=True)
    assert qos_path.is_file()
    assert qos_path.read_text(encoding='utf-8') == (
        '/clock:\n'
        '  history: keep_last\n'
        '  depth: 1\n'
        '  reliability: best_effort\n'
        '  durability: volatile\n')
    assert '--qos-profile-overrides-path' in command
    assert str(qos_path) in command
    assert '--include-hidden-topics' in command
    assert '/clock' in command
    assert '/robot1/navigate_to_pose/_action/status' in command
    assert '/robot2/follow_path/_action/status' in command


def _complete_export_metadata(robots=('robot1',)):
    topics = list(offloaded_sensor_topics(robots))
    return {
        'complete': True,
        'recorder_return_code': 0,
        'offloaded_topics': topics,
        'message_counts': {topic: 1 for topic in topics},
        'offloaded_reconstruction': {'complete': True},
    }


def test_zero_clock_or_incomplete_export_never_passes_semantic_gate():
    metadata = _complete_export_metadata()
    metadata['message_counts']['/clock'] = 0
    assert semantic_export_complete(metadata, include_offloaded=True) is False

    metadata = _complete_export_metadata()
    metadata['offloaded_reconstruction'] = {'complete': False}
    assert semantic_export_complete(metadata, include_offloaded=True) is False

    metadata = _complete_export_metadata()
    metadata['recorder_return_code'] = 1
    assert semantic_export_complete(metadata, include_offloaded=True) is False


def test_nonzero_required_streams_and_reconstruction_pass_semantic_gate():
    assert semantic_export_complete(
        _complete_export_metadata(), include_offloaded=True) is True


def test_export_retries_only_the_shutdown_conversion_error(monkeypatch):
    calls = []

    def flaky_export(*args, **kwargs):
        calls.append((args, kwargs))
        if len(calls) == 1:
            raise TypeError(
                'Unable to convert function return value to a Python type! '
                'The signature was (arg0: bytes, arg1: object) -> object')
        return {'complete': True, 'message_counts': {}}

    monkeypatch.setattr(passive_rosbag, 'export_index', flaky_export)
    result = export_index_with_bounded_retry(
        Path('/tmp/bag'), Path('/tmp/export.jsonl'), ('robot1',))
    assert result['complete'] is True
    assert len(calls) == 2


def test_export_does_not_retry_unrelated_type_error(monkeypatch):
    calls = []

    def broken_export(*args, **kwargs):
        calls.append((args, kwargs))
        raise TypeError('unrelated exporter conversion failure')

    monkeypatch.setattr(passive_rosbag, 'export_index', broken_export)
    with pytest.raises(TypeError, match='unrelated exporter'):
        export_index_with_bounded_retry(
            Path('/tmp/bag'), Path('/tmp/export.jsonl'), ('robot1',))
    assert len(calls) == 1


def test_rosbag_migration_does_not_remove_runtime_control_entities():
    source = (Path(__file__).parents[1] / 'my_epuck_project' /
              'cooperative_experiment_logger.py').read_text(encoding='utf-8')
    # The migration is limited to the already-approved passive sensor/action
    # entities; control-neutral odometry, TF, maps, candidates, and
    # cooperative state remain Python subscriptions.
    assert "self.observe(Odometry" in source
    assert "self.observe(TFMessage, '/tf'" in source
    assert "self.observe(FrontierCandidateArray" in source
    assert "self.observe(TaskSnapshot" in source
    assert "self.observe(PairDecision" in source
    assert 'RUNTIME_CONTROL_DEPENDENCY_REMOVED = NO' not in source
