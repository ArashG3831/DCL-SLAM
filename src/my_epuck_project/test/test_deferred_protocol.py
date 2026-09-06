from types import SimpleNamespace
import json

import pytest

import my_epuck_project.deferred_protocol as replay_module


def _decision(round_id, union_hash, decision_hash, robot1='', robot2='',
              diagnostics='{}'):
    return SimpleNamespace(
        round_id=round_id,
        union_hash=union_hash,
        decision_hash=decision_hash,
        robot1_canonical_task_id=robot1,
        robot2_canonical_task_id=robot2,
        diagnostics_json=diagnostics,
    )


class _Reader:
    def __init__(self, messages):
        self.messages = iter(messages)
        self._next = None

    def open(self, *_args):
        return None

    def get_all_topics_and_types(self):
        return [SimpleNamespace(
            name='/robot1/pair_decision',
            type='my_epuck_interfaces/msg/PairDecision'),
                SimpleNamespace(
            name='/robot2/pair_decision',
            type='my_epuck_interfaces/msg/PairDecision')]

    def has_next(self):
        if self._next is not None:
            return True
        try:
            self._next = next(self.messages)
        except StopIteration:
            return False
        return True

    def read_next(self):
        value = self._next
        self._next = None
        return value


def test_pair_decision_outcome_reuses_legacy_classification():
    outcome, diagnostics = replay_module.pair_decision_outcome(_decision(
        'r1', 'u1', 'd1', diagnostics=(
            '{"union_task_count": 2, "robot1_valid_bid_count": 1}')))
    assert outcome == 'IDLE_BY_DETERMINISTIC_ASSIGNMENT'
    assert diagnostics['union_task_count'] == 2

    outcome, _ = replay_module.pair_decision_outcome(
        _decision('r2', 'u2', 'd2', robot1='task'))
    assert outcome == 'DISPATCHABLE_ASSIGNMENT'


def test_pair_decision_authority_switch_requires_parity():
    live = {'IDLE_BY_DETERMINISTIC_ASSIGNMENT': 2}
    replay = {
        'status': 'PARITY_PASS',
        'deferred': {'DISPATCHABLE_ASSIGNMENT': 1},
    }
    assert replay_module.select_pair_decision_outcomes(live, replay) == (
        {'DISPATCHABLE_ASSIGNMENT': 1}, 'deferred')

    failed = {
        'status': 'DEFERRED_REPLAY_FAILED',
        'deferred': {'DISPATCHABLE_ASSIGNMENT': 1},
    }
    assert replay_module.select_pair_decision_outcomes(live, failed) == (
        live, 'live')


def test_pair_decision_replay_preserves_live_duplicate_suppression(
        monkeypatch, tmp_path):
    bag = tmp_path / 'bag'
    bag.mkdir()
    first = _decision('r1', 'u1', 'd1', robot1='task')
    duplicate = _decision('r1', 'u1', 'd1', robot1='task')
    second = _decision('r2', 'u2', 'd2', diagnostics='{"union_task_count": 0}')
    messages = [
        ('/robot1/pair_decision', first, 1),
        ('/robot1/pair_decision', duplicate, 2),
        ('/robot2/pair_decision', second, 3),
    ]
    class FakeRosbag:
        SequentialReader = staticmethod(lambda: _Reader(messages))
        StorageOptions = staticmethod(lambda **kwargs: kwargs)
        ConverterOptions = staticmethod(lambda **kwargs: kwargs)
    monkeypatch.setitem(__import__('sys').modules, 'rosbag2_py', FakeRosbag())
    monkeypatch.setattr(replay_module, 'get_message', lambda _name: object())
    monkeypatch.setattr(replay_module, 'deserialize_message',
                        lambda serialized, _type: serialized)

    replay = replay_module.replay_pair_decisions_from_bag(
        bag, ('robot1', 'robot2'))
    assert replay['round_outcomes'] == {
        'DISPATCHABLE_ASSIGNMENT': 1,
        'NO_CANONICAL_TASKS': 1,
    }
    assert replay['deserialized_messages'] == 3
    assert len(replay['records']) == 2


def test_agreement_replay_counts_only_agreement_events(monkeypatch, tmp_path):
    bag = tmp_path / 'bag'
    bag.mkdir()
    messages = [
        ('/robot1/distributed_event', SimpleNamespace(
            event_type='DECISION_AGREED', round_id='r1', decision_hash='d1'), 1),
        ('/robot2/distributed_event', SimpleNamespace(
            event_type='STATE_TRANSITION', round_id='r1', decision_hash='d1'), 2),
        ('/robot1/distributed_event', SimpleNamespace(
            event_type='DECISION_AGREED', round_id='r1', decision_hash='d1'), 3),
        ('/robot2/distributed_event', SimpleNamespace(
            event_type='DECISION_AGREED', round_id='r2', decision_hash=''), 4),
    ]
    class FakeRosbag:
        SequentialReader = staticmethod(lambda: _Reader(messages))
        StorageOptions = staticmethod(lambda **kwargs: kwargs)
        ConverterOptions = staticmethod(lambda **kwargs: kwargs)
    monkeypatch.setitem(__import__('sys').modules, 'rosbag2_py', FakeRosbag())
    monkeypatch.setattr(replay_module, 'get_message', lambda _name: object())
    monkeypatch.setattr(replay_module, 'deserialize_message',
                        lambda serialized, _type: serialized)

    replay = replay_module.replay_agreement_counters_from_bag(
        bag, ('robot1', 'robot2'))
    assert replay['agreement_publications'] == 3
    assert replay['unique_agreed_rounds'] == 2
    assert replay['unique_agreed_decisions'] == 1
    assert replay['deserialized_messages'] == 4


def test_warning_replay_reuses_legacy_deduplication_semantics(tmp_path):
    receipts = tmp_path / 'rosout_receipts.jsonl'
    receipts.write_text(
        '\n'.join([
            '{"level":30,"name":"costmap","message":"update 1.0",'
            '"wall_time_utc":"t1"}',
            '{"level":30,"name":"costmap","message":"update 2.0",'
            '"wall_time_utc":"t2"}',
            '{"level":20,"name":"ignored","message":"info",'
            '"wall_time_utc":"t3"}',
        ]) + '\n', encoding='utf-8')
    replay = replay_module.replay_warning_records_from_receipts(receipts)
    assert replay['receipt_count'] == 3
    assert replay['warning_receipt_count'] == 2
    assert len(replay['records']) == 1
    record = replay['records'][0]
    assert record['category'] == 'COSTMAP_WARNING'
    assert record['occurrence_count'] == 2
    assert record['normalized_message'] == 'update <num>'


def test_warning_replay_rejects_corrupt_receipt(tmp_path):
    receipts = tmp_path / 'rosout_receipts.jsonl'
    receipts.write_text('{not-json}\n', encoding='utf-8')
    with pytest.raises(ValueError, match='invalid rosout receipt'):
        replay_module.replay_warning_records_from_receipts(receipts)


def test_nav2_diagnostic_replay_preserves_rate_limit_and_category(tmp_path):
    receipts = tmp_path / 'rosout_receipts.jsonl'
    rows = [
        {'level': 30, 'name': 'planner_server',
         'message': 'missed its desired rate 10.0', 'elapsed_s': 1.0,
         'source_stamp_sec': 1, 'source_stamp_nanosec': 2},
        {'level': 30, 'name': 'planner_server',
         'message': 'missed its desired rate 10.0', 'elapsed_s': 1.1,
         'source_stamp_sec': 1, 'source_stamp_nanosec': 3},
        {'level': 40, 'name': 'planner_server',
         'message': 'missed its desired rate 10.0', 'elapsed_s': 1.4,
         'source_stamp_sec': 1, 'source_stamp_nanosec': 4},
    ]
    receipts.write_text(
        ''.join(json.dumps(row) + '\n' for row in rows), encoding='utf-8')
    replay = replay_module.replay_nav2_diagnostics_from_receipts(receipts)
    assert replay['receipt_count'] == 3
    assert len(replay['records']) == 2
    assert replay['records'][0]['category'] == 'MISSED_RATE_WARNING'


def test_nav2_diagnostic_replay_preserves_final_artifact_fields(tmp_path):
    receipts = tmp_path / 'rosout_receipts.jsonl'
    receipts.write_text(json.dumps({
        'schema_version': '1.1.0',
        'run_id': 'run-1',
        'receipt_sequence': 7,
        'wall_time_utc': 'receipt-time',
        'wall_elapsed_s': 1.1,
        'level': 40,
        'name': 'planner_server',
        'message': 'planner failure',
        'elapsed_s': 12.5,
        'source_stamp_sec': 12,
        'source_stamp_nanosec': 50,
        'diagnostic_event_sequence': 19,
        'diagnostic_wall_time_utc': 'diagnostic-time',
        'diagnostic_ros_time_sec': 12,
        'diagnostic_ros_time_nanosec': 50,
        'diagnostic_elapsed_s': 12.6,
        'diagnostic_wall_elapsed_s': 1.2,
    }) + '\n', encoding='utf-8')
    replay = replay_module.replay_nav2_diagnostics_from_receipts(receipts)
    assert replay['authority_ready'] is True
    assert replay['records'] == [{
        'schema_version': '1.1.0',
        'run_id': 'run-1',
        'event_sequence': 19,
        'wall_time_utc': 'diagnostic-time',
        'ros_time_sec': 12,
        'ros_time_nanosec': 50,
        'elapsed_s': 12.6,
        'wall_elapsed_s': 1.2,
        'robot_id': None,
        'source': '/rosout:planner_server',
        'severity': 'ERROR',
        'category': 'CONTROLLER_OR_PLANNER_ERROR',
        'message': 'planner failure',
        'node': 'planner_server',
        'source_stamp_sec': 12,
        'source_stamp_nanosec': 50,
    }]
