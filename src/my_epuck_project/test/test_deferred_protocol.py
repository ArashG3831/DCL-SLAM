from types import SimpleNamespace

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
