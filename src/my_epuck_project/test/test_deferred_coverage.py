import hashlib
import json
from types import SimpleNamespace

import numpy as np

import my_epuck_project.deferred_coverage as replay_module


def _map():
    return SimpleNamespace(
        header=SimpleNamespace(stamp=SimpleNamespace(sec=1, nanosec=0)),
        info=SimpleNamespace(
            width=3, height=1, resolution=0.03,
            origin=SimpleNamespace(
                position=SimpleNamespace(x=0.0, y=0.0),
                orientation=SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0))),
        data=[0, 100, -1])


class _Reader:
    def __init__(self, messages):
        self.messages = iter(messages)

    def open(self, *_args):
        return None

    def has_next(self):
        if hasattr(self, '_next'):
            return True
        try:
            self._next = next(self.messages)
            return True
        except StopIteration:
            return False

    def read_next(self):
        value = self._next
        del self._next
        return value


def test_coverage_replay_uses_causal_receipts_and_legacy_semantics(
        monkeypatch, tmp_path):
    bag = tmp_path / 'bag'
    bag.mkdir()
    payload = _map()
    messages = [
        ('/robot1/map', payload, 1),
        ('/robot2/map', payload, 2),
        ('/robot1/shared_map', payload, 3),
        ('/robot2/shared_map', payload, 4),
    ]
    fake_rosbag = SimpleNamespace(
        SequentialReader=lambda: _Reader(messages),
        StorageOptions=lambda **kwargs: kwargs,
        ConverterOptions=lambda **kwargs: kwargs,
    )
    monkeypatch.setitem(__import__('sys').modules, 'rosbag2_py', fake_rosbag)
    monkeypatch.setattr(replay_module, 'get_message', lambda _name: object())
    monkeypatch.setattr(replay_module, 'deserialize_message',
                        lambda serialized, _type: serialized)

    digest = hashlib.sha256(
        np.asarray(payload.data, dtype=np.int8).tobytes()).hexdigest()
    receipt_path = tmp_path / 'map_receipts.jsonl'
    with receipt_path.open('w', encoding='utf-8') as stream:
        for sequence, (robot, key) in enumerate((
                ('robot1', 'map'), ('robot2', 'map'),
                ('robot1', 'shared_map'), ('robot2', 'shared_map')), 1):
            stream.write(json.dumps({
                'sequence': sequence, 'robot_id': robot, 'map_key': key,
                'received_ros_time_s': 2.0,
                'received_wall_elapsed_s': 2.0,
                'data_sha256': digest,
            }) + '\n')
    coverage_path = tmp_path / 'coverage.csv'
    coverage_path.write_text(
        'ros_time_sec,ros_time_nanosec,wall_elapsed_s\n2,0,2.0\n',
        encoding='utf-8')
    result = replay_module.replay_coverage_from_bag(
        bag, receipt_path, coverage_path, ('robot1', 'robot2'),
        'shared_map', 0.03, (0.0, 0.0, 0.0), 2.0)
    semantic = result['rows'][0]['semantic']
    assert result['sample_count'] == 1
    assert semantic['robot1_shared_known'] == 2
    assert semantic['known_area_m2'] == 2 * 0.03 ** 2
    assert semantic['total_known_union_cells'] == 2

