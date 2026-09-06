from types import SimpleNamespace

import pytest
from builtin_interfaces.msg import Time as RosTime

import my_epuck_project.deferred_tf_trajectory as replay_module


def _odom(x, y, stamp=1):
    return SimpleNamespace(
        header=SimpleNamespace(
            frame_id='robot1/odom',
            stamp=RosTime(sec=stamp, nanosec=0)),
        pose=SimpleNamespace(pose=SimpleNamespace(
            position=SimpleNamespace(x=x, y=y, z=0.0),
            orientation=SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0))),
    )


class _Reader:
    def __init__(self, messages):
        self.messages = list(messages)
        self.index = 0

    def open(self, *_args):
        return None

    def get_all_topics_and_types(self):
        return [
            SimpleNamespace(name='/tf', type='tf2_msgs/msg/TFMessage'),
            SimpleNamespace(name='/tf_static', type='tf2_msgs/msg/TFMessage'),
            SimpleNamespace(name='/robot1/odom', type='nav_msgs/msg/Odometry'),
        ]

    def has_next(self):
        return self.index < len(self.messages)

    def read_next(self):
        item = self.messages[self.index]
        self.index += 1
        return item


class _MissingTypeReader(_Reader):
    def get_all_topics_and_types(self):
        return []


class _Buffer:
    def set_transform(self, *_args):
        return None

    def set_transform_static(self, *_args):
        return None

    def lookup_transform(self, *_args, **_kwargs):
        return SimpleNamespace(transform=SimpleNamespace(
            translation=SimpleNamespace(x=1.0, y=2.0, z=0.0),
            rotation=SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0)))


def test_complete_raw_tf_replay_uses_bag_order_and_tf2(monkeypatch, tmp_path):
    bag = tmp_path / 'bag'
    bag.mkdir()
    messages = [
        ('/tf_static', SimpleNamespace(transforms=[]), 1),
        ('/robot1/odom', _odom(3.0, 4.0), 2),
    ]
    fake_rosbag = SimpleNamespace(
        SequentialReader=lambda: _Reader(messages),
        StorageOptions=lambda **kwargs: kwargs,
        ConverterOptions=lambda **kwargs: kwargs,
    )
    monkeypatch.setitem(__import__('sys').modules, 'rosbag2_py', fake_rosbag)
    monkeypatch.setattr(replay_module, 'Buffer', _Buffer)
    monkeypatch.setattr(replay_module, 'get_message', lambda _name: object())
    monkeypatch.setattr(
        replay_module, 'deserialize_message', lambda payload, _type: payload)

    result = replay_module.replay_shared_trajectory_from_bag(
        bag, ('robot1',), 'shared_map')

    assert result['accepted_samples'] == 1
    assert result['skipped_transform_samples'] == 0
    assert result['deserialized_messages'] == 2
    assert result['summary']['bins_per_robot'] == {'robot1': 0}
    assert result['summary']['distance_travelled_m'] == {'robot1': 0.0}


def test_replay_fails_closed_when_required_tf_stream_is_missing(
        monkeypatch, tmp_path):
    bag = tmp_path / 'bag'
    bag.mkdir()
    fake_rosbag = SimpleNamespace(
        SequentialReader=lambda: _MissingTypeReader([]),
        StorageOptions=lambda **kwargs: kwargs,
        ConverterOptions=lambda **kwargs: kwargs,
    )
    monkeypatch.setitem(__import__('sys').modules, 'rosbag2_py', fake_rosbag)
    with pytest.raises(ValueError, match='raw trajectory topics missing'):
        replay_module.replay_shared_trajectory_from_bag(
            bag, ('robot1',), 'shared_map')
