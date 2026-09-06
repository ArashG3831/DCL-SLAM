from types import SimpleNamespace

import my_epuck_project.deferred_navigation as replay_module


class _Reader:
    def __init__(self, messages, topics):
        self.messages = iter(messages)
        self._next = None
        self.topics = topics

    def open(self, *_args):
        return None

    def get_all_topics_and_types(self):
        return [SimpleNamespace(name=topic, type=type_name)
                for topic, type_name in self.topics.items()]

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


def _status(value, goal_id):
    return SimpleNamespace(
        status_list=[SimpleNamespace(
            goal_info=SimpleNamespace(goal_id=SimpleNamespace(
                uuid=bytes([goal_id]) * 16)),
            status=value,
        )]
    )


def _feedback(recoveries, distance):
    return SimpleNamespace(
        feedback=SimpleNamespace(
            number_of_recoveries=recoveries,
            distance_remaining=distance,
            current_pose=SimpleNamespace(
                header=SimpleNamespace(stamp=SimpleNamespace(
                    sec=4, nanosec=5))),
        )
    )


def _path():
    point = lambda x, y: SimpleNamespace(
        pose=SimpleNamespace(position=SimpleNamespace(x=x, y=y)))
    return SimpleNamespace(
        header=SimpleNamespace(
            stamp=SimpleNamespace(sec=3, nanosec=4), frame_id='shared_map'),
        poses=[point(0.0, 0.0), point(0.3, 0.4), point(0.3, 0.9)])


def test_replay_navigation_evidence_reconstructs_status_path_and_recovery(
        monkeypatch, tmp_path):
    bag = tmp_path / 'bag'
    bag.mkdir()
    topics = {
        '/robot1/navigate_to_pose/_action/status':
            'action_msgs/msg/GoalStatusArray',
        '/robot1/navigate_to_pose/_action/feedback':
            'nav2_msgs/action/NavigateToPose_FeedbackMessage',
        '/robot1/follow_path/_action/status':
            'action_msgs/msg/GoalStatusArray',
        '/robot1/compute_path_to_pose/_action/status':
            'action_msgs/msg/GoalStatusArray',
        '/robot1/plan': 'nav_msgs/msg/Path',
    }
    messages = [
        ('/robot1/navigate_to_pose/_action/status', _status(1, 1), 10),
        ('/robot1/navigate_to_pose/_action/status', _status(4, 1), 20),
        ('/robot1/navigate_to_pose/_action/feedback', _feedback(0, 2.0), 30),
        ('/robot1/navigate_to_pose/_action/feedback', _feedback(1, 1.0), 40),
        ('/robot1/plan', _path(), 50),
    ]

    class FakeRosbag:
        SequentialReader = staticmethod(lambda: _Reader(messages, topics))
        StorageOptions = staticmethod(lambda **kwargs: kwargs)
        ConverterOptions = staticmethod(lambda **kwargs: kwargs)

    monkeypatch.setitem(__import__('sys').modules, 'rosbag2_py', FakeRosbag())
    monkeypatch.setattr(replay_module, 'get_message', lambda name: name)
    monkeypatch.setattr(replay_module, 'deserialize_message',
                        lambda serialized, _type: serialized)

    result = replay_module.replay_navigation_evidence_from_bag(
        bag, ('robot1',))
    assert result['complete'] is True
    assert result['message_counts'][
        '/robot1/navigate_to_pose/_action/status'] == 2
    assert result['status_counts'] == {
        'NAVIGATE_TO_POSE.ACCEPTED': 1,
        'NAVIGATE_TO_POSE.SUCCEEDED': 1,
    }
    assert len(result['recovery_transitions']) == 2
    assert result['recovery_transitions'][1]['recoveries'] == 1
    assert result['plan_records'][0]['pose_count'] == 3
    assert result['plan_records'][0]['path_length_m'] == 1.0


def test_replay_navigation_evidence_fails_closed_without_robot_status(
        monkeypatch, tmp_path):
    bag = tmp_path / 'bag'
    bag.mkdir()
    topics = {
        '/robot1/plan': 'nav_msgs/msg/Path',
    }

    class FakeRosbag:
        SequentialReader = staticmethod(lambda: _Reader([], topics))
        StorageOptions = staticmethod(lambda **kwargs: kwargs)
        ConverterOptions = staticmethod(lambda **kwargs: kwargs)

    monkeypatch.setitem(__import__('sys').modules, 'rosbag2_py', FakeRosbag())
    monkeypatch.setattr(replay_module, 'get_message', lambda name: name)
    monkeypatch.setattr(replay_module, 'deserialize_message',
                        lambda serialized, _type: serialized)

    result = replay_module.replay_navigation_evidence_from_bag(
        bag, ('robot1',))
    assert result['complete'] is False
    assert result['status'] == 'MISSING_NAVIGATION_STATUS'


def test_replay_navigation_evidence_accepts_present_not_invoked_status(
        monkeypatch, tmp_path):
    bag = tmp_path / 'bag'
    bag.mkdir()
    topics = {
        '/robot1/navigate_to_pose/_action/status':
            'action_msgs/msg/GoalStatusArray',
    }

    class FakeRosbag:
        SequentialReader = staticmethod(lambda: _Reader([], topics))
        StorageOptions = staticmethod(lambda **kwargs: kwargs)
        ConverterOptions = staticmethod(lambda **kwargs: kwargs)

    monkeypatch.setitem(__import__('sys').modules, 'rosbag2_py', FakeRosbag())
    monkeypatch.setattr(replay_module, 'get_message', lambda name: name)
    monkeypatch.setattr(replay_module, 'deserialize_message',
                        lambda serialized, _type: serialized)

    result = replay_module.replay_navigation_evidence_from_bag(
        bag, ('robot1',))
    assert result['complete'] is True
    assert result['status'] == 'COMPLETE'
    assert result['navigate_to_pose_invoked'] == {'robot1': False}
    assert result['status_transitions'] == []
