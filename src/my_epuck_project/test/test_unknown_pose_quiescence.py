"""Focused post-handoff quiescence tests for the unknown-pose frontend."""

from collections import Counter, OrderedDict, deque
from types import SimpleNamespace

from my_epuck_project.unknown_pose_frontend import UnknownPoseFrontend


class _Future:
    def __init__(self):
        self.cancelled = False

    def cancel(self):
        self.cancelled = True
        return True


class _Executor:
    def __init__(self):
        self.submitted = []
        self.shutdown_args = None

    def submit(self, function, *args, **kwargs):
        future = _Future()
        self.submitted.append((function, args, kwargs, future))
        return future

    def shutdown(self, **kwargs):
        self.shutdown_args = kwargs


class _Listener:
    def __init__(self):
        self.unregistered = False

    def unregister(self):
        self.unregistered = True


def _quiescence_fixture():
    frontend = UnknownPoseFrontend.__new__(UnknownPoseFrontend)
    frontend._post_handoff_quiesced = False
    frontend._registration_shutdown = False
    frontend.accepted = object()
    frontend._last_evidence_status = False
    frontend.evidence_acquisition_started = True
    frontend._evidence_opportunity_deadline_wall = 12.0
    frontend.evidence_acquisition_deadline_wall = 13.0
    frontend.evidence_status_pub = object()
    frontend._record_diagnostic_event = lambda *args, **kwargs: None
    frontend._publish_evidence_status = (
        lambda active: setattr(frontend, '_last_evidence_status', bool(active)))
    frontend._registration_pending_contexts = deque(['queued'], maxlen=4)
    frontend._registration_pending_keys = {'queued'}
    frontend.pending_requests = {'request'}
    frontend.completed_request_keys = {'completed'}
    frontend.request_candidate_by_request_key = {'request': object()}
    frontend.request_metadata_by_request_key = {'request': object()}
    frontend.pending_descriptor_pair_keys = deque(['pair'])
    frontend.pending_descriptor_pair_key_set = {'pair'}
    frontend.pending_peer_descriptor_keys = deque(['peer'])
    frontend.pending_own_descriptor_keys = deque(['own'])
    frontend.pending_peer_evidence_announcements = {'evidence': object()}
    frontend.peer_evidence_announcements = {'evidence': object()}
    frontend.received_peer_crops = {'crop': object()}
    frontend.peer_descriptors = {'descriptor': object()}
    frontend.keyframes = {'keyframe': object()}
    frontend.keyframe_viewpoints = {'keyframe': object()}
    frontend.local_full_map_snapshots = {'local': object()}
    frontend.peer_full_map_snapshots = {'peer': object()}
    frontend.latest_full_map_snapshot = object()
    frontend.full_map_proposal = object()
    frontend.pending_proposals = {'proposal': object()}
    frontend.peer_proposals = {'proposal': object()}
    frontend._registration_context = object()
    frontend._full_map_registration_context = object()
    frontend._registration_future = _Future()
    frontend._full_map_registration_future = _Future()
    frontend._registration_executor = _Executor()
    frontend.timer = object()
    frontend.tf_listener = _Listener()
    frontend.tf_broadcaster = object()
    frontend.map_sub = object()
    frontend.peer_map_pub = object()
    for attribute in (
            'peer_full_map_sub', 'full_map_snapshot_request_sub',
            'full_map_snapshot_response_sub', 'descriptor_sub',
            'request_sub', 'crop_sub', 'hypothesis_sub', 'descriptor_pub',
            'request_pub', 'crop_pub', 'hypothesis_pub', 'evidence_status_pub',
            'full_map_snapshot_request_pub', 'full_map_snapshot_response_pub'):
        setattr(frontend, attribute, object())
    destroyed = []
    frontend.destroy_subscription = (
        lambda entity: destroyed.append(('subscription', entity)))
    frontend.destroy_publisher = (
        lambda entity: destroyed.append(('publisher', entity)))
    frontend.destroy_timer = lambda entity: destroyed.append(('timer', entity))
    frontend._destroyed = destroyed
    return frontend


def test_pre_lock_full_map_scheduler_still_submits_registration_work():
    frontend = UnknownPoseFrontend.__new__(UnknownPoseFrontend)
    frontend._post_handoff_quiesced = False
    frontend._registration_shutdown = False
    frontend.accepted = None
    frontend.robot_id = 'robot1'
    frontend.peer_robot_id = 'robot2'
    frontend.full_map_registration_period_s = 1.0
    frontend._last_full_map_attempt_ros_s = 0.0
    frontend._last_full_map_attempt_pair = None
    frontend._full_map_registration_future = None
    frontend._full_map_registration_sequence = 0
    frontend.latest_full_map_snapshot = {
        'id': 'local', 'fingerprint': 'local', 'robot_id': 'robot1',
        'timestamp_ns': 1, 'crop': None}
    frontend.peer_full_map_snapshots = OrderedDict((
        ('peer', {'id': 'peer', 'fingerprint': 'peer', 'robot_id': 'robot2',
                  'timestamp_ns': 2, 'crop': None}),))
    frontend._ros_time_s = lambda: 2.0
    frontend._full_map_pair_operable = lambda source, target: (True, {})
    frontend._capture_full_map_attempt = lambda *args: None
    frontend._record_diagnostic_event = lambda *args, **kwargs: None
    frontend._registration_executor = _Executor()

    assert frontend._maybe_schedule_full_map_registration() is None
    assert len(frontend._registration_executor.submitted) == 1


def test_post_lock_quiesces_registration_entities_but_retains_relay_and_tf():
    frontend = _quiescence_fixture()
    retained_tf = frontend.tf_broadcaster
    frontend._enter_post_handoff_quiescence()

    assert frontend._post_handoff_quiesced
    assert frontend._registration_shutdown
    assert frontend._registration_future is None
    assert frontend._full_map_registration_future is None
    assert not frontend._registration_pending_contexts
    assert frontend._registration_executor.shutdown_args == {
        'wait': False, 'cancel_futures': True}
    assert frontend.tf_broadcaster is retained_tf
    assert frontend.map_sub is not None
    assert frontend.peer_map_pub is not None
    assert frontend.tf_listener is None
    assert len(frontend._destroyed) == 15


def test_post_lock_local_map_callback_still_triggers_rate_limited_relay():
    frontend = UnknownPoseFrontend.__new__(UnknownPoseFrontend)
    frontend._post_handoff_quiesced = True
    frontend.accepted = object()
    frontend.full_map_registration = True
    frontend.first_map_wall = None
    frontend.map_revision = 0
    frontend.last_export_ros_s = 0.0
    frontend.last_export_map_fingerprint = 'old'
    frontend._ros_time_s = lambda: 5.0
    frontend._store_local_full_map_snapshot = (
        lambda: (_ for _ in ()).throw(AssertionError('snapshot copied')))
    frontend._publish_full_map_snapshot_if_due = (
        lambda: (_ for _ in ()).throw(AssertionError('registration export')))
    relayed = []
    frontend.publish_local_map = lambda: relayed.append(True)
    frontend.counters = Counter()
    message = SimpleNamespace(
        info=SimpleNamespace(
            width=1, height=1, resolution=0.03,
            origin=SimpleNamespace(
                position=SimpleNamespace(x=0.0, y=0.0, z=0.0),
                orientation=SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0))),
        data=[0])

    frontend.map_callback(message)

    assert frontend.latest_map is message
    assert relayed == [True]


def test_post_lock_callbacks_and_schedulers_cannot_restart_registration():
    frontend = _quiescence_fixture()
    frontend._enter_post_handoff_quiescence()
    executor = frontend._registration_executor
    executor.submitted.clear()
    frontend.latest_full_map_snapshot = {
        'id': 'local', 'fingerprint': 'local', 'robot_id': 'robot1'}
    frontend.peer_full_map_snapshots = OrderedDict((
        ('peer', {'id': 'peer', 'fingerprint': 'peer', 'robot_id': 'robot2'}),))
    frontend.robot_id = 'robot1'
    frontend.peer_robot_id = 'robot2'
    frontend.accepted = object()
    frontend._full_map_registration_future = None
    frontend._ros_time_s = lambda: 100.0
    frontend.full_map_registration_period_s = 1.0
    frontend._last_full_map_attempt_ros_s = 0.0

    assert frontend._maybe_schedule_full_map_registration() is None
    assert frontend._start_registration_context((None,) * 12) is False
    assert executor.submitted == []
