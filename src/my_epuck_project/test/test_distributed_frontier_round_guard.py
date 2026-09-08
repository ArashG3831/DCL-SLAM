"""Regression test for the multi-threaded coordinator round race."""

from pathlib import Path
import time
from types import SimpleNamespace

import pytest

from my_epuck_interfaces.msg import DistributedExplorationStatus

from my_epuck_project.distributed_assignment.models import (
    Bounds,
    CoordinatorState,
    FailureClass,
    PhysicalTask,
    TaskSnapshot,
)
from my_epuck_project.distributed_assignment.protocol import PeerLiveness, receive
from my_epuck_project.distributed_assignment.ros_conversion import text_to_uuid
from my_epuck_project.distributed_assignment.local_nav2 import (
    DispatchPreconditions,
    NavigationOutcome,
    PathEvaluation,
)
from my_epuck_project.distributed_assignment.scoring import AssignmentWeights
from my_epuck_project.distributed_frontier_assignment import (
    ActiveNavigationAction,
    DISTRIBUTED_EVENT_QOS,
    DistributedFrontierAssignment,
    InitialExplorationBarrier,
    evidence_hold_active,
    eligible_solo_tasks,
    solo_retry_delay_s,
)
from my_epuck_project.passive_rosbag import passive_topics
from my_epuck_project.distributed_assignment.scoring import rank_solo_tasks
from my_epuck_project.distributed_assignment.traffic_scheduler import schedule_traffic

SOURCE = Path(__file__).parents[1] / 'my_epuck_project' / (
    'distributed_frontier_assignment.py'
)
LOCAL_NAV2_SOURCE = Path(__file__).parents[1] / 'my_epuck_project' / (
    Path('distributed_assignment') / 'local_nav2.py'
)
FRONTIER_GENERATOR_SOURCE = Path(__file__).parents[2] / (
    Path('my_epuck_frontier_candidates') / 'src' /
    'frontier_candidate_generator.cpp'
)


def test_distributed_event_qos_is_explicit_reliable_volatile_keep_last():
    """Live allocator event endpoints use one compatible non-latched QoS."""
    assert DISTRIBUTED_EVENT_QOS.history.name == 'KEEP_LAST'
    assert DISTRIBUTED_EVENT_QOS.depth == 50
    assert DISTRIBUTED_EVENT_QOS.reliability.name == 'RELIABLE'
    assert DISTRIBUTED_EVENT_QOS.durability.name == 'VOLATILE'
    source = SOURCE.read_text(encoding='utf-8')
    assert "DistributedExplorationEvent, 'distributed_event'," in source
    assert "self._peer_event_callback, DISTRIBUTED_EVENT_QOS" in source
    assert source.count('DISTRIBUTED_EVENT_QOS') >= 3
    assert 'DistributedExplorationEvent, \'distributed_event\', 50' not in source


def test_observer_distributed_event_qos_remains_compatible():
    """The observer continues requesting reliable/volatile event delivery."""
    logger = (Path(__file__).parents[1] / 'my_epuck_project' /
              'cooperative_experiment_logger.py').read_text(encoding='utf-8')
    assert "self.qos(True,False,50),f'{r}.distributed_event'" in logger


def test_initial_barrier_blocks_one_ready_replica():
    barrier = InitialExplorationBarrier(enabled=True)
    barrier.observe_local_ready()
    assert barrier.dispatch_allowed is False
    assert barrier.maybe_release() is False


def test_initial_barrier_releases_only_after_both_ready():
    barrier = InitialExplorationBarrier(enabled=True)
    barrier.observe_local_ready()
    barrier.observe_peer_ready()
    assert barrier.maybe_release() is True
    assert barrier.dispatch_allowed is True
    assert barrier.maybe_release() is False


def test_initial_barrier_is_order_independent_and_idempotent():
    first = InitialExplorationBarrier(enabled=True)
    second = InitialExplorationBarrier(enabled=True)
    first.observe_local_ready()
    first.observe_peer_ready()
    second.observe_peer_ready()
    second.observe_local_ready()
    first.observe_peer_ready()
    second.observe_local_ready()
    assert first.maybe_release() is True
    assert second.maybe_release() is True
    assert first.dispatch_allowed == second.dispatch_allowed == True


def test_initial_barrier_handoff_supersedes_waiting_release():
    barrier = InitialExplorationBarrier(enabled=True)
    barrier.observe_local_ready()
    barrier.observe_handoff()
    barrier.observe_peer_ready()
    assert barrier.maybe_release() is False
    assert barrier.dispatch_allowed is False


def test_single_robot_barrier_bypasses_missing_peer():
    barrier = InitialExplorationBarrier(enabled=False)
    assert barrier.dispatch_allowed is True
    assert barrier.maybe_release() is False


def test_initial_barrier_launches_for_unknown_pose_local_nodes():
    launch = (Path(__file__).parents[1] / 'launch' /
              'two_robots_decentralized_exploration_launch.py').read_text()
    assert "'initial_peer_readiness_barrier': True" in launch
    assignment = SOURCE.read_text()
    assert 'INITIAL_LOCAL_READY' in assignment
    assert 'INITIAL_PEER_READY_OBSERVED' in assignment
    assert 'INITIAL_EXPLORATION_BARRIER_RELEASED' in assignment
    assert 'INITIAL_LOCAL_EXPLORATION_SKIPPED_DUE_TO_HANDOFF' in assignment


def test_local_tick_and_dispatch_selection_share_initial_barrier_guard():
    source = SOURCE.read_text()
    tick = source[source.index('    def _tick(self)'):
                  source.index('    def _traffic_for_decision')]
    solo = source[source.index('    def _continue_degraded_solo'):
                  source.index('    def _continue_bidding')]
    assert '_initial_exploration_barrier.dispatch_allowed' in tick
    assert '_initial_exploration_barrier.dispatch_allowed' in solo


def test_common_start_release_is_required_before_any_exploration_send():
    source = SOURCE.read_text(encoding='utf-8')
    assert "'common_start_release_required'" in source
    assert 'def _exploration_dispatch_allowed' in source
    assert 'not self._exploration_dispatch_allowed()' in source
    dispatch = source[source.index('    def _start_local_dispatch'):]
    assert "'common START_RELEASE barrier pending'" in dispatch


def test_shared_launch_releases_once_after_both_assignment_peers_ready():
    launch = (Path(__file__).parents[1] / 'launch' /
              'two_robots_decentralized_exploration_launch.py').read_text()
    activation = (Path(__file__).parents[1] / 'my_epuck_project' /
                  'unknown_pose_shared_stack_activation.py').read_text()
    assert 'common_start_release_required' in launch
    assert 'publish_cooperative_start_ready' in launch
    assert "'/cslam/unknown_pose/start_release'" in activation
    assert '_cooperative_start_ready.values()' in activation
    assert 'START_RELEASE' in activation


def test_passive_rosbag_topic_set_is_explicit_and_lossless():
    topics = passive_topics(('robot1', 'robot2'))
    assert len(topics) == 10
    assert '/robot1/plan' in topics
    assert '/robot2/navigate_to_pose/_action/feedback' in topics
    assert len(set(topics)) == len(topics)


def test_first_actionable_pair_is_gated_by_shared_tf_and_logs_startup_milestones():
    """Avoid publishing a known-invalid first pair during TF startup."""
    source = SOURCE.read_text(encoding='utf-8')
    tick = source[source.index('    def _tick(self)'):
                  source.index('    def _traffic_for_decision')]
    tf_gate = tick.index('tf_ready, tf_age_s, tf_reason = self._nav2.shared_tf_status()')
    # The call may pass optional diagnostic arguments after the required round
    # and generation arguments; keep this source-order guard independent of
    # those optional arguments.
    decision_publish = tick.index('self._publish_decision')
    assert tf_gate < decision_publish
    assert "'SHARED_TF_READY'" in tick
    assert "'FIRST_VALID_TASK_SNAPSHOTS'" in tick
    assert "'FIRST_VALID_PAIR_DECISION'" in tick
    assert "'FIRST_COOPERATIVE_GOAL'" in source


def test_evidence_pacing_uses_ros_simulation_time_not_wall_time():
    """Fast-mode evidence cadence must not be stretched by the host RTF."""
    source = (Path(__file__).parents[1] / 'my_epuck_project' /
              'unknown_pose_frontend.py').read_text(encoding='utf-8')
    assert 'def _ros_time_s(self)' in source
    assert '_last_full_map_export_ros_s' in source
    assert '_last_full_map_attempt_ros_s' in source
    assert 'self.last_export_ros_s' in source


def test_tick_owns_a_local_round_and_abandons_replaced_rounds():
    """A terminal callback must not make _tick dereference None."""
    text = SOURCE.read_text(encoding='utf-8')
    assert 'round_work = self._round' in text
    assert 'if round_work is None:' in text
    assert 'self._round_is_current(round_work, generation)' in text
    assert 'self._round_lifecycle.generation' in text


def test_local_dispatch_has_score_selection_and_common_final_path_gate():
    """The local branch cannot fall back to canonical-first dispatch."""
    source = SOURCE.read_text(encoding='utf-8')
    assert 'rank_solo_tasks(' in source
    assert 'if not path_is_valid_finite(final_path):' in source
    assert 'maximum_solo_path_m' not in source


def test_successful_physical_frontier_is_not_redispatched_while_present():
    """A tiny successful residual must not cause same-task goal churn."""
    completed = PhysicalTask(
        'robot1', 'session', 1, 7, 'completed-region', 1,
        (1.0, 1.0), Bounds((0.9, 0.9), (1.1, 1.1)), (1.0, 1.0),
        visible_reveal_gain=1.0, local_ordering_score=1.0,
        local_path_valid=True, local_path_length_m=0.06,
    )
    alternate = PhysicalTask(
        'robot1', 'session', 1, 7, 'new-region', 2,
        (2.0, 2.0), Bounds((1.9, 1.9), (2.1, 2.1)), (2.0, 2.0),
        visible_reveal_gain=1.0, local_ordering_score=1.0,
        local_path_valid=True, local_path_length_m=1.0,
    )
    selected = eligible_solo_tasks(
        (completed, alternate), set(), {'completed-region'}, 0.05, 0.0,
    )
    assert [task.physical_signature for task in selected] == ['new-region']


def _task(signature, score, gain, path, samples=()):
    return PhysicalTask(
        'robot1', 'session', 1, 7, signature, 1,
        (path, 0.0), Bounds((path - 0.1, -0.1), (path + 0.1, 0.1)),
        (path, 0.0), visible_reveal_gain=gain,
        local_ordering_score=score, local_path_valid=True,
        local_path_length_m=path, local_path=tuple(samples),
    )


def test_solo_priority_uses_generator_score_before_identity_tie_breaking():
    """A canonical/hash ordering cannot replace the generator's score."""
    weak = _task('00000000', 0.20, 0.10, 1.0, ((0.0, 0.0), (1.0, 0.0)))
    strong = _task('ffffffff', 0.90, 2.0, 5.0, ((0.0, 0.0), (5.0, 0.0)))
    assert rank_solo_tasks((weak, strong))[0] is strong


@pytest.mark.parametrize('robot_id', ('robot1', 'robot2'))
def test_degraded_solo_score_and_finite_path_check_are_symmetric(robot_id):
    """Both robot instances use score priority and the same path validity."""
    weak = _task('00000000', 0.20, 0.10, 1.0,
                 ((0.0, 0.0), (1.0, 0.0)))
    strong = _task('ffffffff', 0.90, 2.0, 5.0,
                   ((0.0, 0.0), (5.0, 0.0)))
    weak = weak.__class__(**{**weak.__dict__, 'source_robot_id': robot_id})
    strong = strong.__class__(
        **{**strong.__dict__, 'source_robot_id': robot_id})
    eligible = eligible_solo_tasks((weak, strong), set(), set(), 0.05, 0.0)
    assert rank_solo_tasks(eligible)[0] is strong

    class FakeLogger:
        def info(self, *_args, **_kwargs):
            pass

        def warning(self, *_args, **_kwargs):
            pass

    class FakeNav2:
        def __init__(self):
            self.send_count = 0

        def send_navigation(self, *_args, **_kwargs):
            self.send_count += 1
            return True

    member = strong.__class__(
        **{**strong.__dict__, 'local_path_length_m': 17.9})
    task = SimpleNamespace(members=(member,), canonical_id='canonical-strong')
    fresh = PathEvaluation(
        True, 18.01, ((0.0, 0.0), (18.01, 0.0)), 0, 0, '',
        failure_class=FailureClass.UNKNOWN,
    )
    checks = DispatchPreconditions(
        action_server_ready=True, lifecycle_active=True,
        transform_available=True, transform_age_s=0.1,
        goal_inside_map=True, goal_inside_costmap=True,
        goal_map_value=0, goal_costmap_value=0,
        local_path_clear=True, no_local_goal_active=True,
        final_path_valid=True, reason='',
    )
    node = DistributedFrontierAssignment.__new__(DistributedFrontierAssignment)
    fake_nav = FakeNav2()
    invalidations = []
    node.get_logger = lambda: FakeLogger()
    node._robot_id = robot_id
    node._local_only = True
    node._active_task = task
    node._active_round_id = 'degraded-test'
    node._solo_route_history = ()
    node._weights = AssignmentWeights()
    node._nav2 = fake_nav
    node._dispatch_count = 0
    node._dispatch_in_progress = False
    node._traffic_reallocation_after_clear = False
    node._released_traffic_winner_robot_id = ''
    node._first_cooperative_goal_logged = True
    node._emit_event = lambda *args, **kwargs: None
    node._transition = lambda *args, **kwargs: None
    node._invalidate_round = lambda *args: invalidations.append(args)
    node._dispatch_after_checks(task, fresh, checks)
    assert fake_nav.send_count == 1
    assert not invalidations


def test_solo_route_penalty_prefers_novel_frontier_but_allows_necessary_transit():
    """Route memory discourages returns without banning necessary transit."""
    history = (((0.0, 0.0), (1.0, 0.0), (2.0, 0.0)),)
    returning = _task('returning', 0.90, 1.0, 2.0,
                      ((0.0, 0.0), (1.0, 0.0), (2.0, 0.0)))
    novel = _task('novel', 0.70, 1.0, 3.0,
                  ((0.0, 0.0), (0.0, 1.0), (0.0, 2.0)))
    assert rank_solo_tasks((returning, novel), history)[0] is novel
    assert rank_solo_tasks((returning,), history)[0] is returning


def test_solo_eligibility_requires_valid_finite_stored_path():
    """Invalid candidates cannot enter selection, regardless of distance."""
    invalid = _task('invalid', 1.0, 1.0, 1.0)
    invalid = invalid.__class__(
        **{**invalid.__dict__, 'local_path_valid': False})
    long_path = _task('long', 1.0, 1.0, 50.0)
    assert eligible_solo_tasks(
        (invalid, long_path), set(), set(), 0.05, 0.0) == (long_path,)


def test_retryable_failures_use_bounded_exponential_backoff():
    assert [solo_retry_delay_s(count) for count in range(1, 7)] == [
        1, 2, 4, 8, 16, 30]
    assert solo_retry_delay_s(100) == 30.0


@pytest.mark.parametrize('robot_id', ('robot1', 'robot2'))
def test_evidence_hold_lease_is_symmetric_and_expires(robot_id):
    """A live frontend lease blocks new goals, then expires safely."""
    del robot_id  # Both IDs use the same pure lease policy.
    assert evidence_hold_active(10.0, 9.99)
    assert not evidence_hold_active(10.0, 10.0)
    assert not evidence_hold_active(float('nan'), 0.0)


@pytest.mark.parametrize('robot_id', ('robot1', 'robot2'))
def test_evidence_hold_does_not_abort_active_goal_for_each_robot(robot_id):
    """The advisory lease must not turn evidence into a Nav2 failure."""
    class FakeLogger:
        def info(self, *_args, **_kwargs):
            pass

    class FakeNav2:
        local_goal_active = True

        def __init__(self):
            self.cancel_count = 0

        def cancel_navigation(self):
            self.cancel_count += 1
            return True

    node = DistributedFrontierAssignment.__new__(DistributedFrontierAssignment)
    node._robot_id = robot_id
    node._nav2 = FakeNav2()
    node._evidence_hold_timeout_s = 2.0
    node._evidence_hold_until_wall_s = {'robot1': 0.0, 'robot2': 0.0}
    node.get_logger = lambda: FakeLogger()

    node._evidence_status_callback(SimpleNamespace(data=True), robot_id)
    node._evidence_status_callback(SimpleNamespace(data=True), robot_id)

    assert node._nav2.cancel_count == 0


def test_frontend_evidence_hold_is_runtime_observable_without_pose_data():
    """The allocator hold uses only a bounded frontend status signal."""
    frontend = (Path(__file__).parents[1] / 'my_epuck_project' /
                'unknown_pose_frontend.py').read_text(encoding='utf-8')
    coordinator = SOURCE.read_text(encoding='utf-8')
    assert 'evidence_acquisition_active' in frontend
    assert 'EVIDENCE_ACQUISITION_STATUS' in frontend
    assert 'evidence_hold_active(' in coordinator
    assert 'evidence acquisition opportunity active' in coordinator
    assert 'RelativePoseHypothesis' not in coordinator.split(
        'def _evidence_status_callback', 1)[1].split(
            'def _tick', 1)[0]


def test_peer_activity_is_not_a_global_assignment_barrier():
    """An idle robot may seek independent work while peer navigation is active."""
    text = SOURCE.read_text(encoding='utf-8')
    assert 'peer_navigation_blocks_dispatch' not in text
    assert 'active peer goal is not a global assignment barrier' in text


def test_cooperative_waits_require_peer_loss_before_degraded_solo():
    """Transient evidence waits must preserve pair traffic coordination."""
    source = SOURCE.read_text(encoding='utf-8')
    assert 'def _continue_local_work_while_waiting' in source
    assert 'def _peer_unavailable_for_degraded_solo' in source
    assert "temporary local work while cost-only certificate is pending" in source
    assert "temporary local work while peer bid is pending" in source
    assert "temporary local work while peer agreement is pending" in source
    assert "_cost_only_dispatch_certificate(" in source


def test_degraded_solo_selection_advertises_a_peer_reservation():
    """The temporary local task is visible and excluded by canonical hard IDs."""
    source = SOURCE.read_text(encoding='utf-8')
    assert "'DEGRADED_SOLO_COMMITMENT'" in source
    assert 'active_canonical_task_id' in source
    assert '_temporary_peer_reservation_ids(now)' in source


def test_waiting_fallback_marks_selected_work_as_degraded_solo():
    node = DistributedFrontierAssignment.__new__(DistributedFrontierAssignment)
    node._local_only = False
    node._peer_id = 'robot2'
    node._peer_status = None
    node._peer_liveness = PeerLiveness(timeout_s=0.0)
    node._peer_liveness.observe('lost-peer', 0.0)
    node._nav2 = SimpleNamespace(local_goal_active=False)
    node._dispatch_in_progress = False
    node._active_task = None
    node._active_decision_hash = ''
    transitions = []
    published = []
    events = []
    node._transition = lambda state, reason: transitions.append((state, reason))
    node._publish_status = lambda: published.append(True)
    node._emit_event = lambda *args, **kwargs: events.append(args)
    node._continue_degraded_solo = lambda snapshot: (
        setattr(node, '_active_task', SimpleNamespace(canonical_id='temporary')),
        setattr(node, '_active_decision_hash', 'DEGRADED_SOLO'),
    )
    assert node._continue_local_work_while_waiting(
        object(), 'certificate evidence pending')
    assert transitions[0][0].value == 'DEGRADED_SOLO'
    assert published == [True]
    assert events[0][0] == 'DEGRADED_SOLO_COMMITMENT'


def test_live_peer_conflict_cannot_dispatch_through_degraded_solo():
    """A known peer keeps a conflicting pair on the traffic-scheduled path."""
    conflict = schedule_traffic(
        ((0.0, 0.0), (1.0, 0.0)),
        ((0.5, -1.0), (0.5, 1.0)),
        robot1_safe_radius_m=0.08,
        robot2_safe_radius_m=0.08,
        reference_speed_mps=0.13,
    )
    assert conflict.conflict

    node = DistributedFrontierAssignment.__new__(DistributedFrontierAssignment)
    now = time.monotonic()
    node._local_only = False
    node._peer_id = 'robot2'
    node._peer_liveness = PeerLiveness(timeout_s=6.0)
    node._peer_liveness.observe('live-peer', now)
    node._peer_status = receive(SimpleNamespace(), 3.0, now)
    node._snapshots = {}
    node._nav2 = SimpleNamespace(local_goal_active=False)
    node._dispatch_in_progress = False
    node._active_task = None
    node._active_decision_hash = ''
    attempted = []
    node._continue_degraded_solo = attempted.append

    assert not node._continue_local_work_while_waiting(
        object(), 'peer bid pending')
    assert attempted == []
    assert node._peer_liveness.state == CoordinatorState.WAITING_FOR_INPUTS


def _fallback_node(task, receipt=0.0, ttl=3.0):
    node = DistributedFrontierAssignment.__new__(DistributedFrontierAssignment)
    snapshot = TaskSnapshot(
        'robot1', 'local-session', 1, 7, 'map-fingerprint', 100, ttl,
        (task,),
    )
    node._robot_id = 'robot1'
    node._snapshots = {'robot1': receive(snapshot, ttl, receipt)}
    node._current_local_frontier_ids = {
        'robot1': frozenset((str(task.local_frontier_id),)),
        'robot2': None,
    }
    return node


def test_unchanged_local_seed_survives_expired_peer_evidence_ttl():
    """Local fallback is not coupled to the short peer/bid freshness TTL."""
    task = _task('local-seed', 1.0, 1.0, 1.0)
    node = _fallback_node(task)
    node._bid_batches = {
        'robot2': receive(SimpleNamespace(), 3.0, 0.0),
    }
    assert node._bid_batches['robot2'].fresh(4.0) is False
    fallback = node._local_fallback_snapshot(4.0)
    assert fallback is not None
    assert fallback.tasks == (task,)


def test_disappeared_or_changed_frontier_is_not_reused_from_stale_seed():
    """A current candidate batch can invalidate an expired local seed."""
    task = _task('local-seed', 1.0, 1.0, 1.0)
    node = _fallback_node(task)
    node._current_local_frontier_ids['robot1'] = frozenset()
    assert node._local_fallback_snapshot(4.0) is None
    node._current_local_frontier_ids['robot1'] = frozenset(('different-id',))
    assert node._local_fallback_snapshot(4.0) is None


def test_current_path_and_reservation_gates_remain_after_stale_seed_reuse():
    """Reusing a seed never bypasses current Nav2 or peer-reservation gates."""
    source = SOURCE.read_text(encoding='utf-8')
    solo = source[source.index('    def _continue_degraded_solo'):
                  source.index('    def _continue_bidding')]
    tick = source[source.index('    def _tick'):
                  source.index('    def _traffic_for_decision')]
    assert "caller='DEGRADED_SOLO_DISPATCH'" in solo
    assert 'if not result.valid:' in solo
    assert 'check_dispatch_preconditions' in solo
    assert '_temporary_peer_reservation_ids(now)' in tick


def test_cooperative_pair_path_remains_the_normal_resume_path():
    """Fresh paired snapshots still enter the existing canonical round path."""
    source = SOURCE.read_text(encoding='utf-8')
    tick = source[source.index('    def _tick'):
                  source.index('    def _traffic_for_decision')]
    assert 'build_canonical_union(' in tick
    assert 'choose_pair_assignment' in tick
    assert 'traffic_compatible' in tick
    assert 'self._traffic_for_bid_pair(' in tick
    assert 'if self._assignment_strategy == \'frontier_cost_only\':' in tick
    assert 'self._continue_local_work_while_waiting(' in tick


def test_terminal_result_resumes_pair_path_while_peer_is_live():
    """Immediate fallback still passes through the peer-loss safety gate."""
    source = SOURCE.read_text(encoding='utf-8')
    navigation = source[source.index('    def _navigation_finished'):
                         source.index('    def _publish_failure')]
    assert 'self._reset_round(\'navigation terminal result\')' in navigation
    assert 'self._start_immediate_fallback_after_terminal()' in navigation
    helper = source[source.index(
        '    def _start_immediate_fallback_after_terminal'):
        source.index('    @staticmethod\n    def _finite_path_samples')]
    assert 'if self._consume_local_fallback_trigger():' in helper
    assert 'self._local_fallback_snapshot(now)' in helper
    assert 'self._continue_local_work_while_waiting(' in helper
    # The cooperative fallback boundary now requires peer loss before this
    # helper can start candidate evaluation.
    assert 'self._peer_unavailable_for_degraded_solo(now)' in source
    assert "caller='DEGRADED_SOLO_DISPATCH'" in source


def test_current_local_candidate_preempts_pending_cooperative_wait():
    """Current local work enters the existing fallback before another round."""
    source = SOURCE.read_text(encoding='utf-8')
    candidate = source[source.index('    def _candidate_callback'):
                       source.index('    @staticmethod\n    def _decode_frontier_regions')]
    tick = source[source.index('    def _tick_impl'):
                  source.index('    def _traffic_for_decision')]
    assert '_local_fallback_trigger_pending = True' in candidate
    assert 'def _consume_local_fallback_trigger' in source
    assert 'if self._consume_local_fallback_trigger():' in tick
    assert 'self._continue_local_work_while_waiting(local, reason)' in source
    assert "caller='DEGRADED_SOLO_DISPATCH'" in source
    assert 'check_dispatch_preconditions' in source


def test_empty_current_local_batch_clears_fallback_trigger():
    """A disappeared source batch cannot leave stale fallback work armed."""
    source = SOURCE.read_text(encoding='utf-8')
    candidate = source[source.index('    def _candidate_callback'):
                       source.index('    @staticmethod\n    def _decode_frontier_regions')]
    branch = candidate[candidate.index(
        'if message.source_robot_id == getattr(self, \'_robot_id\', None)'):
        candidate.index('fallback = CandidateEvidence(')]
    assert 'if message.candidates:' in branch
    assert 'self._local_fallback_trigger_pending = False' in branch


def test_local_fallback_trigger_remains_safety_gated():
    """The trigger cannot bypass current path or final dispatch checks."""
    source = SOURCE.read_text(encoding='utf-8')
    helper = source[source.index('    def _consume_local_fallback_trigger'):
                    source.index('    def _temporary_peer_reservation_ids')]
    assert '_local_fallback_snapshot(time.monotonic())' in helper
    assert 'self._continue_local_work_while_waiting(local, reason)' in helper
    solo = source[source.index('    def _continue_degraded_solo'):
                  source.index('    def _continue_bidding')]
    assert "caller='DEGRADED_SOLO_DISPATCH'" in solo
    assert 'check_dispatch_preconditions' in solo


class _NavigationOwnershipNav2:
    """ROS-free action client double for allocator ownership races."""

    def __init__(self):
        self.local_goal_active = False
        self.callback = None
        self.cancel_count = 0

    def send_navigation(self, _task, callback, diagnostic_path=()):
        del diagnostic_path
        self.callback = callback
        self.local_goal_active = True
        return True

    def cancel_navigation(self):
        self.cancel_count += 1
        return self.local_goal_active


class _NavigationOwnershipLogger:
    def info(self, *_args, **_kwargs):
        pass

    def warning(self, *_args, **_kwargs):
        pass

    def error(self, *_args, **_kwargs):
        pass


def _navigation_ownership_node():
    member = _task(
        'ownership-task', 1.0, 1.0, 1.0,
        ((0.0, 0.0), (1.0, 0.0)),
    )
    task = SimpleNamespace(members=(member,), canonical_id='ownership-canonical')
    node = DistributedFrontierAssignment.__new__(DistributedFrontierAssignment)
    node._robot_id = 'robot1'
    node._local_only = True
    node._active_task = task
    node._active_round_id = 'ownership-round'
    node._active_decision_hash = 'ownership-decision'
    node._active_navigation_action = None
    node._navigation_action_sequence = 0
    node._nav2 = _NavigationOwnershipNav2()
    node._dispatch_count = 0
    node._dispatch_in_progress = False
    node._traffic_reallocation_after_clear = False
    node._released_traffic_winner_robot_id = ''
    node._first_cooperative_goal_logged = True
    node._active_dispatch_path = ()
    node._active_commitments = {}
    node._completed_solo_physical_signatures = set()
    node._completed_shared_canonical_ids = set()
    node._solo_route_history = []
    node._solo_retry_not_before = {}
    node._solo_retry_counts = {}
    node._failure_task_diagnostics = {}
    node._hard_failure_signatures = {}
    node._hard_failure_counts = {}
    node._weights = AssignmentWeights()
    node._post_goal_settle_s = 1.0
    node._last_semantic_fingerprint = ''
    node._last_solo_snapshot_key = None
    node._settle_until_steady_s = 0.0
    node._transition = lambda *_args, **_kwargs: None
    node._events = []
    node._emit_event = lambda *args, **kwargs: node._events.append((args, kwargs))
    node._published_failures = []
    node._publish_failure = lambda *args, **kwargs: node._published_failures.append(
        (args, kwargs))
    node._reset_round = lambda *_args, **_kwargs: None
    node._start_immediate_fallback_after_terminal = lambda: None
    node.get_logger = lambda: _NavigationOwnershipLogger()
    node.get_clock = lambda: SimpleNamespace(
        now=lambda: SimpleNamespace(nanoseconds=1),
    )
    return node, task


def _successful_navigation_outcome():
    return NavigationOutcome(
        True, 4, 0, '', FailureClass.UNKNOWN, 1.0, 1.0, 0,
    )


def _dispatch_ready_checks():
    return DispatchPreconditions(
        action_server_ready=True, lifecycle_active=True,
        transform_available=True, transform_age_s=0.1,
        goal_inside_map=True, goal_inside_costmap=True,
        goal_map_value=0, goal_costmap_value=0,
        local_path_clear=True, no_local_goal_active=True,
        final_path_valid=True, reason='',
    )


def _dispatch_path():
    return PathEvaluation(
        True, 1.0, ((0.0, 0.0), (1.0, 0.0)), 0, 0, '',
        failure_class=FailureClass.UNKNOWN,
    )


def test_accepted_goal_then_round_invalidation_keeps_terminal_identity():
    """Round invalidation must not orphan an already submitted Nav2 goal."""
    node, task = _navigation_ownership_node()
    node._dispatch_after_checks(task, _dispatch_path(), _dispatch_ready_checks())

    action = node._active_navigation_action
    assert action is not None
    assert node._nav2.callback is not None
    DistributedFrontierAssignment._invalidate_round(
        node, FailureClass.EXPLICIT_CANCELLATION, 'test round invalidation',
    )
    assert node._active_navigation_action is action
    assert node._active_task is task

    node._nav2.local_goal_active = False
    node._nav2.callback(_successful_navigation_outcome())

    assert node._active_navigation_action is None
    assert node._active_task is None
    navigation_events = [
        (args, kwargs) for args, kwargs in node._events
        if args and args[0] == 'NAVIGATION_SUCCEEDED'
    ]
    assert len(navigation_events) == 1
    assert navigation_events[0][1]['action'].canonical_task_id == (
        'ownership-canonical')


def test_pending_send_then_late_acceptance_is_cancelable_and_attributed():
    """A late goal response still closes the original action record safely."""
    node, task = _navigation_ownership_node()
    node._dispatch_after_checks(task, _dispatch_path(), _dispatch_ready_checks())
    action = node._active_navigation_action
    assert action is not None
    assert action.state == 'ACTIVE'

    DistributedFrontierAssignment._invalidate_round(
        node, FailureClass.EXPLICIT_CANCELLATION, 'pending-send race',
    )
    assert node._active_navigation_action is action
    assert node._request_navigation_cancel()
    assert action.state == 'CANCELLING'
    assert node._nav2.cancel_count == 1

    node._nav2.local_goal_active = False
    node._nav2.callback(_successful_navigation_outcome())
    assert node._active_navigation_action is None
    assert any(
        args and args[0] == 'NAVIGATION_SUCCEEDED' and
        kwargs['action'].action_id == action.action_id
        for args, kwargs in node._events
    )


def test_stale_navigation_terminal_cannot_clear_new_action():
    """An old callback cannot publish or mutate semantic allocator state."""
    node, new_task = _navigation_ownership_node()
    old_member = _task(
        'old-ownership-task', 1.0, 1.0, 1.0,
        ((0.0, 0.0), (1.0, 0.0)),
    )
    old_task = SimpleNamespace(
        members=(old_member,), canonical_id='old-ownership-canonical',
    )
    old_action = ActiveNavigationAction(
        'robot1:old:old-ownership-canonical', old_task,
        old_task.canonical_id, old_member.physical_signature,
        'old-round', 'old-decision', 1,
    )
    new_action = ActiveNavigationAction(
        'robot1:new:ownership-canonical', new_task,
        new_task.canonical_id, new_task.members[0].physical_signature,
        'new-round', 'new-decision', 2, state='ACTIVE',
    )
    node._active_navigation_action = new_action
    node._active_task = new_task
    node._nav2.local_goal_active = True
    before_completed_solo = set(node._completed_solo_physical_signatures)
    before_completed_shared = set(node._completed_shared_canonical_ids)
    before_route_history = list(node._solo_route_history)
    before_retry_not_before = dict(node._solo_retry_not_before)
    before_retry_counts = dict(node._solo_retry_counts)

    DistributedFrontierAssignment._navigation_finished(
        node, _successful_navigation_outcome(), old_action,
    )

    assert old_action.state == 'TERMINAL'
    assert node._active_navigation_action is new_action
    assert node._active_task is new_task
    assert node._events == []
    assert node._published_failures == []
    assert node._completed_solo_physical_signatures == before_completed_solo
    assert node._completed_shared_canonical_ids == before_completed_shared
    assert node._solo_route_history == before_route_history
    assert node._solo_retry_not_before == before_retry_not_before
    assert node._solo_retry_counts == before_retry_counts


def test_stale_navigation_failure_cannot_publish_or_suppress():
    """An old failure cannot create hard-failure or retry suppression."""
    node, new_task = _navigation_ownership_node()
    old_member = _task(
        'old-failure-task', 1.0, 1.0, 1.0,
        ((0.0, 0.0), (1.0, 0.0)),
    )
    old_task = SimpleNamespace(
        members=(old_member,), canonical_id='old-failure-canonical',
    )
    old_action = ActiveNavigationAction(
        'robot1:old:old-failure-canonical', old_task,
        old_task.canonical_id, old_member.physical_signature,
        'old-failure-round', 'old-failure-decision', 1,
    )
    new_action = ActiveNavigationAction(
        'robot1:new:ownership-canonical', new_task,
        new_task.canonical_id, new_task.members[0].physical_signature,
        'new-round', 'new-decision', 2, state='ACTIVE',
    )
    node._active_navigation_action = new_action
    node._active_task = new_task
    node._nav2.local_goal_active = True

    DistributedFrontierAssignment._navigation_finished(
        node,
        NavigationOutcome(
            True, 6, 105, 'no progress', FailureClass.CONTROLLER_NO_PROGRESS,
            2.0, 0.2, 1,
        ),
        old_action,
    )

    assert old_action.state == 'TERMINAL'
    assert node._active_navigation_action is new_action
    assert node._hard_failure_signatures == {}
    assert node._hard_failure_counts == {}
    assert node._published_failures == []
    assert node._events == []
    assert node._solo_retry_not_before == {}
    assert node._solo_retry_counts == {}


def _peer_navigation_event_node():
    member = _task('peer-physical', 1.0, 1.0, 1.0)
    task = SimpleNamespace(members=(member,))
    commitment = SimpleNamespace(
        canonical_id='peer-canonical',
        source_session_id='11' * 16,
        decision_round_id='peer-round',
        decision_hash='peer-decision',
        task=task,
    )
    node = DistributedFrontierAssignment.__new__(DistributedFrontierAssignment)
    node._robot_id = 'robot1'
    node._peer_id = 'robot2'
    node._active_commitments = {'robot2': commitment}
    node._completed_shared_canonical_ids = set()
    node._clear_calls = []
    node._reset_calls = []
    node._clear_active_commitment = lambda robot_id, reason: (
        node._clear_calls.append((robot_id, reason)),
        node._active_commitments.pop(robot_id, None),
    )
    node._reset_round = lambda *args: node._reset_calls.append(args)
    node._round = None
    node._peer_status = None
    node._peer_liveness = PeerLiveness(10.0)
    node.get_logger = lambda: _NavigationOwnershipLogger()
    return node, commitment, member


def _peer_navigation_event(commitment, member, **changes):
    values = {
        'source_robot_id': 'robot2',
        'event_type': 'NAVIGATION_SUCCEEDED',
        'source_session_id': text_to_uuid(commitment.source_session_id),
        'canonical_task_id': commitment.canonical_id,
        'physical_task_signature': member.physical_signature,
        'round_id': commitment.decision_round_id,
        'decision_hash': commitment.decision_hash,
    }
    values.update(changes)
    return SimpleNamespace(**values)


@pytest.mark.parametrize('field,value', [
    ('canonical_task_id', 'wrong-canonical'),
    ('physical_task_signature', 'wrong-physical'),
    ('round_id', 'wrong-round'),
    ('decision_hash', 'wrong-decision'),
    ('source_session_id', text_to_uuid('22' * 16)),
])
def test_peer_rejects_navigation_event_with_any_identity_mismatch(field, value):
    """Canonical-ID equality alone cannot release a peer commitment."""
    node, commitment, member = _peer_navigation_event_node()
    message = _peer_navigation_event(commitment, member, **{field: value})

    node._peer_event_callback(message)

    assert node._clear_calls == []
    assert node._reset_calls == []
    assert node._completed_shared_canonical_ids == set()
    assert node._active_commitments['robot2'] is commitment


def test_peer_accepts_exact_navigation_success_identity():
    """A fully matching current terminal event preserves existing behavior."""
    node, commitment, member = _peer_navigation_event_node()
    node._peer_event_callback(_peer_navigation_event(commitment, member))

    assert len(node._clear_calls) == 1
    assert node._clear_calls[0][0] == 'robot2'
    assert node._completed_shared_canonical_ids == {'peer-canonical'}


def _terminal_drain_node(canonical_id, physical_signature, round_id,
                         decision_hash):
    """Build a peer shell for a terminal racing an inactive heartbeat."""
    member = _task(physical_signature, 1.0, 1.0, 1.0,
                   ((0.0, 0.0), (1.0, 0.0)))
    task = SimpleNamespace(members=(member,))
    commitment = SimpleNamespace(
        canonical_id=canonical_id,
        source_session_id='11' * 16,
        decision_round_id=round_id,
        decision_hash=decision_hash,
        task=task,
    )
    node = DistributedFrontierAssignment.__new__(DistributedFrontierAssignment)
    node._robot_id = 'robot2'
    node._peer_id = 'robot1'
    node._active_commitments = {'robot1': commitment}
    node._completed_shared_canonical_ids = set()
    node._clear_calls = []
    node._reset_calls = []
    node._round = None
    node._peer_status = None
    node._peer_liveness = PeerLiveness(10.0)
    node._clear_active_commitment = lambda robot_id, reason: (
        node._clear_calls.append((robot_id, reason)),
        node._active_commitments.pop(robot_id, None),
    )
    node._reset_round = lambda *args: node._reset_calls.append(args)
    node.get_logger = lambda: _NavigationOwnershipLogger()
    status = SimpleNamespace(
        source_robot_id='robot1',
        source_session_id=text_to_uuid(commitment.source_session_id),
        validity=SimpleNamespace(sec=10, nanosec=0),
        state=DistributedExplorationStatus.WAITING_FOR_INPUTS,
        local_nav_goal_active=False,
        active_canonical_task_id='',
    )
    terminal = SimpleNamespace(
        source_robot_id='robot1',
        event_type='NAVIGATION_SUCCEEDED',
        source_session_id=text_to_uuid(commitment.source_session_id),
        canonical_task_id=canonical_id,
        physical_task_signature=physical_signature,
        round_id=round_id,
        decision_hash=decision_hash,
    )
    return node, status, terminal, commitment


@pytest.mark.parametrize('canonical_id,physical_signature,round_id,decision_hash', [
    (
        '2e45d1635c936e029ece1c04',
        '90ebe16146a8a8975c115c65',
        'continuation-action-round:238cea5d886360f5efaf47d00ecc6983dde1d6d6c34132ac8a8b237d25638bb1',
        'bc8e67f8847c806e50e590111ffd73df88447720bea49d7c1a4fef221843c6bd',
    ),
    (
        '9c9161cca1cfdb3d5394bf35',
        '60e6716feac743c5504d0ad5',
        '45e9832c065bd87a1d0717cc4e63e7c6899be5cd32a7dbe68b36c4a3f24f7113',
        '25225bd2ed1ec368d959d7401b3979c1d19e82ff88060833803bd5e2ddc9f278',
    ),
])
def test_peer_commitment_survives_inactive_status_until_exact_terminal(
        canonical_id, physical_signature, round_id, decision_hash):
    """The 2e45/9c916 races must drain through the strict terminal matcher."""
    node, status, terminal, commitment = _terminal_drain_node(
        canonical_id, physical_signature, round_id, decision_hash,
    )

    node._status_callback(status)

    assert node._active_commitments['robot1'] is commitment
    node._peer_event_callback(terminal)

    assert node._completed_shared_canonical_ids == {canonical_id}
    assert node._active_commitments == {}
    assert len(node._clear_calls) == 1


def test_active_different_peer_task_still_invalidates_old_commitment():
    """An actively advertised replacement remains a hard identity change."""
    node, commitment, _ = _peer_navigation_event_node()
    node._status_callback(SimpleNamespace(
        source_robot_id='robot2',
        source_session_id=text_to_uuid(commitment.source_session_id),
        validity=SimpleNamespace(sec=10, nanosec=0),
        state=DistributedExplorationStatus.NAVIGATING,
        local_nav_goal_active=True,
        active_canonical_task_id='new-active-task',
    ))

    assert node._active_commitments == {}
    assert node._clear_calls


def test_failed_navigation_clears_only_its_action_after_attribution():
    """Normal Nav2 failure remains attributable and releases ownership once."""
    node, task = _navigation_ownership_node()
    node._dispatch_after_checks(task, _dispatch_path(), _dispatch_ready_checks())
    action = node._active_navigation_action
    node._nav2.local_goal_active = False
    node._nav2.callback(NavigationOutcome(
        True, 6, 104, 'controller stopped', FailureClass.CONTROLLER_NO_PROGRESS,
        2.0, 0.2, 1,
    ))

    assert action.state == 'TERMINAL'
    assert node._active_navigation_action is None
    assert node._active_task is None
    assert any(
        args and args[0] == 'NAVIGATION_FAILED' and
        kwargs['action'] is action
        for args, kwargs in node._events
    )


def test_existing_cancel_paths_mark_action_cancelling_without_clearing_it():
    """Explicit timeout/handoff cancellation retains asynchronous ownership."""
    node, task = _navigation_ownership_node()
    node._dispatch_after_checks(task, _dispatch_path(), _dispatch_ready_checks())
    action = node._active_navigation_action
    assert action is not None
    assert node._request_navigation_cancel()
    assert action.state == 'CANCELLING'
    assert node._active_navigation_action is action


def test_degraded_solo_retries_when_compute_path_lease_is_busy():
    """Planner-lease contention preserves the local fallback seed."""
    source = SOURCE.read_text(encoding='utf-8')
    solo = source[source.index('    def _continue_degraded_solo'):
                  source.index('    def _continue_bidding')]
    assert "caller='DEGRADED_SOLO_DISPATCH'" in solo
    assert 'self._local_fallback_trigger_pending = True' in solo
    assert 'waiting for local ComputePathToPose query lease' in solo
    assert 'self._last_solo_snapshot_key = None' in solo
    assert "PATH_QUERY_LEASE_BUSY" in solo
    assert "if failure_reason != 'PATH_QUERY_LEASE_BUSY':" in solo
    assert 'degraded solo local path action unavailable' in solo


def test_fallback_priority_uses_the_shared_serialized_planner_lease():
    """Urgent fallback yields between bulk queries without concurrency."""
    nav2 = LOCAL_NAV2_SOURCE.read_text(encoding='utf-8')
    generator = FRONTIER_GENERATOR_SOURCE.read_text(encoding='utf-8')
    assert "PATH_QUERY_LEASE_BUSY" in nav2
    assert '_request_path_priority()' in nav2
    assert '_clear_path_priority()' in nav2
    assert 'fallback_priority_requested()' in generator
    assert 'schedule_query_retry(10ms)' in generator
    assert 'schedule_query_retry(1ms)' in generator
    assert 'FRONTIER_GENERATOR_LEASE_SUMMARY' in generator


def test_planner_lease_priority_preserves_single_query_and_failed_path_gates():
    """Priority changes ordering only; path validity still gates dispatch."""
    nav2 = LOCAL_NAV2_SOURCE.read_text(encoding='utf-8')
    generator = FRONTIER_GENERATOR_SOURCE.read_text(encoding='utf-8')
    assert 'self._path_callback is not None' in nav2
    assert 'if self._path_callback is not None:' in nav2
    assert 'if failure_reason != \'PATH_QUERY_LEASE_BUSY\':' in SOURCE.read_text(
        encoding='utf-8')
    assert 'active_request_' in generator
    assert 'release_path_lock();\n        schedule_query_retry' in generator
    assert 'planner_->async_send_goal' in generator


def test_planner_lease_profiling_is_opt_in_and_does_not_change_selection():
    """The new lease counters are diagnostics-only and keep policy inputs."""
    generator = FRONTIER_GENERATOR_SOURCE.read_text(encoding='utf-8')
    assert 'MY_EPUCK_FRONTIER_CANDIDATE_TIMING' in generator
    assert 'maximum_path_queries_per_cycle_' in generator
    assert 'fair_frontier_query_order' in generator
    assert 'selection_policy_' in generator
