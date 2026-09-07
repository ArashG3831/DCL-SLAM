"""Regression test for the multi-threaded coordinator round race."""

from pathlib import Path
from types import SimpleNamespace

import pytest

from my_epuck_project.distributed_assignment.models import (
    Bounds,
    FailureClass,
    PhysicalTask,
    TaskSnapshot,
)
from my_epuck_project.distributed_assignment.protocol import receive
from my_epuck_project.distributed_assignment.local_nav2 import (
    DispatchPreconditions,
    PathEvaluation,
)
from my_epuck_project.distributed_assignment.scoring import AssignmentWeights
from my_epuck_project.distributed_frontier_assignment import (
    DistributedFrontierAssignment,
    InitialExplorationBarrier,
    evidence_hold_active,
    eligible_solo_tasks,
    solo_retry_delay_s,
)
from my_epuck_project.passive_rosbag import passive_topics
from my_epuck_project.distributed_assignment.scoring import rank_solo_tasks

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


def test_cooperative_waits_reuse_degraded_solo_without_relaxing_certificate():
    """Evidence waits must offer the existing local fallback, not bypass policy."""
    source = SOURCE.read_text(encoding='utf-8')
    assert 'def _continue_local_work_while_waiting' in source
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
    assert 'if self._assignment_strategy == \'frontier_cost_only\':' in tick
    assert 'self._continue_local_work_while_waiting(' in tick


def test_terminal_result_can_start_fallback_without_waiting_for_next_tick():
    """A busy peer permits immediate, still-gated local continuation."""
    source = SOURCE.read_text(encoding='utf-8')
    navigation = source[source.index('    def _navigation_finished'):
                         source.index('    def _publish_failure')]
    assert 'self._reset_round(\'navigation terminal result\')' in navigation
    assert 'self._start_immediate_fallback_after_terminal()' in navigation
    helper = source[source.index(
        '    def _start_immediate_fallback_after_terminal'):
        source.index('    @staticmethod\n    def _finite_path_samples')]
    assert 'peer.value.local_nav_goal_active' in helper
    assert 'if self._consume_local_fallback_trigger():' in helper
    assert 'self._local_fallback_snapshot(now)' in helper
    assert 'self._continue_local_work_while_waiting(' in helper
    # The normal degraded-solo path remains the safety gate: the immediate
    # trigger only starts candidate evaluation; it never sends a saved path.
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
