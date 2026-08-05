from my_epuck_project.pipeline_telemetry import (
    DwbStallDetector,
    StageMetrics,
    attenuation,
    command_kind,
    motion_state,
    stationary_cause,
)


def test_stage_metrics_keep_linear_angular_only_and_zero_separate():
    metrics = StageMetrics()
    metrics.observe(0.13, 0.0, 10.0)
    metrics.observe(0.0, 0.3, 10.1)
    metrics.observe(0.0, 0.0, 10.2)
    snapshot = metrics.snapshot(10.25)
    assert snapshot['message_count'] == 3
    assert snapshot['nonzero_linear_count'] == 1
    assert snapshot['angular_only_count'] == 1
    assert snapshot['zero_command_count'] == 1
    assert command_kind(snapshot) == 'ZERO'


def test_attenuation_identifies_suppression_and_reduction():
    prior = {'fresh': True, 'latest_linear_mps': 0.10}
    assert attenuation(prior, {'fresh': True, 'latest_linear_mps': 0.0})[
        'state'] == 'SUPPRESSED'
    assert attenuation(prior, {'fresh': True, 'latest_linear_mps': 0.05})[
        'state'] == 'REDUCED'
    assert attenuation(prior, {'fresh': True, 'latest_linear_mps': 0.10})[
        'state'] == 'PRESERVED'


def test_motion_segmentation_distinguishes_translation_rotation_and_stillness():
    stationary = motion_state([
        {'x_m': 0.0, 'y_m': 0.0, 'yaw_rad': 0.0},
        {'x_m': 0.005, 'y_m': 0.0, 'yaw_rad': 0.01},
    ])
    rotating = motion_state([
        {'x_m': 0.0, 'y_m': 0.0, 'yaw_rad': 0.0},
        {'x_m': 0.005, 'y_m': 0.0, 'yaw_rad': 0.10},
    ])
    translating = motion_state([
        {'x_m': 0.0, 'y_m': 0.0, 'yaw_rad': 0.0},
        {'x_m': 0.011, 'y_m': 0.0, 'yaw_rad': 0.0},
    ])
    assert stationary['state'] == 'FULLY_STATIONARY'
    assert rotating['state'] == 'ROTATING_IN_PLACE'
    assert translating['state'] == 'TRANSLATING'


def test_stationary_cause_uses_pipeline_evidence_in_priority_order():
    base = {'active_goal': True, 'frontier_phase': False,
            'planner_active': False, 'handoff_active': False,
            'recovery_active': False, 'start_not_traversable': False,
            'diagnostic_timeout': False, 'goal_transition': False,
            'collision_action': None, 'final_linear_nonzero': False}
    assert stationary_cause({**base, 'dwb_kind': 'ZERO',
                             'smoother_attenuation': 'PRESERVED',
                             'collision_attenuation': 'PRESERVED'}) == \
        'ACTIVE_GOAL_DWB_ZERO'
    assert stationary_cause({**base, 'dwb_kind': 'LINEAR',
                             'smoother_attenuation': 'SUPPRESSED',
                             'collision_attenuation': 'PRESERVED'}) == \
        'SMOOTHER_SUPPRESSED_OR_REDUCED'
    assert stationary_cause({**base, 'dwb_kind': 'LINEAR',
                             'smoother_attenuation': 'PRESERVED',
                             'collision_attenuation': 'SUPPRESSED'}) == \
        'COLLISION_MONITOR_STOPPED'
    assert stationary_cause({**base, 'dwb_kind': 'LINEAR',
                             'smoother_attenuation': 'PRESERVED',
                             'collision_attenuation': 'PRESERVED',
                             'final_linear_nonzero': True}) == \
        'FINAL_COMMAND_NONZERO_BUT_NO_ODOM'


def test_stationary_cause_distinguishes_no_goal_from_frontier_wait():
    base = {'active_goal': False, 'planner_active': False,
            'handoff_active': False, 'recovery_active': False,
            'start_not_traversable': False, 'diagnostic_timeout': False,
            'goal_transition': False, 'dwb_kind': 'MISSING_OR_STALE',
            'smoother_attenuation': 'MISSING', 'collision_attenuation': 'MISSING',
            'collision_action': None, 'final_linear_nonzero': False}
    assert stationary_cause({**base, 'frontier_phase': False}) == 'NO_ACTIVE_GOAL'
    assert stationary_cause({**base, 'frontier_phase': True}) == \
        'WAITING_FOR_FRONTIER_CANDIDATE'


def test_no_active_goal_and_scheduled_frontier_idle_beat_retained_selector_state():
    base = {'active_goal': False, 'planner_active': False,
            'handoff_active': True, 'recovery_active': False,
            'start_not_traversable': False, 'diagnostic_timeout': False,
            'goal_transition': False, 'dwb_kind': 'MISSING_OR_STALE',
            'smoother_attenuation': 'MISSING', 'collision_attenuation': 'MISSING',
            'collision_action': None, 'final_linear_nonzero': False}
    assert stationary_cause({**base, 'intentional_frontier_idle': False}) == \
        'GOAL_PRECHECK_OR_HANDOFF'
    assert stationary_cause({**base, 'handoff_active': False,
                             'intentional_frontier_idle': False}) == \
        'NO_ACTIVE_GOAL'
    assert stationary_cause({**base, 'intentional_frontier_idle': True}) == \
        'WAITING_FOR_FRONTIER_CANDIDATE'


def test_active_goal_dwb_evidence_beats_retained_handoff_state():
    context = {'active_goal': True, 'planner_active': False,
               'handoff_active': True, 'recovery_active': False,
               'start_not_traversable': False, 'diagnostic_timeout': False,
               'goal_transition': False, 'dwb_kind': 'ZERO',
               'smoother_attenuation': 'NO_PRECEDING_LINEAR_COMMAND',
               'collision_attenuation': 'NO_PRECEDING_LINEAR_COMMAND',
               'collision_action': None, 'final_linear_nonzero': False}
    assert stationary_cause(context) == 'ACTIVE_GOAL_DWB_ZERO'


def test_dwb_stall_detector_is_thresholded_and_emits_one_event_per_episode():
    detector = DwbStallDetector()
    goal = ('robot1', 'frontier_robot1')
    assert detector.observe(10.0, goal, 'ANGULAR_ONLY') == []
    assert detector.observe(11.9, goal, 'ANGULAR_ONLY') == []
    assert detector.observe(12.0, goal, 'ANGULAR_ONLY') == [
        {'trigger': 'ANGULAR_ONLY_CONTINUOUS', 'duration_s': 2.0}]
    assert detector.observe(13.0, goal, 'ANGULAR_ONLY') == []
    assert detector.observe(13.1, goal, 'ZERO') == []
    assert detector.observe(14.1, goal, 'ZERO') == [
        {'trigger': 'ZERO_CONTINUOUS', 'duration_s': 1.0}]


def test_dwb_stall_detector_captures_transition_from_forward_command():
    detector = DwbStallDetector()
    goal = ('robot2', 'manual_medium')
    assert detector.observe(1.0, goal, 'LINEAR') == []
    assert detector.observe(1.1, goal, 'ZERO') == [
        {'trigger': 'TRANSITION_FROM_FORWARD_TO_ZERO', 'duration_s': 0.0}]
