from my_epuck_project.pipeline_telemetry import (
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
