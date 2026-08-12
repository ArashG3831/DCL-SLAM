from my_epuck_project.dwb_zero_event_capture import ZeroEventCapturePolicy


def selected(vx=0.0, vy=0.0, wz=0.0):
    return {'velocity': {'linear_x_mps': vx, 'linear_y_mps': vy,
                         'angular_z_radps': wz}}


def test_requires_active_goal_and_fresh_evaluation_and_excludes_idle():
    p = ZeroEventCapturePolicy()
    assert p.observe(0.0, 'robot2', ('r', 1), 'manual', False, False, True,
                     selected()) == 'INACTIVE'
    assert p.observe(0.0, 'robot2', ('r', 1), 'manual', True, True, True,
                     selected()) == 'INACTIVE'
    assert p.observe(0.0, 'robot2', ('r', 1), 'manual', True, False, False,
                     selected()) == 'INACTIVE'


def test_numerical_zero_tolerance_and_one_second_trigger():
    p = ZeroEventCapturePolicy(zero_tolerance=1e-5)
    key = ('robot2', 'manual_medium', 10.0)
    assert p.observe(10.0, 'robot2', key, 'manual', True, False, True,
                     selected(1e-6, 0.0, -1e-6)) == 'TRACKING'
    assert p.observe(10.99, 'robot2', key, 'manual', True, False, True,
                     selected(0.0, 0.0, 0.0)) == 'TRACKING'
    assert p.observe(11.01, 'robot2', key, 'manual', True, False, True,
                     selected()) == 'TRIGGERED'
    assert p.trigger_robot == 'robot2'
    assert p.zero_started_sim_s['robot2'] == 10.0


def test_first_event_only_and_bounded_window():
    p = ZeroEventCapturePolicy(post_zero_s=8.0, hard_window_s=30.0)
    key = ('robot2', 'manual_medium', 10.0)
    for t in (10.0, 11.1):
        p.observe(t, 'robot2', key, 'manual', True, False, True, selected())
    assert p.triggered
    assert p.observe(12.0, 'robot1', ('r1', 1), 'manual', True, False, True,
                      selected()) == 'INACTIVE'
    assert p.should_close(19.2, key, selected(0.026, 0.0, 0.0), True) == 'END'
    assert p.closed


def test_goal_change_and_recovery_close_capture():
    p = ZeroEventCapturePolicy()
    key = ('robot1', 'frontier', 3.0)
    p.observe(0.0, 'robot1', key, 'frontier', True, False, True, selected())
    assert p.observe(1.1, 'robot1', key, 'frontier', True, False, True,
                     selected()) == 'TRIGGERED'
    assert p.should_close(1.2, key, selected(), True, recovery_count=1) == 'END'
