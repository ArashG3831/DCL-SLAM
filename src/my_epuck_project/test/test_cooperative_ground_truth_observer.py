import io

import pytest

from my_epuck_project.cooperative_ground_truth_observer import (
    _BufferedCsvSink,
    _effective_step_period_ms,
    _enable_contact_tracking,
)


def test_buffered_csv_sink_preserves_rows_and_bounds_flush_work():
    stream = io.StringIO()
    sink = _BufferedCsvSink(stream, ['value'], max_buffer_bytes=8)
    sink.writerow({'value': 'first'})
    sink.writerow({'value': 'second'})
    sink.flush()
    assert stream.getvalue().splitlines() == ['value', 'first', 'second']
    assert sink.flushes >= 1


def test_runner_canonical_ground_truth_sampling_default_is_20ms():
    from my_epuck_project.cooperative_trial_fast import parser

    args = parser().parse_args([])
    assert args.forensic_ground_truth_sample_period_s == pytest.approx(0.02)
from my_epuck_project.cooperative_experiment_logger import (
    _compose_planar,
    _invert_planar,
    _wrap_planar_yaw,
    CooperativeExperimentLogger,
)


def test_forensic_sampling_batches_basic_steps():
    assert _effective_step_period_ms(4, 0.10) == 100
    assert _effective_step_period_ms(20, 0.10) == 100


def test_forensic_sampling_never_uses_non_aligned_step():
    assert _effective_step_period_ms(4, 0.021) == 24
    assert _effective_step_period_ms(20, 0.001) == 20


def test_contact_capture_preserves_requested_faster_period():
    assert _effective_step_period_ms(4, 0.10, 20) == 20
    assert _effective_step_period_ms(20, 0.10, 20) == 20


def test_contact_tracking_is_enabled_for_both_active_robot_ids():
    class FakeNode:
        def __init__(self):
            self.calls = []

        def enableContactPointsTracking(self, period_ms, include_descendants):
            self.calls.append((period_ms, include_descendants))

    robots = {'robot1': FakeNode(), 'robot2': FakeNode()}
    _enable_contact_tracking(robots, 20)
    assert robots['robot1'].calls == [(20, True)]
    assert robots['robot2'].calls == [(20, True)]


def test_physical_map_frame_formula_uses_map_frames_not_robot_separation():
    # Synthetic planar transforms use T_A_B: p_A = T_A_B p_B.
    world_base1 = (4.0, -1.0, 0.3)
    world_base2 = (-2.0, 3.0, -0.4)
    map1_base1 = (0.8, 0.2, -0.1)
    map2_base2 = (-0.4, 0.6, 0.25)
    world_map1 = _compose_planar(world_base1, _invert_planar(map1_base1))
    world_map2 = _compose_planar(world_base2, _invert_planar(map2_base2))
    expected = _compose_planar(_invert_planar(world_map2), world_map1)
    # Applying the derived map2<-map1 transform must map a point expressed in
    # map1 back to the same world point expressed through map2.
    map1_point = (1.2, -0.7, 0.0)
    world_point = _compose_planar(world_map1, map1_point)
    recovered = _compose_planar(
        world_map2, _compose_planar(expected, map1_point))
    assert recovered[:2] == pytest.approx(world_point[:2])
    assert recovered[2] == pytest.approx(world_point[2])


def test_physical_map_frame_yaw_wrap_is_shortest_difference():
    assert _wrap_planar_yaw(3.2) == pytest.approx(3.2 - 2.0 * 3.141592653589793)


def test_physical_map_frame_reconstructs_local_chain():
    def obs(transform, age=0.0):
        return {
            'available': True,
            'age_s': age,
            'interpolation_used': False,
            'interpolation_age_s': age,
            'translation': {'x': transform[0], 'y': transform[1], 'z': 0.0},
            'quaternion': {'x': 0.0, 'y': 0.0,
                           'z': __import__('math').sin(transform[2] / 2.0),
                           'w': __import__('math').cos(transform[2] / 2.0)},
        }
    map_to_base, metadata = CooperativeExperimentLogger._map_base_from_sync_row({
        'map_to_base': {'available': False},
        'map_to_odom': obs((1.0, 2.0, 0.2), age=1.7),
        'odom_to_base': obs((0.5, -0.1, -0.1), age=0.02),
    })
    assert map_to_base == pytest.approx(_compose_planar(
        (1.0, 2.0, 0.2), (0.5, -0.1, -0.1)))
    assert metadata['map_to_odom_age_s'] == pytest.approx(1.7)
    assert metadata['odom_to_base_age_s'] == pytest.approx(0.02)
