import importlib.util
from pathlib import Path


MODULE_PATH = Path(__file__).parents[1] / 'tools' / 'analyze_dwb_critic_counterfactual.py'
SPEC = importlib.util.spec_from_file_location('dwb_counterfactual', MODULE_PATH)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


def trajectory(total, align, path, goal, vx):
    return {
        'total_score': total,
        'critic_contributions': {
            'PathAlign': align, 'PathDist': path, 'GoalDist': goal,
        },
        'velocity': {'linear_x_mps': vx, 'angular_z_radps': 0.0},
    }


def test_rescore_keeps_baseline_and_scales_only_requested_critics():
    record = trajectory(7.28, 0.32, 0.72, 6.24, 0.026)
    assert MODULE.rescored_total(record, MODULE.BASELINE) == 7.28
    assert MODULE.rescored_total(record, {
        'path_align': 4.0, 'path_dist': 12.0, 'goal_dist': 24.0,
    }) == 6.76


def test_retained_choice_changes_only_when_forward_beats_selected():
    selected = trajectory(6.48, 0.0, 0.0, 6.48, 0.0)
    forward = trajectory(7.28, 0.32, 0.72, 6.24, 0.026)
    choice, selected_score, forward_score = MODULE.choose_retained_trajectory(
        selected, forward, {'path_align': 4.0, 'path_dist': 12.0, 'goal_dist': 24.0})
    assert choice == 'selected'
    assert selected_score == 6.48
    assert forward_score == 6.76
    choice, _, forward_score = MODULE.choose_retained_trajectory(
        selected, forward, {'path_align': 0.0, 'path_dist': 4.0, 'goal_dist': 24.0})
    assert choice == 'forward'
    assert forward_score < 6.48


def test_ties_preserve_recorded_dwb_selection():
    selected = trajectory(1.0, 0.0, 0.0, 1.0, 0.0)
    forward = trajectory(1.0, 0.0, 0.0, 1.0, 0.026)
    choice, _, _ = MODULE.choose_retained_trajectory(selected, forward, MODULE.BASELINE)
    assert choice == 'selected'


def test_candidate_matrix_contains_individual_and_limited_combined_changes():
    configurations = {name for name, _ in MODULE.scale_candidates()}
    assert 'path_align_4' in configurations
    assert 'path_dist_12' in configurations
    assert 'goal_dist_48' in configurations
    assert 'combined_pa4_pd12_gd36' in configurations


def test_score_fingerprint_collapses_repeated_identical_score_contexts():
    selected = trajectory(6.48, 0.0, 0.0, 6.48, 0.0)
    forward = trajectory(7.28, 0.32, 0.72, 6.24, 0.026)
    first = {'selected': selected, 'forward': forward}
    second = {'selected': dict(selected), 'forward': dict(forward)}
    assert MODULE.score_fingerprint(first) == MODULE.score_fingerprint(second)


def test_full_winner_rescores_all_valid_trajectories_and_reports_ties():
    first = trajectory(7.0, 0.0, 0.0, 7.0, 0.0)
    first.update({'trajectory_index': 0, 'valid': True, 'selected': True})
    forward = trajectory(7.0, 0.0, 0.1, 6.0, 0.026)
    forward.update({'trajectory_index': 1, 'valid': True, 'selected': False})
    another = trajectory(7.5, 0.0, 0.0, 7.5, 0.052)
    another.update({'trajectory_index': 2, 'valid': True, 'selected': False})
    winners, total, margin = MODULE._winner(
        [first, forward, another], {'path_align': 8.0, 'path_dist': 0.0, 'goal_dist': 24.0})
    assert [item['trajectory_index'] for item in winners] == [1]
    assert total < 7.0
    assert margin is not None


def test_full_scale_matrix_matches_requested_baseline_and_path_pairs():
    configurations = {name for name, _ in MODULE.full_scale_candidates()}
    assert 'pa8_pd24_gd24' in configurations
    assert 'pa1_pd6_gd24' in configurations
