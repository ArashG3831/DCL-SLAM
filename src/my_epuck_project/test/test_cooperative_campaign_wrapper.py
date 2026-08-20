from pathlib import Path

import pytest

from my_epuck_project.cooperative_campaign_wrapper import (
    PROFILE, main, parser, runner_arguments, validate)


def config_for(tmp_path):
    root = Path(__file__).resolve().parents[3]
    world = root / 'src' / 'my_epuck_project' / 'worlds' / (
        'epuck_d500_two_world_unknown_pose_16m_dynamic_low_slip_4ms_finite.wbt')
    return parser().parse_args([
        '--workspace', str(root),
        '--world-path', str(world),
        '--results-directory', str(tmp_path / 'results'),
        '--observer-url',
        'tcp://127.0.0.1:23000/ForensicGroundTruthSupervisor',
    ])


def test_dry_run_constructs_safe_final_argv_without_launching(tmp_path, capsys):
    config = config_for(tmp_path)
    config.dry_run = True
    assert main([
        '--workspace', str(config.workspace),
        '--world-path', str(config.world_path),
        '--results-directory', str(config.results_directory),
        '--observer-url', config.observer_url,
        '--dry-run',
    ]) == 0
    output = capsys.readouterr().out
    assert 'CAMPAIGN_DRY_RUN_OK' in output
    assert PROFILE in output
    assert '\\' not in output
    assert 'epuck_d500_two_world_unknown_pose_16m_dynamic_low_slip_4ms_finite.wbt' in output


def test_runner_arguments_have_no_shell_continuation_tokens(tmp_path):
    config = config_for(tmp_path)
    args = runner_arguments(config)
    assert '\\' not in args
    assert not any('\\' in token for token in args)
    assert args[args.index('--world-profile') + 1] == PROFILE
    assert str(config.world_path.resolve()) in args
    assert '--enable-observer' in args
    assert '--enable-forensic-capture' in args


def test_profile_world_and_nested_resolvers_are_consistent(tmp_path):
    metadata = validate(config_for(tmp_path))
    assert metadata['profile'] == PROFILE
    assert metadata['physics_profile'] == 'dynamic_low_slip_4ms_finite'
    assert metadata['observer_url'].endswith(
        '/ForensicGroundTruthSupervisor')
    assert metadata['runner_argv'].count('\\') == 0


def test_conflicting_world_fails_before_launch(tmp_path):
    config = config_for(tmp_path)
    config.world_path = config.workspace / 'src' / 'my_epuck_project' / 'worlds' / (
        'epuck_d500_two_world_large.wbt')
    with pytest.raises(ValueError, match='world profile/path mismatch'):
        validate(config)
