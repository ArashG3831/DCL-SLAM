import json
from pathlib import Path

import pytest
from launch import LaunchContext
from launch_ros.parameter_descriptions import ParameterValue

from my_epuck_project.cooperative_regression import parser, seed_schedule


def test_campaign_condition_and_frozen_seed_schedule():
    args = parser().parse_args([
        '--trials', '5', '--experiment-condition', 'D',
        '--webots-random-seed', '1001', '--webots-random-seed', '1002',
        '--webots-random-seed', '1003', '--webots-random-seed', '1004',
        '--webots-random-seed', '1005',
    ])
    assert args.experiment_condition == 'D'
    assert seed_schedule(args) == [1001, 1002, 1003, 1004, 1005]


def test_one_seed_is_repeated_for_requested_trials():
    args = parser().parse_args([
        '--trials', '5', '--webots-random-seed', '1001'])
    assert seed_schedule(args) == [1001] * 5


def test_negative_or_mismatched_seed_schedule_is_rejected():
    args = parser().parse_args([
        '--trials', '5', '--webots-random-seed', '-1'])
    with pytest.raises(SystemExit):
        seed_schedule(args)
    args = parser().parse_args([
        '--trials', '5', '--webots-random-seed', '1001',
        '--webots-random-seed', '1002'])
    with pytest.raises(SystemExit):
        seed_schedule(args)


def test_seed_provenance_is_forced_to_ros_string_on_all_logger_paths():
    root = Path(__file__).resolve().parents[1] / 'launch'
    for name in (
            'single_robot_thesis_baseline_launch.py',
            'two_robots_independent_exploration_launch.py',
            'two_robots_decentralized_exploration_launch.py'):
        source = (root / name).read_text(encoding='utf-8')
        assert 'ParameterValue' in source
        assert 'value_type=str' in source

    raw = '{"requested_seed":1001,"effective_seed":1001}'
    value = ParameterValue(raw, value_type=str).evaluate(LaunchContext())
    assert isinstance(value, str)
    assert json.loads(value) == {
        'requested_seed': 1001, 'effective_seed': 1001}
