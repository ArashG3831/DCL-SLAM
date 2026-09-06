"""Focused parity tests for the first legacy-observer salvage primitive."""

import csv

import pytest

from my_epuck_project.experiment_metrics import (
    LiveDistanceAccumulator,
    LocalTrajectory,
    replay_local_trajectory_csv,
)
from my_epuck_project.cooperative_experiment_logger import (
    CooperativeExperimentLogger,
)


def _write_odom(path, robot, points):
    with path.open('w', newline='', encoding='utf-8') as stream:
        writer = csv.DictWriter(stream, fieldnames=(
            'robot_id', 'received_ros_time_s', 'pose_x', 'pose_y'))
        writer.writeheader()
        for index, (x, y) in enumerate(points):
            writer.writerow({
                'robot_id': robot,
                'received_ros_time_s': float(index),
                'pose_x': x,
                'pose_y': y,
            })


def test_deferred_local_trajectory_reuses_live_metric_semantics(tmp_path):
    points = [(0.0, 0.0), (0.10, 0.0), (0.10, 0.10), (0.0, 0.0)]
    live = LocalTrajectory(bin_size=0.05, exclusion_radius=0.0)
    for x, y in points:
        live.add('robot1', x, y)

    path = tmp_path / 'robot1_odom.csv'
    _write_odom(path, 'robot1', points)
    replayed = replay_local_trajectory_csv(
        path, 'robot1', bin_size=0.05, exclusion_radius=0.0)

    assert replayed.summary() == live.summary()


def test_live_distance_accumulator_preserves_legacy_distance_scalar():
    points = [(0.0, 0.0), (0.10, 0.0), (0.10, 0.10), (0.0, 0.0)]
    full = LocalTrajectory(bin_size=0.05, exclusion_radius=0.0)
    scalar = LiveDistanceAccumulator()
    for x, y in points:
        full.add('robot1', x, y)
        scalar.add('robot1', x, y)

    assert scalar.total_distance == full.total_distance


def test_deferred_local_trajectory_fails_closed_for_missing_evidence(tmp_path):
    with pytest.raises(FileNotFoundError):
        replay_local_trajectory_csv(tmp_path / 'missing.csv', 'robot1')


def test_deferred_local_trajectory_fails_closed_for_malformed_rows(tmp_path):
    path = tmp_path / 'robot1_odom.csv'
    _write_odom(path, 'robot1', [(0.0, 0.0)])
    with path.open('a', encoding='utf-8') as stream:
        stream.write('robot1,1,nan,0.0\n')

    with pytest.raises(ValueError):
        replay_local_trajectory_csv(path, 'robot1')


def test_logger_finalization_helper_reports_live_deferred_parity(tmp_path):
    points = [(0.0, 0.0), (0.2, 0.0), (0.2, 0.2)]
    forensic = tmp_path / 'forensic'
    forensic.mkdir()
    path = forensic / 'robot1_odom.csv'
    _write_odom(path, 'robot1', points)

    node = object.__new__(CooperativeExperimentLogger)
    node.robots = ['robot1']
    node.directory = tmp_path
    node.forensic = object()
    node.p = {
        'trajectory_bin_size_m': 0.05,
        'initial_overlap_exclusion_radius_m': 0.0,
    }
    node.local_trajectory = LocalTrajectory(bin_size=0.05,
                                            exclusion_radius=0.0)
    for x, y in points:
        node.local_trajectory.add('robot1', x, y)

    result = node._replay_local_trajectory_for_parity()

    assert result['status'] == 'PARITY_PASS'
    assert result['equal'] is True
    assert result['live'] == result['deferred']
    assert node._deferred_local_trajectory.summary() == result['deferred']
