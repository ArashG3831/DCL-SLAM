"""Focused parity tests for the first legacy-observer salvage primitive."""

import csv

import pytest

from my_epuck_project.experiment_metrics import (
    LocalTrajectory,
    replay_local_trajectory_csv,
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
