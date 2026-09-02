#!/usr/bin/env python3
"""No-motion deterministic two-peer assignment integration test."""

from launch import LaunchDescription

from launch_ros.actions import Node


def _source(robot):
    return Node(
        package='my_epuck_project', executable='fixed_task_snapshot_source',
        name='fixed_task_snapshot_source', namespace=robot, output='screen',
        parameters=[{
            'robot_id': robot, 'scenario': 'alternate_hallway',
            'validity_s': 30.0, 'use_sim_time': False,
        }],
    )


def _peer(robot, origin_x):
    return Node(
        package='my_epuck_project', executable='distributed_frontier_assignment',
        name='distributed_frontier_assignment', namespace=robot, output='screen',
        parameters=[{
            'robot_id': robot, 'dispatch_enabled': False,
            'synthetic_bids': True, 'synthetic_origin_x': origin_x,
            'synthetic_origin_y': 0.0, 'maximum_union_tasks': 10,
            'maximum_path_queries': 8, 'bid_validity_s': 30.0,
            'decision_validity_s': 30.0, 'peer_timeout_s': 30.0,
            # Historical fixed-task scenario: explicitly preserve the old
            # weighted result for regression/reproducibility only.
            'assignment_strategy': 'frontier_cost_only',
            'traffic_scheduler_enabled': False,
            'use_sim_time': False,
        }],
    )


def generate_launch_description():
    """Start two fixed sources and two equal no-motion peers."""
    return LaunchDescription([
        _source('robot1'), _source('robot2'),
        _peer('robot1', 0.0), _peer('robot2', 0.1),
    ])
