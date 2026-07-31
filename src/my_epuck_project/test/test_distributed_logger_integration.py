"""Static coverage for distributed protocol report integration."""

from pathlib import Path


LOGGER = Path(__file__).parents[1] / 'my_epuck_project' / (
    'cooperative_experiment_logger.py'
)


def test_logger_observes_every_distributed_protocol_stream():
    """The existing JSONL workflow receives task, bid, decision, status, and failure data."""
    source = LOGGER.read_text(encoding='utf-8')
    for suffix in (
        'task_snapshot', 'task_bids', 'pair_decision', 'distributed_status',
        'distributed_event', 'exploration_failure',
    ):
        assert "f'/{r}/%s'" % suffix in source
    for handler in (
        'distributed_snapshot', 'distributed_bids', 'distributed_decision',
        'distributed_status', 'distributed_event', 'distributed_failure',
    ):
        assert 'def %s(' % handler in source


def test_logger_records_pair_components_and_measured_motion():
    """Audit fields include every pair penalty and actual travelled distance."""
    source = LOGGER.read_text(encoding='utf-8')
    for field in (
        'team_visible_gain', 'combined_path_cost', 'nearby_goal_penalty',
        'route_overlap_penalty', 'hard_failure_penalty',
        'sensing_overlap_penalty', 'workload_imbalance_penalty',
        'travelled_distance_m', 'navigation_duration_s', 'recoveries',
    ):
        assert field in source
