"""Launch inventory and authority-boundary regression tests."""

from pathlib import Path

from my_epuck_project.distributed_frontier_assignment import dispatch_delay_elapsed


LAUNCH = Path(__file__).parents[1] / 'launch'


def test_required_launch_inventory_exists():
    """Expose every required mapping, assignment, final, and baseline launch."""
    expected = {
        'two_robots_decentralized_mapping_only_launch.py',
        'two_robots_mapping_plus_nav2_launch.py',
        'two_robots_assignment_dry_run_launch.py',
        'two_robots_decentralized_exploration_launch.py',
        'single_robot_frontier_baseline_launch.py',
        'two_robots_legacy_claim_baseline_launch.py',
        'fixed_task_distributed_assignment_launch.py',
    }
    assert expected <= {path.name for path in LAUNCH.glob('*.py')}


def test_common_launch_has_two_equal_peers_and_no_central_allocator():
    """The final graph instantiates identical software once per robot."""
    text = (LAUNCH / 'two_robots_distributed_assignment_launch.py').read_text()
    assert "_assignment_peer('robot1')" in text
    assert "_assignment_peer('robot2')" in text
    assert 'central_allocator' not in text
    assert 'hungarian' not in text.lower()
    assert 'cmd_vel' not in text


def test_full_launch_enables_only_the_common_local_dispatch_boundary():
    """The stable final launch switches the replicated graph to dispatch mode."""
    text = (LAUNCH / 'two_robots_decentralized_exploration_launch.py').read_text()
    assert "DeclareLaunchArgument('dispatch_enabled', default_value='true'" in text
    assert 'cooperative_frontier_coordinator' not in text
    assignment_text = (LAUNCH / 'two_robots_distributed_assignment_launch.py').read_text()
    assert "'maximum_tasks_per_source': 10000" in assignment_text
    assert "'maximum_union_tasks': 10000" in assignment_text


def test_frontier_query_budget_matches_candidate_selection_bound():
    """Keep the production C frontier drain aligned with its selection cap."""
    for name in (
            'two_robots_frontier_candidates_launch.py',
            'two_robots_decentralized_exploration_launch.py'):
        text = (LAUNCH / name).read_text()
        assert "'maximum_candidates_before_path_check': 10000" in text
        assert "'maximum_path_queries_per_cycle': 10000" in text


def test_legacy_baseline_remains_explicitly_reachable():
    """Preserve the old claim-only launch for thesis comparison."""
    text = (LAUNCH / 'two_robots_legacy_claim_baseline_launch.py').read_text()
    assert 'two_robots_observed_continuous_exploration_launch.py' in text


def test_prehandoff_dispatch_hold_is_explicit_and_forwarded():
    """Unknown-pose evidence may hold local motion without changing defaults."""
    launch = (LAUNCH / 'two_robots_decentralized_exploration_launch.py').read_text()
    allocator = (LAUNCH.parent / 'my_epuck_project' /
                 'distributed_frontier_assignment.py').read_text()
    assert "DeclareLaunchArgument('prehandoff_dispatch_delay_s'" in launch
    assert "'prehandoff_dispatch_delay_s': LaunchConfiguration(" in launch
    assert 'dispatch_delay_elapsed' in allocator
    assert not dispatch_delay_elapsed(10.0, 15.0, 10.0)
    assert dispatch_delay_elapsed(10.0, 20.0, 10.0)
    assert dispatch_delay_elapsed(10.0, 10.0, 0.0)
