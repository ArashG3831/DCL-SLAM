"""Launch inventory and authority-boundary regression tests."""

from pathlib import Path


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
    assert "'dispatch_enabled': 'true'" in text
    assert 'cooperative_frontier_coordinator' not in text


def test_legacy_baseline_remains_explicitly_reachable():
    """Preserve the old claim-only launch for thesis comparison."""
    text = (LAUNCH / 'two_robots_legacy_claim_baseline_launch.py').read_text()
    assert 'two_robots_observed_continuous_exploration_launch.py' in text
