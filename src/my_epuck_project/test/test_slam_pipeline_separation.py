"""Static tests proving the SLAM branch is separate from safety scans."""

from pathlib import Path


ROOT = Path(__file__).parents[1]


def test_simulation_slam_branch_is_after_masking_and_nav2_stays_fixed():
    """The filtered SLAM topic is downstream while Nav2 stays on fixed scan."""
    launch = (ROOT / 'launch' /
              'two_robots_teammate_filtered_dual_slam_launch.py').read_text()
    nav = (ROOT / 'launch' /
           'two_robots_teammate_filtered_stack_launch.py').read_text()
    assert 'input_topic' in launch and 'scan_d500_fixed' in launch
    assert 'output_topic' in launch and 'scan_d500_slam' in launch
    assert 'simulation_free_space_completion' in launch
    assert 'physical_free_space_completion' in launch
    params = (ROOT / 'resource' /
              'nav2_d500_frontier_hires_params.yaml').read_text()
    assert "scan_d500_fixed" in params
    assert "scan_d500_slam" not in nav + params


def test_simulation_profiles_use_the_derived_cap():
    """D500 simulation profiles do not retain the stale 1.5 m threshold."""
    resources = ROOT / 'resource'
    for path in resources.glob('slam_toolbox_d500*.yaml'):
        text = path.read_text()
        assert 'max_laser_range: 1.5' not in text
        assert 'max_laser_range: 11.98' in text
    for name in ('slam_toolbox_robot1_teammate_filtered.yaml',
                 'slam_toolbox_robot2_teammate_filtered.yaml',
                 'slam_toolbox_robot1_two_robots.yaml',
                 'slam_toolbox_robot2_two_robots.yaml'):
        assert 'max_laser_range: 11.98' in (resources / name).read_text()


def test_filter_contains_no_unsafe_passthrough_or_mask_ordering():
    """Completion precedes the existing conservative NaN teammate mask."""
    source = (ROOT / 'my_epuck_project' /
              'teammate_scan_filter.py').read_text()
    completion = source.index('complete_natural_no_returns(')
    masking = source.index('filtered_scan(', completion)
    assert completion < masking
    assert 'output.ranges[index] = math.nan' in source
