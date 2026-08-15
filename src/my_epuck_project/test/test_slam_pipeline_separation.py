"""Static tests proving the SLAM branch remains separate from safety scans."""

from pathlib import Path


ROOT = Path(__file__).parents[1]
LAUNCH = ROOT / 'launch' / 'two_robots_teammate_filtered_dual_slam_launch.py'
FILTER = ROOT / 'my_epuck_project' / 'teammate_scan_filter.py'


def test_teammate_masking_occurs_before_natural_no_return_completion():
    source = FILTER.read_text()
    pipeline = source[source.index('def prepare_slam_scan'):]
    masking = pipeline.index('mask_teammate_returns(')
    completion = pipeline.index('complete_natural_no_returns(')
    assert masking < completion
    assert 'output.ranges[index] = math.nan' in source


def test_nav2_and_collision_monitor_keep_the_fixed_scan():
    for robot in ('robot1', 'robot2'):
        params = (ROOT / 'resource' /
                  f'nav2_{robot}_shared_map.yaml').read_text()
        assert f'/{robot}/scan_d500_fixed' in params
        assert f'/{robot}/scan_d500_slam' not in params
        collision = params[params.index('collision_monitor:'):]
        assert f'/{robot}/scan_d500_fixed' in collision


def test_slam_toolbox_consumes_only_the_teammate_filtered_branch():
    launch = LAUNCH.read_text()
    assert "'input_topic': f'/{robot}/scan_d500_fixed'" in launch
    assert "'output_topic': f'/{robot}/scan_d500_slam'" in launch
    for robot in ('robot1', 'robot2'):
        params = (ROOT / 'resource' /
                  f'slam_toolbox_{robot}_teammate_filtered.yaml').read_text()
        assert f'scan_topic: /{robot}/scan_d500_slam' in params
        assert 'max_laser_range: 11.98' in params


def test_removed_pose_state_and_geometry_parameters_are_absent():
    launch = LAUNCH.read_text()
    removed = (
        'peer_shape_type', 'peer_shape_dimensions', 'static_safety_margin',
        'dynamic_motion_margin', 'range_matching_tolerance',
        'queue_overflow_policy', 'allow_latest_transform_fallback',
        'shared_tf_confirmation_scans', 'shared_tf_position_tolerance',
        'shared_tf_yaw_tolerance', 'pose_loss_grace_s',
        'output_stall_grace_s', 'force_preferred_pose_loss',
        'physical_free_space_completion', "'mode': 'simulation'",
    )
    assert not [name for name in removed if name in launch]
    assert "'peer_radius_m': 0.035" in launch
    assert "'range_tolerance_m': 0.005" in launch


def test_removed_degraded_and_natural_index_apis_are_absent():
    source = FILTER.read_text()
    assert 'natural_no_return_indices' not in source
    assert 'degraded_unmasked' not in source
    assert 'publish_degraded' not in source
    assert 'shared_peer_pose' not in source


def test_stack_launch_reuses_authoritative_dual_slam_launch():
    stack = (ROOT / 'launch' /
             'two_robots_teammate_filtered_stack_launch.py').read_text()
    assert 'two_robots_teammate_filtered_dual_slam_launch.py' in stack


def test_simulation_profiles_use_the_derived_cap():
    resources = ROOT / 'resource'
    for path in resources.glob('slam_toolbox_d500*.yaml'):
        text = path.read_text()
        assert 'max_laser_range: 1.5' not in text
        assert 'max_laser_range: 11.98' in text
