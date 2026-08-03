from pathlib import Path
from types import SimpleNamespace

from my_epuck_project.nav2_frontier_diagnostic import (
    ARTIFACT_NAMES,
    boolean,
    deterministic_candidate_key,
    DiagnosticNode,
    _affinity_preexec,
    grid_cell,
    map_change_classification,
    region_fingerprint,
    candidate_physical_signature,
    launch_command,
    LAUNCH_FILE,
    phase_at,
    runner_parser,
    TIME_STATES,
)
from nav_msgs.msg import OccupancyGrid
import rclpy
from rclpy.parameter import Parameter


PROJECT = Path(__file__).resolve().parents[1]


def test_launch_chain_is_below_allocator_and_has_one_goal_sender():
    launch = (PROJECT / 'launch' / LAUNCH_FILE).read_text(encoding='utf-8')
    assert 'two_robots_frontier_candidates_launch.py' in launch
    assert "executable='nav2_frontier_diagnostic'" in launch
    assert 'two_robots_distributed_assignment_launch.py' not in launch
    assert 'two_robots_decentralized_exploration_launch.py' not in launch
    assert 'distributed_frontier_assignment' not in launch
    assert 'cooperative_experiment_logger' not in launch
    assert 'cooperative_trial_collector' not in launch


def test_diagnostic_does_not_import_allocator_or_pair_scoring():
    source = (PROJECT / 'my_epuck_project'
              / 'nav2_frontier_diagnostic.py').read_text(encoding='utf-8')
    assert 'from .distributed' not in source
    assert 'import distributed_frontier_assignment' not in source
    assert 'pair_utility' not in source
    assert "Clock, '/clock', self._on_clock" in source


def test_exact_four_phase_boundaries():
    assert phase_at(0.0) == 'FILTER_AND_MAPPING'
    assert phase_at(59.999) == 'FILTER_AND_MAPPING'
    assert phase_at(60.0) == 'NAV2_ONLY'
    assert phase_at(299.999) == 'NAV2_ONLY'
    assert phase_at(300.0) == 'FRONTIER_GENERATION_ONLY'
    assert phase_at(449.999) == 'FRONTIER_GENERATION_ONLY'
    assert phase_at(450.0) == 'FRONTIER_TO_NAV2'
    assert phase_at(600.0) == 'FRONTIER_TO_NAV2'


def test_short_phase_profile_reaches_frontiers_in_180_seconds():
    assert phase_at(0.0, profile='short') == 'FILTER_AND_MAPPING'
    assert phase_at(19.999, profile='short') == 'FILTER_AND_MAPPING'
    assert phase_at(20.0, profile='short') == 'NAV2_ONLY'
    assert phase_at(99.999, profile='short') == 'NAV2_ONLY'
    assert phase_at(100.0, profile='short') == 'FRONTIER_GENERATION_ONLY'
    assert phase_at(139.999, profile='short') == 'FRONTIER_GENERATION_ONLY'
    assert phase_at(140.0, profile='short') == 'FRONTIER_TO_NAV2'
    assert phase_at(180.0, profile='short') == 'FRONTIER_TO_NAV2'


def test_short_profile_does_not_schedule_medium_manual_goals(tmp_path):
    rclpy.init()
    node = DiagnosticNode(parameter_overrides=[
        Parameter('output_directory', value=str(tmp_path)),
        Parameter('mission_duration_s', value=180.0),
        Parameter('phase_profile', value='short'),
    ])
    try:
        assert node.manual_steps == [
            ('robot1', 'short', False), ('robot2', 'short', False)]
    finally:
        node.close_artifacts()
        node.destroy_node()
        rclpy.shutdown()


def test_node_constructs_with_clock_callback_and_streaming_artifacts(tmp_path):
    rclpy.init()
    node = DiagnosticNode(parameter_overrides=[
        Parameter('output_directory', value=str(tmp_path)),
        Parameter('mission_duration_s', value=1.0),
    ])
    try:
        assert node.get_name() == 'nav2_frontier_diagnostic'
        assert (tmp_path / 'diagnostic_events.jsonl').exists()
        assert (tmp_path / 'diagnostic_timeseries.csv').exists()
        node._event('TEST_INFO', severity='INFO')
        node._event('TEST_WARN', severity='WARN')
        node._event('TEST_ERROR', severity='ERROR')
        failed_cycle = SimpleNamespace(
            name='/robot2/frontier_candidate_generator',
            msg='CANDIDATE_METRICS queries=5 reachable=0 extraction_ms=10')
        for _ in range(3):
            node._rosout(failed_cycle)
        assert node.planner_failure_storm['robot2'] is True
    finally:
        node.close_artifacts()
        node.destroy_node()
        rclpy.shutdown()


def test_selector_uses_reachability_gain_length_then_id():
    def candidate(identifier, gain, length, reachable=True):
        return SimpleNamespace(
            REACHABLE=1,
            reachability_state=1 if reachable else 0,
            information_gain=gain,
            path_length_m=length,
            frontier_id=identifier,
        )

    items = [
        candidate(8, 4.0, 2.0), candidate(7, 5.0, 3.0),
        candidate(6, 5.0, 2.0), candidate(5, 5.0, 2.0),
        candidate(1, 100.0, 0.1, reachable=False),
    ]
    assert [item.frontier_id for item in sorted(
        items, key=deterministic_candidate_key)] == [5, 6, 7, 8, 1]


def test_grid_cell_rejects_outside_and_resolves_inside():
    grid = OccupancyGrid()
    grid.info.resolution = 0.5
    grid.info.width = 4
    grid.info.height = 3
    grid.info.origin.position.x = -1.0
    grid.info.origin.position.y = -0.5
    grid.info.origin.orientation.w = 1.0
    grid.data = list(range(12))
    assert grid_cell(grid, -0.75, -0.25) == (0, 0, 0)
    assert grid_cell(grid, 0.75, 0.75) == (11, 3, 2)
    assert grid_cell(grid, 1.1, 0.0) is None


def test_runner_supports_required_command_surface(tmp_path):
    args = runner_parser().parse_args([
        '--world-profile', 'large', '--execution-profile', 'visual',
        '--sensor-profile', 'full', '--time-mode', 'sim',
        '--fast-mode', 'true', '--rendering', 'true', '--rviz', 'true',
        '--mission-timeout', '600', '--startup-timeout', '300',
        '--emergency-wall-runtime', '1200', '--ros-domain-id', '232',
        '--webots-port', '23667',
    ])
    command = launch_command(args, Path('/tmp/large.wbt'), tmp_path)
    assert command[:4] == ['ros2', 'launch', 'my_epuck_project', LAUNCH_FILE]
    assert 'world_profile:=large' in command
    assert 'webots_mode:=fast' in command
    assert 'webots_gui:=true' in command
    assert 'sensor_profile:=full' in command
    assert 'mission_duration_s:=600.0' in command
    assert args.fusion_cpu_quota_percent == 30.0
    assert 'fusion_cpu_quota_percent:=30' in command
    assert args.cpu_core_limit == 4
    unthrottled = runner_parser().parse_args([
        '--fusion-cpu-quota-percent', '0'])
    assert unthrottled.fusion_cpu_quota_percent == 0.0
    assert boolean('false') is False


def test_headless_profile_defaults_rendering_and_rviz_off():
    args = runner_parser().parse_args(['--execution-profile', 'headless'])
    assert args.rendering is None
    assert args.rviz is None
    assert args.fast_mode is True
    assert args.time_mode == 'sim'


def test_runner_process_affinity_limit_is_optional_and_bounded():
    assert _affinity_preexec(0) is None
    hook = _affinity_preexec(1)
    assert hook is None or callable(hook)


def test_artifact_and_time_breakdown_contracts_are_complete():
    assert ARTIFACT_NAMES == {
        'launch.log', 'diagnostic_events.jsonl',
        'diagnostic_timeseries.csv', 'diagnostic_summary.json',
        'effective_command.txt', 'handoff_rejections.csv',
    }
    assert len(TIME_STATES) == 11
    assert 'active navigation with nonzero cmd_vel' in TIME_STATES
    assert 'active goal with zero cmd_vel' in TIME_STATES
    assert 'idle despite valid candidates' in TIME_STATES


def _handoff_grid():
    grid = OccupancyGrid()
    grid.info.resolution = 0.1
    grid.info.width = 30
    grid.info.height = 30
    grid.info.origin.orientation.w = 1.0
    grid.data = [0] * 900
    return grid


def _candidate(identifier=1):
    point = SimpleNamespace(x=1.0, y=1.0)
    pose = SimpleNamespace(pose=SimpleNamespace(position=point))
    return SimpleNamespace(
        frontier_id=identifier, centroid=SimpleNamespace(x=1.1, y=1.1),
        bounding_box_min=SimpleNamespace(x=1.0, y=1.0),
        bounding_box_max=SimpleNamespace(x=1.2, y=1.2),
        approach_pose=pose)


def test_handoff_revision_change_outside_path_is_allowed_by_region_check():
    grid = _handoff_grid()
    source = region_fingerprint(grid, [(1.0, 1.0)])
    grid.data[-1] = 100  # Remote map update, outside the corridor sample.
    assert map_change_classification(4, 5, source,
                                     region_fingerprint(grid, [(1.0, 1.0)])) == \
        'REVISION_CHANGED_BUT_PATH_REGION_UNCHANGED'


def test_handoff_obstacle_on_path_is_relevant_map_change():
    grid = _handoff_grid()
    source = region_fingerprint(grid, [(1.0, 1.0)])
    index, _, _ = grid_cell(grid, 1.0, 1.0)
    grid.data[index] = 100
    assert map_change_classification(4, 5, source,
                                     region_fingerprint(grid, [(1.0, 1.0)])) == \
        'RELEVANT_MAP_CHANGE'


def test_handoff_same_revision_is_not_stale():
    assert map_change_classification(4, 4, (0,), (100,)) == \
        'MAP_REVISION_UNCHANGED'


def test_handoff_physical_signature_survives_frontier_id_change():
    assert candidate_physical_signature(_candidate(1)) == \
        candidate_physical_signature(_candidate(99))


def test_handoff_physical_signature_changes_for_moved_goal():
    first, second = _candidate(1), _candidate(1)
    second.approach_pose.pose.position.x = 1.4
    assert candidate_physical_signature(first) != candidate_physical_signature(second)


def test_handoff_source_region_is_deterministic():
    grid = _handoff_grid()
    assert region_fingerprint(grid, [(1.0, 1.0)]) == \
        region_fingerprint(grid, [(1.0, 1.0)])


def test_handoff_source_region_preserves_unknown_cells():
    grid = _handoff_grid()
    index, _, _ = grid_cell(grid, 1.0, 1.0)
    grid.data[index] = -1
    assert -1 in region_fingerprint(grid, [(1.0, 1.0)])


def test_handoff_region_outside_grid_is_explicit():
    grid = _handoff_grid()
    assert None in region_fingerprint(grid, [(-1.0, -1.0)])


def test_handoff_revision_policy_is_not_global_equality():
    source = (PROJECT / 'my_epuck_project' / 'nav2_frontier_diagnostic.py').read_text(
        encoding='utf-8')
    assert "remote map revision must not suppress a safe current path" in source


def test_handoff_final_validation_keeps_path_queries_serialized():
    source = (PROJECT / 'my_epuck_project' / 'nav2_frontier_diagnostic.py').read_text(
        encoding='utf-8')
    assert "'frontier_final_validation'" in source
    assert 'if self.planner_requests[robot] is not None:' in source


def test_handoff_artifacts_are_buffered_and_periodically_flushed():
    source = (PROJECT / 'my_epuck_project' / 'nav2_frontier_diagnostic.py').read_text(
        encoding='utf-8')
    assert "buffering=65536" in source
    record_body = source[source.index('def _record_handoff'):source.index('def _handoff_safe_goal')]
    assert '.flush()' not in record_body
    assert 'periodic every 5 wall seconds' in source


def test_repeated_upstream_rejections_are_rate_limited():
    source = (PROJECT / 'my_epuck_project' / 'nav2_frontier_diagnostic.py').read_text(
        encoding='utf-8')
    assert '_candidate_reject_pending' in source
    assert '_candidate_reject_last_emit' in source
    assert 'occurrences=count' in source


def test_timeseries_has_wall_clock_for_rtf_windows():
    source = (PROJECT / 'my_epuck_project' / 'nav2_frontier_diagnostic.py').read_text(
        encoding='utf-8')
    assert "'wall_time_s', 'sim_time_s'" in source
    assert "'wall_time_s': f'{time.monotonic():.6f}'" in source


def test_known_cell_telemetry_uses_bounded_cache():
    source = (PROJECT / 'my_epuck_project' / 'nav2_frontier_diagnostic.py').read_text(
        encoding='utf-8')
    assert 'def _known_cells(self' in source
    assert 'len(self._known_cells_cache) > 64' in source


def test_setup_adds_only_node_console_entry_for_this_diagnostic():
    setup = (PROJECT / 'setup.py').read_text(encoding='utf-8')
    assert setup.count("'nav2_frontier_diagnostic = '") == 1
    assert 'run_nav2_frontier_diagnostic =' not in setup
