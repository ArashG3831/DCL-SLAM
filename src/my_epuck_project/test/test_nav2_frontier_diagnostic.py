from pathlib import Path
from types import SimpleNamespace

from my_epuck_project.nav2_frontier_diagnostic import (
    ARTIFACT_NAMES,
    boolean,
    deterministic_candidate_key,
    DiagnosticNode,
    grid_cell,
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


def test_artifact_and_time_breakdown_contracts_are_complete():
    assert ARTIFACT_NAMES == {
        'launch.log', 'diagnostic_events.jsonl',
        'diagnostic_timeseries.csv', 'diagnostic_summary.json',
        'effective_command.txt',
    }
    assert len(TIME_STATES) == 11
    assert 'active navigation with nonzero cmd_vel' in TIME_STATES
    assert 'active goal with zero cmd_vel' in TIME_STATES
    assert 'idle despite valid candidates' in TIME_STATES


def test_setup_adds_only_node_console_entry_for_this_diagnostic():
    setup = (PROJECT / 'setup.py').read_text(encoding='utf-8')
    assert setup.count("'nav2_frontier_diagnostic = '") == 1
    assert 'run_nav2_frontier_diagnostic =' not in setup
