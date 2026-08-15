from pathlib import Path

from my_epuck_project.motion_course_spec import COURSE


PROJECT = Path(__file__).resolve().parents[1]


def test_authoritative_large_world_nav2_limit_is_013():
    for robot in ('robot1', 'robot2'):
        text = (PROJECT / 'resource' / f'nav2_{robot}_shared_map.yaml').read_text()
        assert 'max_vel_x: 0.13' in text


def test_course_has_both_turn_directions_and_returns_to_start_heading():
    turns = [segment for segment in COURSE if segment['kind'] == 'turn']
    assert {segment['direction'] for segment in turns} == {-1, 1}
    assert abs(sum(segment['distance_m'] for segment in COURSE if segment['kind'] == 'straight') - 3.6) < 1.0e-9
    assert abs(sum(segment['angle_rad'] * segment['direction'] for segment in turns)) < 1.0e-9


def test_supervisor_is_external_and_recorder_uses_explicit_clock_stream():
    supervisor = (PROJECT / 'my_epuck_project' / 'motion_course_supervisor.py').read_text()
    recorder = (PROJECT / 'my_epuck_project' / 'motion_slam_recorder.py').read_text()
    assert 'from controller import Supervisor' in supervisor
    assert 'create_subscription(Clock, \'/clock\'' in recorder
    assert 'create_publisher' not in supervisor


def test_course_capture_records_all_scan_branches():
    node = (PROJECT / 'my_epuck_project' / 'motion_course_node.py').read_text()
    for topic in ('scan_d500', 'scan_d500_fixed', 'scan_d500_slam'):
        assert topic in node
    assert 'scan_timestamps.csv' in node


def test_course_launch_does_not_change_production_nav2_or_slam_files():
    launch = (PROJECT / 'launch' / 'motion_characterization_launch.py').read_text()
    assert "DeclareLaunchArgument('course_mode'" in launch
    assert 'motion_course_node' in launch
    assert 'motion_course_supervisor' in launch
    assert 'stop_on_joints_failure' in launch
    assert 'stop_on_diffdrive_failure' in launch
