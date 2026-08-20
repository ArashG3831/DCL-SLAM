from pathlib import Path
import inspect

from my_epuck_project.cooperative_trial_fast import (
    GRAPH_SUFFIXES,
    LAUNCH_FILE,
    LOCAL_NAV2_NODES,
    ReadyProbe,
    boolean,
    is_campaign_webots_driver,
    launch_command,
    parser,
)


def test_fast_parser_exposes_only_single_trial_options():
    args = parser().parse_args([])
    assert args.world_profile == 'large'
    assert args.sensor_profile == 'full'
    assert args.rendering is False
    assert args.rviz is False
    assert args.hold_open is False
    assert args.enable_observer is False
    assert args.enable_forensic_capture is False


def test_boolean_parser():
    assert boolean('true') is True
    assert boolean('false') is False


def test_launch_command_reuses_authoritative_campaign_launch(tmp_path):
    args = parser().parse_args([
        '--world-profile', 'small', '--sensor-profile', 'throughput',
        '--webots-mode', 'realtime', '--rendering', 'true',
        '--diagnostic-mode', 'true',
    ])
    command = launch_command(args, Path('/tmp/test_world.wbt'))
    assert command[:4] == [
        'ros2', 'launch', 'my_epuck_project', LAUNCH_FILE]
    assert 'enable_observer:=false' in command
    assert 'enable_forensic_capture:=false' in command
    assert 'nav2_autostart:=false' in command
    assert 'unknown_initial_pose:=true' in command
    assert 'controller_variant:=rpp' in command
    assert 'sensor_profile:=throughput' in command
    assert not any(
        item.startswith('slam_tf_publish_probe_library:=')
        for item in command)
    assert not any(
        item.startswith('slam_tf_publish_probe_log:=')
        for item in command)
    assert not any(
        item.startswith('slam_tf_publication_mode:=')
        for item in command)
    assert 'webots_gui:=true' in command


def test_launch_command_can_enable_passive_evidence_in_attempt_directory(tmp_path):
    args = parser().parse_args([
        '--enable-observer', 'true',
        '--enable-forensic-capture', 'true',
    ])
    command = launch_command(
        args, Path('/tmp/test_world.wbt'),
        output_root=tmp_path / 'observer', run_id='attempt_01')
    assert 'enable_observer:=true' in command
    assert 'enable_forensic_capture:=true' in command
    assert f'output_root:={tmp_path / "observer"}' in command
    assert 'run_id:=attempt_01' in command


def test_readiness_graph_is_the_cooperative_graph():
    assert GRAPH_SUFFIXES == (
        '/robot1/distributed_frontier_assignment',
        '/robot2/distributed_frontier_assignment',
        '/robot1/map_fusion',
        '/robot2/map_fusion',
    )


def test_unknown_pose_readiness_uses_local_nav2_and_local_map_tf():
    assert LOCAL_NAV2_NODES
    assert all(name.startswith('local_') for name in LOCAL_NAV2_NODES)
    source = ReadyProbe.tf_ready.__code__.co_consts
    assert 'shared_map' not in source
    method_source = inspect.getsource(ReadyProbe.activate_and_check_nav2)
    assert 'LOCAL_NAV2_NODES' in method_source
    assert 'lifecycle_manager_navigation/manage_nodes' not in method_source


def test_campaign_driver_filter_is_namespaced_and_not_broad_kill():
    valid = [
        '/home/arash/webots_ws/install/webots_ros2_driver/lib/webots_ros2_driver/driver',
        '-r', '__ns:=/robot1',
        '--params-file', '/tmp/my_epuck_project_robot1_ros2_control.yml',
    ]
    unrelated = [
        '/opt/other_driver', '-r', '__ns:=/robot1',
        '--params-file', '/tmp/other.yml',
    ]
    assert is_campaign_webots_driver(valid)
    assert not is_campaign_webots_driver(unrelated)
