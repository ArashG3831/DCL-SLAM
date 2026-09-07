import hashlib
from pathlib import Path

import pytest

from my_epuck_project.thesis_baseline_topology import (
    add_forensic_supervisor,
    environment_geometry_signature,
    materialize_single_robot_world,
    materialize_seeded_world,
    parse_world_random_seed,
    replace_world_random_seed,
    sha256_file,
    parse_active_robots,
)
from my_epuck_project.cooperative_trial_fast import launch_command, parser
from my_epuck_project.cooperative_profiles import profile_for_world


ROOT = Path(__file__).resolve().parents[1]
WORLD = ROOT / 'worlds' / (
    'epuck_d500_two_world_unknown_pose_close_start_dynamic_low_slip_20ms_finite.wbt')
LAUNCH = ROOT / 'launch'


def test_active_robots_parser_accepts_single_and_pair():
    assert parse_active_robots('robot1') == ('robot1',)
    assert parse_active_robots(' robot1, robot2 ') == ('robot1', 'robot2')


@pytest.mark.parametrize('value', ['', ' ', 'robot3', 'robot1,robot1'])
def test_active_robots_parser_rejects_invalid_or_empty_values(value):
    with pytest.raises(ValueError, match='active_robots'):
        parse_active_robots(value)


def test_condition_a_derives_one_robot_without_changing_environment(tmp_path):
    source = WORLD.read_text(encoding='utf-8')
    derived = materialize_single_robot_world(WORLD, tmp_path)
    result = derived.read_text(encoding='utf-8')
    assert 'name "robot1"' in result
    assert 'name "robot2"' not in result
    assert environment_geometry_signature(source) == environment_geometry_signature(result)
    launch = (LAUNCH / 'single_robot_thesis_baseline_launch.py').read_text()
    assert "executable='frontier_explorer'" in launch
    assert "namespace='robot1'" in launch
    assert 'active_robots' in launch


def test_condition_a_derived_world_is_profile_resolvable(tmp_path):
    source_bytes = WORLD.read_bytes()
    derived = materialize_single_robot_world(WORLD, tmp_path / 'unique_a_world')
    derived_text = derived.read_text(encoding='utf-8')
    source_text = source_bytes.decode('utf-8')

    assert derived.parent != WORLD.parent
    assert derived.parent.name == 'worlds'
    assert derived.parent.parent.joinpath('protos').is_symlink()
    assert derived.parent.parent.joinpath('protos').is_dir()
    for relative_proto in (
            '../protos/arena/ThesisRectangleArena.proto',
            '../protos/e-puck/E-puck.proto'):
        assert (derived.parent / relative_proto).resolve().is_file()
    assert derived.name == WORLD.name
    assert derived.is_file()
    assert 'name "robot1"' in derived_text
    assert 'name "robot2"' not in derived_text
    assert environment_geometry_signature(source_text) == (
        environment_geometry_signature(derived_text))
    assert hashlib.sha256(derived.read_bytes()).hexdigest() != (
        hashlib.sha256(source_bytes).hexdigest())

    selected = profile_for_world(
        'large_unknown_pose_close_start_20ms_scan_matching',
        derived.parent,
        explicit_world_path=derived,
    )
    assert selected['name'] == 'large_unknown_pose_close_start_20ms_scan_matching'
    assert Path(selected['world_path']) == derived.resolve()
    assert 'far_start' not in selected['world_path']


def test_condition_a_forensic_supervisor_is_not_robot2_or_geometry(tmp_path):
    source = WORLD.read_text(encoding='utf-8')
    derived = materialize_single_robot_world(WORLD, tmp_path / 'a_forensics')
    before = environment_geometry_signature(derived.read_text())
    add_forensic_supervisor(derived)
    result = derived.read_text(encoding='utf-8')
    assert 'name "ForensicGroundTruthSupervisor"' in result
    assert 'name "robot1"' in result
    assert 'name "robot2"' not in result
    assert environment_geometry_signature(result) == before
    assert environment_geometry_signature(source) == before


def test_seed_materialization_is_deterministic_and_geometry_preserving(tmp_path):
    first = materialize_seeded_world(WORLD, tmp_path / 'seed1', 1001)
    second = materialize_seeded_world(WORLD, tmp_path / 'seed2', 1001)
    third = materialize_seeded_world(WORLD, tmp_path / 'seed3', 1002)
    assert parse_world_random_seed(first) == 1001
    assert first.read_bytes() == second.read_bytes()
    assert first.read_bytes() != third.read_bytes()
    assert environment_geometry_signature(first.read_text()) == (
        environment_geometry_signature(WORLD.read_text()))
    assert environment_geometry_signature(third.read_text()) == (
        environment_geometry_signature(WORLD.read_text()))
    assert sha256_file(first) != sha256_file(WORLD)


def test_seeded_single_robot_materialization_changes_only_allowed_fields(tmp_path):
    derived = materialize_single_robot_world(WORLD, tmp_path / 'seeded_a', 1001)
    assert parse_world_random_seed(derived) == 1001
    assert 'name "robot1"' in derived.read_text()
    assert 'name "robot2"' not in derived.read_text()
    assert environment_geometry_signature(derived.read_text()) == (
        environment_geometry_signature(WORLD.read_text()))


def test_seed_parser_rejects_negative_values():
    with pytest.raises(ValueError):
        replace_world_random_seed(WORLD.read_text(), -1)


def test_condition_a_has_no_cooperative_target_selection_nodes():
    source = (LAUNCH / 'single_robot_thesis_baseline_launch.py').read_text()
    for forbidden in ('distributed_frontier_assignment', 'task_snapshot',
                      'certificate', 'peer_frontier'):
        assert forbidden not in source
    assert "'webots_port': LaunchConfiguration('webots_port')" in source


def test_condition_b_has_two_local_explorers_and_no_coordinator():
    source = (LAUNCH / 'two_robots_independent_exploration_launch.py').read_text()
    assert "for robot in ('robot1', 'robot2')" in source
    assert source.count("executable='frontier_explorer'") == 1
    assert "'map_topic': f'/{robot}/map'" in source
    assert "'active_robots': 'robot1,robot2'" in source
    for forbidden in ('distributed_frontier_assignment', 'task_snapshot',
                      'certificate', 'peer_frontier', 'shared_map'):
        assert forbidden not in source
    assert "'coverage_source': 'local_map_union'" in source
    assert "'webots_port': LaunchConfiguration('webots_port')" in source
    assert 'ForensicGroundTruthSupervisor' in source


def test_baseline_launches_are_distinct_from_cooperative_launch():
    a = (LAUNCH / 'single_robot_thesis_baseline_launch.py').read_text()
    b = (LAUNCH / 'two_robots_independent_exploration_launch.py').read_text()
    cooperative = (LAUNCH / 'two_robots_decentralized_exploration_launch.py').read_text()
    assert 'single_robot_thesis_baseline_launch.py' not in cooperative
    assert 'two_robots_independent_exploration_launch.py' not in cooperative
    assert 'independent_local_maps' in a and 'independent_local_maps' in b


def test_runner_selects_baseline_launches_without_cooperative_arguments(tmp_path):
    for condition, launch_name in (
            ('A', 'single_robot_thesis_baseline_launch.py'),
            ('B', 'two_robots_independent_exploration_launch.py')):
        args = parser().parse_args(['--experiment-condition', condition])
        command = launch_command(args, tmp_path / 'world.wbt')
        assert launch_name in command
        assert not any('assignment_strategy:=' in item for item in command)
        assert not any('local_path_gate_mode:=' in item for item in command)
