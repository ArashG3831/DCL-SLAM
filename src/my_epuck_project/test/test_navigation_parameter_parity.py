"""Test shared navigation behavior and the DWB command lattice contract."""

# flake8: noqa

from pathlib import Path
import math

import yaml

from my_epuck_project.navigation_parameter_parity import (
    live_snapshot_report, parity_report, parameter_tree_parity)


PACKAGE = Path(__file__).resolve().parents[1]
ROBOT1 = PACKAGE / "resource/nav2_robot1_shared_map.yaml"
ROBOT2 = PACKAGE / "resource/nav2_robot2_shared_map.yaml"


def _dwb(path):
    """Read the DWB parameter block from a robot parameter file."""
    with path.open(encoding="utf-8") as stream:
        return yaml.safe_load(stream)["controller_server"]["ros__parameters"]["FollowPath"]


def _samples(minimum, maximum, count):
    """Model the installed iterator's regular samples and zero insertion."""
    step = (maximum - minimum) / (count - 1)
    values = [minimum + index * step for index in range(count)]
    if minimum < 0.0 < maximum and count % 2 == 0:
        values.insert(count // 2, 0.0)
    return values


def test_robot_navigation_parameters_are_behaviorally_identical():
    """Identity-only source differences must normalize to one hash."""
    report = parity_report(ROBOT1, ROBOT2)
    assert report["conclusion"] == "EXPECTED_IDENTITY_FIELDS_ONLY"
    assert report["normalized_hash"] == report["robot1_parameter_hash"]


def test_changed_robot2_dwb_parameter_fails_parity(tmp_path):
    """A behavioral DWB change must fail the parity report."""
    changed = tmp_path / "robot2.yaml"
    data = yaml.safe_load(ROBOT2.read_text(encoding="utf-8"))
    data["controller_server"]["ros__parameters"]["FollowPath"]["BaseObstacle.scale"] = 0.21
    changed.write_text(yaml.safe_dump(data), encoding="utf-8")
    assert parity_report(ROBOT1, changed)["conclusion"] == (
        "UNEXPECTED_ROBOT_PARAMETER_DIVERGENCE")


def test_changed_robot2_smoother_and_goal_checker_fail_parity(tmp_path):
    """Smoother or goal-checker divergence must fail the parity report."""
    data = yaml.safe_load(ROBOT2.read_text(encoding="utf-8"))
    data["velocity_smoother"]["ros__parameters"]["deadband_velocity"] = [0.01, 0.0, 0.0]
    data["controller_server"]["ros__parameters"]["general_goal_checker"]["yaw_goal_tolerance"] = 0.1
    changed = tmp_path / "robot2.yaml"
    changed.write_text(yaml.safe_dump(data), encoding="utf-8")
    assert parity_report(ROBOT1, changed)["conclusion"] == (
        "UNEXPECTED_ROBOT_PARAMETER_DIVERGENCE")


def test_fixed_lattice_has_stop_rotation_and_executable_forward_samples():
    """The fixed lattice stays within the Webots wheel-speed envelope."""
    dwb = _dwb(ROBOT1)
    assert dwb["max_vel_x"] == 0.13
    assert dwb["max_speed_xy"] == 0.13
    linear = _samples(0.0, dwb["max_vel_x"], dwb["vx_samples"])
    angular = _samples(-dwb["max_vel_theta"], dwb["max_vel_theta"], dwb["vtheta_samples"])
    assert 0.0 in linear
    assert 0.0 in angular
    assert math.isclose(
        min(value for value in linear if value > 0.0), 0.026,
        abs_tol=1e-12)
    effective_separation = 0.052 * 1.095
    worst_case_wheel_speed = (
        dwb["max_vel_x"] + dwb["max_vel_theta"] * effective_separation / 2.0
    ) / 0.02
    assert worst_case_wheel_speed <= 7.0
    assert math.isclose(
        min(abs(value) for value in angular if value != 0.0), 0.035,
        abs_tol=1e-12)
    assert any(value > 0.0 and angle == 0.0 for value in linear for angle in angular)
    assert any(value == 0.0 and angle != 0.0 for value in linear for angle in angular)


def test_old_even_lattice_contains_sub_response_samples():
    """The old lattice contains the observed sub-response samples."""
    assert min(_samples(0.0, 0.05, 20)[1:]) == 0.002631578947368421
    old_angular = _samples(-0.35, 0.35, 20)
    assert 0.0 in old_angular
    assert math.isclose(
        min(abs(value) for value in old_angular if value != 0.0),
        0.018421052631578947, abs_tol=1e-12)


def test_live_snapshot_parity_normalizes_node_and_frame_identity():
    """Live YAML-shaped trees normalize namespace and frame identities."""
    robot1 = {'/robot1/controller_server': {'frame': 'robot1/base_link',
                                             'FollowPath': {'vx_samples': 6}}}
    robot2 = {'/robot2/controller_server': {'frame': 'robot2/base_link',
                                             'FollowPath': {'vx_samples': 6}}}
    report = live_snapshot_report(robot1, robot2, [
        {'node': '/robot1/controller_server', 'returncode': 0, 'parsed': True},
        {'node': '/robot2/controller_server', 'returncode': 0, 'parsed': True},
    ])
    assert report['conclusion'] == 'EXPECTED_IDENTITY_FIELDS_ONLY'


def test_live_snapshot_behavioral_difference_fails():
    """A live Robot 2 lattice change fails normalized parity."""
    statuses = [{'node': '/robot1/controller_server', 'returncode': 0, 'parsed': True},
                {'node': '/robot2/controller_server', 'returncode': 0, 'parsed': True}]
    report = live_snapshot_report({'vx_samples': 6}, {'vx_samples': 7}, statuses)
    assert report['conclusion'] == 'UNEXPECTED_ROBOT_PARAMETER_DIVERGENCE'


def test_missing_live_parameter_service_is_not_reported_as_parity():
    """Missing services produce an honest bounded failure classification."""
    report = live_snapshot_report({}, {}, [
        {'node': '/robot2/controller_server', 'returncode': 1, 'parsed': False},
    ])
    assert report['conclusion'] == 'LIVE_PARAMETER_SNAPSHOT_FAILED'
    assert report['missing_live_parameter_services'] == ['/robot2/controller_server']
