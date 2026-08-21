"""Deterministic tests for the Webots controller startup contract."""

from my_epuck_project.controller_readiness_gate import active_required
from my_epuck_project.controller_startup_guard import (
    controller_action,
    manager_state_available,
    spawner_ros_home,
)


def test_each_robot_requires_both_named_controllers_active():
    ready = {
        'diffdrive_controller': 'active',
        'joint_state_broadcaster': 'active',
    }
    assert active_required(ready)
    assert not active_required({**ready, 'diffdrive_controller': 'inactive'})
    assert not active_required({'diffdrive_controller': 'active'})


def test_startup_guard_is_state_aware_and_never_treats_missing_as_ready():
    assert controller_action(None) == 'LOAD'
    assert controller_action('unconfigured') == 'CONFIGURE'
    assert controller_action('inactive') == 'ACTIVATE'
    assert controller_action('active') == 'READY'
    assert controller_action('error') == 'RETRY'


def test_controller_spawners_use_independent_campaign_owned_ros_homes():
    robot1_home = spawner_ros_home('/robot1/controller_manager', process_id=101)
    robot2_home = spawner_ros_home('/robot2/controller_manager', process_id=101)
    assert robot1_home != robot2_home
    assert robot1_home.endswith('robot1_101')
    assert robot2_home.endswith('robot2_101')


def test_startup_waits_for_manager_services_before_spawning():
    assert not manager_state_available(None)
    assert manager_state_available({})
