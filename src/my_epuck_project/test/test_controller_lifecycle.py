"""Deterministic tests for the Webots controller startup contract."""

import subprocess

from my_epuck_project.controller_readiness_gate import active_required
from my_epuck_project.controller_startup_guard import (
    ControllerStartupGuard,
    controller_action,
    manager_state_available,
    spawner_ros_home,
)


def test_startup_guard_uses_bounded_but_startup_tolerant_service_budget():
    """The guard allows slow Webots manager discovery without hanging."""
    from pathlib import Path
    source = (
        Path(__file__).resolve().parents[1]
        / 'my_epuck_project' / 'controller_startup_guard.py'
    ).read_text(encoding='utf-8')
    assert "declare_parameter('service_timeout_s', 15.0)" in source
    assert 'timeout=self.service_timeout_s + 1.0' in source


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


def test_standard_spawner_timeout_returns_to_bounded_retry(monkeypatch):
    """A wedged child spawner must not deadlock the startup guard."""
    guard = object.__new__(ControllerStartupGuard)
    guard.controller_name = 'diffdrive_controller'
    guard.manager = '/robot1/controller_manager'
    guard.param_file = ''
    guard.controller_ros_args = ''
    guard.service_timeout_s = 0.2

    class Logger:
        def info(self, message):
            del message

        def warning(self, message):
            del message

    guard.get_logger = lambda: Logger()

    def blocked(*args, **kwargs):
        del args, kwargs
        raise subprocess.TimeoutExpired('spawner', 0.2)

    monkeypatch.setattr(subprocess, 'run', blocked)
    assert guard.invoke_standard_spawner() is False
