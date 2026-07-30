"""Deterministic tests for direct live parameter snapshot conversion."""

import os
from types import SimpleNamespace

from rcl_interfaces.msg import ParameterValue
from rcl_interfaces.srv import ListParameters

from my_epuck_project.navigation_live_parameters import (
    collect_snapshots, native_parameter_values, snapshot_node)


def test_native_parameter_values_preserve_ros_scalar_and_array_types():
    """Scalar, array, string, and bool values become JSON-safe natives."""
    values = [
        ParameterValue(type=1, bool_value=True),
        ParameterValue(type=2, integer_value=6),
        ParameterValue(type=8, double_array_value=[0.1, 0.2]),
        ParameterValue(type=9, string_array_value=['a', 'b']),
    ]
    result = native_parameter_values(['flag', 'samples', 'scales', 'names'], values)
    assert result == {'flag': True, 'samples': 6,
                      'scales': [0.1, 0.2], 'names': ['a', 'b']}


class _FakeFuture:
    def __init__(self, result):
        self._result = result

    def done(self):
        return True

    def exception(self):
        return None

    def result(self):
        return self._result


class _FakeClient:
    def __init__(self, result=None, available=True):
        self.result = result
        self.available = available

    def wait_for_service(self, timeout_sec):
        del timeout_sec
        return self.available

    def call_async(self, request):
        del request
        return _FakeFuture(self.result)


class _FakeNode:
    def __init__(self, clients):
        self.clients = clients

    def create_client(self, service_type, name):
        del service_type
        return self.clients[name]


class _FakeExecutor:
    def spin_until_future_complete(self, future, timeout_sec=None):
        del future, timeout_sec


def test_snapshot_retries_and_reads_lifecycle_state(monkeypatch):
    """A delayed service becomes available within the bounded retry window."""
    listed = SimpleNamespace(result=ListParameters.Response().result)
    listed.result.names = ['FollowPath.vx_samples']
    params = SimpleNamespace(values=[ParameterValue(type=2, integer_value=6)])
    state = SimpleNamespace(current_state=SimpleNamespace(id=3, label='active'))
    node = _FakeNode({
        '/robot1/controller_server/list_parameters': _FakeClient(listed),
        '/robot1/controller_server/get_parameters': _FakeClient(params),
        '/robot1/controller_server/get_state': _FakeClient(state),
    })
    result = snapshot_node(node, '/robot1/controller_server', timeout_s=1.0,
                          executor=_FakeExecutor())
    assert result['status'] == 'OK'
    assert result['parameters']['FollowPath.vx_samples'] == 6
    assert result['lifecycle']['label'] == 'active'


def test_snapshot_reports_missing_service_without_unbounded_wait(monkeypatch):
    """Unavailable parameter services become an explicit timeout record."""
    node = _FakeNode({
        '/robot2/controller_server/list_parameters': _FakeClient(available=False),
        '/robot2/controller_server/get_parameters': _FakeClient(available=False),
        '/robot2/controller_server/get_state': _FakeClient(available=False),
    })
    result = snapshot_node(node, '/robot2/controller_server', timeout_s=0.05,
                          executor=_FakeExecutor())
    assert result['status'] == 'TIMEOUT'


def test_empty_or_invalid_allocated_domain_is_rejected():
    """Domain allocation is explicit and cannot silently fall back to default."""
    import pytest
    with pytest.raises(ValueError):
        collect_snapshots([], '', 0.1)
    with pytest.raises(ValueError):
        collect_snapshots([], 233, 0.1)


def test_collect_snapshots_overrides_and_restores_parent_ros_domain(monkeypatch):
    """The fresh context is initialized while the allocated domain is active."""
    observed = {}
    class FakeNode:
        def destroy_node(self):
            pass
    class FakeExecutor:
        def __init__(self, *, context):
            observed['context'] = context

        def add_node(self, node):
            observed['added_node'] = node

        def remove_node(self, node):
            observed['removed_node'] = node

        def shutdown(self):
            observed['shutdown'] = True

    fake_node = FakeNode()
    monkeypatch.delenv('ROS_DOMAIN_ID', raising=False)
    monkeypatch.setattr(
        'my_epuck_project.navigation_live_parameters.rclpy.init',
        lambda context: observed.update(domain=os.environ['ROS_DOMAIN_ID']))
    monkeypatch.setattr(
        'my_epuck_project.navigation_live_parameters.rclpy.create_node',
        lambda *args, **kwargs: fake_node)
    monkeypatch.setattr(
        'my_epuck_project.navigation_live_parameters.SingleThreadedExecutor',
        FakeExecutor)
    monkeypatch.setattr(
        'my_epuck_project.navigation_live_parameters.snapshot_node',
        lambda node, name, timeout_s, executor: {'node': name, 'status': 'OK'})
    monkeypatch.setattr(
        'my_epuck_project.navigation_live_parameters.rclpy.shutdown',
        lambda context: None)
    monkeypatch.setattr(
        'my_epuck_project.navigation_live_parameters.rclpy.Context.ok',
        lambda self: True)
    assert collect_snapshots(['/robot1/controller_server'], 102, 1.0)
    assert observed['domain'] == '102'
    assert observed['added_node'] is fake_node
    assert observed['removed_node'] is fake_node
    assert observed['shutdown'] is True
    assert 'ROS_DOMAIN_ID' not in os.environ


def test_snapshot_client_uses_context_bound_executor(monkeypatch):
    """A fresh-domain context must not create rclpy's global executor."""
    calls = []

    class FakeExecutor:
        def spin_until_future_complete(self, future, timeout_sec=None):
            calls.append((future, timeout_sec))

    listed = SimpleNamespace(result=SimpleNamespace(names=['value']))
    params = SimpleNamespace(values=[ParameterValue(type=2, integer_value=6)])
    node = _FakeNode({
        '/robot1/controller_server/list_parameters': _FakeClient(listed),
        '/robot1/controller_server/get_parameters': _FakeClient(params),
        '/robot1/controller_server/get_state': _FakeClient(None),
    })
    result = snapshot_node(node, '/robot1/controller_server', 1.0,
                          FakeExecutor())
    assert result['status'] == 'OK'
    assert len(calls) == 3
