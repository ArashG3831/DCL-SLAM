"""Bounded direct rclpy parameter snapshots for live Nav2 nodes."""

import os
import time

import rclpy
from lifecycle_msgs.srv import GetState
from rcl_interfaces.srv import GetParameters, ListParameters
from rclpy.executors import SingleThreadedExecutor
from rclpy.parameter import parameter_value_to_python

from .ros_runtime_preflight import ROS_DOMAIN_MIN, ROS_DOMAIN_MAX


SNAPSHOT_SCHEMA = '2.0.0'


def native_parameter_values(names, values):
    """Convert ROS parameter values to stable native Python values."""
    return {name: parameter_value_to_python(value)
            for name, value in zip(names, values)}


def _call(node, client, request, deadline, executor=None):
    """Wait for one service call without exceeding the shared deadline."""
    if executor is None:
        raise ValueError('parameter service calls require an explicit executor')
    while time.monotonic() < deadline:
        remaining = max(0.01, deadline - time.monotonic())
        if not client.wait_for_service(timeout_sec=min(0.25, remaining)):
            continue
        future = client.call_async(request)
        executor.spin_until_future_complete(
            future, timeout_sec=min(0.5, remaining))
        if future.done() and future.exception() is None:
            return future.result()
    return None


def snapshot_node(node, node_name, timeout_s=4.0, executor=None):
    """Read one node's parameters and lifecycle state with bounded retries."""
    started = time.monotonic()
    deadline = started + timeout_s
    list_client = node.create_client(ListParameters,
                                     f'{node_name}/list_parameters')
    get_client = node.create_client(GetParameters,
                                    f'{node_name}/get_parameters')
    state_client = node.create_client(GetState, f'{node_name}/get_state')
    listed = _call(node, list_client, ListParameters.Request(), deadline,
                   executor)
    if listed is None:
        return {'node': node_name, 'schema_version': SNAPSHOT_SCHEMA,
                'status': 'TIMEOUT', 'elapsed_s': time.monotonic() - started}
    # Prefixes are grouping metadata, not parameter names accepted by get_parameters.
    names = sorted(set(listed.result.names))
    request = GetParameters.Request(names=names)
    response = _call(node, get_client, request, deadline, executor)
    if response is None:
        return {'node': node_name, 'schema_version': SNAPSHOT_SCHEMA,
                'status': 'TIMEOUT', 'elapsed_s': time.monotonic() - started}
    result = {
        'node': node_name,
        'schema_version': SNAPSHOT_SCHEMA,
        'status': 'OK',
        'captured_wall_time_ns': time.time_ns(),
        'elapsed_s': time.monotonic() - started,
        'parameters': native_parameter_values(names, response.values),
    }
    state = _call(node, state_client, GetState.Request(), deadline, executor)
    if state is not None:
        result['lifecycle'] = {
            'id': int(state.current_state.id),
            'label': state.current_state.label,
        }
    else:
        result['lifecycle'] = {'status': 'UNAVAILABLE'}
    return result


def collect_snapshots(node_names, domain_id, timeout_s=8.0):
    """Collect bounded snapshots in one rclpy context and ROS domain."""
    if isinstance(domain_id, bool) or not isinstance(domain_id, int):
        raise ValueError('allocated ROS domain must be an integer')
    if not ROS_DOMAIN_MIN <= domain_id <= ROS_DOMAIN_MAX:
        raise ValueError(
            f'allocated ROS domain must be between {ROS_DOMAIN_MIN} and '
            f'{ROS_DOMAIN_MAX} for the active CycloneDDS port profile')
    previous_domain = os.environ.get('ROS_DOMAIN_ID')
    os.environ['ROS_DOMAIN_ID'] = str(domain_id)
    context = rclpy.context.Context()
    try:
        rclpy.init(context=context)
        node = rclpy.create_node('navigation_live_parameter_snapshot',
                                 context=context)
        executor = SingleThreadedExecutor(context=context)
        executor.add_node(node)
        deadline = time.monotonic() + timeout_s
        snapshots = []
        for name in node_names:
            remaining = max(0.1, deadline - time.monotonic())
            snapshot = snapshot_node(node, name, remaining, executor)
            snapshot['allocated_domain_id'] = domain_id
            snapshot['effective_context_domain_id'] = domain_id
            snapshots.append(snapshot)
        return snapshots
    finally:
        if 'executor' in locals():
            executor.remove_node(node)
            executor.shutdown()
        if 'node' in locals():
            node.destroy_node()
        if context.ok():
            rclpy.shutdown(context=context)
        if previous_domain is None:
            os.environ.pop('ROS_DOMAIN_ID', None)
        else:
            os.environ['ROS_DOMAIN_ID'] = previous_domain
