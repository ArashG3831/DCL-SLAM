"""Deferred replay for the legacy pair-decision accounting family.

The live logger and this replay deliberately share ``pair_decision_outcome``;
the replay does not define a second classification rule.
"""

from __future__ import annotations

import json
from collections import Counter
from pathlib import Path

from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


def pair_decision_outcome(message):
    """Return the legacy deterministic outcome and parsed diagnostics."""
    try:
        diagnostics = json.loads(message.diagnostics_json or '{}')
    except (TypeError, ValueError):
        diagnostics = {}
    if not message.robot1_canonical_task_id and not message.robot2_canonical_task_id:
        if not diagnostics.get('union_task_count', 0):
            outcome = 'NO_CANONICAL_TASKS'
        elif diagnostics.get('rejected_failure_suppression_count', 0):
            outcome = 'TASK_SUPPRESSED_BY_FAILURE_MEMORY'
        elif diagnostics.get('rejected_path_threshold_count', 0):
            outcome = 'TASKS_OUT_OF_RANGE'
        elif diagnostics.get('robot1_valid_bid_count', 0) == 0 and \
                diagnostics.get('robot2_valid_bid_count', 0) == 0:
            outcome = 'NO_REACHABLE_TASK'
        else:
            outcome = 'IDLE_BY_DETERMINISTIC_ASSIGNMENT'
    else:
        outcome = 'DISPATCHABLE_ASSIGNMENT'
    return outcome, diagnostics


def replay_pair_decisions_from_bag(bag_directory: Path, robots):
    """Replay pair-decision callbacks with legacy duplicate suppression."""
    import rosbag2_py

    bag_directory = Path(bag_directory)
    if not bag_directory.is_dir():
        raise FileNotFoundError(bag_directory)
    robots = tuple(str(robot) for robot in robots)
    topics = {f'/{robot}/pair_decision': robot for robot in robots}
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(bag_directory), storage_id='sqlite3'),
        rosbag2_py.ConverterOptions(
            input_serialization_format='cdr',
            output_serialization_format='cdr'))
    type_map = {item.name: item.type
                for item in reader.get_all_topics_and_types()}
    missing = [topic for topic in topics
               if topic in type_map and
               type_map[topic] != 'my_epuck_interfaces/msg/PairDecision']
    if missing:
        raise ValueError(f'pair-decision topics missing or mistyped: {missing}')
    message_type = get_message('my_epuck_interfaces/msg/PairDecision')
    previous = {}
    outcomes = Counter()
    records = []
    deserialized_messages = 0
    while reader.has_next():
        topic, serialized, _bag_timestamp = reader.read_next()
        robot = topics.get(topic)
        if robot is None:
            continue
        message = deserialize_message(serialized, message_type)
        deserialized_messages += 1
        fingerprint = (
            message.round_id, message.union_hash, message.decision_hash)
        if previous.get(robot) == fingerprint:
            continue
        previous[robot] = fingerprint
        outcome, diagnostics = pair_decision_outcome(message)
        outcomes[outcome] += 1
        records.append({
            'robot_id': robot,
            'round_id': message.round_id,
            'union_hash': message.union_hash,
            'decision_hash': message.decision_hash,
            'decision_outcome': outcome,
            'diagnostics': diagnostics,
        })
    return {
        'round_outcomes': dict(outcomes),
        'records': records,
        'deserialized_messages': deserialized_messages,
        'robots': list(robots),
    }
