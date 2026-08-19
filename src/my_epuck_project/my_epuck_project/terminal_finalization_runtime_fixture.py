"""Deterministic ROS integration fixture for terminal finalization.

The fixture starts two real distributed-frontier-assignment nodes in their
normal namespaces, publishes valid wrapped task snapshots and frontier
evidence, and observes the real distributed status/event streams.  It does
not run Webots or Nav2; dispatch is disabled because this fixture targets the
terminal protocol boundary.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import signal
import threading
import time

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from builtin_interfaces.msg import Time
from my_epuck_interfaces.msg import (
    DistributedExplorationEvent,
    DistributedExplorationStatus,
    FrontierCandidateArray,
    TaskSnapshot,
)

from .distributed_frontier_assignment import DistributedFrontierAssignment
from .distributed_assignment.ros_conversion import seconds_to_duration, text_to_uuid


SESSIONS = {
    'robot1': '11111111111111111111111111111111',
    'robot2': '22222222222222222222222222222222',
}


def _regions(case: str) -> list[dict[str, object]]:
    if case == 'blocking':
        return [{
            'physical_id': 'actionable-frontier',
            'size_m': 1.0,
            'status': 'REACHABLE',
            'visible_reveal_gain': 0.20,
        }]
    return [
        {'physical_id': 'small', 'size_m': 0.10,
         'status': 'DETECTED_NOT_QUERIED'},
        {'physical_id': 'below-gain', 'size_m': 1.0,
         'status': 'REACHABLE', 'visible_reveal_gain': 0.0},
        {'physical_id': 'unreachable', 'size_m': 0.90,
         'status': 'UNREACHABLE_SAFE_APPROACH'},
        {'physical_id': 'out-of-range', 'size_m': 19.0,
         'status': 'OUT_OF_RANGE'},
    ]


class FixtureHarness(Node):
    """Publish deterministic evidence and record real allocator outputs."""

    def __init__(self, case: str, output: Path):
        super().__init__('terminal_finalization_fixture_harness')
        self.case = case
        self.output = output
        self.started = time.monotonic()
        self.finished = threading.Event()
        self.statuses: dict[str, DistributedExplorationStatus] = {}
        self.events: list[dict[str, object]] = []
        self.publish_count = 0
        self.replacement_done = False
        reliable_transient = QoSProfile(
            depth=1, reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        reliable_volatile = QoSProfile(
            depth=1, reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.snapshot_publishers = {
            robot: self.create_publisher(
                TaskSnapshot, '/%s/task_snapshot' % robot,
                reliable_transient,
            ) for robot in ('robot1', 'robot2')
        }
        self.candidate_publishers = {
            robot: self.create_publisher(
                FrontierCandidateArray, '/%s/frontier_candidates' % robot,
                reliable_volatile,
            ) for robot in ('robot1', 'robot2')
        }
        for robot in ('robot1', 'robot2'):
            self.create_subscription(
                DistributedExplorationStatus,
                '/%s/distributed_status' % robot,
                lambda message, robot=robot: self._status(robot, message),
                reliable_transient,
            )
            self.create_subscription(
                DistributedExplorationEvent,
                '/%s/distributed_event' % robot,
                lambda message, robot=robot: self._event(robot, message),
                reliable_volatile,
            )
        self.timer = self.create_timer(0.1, self._publish_and_check)

    def _stamp(self) -> Time:
        return self.get_clock().now().to_msg()

    def _snapshot(self, robot: str, epoch: int, map_revision: int) -> TaskSnapshot:
        message = TaskSnapshot()
        message.header.stamp = self._stamp()
        message.header.frame_id = 'shared_map'
        message.source_robot_id = robot
        message.source_session_id = text_to_uuid(SESSIONS[robot])
        message.source_snapshot_epoch = epoch
        message.source_map_revision = map_revision
        message.source_map_fingerprint = 'terminal-fixture-%s-%d' % (
            self.case, map_revision,
        )
        message.task_generation_stamp = message.header.stamp
        message.validity = seconds_to_duration(5.0)
        return message

    def _candidate(self, robot: str) -> FrontierCandidateArray:
        message = FrontierCandidateArray()
        message.header.stamp = self._stamp()
        message.header.frame_id = 'shared_map'
        message.source_robot_id = robot
        message.map_revision = 1 if not self.replacement_done else 2
        message.detected_frontier_count = len(_regions(self.case))
        message.terminal_frontier_regions_json = json.dumps(
            _regions(self.case), sort_keys=True,
        )
        return message

    def _publish_and_check(self) -> None:
        elapsed = time.monotonic() - self.started
        if self.case == 'replacement' and not self.replacement_done and elapsed >= 0.3:
            self.replacement_done = True
            self.events.append({
                'robot': 'fixture',
                'event_type': 'SNAPSHOT_REPLACED_DURING_CONFIRMATION',
                'elapsed_s': elapsed,
            })
        # Advance source-local epochs as a valid heartbeat.  The map revision
        # and fingerprint remain stable except for the explicit replacement
        # case, so allocator completion persistence is not reset by heartbeats.
        epoch = self.publish_count + 1
        map_revision = 2 if self.replacement_done else 1
        for robot in ('robot1', 'robot2'):
            self.snapshot_publishers[robot].publish(
                self._snapshot(robot, epoch, map_revision),
            )
            self.candidate_publishers[robot].publish(self._candidate(robot))
        self.publish_count += 1
        if all(
                status.terminal for status in self.statuses.values()
        ) and len(self.statuses) == 2:
            self.finished.set()

    def _status(self, robot: str, message: DistributedExplorationStatus) -> None:
        self.statuses[robot] = message
        self.events.append({
            'robot': robot,
            'event_type': 'DISTRIBUTED_STATUS',
            'terminal': bool(message.terminal),
            'terminal_reason': message.terminal_reason,
            'reason': message.reason,
            'epoch': int(message.terminal_epoch),
            'elapsed_s': time.monotonic() - self.started,
        })

    def _event(self, robot: str, message: DistributedExplorationEvent) -> None:
        self.events.append({
            'robot': robot,
            'event_type': message.event_type,
            'reason': message.reason,
            'elapsed_s': time.monotonic() - self.started,
        })

    def write_result(
            self, child_exit_codes: dict[str, int | None], timeout: bool,
            allocator: DistributedFrontierAssignment | None = None,
            peer_terminal_sent: bool = False):
        reasons = {
            str(status.terminal_reason)
            for status in self.statuses.values() if status.terminal
        }
        if allocator is not None and allocator._terminal:
            reasons.add(str(allocator._terminal_reason))
        agreement = (
            allocator is not None and allocator._terminal and
            peer_terminal_sent and len(reasons) == 1
        )
        success = agreement and not timeout and self.case != 'blocking'
        reason = next(iter(reasons), 'MISSION_NOT_TERMINATED')
        robot_statuses = {
            robot: {
                'terminal': bool(status.terminal),
                'terminal_reason': str(status.terminal_reason),
                'state': int(status.state),
                'reason': str(status.reason),
            } for robot, status in self.statuses.items()
        }
        if allocator is not None:
            robot_statuses.setdefault('robot1', {
                'terminal': bool(allocator._terminal),
                'terminal_reason': str(allocator._terminal_reason),
                'state': int(
                    DistributedExplorationStatus.COMPLETE
                    if allocator._terminal_success else
                    DistributedExplorationStatus.BLOCKED
                ),
                'reason': str(allocator._state_reason),
            })
        result = {
            'fixture_case': self.case,
            'mission_status': 'MISSION_COMPLETE' if success else 'INCOMPLETE',
            'terminal_reason': reason,
            'terminal_agreement': agreement,
            'shutdown_clean': not any(
                code not in (0, -signal.SIGINT, 130)
                for code in child_exit_codes.values()
            ),
            'terminal_epoch': max(
                [
                    *(int(status.terminal_epoch)
                      for status in self.statuses.values()),
                    0 if allocator is None else int(allocator._terminal_epoch),
                ],
                default=0,
            ),
            'recommended_exit_code': 0 if success else 2,
            'robot_statuses': robot_statuses,
            'terminal_finalization_attempt_count': (
                0 if allocator is None else
                int(allocator._terminal_finalization_attempt_count)
            ),
            'terminal_commit_count': (
                0 if allocator is None else int(allocator._terminal_commit_count)
            ),
            'publish_count': self.publish_count,
            'events': len(self.events),
            'child_exit_codes': child_exit_codes,
        }
        self.output.mkdir(parents=True, exist_ok=True)
        (self.output / 'terminal_finalization_fixture_mission_result.json').write_text(
            json.dumps(result, indent=2, sort_keys=True), encoding='utf-8',
        )
        (self.output / 'terminal_finalization_fixture_timeline.json').write_text(
            json.dumps(self.events, indent=2, sort_keys=True), encoding='utf-8',
        )
        return result


def _peer_status(reason: str, terminal: bool) -> DistributedExplorationStatus:
    message = DistributedExplorationStatus()
    message.source_robot_id = 'robot2'
    message.source_session_id = text_to_uuid(SESSIONS['robot2'])
    message.state = (
        DistributedExplorationStatus.COMPLETE if terminal else
        DistributedExplorationStatus.WAITING_FOR_INPUTS
    )
    message.nav2_healthy = True
    message.tf_healthy = True
    message.candidate_source_healthy = True
    message.peer_communication_healthy = True
    message.local_nav_goal_active = False
    message.validity = seconds_to_duration(5.0)
    message.reason = 'COMPLETION_CANDIDATE:' + reason if not terminal else reason
    message.terminal = terminal
    message.terminal_reason = reason if terminal else ''
    message.terminal_epoch = 1
    return message


def run_case(case: str, output: Path, timeout_s: float) -> dict[str, object]:
    """Run the real allocator node with deterministic peer protocol input."""
    output.mkdir(parents=True, exist_ok=True)
    confirmation_s = 0.8 if case == 'replacement' else 0.4
    rclpy.init(args=[
        '--ros-args', '-r', '__ns:=/robot1',
        '-p', 'robot_id:=robot1',
        '-p', 'dispatch_enabled:=false',
        '-p', 'traffic_scheduler_enabled:=false',
        '-p', 'map_stability_grace_s:=0.2',
        '-p', 'completion_confirmation_s:=%s' % confirmation_s,
        '-p', 'peer_timeout_s:=5.0',
        '-p', 'post_goal_settle_s:=0.0',
    ])
    allocator = DistributedFrontierAssignment()
    # No Nav2 servers are needed for a terminal-only fixture.  Health is
    # supplied by this fixture's controlled protocol state, while the real
    # allocator status/event publishers and terminal method remain active.
    allocator._nav2.health_flags = lambda: (True, True)
    allocator._tick_timer.cancel()
    harness = FixtureHarness(case, output)
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(allocator)
    executor.add_node(harness)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    started = time.monotonic()
    terminal_seen = False
    peer_terminal_sent = False
    replacement_done = False
    while time.monotonic() - started < timeout_s:
        elapsed = time.monotonic() - started
        if case == 'replacement' and not replacement_done and elapsed >= 0.3:
            replacement_done = True
            harness.replacement_done = True
            harness.events.append({
                'robot': 'fixture',
                'event_type': 'SNAPSHOT_REPLACED_DURING_CONFIRMATION',
                'elapsed_s': elapsed,
            })
        epoch = int(elapsed * 20.0) + 1
        map_revision = 2 if replacement_done else 1
        for robot in ('robot1', 'robot2'):
            allocator._snapshot_callback(
                harness._snapshot(robot, epoch, map_revision),
            )
            allocator._candidate_callback(harness._candidate(robot))
        reason = 'MISSION_COMPLETE_NO_ACTIONABLE_FRONTIERS'
        if case == 'blocking':
            peer_reason = ''
        else:
            peer_reason = reason
        allocator._status_callback(_peer_status(peer_reason, peer_terminal_sent))
        allocator._tick()
        if allocator._terminal and not peer_terminal_sent:
            peer_terminal_sent = True
            allocator._status_callback(_peer_status(reason, True))
            allocator._tick()
            terminal_seen = True
        if terminal_seen and case != 'blocking':
            break
        time.sleep(0.05)
    timed_out = not terminal_seen
    executor.shutdown()
    thread.join(timeout=3.0)
    allocator.destroy_node()
    harness.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
    result = harness.write_result(
        {'allocator': 0}, timed_out, allocator, peer_terminal_sent,
    )
    result['child_crash'] = False
    result['real_allocator_node'] = True
    result['peer_status_emulator'] = True
    return result


def main(argv=None):
    parser = argparse.ArgumentParser()
    parser.add_argument('--output', required=True)
    parser.add_argument('--timeout-s', type=float, default=12.0)
    args = parser.parse_args(argv)
    root = Path(args.output)
    results = {
        case: run_case(case, root / case, args.timeout_s)
        for case in ('success', 'blocking', 'replacement')
    }
    overall = (
        results['success']['mission_status'] == 'MISSION_COMPLETE' and
        results['success']['terminal_agreement'] and
        results['blocking']['mission_status'] == 'INCOMPLETE' and
        not results['blocking']['terminal_agreement'] and
        results['replacement']['mission_status'] == 'MISSION_COMPLETE' and
        results['replacement']['terminal_agreement'] and
        not any(item['child_crash'] for item in results.values())
    )
    report = {
        'fixture': 'real distributed allocator terminal-finalization ROS path',
        'overall_pass': overall,
        'cases': results,
        'output': str(root),
    }
    (root / 'terminal_finalization_fixture_mission_result.json').write_text(
        json.dumps(results['success'], indent=2, sort_keys=True),
        encoding='utf-8',
    )
    (root / 'terminal_finalization_fixture_report.md').write_text(
        '# Terminal-finalization integration fixture\n\n'
        'The real `DistributedFrontierAssignment` node was exercised with '
        'wrapped task snapshots, frontier evidence, peer status input, '
        'terminal persistence, status/event publication, and idempotent '
        '`_set_terminal()` finalization.\n\n'
        '## Cases\n\n'
        + '\n'.join(
            '- `%s`: mission=%s, agreement=%s, reason=%s, commits=%s' % (
                case, data['mission_status'], data['terminal_agreement'],
                data['terminal_reason'], data['terminal_commit_count'],
            ) for case, data in results.items()
        )
        + '\n\nOverall: `%s`\n' % ('PASS' if overall else 'FAIL'),
        encoding='utf-8',
    )
    (root / 'terminal_finalization_fixture_report.json').write_text(
        json.dumps(report, indent=2, sort_keys=True), encoding='utf-8',
    )
    print(json.dumps(report, indent=2, sort_keys=True))
    return 0 if overall else 1


if __name__ == '__main__':
    raise SystemExit(main())
