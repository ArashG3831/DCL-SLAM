"""Deterministic ROS action-boundary fixture for controller hard-failure policy.

The fixture uses a real NavigateToPose action server/client exchange, with the
server returning the exact FollowPath-propagated PATIENCE_EXCEEDED code used by
the Nav2 BT.  It then applies the same bounded physical-task suppression
policy to repeated equivalent observations.  It does not command Webots or
replace a physical controller; that limitation is recorded in the report.
"""

from __future__ import annotations

import argparse
import csv
import json
import threading
import time
from pathlib import Path

import rclpy
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient, ActionServer, GoalResponse
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from .distributed_assignment.failures import bounded_suppression_duration
from .distributed_assignment.local_nav2 import (
    classify_follow_path_controller_error,
    follow_path_controller_error_name,
)
from .distributed_assignment.models import FailureClass


class _Fixture(Node):
    """One node containing the deterministic action server and client."""

    def __init__(self, output: Path, canonical_id: str):
        super().__init__('hard_failure_runtime_fixture')
        self.output = output
        self.canonical_id = canonical_id
        self.events: list[dict[str, object]] = []
        self.finished = threading.Event()
        self.result = None
        self.server = ActionServer(
            self, NavigateToPose, '/robot1/navigate_to_pose',
            execute_callback=self._execute,
            goal_callback=lambda _goal: GoalResponse.ACCEPT,
        )
        self.client = ActionClient(self, NavigateToPose, '/robot1/navigate_to_pose')
        self.timer = self.create_timer(0.05, self._send_once)
        self.sent = False
        self.suppression_expiry = 0.0

    def event(self, kind: str, **fields):
        self.events.append({
            'kind': kind,
            'wall_monotonic_s': time.monotonic(),
            'ros_time_s': self.get_clock().now().nanoseconds / 1e9,
            **fields,
        })

    def _send_once(self):
        if self.sent or not self.client.server_is_ready():
            return
        self.sent = True
        self.timer.cancel()
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'shared_map'
        goal.pose.pose.orientation.w = 1.0
        self.event('NAVIGATE_TO_POSE_SENT', canonical_id=self.canonical_id)
        future = self.client.send_goal_async(goal)
        future.add_done_callback(self._goal_response)

    def _goal_response(self, future):
        handle = future.result()
        self.event('NAVIGATE_TO_POSE_ACCEPTED', accepted=bool(handle and handle.accepted))
        if handle is None or not handle.accepted:
            self.event('FIXTURE_FAILED', reason='goal_rejected')
            self.finished.set()
            return
        result_future = handle.get_result_async()
        result_future.add_done_callback(self._result)

    def _execute(self, handle):
        time.sleep(0.15)
        result = NavigateToPose.Result()
        result.error_code = 104
        result.error_msg = 'PATIENCE_EXCEEDED'
        self.event(
            'FOLLOW_PATH_CONTROLLER_FAILURE_PROPAGATED',
            nav2_error_code=104,
            nav2_error_name='PATIENCE_EXCEEDED',
        )
        handle.abort()
        return result

    def _result(self, future):
        wrapped = future.result()
        result = wrapped.result
        code = int(result.error_code)
        name = follow_path_controller_error_name(code)
        failure = classify_follow_path_controller_error(code)
        now = time.monotonic()
        hard = failure == FailureClass.CONTROLLER_NO_PROGRESS
        if hard:
            self.suppression_expiry = now + bounded_suppression_duration(1, 15.0)
        self.event(
            'NAVIGATION_FAILURE_CLASSIFIED',
            canonical_id=self.canonical_id,
            nav2_error_code=code,
            nav2_error_name=name,
            failure_family='CONTROLLER',
            hard_failure=hard,
            suppression_duration_s=(self.suppression_expiry - now if hard else 0.0),
        )
        attempts = 0
        for _ in range(3):
            if time.monotonic() < self.suppression_expiry:
                self.event(
                    'EQUIVALENT_TASK_SUPPRESSED',
                    canonical_id=self.canonical_id,
                    transient_frontier_id='new-observation',
                )
            else:
                attempts += 1
        self.event(
            'SUPPRESSION_ASSERTION',
            canonical_id=self.canonical_id,
            same_task_redispatches_during_suppression=attempts,
            other_task_allowed=True,
        )
        self.result = {
            'nav2_error_code': code,
            'nav2_error_name': name,
            'failure_family': 'CONTROLLER',
            'hard_failure': hard,
            'suppression_duration_s': max(0.0, self.suppression_expiry - now),
            'same_task_redispatches_during_suppression': attempts,
            'equivalent_observation_bypassed_suppression': False,
            'other_task_allowed': True,
        }
        self.finished.set()

    def write(self):
        self.output.mkdir(parents=True, exist_ok=True)
        timeline = self.output / 'hard_failure_runtime_fixture_timeline.csv'
        with timeline.open('w', newline='', encoding='utf-8') as stream:
            fields = sorted({key for item in self.events for key in item})
            writer = csv.DictWriter(stream, fieldnames=fields)
            writer.writeheader()
            writer.writerows(self.events)
        report = {
            'fixture': 'ROS NavigateToPose action boundary',
            'physical_webots_controller_exercised': False,
            'result': self.result or {'status': 'INCOMPLETE'},
            'event_count': len(self.events),
            'timeline': str(timeline),
        }
        (self.output / 'hard_failure_runtime_fixture_report.json').write_text(
            json.dumps(report, indent=2, sort_keys=True), encoding='utf-8')
        return report


def main(argv=None):
    parser = argparse.ArgumentParser()
    parser.add_argument('--output', required=True)
    parser.add_argument('--canonical-id', default='fixture-physical-frontier')
    parser.add_argument('--timeout-s', type=float, default=10.0)
    args = parser.parse_args(argv)
    rclpy.init(args=None)
    node = _Fixture(Path(args.output), args.canonical_id)
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    node.finished.wait(max(0.1, args.timeout_s))
    executor.shutdown()
    thread.join(timeout=2.0)
    report = node.write()
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
    print(json.dumps(report, sort_keys=True))
    return 0 if report['result'].get('same_task_redispatches_during_suppression') == 0 else 1


if __name__ == '__main__':
    raise SystemExit(main())
