#!/usr/bin/env python3
"""Bounded CycloneDDS entity-lifecycle stress probe.

This intentionally exercises publisher/subscriber/service/client creation and
destruction in short-lived ROS processes, without Webots.  It has a fixed
iteration/concurrency/wall-time bound and records every child exit.
"""
from __future__ import annotations

import argparse
import datetime as dt
import json
import os
import pathlib
import signal
import subprocess
import sys
import time


def child_main(index: int, lifetime: float) -> int:
    import rclpy
    from example_interfaces.srv import AddTwoInts
    from rclpy.node import Node
    from std_msgs.msg import String

    rclpy.init(args=None)
    node = Node(f'cyclone_churn_{index}')
    topic = f'/cyclone_churn/node_{index}'
    publisher = node.create_publisher(String, topic, 10)
    node.create_subscription(String, topic, lambda _: None, 10)
    node.create_service(AddTwoInts, f'{topic}/service',
                        lambda request, response: response)
    node.create_client(AddTwoInts, f'{topic}/client')
    message = String()
    message.data = str(index)
    deadline = time.monotonic() + lifetime
    while time.monotonic() < deadline and rclpy.ok():
        publisher.publish(message)
        rclpy.spin_once(node, timeout_sec=0.02)
    node.destroy_node()
    rclpy.shutdown()
    return 0


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument('--iterations', type=int, default=20)
    parser.add_argument('--concurrency', type=int, default=4)
    parser.add_argument('--child-lifetime', type=float, default=0.25)
    parser.add_argument('--wall-timeout', type=float, default=120.0)
    parser.add_argument('--output', type=pathlib.Path)
    parser.add_argument('--child', type=int)
    args = parser.parse_args()
    if args.child is not None:
        return child_main(args.child, args.child_lifetime)
    if args.output is None:
        raise SystemExit('--output is required for the parent probe')
    if args.iterations < 1 or args.concurrency < 1:
        raise SystemExit('iterations and concurrency must be positive')
    args.output.parent.mkdir(parents=True, exist_ok=True)
    started = time.monotonic()
    records: list[dict] = []
    next_child = 0
    active: dict[subprocess.Popen[str], int] = {}
    aborted = False
    while (next_child < args.iterations or active) and \
            time.monotonic() - started < args.wall_timeout:
        while next_child < args.iterations and len(active) < args.concurrency:
            child_index = next_child
            next_child += 1
            process = subprocess.Popen(
                [sys.executable, __file__, '--child', str(child_index),
                 '--child-lifetime', str(args.child_lifetime)],
                stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                text=True, start_new_session=True)
            active[process] = child_index
        for process, child_index in list(active.items()):
            returncode = process.poll()
            if returncode is None:
                continue
            output = process.communicate(timeout=2.0)[0]
            records.append({
                'utc': dt.datetime.now(dt.timezone.utc).isoformat(),
                'child': child_index,
                'returncode': returncode,
                'signal': -returncode if returncode < 0 else None,
                'output': output[-4000:],
            })
            if returncode != 0 or 'ddsi_entity_index' in output or \
                    'Assertion' in output:
                aborted = True
            del active[process]
        time.sleep(0.02)
    timed_out = bool(active)
    for process, child_index in list(active.items()):
        try:
            os.killpg(process.pid, signal.SIGTERM)
        except ProcessLookupError:
            pass
        output = process.communicate(timeout=5.0)[0]
        records.append({
            'utc': dt.datetime.now(dt.timezone.utc).isoformat(),
            'child': child_index,
            'returncode': process.returncode,
            'signal': -process.returncode if process.returncode and process.returncode < 0 else None,
            'output': output[-4000:],
            'forced_timeout_stop': True,
        })
        del active[process]
    summary = {
        'started_utc': dt.datetime.fromtimestamp(
            time.time() - (time.monotonic() - started), dt.timezone.utc).isoformat(),
        'finished_utc': dt.datetime.now(dt.timezone.utc).isoformat(),
        'iterations_requested': args.iterations,
        'children_recorded': len(records),
        'concurrency': args.concurrency,
        'wall_timeout_s': args.wall_timeout,
        'timed_out': timed_out,
        'aborted_child_or_assertion': aborted,
        'rmw_implementation': os.environ.get('RMW_IMPLEMENTATION', ''),
        'cyclonedds_uri': os.environ.get('CYCLONEDDS_URI', ''),
        'records': records,
    }
    args.output.write_text(json.dumps(summary, indent=2), encoding='utf-8')
    print(json.dumps({k: summary[k] for k in summary if k != 'records'}, sort_keys=True))
    return 1 if timed_out or aborted or len(records) != args.iterations else 0


if __name__ == '__main__':
    raise SystemExit(main())
