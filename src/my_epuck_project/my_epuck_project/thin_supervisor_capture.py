"""Minimal in-process Supervisor capture for the thin evidence path.

This module is enabled only by the thin recorder.  It reuses the existing
Webots Supervisor connection owned by ``paced_ros2_supervisor`` so raw GT and
contact evidence do not require a second controller process or a second
physics-step loop.
"""

from __future__ import annotations

import json
import csv
import io
import math
import os
from pathlib import Path
import threading
import time

from .cooperative_ground_truth_observer import (
    _find_named_node,
    _write_runtime_metrics,
)


class _BufferedCsvRows:
    """Small list-row sink for the hot GT/contact capture path.

    The legacy sink is intentionally dict-oriented for its broad diagnostic
    observer.  Thin capture already has fixed schemas, so avoiding a dict and
    field-name lookup for each 20 ms row preserves the exact CSV contract with
    less Python work in the Supervisor callback.
    """

    def __init__(self, stream, fieldnames, max_buffer_bytes=256 * 1024,
                 max_buffer_rows=1024):
        self.stream = stream
        self.max_buffer_bytes = int(max_buffer_bytes)
        # Keep numeric rows as Python values in the physics callback.  CSV
        # formatting is deferred to bounded chunk flushes, so the Supervisor
        # step does not repeatedly allocate StringIO/csv output for every
        # 20 ms sample while preserving the existing on-disk schema.
        self.max_buffer_rows = max(1, int(max_buffer_rows))
        self.fieldnames = tuple(fieldnames)
        header = io.StringIO()
        csv.writer(header, lineterminator='\n').writerow(self.fieldnames)
        self.stream.write(header.getvalue())
        self.rows_buffer = []
        self.rows = 0
        self.flushes = 0
        self.max_buffer_bytes_seen = 0

    def writerow(self, row):
        self.rows_buffer.append(tuple(row))
        self.rows += 1
        if len(self.rows_buffer) >= self.max_buffer_rows:
            self.flush()

    def flush(self):
        if not self.rows_buffer:
            self.stream.flush()
            self.flushes += 1
            return
        buffer = io.StringIO()
        csv.writer(buffer, lineterminator='\n').writerows(self.rows_buffer)
        payload = buffer.getvalue()
        self.max_buffer_bytes_seen = max(self.max_buffer_bytes_seen,
                                         len(payload))
        if payload:
            self.stream.write(payload)
        self.rows_buffer.clear()
        self.stream.flush()
        self.flushes += 1

    def close(self):
        self.flush()
        self.stream.close()


class ThinSupervisorCapture:
    """Capture the same GT/contact schema using an existing Supervisor."""

    def __init__(self, supervisor, robots, output, ready_file, runtime_directory,
                 sample_period_s=0.02, contact_output='',
                 contact_sampling_period_ms=20):
        self.supervisor = supervisor
        self.robots = tuple(str(robot) for robot in robots)
        self.nodes = {}
        for name in self.robots:
            node = _find_named_node(supervisor, name)
            if node is None:
                raise RuntimeError(f'SUPERVISOR_ROBOT_NOT_FOUND name={name}')
            self.nodes[name] = node
        self.output = os.path.abspath(str(output))
        self.ready_file = os.path.abspath(str(ready_file))
        self.runtime_directory = os.path.abspath(str(runtime_directory))
        self.sample_period_s = max(float(sample_period_s), 0.001)
        self.contact_output = (os.path.abspath(str(contact_output))
                               if contact_output else '')
        self.contact_period_ms = max(1, int(contact_sampling_period_ms))
        self.timestep_ms = max(1, int(supervisor.getBasicTimeStep()))
        self.next_pose_sample_s = supervisor.getTime()
        self.start_sim = supervisor.getTime()
        self.last_sim = self.start_sim
        self.start_wall = __import__('time').monotonic()
        self.step_calls = 0
        self.sample_rows = 0
        self.contact_queries = 0
        self.contact_rows = 0
        self.shutdown_request = os.path.join(
            self.runtime_directory, 'shutdown.requested')
        self._close_lock = threading.Lock()
        self._closed = False
        self.gt_stream = None
        self.contact_stream = None
        self.gt_sink = None
        self.contact_sink = None
        Path(self.output).parent.mkdir(parents=True, exist_ok=True)
        Path(self.runtime_directory).mkdir(parents=True, exist_ok=True)
        self.gt_stream = open(self.output, 'w', newline='', encoding='utf-8')
        self.gt_sink = _BufferedCsvRows(self.gt_stream, [
            'sim_time_s', 'robot_id', 'world_x_m', 'world_y_m', 'world_z_m',
            'heading_x', 'heading_y', 'heading_z', 'planar_yaw_rad',
            'linear_velocity_x_mps', 'linear_velocity_y_mps',
            'linear_velocity_z_mps', 'ground_speed_mps',
            'angular_velocity_x_rps', 'angular_velocity_y_rps',
            'angular_velocity_z_rps',
        ])
        if self.contact_output:
            Path(self.contact_output).parent.mkdir(parents=True, exist_ok=True)
            self.contact_stream = open(self.contact_output, 'w',
                                       newline='', encoding='utf-8')
            self.contact_sink = _BufferedCsvRows(self.contact_stream, [
                'sim_time_s', 'robot_id', 'contact_count', 'point_x_m',
                'point_y_m', 'point_z_m', 'contacted_node_id',
                'contacted_node_def', 'contacted_node_name',
            ])
        Path(self.ready_file).parent.mkdir(parents=True, exist_ok=True)
        Path(self.ready_file).write_text(json.dumps({
            'sim_time_s': self.start_sim,
            'robot_defs': sorted(self.nodes),
            'basic_time_step_ms': self.timestep_ms,
            'capture_mode': 'paced_ros2_supervisor_in_process',
        }, sort_keys=True), encoding='utf-8')
        self._shutdown_monitor = threading.Thread(
            target=self._monitor_shutdown_request,
            name='thin-supervisor-capture-shutdown', daemon=True)
        self._shutdown_monitor.start()

    def _monitor_shutdown_request(self):
        while not self._closed:
            if os.path.isfile(self.shutdown_request):
                self.close(None, 'shutdown_requested')
                return
            time.sleep(0.02)

    def capture(self, now):
        # Shutdown is requested by a separate thread while Webots can still
        # deliver one final physics callback.  Keep the close/capture critical
        # section together so a sink cannot be closed or cleared between the
        # liveness check and the last row write.
        with self._close_lock:
            if self._closed:
                return
            self.step_calls += 1
            self.last_sim = float(now)
            if now + 1.0e-9 >= self.next_pose_sample_s:
                for robot_id, node in self.nodes.items():
                    position = node.getPosition()
                    orientation = node.getOrientation()
                    velocity = node.getVelocity()
                    heading_x, heading_y, heading_z = (
                        orientation[0], orientation[3], orientation[6])
                    self.gt_sink.writerow([
                        now, robot_id, position[0], position[1], position[2],
                        heading_x, heading_y, heading_z,
                        math.atan2(heading_y, heading_x), velocity[0],
                        velocity[1], velocity[2], math.hypot(velocity[0], velocity[1]),
                        velocity[3], velocity[4], velocity[5],
                    ])
                self.sample_rows += 1
                while self.next_pose_sample_s <= now + 1.0e-9:
                    self.next_pose_sample_s += self.sample_period_s
            if self.contact_sink is None:
                return
            self.contact_queries += len(self.nodes)
            for robot_id, node in self.nodes.items():
                try:
                    contacts = node.getContactPoints(True)
                except Exception:
                    contacts = []
                if not contacts:
                    self.contact_sink.writerow([
                        now, robot_id, 0, '', '', '', '', '', '',
                    ])
                    self.contact_rows += 1
                    continue
                for contact in contacts:
                    point = contact.getPoint()
                    self.contact_sink.writerow([
                        now, robot_id, len(contacts), point[0], point[1],
                        point[2], contact.getNodeId(), '', '',
                    ])
                    self.contact_rows += 1

    def close(self, end_sim=None, reason='shutdown'):
        with self._close_lock:
            if self._closed or self.gt_sink is None:
                return
            self._closed = True
            gt_sink = self.gt_sink
            contact_sink = self.contact_sink
            end_sim = self.last_sim if end_sim is None else float(end_sim)
            end_wall = time.monotonic()
            gt_sink.close()
            if contact_sink is not None:
                contact_sink.close()
        _write_runtime_metrics(os.path.join(self.runtime_directory,
                                            'runtime_metrics.json'), {
            'observer': 'thin_supervisor_capture',
            'capture_mode': 'paced_ros2_supervisor_in_process',
            'basic_time_step_ms': self.timestep_ms,
            'requested_sample_period_s': self.sample_period_s,
            'contact_sampling_period_ms': (
                self.contact_period_ms if self.contact_sink is not None else None),
            'effective_step_period_ms': self.timestep_ms,
            'step_calls': self.step_calls,
            'sample_rows': self.sample_rows,
            'pose_sample_period_s': self.sample_period_s,
            'contact_queries': self.contact_queries,
            'contact_rows': self.contact_rows,
            'gt_buffer_flushes': gt_sink.flushes,
            'contact_buffer_flushes': (
                contact_sink.flushes if contact_sink is not None else 0),
            'gt_max_buffer_bytes': gt_sink.max_buffer_bytes_seen,
            'contact_max_buffer_bytes': (
                contact_sink.max_buffer_bytes_seen
                if contact_sink is not None else 0),
            'sim_start_s': self.start_sim,
            'sim_end_s': end_sim,
            'sim_elapsed_s': max(0.0, end_sim - self.start_sim),
            'monotonic_wall_start_s': self.start_wall,
            'monotonic_wall_end_s': end_wall,
            'monotonic_wall_elapsed_s': max(0.0, end_wall - self.start_wall),
            'simulation_seconds_per_wall_second': (
                max(0.0, end_sim - self.start_sim) /
                max(1.0e-9, end_wall - self.start_wall)),
            'exit_reason': reason,
            'finalized': True,
            'output': self.output,
        })
        self.gt_sink = None
        self.contact_sink = None
        self.gt_stream = None
        self.contact_stream = None
