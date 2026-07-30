"""Passive final-state and lossless occupancy-map collector."""

import json
import math
import os
from pathlib import Path
import signal
import threading
import time

from action_msgs.msg import GoalStatus, GoalStatusArray
from my_epuck_interfaces.msg import ExplorationClaim, ExplorationStatus
from nav_msgs.msg import OccupancyGrid
import numpy as np
import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)


SCHEMA_VERSION = '1.0.0'
CLAIM_STATES = {
    0: 'UNKNOWN', 1: 'PROPOSING', 2: 'NAVIGATING', 3: 'SUCCEEDED',
    4: 'FAILED', 5: 'RELEASED', 6: 'CANCELED',
}
STATUS_STATES = {
    0: 'STARTING', 1: 'ACTIVE', 2: 'NAVIGATING',
    3: 'NO_ELIGIBLE_CANDIDATES', 4: 'MISSION_COMPLETE',
    5: 'STOPPED', 6: 'ERROR',
}
ACTIVE_GOAL_STATES = {
    GoalStatus.STATUS_ACCEPTED,
    GoalStatus.STATUS_EXECUTING,
    GoalStatus.STATUS_CANCELING,
}


def atomic_json(path, value):
    """Write strict JSON atomically."""
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(f'.{path.name}.{os.getpid()}.tmp')
    with temporary.open('w', encoding='utf-8') as stream:
        json.dump(value, stream, indent=2, sort_keys=True, allow_nan=False)
        stream.flush()
        os.fsync(stream.fileno())
    os.replace(temporary, path)


def duration_dict(value):
    return {'sec': int(value.sec), 'nanosec': int(value.nanosec)}


def stamp_dict(value):
    return {'sec': int(value.sec), 'nanosec': int(value.nanosec)}


def point_dict(value):
    return {'x': float(value.x), 'y': float(value.y), 'z': float(value.z)}


def quaternion_dict(value):
    return {
        'x': float(value.x), 'y': float(value.y),
        'z': float(value.z), 'w': float(value.w),
    }


def pose_stamped_dict(value):
    return {
        'header': {
            'frame_id': value.header.frame_id,
            'stamp': stamp_dict(value.header.stamp),
        },
        'pose': {
            'position': point_dict(value.pose.position),
            'orientation': quaternion_dict(value.pose.orientation),
        },
    }


def status_dict(message, received_monotonic, now_monotonic=None):
    now = time.monotonic() if now_monotonic is None else now_monotonic
    return {
        'source_robot_id': message.source_robot_id,
        'source_session_id': bytes(message.source_session_id.uuid).hex(),
        'message_revision': int(message.message_revision),
        'state': STATUS_STATES.get(message.state, str(message.state)),
        'state_value': int(message.state),
        'reason': message.reason,
        'candidate_count': int(message.candidate_count),
        'eligible_candidate_count': int(message.eligible_candidate_count),
        'active_claim_id': int(message.active_claim_id),
        'active_frontier_id': int(message.active_frontier_id),
        'local_map_revision': int(message.local_map_revision),
        'shared_map_checksum': int(message.shared_map_checksum),
        'status_ttl': duration_dict(message.status_ttl),
        'ros_timestamp': stamp_dict(message.header.stamp),
        'frame_id': message.header.frame_id,
        'collector_receive_monotonic_s': received_monotonic,
        'age_at_write_s': max(0.0, now - received_monotonic),
    }


def claim_dict(message, received_monotonic, now_monotonic=None):
    now = time.monotonic() if now_monotonic is None else now_monotonic
    state = CLAIM_STATES.get(message.state, str(message.state))
    return {
        'source_robot_id': message.source_robot_id,
        'source_session_id': bytes(message.source_session_id.uuid).hex(),
        'message_revision': int(message.message_revision),
        'claim_id': int(message.claim_id),
        'state': state,
        'state_value': int(message.state),
        'state_reason': message.state_reason,
        'frontier_id': int(message.frontier_id),
        'map_revision': int(message.map_revision),
        'frontier_centroid': point_dict(message.frontier_centroid),
        'bounding_box_min': point_dict(message.bounding_box_min),
        'bounding_box_max': point_dict(message.bounding_box_max),
        'approach_pose': pose_stamped_dict(message.approach_pose),
        'path_length_m': float(message.path_length_m),
        'information_gain': float(message.information_gain),
        'utility_score': float(message.utility_score),
        'claim_ttl': duration_dict(message.claim_ttl),
        'ros_timestamp': stamp_dict(message.header.stamp),
        'frame_id': message.header.frame_id,
        'collector_receive_monotonic_s': received_monotonic,
        'age_at_write_s': max(0.0, now - received_monotonic),
        'reserving': state in ('PROPOSING', 'NAVIGATING'),
        'terminal': state in ('SUCCEEDED', 'FAILED', 'RELEASED', 'CANCELED'),
    }


def validate_map_message(message, expected_frame='shared_map'):
    """Return a list of lossless-map validation errors."""
    errors = []
    info = message.info
    values = np.asarray(message.data)
    if len(values) != int(info.width) * int(info.height):
        errors.append('data_length_mismatch')
    if not math.isfinite(info.resolution) or info.resolution <= 0.0:
        errors.append('invalid_resolution')
    origin = info.origin
    components = [
        origin.position.x, origin.position.y, origin.position.z,
        origin.orientation.x, origin.orientation.y,
        origin.orientation.z, origin.orientation.w,
    ]
    if not all(math.isfinite(component) for component in components):
        errors.append('nonfinite_origin')
    norm = math.sqrt(sum(component * component for component in (
        origin.orientation.x, origin.orientation.y,
        origin.orientation.z, origin.orientation.w,
    )))
    if not math.isfinite(norm) or abs(norm - 1.0) > 1e-3:
        errors.append('invalid_origin_quaternion')
    if values.size and (values.min() < -1 or values.max() > 100):
        errors.append('occupancy_out_of_range')
    if message.header.frame_id != expected_frame:
        errors.append('unexpected_frame')
    return errors


def map_metadata(message, topic, run_id, robot_id, received_monotonic):
    origin = message.info.origin
    return {
        'schema_version': SCHEMA_VERSION,
        'run_id': run_id,
        'robot_id': robot_id,
        'source_topic': topic,
        'frame_id': message.header.frame_id,
        'map_load_timestamp': stamp_dict(message.info.map_load_time),
        'message_header_timestamp': stamp_dict(message.header.stamp),
        'collector_receive_monotonic_s': received_monotonic,
        'resolution': float(message.info.resolution),
        'width': int(message.info.width),
        'height': int(message.info.height),
        'origin': {
            'position': point_dict(origin.position),
            'orientation': quaternion_dict(origin.orientation),
        },
        'dtype': 'int8',
        'validation_errors': validate_map_message(message),
    }


def atomic_save_map(path, message, metadata):
    """Atomically save exact int8 occupancy data and embedded metadata."""
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(f'.{path.name}.{os.getpid()}.tmp')
    data = np.asarray(message.data, dtype=np.int8).reshape(
        int(message.info.height), int(message.info.width))
    with temporary.open('wb') as stream:
        np.savez_compressed(
            stream,
            occupancy=data,
            metadata_json=np.asarray(
                json.dumps(metadata, sort_keys=True, allow_nan=False)),
        )
        stream.flush()
        os.fsync(stream.fileno())
    os.replace(temporary, path)


class CooperativeTrialCollector(Node):
    """Observer which never creates a publisher, service, or action client."""

    def __init__(self):
        super().__init__('cooperative_trial_collector')
        defaults = {
            'output_dir': '',
            'run_id': '',
            'status_fresh_s': 4.0,
            'settling_period_s': 4.0,
        }
        for name, default in defaults.items():
            self.declare_parameter(name, default)
        self.output_dir = Path(self.get_parameter('output_dir').value)
        self.run_id = self.get_parameter('run_id').value
        self.status_fresh_s = float(
            self.get_parameter('status_fresh_s').value)
        self.settling_period_s = float(
            self.get_parameter('settling_period_s').value)
        if not self.output_dir:
            raise ValueError('output_dir is required')
        self.output_dir.mkdir(parents=True, exist_ok=True)
        self.start_monotonic = time.monotonic()
        self.start_ros_seconds = self.get_clock().now().nanoseconds * 1e-9
        self.lock = threading.RLock()
        self.finalized = False
        self.messages = {
            robot: {'status': None, 'claim': None, 'shared_map': None,
                    'local_map': None, 'navigate_status': None}
            for robot in ('robot1', 'robot2')
        }
        self.received = {robot: {} for robot in self.messages}
        self.both_complete_since = None
        self.settled_snapshot = None
        self.cached_graph = []
        self.last_graph_refresh = 0.0
        transient = QoSProfile(
            history=HistoryPolicy.KEEP_LAST, depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        reliable = QoSProfile(
            history=HistoryPolicy.KEEP_LAST, depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        for robot in self.messages:
            self.create_subscription(
                ExplorationStatus, f'/cslam/{robot}/exploration_status',
                lambda message, item=robot:
                    self.store(item, 'status', message), reliable)
            self.create_subscription(
                ExplorationClaim, f'/cslam/{robot}/exploration_claim',
                lambda message, item=robot:
                    self.store(item, 'claim', message), reliable)
            self.create_subscription(
                OccupancyGrid, f'/{robot}/shared_map',
                lambda message, item=robot:
                    self.store(item, 'shared_map', message), transient)
            self.create_subscription(
                OccupancyGrid, f'/{robot}/map',
                lambda message, item=robot:
                    self.store(item, 'local_map', message), transient)
            self.create_subscription(
                GoalStatusArray,
                f'/{robot}/navigate_to_pose/_action/status',
                lambda message, item=robot:
                    self.store(item, 'navigate_status', message), transient)
        # Health/status is a process-control channel.  Keep it wall-timed so
        # a missing or paused simulation clock cannot prevent watchdogs and
        # artifact finalization from running.
        self.create_timer(
            0.5, self.write_health,
            clock=Clock(clock_type=ClockType.STEADY_TIME))
        self.write_health()

    def store(self, robot, kind, message):
        with self.lock:
            if self.finalized:
                return
            self.messages[robot][kind] = message
            self.received[robot][kind] = self.get_clock().now().nanoseconds * 1e-9

    def ros_seconds(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def graph_snapshot(self):
        return sorted({
            f'/{namespace.strip("/")}/{name}'.replace('//', '/')
            for name, namespace in self.get_node_names_and_namespaces()
        })

    def refresh_graph(self, now):
        if now - self.last_graph_refresh >= 5.0:
            self.cached_graph = self.graph_snapshot()
            self.last_graph_refresh = now

    def complete_and_fresh(self, now):
        for robot in self.messages:
            status = self.messages[robot]['status']
            received = self.received[robot].get('status')
            if status is None or received is None:
                return False
            if status.state != ExplorationStatus.COMPLETE:
                return False
            if now - received > self.status_fresh_s:
                return False
        return True

    def readiness(self):
        required = ('status', 'claim', 'shared_map')
        return all(
            all(self.messages[robot][kind] is not None for kind in required)
            for robot in self.messages
        )

    def navigation_active(self, robot):
        message = self.messages[robot]['navigate_status']
        if message is None:
            return None
        return any(status.status in ACTIVE_GOAL_STATES
                   for status in message.status_list)

    def health_document(self):
        now = self.ros_seconds()
        complete = self.complete_and_fresh(now)
        if complete:
            if self.both_complete_since is None:
                self.both_complete_since = now
        else:
            self.both_complete_since = None
        settled = (
            self.both_complete_since is not None
            and now - self.both_complete_since >= self.settling_period_s
        )
        if settled and self.settled_snapshot is None:
            self.settled_snapshot = {
                robot: max(
                    0.0, now - self.received[robot]['status'])
                for robot in self.messages
            }
        return {
            'schema_version': SCHEMA_VERSION,
            'run_id': self.run_id,
            'collector_pid': os.getpid(),
            'ready': self.readiness(),
            'both_mission_complete': complete,
            'settled': settled,
            'settled_status_age_s': self.settled_snapshot,
            'elapsed_s': now - self.start_ros_seconds,
            'wall_elapsed_s': time.monotonic() - self.start_monotonic,
            'sim_time_seconds': now,
            'received': {
                robot: sorted(self.received[robot]) for robot in self.messages
            },
            'nodes': self.cached_graph,
            'finalized': self.finalized,
        }

    def write_health(self):
        with self.lock:
            if not self.finalized:
                self.refresh_graph(time.monotonic())
                atomic_json(
                    self.output_dir / 'collector_status.json',
                    self.health_document(),
                )

    def final_document(self, reason):
        now = self.ros_seconds()
        robots = {}
        for robot, values in self.messages.items():
            status = values['status']
            claim = values['claim']
            robots[robot] = {
                'status': (
                    status_dict(status, self.received[robot]['status'], now)
                    if status is not None else None
                ),
                'claim': (
                    claim_dict(claim, self.received[robot]['claim'], now)
                    if claim is not None else None
                ),
                'navigation_active': self.navigation_active(robot),
            }
            if self.settled_snapshot is not None:
                robots[robot]['status']['age_at_collection_s'] = (
                    self.settled_snapshot[robot])
        return {
            'schema_version': SCHEMA_VERSION,
            'run_id': self.run_id,
            'finalization_reason': reason,
            'collector_pid': os.getpid(),
            'collector_elapsed_s': now - self.start_ros_seconds,
            'collector_wall_elapsed_s': time.monotonic() - self.start_monotonic,
            'robots': robots,
            'graph_nodes': self.cached_graph,
        }

    def finalize(self, reason='shutdown'):
        with self.lock:
            if self.finalized:
                return False
            document = self.final_document(reason)
            for robot, values in self.messages.items():
                message = values['shared_map']
                if message is None:
                    continue
                topic = f'/{robot}/shared_map'
                metadata = map_metadata(
                    message, topic, self.run_id, robot,
                    self.received[robot]['shared_map'])
                atomic_save_map(
                    self.output_dir / f'{robot}_final_shared_map.npz',
                    message, metadata)
                atomic_json(
                    self.output_dir
                    / f'{robot}_final_shared_map_metadata.json',
                    metadata)
            atomic_json(self.output_dir / 'final_state.json', document)
            self.finalized = True
            status = self.health_document()
            status['finalized'] = True
            atomic_json(self.output_dir / 'collector_status.json', status)
            return True


def main(args=None):
    rclpy.init(args=args)
    node = CooperativeTrialCollector()
    stopping = threading.Event()

    def stop(signum, frame):
        del signum, frame
        stopping.set()

    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)
    try:
        while rclpy.ok() and not stopping.is_set():
            rclpy.spin_once(node, timeout_sec=0.2)
    except ExternalShutdownException:
        stopping.set()
    finally:
        node.finalize('signal' if stopping.is_set() else 'rclpy_shutdown')
        if node.context.ok():
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
