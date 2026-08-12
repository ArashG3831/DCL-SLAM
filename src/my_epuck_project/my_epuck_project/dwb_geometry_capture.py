"""Pure bounded helpers for the first-frontier DWB geometry capture.

This module deliberately contains no ROS publishers or command-path logic.  It
only defines the trigger/window policy and lossless serializers used by the
existing diagnostic node.
"""

from __future__ import annotations

from dataclasses import dataclass, field
import math


@dataclass
class GeometryCapturePolicy:
    """Bound one first-frontier capture around an existing stall trigger."""

    pre_window_s: float = 2.0
    post_zero_s: float = 5.0
    hard_window_s: float = 30.0
    target_seen: bool = False
    target_goal_key: object = None
    triggered: bool = False
    trigger_sim_s: float | None = None
    zero_started_sim_s: float | None = None
    closed: bool = False
    trigger_reason: str | None = None
    events: list = field(default_factory=list)

    def observe_trigger(self, now_s: float, goal_key, goal_kind: str,
                        reason: str) -> bool:
        """Activate only on the first frontier goal's existing stall event."""
        if self.triggered or self.target_seen or goal_kind != 'frontier':
            return False
        self.target_seen = True
        self.target_goal_key = goal_key
        if reason != 'ANGULAR_ONLY_OVER_2S':
            return False
        self.triggered = True
        self.trigger_sim_s = float(now_s)
        self.trigger_reason = reason
        self.events.append({'event': 'GEOMETRY_CAPTURE_TRIGGER',
                            'sim_time_s': float(now_s), 'reason': reason})
        return True

    def observe_state(self, now_s: float, goal_key, command_kind: str,
                      recovery_count: int = 0, goal_active: bool = True) -> str:
        """Advance the bounded window and return ACTIVE/END/INACTIVE."""
        if not self.triggered or self.closed:
            return 'INACTIVE'
        if goal_key != self.target_goal_key:
            self.closed = True
            return 'END'
        now_s = float(now_s)
        if command_kind == 'ZERO' and self.zero_started_sim_s is None:
            self.zero_started_sim_s = now_s
            self.events.append({'event': 'ANGULAR_ONLY_TO_ZERO',
                                'sim_time_s': now_s})
        if recovery_count > 0:
            self.events.append({'event': 'RECOVERY_OBSERVED',
                                'sim_time_s': now_s,
                                'recovery_count': int(recovery_count)})
            self.closed = True
            return 'END'
        if not goal_active:
            self.closed = True
            return 'END'
        if self.trigger_sim_s is not None and now_s - self.trigger_sim_s >= self.hard_window_s:
            self.closed = True
            return 'END'
        if (self.zero_started_sim_s is not None and
                now_s - self.zero_started_sim_s >= self.post_zero_s):
            self.closed = True
            return 'END'
        return 'ACTIVE'

    def window_start(self) -> float | None:
        if self.trigger_sim_s is None:
            return None
        return self.trigger_sim_s - self.pre_window_s


def _stamp_seconds(stamp) -> float:
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def serialize_path_message(message, geometry_revision: int,
                           source: str) -> dict:
    """Serialize every NavPath point and orientation in message order."""
    if message is None:
        return {'source': source, 'geometry_revision': geometry_revision,
                'available': False}
    points = []
    for index, pose in enumerate(message.poses):
        q = pose.pose.orientation
        yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        points.append({
            'index': index,
            'x_m': float(pose.pose.position.x),
            'y_m': float(pose.pose.position.y),
            'yaw_rad': float(yaw),
        })
    length = sum(math.hypot(right['x_m'] - left['x_m'],
                            right['y_m'] - left['y_m'])
                 for left, right in zip(points, points[1:]))
    return {
        'source': source,
        'geometry_revision': int(geometry_revision),
        'available': True,
        'frame_id': message.header.frame_id,
        'stamp_s': _stamp_seconds(message.header.stamp),
        'point_count': len(points),
        'length_m': float(length),
        'points': points,
        'source_indices_available': False,
    }


def serialize_costmap_message(message, reason: str, robot_pose=None) -> dict:
    """Serialize one bounded-by-event raw OccupancyGrid snapshot."""
    if message is None:
        return {'available': False, 'reason': 'NO_MESSAGE', 'capture_reason': reason}
    origin = message.info.origin
    q = origin.orientation
    origin_yaw = math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z))
    return {
        'available': True,
        'capture_reason': reason,
        'frame_id': message.header.frame_id,
        'stamp_s': _stamp_seconds(message.header.stamp),
        'width': int(message.info.width), 'height': int(message.info.height),
        'resolution_m': float(message.info.resolution),
        'origin': {'x_m': float(origin.position.x),
                   'y_m': float(origin.position.y),
                   'yaw_rad': float(origin_yaw)},
        'robot_pose_in_costmap_frame': robot_pose,
        'data': [int(value) for value in message.data],
    }


def trajectory_sequence_record(score, index: int, role: str) -> dict:
    """Serialize all intermediate poses/time offsets for one candidate."""
    traj = score.traj
    poses = []
    offsets = list(getattr(traj, 'time_offsets', []))
    for pose_index, pose in enumerate(traj.poses):
        offset = offsets[pose_index] if pose_index < len(offsets) else None
        poses.append({
            'index': pose_index,
            'x_m': float(pose.x), 'y_m': float(pose.y),
            'yaw_rad': float(pose.theta),
            'relative_time_s': None if offset is None else _stamp_seconds(offset),
        })
    return {
        'trajectory_index': int(index), 'role': role,
        'velocity': {'vx_mps': float(traj.velocity.x),
                     'vy_mps': float(traj.velocity.y),
                     'wz_radps': float(traj.velocity.theta)},
        'total_score': float(score.total),
        'valid': math.isfinite(float(score.total)) and float(score.total) >= 0.0,
        'poses': poses,
    }


def select_sequence_indices(message, forward_threshold: float = 0.026,
                             additional_forward: int = 3) -> list[tuple[int, str]]:
    """Select bounded, deterministic candidates while preserving array order."""
    scores = list(message.twists)
    selected = int(message.best_index)
    valid = [(index, score) for index, score in enumerate(scores)
             if math.isfinite(float(score.total)) and float(score.total) >= 0.0]
    forward = [(index, score) for index, score in valid
               if float(score.traj.velocity.x) >= forward_threshold]
    angular = [(index, score) for index, score in valid
               if abs(float(score.traj.velocity.x)) < 1e-9 and
               abs(float(score.traj.velocity.theta)) > 1e-9]
    zero = [(index, score) for index, score in valid
            if abs(float(score.traj.velocity.x)) < 1e-9 and
            abs(float(score.traj.velocity.theta)) < 1e-9]
    selected_roles = []
    seen = set()
    def add(index, role):
        if index in seen:
            return
        seen.add(index)
        selected_roles.append((index, role))
    add(selected, 'selected')
    for rank, (index, _score) in enumerate(
            sorted(forward, key=lambda item: (float(item[1].total), item[0]))):
        if rank >= additional_forward + 1:
            break
        add(index, 'best_forward' if rank == 0 else f'additional_forward_{rank}')
    if angular:
        add(min(angular, key=lambda item: (float(item[1].total), item[0]))[0],
            'lowest_score_angular_only')
    if zero:
        add(min(zero, key=lambda item: (float(item[1].total), item[0]))[0],
            'lowest_score_zero')
    return selected_roles
