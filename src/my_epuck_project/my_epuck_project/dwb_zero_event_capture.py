"""Bounded helpers for a genuine DWB-selected-zero capture.

The policy is deliberately independent of downstream ``cmd_vel`` topics: the
selected velocity comes from LocalPlanEvaluation.  The ROS node owns the
ring-buffer and artifact streams; this module keeps trigger semantics and
lossless bounded serializers testable without ROS.
"""

from __future__ import annotations

from dataclasses import dataclass, field
import math

from .dwb_geometry_capture import serialize_path_message, trajectory_sequence_record


@dataclass
class ZeroEventCapturePolicy:
    pre_window_s: float = 5.0
    post_zero_s: float = 8.0
    hard_window_s: float = 30.0
    zero_tolerance: float = 1e-6
    evaluation_freshness_s: float = 0.25
    triggered: bool = False
    trigger_robot: str | None = None
    trigger_goal_key: object = None
    trigger_sim_s: float | None = None
    zero_started_sim_s: dict = field(default_factory=dict)
    closed: bool = False
    events: list = field(default_factory=list)

    def selected_is_zero(self, selected: dict | None) -> bool:
        if not selected:
            return False
        velocity = selected.get('velocity') or {}
        return (abs(float(velocity.get('linear_x_mps', 0.0))) <= self.zero_tolerance
                and abs(float(velocity.get('angular_z_radps', 0.0))) <= self.zero_tolerance
                and abs(float(velocity.get('linear_y_mps', 0.0))) <= self.zero_tolerance)

    def observe(self, now_s: float, robot: str, goal_key, goal_source: str,
                active_goal: bool, intentional_idle: bool,
                evaluation_fresh: bool, selected: dict | None) -> str:
        """Return INACTIVE, TRACKING, TRIGGERED, or END."""
        if self.closed:
            return 'END'
        if self.triggered and robot != self.trigger_robot:
            return 'INACTIVE'
        eligible = (active_goal and not intentional_idle and goal_source in ('manual', 'frontier')
                    and evaluation_fresh and self.selected_is_zero(selected))
        if not eligible:
            self.zero_started_sim_s.pop(robot, None)
            if self.triggered and self.trigger_robot == robot:
                # Continue the captured event after DWB leaves zero; closure is
                # controlled by the post-zero window below.
                return 'TRIGGERED'
            return 'INACTIVE'
        started = self.zero_started_sim_s.setdefault(robot, float(now_s))
        duration = float(now_s) - started
        if not self.triggered and duration >= 1.0:
            self.triggered = True
            self.trigger_robot = robot
            self.trigger_goal_key = goal_key
            self.trigger_sim_s = float(now_s)
            self.events.append({
                'event': 'DWB_SELECTED_ZERO_TRIGGER',
                'robot': robot, 'sim_time_s': float(now_s),
                'zero_started_sim_s': float(started),
                'continuous_zero_duration_s': float(duration),
                'goal_key': repr(goal_key),
            })
            return 'TRIGGERED'
        if self.triggered and self.trigger_robot == robot:
            return 'TRIGGERED'
        return 'TRACKING'

    def should_close(self, now_s: float, goal_key, selected: dict | None,
                     active_goal: bool, recovery_count: int = 0) -> str:
        if not self.triggered or self.closed:
            return 'INACTIVE'
        if goal_key != self.trigger_goal_key or not active_goal:
            self.closed = True
            return 'END'
        elapsed = float(now_s) - float(self.trigger_sim_s)
        if recovery_count > 0:
            self.events.append({'event': 'RECOVERY_OBSERVED',
                                'sim_time_s': float(now_s),
                                'recovery_count': int(recovery_count)})
            self.closed = True
            return 'END'
        if elapsed >= self.hard_window_s:
            self.closed = True
            return 'END'
        if elapsed >= self.post_zero_s:
            self.closed = True
            return 'END'
        return 'ACTIVE'

    def window_start(self) -> float | None:
        return None if self.trigger_sim_s is None else self.trigger_sim_s - self.pre_window_s


def evaluation_event_record(robot: str, sim_time_s: float, reason: str,
                            message_stamp_s: float, evaluation: dict) -> dict:
    return {
        'schema_version': 1, 'record_type': 'dwb_zero_event_evaluation',
        'robot': robot, 'sim_time_s': float(sim_time_s),
        'capture_reason': reason, 'message_stamp_s': float(message_stamp_s),
        'evaluation': evaluation,
    }


def sequence_records(message, roles: list[tuple[int, str]]) -> list[dict]:
    return [trajectory_sequence_record(message.twists[index], index, role)
            for index, role in roles if 0 <= index < len(message.twists)]
