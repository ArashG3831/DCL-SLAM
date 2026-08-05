"""Bounded, observation-only helpers for Nav2 command-pipeline telemetry."""

from __future__ import annotations

from dataclasses import dataclass
import math


PIPELINE_STAGES = (
    'dwb_controller', 'velocity_smoother', 'collision_monitor',
    'final_stamped_command',
)
PRECEDING_STAGE = dict(zip(PIPELINE_STAGES[1:], PIPELINE_STAGES[:-1]))

LINEAR_COMMAND_EPS_MPS = 0.005
ANGULAR_COMMAND_EPS_RADPS = 0.02
TRANSLATION_THRESHOLD_M = 0.01
ROTATION_THRESHOLD_RAD = 0.04
STAGE_FRESHNESS_S = 0.25
DWB_ANGULAR_STALL_S = 2.0
DWB_ZERO_STALL_S = 1.0
DWB_STALE_STALL_S = 1.0


def wrap_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


@dataclass
class StageMetrics:
    """Cumulative message statistics plus the latest command for one stage."""

    message_count: int = 0
    nonzero_linear_count: int = 0
    angular_only_count: int = 0
    zero_command_count: int = 0
    absolute_linear_sum: float = 0.0
    absolute_angular_sum: float = 0.0
    peak_absolute_linear: float = 0.0
    peak_absolute_angular: float = 0.0
    latest_linear: float = 0.0
    latest_angular: float = 0.0
    latest_sim_s: float | None = None

    def observe(self, linear: float, angular: float, sim_s: float) -> None:
        linear = float(linear)
        angular = float(angular)
        self.message_count += 1
        self.absolute_linear_sum += abs(linear)
        self.absolute_angular_sum += abs(angular)
        self.peak_absolute_linear = max(self.peak_absolute_linear, abs(linear))
        self.peak_absolute_angular = max(self.peak_absolute_angular, abs(angular))
        self.latest_linear = linear
        self.latest_angular = angular
        self.latest_sim_s = float(sim_s)
        if abs(linear) > LINEAR_COMMAND_EPS_MPS:
            self.nonzero_linear_count += 1
        elif abs(angular) > ANGULAR_COMMAND_EPS_RADPS:
            self.angular_only_count += 1
        else:
            self.zero_command_count += 1

    def snapshot(self, now_sim_s: float) -> dict:
        count = max(1, self.message_count)
        age = None if self.latest_sim_s is None else max(
            0.0, float(now_sim_s) - self.latest_sim_s)
        return {
            'latest_linear_mps': self.latest_linear,
            'latest_angular_radps': self.latest_angular,
            'message_count': self.message_count,
            'nonzero_linear_count': self.nonzero_linear_count,
            'angular_only_count': self.angular_only_count,
            'zero_command_count': self.zero_command_count,
            'mean_absolute_linear_mps': self.absolute_linear_sum / count,
            'mean_absolute_angular_radps': self.absolute_angular_sum / count,
            'peak_absolute_linear_mps': self.peak_absolute_linear,
            'peak_absolute_angular_radps': self.peak_absolute_angular,
            'time_since_last_message_s': age,
            'fresh': age is not None and age <= STAGE_FRESHNESS_S,
        }


def command_kind(stage: dict) -> str:
    """Classify a fresh command without treating stale data as a command."""
    if not stage or not stage.get('fresh', False):
        return 'MISSING_OR_STALE'
    linear = abs(float(stage.get('latest_linear_mps', 0.0)))
    angular = abs(float(stage.get('latest_angular_radps', 0.0)))
    if linear > LINEAR_COMMAND_EPS_MPS:
        return 'LINEAR'
    if angular > ANGULAR_COMMAND_EPS_RADPS:
        return 'ANGULAR_ONLY'
    return 'ZERO'


def attenuation(preceding: dict, current: dict) -> dict:
    """Describe bounded suppression/reduction relative to the prior stage."""
    if not preceding or not current:
        return {'state': 'MISSING'}
    if not preceding.get('fresh') or not current.get('fresh'):
        return {'state': 'STALE'}
    before = abs(float(preceding.get('latest_linear_mps', 0.0)))
    after = abs(float(current.get('latest_linear_mps', 0.0)))
    if before <= LINEAR_COMMAND_EPS_MPS:
        return {'state': 'NO_PRECEDING_LINEAR_COMMAND'}
    ratio = after / before
    if after <= LINEAR_COMMAND_EPS_MPS:
        return {'state': 'SUPPRESSED', 'linear_ratio': ratio}
    if ratio < 0.8:
        return {'state': 'REDUCED', 'linear_ratio': ratio}
    return {'state': 'PRESERVED', 'linear_ratio': ratio}


def motion_state(window: list[dict]) -> dict:
    """Classify one rolling one-second odometry window."""
    if len(window) < 2:
        return {'state': 'INSUFFICIENT_HISTORY', 'translation_m': 0.0,
                'rotation_rad': 0.0}
    first, last = window[0], window[-1]
    translation = math.hypot(last['x_m'] - first['x_m'],
                             last['y_m'] - first['y_m'])
    rotation = abs(wrap_angle(last['yaw_rad'] - first['yaw_rad']))
    if translation >= TRANSLATION_THRESHOLD_M:
        state = 'TRANSLATING'
    elif rotation >= ROTATION_THRESHOLD_RAD:
        state = 'ROTATING_IN_PLACE'
    else:
        state = 'FULLY_STATIONARY'
    return {'state': state, 'translation_m': translation,
            'rotation_rad': rotation}


def stationary_cause(context: dict) -> str:
    """Return the strongest supported stationary cause, otherwise UNKNOWN."""
    # Selector objects can intentionally outlive an action goal.  A retained
    # selector is not evidence of a live precheck, and must never hide the
    # much stronger fact that no navigation action is active.
    if not context.get('active_goal'):
        if context.get('intentional_frontier_idle',
                       context.get('frontier_phase', False)):
            return 'WAITING_FOR_FRONTIER_CANDIDATE'
        if context.get('handoff_active'):
            return 'GOAL_PRECHECK_OR_HANDOFF'
        return 'NO_ACTIVE_GOAL'
    if context.get('diagnostic_timeout'):
        return 'DIAGNOSTIC_TIMEOUT'
    if context.get('recovery_active'):
        return 'RECOVERY_ACTIVE'
    if context.get('start_not_traversable'):
        return 'COSTMAP_START_NOT_TRAVERSABLE'
    if context.get('planner_active'):
        return 'WAITING_FOR_PLANNER'
    dwb = context.get('dwb_kind')
    if dwb == 'ZERO':
        return 'ACTIVE_GOAL_DWB_ZERO'
    if dwb == 'ANGULAR_ONLY':
        return 'ACTIVE_GOAL_DWB_ANGULAR_ONLY'
    if context.get('smoother_attenuation') in ('SUPPRESSED', 'REDUCED'):
        return 'SMOOTHER_SUPPRESSED_OR_REDUCED'
    if context.get('collision_attenuation') == 'SUPPRESSED' or \
            context.get('collision_action') in (1, 2):
        return 'COLLISION_MONITOR_STOPPED'
    if context.get('final_linear_nonzero'):
        return 'FINAL_COMMAND_NONZERO_BUT_NO_ODOM'
    if context.get('goal_transition'):
        return 'GOAL_CANCEL_OR_TRANSITION'
    return 'UNKNOWN'


@dataclass
class DwbStallDetector:
    """Bounded edge detector for diagnostically significant DWB stalls.

    It is deliberately command-observation-only: it does not influence the
    controller, action server, or planner.  One event is emitted for each
    qualifying continuous command episode and a separate event marks an
    immediate transition away from a selected forward command.
    """

    goal_key: tuple | None = None
    kind: str = 'MISSING_OR_STALE'
    started_sim_s: float | None = None
    previous_kind: str = 'MISSING_OR_STALE'
    emitted: set[str] = None

    def __post_init__(self):
        if self.emitted is None:
            self.emitted = set()

    def observe(self, now_sim_s: float, goal_key: tuple | None,
                kind: str) -> list[dict]:
        """Return newly triggered bounded event descriptors."""
        if goal_key is None or kind == 'LINEAR':
            self.goal_key = goal_key
            self.kind = kind
            self.previous_kind = kind
            self.started_sim_s = None
            self.emitted.clear()
            return []

        if goal_key != self.goal_key or kind != self.kind:
            prior = self.kind if goal_key == self.goal_key else 'MISSING_OR_STALE'
            self.goal_key = goal_key
            self.kind = kind
            self.started_sim_s = float(now_sim_s)
            self.emitted.clear()
            events = []
            if prior == 'LINEAR' and kind in ('ANGULAR_ONLY', 'ZERO'):
                events.append({
                    'trigger': f'TRANSITION_FROM_FORWARD_TO_{kind}',
                    'duration_s': 0.0,
                })
            self.previous_kind = kind
            return events

        if self.started_sim_s is None:
            self.started_sim_s = float(now_sim_s)
        duration = max(0.0, float(now_sim_s) - self.started_sim_s)
        thresholds = {
            'ANGULAR_ONLY': (DWB_ANGULAR_STALL_S,
                             'ANGULAR_ONLY_CONTINUOUS'),
            'ZERO': (DWB_ZERO_STALL_S, 'ZERO_CONTINUOUS'),
            'MISSING_OR_STALE': (DWB_STALE_STALL_S,
                                 'COMMAND_STALE_CONTINUOUS'),
        }
        threshold, trigger = thresholds.get(kind, (math.inf, ''))
        if trigger and duration >= threshold and trigger not in self.emitted:
            self.emitted.add(trigger)
            return [{'trigger': trigger, 'duration_s': duration}]
        return []
