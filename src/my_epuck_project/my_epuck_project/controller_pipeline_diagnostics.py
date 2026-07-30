"""Bounded, observation-only Nav2 command-pipeline diagnostics."""

from collections import deque
import csv
from dataclasses import dataclass, field
import json
import math
from pathlib import Path
import time

import rclpy
from geometry_msgs.msg import Twist, TwistStamped
from nav2_msgs.msg import CollisionMonitorState
from nav2_msgs.action._navigate_to_pose import NavigateToPose_FeedbackMessage
from nav_msgs.msg import Odometry, Path as NavPath
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy._rclpy_pybind11 import RCLError
from sensor_msgs.msg import LaserScan

try:
    from dwb_msgs.msg import LocalPlanEvaluation
except ImportError:  # pragma: no cover - permits non-Nav2 unit imports
    LocalPlanEvaluation = None


COMMAND_STAGES = (
    'controller_raw', 'smoother_input', 'smoother_output',
    'collision_input', 'collision_output', 'final_base_command',
)


@dataclass
class Sample:
    sim_s: float
    wall_s: float
    pose_x: float = 0.0
    pose_y: float = 0.0
    pose_yaw: float = 0.0
    odom_vx: float = 0.0
    odom_wz: float = 0.0
    commands: dict = field(default_factory=dict)
    command_meta: dict = field(default_factory=dict)
    distance_remaining: float | None = None
    active_goal: bool = False
    collision_state: dict = field(default_factory=dict)
    dwb: dict = field(default_factory=dict)

    def as_dict(self):
        value = self.__dict__.copy()
        value['commands'] = dict(self.commands)
        value['command_meta'] = dict(self.command_meta)
        value['collision_state'] = dict(self.collision_state)
        value['dwb'] = dict(self.dwb)
        return value


@dataclass
class Incident:
    incident_id: str
    robot: str
    start_sim_s: float
    trigger_sim_s: float
    active_goal: bool
    distance_remaining_m: float | None
    classification: str = 'UNKNOWN_COMMAND_STALL'
    contributing: list = field(default_factory=list)
    confidence: str = 'low'
    missing_evidence: list = field(default_factory=list)
    explanation: str = ''
    end_sim_s: float | None = None
    samples: list = field(default_factory=list)

    def as_dict(self):
        return self.__dict__.copy()


def _stage_stamp(sample, stage):
    """Return a command's simulation timestamp when one was recorded."""
    meta = sample.command_meta.get(stage)
    if meta and meta.get('sim_s') is not None:
        return float(meta['sim_s'])
    value = sample.commands.get(stage)
    if isinstance(value, (tuple, list)) and len(value) >= 3:
        return float(value[2])
    return None


def _fresh_stages(samples, freshness_s):
    """Return fresh/stale/missing status for the latest stage observations."""
    latest = samples[-1]
    status = {}
    for stage in COMMAND_STAGES:
        if stage not in latest.commands:
            status[stage] = 'missing'
            continue
        stamp = _stage_stamp(latest, stage)
        if stamp is None:
            status[stage] = 'unknown_age'
        else:
            status[stage] = (
                'fresh' if latest.sim_s - stamp <= freshness_s else 'stale')
    return status


def _axis_response(samples, command_stage='final_base_command'):
    """Evaluate linear/angular response across the bounded evidence window."""
    commands = [s.commands.get(command_stage) for s in samples]
    commands = [c for c in commands if c is not None]
    if not commands:
        return {'linear_commanded': False, 'angular_commanded': False,
                'linear_response': False, 'angular_response': False}
    linear_peak = max(abs(float(c[0])) for c in commands)
    angular_peak = max(abs(float(c[1])) for c in commands)
    linear_response = max(abs(s.odom_vx) for s in samples) >= max(
        0.005, min(0.02, linear_peak * 0.2))
    angular_response = max(abs(s.odom_wz) for s in samples) >= max(
        0.02, min(0.08, angular_peak * 0.2))
    if len(samples) > 1:
        translation = math.hypot(
            samples[-1].pose_x - samples[0].pose_x,
            samples[-1].pose_y - samples[0].pose_y)
        heading = abs((samples[-1].pose_yaw - samples[0].pose_yaw + math.pi)
                      % (2 * math.pi) - math.pi)
        linear_response = linear_response or translation >= 0.01
        angular_response = angular_response or heading >= 0.04
    return {
        'linear_commanded': linear_peak > 0.005,
        'angular_commanded': angular_peak > 0.02,
        'linear_response': linear_response,
        'angular_response': angular_response,
        'linear_peak_command_mps': linear_peak,
        'angular_peak_command_radps': angular_peak,
    }


def command_reason(samples, freshness_s=1.5):
    """Classify a stall using fresh, per-axis command/response evidence."""
    if not samples:
        return 'UNKNOWN_COMMAND_STALL', [], 'low', ['all command stages']
    latest = samples[-1]
    commands = latest.commands
    missing = [stage for stage in COMMAND_STAGES if stage not in commands]
    freshness = _fresh_stages(samples, freshness_s)
    if freshness.get('controller_raw') in ('stale', 'unknown_age'):
        return 'COMMAND_STAGE_STALE', [], 'high' if freshness['controller_raw'] == 'stale' else 'low', missing
    if freshness.get('controller_raw') == 'missing':
        return 'UNKNOWN_COMMAND_STALL', [], 'low', missing
    def nonzero(name):
        value = commands.get(name)
        return value is not None and (abs(value[0]) > 0.005 or abs(value[1]) > 0.02)
    controller = commands.get('controller_raw')
    if (controller is not None and abs(controller[0]) <= 0.005 and
            abs(controller[1]) > 0.02 and 'smoother_output' not in commands):
        return 'DWB_ROTATION_ONLY', [], 'medium', missing
    if ('controller_raw' in commands and 'smoother_output' in commands and
            freshness.get('smoother_output') in ('stale', 'unknown_age')):
        return 'COMMAND_STAGE_STALE', [], 'high', missing
    if ('controller_raw' in commands and 'smoother_output' in commands and
            nonzero('controller_raw') and not nonzero('smoother_output')):
        return 'VELOCITY_SMOOTHER_SUPPRESSION', [], 'high', missing
    if ('smoother_output' in commands and 'collision_output' in commands and
            freshness.get('collision_output') in ('stale', 'unknown_age')):
        return 'COMMAND_STAGE_STALE', [], 'high', missing
    if ('smoother_output' in commands and 'collision_output' in commands and
            nonzero('smoother_output') and not nonzero('collision_output')):
        state = latest.collision_state
        action = state.get('action_type')
        if action == 1:
            return 'COLLISION_MONITOR_STOP', [], 'high', missing
        if action == 2:
            return 'COLLISION_MONITOR_SLOWDOWN', [], 'high', missing
        return 'UNKNOWN_COMMAND_STALL', ['COLLISION_MONITOR_STOP'], 'medium', missing
    if ('collision_output' in commands and 'final_base_command' in commands and
            freshness.get('final_base_command') in ('stale', 'unknown_age')):
        return 'COMMAND_STAGE_STALE', [], 'high', missing
    if ('collision_output' in commands and 'final_base_command' in commands and
            nonzero('collision_output') and not nonzero('final_base_command')):
        return 'FINAL_COMMAND_NOT_DELIVERED', [], 'high', missing
    if 'final_base_command' in commands and nonzero('final_base_command'):
        response = _axis_response(samples)
        if (response['linear_commanded'] and
                not response['linear_response'] and
                response['angular_commanded'] and
                response['angular_response']):
            return 'BASE_LINEAR_NOT_RESPONDING', [], 'medium', missing
        if (response['angular_commanded'] and
                not response['angular_response'] and
                response['linear_commanded'] and
                response['linear_response']):
            return 'BASE_ANGULAR_NOT_RESPONDING', [], 'medium', missing
        if ((response['linear_commanded'] and not response['linear_response']) or
                (response['angular_commanded'] and not response['angular_response'])):
            if response['linear_commanded'] and not response['angular_commanded']:
                return 'BASE_LINEAR_NOT_RESPONDING', [], 'medium', missing
            if response['angular_commanded'] and not response['linear_commanded']:
                return 'BASE_ANGULAR_NOT_RESPONDING', [], 'medium', missing
            return 'BASE_NOT_RESPONDING', [], 'medium', missing
    if 'controller_raw' not in commands:
        return 'UNKNOWN_COMMAND_STALL', [], 'low', missing
    if not nonzero('controller_raw'):
        dwb = latest.dwb
        if dwb.get('forward_valid') is False:
            return 'DWB_NO_LEGAL_FORWARD_TRAJECTORY', [], 'high', missing
        if nonzero('controller_raw') is False and abs(commands.get('controller_raw', (0, 0))[1]) > 0.02:
            return 'DWB_ROTATION_ONLY', [], 'medium', missing
        return 'DWB_ZERO_COMMAND', [], 'medium', missing
    return 'UNKNOWN_COMMAND_STALL', [], 'low', missing


class RollingCapture:
    """Fixed-size pre-trigger history and post-trigger incident capture."""

    def __init__(self, pre_s=5.0, post_s=15.0, max_samples=1600,
                 max_incidents=8):
        self.pre_s = pre_s
        self.post_s = post_s
        self.max_samples = max_samples
        self.max_incidents = max_incidents
        self.history = deque(maxlen=max_samples)
        self.incidents = []
        self.active = None

    def add(self, sample):
        self.history.append(sample)
        if self.active is not None:
            self.active.samples.append(sample.as_dict())
            if sample.sim_s - self.active.trigger_sim_s >= self.post_s:
                self.active.end_sim_s = sample.sim_s
                self._finish()

    def trigger(self, robot, sample):
        if self.active is not None or len(self.incidents) >= self.max_incidents:
            return None
        start = sample.sim_s - self.pre_s
        incident = Incident(
            incident_id=f'{robot}-{len(self.incidents) + 1:03d}',
            robot=robot, start_sim_s=max(0.0, start),
            trigger_sim_s=sample.sim_s,
            active_goal=sample.active_goal,
            distance_remaining_m=sample.distance_remaining,
            samples=[item.as_dict() for item in self.history if item.sim_s >= start],
        )
        self.active = incident
        return incident

    def clear(self, sim_s):
        if self.active is not None:
            self.active.end_sim_s = sim_s
            self._finish()

    def _finish(self):
        if self.active is not None:
            self.incidents.append(self.active)
            self.active = None


class PipelineDiagnosticNode(Node):
    """Observe existing Nav2 stages and write bounded telemetry and incidents."""

    def __init__(self, **node_kwargs):
        super().__init__('controller_pipeline_diagnostics', **node_kwargs)
        self.declare_parameter('robot_ids', ['robot1', 'robot2'])
        self.declare_parameter('output_root', '/tmp/controller_diagnostics')
        self.declare_parameter('sample_rate_hz', 2.0)
        self.declare_parameter('stall_window_s', 3.0)
        self.declare_parameter('stall_distance_threshold_m', 0.15)
        self.declare_parameter('stall_displacement_threshold_m', 0.01)
        self.declare_parameter('goal_feedback_timeout_s', 2.0)
        self.robots = list(self.get_parameter('robot_ids').value)
        root = Path(self.get_parameter('output_root').value)
        root.mkdir(parents=True, exist_ok=True)
        self.root = root
        self.rows = {robot: [] for robot in self.robots}
        self.captures = {robot: RollingCapture() for robot in self.robots}
        self.latest = {robot: {'commands': {}, 'last_odom': None} for robot in self.robots}
        self.latest_sim_s = 0.0
        self._last_graph_check_wall = 0.0
        self._finalized = False
        self.counters = {robot: {} for robot in self.robots}
        for robot in self.robots:
            self._create_robot_subscriptions(robot)
        period = 1.0 / float(self.get_parameter('sample_rate_hz').value)
        self.timer = self.create_timer(period, self._sample)
        self.wall_started = time.monotonic()

    def _create_robot_subscriptions(self, robot):
        qos = 10
        def twist(stage):
            return lambda msg: self._command(robot, stage, msg.linear.x, msg.angular.z)
        self.create_subscription(Twist, f'/{robot}/cmd_vel_nav', twist('controller_raw'), qos)
        self.create_subscription(Twist, f'/{robot}/cmd_vel_smoothed', twist('smoother_output'), qos)
        self.create_subscription(Twist, f'/{robot}/cmd_vel_unstamped', twist('collision_output'), qos)
        self.create_subscription(TwistStamped, f'/{robot}/cmd_vel', lambda msg: self._command(robot, 'final_base_command', msg.twist.linear.x, msg.twist.angular.z), qos)
        self.create_subscription(Odometry, f'/{robot}/odom', lambda msg: self._odom(robot, msg), 10)
        self.create_subscription(NavigateToPose_FeedbackMessage, f'/{robot}/navigate_to_pose/_action/feedback', lambda msg: self._feedback(robot, msg), 10)
        self.create_subscription(LaserScan, f'/{robot}/scan_d500_fixed', lambda msg: self._stamp(robot, 'lidar', msg), 10)
        self.create_subscription(CollisionMonitorState, f'/{robot}/collision_monitor_state', lambda msg: self._collision(robot, msg), 10)
        if LocalPlanEvaluation is not None:
            self.create_subscription(LocalPlanEvaluation, f'/{robot}/evaluation', lambda msg: self._dwb(robot, msg), 10)
        self.create_subscription(NavPath, f'/{robot}/plan', lambda msg: self._stamp(robot, 'global_plan', msg), 10)
        self.create_subscription(NavPath, f'/{robot}/local_plan', lambda msg: self._stamp(robot, 'local_plan', msg), 10)

    def _command(self, robot, stage, linear, angular):
        sim_s = self.get_clock().now().nanoseconds * 1e-9
        wall_s = time.monotonic()
        self.latest[robot]['commands'][stage] = (float(linear), float(angular), sim_s)
        self.latest[robot].setdefault('command_meta', {})[stage] = {
            'sim_s': sim_s, 'wall_s': wall_s, 'expected_hz': 20.0,
        }
        if stage == 'controller_raw':
            self.latest[robot]['commands']['smoother_input'] = self.latest[robot]['commands'][stage]
        if stage == 'smoother_output':
            self.latest[robot]['commands']['collision_input'] = self.latest[robot]['commands'][stage]

    def _odom(self, robot, msg):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        self.latest[robot]['last_odom'] = (p.x, p.y, yaw, msg.twist.twist.linear.x, msg.twist.twist.angular.z)
        self._stamp(robot, 'odom', msg)

    def _feedback(self, robot, msg):
        feedback = msg.feedback
        self.latest[robot]['distance_remaining'] = float(feedback.distance_remaining)
        self.latest[robot]['feedback_wall'] = time.monotonic()
        self.latest[robot]['navigation_time_s'] = float(feedback.navigation_time.sec) + float(feedback.navigation_time.nanosec) * 1e-9
        self.latest[robot]['recoveries'] = int(feedback.number_of_recoveries)
        self._stamp(robot, 'feedback', msg)

    def _stamp(self, robot, key, msg):
        self.latest[robot][f'{key}_stamp'] = getattr(getattr(msg, 'header', None), 'stamp', None)
        self.latest[robot][f'{key}_wall'] = time.monotonic()

    def _collision(self, robot, msg):
        self.latest[robot]['collision_state'] = {'action_type': int(msg.action_type), 'polygon_name': msg.polygon_name}
        self.latest[robot].setdefault('collision_observation', {
            'topic': f'/{robot}/collision_monitor_state',
            'topic_available': True, 'message_count': 0,
            'action_types': [],
        })
        observation = self.latest[robot]['collision_observation']
        observation['message_count'] += 1
        observation['action_types'] = sorted(set(observation['action_types']) | {int(msg.action_type)})
        observation['last_message_sim_s'] = self.get_clock().now().nanoseconds * 1e-9
        self._stamp(robot, 'collision_state', msg)

    def _dwb(self, robot, msg):
        valid = 0
        forward = 0
        rejected = []
        for score in msg.twists:
            vx = score.traj.velocity.x
            if score.total < 1e8:
                valid += 1
                if vx > 0.005:
                    forward += 1
            else:
                rejected.append({'vx': vx, 'critics': [s.name for s in score.scores]})
        self.latest[robot]['dwb'] = {'trajectory_count': len(msg.twists), 'valid_count': valid, 'forward_valid_count': forward, 'forward_valid': bool(forward), 'best_index': int(msg.best_index), 'rejected': rejected[:40]}
        self._stamp(robot, 'dwb', msg)

    def _sample(self):
        sim_s = self.get_clock().now().nanoseconds * 1e-9
        self.latest_sim_s = sim_s
        wall_s = time.monotonic() - self.wall_started
        if time.monotonic() - self._last_graph_check_wall >= 1.0:
            self._last_graph_check_wall = time.monotonic()
            topics = {name for name, _ in self.get_topic_names_and_types()}
            for robot in self.robots:
                observation = self.latest[robot].setdefault('collision_observation', {
                    'topic': f'/{robot}/collision_monitor_state',
                    'topic_available': None, 'message_count': 0,
                    'action_types': [],
                })
                observation['topic_available'] = observation['topic'] in topics
        for robot in self.robots:
            state = self.latest[robot]
            odom = state.get('last_odom') or (0., 0., 0., 0., 0.)
            feedback_wall = state.get('feedback_wall')
            active = feedback_wall is not None and time.monotonic() - feedback_wall <= float(self.get_parameter('goal_feedback_timeout_s').value)
            distance = state.get('distance_remaining')
            command_meta = {}
            for stage, meta in state.get('command_meta', {}).items():
                value = dict(meta)
                value['sim_age_s'] = sim_s - value['sim_s']
                value['wall_age_s'] = time.monotonic() - value['wall_s']
                command_meta[stage] = value
            sample = Sample(sim_s, wall_s, odom[0], odom[1], odom[2], odom[3], odom[4], dict(state['commands']), command_meta, distance, active, dict(state.get('collision_state', {})), dict(state.get('dwb', {})))
            capture = self.captures[robot]
            capture.add(sample)
            self.rows[robot].append(sample.as_dict())
            self._detect(robot, sample)

    def _detect(self, robot, sample):
        capture = self.captures[robot]
        if not sample.active_goal or sample.distance_remaining is None or sample.distance_remaining <= float(self.get_parameter('stall_distance_threshold_m').value):
            capture.clear(sample.sim_s)
            return
        window = [item for item in capture.history if sample.sim_s - item.sim_s <= float(self.get_parameter('stall_window_s').value)]
        if not window or window[-1].sim_s - window[0].sim_s < float(self.get_parameter('stall_window_s').value):
            return
        displacement = math.hypot(window[-1].pose_x - window[0].pose_x, window[-1].pose_y - window[0].pose_y)
        if displacement >= float(self.get_parameter('stall_displacement_threshold_m').value):
            capture.clear(sample.sim_s)
            return
        incident = capture.trigger(robot, sample)
        if incident is not None:
            classification, contributing, confidence, missing = command_reason(window)
            incident.classification = classification
            incident.contributing = contributing
            incident.confidence = confidence
            incident.missing_evidence = missing
            incident.explanation = f'Active goal remained beyond tolerance while displacement was {displacement:.4f} m over the stall window.'

    def finalize(self):
        if self._finalized:
            return
        self._finalized = True
        for robot, capture in self.captures.items():
            capture.clear(self.latest_sim_s)
            path = self.root / f'{robot}_controller_pipeline.csv'
            fields = ['sim_s', 'wall_s', 'pose_x', 'pose_y', 'pose_yaw', 'odom_vx', 'odom_wz', 'active_goal', 'distance_remaining'] + list(COMMAND_STAGES) + ['command_stage_meta', 'collision_state', 'dwb']
            with path.open('w', newline='', encoding='utf-8') as stream:
                writer = csv.DictWriter(stream, fieldnames=fields)
                writer.writeheader()
                for row in self.rows[robot]:
                    out = {key: row.get(key) for key in fields}
                    for stage in COMMAND_STAGES:
                        value = row['commands'].get(stage)
                        out[stage] = '' if value is None else json.dumps(value)
                    out['command_stage_meta'] = json.dumps(row['command_meta'])
                    out['collision_state'] = json.dumps(row['collision_state'])
                    out['dwb'] = json.dumps(row['dwb'])
                    writer.writerow(out)
            with (self.root / f'{robot}_controller_stalls.jsonl').open('w', encoding='utf-8') as stream:
                for incident in capture.incidents:
                    stream.write(json.dumps(incident.as_dict()) + '\n')
            observation = self.latest[robot].get('collision_observation', {
                'topic': f'/{robot}/collision_monitor_state',
                'topic_available': False, 'message_count': 0,
                'action_types': [],
            })
            if observation['message_count']:
                observation_status = 'current_action_observed'
            elif observation['topic_available']:
                observation_status = 'topic_available_no_message'
            else:
                observation_status = 'state_topic_unavailable'
            observation = dict(observation)
            observation['status'] = observation_status
            report = {'robot': robot, 'sampling_rate_hz': float(self.get_parameter('sample_rate_hz').value), 'stall_thresholds': {'distance_m': float(self.get_parameter('stall_distance_threshold_m').value), 'displacement_m': float(self.get_parameter('stall_displacement_threshold_m').value), 'window_s': float(self.get_parameter('stall_window_s').value)}, 'rolling_buffer': {'pre_trigger_s': capture.pre_s, 'post_trigger_s': capture.post_s, 'max_samples': capture.max_samples, 'max_incidents': capture.max_incidents}, 'collision_monitor_state_observation': observation, 'incidents_by_class': {}, 'incidents': [item.as_dict() for item in capture.incidents], 'sample_count': len(self.rows[robot])}
            for incident in capture.incidents:
                report['incidents_by_class'][incident.classification] = report['incidents_by_class'].get(incident.classification, 0) + 1
            (self.root / f'{robot}_controller_stall_report.json').write_text(json.dumps(report, indent=2), encoding='utf-8')


def shutdown_node(node):
    """Flush and destroy a node across normal and external ROS shutdown."""
    node.finalize()
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = PipelineDiagnosticNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except (RCLError, RuntimeError):
        if rclpy.ok():
            raise
    finally:
        shutdown_node(node)
