"""Strictly passive structured observer for two-robot exploration experiments."""
import csv, hashlib, json, math, os, re, signal, socket, statistics, subprocess, sys, threading, time, uuid
from collections import Counter, deque
from dataclasses import asdict
from pathlib import Path

import numpy as np
import rclpy
from action_msgs.msg import GoalStatusArray
from geometry_msgs.msg import Twist, TwistStamped
from nav_msgs.msg import OccupancyGrid, Odometry, Path as NavPath
from nav2_msgs.action._navigate_to_pose import NavigateToPose_FeedbackMessage
from rcl_interfaces.msg import Log
from rclpy.duration import Duration
from rclpy.context import Context
from rclpy.signals import SignalHandlerOptions
from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import JointState, LaserScan
from tf2_ros import Buffer, TransformException, TransformListener
from my_epuck_interfaces.msg import (
    DistributedExplorationEvent,
    DistributedExplorationStatus,
    ExplorationClaim,
    ExplorationEvent,
    ExplorationFailure,
    ExplorationStatus,
    FrontierCandidateArray,
    PairDecision,
    PeerMap,
    TaskBidArray,
    TaskSnapshot,
)
from .experiment_metrics import CoverageAttribution, Grid, MotionDetector, MotionSample, TrajectoryOverlap, WarningDeduplicator, allocate_run_directory, atomic_json, duplicate_goal, equivalent_frontiers, finite, known_counts, known_world_cells, utc_now
from .forensic_evidence import ForensicEvidenceWriter

SCHEMA='1.1.0'; STATES={0:'UNKNOWN',1:'PROPOSING',2:'NAVIGATING',3:'SUCCEEDED',4:'FAILED',5:'RELEASED',6:'CANCELED'}; STATUS_STATES={0:'STARTING',1:'ACTIVE',2:'NAVIGATING',3:'NO_ELIGIBLE_CANDIDATES',4:'COMPLETE',5:'STOPPED',6:'ERROR'}
TIME_FIELDS=['run_id','wall_time_utc','ros_time_sec','ros_time_nanosec','elapsed_s','wall_elapsed_s','event_sequence']
TELEMETRY=TIME_FIELDS+['robot_id','pose_x','pose_y','pose_yaw','linear_speed_mps','angular_speed_radps','commanded_linear_mps','commanded_angular_radps','cmd_vel_received','cmd_vel_age_s','cmd_vel_source','distance_travelled_m','claim_state','claim_id','frontier_id','goal_x','goal_y','goal_yaw','navigation_active','distance_remaining_m','recoveries','candidate_count','local_known_cells','shared_known_cells','local_costmap_obstacles','global_costmap_known','global_costmap_obstacles','odom_age_s','scan_age_s','map_age_s','shared_map_age_s','claim_age_s','feedback_age_s']
COVERAGE=TIME_FIELDS+['robot1_local_known','robot2_local_known','robot1_shared_known','robot2_shared_known','shared_free_cells','shared_occupied_cells','shared_unknown_cells','known_area_m2','coverage_gain_cells','coverage_gain_since_start_cells','unique_first_seen_robot1_cells','unique_first_seen_robot2_cells','later_duplicated_by_robot1_cells','later_duplicated_by_robot2_cells','simultaneously_observed_cells','total_known_union_cells','duplicated_known_fraction','shared_maps_equivalent']
HEALTH=TIME_FIELDS+['robot_id','topic_name','topic_rate_hz','topic_age_s','expected_min_rate_hz','stale']
GOAL_LEDGER_EVENTS={
    'CLAIM_PROPOSED', 'CLAIM_CONFLICT_DETECTED', 'EQUIVALENT_FRONTIER_DUPLICATE',
    'ARBITRATION_WON', 'ARBITRATION_LOST', 'NAV_GOAL_SENT',
    'NAV_GOAL_ACCEPTED', 'NAV_GOAL_REJECTED', 'NAVIGATION_SUCCEEDED',
    'NAVIGATION_FAILED', 'NAVIGATION_CANCELED', 'NAVIGATION_CANCELLED',
    'NAVIGATION_TIMEOUT', 'ROUND_INVALIDATED', 'DISTRIBUTED_TASK_FAILURE',
    'DISTRIBUTED_PAIR_DECISION', 'DISTRIBUTED_STATUS',
    'TRAFFIC_WAITING', 'TRAFFIC_RELEASED_FRESH_REALLOCATION',
    'MISSION_COMPLETE', 'MISSION_ABORTED',
}


def is_shutdown_conversion_error(error, shutdown_requested, context_valid):
    """Recognize only the known queued-take teardown signature."""
    return (shutdown_requested and not context_valid
            and isinstance(error, RuntimeError)
            and str(error).startswith('Unable to convert call argument'))

def yaw(q): return math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
def as_grid(m): return Grid(m.info.width,m.info.height,m.info.resolution,m.info.origin.position.x,m.info.origin.position.y,yaw(m.info.origin.orientation),np.asarray(m.data,dtype=np.int8))
def stamp(m):
    s=getattr(getattr(m,'header',None),'stamp',None); return (int(s.sec),int(s.nanosec)) if s else (0,0)
def default_run_id(): return time.strftime('%Y-%m-%dT%H%M%SZ',time.gmtime())+'_'+uuid.uuid4().hex[:4]

class CooperativeExperimentLogger(Node):
    def __init__(self, **node_kwargs):
        super().__init__('cooperative_experiment_logger', **node_kwargs)
        defaults={'run_id':'','output_root':'/home/arash/webots_ws/results','launch_file':'two_robots_observed_single_goal_launch.py','robot_ids':['robot1','robot2'],'global_frame':'shared_map','telemetry_rate_hz':1.,'coverage_rate_hz':.5,'topic_health_rate_hz':.2,'console_summary_period_s':5.,'warning_summary_period_s':30.,'progress_window_s':10.,'minimum_distance_remaining_improvement_m':.03,'minimum_robot_displacement_m':.02,'stuck_window_s':6.,'commanded_linear_threshold_mps':.02,'commanded_angular_threshold_radps':.15,'cmd_vel_zero_linear_epsilon_mps':.001,'cmd_vel_zero_angular_epsilon_radps':.001,'cmd_vel_no_command_timeout_s':1.5,'stuck_displacement_threshold_m':.015,'oscillation_window_s':10.,'angular_sign_change_threshold':4,'oscillation_displacement_threshold_m':.04,'simultaneous_coverage_window_s':2.,'trajectory_bin_size_m':.05,'initial_overlap_exclusion_radius_m':.15,'duplicate_goal_tolerance_m':.15,'shared_map_divergence_grace_s':3.,'enable_rosout_collection':True,'enable_coverage_attribution':True,'enable_trajectory_overlap':True,'enable_console_status':True,'odom_stale_s':2.,'scan_stale_s':2.,'map_stale_s':5.,'shared_map_stale_s':5.,'candidate_stale_s':5.,'claim_stale_s':4.,'status_stale_s':4.,'feedback_stale_s':3.,'costmap_stale_s':5.,'enable_forensic_capture':False,'forensic_snapshot_interval_s':15.,'enable_contact_capture':False,'contact_sampling_period_ms':20,'webots_port':23000,'terminal_small_frontier_length_m':0.20}
        defaults.update({
            'world_profile': 'small',
            'source_world_path': '',
            'installed_world_path': '',
            'world_dimensions': [0.0, 0.0],
            'robot_start_poses_json': '{}',
            # Direct/unit and physical deployments have no Webots world.
            # Simulation launch always overrides this with WORLD_DERIVED.
            'known_relative_transform': [-0.3, 0.0, -math.pi],
            'transform_source': 'EXPLICIT_PHYSICAL',
            'slam_resolution': 0.01,
            'fusion_resolution': 0.01,
            'global_costmap_resolution': 0.005,
            'local_costmap_resolution': 0.005,
            'lidar_maximum_range': 12.0,
            'initial_configuration_json': '{}',
            'world_sha256': '',
            'coverage_attribution_resolution': 0.01,
        })
        for k,v in defaults.items(): self.declare_parameter(k,v)
        self.p={k:self.get_parameter(k).value for k in defaults}; self.robots=list(self.p['robot_ids']); self.start=time.monotonic(); self.start_ros=self.get_clock().now().nanoseconds*1e-9; self.start_utc=utc_now(); self.sequence=0; self.finalized=False; self._finalizing=False; self._closed=False; self.write_failures=0; self.dropped_samples=0
        if len(self.p['known_relative_transform']) != 3:
            raise ValueError('known_relative_transform must be explicit (physical) or world-derived')
        self._state_lock=threading.RLock(); self._io_lock=threading.RLock(); self._lifecycle_lock=threading.Lock(); self.internal_errors=Counter(); self._reporting_internal_error=False; self._observer_timers=[]
        self._map_cache={}; self._transformed_cache={}; self._last_attributed={}; self._cpu_samples=[]; self._rss_samples=[]; self._cpu_previous=None
        self.run_id,self.directory=allocate_run_directory(Path(self.p['output_root']),self.p['run_id'] or default_run_id())
        self._artifact_finalization = {
            'complete': False,
            'status': 'NOT_FINALIZED',
            'required': [],
            'missing': [],
        }
        self.robot_counts={r:Counter() for r in self.robots}; self.cycle_durations={r:[] for r in self.robots}; self.cycle_starts={}; self.region_attempts={r:Counter() for r in self.robots}; self.exhausted_since={r:None for r in self.robots}; self.exhausted_duration={r:0. for r in self.robots}; self.mission_completion_time=None; self.mission_terminal_reason=''; self.statuses={}
        self.files=[]; self.events=open(self.directory/'events.jsonl','a',encoding='utf-8',buffering=1); self.goal_decisions=open(self.directory/'goal_decision_ledger.jsonl','a',encoding='utf-8',buffering=1); self.files.append(self.goal_decisions); self.nav2_diagnostics=open(self.directory/'nav2_diagnostics.jsonl','a',encoding='utf-8',buffering=1); self.files.append(self.nav2_diagnostics); self.frontier_regions_file=None; self.nav2_diagnostic_count=0; self._diagnostic_last={}; self.action_goal_states={}; self.warns=WarningDeduplicator(); self.counts=Counter(); self.last={}; self.windows={}; self.stale={}; self.latest={r:{} for r in self.robots}; self.claims={}; self.distributed_last={}
        # Protocol counters deliberately separate replicated publications from
        # unique decisions and local navigation outcomes.
        self.unique_agreed_rounds=set(); self.unique_agreed_decisions=set()
        self.round_outcomes=Counter(); self.planner_query_counts=Counter()
        self.planner_query_duration_s=Counter()
        self.agreement_publications=0; self.dispatch_attempts=0; self.goals_terminal=0
        self.detectors={r:MotionDetector(self.p['progress_window_s'],self.p['minimum_distance_remaining_improvement_m'],self.p['minimum_robot_displacement_m'],self.p['stuck_window_s'],self.p['commanded_linear_threshold_mps'],self.p['commanded_angular_threshold_radps'],self.p['stuck_displacement_threshold_m'],self.p['oscillation_window_s'],int(self.p['angular_sign_change_threshold']),self.p['oscillation_displacement_threshold_m']) for r in self.robots}
        self.attribution=CoverageAttribution(self.p['simultaneous_coverage_window_s']); self.trajectory=TrajectoryOverlap(self.p['trajectory_bin_size_m'],self.p['initial_overlap_exclusion_radius_m']); self.initial_known=None; self.previous_known=None; self.writers={}
        forensic_enabled = self.p['enable_forensic_capture']
        if isinstance(forensic_enabled, str):
            forensic_enabled = forensic_enabled.lower() == 'true'
        contact_enabled = self.p.get('enable_contact_capture', False)
        if isinstance(contact_enabled, str):
            contact_enabled = contact_enabled.lower() == 'true'
        self.contact_capture = bool(contact_enabled)
        try:
            initial_configuration = json.loads(
                self.p['initial_configuration_json'])
            scan_matching_enabled = bool(
                initial_configuration.get('use_scan_matching', False) or
                initial_configuration.get(
                    'slam_runtime_parameters', {}).get(
                        'use_scan_matching', False))
        except (TypeError, ValueError, json.JSONDecodeError):
            scan_matching_enabled = False
        self.scan_matching_enabled = scan_matching_enabled
        self.forensic = (ForensicEvidenceWriter(
            self.directory, self.robots, self.p['forensic_snapshot_interval_s'],
            scan_matching_enabled=scan_matching_enabled)
            if forensic_enabled else None)
        self.ground_truth_process = None
        self.ground_truth_log = None
        self.ground_truth_ready_file = None
        self.ground_truth_exit_reported = False
        if self.forensic is not None or self.contact_capture:
            self.start_forensic_ground_truth()
        self.stack_ready=False; self.divergence_since=None; self.divergence_reported=False; self.last_progress={}; self.tf_state={}; self.shared_map_seen=set()
        self.tf_buffer=Buffer(); self.tf_listener=TransformListener(self.tf_buffer,self)
        for r in self.robots: self.writers[r]=self.csv_file(f'{r}_timeseries.csv',TELEMETRY)
        self.coverage=self.csv_file('coverage.csv',COVERAGE); self.health=self.csv_file('topic_health.csv',HEALTH)
        (self.directory/'README.txt').write_text('Passive data; schema and formulas: my_epuck_project/docs/cooperative_experiment_logging.md\n',encoding='utf-8')
        self.write_manifest(False,'running'); self.event('RUN_START','experiment run started',console=True); self.subscribe()
        self._observer_timers.append(self.create_timer(1/self.p['telemetry_rate_hz'],lambda:self.safe_call('telemetry',self.sample_telemetry)))
        self._observer_timers.append(self.create_timer(1/self.p['coverage_rate_hz'],lambda:self.safe_call('coverage',self.sample_coverage)))
        self._observer_timers.append(self.create_timer(1/self.p['topic_health_rate_hz'],lambda:self.safe_call('topic_health',self.sample_health)))
        self._observer_timers.append(self.create_timer(self.p['console_summary_period_s'],lambda:self.safe_call('console_status',self.console)))
        self._observer_timers.append(self.create_timer(1.,lambda:self.safe_call('process_resources',self.sample_process_resources)))
        self._observer_timers.append(self.create_timer(5.,lambda:self.safe_call('file_flush',self.flush)))
        if self.ground_truth_process is not None:
            self._observer_timers.append(self.create_timer(
                1., lambda: self.safe_call(
                    'forensic_supervisor_monitor',
                    self.monitor_forensic_ground_truth)))
        if self.forensic is not None:
            self._observer_timers.append(self.create_timer(
                float(self.p['forensic_snapshot_interval_s']),
                lambda: self.safe_call('forensic_snapshot',
                                       self.forensic_snapshot)))

    def csv_file(self,name,fields):
        f=open(self.directory/name,'w',newline='',encoding='utf-8'); self.files.append(f); w=csv.DictWriter(f,fieldnames=fields); w.writeheader(); return w
    def csv_row(self,writer,row):
        with self._io_lock:
            if self._closed:self.dropped_samples+=1; return
            writer.writerow(finite(row))
    def qos(self,reliable=True,transient=False,depth=10): return QoSProfile(history=HistoryPolicy.KEEP_LAST,depth=depth,reliability=ReliabilityPolicy.RELIABLE if reliable else ReliabilityPolicy.BEST_EFFORT,durability=DurabilityPolicy.TRANSIENT_LOCAL if transient else DurabilityPolicy.VOLATILE)
    def observe(self,message_type,topic,callback,qos,subsystem):
        return self.create_subscription(message_type,topic,lambda message:self.safe_call(subsystem,callback,message),qos)
    def start_forensic_ground_truth(self):
        """Start an external read-only Webots Supervisor observer.

        The observer is diagnostic-only and retries its controller connection
        while Webots is starting.  It never publishes ROS data or commands.
        """
        forensic_dir = self.directory / 'forensic'
        forensic_dir.mkdir(parents=True, exist_ok=True)
        output = forensic_dir / 'supervisor_ground_truth.csv'
        contact_output = forensic_dir / 'contact_points.csv'
        log_path = forensic_dir / 'supervisor_ground_truth.log'
        ready_file = forensic_dir / 'supervisor_ready.json'
        self.ground_truth_ready_file = ready_file
        environment = os.environ.copy()
        try:
            from ament_index_python.packages import get_package_prefix
            driver_prefix = get_package_prefix('webots_ros2_driver')
            python_version = f'python{sys.version_info.major}.{sys.version_info.minor}'
            controller_python = os.path.join(
                driver_prefix, 'lib', 'controller', 'python')
            driver_site = os.path.join(
                driver_prefix, 'lib', python_version, 'site-packages')
            environment['WEBOTS_HOME'] = driver_prefix
            environment['PYTHONPATH'] = os.pathsep.join(
                p for p in (controller_python, driver_site,
                            environment.get('PYTHONPATH', '')) if p)
        except Exception as exc:
            self.event('FORENSIC_SUPERVISOR_START_FAILED', str(exc),
                       severity='WARN', allow_during_shutdown=True)
            return
        environment['WEBOTS_CONTROLLER_URL'] = (
            f"tcp://127.0.0.1:{self.p['webots_port']}/"
            'ForensicGroundTruthSupervisor')
        command = [sys.executable, '-m',
                   'my_epuck_project.cooperative_ground_truth_observer',
                   '--output', str(output), '--robot-def', 'robot1',
                   '--robot-def', 'robot2', '--sample-period-s', '0.10',
                   '--ready-file', str(ready_file),
                   '--controller-url', environment['WEBOTS_CONTROLLER_URL'],
                   '--runtime-directory', str(forensic_dir / 'runtime')]
        if self.contact_capture:
            command.extend([
                '--contact-output', str(contact_output),
                '--contact-sampling-period-ms', str(int(
                    self.p.get('contact_sampling_period_ms', 20))),
            ])
        try:
            self.ground_truth_log = log_path.open('w', encoding='utf-8')
            self.ground_truth_process = subprocess.Popen(
                command, env=environment, stdout=self.ground_truth_log,
                stderr=subprocess.STDOUT, start_new_session=True)
            self.event('FORENSIC_SUPERVISOR_STARTED',
                       'external Webots ground-truth observer started',
                       supervisor_pid=self.ground_truth_process.pid,
                       output=str(output), allow_during_shutdown=True)
        except OSError as exc:
            self.event('FORENSIC_SUPERVISOR_START_FAILED', str(exc),
                       severity='WARN', allow_during_shutdown=True)

    def monitor_forensic_ground_truth(self):
        """Record Supervisor readiness and unexpected child termination.

        The Supervisor is an external diagnostic process.  A failed observer
        must be visible in the run artifact without taking down the passive
        ROS logger or changing the production control graph.
        """
        process = self.ground_truth_process
        if process is None:
            return
        if (self.ground_truth_ready_file is not None and
                self.ground_truth_ready_file.exists() and
                not self.counts['FORENSIC_SUPERVISOR_READY']):
            try:
                ready = json.loads(self.ground_truth_ready_file.read_text(
                    encoding='utf-8'))
            except (OSError, ValueError) as exc:
                self.event('FORENSIC_SUPERVISOR_READY_FILE_ERROR', str(exc),
                           severity='WARN', allow_during_shutdown=True)
            else:
                self.event('FORENSIC_SUPERVISOR_READY',
                           'external Webots Supervisor connected and found all robots',
                           supervisor_pid=process.pid, **ready,
                           allow_during_shutdown=True)
        return_code = process.poll()
        if return_code is not None and not self.ground_truth_exit_reported:
            self.ground_truth_exit_reported = True
            severity = 'INFO' if return_code == 0 else 'ERROR'
            self.event('FORENSIC_SUPERVISOR_EXITED',
                       'external Webots ground-truth observer exited',
                       severity=severity, return_code=return_code,
                       log_path=str(self.directory / 'forensic' /
                                    'supervisor_ground_truth.log'),
                       allow_during_shutdown=True)

    def stop_forensic_ground_truth(self):
        process = self.ground_truth_process
        if process is None:
            return
        if process.poll() is None:
            try:
                process.terminate()
                process.wait(timeout=8.0)
            except (OSError, subprocess.TimeoutExpired, KeyboardInterrupt):
                try:
                    if process.poll() is None:
                        process.kill()
                    process.wait(timeout=3.0)
                except (OSError, subprocess.TimeoutExpired, KeyboardInterrupt):
                    pass
        if self.ground_truth_log is not None:
            try:
                self.ground_truth_log.flush()
                self.ground_truth_log.close()
            except OSError:
                pass
        self.event('FORENSIC_SUPERVISOR_STOPPED',
                   'external ground-truth observer stopped',
                   return_code=process.returncode, allow_during_shutdown=True)

    def forensic_snapshot(self, force=False):
        if self.forensic is None:
            return
        now_ros = self.ros_seconds()
        now_wall = time.monotonic() - self.start
        if force:
            self.forensic.save_final_maps(self.latest, now_ros, now_wall)
        else:
            self.forensic.capture_maps(self.latest, now_ros, now_wall)
        for robot in self.robots:
            for target, source in (
                    (self.p['global_frame'], f'{robot}/map'),
                    (self.p['global_frame'], f'{robot}/odom'),
                    (f'{robot}/odom', f'{robot}/base_footprint')):
                try:
                    transform = self.tf_buffer.lookup_transform(
                        target, source, Time(),
                        timeout=Duration(seconds=0.03))
                    self.forensic.record_transform(
                        now_ros, now_wall, target, source, transform=transform)
                except TransformException as exc:
                    self.forensic.record_transform(
                        now_ros, now_wall, target, source, error=str(exc))
        self.forensic.flush()
    def subscribe(self):
        for r in self.robots:
            self.observe(Odometry,f'/{r}/odom',lambda m,x=r:self.odom(x,m),qos_profile_sensor_data,f'{r}.odom')
            # Passive controller-health evidence; it never gates or commands
            # the running stack.
            self.observe(JointState,f'/{r}/joint_states',lambda m,x=r:self.mark(x,'joint_states',m),qos_profile_sensor_data,f'{r}.joint_states')
            for s in ('scan_d500_fixed','scan_d500_slam'): self.observe(LaserScan,f'/{r}/{s}',lambda m,x=r,k=s:self.mark(x,k,m),qos_profile_sensor_data,f'{r}.{s}')
            for s in ('map','shared_map','local_costmap/costmap','global_costmap/costmap'): self.observe(OccupancyGrid,f'/{r}/{s}',lambda m,x=r,k=s:self.mark(x,k,m),self.qos(True,True,1),f'{r}.{s}')
            self.observe(PeerMap,f'/cslam/{r}/local_map',lambda m,x=r:self.mark(x,'peer_map',m),self.qos(True,True,1),f'{r}.peer_map')
            self.observe(FrontierCandidateArray,f'/{r}/frontier_candidates',lambda m,x=r:self.candidates(x,m),self.qos(True,False,1),f'{r}.candidates')
            self.observe(ExplorationClaim,f'/cslam/{r}/exploration_claim',lambda m,x=r:self.claim(x,m),self.qos(True,False,10),f'{r}.claim')
            self.observe(ExplorationStatus,f'/cslam/{r}/exploration_status',lambda m,x=r:self.status(x,m),self.qos(True,False,10),f'{r}.status')
            self.observe(ExplorationEvent,f'/cslam/{r}/exploration_event',lambda m,x=r:self.coordinator_event(x,m),self.qos(True,False,50),f'{r}.event')
            self.observe(TaskSnapshot,f'/{r}/task_snapshot',lambda m,x=r:self.distributed_snapshot(x,m),self.qos(True,True,1),f'{r}.task_snapshot')
            self.observe(TaskBidArray,f'/{r}/task_bids',lambda m,x=r:self.distributed_bids(x,m),self.qos(True,True,1),f'{r}.task_bids')
            self.observe(PairDecision,f'/{r}/pair_decision',lambda m,x=r:self.distributed_decision(x,m),self.qos(True,True,1),f'{r}.pair_decision')
            self.observe(DistributedExplorationStatus,f'/{r}/distributed_status',lambda m,x=r:self.distributed_status(x,m),self.qos(True,True,1),f'{r}.distributed_status')
            self.observe(DistributedExplorationEvent,f'/{r}/distributed_event',lambda m,x=r:self.distributed_event(x,m),self.qos(True,False,50),f'{r}.distributed_event')
            self.observe(ExplorationFailure,f'/{r}/exploration_failure',lambda m,x=r:self.distributed_failure(x,m),self.qos(True,True,10),f'{r}.exploration_failure')
            self.observe(NavigateToPose_FeedbackMessage,f'/{r}/navigate_to_pose/_action/feedback',lambda m,x=r:self.feedback(x,m),self.qos(),f'{r}.feedback')
            self.observe(GoalStatusArray,f'/{r}/navigate_to_pose/_action/status',lambda m,x=r:self.mark(x,'navigate_status',m),self.qos(True,True,1),f'{r}.navigate_status')
            self.observe(GoalStatusArray,f'/{r}/follow_path/_action/status',lambda m,x=r:self.action_status(x,'FOLLOW_PATH',m),self.qos(True,True,1),f'{r}.follow_path_status')
            self.observe(GoalStatusArray,f'/{r}/compute_path_to_pose/_action/status',lambda m,x=r:self.action_status(x,'COMPUTE_PATH_TO_POSE',m),self.qos(True,True,1),f'{r}.compute_path_status')
            self.observe(NavPath,f'/{r}/plan',lambda m,x=r:self.plan(x,m),self.qos(),f'{r}.plan')
            self.observe(Twist,f'/{r}/cmd_vel_nav',lambda m,x=r:self.command(x,m,'cmd_vel_nav'),self.qos(),f'{r}.cmd_vel_nav')
            self.observe(TwistStamped,f'/{r}/cmd_vel',lambda m,x=r:self.command(x,m.twist,'cmd_vel'),self.qos(),f'{r}.cmd_vel')
        if self.p['enable_rosout_collection']: self.observe(Log,'/rosout',self.rosout,self.qos(True,True,1000),'rosout')
    def ros_seconds(self): return self.get_clock().now().nanoseconds*1e-9
    def ros_now(self): n=self.get_clock().now().nanoseconds; return n//1000000000,n%1000000000
    def common(self,source='/cooperative_experiment_logger',robot=None,source_stamp=None):
        with self._state_lock:
            self.sequence+=1; sequence=self.sequence
        sec,nsec=source_stamp or self.ros_now(); return {'schema_version':SCHEMA,'run_id':self.run_id,'event_sequence':sequence,'wall_time_utc':utc_now(),'ros_time_sec':sec,'ros_time_nanosec':nsec,'elapsed_s':self.ros_seconds()-self.start_ros,'wall_elapsed_s':time.monotonic()-self.start,'robot_id':robot,'source':source}
    def event(self,event_type,message,robot=None,source='/cooperative_experiment_logger',severity='INFO',source_stamp=None,console=False,allow_during_shutdown=False,**extra):
        if (self._finalizing or self._closed) and not allow_during_shutdown:return None
        row=self.common(source,robot,source_stamp); row.update(severity=severity,event_type=event_type,message=message); row.update(finite(extra))
        with self._state_lock:self.counts[event_type]+=1
        try:
            encoded=json.dumps(finite(row),separators=(',',':'),allow_nan=False)+'\n'
            with self._io_lock:
                if self._closed:return None
                self.events.write(encoded)
                if event_type in GOAL_LEDGER_EVENTS:
                    ledger = dict(row)
                    ledger['decision_stage'] = event_type
                    ledger['observed_navigation_active'] = bool(
                        robot is not None and
                        self.latest.get(robot, {}).get(
                            'navigation_active', False))
                    self.goal_decisions.write(
                        json.dumps(finite(ledger), separators=(',', ':'),
                                   allow_nan=False) + '\n')
        except (OSError,TypeError,ValueError) as exc:
            self.write_failures+=1; self.get_logger().error(f'event write failed: {exc}',throttle_duration_sec=10.)
        if console or severity=='ERROR' or event_type.endswith(('_STARTED','_CLEARED')):
            text=f"[{row['elapsed_s']:.1f}s][{robot or '-'}] {event_type} {message}"
            # Separate fixed call sites are required by rclpy's severity-per-call-site cache.
            if severity=='ERROR':self.get_logger().error(text)
            elif severity=='WARN':self.get_logger().warning(text)
            else:self.get_logger().info(text)
        return row
    def safe_call(self,subsystem,operation,*args):
        if self._finalizing or self._closed:
            self.dropped_samples+=1; return None
        try:return operation(*args)
        except Exception as exc:
            self.record_internal_error(subsystem,exc); return None
    def record_internal_error(self,subsystem,exc):
        key=f'{subsystem}:{type(exc).__name__}:{exc}'
        with self._state_lock:
            self.internal_errors[key]+=1; occurrence=self.internal_errors[key]
        if self._reporting_internal_error:return
        self._reporting_internal_error=True
        try:
            if occurrence==1 or occurrence%100==0:
                self.event('LOGGER_INTERNAL_ERROR',f'{subsystem}: {type(exc).__name__}: {exc}',severity='ERROR',subsystem=subsystem,occurrence_count=occurrence)
        except Exception:
            self.get_logger().error(f'logger internal error reporting failed in {subsystem}',throttle_duration_sec=10.)
        finally:self._reporting_internal_error=False
    def mark(self,r,key,msg):
        now=self.ros_seconds()
        with self._state_lock:
            self.last[(r,key)]=now; self.windows.setdefault((r,key),deque()).append(now); self.latest[r][key]=msg
            if key == 'shared_map':
                self.shared_map_seen.add(r)
        if self.forensic is not None and key == 'peer_map':
            self.forensic.record_peer_map(
                r, msg, now, time.monotonic() - self.start)
        if self.forensic is not None and self.scan_matching_enabled and key == 'map':
            self.record_scan_correction_at_map_update(r, now)

    def record_scan_correction_at_map_update(self, robot, now_ros):
        """Record passive scan-match correction evidence at each local map update."""
        now_wall = time.monotonic() - self.start
        try:
            transform = self.tf_buffer.lookup_transform(
                f'{robot}/map', f'{robot}/odom', Time(),
                timeout=Duration(seconds=0.03))
            self.forensic.record_scan_correction(
                robot, now_ros, now_wall, self.latest[robot].get('odom'),
                map_to_odom=transform)
        except TransformException as exc:
            self.forensic.record_scan_correction(
                robot, now_ros, now_wall, self.latest[robot].get('odom'),
                error=str(exc))
    def age(self,r,key):
        value=self.last.get((r,key)); return self.ros_seconds()-value if value else None
    def odom(self,r,msg):
        self.mark(r,'odom',msg)
        if self.forensic is not None:
            self.forensic.record_odom(
                r, msg, self.ros_seconds(), time.monotonic() - self.start)
        p=msg.pose.pose.position
        local_yaw=yaw(msg.pose.pose.orientation)
        try:
            transform=self.tf_buffer.lookup_transform(
                self.p['global_frame'], msg.header.frame_id,
                Time.from_msg(msg.header.stamp),
                timeout=Duration(seconds=0.05))
            t=transform.transform.translation
            heading=yaw(transform.transform.rotation)
            cosine, sine=math.cos(heading), math.sin(heading)
            shared_x=t.x+cosine*p.x-sine*p.y
            shared_y=t.y+sine*p.x+cosine*p.y
            shared_yaw=(heading+local_yaw+math.pi)%(2*math.pi)-math.pi
        except TransformException:
            # Do not feed local-frame points to cross-robot metrics.
            return
        self.latest[r]['pose']=(shared_x,shared_y,shared_yaw)
        self.latest[r]['speed']=(msg.twist.twist.linear.x,msg.twist.twist.angular.z)
        if self.p['enable_trajectory_overlap']: self.trajectory.add(r,shared_x,shared_y)
    def command(self,r,msg,source='cmd_vel'):
        self.mark(r,'cmd_vel',msg)
        now=self.ros_seconds(); linear=float(msg.linear.x); angular=float(msg.angular.z)
        self.latest[r].update(command=(linear,angular),cmd_vel_received_ros_s=now,cmd_vel_source=source)
        zero=(abs(linear)<=float(self.p['cmd_vel_zero_linear_epsilon_mps']) and
              abs(angular)<=float(self.p['cmd_vel_zero_angular_epsilon_radps']))
        previous=self.latest[r].get('cmd_vel_effectively_zero')
        if previous is not None and previous != zero:
            self.event('CMD_VEL_ZERO_CLEARED' if not zero else 'CMD_VEL_ZERO_STARTED',
                       'effective command state changed',r,f'/{r}/{source}',
                       linear_mps=linear,angular_radps=angular,
                       zero_linear_epsilon_mps=self.p['cmd_vel_zero_linear_epsilon_mps'],
                       zero_angular_epsilon_radps=self.p['cmd_vel_zero_angular_epsilon_radps'])
        self.latest[r]['cmd_vel_effectively_zero']=zero
    def plan(self,r,msg): self.mark(r,'plan',msg); self.latest[r]['path_length']=sum(math.hypot(b.pose.position.x-a.pose.position.x,b.pose.position.y-a.pose.position.y) for a,b in zip(msg.poses,msg.poses[1:]))
    def action_status(self,r,action,msg):
        self.mark(r,action.lower(),msg)
        names={0:'UNKNOWN',1:'ACCEPTED',2:'EXECUTING',3:'CANCELING',4:'SUCCEEDED',5:'CANCELED',6:'ABORTED'}
        for status in msg.status_list:
            goal_id=bytes(status.goal_info.goal_id.uuid).hex()
            value=int(status.status); key=(r,action,goal_id); previous=self.action_goal_states.get(key)
            if previous==value: continue
            self.action_goal_states[key]=value
            self.event(f'{action}_STATUS', 'action goal status changed', r,
                       f'/{r}/{action.lower()}/_action/status',
                       source_stamp=stamp(msg), action=action,
                       goal_uuid=goal_id, status_value=value,
                       status_name=names.get(value,str(value)))
    def candidates(self,r,msg):
        old=self.latest[r].get('candidate_count'); count=len(msg.candidates); self.mark(r,'frontier_candidates',msg); self.latest[r]['candidate_count']=count
        # Keep the observer passive, but retain the bounded candidate evidence
        # needed to audit one distributed-assignment round after a live run.
        candidates=[{'frontier_id':item.frontier_id,
                     'centroid':[item.centroid.x,item.centroid.y],
                     'bounds':[item.bounding_box_min.x,item.bounding_box_min.y,
                               item.bounding_box_max.x,item.bounding_box_max.y],
                     'approach':[item.approach_pose.pose.position.x,
                                 item.approach_pose.pose.position.y],
                     'visible_reveal_gain':item.information_gain,
                     'score':item.score,
                     'reachability_state':item.reachability_state,
                     'path_length_m':item.path_length_m,
                     'local_path_length_m':item.local_path_length_m,
                     'local_path_samples':len(item.local_path_samples)}
                    for item in msg.candidates]
        self.event('CANDIDATE_BATCH_RECEIVED',f'{count} reachable candidates',r,f'/{r}/frontier_candidates',source_stamp=stamp(msg),map_revision=msg.map_revision,candidate_count=count,detected_frontier_count=msg.detected_frontier_count,detected_not_queried_count=getattr(msg,'detected_not_queried_count',msg.unclassified_frontier_count),small_frontier_count=msg.small_frontier_count,out_of_range_frontier_count=msg.out_of_range_frontier_count,unreachable_frontier_count=msg.unreachable_frontier_count,planner_failure_count=msg.planner_failure_count,unclassified_frontier_count=msg.unclassified_frontier_count,candidates=candidates)
        diagnostic_regions = getattr(msg, 'diagnostic_regions_json', '')
        if diagnostic_regions:
            if self.frontier_regions_file is None:
                self.frontier_regions_file = open(
                    self.directory / 'frontier_regions.jsonl', 'a',
                    encoding='utf-8', buffering=1)
                self.files.append(self.frontier_regions_file)
            try:
                payload = json.loads(diagnostic_regions)
                payload.update({'capture_robot': r,
                                'capture_stamp': stamp(msg),
                                'capture_elapsed_s': self.ros_seconds() - self.start_ros})
                self.frontier_regions_file.write(
                    json.dumps(finite(payload), separators=(',', ':')) + '\n')
            except (TypeError, ValueError, OSError) as exc:
                self.record_internal_error('frontier_region_capture', exc)
        if old is not None and old!=count:self.event('CANDIDATE_COUNT_CHANGED',f'{old} -> {count}',r)
        if not count:self.event('NO_REACHABLE_CANDIDATES','candidate batch empty',r)
    @staticmethod
    def uuid_text(value): return bytes(value.uuid).hex()
    def distributed_changed(self,key,value):
        previous=self.distributed_last.get(key); self.distributed_last[key]=value; return previous!=value
    def distributed_snapshot(self,r,msg):
        self.mark(r,'task_snapshot',msg)
        if not self.distributed_changed((r,'snapshot'),(self.uuid_text(msg.source_session_id),msg.source_snapshot_epoch,msg.source_map_revision,msg.source_map_fingerprint)):return
        tasks=[{'physical_signature':task.physical_signature,'local_frontier_id':task.local_frontier_id,'centroid':[task.centroid.x,task.centroid.y],'bounds':[task.bounding_box_min.x,task.bounding_box_min.y,task.bounding_box_max.x,task.bounding_box_max.y],'approach':[task.approach_pose.pose.position.x,task.approach_pose.pose.position.y],'visible_reveal_gain':task.visible_reveal_gain,'local_ordering_score':task.local_ordering_score,'local_path_valid':task.local_path_valid,'local_path_length_m':task.local_path_length_m,'local_path_samples':len(task.local_path_samples),'frontier_geometry_samples':len(task.frontier_geometry),'visible_cell_samples':len(task.visible_cells)} for task in msg.tasks]
        self.event('DISTRIBUTED_TASK_SNAPSHOT',f'{len(tasks)} bounded physical tasks',r,f'/{r}/task_snapshot',source_stamp=stamp(msg),source_session_id=self.uuid_text(msg.source_session_id),snapshot_epoch=msg.source_snapshot_epoch,source_map_revision=msg.source_map_revision,source_map_fingerprint=msg.source_map_fingerprint,tasks=tasks)
    def distributed_bids(self,r,msg):
        self.mark(r,'task_bids',msg)
        fingerprint=(msg.round_id,msg.union_hash,msg.source_snapshot_epoch,tuple((bid.canonical_task_id,bid.path_valid,round(bid.path_length_m,4)) for bid in msg.bids))
        if not self.distributed_changed((r,'bids'),fingerprint):return
        bids=[{'canonical_task_id':bid.canonical_task_id,'path_valid':bid.path_valid,'path_length_m':bid.path_length_m,'estimated_travel_cost':bid.estimated_travel_cost,'heading_cost':bid.heading_cost,'own_utility_contribution':bid.own_utility_contribution,'path_samples':[[point.x,point.y] for point in bid.path_samples]} for bid in msg.bids]
        self.event('DISTRIBUTED_BID_ARRAY',f'{len(bids)} bounded local bids',r,f'/{r}/task_bids',source_stamp=stamp(msg),source_session_id=self.uuid_text(msg.source_session_id),round_id=msg.round_id,union_hash=msg.union_hash,source_snapshot_epoch=msg.source_snapshot_epoch,bids=bids)
    def distributed_decision(self,r,msg):
        self.mark(r,'pair_decision',msg)
        if not self.distributed_changed((r,'decision'),(msg.round_id,msg.union_hash,msg.decision_hash)):return
        try:
            diagnostics = json.loads(msg.diagnostics_json or '{}')
        except (TypeError, ValueError):
            diagnostics = {}
        if not msg.robot1_canonical_task_id and not msg.robot2_canonical_task_id:
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
        self.round_outcomes[outcome] += 1
        self.event('DISTRIBUTED_PAIR_DECISION','replicated complete pair decision',r,f'/{r}/pair_decision',source_stamp=stamp(msg),source_session_id=self.uuid_text(msg.source_session_id),round_id=msg.round_id,union_hash=msg.union_hash,robot1_snapshot_epoch=msg.robot1_snapshot_epoch,robot2_snapshot_epoch=msg.robot2_snapshot_epoch,robot1_bid_fingerprint=msg.robot1_bid_fingerprint,robot2_bid_fingerprint=msg.robot2_bid_fingerprint,robot1_task=msg.robot1_canonical_task_id or 'IDLE',robot2_task=msg.robot2_canonical_task_id or 'IDLE',decision_hash=msg.decision_hash,total_team_score=msg.total_team_score,team_visible_gain=msg.team_visible_gain,combined_path_cost=msg.combined_path_cost,nearby_goal_penalty=msg.nearby_goal_penalty,route_overlap_penalty=msg.route_overlap_penalty,hard_failure_penalty=msg.hard_failure_penalty,sensing_overlap_penalty=msg.sensing_overlap_penalty,workload_imbalance_penalty=msg.workload_imbalance_penalty,coordinator_state=msg.coordinator_state,decision_diagnostics_json=msg.diagnostics_json,decision_outcome=outcome,decision_idle_reason=diagnostics.get('idle_reason'),decision_availability_reason=diagnostics.get('availability_reason'))
    def distributed_status(self,r,msg):
        self.mark(r,'distributed_status',msg)
        self.latest[r]['distributed_state']=msg.state
        distributed_states = {
            DistributedExplorationStatus.WAITING_FOR_INPUTS: 'WAITING_FOR_INPUTS',
            DistributedExplorationStatus.BIDDING: 'BIDDING',
            DistributedExplorationStatus.WAITING_FOR_MATCHING_DECISION: 'WAITING_FOR_MATCHING_DECISION',
            DistributedExplorationStatus.WAITING_FOR_TRAFFIC: 'WAITING_FOR_TRAFFIC',
            DistributedExplorationStatus.NAVIGATING: 'NAVIGATING',
            DistributedExplorationStatus.DEGRADED_SOLO: 'DEGRADED_SOLO',
            DistributedExplorationStatus.COMPLETE: 'COMPLETE',
            DistributedExplorationStatus.BLOCKED: 'BLOCKED',
        }
        self.latest[r]['claim_state'] = distributed_states.get(
            msg.state, str(msg.state),
        )
        self.latest[r]['navigation_active'] = bool(msg.local_nav_goal_active)
        self.latest[r].update(
            terminal=bool(msg.terminal),
            terminal_reason=msg.terminal_reason,
            terminal_epoch=int(msg.terminal_epoch),
            terminal_map_revision=int(msg.terminal_map_revision),
            remaining_frontier_count=int(msg.remaining_frontier_count),
            remaining_small_frontier_count=int(msg.remaining_small_frontier_count),
            remaining_out_of_range_count=int(msg.remaining_out_of_range_count),
            remaining_unreachable_count=int(msg.remaining_unreachable_count),
            planner_failure_count=int(msg.planner_failure_count),
            detected_not_queried_count=int(
                getattr(msg, 'detected_not_queried_count', 0)),
            below_minimum_gain_count=int(
                getattr(msg, 'below_minimum_gain_count', 0)),
            actionable_reachable_count=int(
                getattr(msg, 'actionable_reachable_count', 0)),
        )
        health=(msg.nav2_healthy,msg.tf_healthy,msg.candidate_source_healthy,msg.peer_communication_healthy)
        if not self.distributed_changed((r,'status'),(msg.state,msg.round_id,msg.decision_hash,msg.active_canonical_task_id,health,msg.reason)):return
        self.event('DISTRIBUTED_STATUS',msg.reason,r,f'/{r}/distributed_status',source_stamp=stamp(msg),source_session_id=self.uuid_text(msg.source_session_id),state=msg.state,round_id=msg.round_id,union_hash=msg.union_hash,decision_hash=msg.decision_hash,active_canonical_task_id=msg.active_canonical_task_id,local_nav_goal_active=msg.local_nav_goal_active,nav2_healthy=msg.nav2_healthy,tf_healthy=msg.tf_healthy,candidate_source_healthy=msg.candidate_source_healthy,peer_communication_healthy=msg.peer_communication_healthy,terminal=msg.terminal,terminal_reason=msg.terminal_reason,terminal_epoch=msg.terminal_epoch,remaining_frontier_count=msg.remaining_frontier_count,remaining_small_frontier_count=msg.remaining_small_frontier_count,remaining_out_of_range_count=msg.remaining_out_of_range_count,remaining_unreachable_count=msg.remaining_unreachable_count,planner_failure_count=msg.planner_failure_count,detected_not_queried_count=getattr(msg,'detected_not_queried_count',0),below_minimum_gain_count=getattr(msg,'below_minimum_gain_count',0),actionable_reachable_count=getattr(msg,'actionable_reachable_count',0))
        if msg.terminal:
            self.mission_terminal_reason = msg.terminal_reason or msg.reason
        if msg.state == DistributedExplorationStatus.COMPLETE and self.mission_completion_time is None:
            self.mission_completion_time = self.ros_seconds() - self.start_ros
    def distributed_event(self,r,msg):
        self.mark(r,'distributed_event',msg)
        if msg.event_type == 'DECISION_AGREED':
            self.agreement_publications += 1
            self.unique_agreed_rounds.add(msg.round_id)
            if msg.decision_hash:
                self.unique_agreed_decisions.add(msg.decision_hash)
        elif msg.event_type == 'NAV_GOAL_SENT':
            self.dispatch_attempts += 1
        elif msg.event_type in ('NAVIGATION_SUCCEEDED', 'NAVIGATION_FAILED', 'NAVIGATION_CANCELLED'):
            self.goals_terminal += 1
        fields = dict(source_session_id=self.uuid_text(msg.source_session_id),
                      round_id=msg.round_id, union_hash=msg.union_hash,
                      decision_hash=msg.decision_hash,
                      canonical_task_id=msg.canonical_task_id,
                      physical_task_signature=msg.physical_task_signature,
                      previous_state=msg.previous_state,
                      next_state=msg.next_state,
                      path_length_m=msg.path_length_m,
                      travelled_distance_m=msg.travelled_distance_m,
                      navigation_duration_s=msg.navigation_duration_s,
                      newly_discovered_cells=msg.newly_discovered_cells,
                      peer_first_discovered_cells=msg.peer_first_discovered_cells,
                      duplicated_cells=msg.duplicated_cells,
                      route_overlap_score=msg.route_overlap_score,
                      sensing_overlap_estimate=msg.sensing_overlap_estimate,
                      result=msg.result, failure_class=msg.failure_class,
                      recoveries=msg.recoveries,
                      nav2_error_code=msg.nav2_error_code,
                      nav2_error_message=msg.nav2_error_message)
        source = f'/{r}/distributed_event'
        self.event(msg.event_type, msg.reason, r, source,
                   source_stamp=stamp(msg), **fields)
        # The replicated executor reports the local action acceptance as a
        # state-transition event rather than using the legacy claim topic.
        # Normalize that protocol event so navigation telemetry retains the
        # accepted-goal count used by the existing report schema.
        if (msg.event_type == 'STATE_TRANSITION' and
                msg.reason == 'local agreed goal accepted for send'):
            self.event('NAV_GOAL_ACCEPTED', msg.reason, r, source,
                       source_stamp=stamp(msg), **fields)
    def distributed_failure(self,r,msg):
        self.mark(r,'exploration_failure',msg)
        self.event('DISTRIBUTED_TASK_FAILURE',msg.evidence,r,f'/{r}/exploration_failure',source_stamp=stamp(msg),severity='WARN',source_session_id=self.uuid_text(msg.source_session_id),round_id=msg.round_id,canonical_task_id=msg.canonical_task_id,physical_task_signature=msg.physical_task_signature,approach=[msg.approach_pose.pose.position.x,msg.approach_pose.pose.position.y],failure_class=msg.failure_class,path_length_m=msg.path_length_m,path_samples=[[point.x,point.y] for point in msg.path_samples],retry_count=msg.retry_count,nav2_error_code=msg.nav2_error_code,nav2_error_message=msg.nav2_error_message)
    def claim_fields(self,msg):
        return {'source_session_id':bytes(msg.source_session_id.uuid).hex(),'message_revision':msg.message_revision,'claim_id':msg.claim_id,'frontier_id':msg.frontier_id,'map_revision':msg.map_revision,'claim_state':STATES.get(msg.state,str(msg.state)),'frontier_centroid_x':msg.frontier_centroid.x,'frontier_centroid_y':msg.frontier_centroid.y,'approach_x':msg.approach_pose.pose.position.x,'approach_y':msg.approach_pose.pose.position.y,'approach_yaw':yaw(msg.approach_pose.pose.orientation),'path_length_m':msg.path_length_m,'information_gain':msg.information_gain,'utility_score':msg.utility_score,'release_reason':msg.state_reason}
    def claim(self,r,msg):
        self.mark(r,'exploration_claim',msg); old=self.claims.get(r); self.claims[r]=msg
        if old is not None and (old.state,old.claim_id,old.frontier_id,bytes(old.source_session_id.uuid))==(msg.state,msg.claim_id,msg.frontier_id,bytes(msg.source_session_id.uuid)):
            return
        state=STATES.get(msg.state,str(msg.state)); d=self.latest[r]; d.update(claim_state=state,claim_id=msg.claim_id,frontier_id=msg.frontier_id,goal=(msg.approach_pose.pose.position.x,msg.approach_pose.pose.position.y,yaw(msg.approach_pose.pose.orientation))); fields=self.claim_fields(msg); source=f'/cslam/{r}/exploration_claim'
        if old and bytes(old.source_session_id.uuid)!=bytes(msg.source_session_id.uuid):self.event('PEER_SESSION_CHANGED','source session changed',r,source,**fields)
        if msg.state==ExplorationClaim.PROPOSING:
            self.event('CLAIM_PROPOSED',msg.state_reason or 'frontier proposed',r,source,console=True,**fields)
            peer=next((p for name,p in self.claims.items() if name!=r and p.state in (ExplorationClaim.PROPOSING,ExplorationClaim.NAVIGATING)),None)
            if peer:
                a={'centroid_x':msg.frontier_centroid.x,'centroid_y':msg.frontier_centroid.y,'min_x':msg.bounding_box_min.x,'max_x':msg.bounding_box_max.x,'min_y':msg.bounding_box_min.y,'max_y':msg.bounding_box_max.y}; b={'centroid_x':peer.frontier_centroid.x,'centroid_y':peer.frontier_centroid.y,'min_x':peer.bounding_box_min.x,'max_x':peer.bounding_box_max.x,'min_y':peer.bounding_box_min.y,'max_y':peer.bounding_box_max.y}
                if equivalent_frontiers(a,b):
                    self.event('EQUIVALENT_FRONTIER_DUPLICATE','production-equivalent simultaneous claims',r,source,peer_robot_id=peer.source_robot_id); self.event('CLAIM_CONFLICT_DETECTED','equivalent active peer claim observed',r,source,peer_robot_id=peer.source_robot_id)
        elif msg.state==ExplorationClaim.NAVIGATING:
            self.event('ARBITRATION_WON',msg.state_reason or 'proposal proceeded',r,source,console=True,arbitration_result='WON',arbitration_reason=msg.state_reason,**fields); self.event('NAV_GOAL_SENT','dispatch inferred from coordinator transition',r,source,console=True,goal_x=fields['approach_x'],goal_y=fields['approach_y'],goal_yaw=fields['approach_yaw'],**fields); self.event('NAV_GOAL_ACCEPTED',msg.state_reason or 'goal accepted',r,source,console=True,**fields); d['navigation_active']=True
            for name,peer in self.claims.items():
                if name!=r and peer.state==ExplorationClaim.NAVIGATING and duplicate_goal((fields['approach_x'],fields['approach_y']),(peer.approach_pose.pose.position.x,peer.approach_pose.pose.position.y),self.p['duplicate_goal_tolerance_m']):self.event('DUPLICATE_GOAL_REGION','active goals within tolerance',r,source,peer_robot_id=name)
        elif msg.state in (ExplorationClaim.RELEASED,ExplorationClaim.CANCELED):
            kind='ARBITRATION_LOST' if 'arbitration' in msg.state_reason.lower() else ('NAVIGATION_CANCELED' if msg.state==ExplorationClaim.CANCELED else 'CLAIM_RELEASED'); self.event(kind,msg.state_reason or state,r,source,console=True,arbitration_result='LOST' if kind=='ARBITRATION_LOST' else None,**fields); d['navigation_active']=False
        elif msg.state in (ExplorationClaim.SUCCEEDED,ExplorationClaim.FAILED):
            kind='NAVIGATION_SUCCEEDED' if msg.state==ExplorationClaim.SUCCEEDED else ('NAV_GOAL_REJECTED' if 'reject' in msg.state_reason.lower() else ('NAVIGATION_TIMEOUT' if 'timeout' in msg.state_reason.lower() else 'NAVIGATION_FAILED')); self.event(kind,msg.state_reason or state,r,source,severity='ERROR' if msg.state==ExplorationClaim.FAILED else 'INFO',console=True,failure_reason=self.classify(msg.state_reason) if msg.state==ExplorationClaim.FAILED else None,**fields); d['navigation_active']=False
        if msg.state!=ExplorationClaim.NAVIGATING:
            for kind in self.detectors[r].reset():self.event(kind,'navigation no longer active',r)
    def classify(self,reason):
        text=reason.lower()
        for key,value in [('server','ACTION_SERVER_UNAVAILABLE'),('reject','GOAL_REJECTED'),('planner','PLANNER_FAILURE'),('controller','CONTROLLER_FAILURE'),('transform','TF_FAILURE'),('tf','TF_FAILURE'),('costmap','COSTMAP_FAILURE'),('collision','COLLISION_MONITOR_BLOCK'),('timeout','NAVIGATION_TIMEOUT'),('arbitration','CANCELED_BY_ARBITRATION')]:
            if key in text:return value
        return 'UNKNOWN_NAV2_FAILURE'
    def status(self,r,msg):
        self.mark(r,'exploration_status',msg); old=self.statuses.get(r); self.statuses[r]=msg; state=STATUS_STATES.get(msg.state,str(msg.state)); now=self.ros_seconds()
        self.latest[r]['exploration_status']=state
        if old is None or old.state!=msg.state:
            self.event('EXPLORATION_STATUS_CHANGED',state,r,f'/cslam/{r}/exploration_status',source_stamp=stamp(msg),status_state=state,status_reason=msg.reason,status_revision=msg.message_revision,candidate_count=msg.candidate_count,eligible_candidate_count=msg.eligible_candidate_count)
        if msg.state==ExplorationStatus.NO_ELIGIBLE_CANDIDATES and self.exhausted_since[r] is None:self.exhausted_since[r]=now
        elif msg.state!=ExplorationStatus.NO_ELIGIBLE_CANDIDATES and self.exhausted_since[r] is not None:
            self.exhausted_duration[r]+=now-self.exhausted_since[r]; self.exhausted_since[r]=None
        if msg.state==ExplorationStatus.COMPLETE and self.mission_completion_time is None:self.mission_completion_time=now-self.start_ros
    def coordinator_event(self,r,msg):
        self.mark(r,'exploration_event',msg); fields={'cycle_number':msg.cycle_number,'claim_id':msg.claim_id,'frontier_id':msg.frontier_id,'candidate_map_revision':msg.candidate_map_revision,'selected_rank':msg.selected_rank,'path_length_m':msg.path_length_m,'information_gain':msg.information_gain,'goal_x':msg.goal_pose.pose.position.x,'goal_y':msg.goal_pose.pose.position.y,'goal_yaw':yaw(msg.goal_pose.pose.orientation),'terminal_result':msg.terminal_result,'duration_s':msg.duration_s,'suppression_reason':msg.reason}
        if msg.event_type=='EXPLORATION_CYCLE_STARTED':
            self.cycle_starts[(r,msg.claim_id)]=(self.ros_seconds(),self.trajectory.total_distance.get(r,0.)); self.region_attempts[r][msg.frontier_id]+=1
        if msg.event_type=='EXPLORATION_CYCLE_ENDED':
            start=self.cycle_starts.pop((r,msg.claim_id),None)
            if start:
                fields['duration_s']=self.ros_seconds()-start[0]; fields['actual_travelled_distance_m']=self.trajectory.total_distance.get(r,0.)-start[1]; self.cycle_durations[r].append(fields['duration_s'])
        self.robot_counts[r][msg.event_type]+=1
        if msg.event_type=='MISSION_COMPLETE' and self.mission_completion_time is None:self.mission_completion_time=self.ros_seconds()-self.start_ros
        self.event(msg.event_type,msg.reason or msg.terminal_result or msg.event_type,r,f'/cslam/{r}/exploration_event',source_stamp=stamp(msg),**fields)
    def feedback(self,r,msg):
        self.mark(r,'navigate_feedback',msg); f=msg.feedback; old=self.latest[r].get('recoveries',0); self.latest[r].update(distance_remaining=float(f.distance_remaining),recoveries=int(f.number_of_recoveries))
        if old!=f.number_of_recoveries:self.event('RECOVERY_COUNT_CHANGED',f'{old} -> {f.number_of_recoveries}',r,f'/{r}/navigate_to_pose/_action/feedback',source_stamp=stamp(f.current_pose),recoveries=f.number_of_recoveries,distance_remaining_m=f.distance_remaining)
    def rosout(self,msg):
        if msg.name.lstrip('/')=='cooperative_experiment_logger':return
        text=msg.name+' '+msg.msg
        lower=text.lower()
        diagnostic_rules=(
            ('TF_FAILURE',r'unable to transform robot pose into global plan|transform.*global plan|tf error|lookup would require'),
            ('MISSED_RATE_WARNING',r'missed its desired rate|current loop rate'),
            ('FOLLOW_PATH',r'\[follow_path\]|followpath'),
            ('COMPUTE_PATH',r'compute_path_to_pose|computepathtopose'),
            ('RECOVERY',r'recovery|clear_(local|global|entirely)|\bspin\b|\bback.?up\b|\bwait\b'),
            ('COLLISION_MONITOR',r'collision.?monitor|stop.?zone|emergency stop'),
        )
        diagnostic_category=next((category for category,pattern in diagnostic_rules if re.search(pattern,lower)),None)
        if 'COMPUTE_PATH_REUSED' in msg.msg:
            source_match = re.search(r'source=([A-Z0-9_]+)', msg.msg)
            source = source_match.group(1) if source_match else 'UNKNOWN'
            self.planner_query_counts[f'{source}.REUSED'] += 1
        elif 'COMPUTE_PATH_RESULT' in msg.msg or 'CANDIDATE_PATH_RESULT' in msg.msg:
            source_match = re.search(r'source=([A-Z0-9_]+)', msg.msg)
            source = source_match.group(1) if source_match else 'UNKNOWN'
            if re.search(r'\bok=true\b|\bvalid=true\b', msg.msg):
                outcome = 'SUCCESS'
            elif re.search(r'TIMEOUT|timeout', msg.msg):
                outcome = 'TIMEOUT'
            else:
                outcome = 'FAILURE'
            self.planner_query_counts[f'{source}.{outcome}'] += 1
            duration_match = re.search(r'duration_s=([0-9]+(?:\.[0-9]+)?)', msg.msg)
            if duration_match:
                self.planner_query_duration_s[source] += float(
                    duration_match.group(1))
        controller_or_planner=(re.search(r'controller|planner',lower) and
                               re.search(r'failed|failure|abort|progress checker|no valid control|timeout',lower))
        if controller_or_planner and diagnostic_category is None:
            diagnostic_category='CONTROLLER_OR_PLANNER_ERROR'
        if diagnostic_category is not None:
            source_stamp=(msg.stamp.sec,msg.stamp.nanosec)
            row=self.common('/rosout:'+msg.name,source_stamp=source_stamp)
            row.update(severity='ERROR' if msg.level>=Log.ERROR else ('WARN' if msg.level>=Log.WARN else 'INFO'),category=diagnostic_category,message=msg.msg,node=msg.name)
            key=(msg.name,diagnostic_category,msg.msg); previous=self._diagnostic_last.get(key); now=row['elapsed_s']
            if (previous is None or now-previous>=0.25) and self.nav2_diagnostic_count<10000:
                self._diagnostic_last[key]=now; self.nav2_diagnostic_count+=1
                try:
                    with self._io_lock:self.nav2_diagnostics.write(json.dumps(finite(row),separators=(',',':'),allow_nan=False)+'\n')
                except (OSError,TypeError,ValueError) as exc:self.write_failures+=1; self.get_logger().error(f'Nav2 diagnostic write failed: {exc}',throttle_duration_sec=10.)
        if msg.level<Log.WARN:return
        severity='ERROR' if msg.level>=Log.ERROR else 'WARN'; category=next((v for k,v in [('costmap','COSTMAP_WARNING'),('controller','CONTROLLER_WARNING'),('slam','SLAM_WARNING'),('scan','SCAN_WARNING'),('transform','TF_WARNING'),(' tf','TF_WARNING')] if k in lower),'PROCESS_WARNING')
        with self._state_lock:record,new=self.warns.add(msg.name,severity,msg.msg,utc_now(),category)
        if new:self.event(category,msg.msg,source='/rosout:'+msg.name,severity=severity,source_stamp=(msg.stamp.sec,msg.stamp.nanosec),occurrence_count=1)
    def row_time(self):
        sec,nsec=self.ros_now()
        with self._state_lock:self.sequence+=1; sequence=self.sequence
        return {'run_id':self.run_id,'wall_time_utc':utc_now(),'ros_time_sec':sec,'ros_time_nanosec':nsec,'elapsed_s':self.ros_seconds()-self.start_ros,'wall_elapsed_s':time.monotonic()-self.start,'event_sequence':sequence}
    def map_snapshot(self,r,key):
        with self._state_lock:msg=self.latest[r].get(key)
        if msg is None:return None
        cache_key=(r,key); identity=id(msg); cached=self._map_cache.get(cache_key)
        if cached is not None and cached[0]==identity:return cached
        grid=as_grid(msg); counts=known_counts(grid); digest=hashlib.sha256(grid.data.tobytes()).digest()
        cached=(identity,grid,counts,digest,msg); self._map_cache[cache_key]=cached; return cached
    def map_counts(self,r,key):
        snapshot=self.map_snapshot(r,key)
        if snapshot is None:return 0,0,0
        free,occupied,unknown=snapshot[2]; return free+occupied,free,occupied
    def sample_telemetry(self):
        if not self.stack_ready and all(all(k in self.latest[r] for k in ('odom','map','shared_map','frontier_candidates')) for r in self.robots):
            self.stack_ready=True
            self.event('STACK_READY','critical robot telemetry is available',console=True)
        for r in self.robots:
            d=self.latest[r]; pose=d.get('pose',(None,None,None)); speed=d.get('speed',(0.,0.)); command=d.get('command',(0.,0.)); goal=d.get('goal',(None,None,None)); cmd_age=self.age(r,'cmd_vel'); cmd_received=cmd_age is not None and cmd_age<=float(self.p['cmd_vel_no_command_timeout_s']); row=self.row_time(); row.update(robot_id=r,pose_x=pose[0],pose_y=pose[1],pose_yaw=pose[2],linear_speed_mps=speed[0],angular_speed_radps=speed[1],commanded_linear_mps=command[0],commanded_angular_radps=command[1],cmd_vel_received=cmd_received,cmd_vel_age_s=cmd_age,cmd_vel_source=d.get('cmd_vel_source'),distance_travelled_m=self.trajectory.total_distance.get(r,0.),claim_state=d.get('claim_state','UNKNOWN'),claim_id=d.get('claim_id'),frontier_id=d.get('frontier_id'),goal_x=goal[0],goal_y=goal[1],goal_yaw=goal[2],navigation_active=d.get('navigation_active',False),distance_remaining_m=d.get('distance_remaining'),recoveries=d.get('recoveries',0),candidate_count=d.get('candidate_count',0),local_known_cells=self.map_counts(r,'map')[0],shared_known_cells=self.map_counts(r,'shared_map')[0],local_costmap_obstacles=self.map_counts(r,'local_costmap/costmap')[2],global_costmap_known=self.map_counts(r,'global_costmap/costmap')[0],global_costmap_obstacles=self.map_counts(r,'global_costmap/costmap')[2],odom_age_s=self.age(r,'odom'),scan_age_s=self.age(r,'scan_d500_slam'),map_age_s=self.age(r,'map'),shared_map_age_s=self.age(r,'shared_map'),claim_age_s=self.age(r,'exploration_claim'),feedback_age_s=self.age(r,'navigate_feedback')); self.csv_row(self.writers[r],row)
            if pose[0] is not None:
                sample=MotionSample(row['elapsed_s'],pose[0],pose[1],d.get('distance_remaining'),command[0],command[1]); near=d.get('distance_remaining') is not None and d['distance_remaining']<.08
                for kind in self.detectors[r].update(sample,d.get('navigation_active',False),near_goal=near):
                    self.event(kind,'windowed passive detector changed state',r,distance_remaining_m=d.get('distance_remaining'),pose_x=pose[0],pose_y=pose[1])
                    if kind in ('STUCK_STARTED','OSCILLATION_STARTED') and self.map_counts(r,'local_costmap/costmap')[2]>0:self.event('CORNER_TRAP_SUSPECTED','motion anomaly with nearby costmap obstacles',r,severity='WARN')
                if d.get('navigation_active') and row['elapsed_s']-self.last_progress.get(r,-99)>=5:
                    self.last_progress[r]=row['elapsed_s']; self.event('NAVIGATION_PROGRESS','periodic low-rate progress sample',r,distance_remaining_m=d.get('distance_remaining'),pose_x=pose[0],pose_y=pose[1],distance_travelled_m=self.trajectory.total_distance.get(r,0.),recoveries=d.get('recoveries',0))
    def sample_coverage(self):
        shared=[self.map_snapshot(r,'shared_map') for r in self.robots]; local=[self.map_snapshot(r,'map') for r in self.robots]
        if not all(shared):return
        counts=[snapshot[2] for snapshot in shared]; maps=[snapshot[4] for snapshot in shared]; known=[a+b for a,b,_ in counts]; current=max(known); self.initial_known=current if self.initial_known is None else self.initial_known; gain=current-(self.previous_known if self.previous_known is not None else current); self.previous_known=current
        equivalent=(maps[0].info.width,maps[0].info.height,maps[0].info.resolution,maps[0].info.origin)==(maps[1].info.width,maps[1].info.height,maps[1].info.resolution,maps[1].info.origin) and shared[0][3]==shared[1][3]
        if equivalent:self.divergence_since=None; self.divergence_reported=False
        elif self.divergence_since is None:self.divergence_since=time.monotonic()
        elif not self.divergence_reported and time.monotonic()-self.divergence_since>=self.p['shared_map_divergence_grace_s']:
            self.divergence_reported=True; self.event('SHARED_MAP_DIVERGENCE','independent shared maps differ beyond grace period',severity='WARN')
        transforms={
            'robot1': (0., 0., 0.),
            'robot2': tuple(self.p['known_relative_transform']),
        }
        if self.p['enable_coverage_attribution']:
            for r,snapshot in zip(self.robots,local):
                if snapshot and self._last_attributed.get(r)!=snapshot[0]:
                    transform=transforms.get(r,(0.,0.,0.)); transformed=self._transformed_cache.get(r)
                    if transformed is None or transformed[0]!=snapshot[0]:
                        transformed=(snapshot[0],known_world_cells(snapshot[1],self.p['coverage_attribution_resolution'],transform)); self._transformed_cache[r]=transformed
                    self.attribution.observe(r,transformed[1],time.monotonic()-self.start); self._last_attributed[r]=snapshot[0]
        a=self.attribution.summary(); row=self.row_time(); row.update(robot1_local_known=self.map_counts('robot1','map')[0],robot2_local_known=self.map_counts('robot2','map')[0],robot1_shared_known=known[0],robot2_shared_known=known[1],shared_free_cells=counts[0][0],shared_occupied_cells=counts[0][1],shared_unknown_cells=counts[0][2],known_area_m2=known[0]*maps[0].info.resolution**2,coverage_gain_cells=gain,coverage_gain_since_start_cells=current-self.initial_known,unique_first_seen_robot1_cells=a['unique_first_seen_cells'].get('robot1',0),unique_first_seen_robot2_cells=a['unique_first_seen_cells'].get('robot2',0),later_duplicated_by_robot1_cells=a['later_duplicated_cells'].get('robot1',0),later_duplicated_by_robot2_cells=a['later_duplicated_cells'].get('robot2',0),simultaneously_observed_cells=a['simultaneously_observed_cells'],total_known_union_cells=a['total_known_union_cells'],duplicated_known_fraction=a['duplicated_known_fraction'],shared_maps_equivalent=equivalent); self.csv_row(self.coverage,row)
    def sample_health(self):
        limits={'odom':self.p['odom_stale_s'],'joint_states':self.p['odom_stale_s'],'scan_d500_fixed':self.p['scan_stale_s'],'scan_d500_slam':self.p['scan_stale_s'],'map':self.p['map_stale_s'],'peer_map':self.p['map_stale_s'],'shared_map':self.p['shared_map_stale_s'],'frontier_candidates':self.p['candidate_stale_s'],'exploration_claim':self.p['claim_stale_s'],'exploration_status':self.p['status_stale_s'],'navigate_feedback':self.p['feedback_stale_s'],'local_costmap/costmap':self.p['costmap_stale_s'],'global_costmap/costmap':self.p['costmap_stale_s'],'cmd_vel':2.}
        now=time.monotonic()
        for r in self.robots:
            available=self.tf_buffer.can_transform(self.p['global_frame'],f'{r}/base_footprint',Time(),timeout=Duration(seconds=0.0))
            old_tf=self.tf_state.get(r)
            if self.stack_ready and old_tf!=available:
                self.event('TOPIC_RECOVERED' if available else 'TF_WARNING','shared-frame robot transform available' if available else 'shared-frame robot transform unavailable',r,'/tf',severity='INFO' if available else 'WARN',topic_name='/tf')
            self.tf_state[r]=available
            for key,limit in limits.items():
                window=self.windows.setdefault((r,key),deque())
                while window and window[0]<now-10:window.popleft()
                age=self.age(r,key); stale=age is None or age>limit; old=self.stale.get((r,key)); active=self.latest[r].get('navigation_active',False)
                if old is not None and stale!=old and (key!='navigate_feedback' or active):self.event('TOPIC_STALE' if stale else 'TOPIC_RECOVERED',f'{key} age={age}',r,f'/{r}/{key}',severity='WARN' if stale else 'INFO',topic_name=f'/{r}/{key}',topic_age_s=age)
                self.stale[(r,key)]=stale; row=self.row_time(); row.update(robot_id=r,topic_name=f'/{r}/{key}',topic_rate_hz=len(window)/10.,topic_age_s=age,expected_min_rate_hz=1/limit,stale=stale); self.csv_row(self.health,row)
    def sample_process_resources(self):
        now=time.monotonic(); fields=Path('/proc/self/stat').read_text().split(); ticks=int(fields[13])+int(fields[14]); rss=int(Path('/proc/self/statm').read_text().split()[1])*os.sysconf('SC_PAGE_SIZE')
        previous=self._cpu_previous; self._cpu_previous=(now,ticks); self._rss_samples.append(rss)
        if now-self.start<10. or previous is None:return
        elapsed=now-previous[0]
        if elapsed>0:self._cpu_samples.append((ticks-previous[1])/os.sysconf('SC_CLK_TCK')/elapsed*100.)
    def console(self):
        if not self.p['enable_console_status']:return
        for r in self.robots:
            d=self.latest[r]; p=d.get('pose',(None,None,None)); pose='?,?' if p[0] is None else f'{p[0]:.2f},{p[1]:.2f}'; self.get_logger().info(f'[{time.monotonic()-self.start:.1f}s][{r}] {d.get("claim_state","WAITING")} claim={d.get("claim_id","-")} frontier={d.get("frontier_id","-")} pose=({pose}) remaining={d.get("distance_remaining","-")} candidates={d.get("candidate_count",0)}')
    def flush(self):
        try:
            with self._io_lock:
                if self._closed:return
                self.events.flush()
                for f in self.files:f.flush()
        except OSError as e:self.write_failures+=1; self.get_logger().error(f'flush failed: {e}',throttle_duration_sec=10.)
        if self.forensic is not None:
            try:self.forensic.flush()
            except OSError as e:self.write_failures+=1; self.get_logger().error(f'forensic flush failed: {e}',throttle_duration_sec=10.)

    def required_artifact_status(self, include_campaign_files=True):
        """Return the fail-closed artifact contract for this validation."""
        required=[]
        if include_campaign_files:
            required.extend([self.directory/'summary.json',self.directory/'mission_result.json',self.directory/'run_manifest.json'])
        if self.forensic is not None:
            required.extend([
                self.directory/'forensic'/'transforms.csv',
                self.directory/'forensic'/'supervisor_ground_truth.csv',
                self.directory/'forensic'/'maps'/'robot1_map_final.npz',
                self.directory/'forensic'/'maps'/'robot2_map_final.npz',
            ])
            # Shared-map exports are a post-handoff contract.  A valid
            # no-handoff run must not be marked incomplete merely because
            # those files correctly do not exist.  If either shared-map topic
            # was observed, require both final shared exports so a partial
            # handoff still fails closed.
            if self.shared_map_seen:
                required.extend([
                    self.directory/'forensic'/'maps'/'robot1_shared_map_final.npz',
                    self.directory/'forensic'/'maps'/'robot2_shared_map_final.npz',
                ])
        try:
            unknown_pose = bool(json.loads(
                self.p['initial_configuration_json']).get(
                    'unknown_initial_pose', False))
        except (TypeError, ValueError, json.JSONDecodeError):
            unknown_pose = False
        frontend_directory = None
        if unknown_pose:
            frontend_directory = self.frontend_diagnostic_directory()
            for robot in self.robots:
                required.extend([
                    frontend_directory / f'{robot}_unknown_pose_frontend.json',
                    frontend_directory / f'{robot}_consensus_diagnostics.jsonl',
                    frontend_directory / f'{robot}_physical_evidence_diagnostics.jsonl',
                ])
        def logical_name(path):
            if frontend_directory is not None and path.parent == frontend_directory:
                return f'frontend/{path.name}'
            try:
                return str(path.relative_to(self.directory))
            except ValueError:
                return str(path)
        missing=[logical_name(path) for path in required if not path.is_file()]
        result = {
            'complete': not missing,
            'status': 'COMPLETE' if not missing else 'MISSING_REQUIRED_ARTIFACTS',
            'required': [logical_name(path) for path in required],
            'missing': missing,
        }
        if frontend_directory is not None:
            try:
                result['frontend_directory'] = str(
                    frontend_directory.relative_to(self.directory.parent))
            except ValueError:
                result['frontend_directory'] = str(frontend_directory)
        if self.scan_matching_enabled and self.forensic is not None:
            required.extend([
                self.directory / 'forensic' / 'scan_matching' /
                f'{robot}_corrections.jsonl' for robot in self.robots])
            missing = [logical_name(path) for path in required
                       if not path.is_file()]
            result['required'] = [logical_name(path) for path in required]
            result['missing'] = missing
            result['complete'] = not missing
            result['status'] = 'COMPLETE' if not missing else 'MISSING_REQUIRED_ARTIFACTS'
        return result

    def frontend_diagnostic_directory(self):
        """Resolve frontend artifacts from the manifest-owned run directory.

        The launch graph can create the frontend output directory before the
        logger allocates its own collision-safe run directory.  In that case
        the logger receives ``run_id-01`` while frontend files remain under
        ``run_id/frontend``.  Resolve both locations within the same campaign
        root instead of manufacturing a missing-artifact failure.
        """
        direct = self.directory / 'frontend'
        candidates = [direct]
        parent = self.directory.parent
        # The launch runner deliberately creates the frontend directory before
        # the logger allocates its collision-safe run directory.  In that
        # normal case the files live directly under the observer root (for
        # example ``observer/frontend``), while ``self.directory`` is
        # ``observer/<run-id>``.  Include that root-level location explicitly;
        # treating only run-directory siblings as candidates makes a valid
        # frontend artifact set look missing at shutdown.
        candidates.append(parent / 'frontend')
        try:
            siblings = sorted(parent.iterdir(), key=lambda path: path.name)
        except OSError:
            siblings = []
        for sibling in siblings:
            # The launch runner may allocate the logger directory with a
            # collision suffix while the frontend keeps the manifest-owned
            # unsuffixed run directory.  Both are campaign-owned siblings
            # below this observer root; select only directories that actually
            # contain the required frontend contract.
            if sibling.is_dir() and sibling != self.directory:
                candidates.append(sibling / 'frontend')
        required_names = [
            f'{robot}_{suffix}'
            for robot in self.robots
            for suffix in (
                'unknown_pose_frontend.json',
                'consensus_diagnostics.jsonl',
                'physical_evidence_diagnostics.jsonl')]
        return max(
            candidates,
            key=lambda path: (
                sum((path / name).is_file() for name in required_names),
                int(path == direct)),
        )

    def wait_for_frontend_diagnostics(self, timeout_s=30.0):
        """Allow frontend SIGINT handlers to finish before final validation."""
        try:
            unknown_pose = bool(json.loads(
                self.p['initial_configuration_json']).get(
                    'unknown_initial_pose', False))
        except (TypeError, ValueError, json.JSONDecodeError):
            unknown_pose = False
        if not unknown_pose:
            return True
        deadline = time.monotonic() + max(0.0, float(timeout_s))
        names = [
            f'{robot}_{suffix}'
            for robot in self.robots
            for suffix in (
                'unknown_pose_frontend.json',
                'consensus_diagnostics.jsonl',
                'physical_evidence_diagnostics.jsonl')]
        while time.monotonic() < deadline:
            directory = self.frontend_diagnostic_directory()
            if all((directory / name).is_file() for name in names):
                return True
            time.sleep(0.25)
        return all(
            (self.frontend_diagnostic_directory() / name).is_file()
            for name in names)

    @staticmethod
    def runtime_worktree():
        """Find the checkout that supplied this running package."""
        candidates = [Path(__file__).resolve(), Path.cwd().resolve()]
        for candidate in candidates:
            for parent in (candidate, *candidate.parents):
                if (parent / '.git').exists():
                    return parent
        return None

    def git_value(self,args,default):
        try:
            root = self.runtime_worktree()
            if root is None:
                return default
            return subprocess.check_output(
                ['git', *args], cwd=str(root), text=True,
                stderr=subprocess.DEVNULL).strip()
        except Exception:
            return default
    def write_manifest(self,clean,status):
        value={
            'schema_version':SCHEMA,'run_id':self.run_id,
            'utc_start_time':self.start_utc,
            'utc_end_time':utc_now() if status!='running' else None,
            'elapsed_duration_s':time.monotonic()-self.start,
            'runtime_worktree': str(self.runtime_worktree() or ''),
            'git_commit':self.git_value(['rev-parse','HEAD'],'unknown'),
            'git_branch':self.git_value(
                ['symbolic-ref','--short','-q','HEAD'], 'DETACHED'),
            'worktree_dirty':bool(self.git_value(['status','--porcelain'],'')),
            'launch_file':self.p['launch_file'],
            'launch_arguments':'recorded in logger parameters',
            'world_profile':self.p['world_profile'],
            'world_resource':self.p['installed_world_path'],
            'source_world_path':self.p['source_world_path'],
            'world_dimensions_m':list(self.p['world_dimensions']),
            'world_sha256':self.p['world_sha256'],
            'ros_distribution':os.getenv('ROS_DISTRO',''),
            'rmw_implementation':os.getenv('RMW_IMPLEMENTATION','default'),
            'ros_domain_id':os.getenv('ROS_DOMAIN_ID','0'),
            'hostname':socket.gethostname(),
            'logger_parameters':finite(self.p),
            'slam_resolution':self.p['slam_resolution'],
            'peer_export_resolution':self.p['slam_resolution'],
            'fusion_resolution':self.p['fusion_resolution'],
            'global_costmap_resolution':self.p['global_costmap_resolution'],
            'local_costmap_resolution':self.p['local_costmap_resolution'],
            'lidar_maximum_range':self.p['lidar_maximum_range'],
            'initial_map_costmap_configuration':json.loads(
                self.p['initial_configuration_json']),
            'robot_ids':self.robots,
            'initial_robot_poses':json.loads(
                self.p['robot_start_poses_json']),
            'known_initial_relative_transform':list(
                self.p['known_relative_transform']),
            'transform_source': self.p['transform_source'],
            'transform_frame_convention': {
                'source_frame': 'robot1_initial',
                'target_frame': 'robot2_initial',
                'meaning': 'robot2 pose expressed in robot1 initial frame',
                'transform_source': self.p['transform_source'],
                'world_sha256': self.p['world_sha256'],
            },
            'clean_shutdown':clean,'shutdown_status':status,
            'artifact_finalization':self._artifact_finalization,
        }
        atomic_json(self.directory/'run_manifest.json',value)
    def summary(self,clean):
        elapsed=time.monotonic()-self.start; a=self.attribution.summary(); motion=self.trajectory.summary(); records=list(self.warns.records.values()); rss=0
        try:rss=int(Path('/proc/self/statm').read_text().split()[1])*os.sysconf('SC_PAGE_SIZE')
        except OSError:pass
        cpu=sorted(self._cpu_samples); rss_values=self._rss_samples or [rss]
        percentile=lambda values,fraction: values[min(len(values)-1,max(0,math.ceil(len(values)*fraction)-1))] if values else 0.
        robot_states={r:{'claim_state':self.latest[r].get('claim_state','UNKNOWN'),'claim_id':self.latest[r].get('claim_id'),'frontier_id':self.latest[r].get('frontier_id'),'navigation_active':self.latest[r].get('navigation_active',False)} for r in self.robots}
        continuous={r:{'exploration_cycles':self.robot_counts[r]['EXPLORATION_CYCLE_STARTED'],'completed_goals':self.robot_counts[r]['SUCCESS_COOLDOWN_CREATED'],'failed_goals':self.robot_counts[r]['FAILURE_SUPPRESSION_CREATED'],'average_cycle_duration_s':statistics.fmean(self.cycle_durations[r]) if self.cycle_durations[r] else 0.,'suppression_creations':self.robot_counts[r]['FAILURE_SUPPRESSION_CREATED']+self.robot_counts[r]['SUCCESS_COOLDOWN_CREATED'],'repeated_region_attempts':sum(max(0,n-1) for n in self.region_attempts[r].values()),'maximum_equivalent_region_attempt_count':max(self.region_attempts[r].values(),default=0),'locally_exhausted_duration_s':self.exhausted_duration[r]+((time.monotonic()-self.exhausted_since[r]) if self.exhausted_since[r] is not None else 0.)} for r in self.robots}
        total_distance=sum(motion.get('distance_travelled_m',{}).values()); coverage_gain=(self.previous_known or 0)-(self.initial_known or 0)
        return {'schema_version':SCHEMA,'run':{'run_id':self.run_id,'start_time':self.start_utc,'end_time':utc_now(),'elapsed_duration_s':elapsed,'clean_shutdown':clean},'frames':{'global_frame':self.p['global_frame'],'trajectory_source_frame':'global_frame','coverage_source_frame':'robot_local_map','coverage_target_frame':'robot1_initial','initial_transform_source':self.p['transform_source'],'known_initial_relative_transform':list(self.p['known_relative_transform'])},'mapping':{'initial_known_cells':self.initial_known or 0,'final_known_cells':self.previous_known or 0,'coverage_gain_cells':coverage_gain,'coverage_gain_per_metre_travelled':coverage_gain/total_distance if total_distance>0 else 0.,**a},'motion':motion,'events':dict(self.counts),'coordination':{'agreement_publications':self.agreement_publications,'unique_agreed_rounds':len(self.unique_agreed_rounds),'unique_agreed_decisions':len(self.unique_agreed_decisions),'dispatch_attempts':self.dispatch_attempts,'goals_terminal':self.goals_terminal,'round_outcomes':dict(self.round_outcomes),'planner_query_attribution':dict(self.planner_query_counts),'planner_query_duration_s':dict(self.planner_query_duration_s)},'continuous_exploration':continuous,'mission':{'terminal':bool(self.mission_terminal_reason),'terminal_reason':self.mission_terminal_reason,'terminal_time_s':self.mission_completion_time,'shutdown_clean':clean},'mission_completion_time_s':self.mission_completion_time,'robot_terminal_state':robot_states,'navigation':{'goals_sent':self.counts['NAV_GOAL_SENT'],'goals_accepted':self.counts['NAV_GOAL_ACCEPTED'],'successes':self.counts['NAVIGATION_SUCCEEDED'],'failures':self.counts['NAVIGATION_FAILED'],'cancellations':self.counts['NAVIGATION_CANCELED'],'recoveries':self.counts['RECOVERY_COUNT_CHANGED'],'timeouts':self.counts['NAVIGATION_TIMEOUT']},'anomalies':{'no_progress_episodes':self.counts['NO_PROGRESS_STARTED'],'stuck_episodes':self.counts['STUCK_STARTED'],'stale_topic_episodes':self.counts['TOPIC_STALE'],'warning_occurrences':sum(r.occurrence_count for r in records)},'system':{'logger_pid':os.getpid(),'cpu_measurement':{'scope':'logger process only','normalization':'one CPU core equals 100 percent','sampling_interval_s':1.,'warmup_s':10.,'sample_count':len(cpu),'mean_percent':statistics.fmean(cpu) if cpu else 0.,'median_percent':statistics.median(cpu) if cpu else 0.,'p95_percent':percentile(cpu,.95),'peak_percent':max(cpu,default=0.)},'logger_cpu_percent':statistics.fmean(cpu) if cpu else 0.,'logger_rss_bytes':rss,'rss_mean_bytes':statistics.fmean(rss_values),'rss_peak_bytes':max(rss_values,default=rss),'output_file_sizes':{p.name:p.stat().st_size for p in self.directory.iterdir() if p.is_file()},'dropped_logger_samples':self.dropped_samples,'write_failures':self.write_failures,'internal_logger_error_count':sum(self.internal_errors.values()),'internal_logger_errors':dict(self.internal_errors)},'artifact_finalization':self._artifact_finalization}

    def write_mission_result(self, clean):
        """Write one compact process-facing terminal result beside summary.json."""
        reason = self.mission_terminal_reason
        if reason.startswith('MISSION_COMPLETE_') and self._artifact_finalization.get('complete',False):
            status = 'SUCCEEDED'
            exit_code = 0
        elif reason.startswith('MISSION_COMPLETE_'):
            status = 'FAILED'
            exit_code = 1
        elif reason.startswith('MISSION_ABORT_'):
            status = 'FAILED'
            exit_code = 1
        else:
            status = 'INCOMPLETE'
            exit_code = 2
        robot_states = {
            robot: {
                'state': self.latest[robot].get('claim_state', 'UNKNOWN'),
                'terminal': self.latest[robot].get('terminal', False),
                'terminal_reason': self.latest[robot].get('terminal_reason', ''),
                'navigation_active': self.latest[robot].get(
                    'navigation_active', False),
            }
            for robot in self.robots
        }
        terminal_reasons = {
            self.latest[robot].get('terminal_reason', '')
            for robot in self.robots
            if self.latest[robot].get('terminal', False)
        }
        evidence = {
            key: self.latest['robot1'].get(key, 0)
            for key in (
                'remaining_frontier_count', 'remaining_small_frontier_count',
                'remaining_out_of_range_count', 'remaining_unreachable_count',
                'planner_failure_count', 'detected_not_queried_count',
                'below_minimum_gain_count',
                'actionable_reachable_count',
            )
        }
        atomic_json(self.directory / 'mission_result.json', {
            'mission_status': status,
            'terminal_reason': reason or 'MISSION_NOT_TERMINATED',
            'simulated_duration_s': self.ros_seconds() - self.start_ros,
            'wall_duration_s': time.monotonic() - self.start,
            'accepted_goals': self.counts['NAV_GOAL_ACCEPTED'],
            'successful_goals': self.counts['NAVIGATION_SUCCEEDED'],
            'failed_goals': self.counts['NAVIGATION_FAILED'],
            'final_known_cells': self.previous_known or 0,
            'semantic_agreement': bool(self.unique_agreed_rounds),
            'terminal_agreement': len(terminal_reasons) == 1,
            'robot1_final_state': robot_states['robot1'],
            'robot2_final_state': robot_states['robot2'],
            'remaining_frontier_count': evidence['remaining_frontier_count'],
            'remaining_small_frontier_count': evidence[
                'remaining_small_frontier_count'],
            'remaining_out_of_range_count': evidence[
                'remaining_out_of_range_count'],
            'remaining_unreachable_count': evidence[
                'remaining_unreachable_count'],
            'planner_failed_count': evidence['planner_failure_count'],
            'detected_not_queried_count': evidence[
                'detected_not_queried_count'],
            'below_minimum_gain_count': evidence['below_minimum_gain_count'],
            'actionable_reachable_count': evidence[
                'actionable_reachable_count'],
            'terminal_small_frontier_length_m': float(
                self.p.get('terminal_small_frontier_length_m', 0.20)),
            'final_allocator_epoch': max(
                (self.latest[robot].get('terminal_epoch', 0)
                 for robot in self.robots), default=0,
            ),
            'final_map_revisions': {
                robot: self.latest[robot].get('terminal_map_revision', 0)
                for robot in self.robots
            },
            'terminal_time_s': self.mission_completion_time,
            'shutdown_clean': bool(clean),
            'recommended_exit_code': exit_code,
            'artifact_finalization': self._artifact_finalization,
        })
    def finalize(self,clean=True):
        with self._lifecycle_lock:
            if self.finalized or self._finalizing:return False
            self._finalizing=True
        for timer in self._observer_timers:
            try:timer.cancel()
            except Exception as exc:self.record_internal_error('timer_cancel',exc)
        if self.context.ok():
            self.event('RUN_END' if clean else 'RUN_INTERRUPTED','observer shutting down',console=True,allow_during_shutdown=True)
        if self.forensic is not None or self.contact_capture:
            if self.forensic is not None:
                self.forensic_snapshot(force=True)
            self.stop_forensic_ground_truth()
            if self.forensic is not None:
                # Evidence streams must be closed before their manifest is
                # committed and before launch shutdown can terminate us.
                self.forensic.close()
                atomic_json(self.directory / 'forensic' / 'manifest.json',
                            self.forensic.manifest())
        self.flush()
        successful=False
        try:
            # ROS launch signals all children concurrently.  Frontend
            # finalizers therefore need a bounded opportunity to close their
            # JSON/JSONL streams before this observer freezes the artifact
            # contract; otherwise a valid late file is recorded as missing.
            self.wait_for_frontend_diagnostics()
            self._artifact_finalization=self.required_artifact_status(False)
            clean=bool(clean and self._artifact_finalization['complete'])
            with self._state_lock:warning_records=[asdict(r) for r in self.warns.records.values()]
            with open(self.directory/'warnings.jsonl','w',encoding='utf-8') as f:
                for record in warning_records:f.write(json.dumps(finite(record),allow_nan=False)+'\n')
            atomic_json(self.directory/'summary.json',self.summary(clean)); self.write_mission_result(clean)
            self.write_manifest(clean,'clean' if clean else 'interrupted')
            self._artifact_finalization=self.required_artifact_status(True)
            clean=bool(clean and self._artifact_finalization['complete'])
            # Re-emit the three campaign contracts with the final status, so a
            # missing artifact cannot be mistaken for a successful run.
            atomic_json(self.directory/'summary.json',self.summary(clean))
            self.write_mission_result(clean)
            atomic_json(self.directory/'artifact_finalization.json',self._artifact_finalization)
            self.write_manifest(clean,'clean' if clean else 'finalization_failed')
            successful=self._artifact_finalization['complete']
        except Exception as exc:
            self.write_failures+=1
            if self.context.ok():
                self.get_logger().error(f'final output failed: {exc}')
            try:
                atomic_json(self.directory/'summary.json',self.summary(False)); self.write_manifest(False,'finalization_failed')
            except Exception:
                if self.context.ok():
                    self.get_logger().error('failed to record finalization failure')
        finally:
            with self._io_lock:
                self._closed=True
                for stream in [self.events,*self.files]:
                    try:stream.flush(); stream.close()
                    except Exception as exc:
                        if self.context.ok():
                            self.get_logger().error(f'file close failed: {exc}')
            if self.forensic is not None:
                self.forensic.close()
            with self._lifecycle_lock:self.finalized=True; self._finalizing=False
        return successful

def create_logger_executor(context=None):
    """Bind the logger executor to its dedicated ROS context."""
    return SingleThreadedExecutor(context=context)


def main(args=None):
    context = Context()
    rclpy.init(args=args, context=context,
               signal_handler_options=SignalHandlerOptions.NO)
    node = None
    executor = None
    clean = True
    shutdown_requested = {'value': False}
    previous_handlers = {}

    def request_shutdown(signum, frame):
        del signum, frame
        shutdown_requested['value'] = True
        if executor is not None and context.ok():
            executor.wake()

    try:
        node = CooperativeExperimentLogger(context=context)
        executor = create_logger_executor(context)
        executor.add_node(node)
        for signum in (signal.SIGINT, signal.SIGTERM):
            previous_handlers[signum] = signal.signal(signum, request_shutdown)
        while not shutdown_requested['value'] and context.ok():
            executor.spin_once(timeout_sec=0.5)
    except (KeyboardInterrupt, ExternalShutdownException):
        shutdown_requested['value'] = True
    except RuntimeError as exc:
        if not is_shutdown_conversion_error(
                exc, shutdown_requested['value'], context.ok()):
            clean = False
            raise
    except BaseException: clean=False; raise
    finally:
        for signum, handler in previous_handlers.items():
            signal.signal(signum, handler)
        if node is not None:
            node.finalize(clean)
            if executor is not None:
                executor.remove_node(node)
                executor.shutdown()
            if node.context.ok():
                node.destroy_node()
        if context.ok():
            context.shutdown()
