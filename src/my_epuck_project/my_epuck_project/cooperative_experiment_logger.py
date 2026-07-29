"""Strictly passive structured observer for two-robot exploration experiments."""
import csv, hashlib, json, math, os, socket, statistics, subprocess, threading, time, uuid
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
from rclpy.experimental.events_executor import EventsExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformListener
from my_epuck_interfaces.msg import (
    ExplorationClaim,
    ExplorationEvent,
    ExplorationStatus,
    FrontierCandidateArray,
    PeerMap,
)
from .experiment_metrics import CoverageAttribution, Grid, MotionDetector, MotionSample, TrajectoryOverlap, WarningDeduplicator, allocate_run_directory, atomic_json, duplicate_goal, equivalent_frontiers, finite, known_counts, known_world_cells, utc_now

SCHEMA='1.1.0'; STATES={0:'UNKNOWN',1:'PROPOSING',2:'NAVIGATING',3:'SUCCEEDED',4:'FAILED',5:'RELEASED',6:'CANCELED'}; STATUS_STATES={0:'STARTING',1:'ACTIVE',2:'NAVIGATING',3:'NO_ELIGIBLE_CANDIDATES',4:'COMPLETE',5:'STOPPED',6:'ERROR'}
TELEMETRY=['run_id','wall_time_utc','ros_time_sec','ros_time_nanosec','elapsed_s','event_sequence','robot_id','pose_x','pose_y','pose_yaw','linear_speed_mps','angular_speed_radps','commanded_linear_mps','commanded_angular_radps','distance_travelled_m','claim_state','claim_id','frontier_id','goal_x','goal_y','goal_yaw','navigation_active','distance_remaining_m','recoveries','candidate_count','local_known_cells','shared_known_cells','local_costmap_obstacles','global_costmap_known','global_costmap_obstacles','odom_age_s','scan_age_s','map_age_s','shared_map_age_s','claim_age_s','feedback_age_s']
COVERAGE=['run_id','wall_time_utc','ros_time_sec','ros_time_nanosec','elapsed_s','event_sequence','robot1_local_known','robot2_local_known','robot1_shared_known','robot2_shared_known','shared_free_cells','shared_occupied_cells','shared_unknown_cells','known_area_m2','coverage_gain_cells','coverage_gain_since_start_cells','unique_first_seen_robot1_cells','unique_first_seen_robot2_cells','later_duplicated_by_robot1_cells','later_duplicated_by_robot2_cells','simultaneously_observed_cells','total_known_union_cells','duplicated_known_fraction','shared_maps_equivalent']
HEALTH=['run_id','wall_time_utc','ros_time_sec','ros_time_nanosec','elapsed_s','event_sequence','robot_id','topic_name','topic_rate_hz','topic_age_s','expected_min_rate_hz','stale']

def yaw(q): return math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
def as_grid(m): return Grid(m.info.width,m.info.height,m.info.resolution,m.info.origin.position.x,m.info.origin.position.y,yaw(m.info.origin.orientation),np.asarray(m.data,dtype=np.int8))
def stamp(m):
    s=getattr(getattr(m,'header',None),'stamp',None); return (int(s.sec),int(s.nanosec)) if s else (0,0)
def default_run_id(): return time.strftime('%Y-%m-%dT%H%M%SZ',time.gmtime())+'_'+uuid.uuid4().hex[:4]

class CooperativeExperimentLogger(Node):
    def __init__(self):
        super().__init__('cooperative_experiment_logger')
        defaults={'run_id':'','output_root':'/home/arash/webots_ws/results','launch_file':'two_robots_observed_single_goal_launch.py','robot_ids':['robot1','robot2'],'global_frame':'shared_map','telemetry_rate_hz':1.,'coverage_rate_hz':.5,'topic_health_rate_hz':.2,'console_summary_period_s':5.,'warning_summary_period_s':30.,'progress_window_s':10.,'minimum_distance_remaining_improvement_m':.03,'minimum_robot_displacement_m':.02,'stuck_window_s':6.,'commanded_linear_threshold_mps':.02,'commanded_angular_threshold_radps':.15,'stuck_displacement_threshold_m':.015,'oscillation_window_s':10.,'angular_sign_change_threshold':4,'oscillation_displacement_threshold_m':.04,'simultaneous_coverage_window_s':2.,'trajectory_bin_size_m':.05,'initial_overlap_exclusion_radius_m':.15,'duplicate_goal_tolerance_m':.15,'shared_map_divergence_grace_s':3.,'enable_rosout_collection':True,'enable_coverage_attribution':True,'enable_trajectory_overlap':True,'enable_console_status':True,'odom_stale_s':2.,'scan_stale_s':2.,'map_stale_s':5.,'shared_map_stale_s':5.,'candidate_stale_s':5.,'claim_stale_s':4.,'status_stale_s':4.,'feedback_stale_s':3.,'costmap_stale_s':5.}
        for k,v in defaults.items(): self.declare_parameter(k,v)
        self.p={k:self.get_parameter(k).value for k in defaults}; self.robots=list(self.p['robot_ids']); self.start=time.monotonic(); self.start_utc=utc_now(); self.sequence=0; self.finalized=False; self._finalizing=False; self._closed=False; self.write_failures=0; self.dropped_samples=0
        self._state_lock=threading.RLock(); self._io_lock=threading.RLock(); self._lifecycle_lock=threading.Lock(); self.internal_errors=Counter(); self._reporting_internal_error=False; self._observer_timers=[]
        self._map_cache={}; self._transformed_cache={}; self._last_attributed={}; self._cpu_samples=[]; self._rss_samples=[]; self._cpu_previous=None
        self.run_id,self.directory=allocate_run_directory(Path(self.p['output_root']),self.p['run_id'] or default_run_id())
        self.robot_counts={r:Counter() for r in self.robots}; self.cycle_durations={r:[] for r in self.robots}; self.cycle_starts={}; self.region_attempts={r:Counter() for r in self.robots}; self.exhausted_since={r:None for r in self.robots}; self.exhausted_duration={r:0. for r in self.robots}; self.mission_completion_time=None; self.statuses={}
        self.events=open(self.directory/'events.jsonl','a',encoding='utf-8',buffering=1); self.warns=WarningDeduplicator(); self.counts=Counter(); self.last={}; self.windows={}; self.stale={}; self.latest={r:{} for r in self.robots}; self.claims={}
        self.detectors={r:MotionDetector(self.p['progress_window_s'],self.p['minimum_distance_remaining_improvement_m'],self.p['minimum_robot_displacement_m'],self.p['stuck_window_s'],self.p['commanded_linear_threshold_mps'],self.p['commanded_angular_threshold_radps'],self.p['stuck_displacement_threshold_m'],self.p['oscillation_window_s'],int(self.p['angular_sign_change_threshold']),self.p['oscillation_displacement_threshold_m']) for r in self.robots}
        self.attribution=CoverageAttribution(self.p['simultaneous_coverage_window_s']); self.trajectory=TrajectoryOverlap(self.p['trajectory_bin_size_m'],self.p['initial_overlap_exclusion_radius_m']); self.initial_known=None; self.previous_known=None; self.files=[]; self.writers={}
        self.stack_ready=False; self.divergence_since=None; self.divergence_reported=False; self.last_progress={}; self.tf_state={}
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

    def csv_file(self,name,fields):
        f=open(self.directory/name,'w',newline='',encoding='utf-8'); self.files.append(f); w=csv.DictWriter(f,fieldnames=fields); w.writeheader(); return w
    def csv_row(self,writer,row):
        with self._io_lock:
            if self._closed:self.dropped_samples+=1; return
            writer.writerow(finite(row))
    def qos(self,reliable=True,transient=False,depth=10): return QoSProfile(history=HistoryPolicy.KEEP_LAST,depth=depth,reliability=ReliabilityPolicy.RELIABLE if reliable else ReliabilityPolicy.BEST_EFFORT,durability=DurabilityPolicy.TRANSIENT_LOCAL if transient else DurabilityPolicy.VOLATILE)
    def observe(self,message_type,topic,callback,qos,subsystem):
        return self.create_subscription(message_type,topic,lambda message:self.safe_call(subsystem,callback,message),qos)
    def subscribe(self):
        for r in self.robots:
            self.observe(Odometry,f'/{r}/odom',lambda m,x=r:self.odom(x,m),qos_profile_sensor_data,f'{r}.odom')
            for s in ('scan_d500_fixed','scan_d500_slam'): self.observe(LaserScan,f'/{r}/{s}',lambda m,x=r,k=s:self.mark(x,k,m),qos_profile_sensor_data,f'{r}.{s}')
            for s in ('map','shared_map','local_costmap/costmap','global_costmap/costmap'): self.observe(OccupancyGrid,f'/{r}/{s}',lambda m,x=r,k=s:self.mark(x,k,m),self.qos(True,True,1),f'{r}.{s}')
            self.observe(PeerMap,f'/cslam/{r}/local_map',lambda m,x=r:self.mark(x,'peer_map',m),self.qos(True,True,1),f'{r}.peer_map')
            self.observe(FrontierCandidateArray,f'/{r}/frontier_candidates',lambda m,x=r:self.candidates(x,m),self.qos(True,False,1),f'{r}.candidates')
            self.observe(ExplorationClaim,f'/cslam/{r}/exploration_claim',lambda m,x=r:self.claim(x,m),self.qos(True,False,10),f'{r}.claim')
            self.observe(ExplorationStatus,f'/cslam/{r}/exploration_status',lambda m,x=r:self.status(x,m),self.qos(True,False,10),f'{r}.status')
            self.observe(ExplorationEvent,f'/cslam/{r}/exploration_event',lambda m,x=r:self.coordinator_event(x,m),self.qos(True,False,50),f'{r}.event')
            self.observe(NavigateToPose_FeedbackMessage,f'/{r}/navigate_to_pose/_action/feedback',lambda m,x=r:self.feedback(x,m),self.qos(),f'{r}.feedback')
            self.observe(GoalStatusArray,f'/{r}/navigate_to_pose/_action/status',lambda m,x=r:self.mark(x,'navigate_status',m),self.qos(True,True,1),f'{r}.navigate_status')
            self.observe(NavPath,f'/{r}/plan',lambda m,x=r:self.plan(x,m),self.qos(),f'{r}.plan')
            self.observe(Twist,f'/{r}/cmd_vel_nav',lambda m,x=r:self.command(x,m),self.qos(),f'{r}.cmd_vel_nav')
            self.observe(TwistStamped,f'/{r}/cmd_vel',lambda m,x=r:self.command(x,m.twist),self.qos(),f'{r}.cmd_vel')
        if self.p['enable_rosout_collection']: self.observe(Log,'/rosout',self.rosout,self.qos(True,True,1000),'rosout')
    def ros_now(self): n=self.get_clock().now().nanoseconds; return n//1000000000,n%1000000000
    def common(self,source='/cooperative_experiment_logger',robot=None,source_stamp=None):
        with self._state_lock:
            self.sequence+=1; sequence=self.sequence
        sec,nsec=source_stamp or self.ros_now(); return {'schema_version':SCHEMA,'run_id':self.run_id,'event_sequence':sequence,'wall_time_utc':utc_now(),'ros_time_sec':sec,'ros_time_nanosec':nsec,'elapsed_s':time.monotonic()-self.start,'robot_id':robot,'source':source}
    def event(self,event_type,message,robot=None,source='/cooperative_experiment_logger',severity='INFO',source_stamp=None,console=False,allow_during_shutdown=False,**extra):
        if (self._finalizing or self._closed) and not allow_during_shutdown:return None
        row=self.common(source,robot,source_stamp); row.update(severity=severity,event_type=event_type,message=message); row.update(finite(extra))
        with self._state_lock:self.counts[event_type]+=1
        try:
            encoded=json.dumps(finite(row),separators=(',',':'),allow_nan=False)+'\n'
            with self._io_lock:
                if self._closed:return None
                self.events.write(encoded)
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
        now=time.monotonic()
        with self._state_lock:
            self.last[(r,key)]=now; self.windows.setdefault((r,key),deque()).append(now); self.latest[r][key]=msg
    def age(self,r,key):
        value=self.last.get((r,key)); return time.monotonic()-value if value else None
    def odom(self,r,msg):
        self.mark(r,'odom',msg); p=msg.pose.pose.position; self.latest[r]['pose']=(p.x,p.y,yaw(msg.pose.pose.orientation)); self.latest[r]['speed']=(msg.twist.twist.linear.x,msg.twist.twist.angular.z)
        if self.p['enable_trajectory_overlap']: self.trajectory.add(r,p.x,p.y)
    def command(self,r,msg): self.mark(r,'cmd_vel',msg); self.latest[r]['command']=(msg.linear.x,msg.angular.z)
    def plan(self,r,msg): self.mark(r,'plan',msg); self.latest[r]['path_length']=sum(math.hypot(b.pose.position.x-a.pose.position.x,b.pose.position.y-a.pose.position.y) for a,b in zip(msg.poses,msg.poses[1:]))
    def candidates(self,r,msg):
        old=self.latest[r].get('candidate_count'); count=len(msg.candidates); self.mark(r,'frontier_candidates',msg); self.latest[r]['candidate_count']=count
        self.event('CANDIDATE_BATCH_RECEIVED',f'{count} reachable candidates',r,f'/{r}/frontier_candidates',source_stamp=stamp(msg),map_revision=msg.map_revision,candidate_count=count)
        if old is not None and old!=count:self.event('CANDIDATE_COUNT_CHANGED',f'{old} -> {count}',r)
        if not count:self.event('NO_REACHABLE_CANDIDATES','candidate batch empty',r)
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
        self.mark(r,'exploration_status',msg); old=self.statuses.get(r); self.statuses[r]=msg; state=STATUS_STATES.get(msg.state,str(msg.state)); now=time.monotonic()
        self.latest[r]['exploration_status']=state
        if old is None or old.state!=msg.state:
            self.event('EXPLORATION_STATUS_CHANGED',state,r,f'/cslam/{r}/exploration_status',source_stamp=stamp(msg),status_state=state,status_reason=msg.reason,status_revision=msg.message_revision,candidate_count=msg.candidate_count,eligible_candidate_count=msg.eligible_candidate_count)
        if msg.state==ExplorationStatus.NO_ELIGIBLE_CANDIDATES and self.exhausted_since[r] is None:self.exhausted_since[r]=now
        elif msg.state!=ExplorationStatus.NO_ELIGIBLE_CANDIDATES and self.exhausted_since[r] is not None:
            self.exhausted_duration[r]+=now-self.exhausted_since[r]; self.exhausted_since[r]=None
        if msg.state==ExplorationStatus.COMPLETE and self.mission_completion_time is None:self.mission_completion_time=now-self.start
    def coordinator_event(self,r,msg):
        self.mark(r,'exploration_event',msg); fields={'cycle_number':msg.cycle_number,'claim_id':msg.claim_id,'frontier_id':msg.frontier_id,'candidate_map_revision':msg.candidate_map_revision,'selected_rank':msg.selected_rank,'path_length_m':msg.path_length_m,'information_gain':msg.information_gain,'goal_x':msg.goal_pose.pose.position.x,'goal_y':msg.goal_pose.pose.position.y,'goal_yaw':yaw(msg.goal_pose.pose.orientation),'terminal_result':msg.terminal_result,'duration_s':msg.duration_s,'suppression_reason':msg.reason}
        if msg.event_type=='EXPLORATION_CYCLE_STARTED':
            self.cycle_starts[(r,msg.claim_id)]=(time.monotonic(),self.trajectory.total_distance.get(r,0.)); self.region_attempts[r][msg.frontier_id]+=1
        if msg.event_type=='EXPLORATION_CYCLE_ENDED':
            start=self.cycle_starts.pop((r,msg.claim_id),None)
            if start:
                fields['duration_s']=time.monotonic()-start[0]; fields['actual_travelled_distance_m']=self.trajectory.total_distance.get(r,0.)-start[1]; self.cycle_durations[r].append(fields['duration_s'])
        self.robot_counts[r][msg.event_type]+=1
        if msg.event_type=='MISSION_COMPLETE' and self.mission_completion_time is None:self.mission_completion_time=time.monotonic()-self.start
        self.event(msg.event_type,msg.reason or msg.terminal_result or msg.event_type,r,f'/cslam/{r}/exploration_event',source_stamp=stamp(msg),**fields)
    def feedback(self,r,msg):
        self.mark(r,'navigate_feedback',msg); f=msg.feedback; old=self.latest[r].get('recoveries',0); self.latest[r].update(distance_remaining=float(f.distance_remaining),recoveries=int(f.number_of_recoveries))
        if old!=f.number_of_recoveries:self.event('RECOVERY_COUNT_CHANGED',f'{old} -> {f.number_of_recoveries}',r,f'/{r}/navigate_to_pose/_action/feedback',source_stamp=stamp(f.current_pose),recoveries=f.number_of_recoveries,distance_remaining_m=f.distance_remaining)
    def rosout(self,msg):
        if msg.name.lstrip('/')=='cooperative_experiment_logger':return
        if msg.level<Log.WARN:return
        severity='ERROR' if msg.level>=Log.ERROR else 'WARN'; text=(msg.name+' '+msg.msg).lower(); category=next((v for k,v in [('costmap','COSTMAP_WARNING'),('controller','CONTROLLER_WARNING'),('slam','SLAM_WARNING'),('scan','SCAN_WARNING'),('transform','TF_WARNING'),(' tf','TF_WARNING')] if k in text),'PROCESS_WARNING')
        with self._state_lock:record,new=self.warns.add(msg.name,severity,msg.msg,utc_now(),category)
        if new:self.event(category,msg.msg,source='/rosout:'+msg.name,severity=severity,source_stamp=(msg.stamp.sec,msg.stamp.nanosec),occurrence_count=1)
    def row_time(self):
        sec,nsec=self.ros_now()
        with self._state_lock:self.sequence+=1; sequence=self.sequence
        return {'run_id':self.run_id,'wall_time_utc':utc_now(),'ros_time_sec':sec,'ros_time_nanosec':nsec,'elapsed_s':time.monotonic()-self.start,'event_sequence':sequence}
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
            d=self.latest[r]; pose=d.get('pose',(None,None,None)); speed=d.get('speed',(0.,0.)); command=d.get('command',(0.,0.)); goal=d.get('goal',(None,None,None)); row=self.row_time(); row.update(robot_id=r,pose_x=pose[0],pose_y=pose[1],pose_yaw=pose[2],linear_speed_mps=speed[0],angular_speed_radps=speed[1],commanded_linear_mps=command[0],commanded_angular_radps=command[1],distance_travelled_m=self.trajectory.total_distance.get(r,0.),claim_state=d.get('claim_state','UNKNOWN'),claim_id=d.get('claim_id'),frontier_id=d.get('frontier_id'),goal_x=goal[0],goal_y=goal[1],goal_yaw=goal[2],navigation_active=d.get('navigation_active',False),distance_remaining_m=d.get('distance_remaining'),recoveries=d.get('recoveries',0),candidate_count=d.get('candidate_count',0),local_known_cells=self.map_counts(r,'map')[0],shared_known_cells=self.map_counts(r,'shared_map')[0],local_costmap_obstacles=self.map_counts(r,'local_costmap/costmap')[2],global_costmap_known=self.map_counts(r,'global_costmap/costmap')[0],global_costmap_obstacles=self.map_counts(r,'global_costmap/costmap')[2],odom_age_s=self.age(r,'odom'),scan_age_s=self.age(r,'scan_d500_slam'),map_age_s=self.age(r,'map'),shared_map_age_s=self.age(r,'shared_map'),claim_age_s=self.age(r,'exploration_claim'),feedback_age_s=self.age(r,'navigate_feedback')); self.csv_row(self.writers[r],row)
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
        transforms={'robot1':(0.,0.,0.),'robot2':(-.299999998712,-.000027796077,-3.1415)}
        if self.p['enable_coverage_attribution']:
            for r,snapshot in zip(self.robots,local):
                if snapshot and self._last_attributed.get(r)!=snapshot[0]:
                    transform=transforms.get(r,(0.,0.,0.)); transformed=self._transformed_cache.get(r)
                    if transformed is None or transformed[0]!=snapshot[0]:
                        transformed=(snapshot[0],known_world_cells(snapshot[1],.01,transform)); self._transformed_cache[r]=transformed
                    self.attribution.observe(r,transformed[1],time.monotonic()-self.start); self._last_attributed[r]=snapshot[0]
        a=self.attribution.summary(); row=self.row_time(); row.update(robot1_local_known=self.map_counts('robot1','map')[0],robot2_local_known=self.map_counts('robot2','map')[0],robot1_shared_known=known[0],robot2_shared_known=known[1],shared_free_cells=counts[0][0],shared_occupied_cells=counts[0][1],shared_unknown_cells=counts[0][2],known_area_m2=known[0]*maps[0].info.resolution**2,coverage_gain_cells=gain,coverage_gain_since_start_cells=current-self.initial_known,unique_first_seen_robot1_cells=a['unique_first_seen_cells'].get('robot1',0),unique_first_seen_robot2_cells=a['unique_first_seen_cells'].get('robot2',0),later_duplicated_by_robot1_cells=a['later_duplicated_cells'].get('robot1',0),later_duplicated_by_robot2_cells=a['later_duplicated_cells'].get('robot2',0),simultaneously_observed_cells=a['simultaneously_observed_cells'],total_known_union_cells=a['total_known_union_cells'],duplicated_known_fraction=a['duplicated_known_fraction'],shared_maps_equivalent=equivalent); self.csv_row(self.coverage,row)
    def sample_health(self):
        limits={'odom':self.p['odom_stale_s'],'scan_d500_fixed':self.p['scan_stale_s'],'scan_d500_slam':self.p['scan_stale_s'],'map':self.p['map_stale_s'],'peer_map':self.p['map_stale_s'],'shared_map':self.p['shared_map_stale_s'],'frontier_candidates':self.p['candidate_stale_s'],'exploration_claim':self.p['claim_stale_s'],'exploration_status':self.p['status_stale_s'],'navigate_feedback':self.p['feedback_stale_s'],'local_costmap/costmap':self.p['costmap_stale_s'],'global_costmap/costmap':self.p['costmap_stale_s'],'cmd_vel':2.}
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
    def git_value(self,args,default):
        try:return subprocess.check_output(['git',*args],cwd='/home/arash/webots_ws',text=True,stderr=subprocess.DEVNULL).strip()
        except Exception:return default
    def write_manifest(self,clean,status):
        value={'schema_version':SCHEMA,'run_id':self.run_id,'utc_start_time':self.start_utc,'utc_end_time':utc_now() if status!='running' else None,'elapsed_duration_s':time.monotonic()-self.start,'git_commit':self.git_value(['rev-parse','HEAD'],'unknown'),'worktree_dirty':bool(self.git_value(['status','--porcelain'],'')),'launch_file':self.p['launch_file'],'launch_arguments':'recorded in logger parameters','world_resource':'two_robots.wbt','ros_distribution':os.getenv('ROS_DISTRO',''),'rmw_implementation':os.getenv('RMW_IMPLEMENTATION','default'),'ros_domain_id':os.getenv('ROS_DOMAIN_ID','0'),'hostname':socket.gethostname(),'logger_parameters':finite(self.p),'map_resolution':.01,'robot_ids':self.robots,'initial_robot_poses':{'robot1':[0.,0.,0.],'robot2':[-.3,0.,math.pi]},'known_initial_relative_transform':[-.299999998712,-.000027796077,-3.1415],'clean_shutdown':clean,'shutdown_status':status}; atomic_json(self.directory/'run_manifest.json',value)
    def summary(self,clean):
        elapsed=time.monotonic()-self.start; a=self.attribution.summary(); motion=self.trajectory.summary(); records=list(self.warns.records.values()); rss=0
        try:rss=int(Path('/proc/self/statm').read_text().split()[1])*os.sysconf('SC_PAGE_SIZE')
        except OSError:pass
        cpu=sorted(self._cpu_samples); rss_values=self._rss_samples or [rss]
        percentile=lambda values,fraction: values[min(len(values)-1,max(0,math.ceil(len(values)*fraction)-1))] if values else 0.
        robot_states={r:{'claim_state':self.latest[r].get('claim_state','UNKNOWN'),'claim_id':self.latest[r].get('claim_id'),'frontier_id':self.latest[r].get('frontier_id'),'navigation_active':self.latest[r].get('navigation_active',False)} for r in self.robots}
        continuous={r:{'exploration_cycles':self.robot_counts[r]['EXPLORATION_CYCLE_STARTED'],'completed_goals':self.robot_counts[r]['SUCCESS_COOLDOWN_CREATED'],'failed_goals':self.robot_counts[r]['FAILURE_SUPPRESSION_CREATED'],'average_cycle_duration_s':statistics.fmean(self.cycle_durations[r]) if self.cycle_durations[r] else 0.,'suppression_creations':self.robot_counts[r]['FAILURE_SUPPRESSION_CREATED']+self.robot_counts[r]['SUCCESS_COOLDOWN_CREATED'],'repeated_region_attempts':sum(max(0,n-1) for n in self.region_attempts[r].values()),'maximum_equivalent_region_attempt_count':max(self.region_attempts[r].values(),default=0),'locally_exhausted_duration_s':self.exhausted_duration[r]+((time.monotonic()-self.exhausted_since[r]) if self.exhausted_since[r] is not None else 0.)} for r in self.robots}
        total_distance=sum(motion.get('distance_travelled_m',{}).values()); coverage_gain=(self.previous_known or 0)-(self.initial_known or 0)
        return {'schema_version':SCHEMA,'run':{'run_id':self.run_id,'start_time':self.start_utc,'end_time':utc_now(),'elapsed_duration_s':elapsed,'clean_shutdown':clean},'mapping':{'initial_known_cells':self.initial_known or 0,'final_known_cells':self.previous_known or 0,'coverage_gain_cells':coverage_gain,'coverage_gain_per_metre_travelled':coverage_gain/total_distance if total_distance>0 else 0.,**a},'motion':motion,'events':dict(self.counts),'continuous_exploration':continuous,'mission_completion_time_s':self.mission_completion_time,'robot_terminal_state':robot_states,'navigation':{'goals_sent':self.counts['NAV_GOAL_SENT'],'goals_accepted':self.counts['NAV_GOAL_ACCEPTED'],'successes':self.counts['NAVIGATION_SUCCEEDED'],'failures':self.counts['NAVIGATION_FAILED'],'cancellations':self.counts['NAVIGATION_CANCELED'],'recoveries':self.counts['RECOVERY_COUNT_CHANGED'],'timeouts':self.counts['NAVIGATION_TIMEOUT']},'anomalies':{'no_progress_episodes':self.counts['NO_PROGRESS_STARTED'],'stuck_episodes':self.counts['STUCK_STARTED'],'oscillation_episodes':self.counts['OSCILLATION_STARTED'],'stale_topic_episodes':self.counts['TOPIC_STALE'],'warning_occurrences':sum(r.occurrence_count for r in records)},'system':{'logger_pid':os.getpid(),'cpu_measurement':{'scope':'logger process only','normalization':'one CPU core equals 100 percent','sampling_interval_s':1.,'warmup_s':10.,'sample_count':len(cpu),'mean_percent':statistics.fmean(cpu) if cpu else 0.,'median_percent':statistics.median(cpu) if cpu else 0.,'p95_percent':percentile(cpu,.95),'peak_percent':max(cpu,default=0.)},'logger_cpu_percent':statistics.fmean(cpu) if cpu else 0.,'logger_rss_bytes':rss,'rss_mean_bytes':statistics.fmean(rss_values),'rss_peak_bytes':max(rss_values,default=rss),'output_file_sizes':{p.name:p.stat().st_size for p in self.directory.iterdir() if p.is_file()},'dropped_logger_samples':self.dropped_samples,'write_failures':self.write_failures,'internal_logger_error_count':sum(self.internal_errors.values()),'internal_logger_errors':dict(self.internal_errors)}}
    def finalize(self,clean=True):
        with self._lifecycle_lock:
            if self.finalized or self._finalizing:return False
            self._finalizing=True
        for timer in self._observer_timers:
            try:timer.cancel()
            except Exception as exc:self.record_internal_error('timer_cancel',exc)
        self.event('RUN_END' if clean else 'RUN_INTERRUPTED','observer shutting down',console=True,allow_during_shutdown=True); self.flush()
        successful=False
        try:
            with self._state_lock:warning_records=[asdict(r) for r in self.warns.records.values()]
            with open(self.directory/'warnings.jsonl','w',encoding='utf-8') as f:
                for record in warning_records:f.write(json.dumps(finite(record),allow_nan=False)+'\n')
            atomic_json(self.directory/'summary.json',self.summary(clean)); self.write_manifest(clean,'clean' if clean else 'interrupted'); successful=True
        except Exception as exc:
            self.write_failures+=1; self.get_logger().error(f'final output failed: {exc}')
            try:
                atomic_json(self.directory/'summary.json',self.summary(False)); self.write_manifest(False,'finalization_failed')
            except Exception:self.get_logger().error('failed to record finalization failure')
        finally:
            with self._io_lock:
                self._closed=True
                for stream in [self.events,*self.files]:
                    try:stream.flush(); stream.close()
                    except Exception as exc:self.get_logger().error(f'file close failed: {exc}')
            with self._lifecycle_lock:self.finalized=True; self._finalizing=False
        return successful

def main(args=None):
    rclpy.init(args=args); node=None; executor=None; clean=True
    try:
        node=CooperativeExperimentLogger(); executor=EventsExecutor(); executor.add_node(node); executor.spin()
    except KeyboardInterrupt: pass
    except BaseException: clean=False; raise
    finally:
        if node is not None:
            node.finalize(clean)
            if executor is not None:executor.remove_node(node)
            node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
