import json, math, re
from collections import Counter
from pathlib import Path
import pytest
from my_epuck_project.experiment_metrics import *

def sample(t,x=0,y=0,remaining=1,linear=.1,angular=0): return MotionSample(t,x,y,remaining,linear,angular)
def test_utc_format(): assert re.fullmatch(r'\d{4}-\d\d-\d\dT\d\d:\d\d:\d\d\.\d{3}Z',utc_now())
def test_finite_json(): assert finite({'a':math.nan,'b':math.inf,'c':1.})=={'a':None,'b':None,'c':1.}
def test_atomic_json(tmp_path):
    p=tmp_path/'summary.json'; atomic_json(p,{'x':1}); assert json.loads(p.read_text())=={'x':1}; assert not list(tmp_path.glob('*.tmp'))
def test_directory_collision(tmp_path):
    a,p=allocate_run_directory(tmp_path,'run'); b,q=allocate_run_directory(tmp_path,'run'); assert a=='run' and b=='run-01' and p!=q
def test_directory_sanitizes(tmp_path): assert allocate_run_directory(tmp_path,'a:b/c')[0]=='a_b_c'
def test_warning_deduplication():
    d=WarningDeduplicator(); a,new=d.add('/n','WARN','failed at 1.25','t1','PROCESS_WARNING'); b,new2=d.add('/n','WARN','failed at 2.50','t2','PROCESS_WARNING'); assert new and not new2 and a is b and b.occurrence_count==2 and b.first_occurrence=='t1' and b.last_occurrence=='t2'
def test_warning_distinct_source():
    d=WarningDeduplicator(); d.add('/a','WARN','x 1','t','X'); d.add('/b','WARN','x 2','t','X'); assert len(d.records)==2
def test_warning_normalization_boundary(): assert normalize_warning('wheel2 failed')!=normalize_warning('wheel3 failed')
def test_no_false_detection_inactive():
    d=MotionDetector(progress_window=2,stuck_window=2); events=[]
    for t in range(4):events+=d.update(sample(t),False)
    assert not events
def test_normal_progress():
    d=MotionDetector(progress_window=2); events=[]
    for t in range(4):events+=d.update(sample(t,x=t*.1,remaining=1-t*.1),True)
    assert 'NO_PROGRESS_STARTED' not in events
def test_sustained_no_progress_and_clear():
    d=MotionDetector(progress_window=2,stuck_window=99); events=[]
    for t in range(4):events+=d.update(sample(t),True)
    assert events.count('NO_PROGRESS_STARTED')==1
    events+=d.update(sample(5,x=.2,remaining=.5),True); assert 'NO_PROGRESS_CLEARED' in events
def test_stuck_requires_command():
    d=MotionDetector(progress_window=99,stuck_window=2); events=[]
    for t in range(4):events+=d.update(sample(t,linear=0,angular=0),True)
    assert 'STUCK_STARTED' not in events
def test_stuck_and_recovery():
    d=MotionDetector(progress_window=99,stuck_window=2); events=[]
    for t in range(4):events+=d.update(sample(t),True)
    assert events.count('STUCK_STARTED')==1
    events+=d.update(sample(5,x=.2),True); assert 'STUCK_CLEARED' in events
def test_near_goal_not_stuck():
    d=MotionDetector(progress_window=1,stuck_window=1); assert not sum((d.update(sample(t,remaining=.01),True,near_goal=True) for t in range(3)),[])
def test_oscillation_and_clear():
    d=MotionDetector(progress_window=99,stuck_window=99,oscillation_window=4,sign_changes=4); events=[]
    for t in range(6):events+=d.update(sample(t,angular=.2*(-1)**t),True)
    assert 'OSCILLATION_STARTED' in events
    events+=d.update(sample(7,x=.2,remaining=.5,angular=.2),True); assert 'OSCILLATION_CLEARED' in events
def test_displacement_prevents_oscillation():
    d=MotionDetector(progress_window=99,stuck_window=99,oscillation_window=4,sign_changes=4); events=[]
    for t in range(6):events+=d.update(sample(t,x=t*.1,remaining=1-t*.1,angular=.2*(-1)**t),True)
    assert 'OSCILLATION_STARTED' not in events
def test_known_counts_unknown_ignored(): assert known_counts(Grid(2,2,1,0,0,0,(-1,0,49,100)))==(2,1,1)
def test_transformed_cells_negative_rotated():
    cells=known_world_cells(Grid(1,1,1,-2,-1,math.pi/2,(0,)),.5,(1,0,math.pi)); assert cells=={(7,0)}
def test_dynamic_bounds(): assert len(known_world_cells(Grid(3,1,.1,0,0,0,(0,-1,0)),.1))==2
def test_coverage_attribution():
    c=CoverageAttribution(2); c.observe('r1',{(0,0),(1,0)},0); c.observe('r2',{(0,0),(2,0)},1); c.observe('r2',{(1,0)},5); s=c.summary(); assert s['simultaneously_observed_cells']==1 and s['later_duplicated_cells']['r2']==1 and s['total_known_union_cells']==3
def test_trajectory_overlap_and_revisit():
    t=TrajectoryOverlap(1,0); t.add('r1',0,0); t.add('r1',1.1,0); t.add('r1',1.2,0); t.add('r2',0,0); t.add('r2',1.1,0); s=t.summary(); assert s['cross_robot_bins']==1 and s['repeated_visit_distance_m']['r1']>0
def frontier(x): return {'centroid_x':x,'centroid_y':0,'min_x':x-.1,'max_x':x+.1,'min_y':-.1,'max_y':.1}
def test_frontier_equivalence(): assert equivalent_frontiers(frontier(0),frontier(.1)) and not equivalent_frontiers(frontier(0),frontier(1))
def test_goal_duplicates(): assert duplicate_goal((0,0),(.1,0)) and not duplicate_goal((0,0),(1,0))
def test_event_sequences_strict(): assert list(range(1,5))==sorted(set(range(1,5)))
def test_summary_reconciliation_fixture():
    events=['NAV_GOAL_SENT','NAV_GOAL_ACCEPTED','NAVIGATION_SUCCEEDED']; c=Counter(events); assert c['NAV_GOAL_SENT']==c['NAVIGATION_SUCCEEDED']+c['NAVIGATION_FAILED']
