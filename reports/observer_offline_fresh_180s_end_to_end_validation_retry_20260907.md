# Fresh 180 s Condition-C retry — preflight failure

## Final verdict

`FRESH_180S_RETRY_FAIL`

No retry Webots run was launched.  The retry was blocked at the mandatory
clean-source preflight because the worktree contains genuinely modified tracked
source, launch, interface, package, and test files.  Those changes were not
discarded, committed, or absorbed.  No source, observer, evaluator, or launch
file was changed by this task.

## Requested retry configuration

The intended single retry would have been:

- Condition C, `frontier_cost_only`, MODE_B;
- `large_unknown_pose_close_start_20ms_scan_matching`;
- seed 1001, full sensors, ideal encoders;
- Webots fast/headless, `use_sim_time=true`;
- scientific raw evidence enabled;
- 20 ms Supervisor GT and 20 ms contact polling;
- profiling disabled;
- 180 simulated seconds, with an independent wall watchdog.

The previous failed run remains at:

`results/observer_offline_fresh_180s_end_to_end_validation_20260907/fast_trial_20260907T121203Z/`

It was not overwritten or reused as a retry.

## Environment correction verification

The original library-load cause was corrected at the shell/preflight level,
without replacing the ROS-provided loader path.

From a clean shell after:

```bash
source /opt/ros/jazzy/setup.bash
```

the resulting `LD_LIBRARY_PATH` contains the complete Jazzy vendor path set,
including:

```text
/opt/ros/jazzy/opt/sdformat_vendor/lib
```

Read-only loader checks passed:

```text
ldd /opt/ros/jazzy/lib/libsdformat_urdf_plugin.so
  not_found=0

ldd /opt/ros/jazzy/lib/robot_state_publisher/robot_state_publisher
  not_found=0
```

The WSL route is `172.18.32.1` and `WSL_INTEROP` is present as
`/run/WSL/322_interop`.  These checks establish that the previous missing
`libsdformat14.so.14` issue would not recur from this clean ROS environment.

## Clean-source preflight failure

HEAD at inspection time:

```text
b6382cc5b1c2371f7300402527fa8a2688b57b3e
```

There are 31 modified tracked files.  The following are genuine tracked files,
not merely generated result artifacts:

```text
AGENTS.md
src/my_epuck_frontier_candidates/src/frontier_candidate_generator.cpp
src/my_epuck_interfaces/msg/DistributedExplorationStatus.msg
src/my_epuck_interfaces/msg/RelativePoseHypothesis.msg
src/my_epuck_project/launch/two_robots_decentralized_exploration_launch.py
src/my_epuck_project/launch/two_robots_distributed_assignment_launch.py
src/my_epuck_project/launch/two_robots_namespaced_launch.py
src/my_epuck_project/launch/two_robots_teammate_filtered_dual_slam_launch.py
src/my_epuck_project/launch/two_robots_teammate_filtered_stack_launch.py
src/my_epuck_project/my_epuck_project/__pycache__/__init__.cpython-312.pyc
src/my_epuck_project/my_epuck_project/__pycache__/d500_scan_fix.cpython-312.pyc
src/my_epuck_project/my_epuck_project/__pycache__/imu_webots_plugin.cpython-312.pyc
src/my_epuck_project/my_epuck_project/__pycache__/twist_stamper.cpython-312.pyc
src/my_epuck_project/my_epuck_project/cooperative_map_png_export.py
src/my_epuck_project/my_epuck_project/cooperative_profiles.py
src/my_epuck_project/my_epuck_project/cooperative_regression.py
src/my_epuck_project/my_epuck_project/cooperative_trial_fast.py
src/my_epuck_project/my_epuck_project/distributed_assignment/local_nav2.py
src/my_epuck_project/my_epuck_project/distributed_assignment/scoring.py
src/my_epuck_project/my_epuck_project/distributed_frontier_assignment.py
src/my_epuck_project/my_epuck_project/paced_ros2_supervisor.py
src/my_epuck_project/my_epuck_project/ros_runtime_preflight.py
src/my_epuck_project/my_epuck_project/unknown_pose_frontend.py
src/my_epuck_project/my_epuck_project/unknown_pose_frontend_core.py
src/my_epuck_project/my_epuck_project/unknown_pose_shared_stack_activation.py
src/my_epuck_project/package.xml
src/my_epuck_project/test/test_cooperative_regression.py
src/my_epuck_project/test/test_cooperative_trial_fast.py
src/my_epuck_project/test/test_distributed_frontier_round_guard.py
src/my_epuck_project/test/test_distributed_pair_scoring.py
src/my_epuck_project/test/test_ros_runtime_preflight.py
```

The repository also contains many untracked build/result directories.  They were
left untouched.  No attempt was made to decide whether any of those artifacts
are recoverable user work.

Because production source and launch files are modified and uncommitted, it is
not safe to create a clean acceptance provenance record by committing them as a
group.  It is also not safe to reset or discard them.  The preflight therefore
stops here, as required.

## Preflight checklist

| Check | Result | Evidence |
|---|---|---|
| correct HEAD recorded | PASS | `b6382cc5...` |
| clean committed worktree | **FAIL** | 31 modified tracked files |
| ROS Jazzy environment | PASS | `/opt/ros/jazzy/setup.bash` sourced |
| Jazzy vendor libraries preserved | PASS | `sdformat_vendor` path present |
| `libsdformat_urdf_plugin.so` dependencies | PASS | `ldd`, zero `not found` |
| `robot_state_publisher` dependencies | PASS | `ldd`, zero `not found` |
| approved project install | not authorized | clean-source gate failed first |
| approved Webots driver / NAT endpoint | not authorized | clean-source gate failed first |
| stale-workspace exclusion | not authorized | clean-source gate failed first |
| current interfaces | not authorized | clean-source gate failed first |
| PowerShell stdin/preflight launch checks | not authorized | no launch permitted |
| Webots retry | **NOT RUN** | mandatory stop condition |

## Required answers

**Exact environment cause from the previous run:** the wrapper replaced
`LD_LIBRARY_PATH` and dropped Jazzy vendor directories.  The corrected procedure
preserves the sourced value and prepends any project/driver paths only.

**Why no retry occurred:** the independent clean-worktree acceptance gate failed
on genuine tracked source changes.  No safe authority exists to commit or revert
those changes.

**Source change required for the loader issue:** no.  The loader issue is fixed
by environment construction and preflight validation.

**Source change made in this task:** none.

**Webots runs launched in this task:** zero.

**Final retry status:** `FRESH_180S_RETRY_FAIL`

## Follow-up retry attempt — 2026-09-07

The inherited tracked implementation was preserved without content edits in
checkpoint commit `92942dbb68491747cf95927e36908fec6b4e4376`:

```text
checkpoint: preserve current working implementation before NAT retry
```

That checkpoint contains the previously dirty tracked implementation and the
untracked source modules required by the current installed package. No build,
observer, evaluator, or robotics source was changed for the retry itself.
Historical build/install/log/result directories remain on disk and were not
deleted. They are locally excluded from Git status as generated/noncanonical
artifacts; the retry manifest therefore saw a clean committed source tree.

The corrected ROS loader setup passed all checks:

- `/opt/ros/jazzy/opt/sdformat_vendor/lib` remained in `LD_LIBRARY_PATH`;
- `ldd libsdformat_urdf_plugin.so` reported zero `not found` dependencies;
- `ldd robot_state_publisher` reported zero `not found` dependencies;
- NAT mode and endpoint were `nat` and `172.18.32.1:23420`;
- ROS domain was `230`;
- project and interface packages resolved from
  `install_observer_offline_fresh_180s_20260907`;
- no other Webots/ROS process or selected port listener was present.

The single authorized retry then stopped during the runner's own preflight,
before Webots was spawned, because the launch environment omitted the runner's
required explicit variable:

```text
MY_EPUCK_WEBOTS_DRIVER_PREFIX must identify the intended isolated Webots driver install
```

The variable should have been set to:

```text
/home/arash/webots_ws_close_validation_2eb/install/webots_ros2_driver
```

Evidence is in:

- `results/observer_offline_fresh_180s_retry_20260907/runner_console.log`;
- `results/observer_offline_fresh_180s_retry_20260907/preflight_environment.txt`;
- `results/observer_offline_fresh_180s_retry_20260907/ldd_libsdformat_urdf_plugin.txt`;
- `results/observer_offline_fresh_180s_retry_20260907/ldd_robot_state_publisher.txt`.

No Webots process, ROS launch, controller, `/clock`, or scientific artifact was
started by this follow-up attempt. Because the task authorized exactly one new
retry, no second retry was launched. The final status remains:

```text
FRESH_180S_RETRY_FAIL
```

## Corrected NAT retry — 2026-09-07

This is the one authorized Webots retry after the wrapper correction. It used
committed source `92942dbb68491747cf95927e36908fec6b4e4376`, the current isolated
project install, and the legacy/salvaged observer path with scientific raw
capture enabled. No observer, evaluator, robotics, Nav2, SLAM, allocator, or
parameter source was changed for this run.

### Preflight and provenance

Result directory:

```text
results/observer_offline_fresh_180s_retry3_20260907/fast_trial_20260907T130521Z/
```

The complete preflight record is in
`results/observer_offline_fresh_180s_retry3_20260907/preflight_environment.txt`.

| Check | Result |
|---|---|
| HEAD | `92942dbb68491747cf95927e36908fec6b4e4376` |
| Worktree at launch | clean (`worktree_dirty=false`) |
| ROS | Jazzy, `rmw_cyclonedds_cpp` |
| Project prefix | `install_observer_offline_fresh_180s_20260907` |
| Webots driver | `/home/arash/webots_ws_close_validation_2eb/install/webots_ros2_driver` |
| WSL networking | NAT, `172.18.32.1:23420` |
| ROS domain | 230 |
| ROS vendor library paths | preserved, including `.../opt/sdformat_vendor/lib` |
| `ldd libsdformat_urdf_plugin.so` | zero `not found` dependencies |
| `ldd robot_state_publisher` | zero `not found` dependencies |
| Webots controllers | robot1, robot2, Supervisor, and ROS2 Supervisor connected |

### Run boundary and validity

| Field | Observed value |
|---|---:|
| simulation start | 0.02 s |
| simulation end | 180.08 s (`/clock`; Supervisor artifact ends at 180.80 s during teardown) |
| configured horizon | 180.00 s |
| termination | `SIM_TIME_COMPLETE` |
| horizon overrun | false |
| wall watchdog | armed, not fired |
| runner wall duration | 137.7517 s, including readiness and cleanup |
| clean shutdown | true |
| launch return code | 0 |
| `artifact_finalization.json` | `COMPLETE`, `missing=[]` |
| `passive_rosbag_export.json` | `complete=true`, recorder return code 0 |
| offline evaluator | `thin_offline_evaluation_2.0`, `complete=true` |

`mission_result.json` reports `MISSION_NOT_TERMINATED` because both robots still
had an active goal at the deliberately imposed 180 s boundary. That is
mission-completion status, not an evidence or shutdown failure; the scientific
horizon and finalization gates passed.

The legacy path does not emit the thin-only `runtime_boundary` sidecar. The
strongest post-run active-clock reconstruction is therefore from native rosbag
`/clock` receipt timestamps: first `(wall=1788786335.0255868, sim=0.02)`, final
`(wall=1788786417.7120812, sim=180.74)`, giving **180.72 / 82.68649435 =
2.18560481x**. The Supervisor observer lifetime ratio is **180.8 / 104.52016435
= 1.72980976x** and is secondary because it includes startup and shutdown.
Offline replay took 4.6031 s and is not part of active mission RTF.

Offline replay was run from a clean ROS-enabled shell after sourcing
`/opt/ros/jazzy/setup.bash` and
`install_observer_offline_fresh_180s_20260907/setup.bash`, with the approved
driver prefix prepended while preserving Jazzy library paths. The invocation was
the current installed Python evaluator equivalent of:

```text
python3 -c 'from pathlib import Path; from my_epuck_project.offline_evidence_replay import evaluate_run; evaluate_run(Path(<fresh observer directory>), ("robot1", "robot2"))'
```

It opened the native `passive_rosbag_0.db3` using the installed Jazzy
`rosbag2_py`/interface definitions and generated the offline sidecars in the
fresh observer directory. Because this was deliberately the legacy observer
path, `raw_evidence_finalization.json` and the thin-session linkage field are not
part of this legacy artifact; the authoritative legacy gate is
`artifact_finalization.json`, which is `COMPLETE`, while the evaluator’s own
`thin_metrics.json:complete` is also `true`.

### Raw evidence observed

The native bag contains 87,966 messages, 53 selected topics, and all required
core streams. Important counts are:

| Stream/family | Counts |
|---|---:|
| `/clock` | 1,635 |
| `/robot1/odom`, `/robot2/odom` | 8,790; 8,779 |
| `/tf`, `/tf_static` | 28,758; 11 |
| local `/map` robot1/robot2 | 81; 81 |
| shared `/shared_map` robot1/robot2 | 38; 41 |
| frontier candidates robot1/robot2 | 27; 21 |
| task snapshots robot1/robot2 | 27; 21 |
| task bids robot1/robot2 | 19; 49 |
| distributed status robot1/robot2 | 164; 164 |
| distributed event robot1/robot2 | 47; 49 |
| NavigateToPose status robot1/robot2 | 12; 10 |
| ComputePathToPose status robot1/robot2 | 420; 374 |
| `/robot1/plan`, `/robot2/plan` | 206; 186 |
| `/cslam/unknown_pose/start_release` | 1 |
| GT rows | 18,080 (9,040 per robot) |
| contact queries | 18,080 (20 ms cadence) |
| contact rows | 86,175 |
| rosout | 2,454 |

The two optional `/cslam/unknown_pose/*/exploration_event` and
`exploration_status` aliases had zero messages, while the authoritative
`distributed_event` and `distributed_status` streams were populated and the
semantic contract passed. This is a valid alias-zero state, not missing
cooperation evidence.

### Fresh offline metric audit

All values below are from the fresh run’s generated `thin_metrics.json` and its
sidecars, with the legacy `summary.json`/`mission_result.json` called out where
they contain an additional legacy field. `VALID_ZERO` means the stream was read
and the observed count was genuinely zero. `NOT_INVOKED` is distinct from missing
evidence. `NOT_AVAILABLE_FOR_180S_HORIZON` means the analysis correctly did not
zero-fill a window or milestone beyond the artifact boundary.

#### Exploration and coverage

| Field | Status | Artifact/key | Observed value |
|---|---|---|---:|
| known-area curve | PASS | `thin_metrics.json:coverage.series` | 62 shared-map samples |
| final known area | PASS | `coverage.final_known_area_m2` | 49.56209778 m² |
| final known percentage | PASS | `coverage.final_known_area_m2 / 400 m²` | 12.3905% |
| coverage AUC | PASS | `coverage.known_area_auc_m2_s` | 4,800.70389739 m²·s |
| final known cells | PASS | `coverage.final_known_cells` | 55,069 |
| known-cell AUC | PASS | `coverage.known_cell_auc_cells_s` | 5,334,115.68 cell·s |
| 25/50/75/90% milestones | NOT_AVAILABLE_FOR_180S_HORIZON | `coverage.time_to_world_extent_thresholds_s` | all null; thresholds not reached |
| first-seen ownership | PASS | `coverage.ownership.unique_first_seen_cells` | robot1 22,191; robot2 32,774 |
| later duplicate cells | PASS | `coverage.ownership.later_duplicated_cells` | robot1 26,351; robot2 19,243 |
| duplicated-known fraction | PASS | `coverage.ownership.duplicated_known_fraction` | 0.82950969 |
| simultaneous observation | PASS | `summary.json:mapping.simultaneously_observed_cells` | 7,997 cells |
| coverage per metre | PASS | `summary.json:mapping.coverage_gain_per_metre_travelled` | 3,513.8275 cells/m; 3.3485101 m²/m |
| map ownership transform | PASS | `coverage.ownership.transform_source` | known initial transform |

#### Motion and efficiency

| Field | Status | Artifact/key | Observed value |
|---|---|---|---:|
| robot1 odom distance | PASS | `motion.robot1.travel_distance_m` | 6.54378915 m |
| robot2 odom distance | PASS | `motion.robot2.travel_distance_m` | 8.25744957 m |
| total odom distance | PASS | `efficiency.total_odom_distance_m` | 14.80123873 m |
| GT distance | PASS | `ground_truth.robots.*.travel_distance_m` | 6.52298815; 8.24342649 m |
| repeated/revisited distance | PASS | `summary.json:motion.repeated_visit_distance_m` | 4.58061647; 5.80481768 m |
| shared-frame trajectory | PASS | `shared_frame_motion` | 156/188 bins; 6.60258984/8.29137335 m |
| trajectory overlap | PASS | `ground_truth.trajectory_overlap` | 34 intersection / 339 union; IoU 0.10029499 |
| shared cross-robot overlap | PASS | `shared_frame_motion.cross_robot_overlap_fraction` | 0.12052117 |
| feasible-work time | PASS | `avoidable_idle.robots.*.feasible_work_seconds` | robot1 110.20 s; robot2 98.66 s |
| avoidable idle | PASS | `avoidable_idle.robots.*.avoidable_idle_seconds` | robot1 18.22 s; robot2 0 s |
| avoidable-idle fraction | PASS | `avoidable_idle.robots.*.avoidable_idle_fraction` | 16.5336%; 0% |
| productive engagement | PASS | `avoidable_idle.robots.*.productive_engagement_seconds` | 91.98 s; 98.66 s |
| productive engagement fraction | PASS | `avoidable_idle.robots.*.productive_engagement_fraction` | 83.4664%; 100% |
| longest avoidable idle | PASS | `avoidable_idle.robots.*.longest_avoidable_idle_s` | 18.22 s; 0 s |
| idle interval/reason records | PASS | `avoidable_idle.robots.*.segments` | 22 segments per robot; reasons retained |

The frozen avoidable-idle targets are not met by robot1 in this smoke
(16.5336% > 5%, 18.22 s > 5 s, productive engagement 83.4664% < 95%). Those
are measured scientific target results, not extraction failures.

#### Cooperation, assignment, certificate, DNU, and navigation

| Field | Status | Artifact/key | Observed value |
|---|---|---|---:|
| candidate batches | PASS | `cooperation_summary.candidate_batches` | 48 total; robot1 27, robot2 21 |
| task snapshots | PASS | `cooperation_summary.task_snapshots` | 48 |
| bids | PASS | `cooperation_summary.bids` | 136 records, 136 valid, 68 batches |
| pair decisions | VALID_ZERO | `cooperation_summary.pair_decisions` | 0 records; stream was read |
| assignments/dispatches | PASS | `cooperation_summary.assignments` | 11 dispatches; robot1 6, robot2 5 |
| continuations | VALID_ZERO | `cooperation_summary.continuations` | 0 records |
| agreement publications | VALID_ZERO | `cooperation_summary.agreements` | 0 publications/decisions/rounds |
| claims | VALID_ZERO | `cooperation_summary.claims` | 0 records |
| certificate state | PASS | `protocol_contract.certificate_status` | OBSERVED, 1 observation, complete |
| certificate required payload | PASS | certificate status | all 7 required fields present |
| DNU observations | PASS | `cooperation_summary.dnu` | 376 series rows; robot1 191, robot2 185 |
| DNU / detected-not-queried field | PASS | certificate/status payloads | preserved in raw and replay |
| certificate reason/bounds/scores | PASS | `cooperation.certificate_records[0]` | reason, counts, selected score, optimistic bound, certification present |
| navigation terminals | PASS | `cooperation_summary.terminals` | 9 protocol terminals, all succeeded |
| action replay | PASS | `navigation_action_replay.json` | 13,927 deserialized messages; required status present |
| protocol failures | PASS | `cooperation_summary.failures` | 3 records: 1 hard unreachable, 2 TF/lifecycle |
| Nav2 goal outcomes | PASS | `navigation` | 11 sent/accepted, 9 success, 0 failure, 0 cancel, 0 timeout |
| active at horizon | PASS | `mission_result.goal_accounting` | 2 goals active at boundary, explicitly accounted |

#### Timestamp joins and snappiness

| Field | Status | Artifact/key | Count / p50 / p95 |
|---|---|---|---:|
| terminal → next dispatch | PASS | `snappiness.terminal_to_next_dispatch` | 9 / 0.22 s / 2.78 s |
| candidate-generation timing | PASS | `snappiness.candidate_generation` | 48 / 0 s / 0 s |
| bid latency | PASS | `snappiness.bid` | 136 / 13.08 s / 24.18 s |
| certificate latency | PASS | `snappiness.certificate` | 1 / 0.10 s / 0.10 s |
| agreement → goal | NOT_INVOKED | `snappiness.agreement_to_nav_goal_sent` | no agreement/goal join |
| handoff → first shared goal | PASS | `snappiness.handoff_to_first_shared_goal` | 1 / 3.44 s / 3.44 s |
| ComputePathToPose evidence | PASS | native bag + `navigation_action_replay.json` | 794 status messages; replay complete |
| per-query planner latency | NOT_AVAILABLE_FOR_180S_HORIZON | `summary.json:planner_query_duration_s` | aggregate 0.353 s exists; per-query latency not emitted |

The p95 terminal→dispatch target (3 s) passes at 2.78 s. The handoff→shared-goal
target (3 s) is measured at 3.44 s and therefore fails that performance target;
the join itself is valid. Agreement latency is correctly `NOT_INVOKED`, not zero.

#### Reliability and anomalies

| Field | Status | Artifact/key | Observed value |
|---|---|---|---:|
| no-progress episodes | VALID_ZERO | `summary.json:anomalies` | 0 |
| stuck episodes | PASS | `motion_anomalies.json` | 2 robot1 episodes: 83.06–95.06 and 118.06–121.06 s |
| oscillation | NOT_INVOKED | current replay output | no emitted oscillation event |
| stale-topic episodes | PASS | `summary.json:anomalies` | 27 |
| warnings | PASS | `warnings.jsonl` / evaluator | 156: controller 39, costmap 8, process 103, scan 4, TF 2 |
| Nav2 outcomes | PASS | `navigation_action_replay.json` | complete; statuses and transitions parsed |
| controller failures | VALID_ZERO | navigation/cooperation artifacts | 0 controller failures |
| planner failures | VALID_ZERO | `mission_result.json` | 0 planner failures |

#### Physical GT, contact, handoff, and map-frame quality

| Field | Status | Artifact/key | Observed value |
|---|---|---|---:|
| GT trajectory | PASS | `forensic/supervisor_ground_truth.csv` | 9,040 rows/robot; requested 20 ms |
| contact polling | PASS | `forensic/runtime/runtime_metrics.json` | 18,080 queries; 20 ms |
| contact evidence | PASS | `contact_points.csv` / evaluator | 9,040 samples/robot; 1 episode/robot |
| accepted handoff records | PASS | frontend JSON files | 2 accepted records, same evidence hash |
| handoff physical translation error | PASS | `handoff.structured_files` | 0.00521816 m |
| handoff physical yaw error | PASS | same | 0.00639573 rad / 0.36644861° |
| estimator quality | PASS | same | confidence 0.6723; inlier ratio 0.98245; p95 residual 0.01807 m |
| handoff timing | PASS | same | accepted at 12.84 s and 15.50 s; one common START_RELEASE at 38.24 s |
| reciprocal verification | NOT_INVOKED | `handoff_reciprocal_verification.json` | no accepted reciprocal pair; 2 records, no missing pair |
| approved map-frame quality | PASS | `map_quality_report.json` | physical-GT map-frame residual available |
| map-frame translation/yaw quality | PASS | same | 0.00521816 m; 0.36644861° |
| external truth-raster IoU | NOT_INVOKED | no approved external raster | correctly not fabricated |

#### Fairness and scaling

| Field | Status | Artifact/key | Observed value |
|---|---|---|---:|
| first-seen contribution | PASS | `fairness.robots` | robot1 22,191; robot2 32,774 cells |
| duplicate contribution | PASS | same | robot1 26,351; robot2 19,243 cells |
| distance contribution | PASS | same | 6.54378915; 8.25744957 m |
| dispatched/successful goals | PASS | same | dispatch 6/5; success 5/4 |
| feasible/avoidable idle | PASS | same | 110.20/98.66 feasible s; 18.22/0 avoidable s |
| workload shares | PASS | `fairness.component_shares` | first-seen 40.37297% / 59.62703% |
| Jain first-seen fairness | PASS | `fairness.jain_first_seen_cell_fairness` | 0.96425328 |
| 0–180 s scaling window | PASS | `scaling_windows.windows[0]` | available, half-open `[0,180)` |
| 180–360 s window | NOT_AVAILABLE_FOR_180S_HORIZON | scaling windows | artifact does not reach end |
| 360–600 s window | NOT_AVAILABLE_FOR_180S_HORIZON | scaling windows | artifact does not reach end |
| 600–900 s window | NOT_AVAILABLE_FOR_180S_HORIZON | scaling windows | artifact does not reach end |
| 900–1200 s window | NOT_AVAILABLE_FOR_180S_HORIZON | scaling windows | artifact does not reach end |

The available 0–180 window contains 62 coverage samples, 136 bids, 94 candidate
batches, 1 certificate observation, 96 cooperation events, 11 dispatches, and 9
navigation terminals. Later windows are explicitly unavailable rather than
zero-filled.

### START_RELEASE and navigation-authority checks

The protocol contract passed:

```text
payload_parse_complete=true
work_availability_complete=true
certificate_payload_complete=true
common_start_release_complete=true
start_release_count=1
start_release_sim_time_s=38.24
pre_release_dispatch_count=0
complete=true
```

Both accepted handoff/frontend records identify the shared evidence hash, both
robots were ready, and the run had no pre-release dispatch. The raw action replay
also demonstrates the required authority distinction: the protocol ledger has 9
scientific terminal goals with 2 still active at the horizon, while the raw
action-status streams include one cleanup `ABORTED` state per robot after the
boundary. These are not flattened into one counter.

### Final verdict

The environment/runtime failure is fixed. This corrected NAT run reached the
scientific horizon with the real two-robot stack, not just Webots physics:
robot-state publishers survived, controllers received robot descriptions, odom,
maps, frontiers, cooperation, navigation, handoff, and START_RELEASE evidence
were present, and finalization plus native-bag offline replay completed.

It is a valid 180-second complete-evidence smoke. It is not a claim that the
mission reached exploration termination, that short-horizon performance targets
were all met, or that unavailable reciprocal/raster/milestone fields occurred.

```text
FRESH_180S_RETRY_PASS
```
