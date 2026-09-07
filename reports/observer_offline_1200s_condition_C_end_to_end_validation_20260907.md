# Condition C 1200 s end-to-end observer/evaluation validation

## Verdict

`CONDITION_C_1200S_OFFLINE_PIPELINE_FAIL`

The single authorized 1200 s mission reached the simulation horizon and produced
complete native-bag and GT/contact raw data.  It is not an acceptance-valid run:
the legacy observer was interrupted during finalization, the run manifest was
left non-finalized, the run output was accidentally created below the source
workspace, and the current evaluator did not produce warning or map-quality
results.  No source change or second Webots run was made.

This report separates pipeline validity from scientific target results.  The
measured scientific results are retained below, but they must not be treated as
a campaign-valid Condition C result.

## 1. Source, run, and configuration

| Item | Value |
|---|---|
| Source HEAD at launch | `56c03237fec0a467f5452dd6c414bfb2b174ea97` |
| Branch | `validation/nat-gate-20260826` |
| Actual run directory | `/home/arash/webots_ws_clean_validation_20260823/fast_trial_20260907T133814Z/observer/fast_trial_20260907T133814Z-01/` |
| Intended result root | `/home/arash/webots_ws_clean_validation_20260823/results/observer_offline_1200s_condition_C_20260907/` |
| Observer path | Salvaged legacy/deferred observer (`--observer-architecture legacy`) |
| Condition / strategy / gate | C / `frontier_cost_only` / `MODE_B` |
| World profile | `large_unknown_pose_close_start_20ms_scan_matching` |
| Requested/effective seed | 1001 / 1001 |
| Sensors/encoders | full / ideal wheel encoders |
| Physics | Webots 20 ms, fast/headless, no rendering/RViz |
| Evidence | native rosbag, scientific raw capture, GT 20 ms, contacts 20 ms |
| ROS | Jazzy, CycloneDDS loopback, domain 230 |
| Webots networking | WSL NAT, `172.18.32.1:23421` |
| Project install | `/home/arash/webots_ws_clean_validation_20260823/install_observer_offline_fresh_180s_20260907` |
| Driver install | `/home/arash/webots_ws_close_validation_2eb/install/webots_ros2_driver` |
| Canonical world SHA-256 | `feda86d1c1ba8b3f1b18c8f216a079c61fff222dbda09b0326c68f8e0ef9bb86` |

The preflight passed before launch: tracked source was clean, Jazzy vendor
library paths were preserved, both required `ldd` checks had zero `not found`
dependencies, the approved driver and NAT endpoint were selected, and no
conflicting process or port listener was present.

The wrapper did not export its `RUN_ROOT` into the clean child environment.
Consequently, the runner defaulted to `.` and created the actual run below the
repository root.  Those generated files are untracked and were not staged.
This also explains the run manifest's `worktree_dirty=true`; it reflects the
generated run files in the repository, not a source edit.  The source HEAD was
unchanged.

## 2. Startup, horizon, and shutdown

Startup was healthy:

| Gate | Simulation time / result |
|---|---:|
| Clock ready | 15.821 s |
| Robot interfaces ready | 1.190 s phase |
| Maps ready | 0.911 s phase |
| Nav2 ready | 16.765 s phase |
| Total ready | 35.830 s |
| Common START_RELEASE | 44.34 s |
| Horizon | 1200.00 s |

Both robot state publishers, controllers, odometry, local maps, frontiers,
distributed status/events, handoff, and navigation became active.  The mission
reached the configured horizon without the wall watchdog:

- lifecycle start: `2026-09-07T13:38:13.919017Z`;
- ready: `2026-09-07T13:38:49.749169Z`;
- lifecycle end: `2026-09-07T13:50:49.188453Z`;
- total runner wall duration: `755.269443475001 s`;
- termination: `SIM_TIME_COMPLETE`;
- simulation end: `1200.0 s`;
- launch return code: `0`;
- wall watchdog: armed, not fired;
- horizon overrun: false.

Finalization did not complete.  At `RUN_END` the observer entered
`finalize_passive_rosbag()`, then `_restore_offloaded_artifacts()`, and was
interrupted in `_repair_offloaded_health()` while scanning received-health
records.  The launch log records a second SIGINT and a `KeyboardInterrupt` at
`cooperative_experiment_logger.py` lines 3941–3943 in the installed observer.

Therefore:

- `artifact_finalization.json` / `raw_evidence_finalization.json`: absent;
- `summary.json` / `mission_result.json`: absent;
- run manifest: `artifact_finalization.complete=false`,
  `status=NOT_FINALIZED`, `clean_shutdown=false`;
- `fast_trial_summary.json`: `observer_finalization_complete=false`.

The `double free or corruption` messages and the map-fusion exit appeared during
SIGINT teardown after the horizon.  They are cleanup symptoms in this run, not
evidence of an active-mission startup failure.

## 3. Raw evidence and provenance

`passive_rosbag_export.json` reports `complete=true`, with 53 selected topics.
The native SQLite bag is approximately 593 MiB.  Key counts are:

| Stream | Count |
|---|---:|
| `/clock` | 10,814 |
| `/tf` / `/tf_static` | 206,016 / 11 |
| `/robot1/odom` / `/robot2/odom` | 59,547 / 59,534 |
| `/robot1/map` / `/robot2/map` | 655 / 655 |
| `/robot1/shared_map` / `/robot2/shared_map` | 68 / 62 |
| frontier candidates | 293 / 297 |
| task snapshots | 293 / 297 |
| task bids | 60 / 455 |
| distributed events | 203 / 338 |
| distributed status | 1,204 / 1,203 |
| NavigateToPose feedback | 62,505 / 53,285 |
| `/rosout` | 11,423 |

The raw scientific contract and protocol replay both reported complete.  The
two `/cslam/.../exploration_status` and `/cslam/.../exploration_event` streams
were present in the bag with zero messages; canonical distributed status/event
streams were nonempty and complete.  These zeros were not converted into
missing protocol evidence.

Forensic capture produced 60,034 Supervisor steps, 149 time-stamped map
snapshots plus four final maps, raw TF/odom files, synchronized-frame evidence,
and scan-correction evidence.  Generated evaluator files are in the actual run
directory, including `thin_metrics.json`, `avoidable_idle.json`,
`snappiness.json`, `cooperation_summary.json`, `motion_anomalies.json`,
`fairness.json`, `scaling_windows.json`, `map_quality_report.json`, and
`handoff_reciprocal_verification.json`.

## 4. RTF

The authoritative active interval was measured from native-bag `/clock`
timestamps: first strictly advancing sample at 0.02 s through the last sample at
or before 1200 s.  Startup, shutdown, and offline replay were excluded.

- active simulation delta: `1199.98 s`;
- active monotonic/recorded wall delta: `656.185635313 s`;
- authoritative active RTF: **`1.8287203124x`**;
- Supervisor-lifetime secondary RTF: `1.7536607300x`;
- total runner/lifecycle wall duration: `755.269443475001 s`;
- offline replay wall duration: not used for RTF; the replay command duration was
  not captured as a dedicated authoritative timing boundary;
- finalization duration: unavailable because finalization was interrupted.

Window RTFs, measured from the same `/clock` stream, were:

| Half-open window | Active RTF |
|---|---:|
| `[0,180)` | 2.30991x |
| `[180,360)` | 1.97650x |
| `[360,600)` | 1.61212x |
| `[600,900)` | 1.71552x |
| `[900,1200)` | 1.83507x |

The project hard RTF gate of 2.0x therefore failed.  This is a scientific
performance-target result, separate from the pipeline failures above.

## 5. Coverage and exploration

The current replay used the Condition-C shared-map stream.  It found 109 shared
coverage samples from 17.04 s through 263.54 s, not through the 1200 s horizon.
The shared final forensic maps also have header times around 258–262 s despite
being written at shutdown.  The raw bag contains later local-map messages, but
the current Condition-C replay does not substitute local maps for the shared-map
contract.

| Metric | Replayed result |
|---|---:|
| Final known cells | 73,416 |
| Final known area | 66.074397 m² |
| World extent area | 400 m² |
| Final world coverage | 16.5186% |
| Known-area AUC | 9,409.409562 m²·s over available samples |
| Known-cell AUC | 10,454,899.980003588 cell·s over available samples |
| Coverage samples | 109 |
| Coverage gain per odom metre | 2.6147836 m²/m |
| Known cells per odom metre | 2,905.3153 cells/m |

The 25%, 50%, 75%, and 90% milestones were `NOT_REACHED`, not zero.

Ownership/duplicate replay produced:

- unique first-seen cells: robot1 `15,029`, robot2 `57,616`;
- first-seen shares: robot1 `20.6883%`, robot2 `79.3117%`;
- later duplicate cells: robot1 `55,203`, robot2 `8,482`;
- duplicated-known fraction: `0.87666047`;
- total known union cells: `72,645`;
- shared-stream final equivalence difference: `0` cells.

Because shared-map evidence stops at 263.54 s, map growth, coverage AUC, and
coverage gain for later long-horizon windows are unavailable rather than zero.
The current evaluator did not emit a separate simultaneous-observation field.

## 6. Motion and efficiency

| Metric | Robot 1 | Robot 2 | Total/other |
|---|---:|---:|---:|
| GT distance | 13.973008 m | 11.238240 m | 25.211247 m |
| Odom distance | 14.007809 m | 11.261738 m | 25.269547 m |
| GT samples | 60,034 | 60,034 | 120,068 rows |
| GT active time | 156.26 s | 128.52 s | — |
| GT idle time | 1,044.40 s | 1,072.14 s | — |
| Feasible-work time | 1,054.26 s | 1,068.04 s | — |
| Avoidable idle | 0 s | 941.24 s | — |
| Avoidable-idle fraction | 0% | 88.1278% | — |
| Productive engagement | 1,054.26 s | 126.80 s | — |
| Productive fraction | 100% | 11.8722% | — |
| Longest avoidable idle | 0 s | 936.12 s | — |

The longest intervals were robot1 `[268.98,1200.66)` at 931.68 s and robot2
`[264.54,1200.66)` at 936.12 s, both classified with feasible work available.
The exact interval/reason records are in `avoidable_idle.json`.

GT trajectory overlap used 0.05 m bins: intersection 46, union 597, IoU
`0.07705193`.  A separate repeated-distance scalar was not emitted by the
current evaluator; duplicate-cell attribution was available as above.

Frozen target results:

| Target | Result |
|---|---|
| Avoidable idle <=5% | FAIL for robot2; robot1 PASS |
| Productive engagement >=95% | FAIL for robot2; robot1 PASS |
| No unexplained avoidable idle >5 s | FAIL; 931.68 s / 936.12 s intervals |

## 7. Cooperation, navigation, certificates, and DNU

Protocol payload parsing and the protocol contract reported complete.  Replayed
counts:

| Item | Result |
|---|---:|
| Candidate batches | 590 (robot1 293, robot2 297) |
| Task snapshots | 590 |
| Candidate-generation records | 1,180 |
| Bid records / batches | 2,914 / 515 |
| Pair-decision records / unique hashes | 2 / 1 |
| Dispatches | 58 (29 each) |
| Agreement publications/unique rounds/decisions | 0 / 0 / 0 |
| Claims | 0 |
| Continuations | 0 |
| Scientific navigation terminals | 57 (robot1 28, robot2 29) |
| Successful terminals | 17 (8, 9) |
| Failed terminals | 40 (20, 20) |
| Goals active at horizon | 1 dispatch without a terminal |
| Failure records | 44: 40 `CONTROLLER_NO_PROGRESS`, 4 `TF_OR_LIFECYCLE` |

The raw action-status replay reported cleanup `ABORTED`/`CANCELED` statuses
(robot1 5/15, robot2 8/12).  These are not flattened into scientific terminal
failures; the protocol ledger remains the scientific-horizon authority.

Certificate state was `OBSERVED`, with 3 complete observations and no payload
parse errors.  The observations were:

| Simulation time | Reason | Evaluated | Detected-not-queried | Blocking | Selected score | Optimistic bound | Certified |
|---:|---|---:|---:|---:|---:|---:|---|
| 48.00 s | `UNQUERIED_OPTION_CAN_BEAT_EVALUATED_ASSIGNMENT` | 4 | 24 | 353 | -26.156467 | -8.820858 | false |
| 48.00 s | `UNQUERIED_OPTION_CAN_BEAT_EVALUATED_ASSIGNMENT` | 4 | 24 | 353 | -26.156467 | -8.820858 | false |
| 1167.78 s | `NO_UNQUERIED_OPTIONS` | 12 | 0 | 0 | -55.659047 | `-inf` | true |

The DNU/status series contained 2,997 observations, 576 nonzero observations,
18,176 total detected-not-queried candidates, and a maximum of 148.  The series
has no structured DNU reason field; no reason breakdown was fabricated.

## 8. Snappiness and timestamp joins

| Join/metric | Count | p50 | p95 | Maximum | Target/status |
|---|---:|---:|---:|---:|---|
| Terminal → next dispatch | 56 | 0.24 s | 29.24 s | 49.22 s | target <=3 s: FAIL |
| Dispatch → terminal | 57 | 42.24 s | 55.76 s | 58.76 s | measured |
| Candidate generation | 590 | 0 s | 0 s | 0 s | measured |
| Task → bid | 2,914 | 4 s | 46 s | 58 s | measured |
| Certificate | 2 | 0.20 s | 0.20 s | 0.20 s | measured |
| Agreement → NAV_GOAL_SENT | 0 | — | — | — | NOT_INVOKED/no join |
| Handoff → first shared goal | 1 | 3.86 s | 3.86 s | 3.86 s | target <=3 s: FAIL |

All values use simulation/header time.  Missing joins remain unavailable; they
were not represented as zero.

## 9. Reliability and anomalies

The offline MotionDetector replay was available:

- robot1: 1 `STUCK` episode, `[267.10,271.10)`, 4.0 s;
- robot2: 26 `STUCK` episodes, total 98.0 s, maximum 5.0 s;
- event totals: robot1 2 transitions, robot2 52 transitions;
- valid zero-event semantics were preserved where no event occurred.

The current evaluator emitted no `warnings.jsonl` and therefore returned
`warnings.available=false`.  `/rosout` raw evidence exists (11,423 messages and
`rosout_receipts.jsonl`), but warning normalization/reporting was not produced.
`nav2_diagnostics.jsonl` is empty; action status and protocol navigation
outcomes are available.  Warning/stale-topic/recovery reporting is therefore
not complete for this run.

## 10. GT, contact, and physical validation

| Evidence | Result |
|---|---|
| Supervisor GT rows | 60,034 per robot; 120,068 evaluated rows |
| GT requested/effective cadence | 0.02 s / 20 ms |
| Supervisor steps | 60,034 |
| Contact queries | 120,068 |
| Contact cadence | 20 ms |
| Expanded contact rows | 684,101 |
| Contact samples | 60,034 per robot |
| Contact episodes | 1 per robot under current episode definition |
| Contact evidence | available; zero-vs-missing distinction preserved |

The capture retained GT pose/velocity and per-contact expansion.  The current
report does not reinterpret contacts as collisions beyond the existing observer
episode definition.

## 11. Handoff and START_RELEASE

Two accepted structured frontend records were present at 12.26 s, with evidence
hashes and estimator quality.  Both robots were ready and accepted handoff at
the common `START_RELEASE` at 44.34 s.  Pre-release dispatch count was zero.

Accepted transform/quality evidence included:

- transform `[-0.5000181780, 0.0052181311, 0.0063957349]`;
- physical-GT translation residual `0.0052181628 m`;
- yaw residual `0.0063957349 rad`;
- final confidence `0.6723175`;
- geometric inlier ratio `0.9824493`;
- median residual `0.0094897 m`;
- p95 residual `0.0180693 m`.

`handoff_reciprocal_verification.json` reports `NO_ACCEPTED_HANDOFF` for
reciprocal direction verification: two accepted records exist, but the replay
does not have two explicit source→target directions.  This is an explicit
unverified state, not a fabricated PASS or zero.

## 12. Map and SLAM quality

The four final map artifacts exist:

- `forensic/maps/robot1_map_final.npz` — 555×343, header 1199.0 s;
- `forensic/maps/robot2_map_final.npz` — 817×377, header 1199.0 s;
- `forensic/maps/robot1_shared_map_final.npz` — 825×383, header 261.98 s;
- `forensic/maps/robot2_shared_map_final.npz` — 825×383, header 257.98 s.

However, `map_quality_report.json` is:

```json
{"available": false, "reason": "no established map-quality result is present"}
```

The evaluator also reports `forensic/physical_gt_evaluation.json` absent.  The
approved `analyze_cooperative_decision_offline.py` source exists, but the
current installed adapter did not produce its direct geometry output for this
run.  No external truth-raster IoU was invented.  Consequently local/shared
map-quality, occupancy agreement, and alignment fields are unavailable in this
fresh evaluation and this required family is not complete.

## 13. Fairness and workload

The approved Jain first-seen-cell fairness index was `0.74422991`.

| Component | Robot 1 | Robot 2 |
|---|---:|---:|
| First-seen cells | 15,029 | 57,616 |
| First-seen share | 20.6883% | 79.3117% |
| Later duplicate cells | 55,203 | 8,482 |
| Distance | 14.007809 m | 11.261738 m |
| Dispatched goals | 29 | 29 |
| Successful goals | 8 | 9 |
| Feasible work | 1,054.26 s | 1,068.04 s |
| Avoidable idle | 0 s | 941.24 s |
| Productive engagement | 1,054.26 s | 126.80 s |

The scalar and all components are present; the severe workload imbalance is a
scientific result, not hidden by the scalar.

## 14. Scaling windows

Windows use half-open `[start,end)` simulation-time semantics.  Event, idle, and
anomaly data are available in all five windows.  Coverage is unavailable after
263.54 s because the Condition-C shared-map series stopped.

| Window | Coverage | Coverage gain / AUC | Dispatches | Terminals | Bids | Certificates | Anomaly events |
|---|---|---:|---:|---:|---:|---:|---:|
| [0,180) | AVAILABLE | 48.085198 m² / 4413.768076 | 11 | 9 | 268 | 2 | 0 |
| [180,360) | AVAILABLE | 15.183899 m² / 4843.046942 | 13 | 13 | 0 | 0 | 14 |
| [360,600) | NOT_AVAILABLE_SOURCE_HAS_NO_TIMESERIES | — | 12 | 12 | 228 | 0 | 21 |
| [600,900) | NOT_AVAILABLE_SOURCE_HAS_NO_TIMESERIES | — | 13 | 13 | 558 | 0 | 13 |
| [900,1200) | NOT_AVAILABLE_SOURCE_HAS_NO_TIMESERIES | — | 9 | 10 | 1848 | 1 | 6 |

Avoidable idle by window was:

| Window | Robot 1 avoidable / feasible | Robot 2 avoidable / feasible |
|---|---:|---:|
| [0,180) | 0 / 100.30 s | 5.12 / 87.86 s |
| [180,360) | 0 / 113.30 s | 95.46 / 139.52 s |
| [360,600) | 0 / 240.00 s | 240.00 / 240.00 s |
| [600,900) | 0 / 300.00 s | 300.00 / 300.00 s |
| [900,1200) | 0 / 300.00 s | 300.00 / 300.00 s |

The later-window coverage fields are explicitly unavailable, never zero-filled.
Thus the long-horizon scaling evidence is incomplete even though protocol and
work-availability streams reach the horizon.

## 15. Pipeline status and exact failure causes

| Requirement | Status | Evidence |
|---|---|---|
| Canonical configuration/preflight | PASS before launch | `results/.../preflight_environment.txt` |
| Robot startup | PASS | `launch.log`, startup gates |
| Horizon exactly 1200 s | PASS | `fast_trial_summary.json` |
| Watchdog | PASS/not fired | `fast_trial_summary.json` |
| Native bag export | PASS | `passive_rosbag_export.json: complete=true` |
| Semantic raw contract | PASS | `thin_metrics.json.semantic_contract.complete=true` |
| Protocol payload replay | PASS | `protocol_contract.complete=true` |
| Certificate payload | PASS | `OBSERVED`, 3 complete records |
| GT/contact fidelity | PASS | 20 ms runtime metrics |
| START_RELEASE/pre-release validity | PASS | one release, 44.34 s, zero pre-release dispatch |
| Offline evaluator execution | PASS | sidecars generated; evaluator self-reported `complete=true` |
| Artifact finalization | FAIL | finalizer interrupted; no finalization artifact |
| Clean finalization/shutdown | FAIL | manifest `clean_shutdown=false` |
| Coverage through 1200 s | FAIL | shared-map evidence ends at 263.54 s |
| Warning normalization | FAIL/UNAVAILABLE | `warnings.jsonl` absent despite raw `/rosout` |
| Approved map-quality report | FAIL/UNAVAILABLE | `map_quality_report.available=false` |
| Reciprocal handoff verification | NOT_AVAILABLE | no explicit reverse direction |
| Five-window scaling completeness | FAIL | later coverage unavailable |
| Active RTF >=2.0x | FAIL target | 1.8287203x; target result, not evidence failure |

The evaluator's top-level `complete=true` reflects its raw/semantic/protocol
predicate and does not override the missing live finalization contract or the
missing required map/warning/scaling outputs.  Therefore the overall pipeline
verdict is FAIL.

## 16. Generated artifact paths

Raw and evaluator outputs are retained at:

`/home/arash/webots_ws_clean_validation_20260823/fast_trial_20260907T133814Z/observer/fast_trial_20260907T133814Z-01/`

Important files:

- `../fast_trial_summary.json`;
- `../launch.log`;
- `passive_rosbag/passive_rosbag_0.db3`;
- `passive_rosbag_export.json`;
- `run_manifest.json`;
- `forensic/runtime/runtime_metrics.json`;
- `forensic/manifest.json`;
- `thin_metrics.json`;
- `avoidable_idle.json`;
- `snappiness.json`;
- `cooperation_summary.json`;
- `motion_anomalies.json`;
- `fairness.json`;
- `scaling_windows.json`;
- `map_quality_report.json`;
- `handoff_reciprocal_verification.json`;
- `active_rtf_1200_measurement.txt`;
- `rtf_windows_1200.txt`.

No raw bag or result artifact was committed.  Only this documentation report is
intended for the documentation checkpoint.

## Final pipeline verdict

`CONDITION_C_1200S_OFFLINE_PIPELINE_FAIL`
