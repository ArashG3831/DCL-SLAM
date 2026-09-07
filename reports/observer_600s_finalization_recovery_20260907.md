# 600 s Condition-C finalization recovery

Date: 2026-09-07
Scope: offline recovery of the preserved 600 s run; no Webots run was launched.

## Result under investigation

- Source run: `/home/arash/webots_ws_clean_validation_20260823/results/thesis_condition_C_600s_20260907T180603Z/fast_trial_20260907T180608Z/`
- Observer artifact: `observer/fast_trial_20260907T180608Z-01/`
- Run configuration: Condition C, `frontier_cost_only`, MODE_B, seed 1001, 20 ms physics, full sensors, scientific raw capture, 600 s horizon.
- Raw bag: `observer/fast_trial_20260907T180608Z-01/passive_rosbag/passive_rosbag_0.db3` (399,630,336 bytes).
- Source recorded in the run manifest: `bb6c59ec...` (the source used by the run); current remediation checkpoint: `5241c04e86d42adaef7935d9b12e7d23bfc31453`.

The mission itself completed normally. The original logger finalization did not.

## Proven root cause

The runner sends SIGINT to the observer and waits 60 seconds. The observer was
still executing its normal finalization sequence, including repeated native-bag
replay. The runner then escalated with a second SIGINT. `main()` had restored the
default SIGINT handler before calling `node.finalize(clean)`, so that second
signal became `KeyboardInterrupt` inside
`_replay_pair_decisions_for_parity()` while `rosbag2_py.SequentialReader.open()`
was opening the bag. The traceback is in the run `launch.log` at the finalization
sequence ending at the `reader.open` call.

The earlier health-repair bound did not remove the remaining full-bag replay
sequence, so it did not prevent this failure. This is a shutdown/finalization
coordination defect, not a robotics or scientific-capture defect.

## Source correction

Commit `5241c04` contains the isolated correction:

1. Keep the observer's request-shutdown SIGINT handler installed while
   `node.finalize(clean)` runs.
2. Restore the previous handlers only after finalization returns.
3. Increase the finite observer-finalization barrier from 60 s to 120 s because
   the preserved 600 s evidence path demonstrably exceeds the old barrier.

The correction does not change observer inputs, mission behavior, sampling,
metric definitions, or raw evidence. Focused tests passed: 131 tests, including
the signal-order regression and clean-install imports.

## Preserved evidence and formal status

The preserved run contains complete raw scientific evidence:

- native rosbag export: `complete=true`, return code 0;
- `/clock`: 5,414 messages, 0.02--600.66 s;
- `/tf`: 101,087 messages and `/tf_static`: 11;
- both odometry streams, local/shared map streams, protocol streams, action
  status, rosout receipts, GT, contacts, handoff records, and final maps;
- GT/contact: 30,035 samples per robot (20 ms evidence cadence);
- semantic protocol contract: complete;
- certificate state: `NOT_INVOKED`, not an incomplete invoked certificate;
- START_RELEASE: one event at 36.52 s; pre-release dispatch count: 0.

The original logger did not write `artifact_finalization.json`; therefore the
original formal logger status remains `NOT_FINALIZED` and `clean_shutdown=false`.
That historical status is not rewritten. It is distinct from the recovered
offline scientific result below.

## Offline recovery

The existing `offline_evidence_replay.evaluate_run()` was run against this exact
raw artifact in the clean ROS 2 Jazzy/project environment. It wrote only to:

`observer/fast_trial_20260907T180608Z-01/recovered_offline_evaluation/`

No raw bag, forensic CSV, map, or receipt source was overwritten. The recovered
bundle reports:

- `complete=true`;
- `semantic_contract.complete=true`;
- `protocol_contract.complete=true`;
- payload parsing complete;
- native-bag replay successful;
- evaluator wall time: 29.34 s on the final replay.

Recovered files include `thin_metrics.json`, `avoidable_idle.json`,
`snappiness.json`, `cooperation_summary.json`, `motion_anomalies.json`,
`fairness.json`, `scaling_windows.json`, `map_quality_report.json`, and
`handoff_reciprocal_verification.json`.

## Recovered scientific metrics

### Coverage, motion, and physical evidence

| Metric | Recovered result |
|---|---:|
| Final shared known cells | 113,194 |
| Final shared known area | 101.8745954459 m² |
| World coverage (400 m² extent) | 25.4686488615% |
| Known-area AUC | 37,570.0721884868 m²·s |
| Known-cell AUC | 41,744,526.52 cells·s |
| 25% milestone | 594.08 s |
| 50/75/90% milestones | not reached |
| First-seen cells | robot1 61,520; robot2 53,782 |
| Later duplicate cells | robot1 35,761; robot2 26,804 |
| Duplicated-known fraction | 0.5426185149 |
| Odom distance | robot1 41.6473145063 m; robot2 35.6728636530 m; total 77.3201781594 m |
| GT distance | robot1 41.5863331457 m; robot2 35.6260852747 m |
| GT trajectory-overlap IoU | 0.0157068063 (0.05 m bins) |
| GT rows/contact samples | 30,035 per robot; 60,070 total |
| Contact episodes | one per robot under the existing contact definition |

### Idle, engagement, cooperation, and navigation

| Metric | Robot 1 | Robot 2 |
|---|---:|---:|
| Feasible-work time | 275.34 s | 286.80 s |
| Avoidable idle | 5.68 s (2.0629%) | 0 s (0%) |
| Productive engagement | 269.66 s (97.9371%) | 286.80 s (100%) |
| Longest avoidable idle | 5.68 s | 0 s |
| Candidate batches | 58 | 62 |
| Dispatches | 15 | 19 |
| Scientific terminal goals | 14 | 18 |
| Scientific successes | 13 | 18 |
| Scientific failures | 1 | 0 |

Totals: 120 candidate batches, 294 bid records (290 valid), 34 dispatches,
32 scientific terminal goals, 31 successes, and 1 failure. The protocol ledger
is the scientific-horizon authority. Native action status includes cleanup
ABORTED statuses after the horizon (robot1 2, robot2 1); those are not flattened
into scientific terminal failures.

### Snappiness and anomalies

| Join/metric | Count | p50 | p95 | Max |
|---|---:|---:|---:|---:|
| Terminal → next dispatch | 32 | 0.22 s | 2.44 s | 8.00 s |
| Dispatch → terminal | 32 | 23.92 s | 62.56 s | 86.66 s |
| Candidate generation | 120 | 0 s | 0 s | 0 s |
| Bid latency | 294 | 14.04 s | 27.10 s | 28.10 s |
| Agreement → NAV_GOAL_SENT | 0 | unavailable | unavailable | unavailable |
| Certificate latency | 0 | unavailable | unavailable | unavailable |
| Handoff → first shared goal | 1 | 0.86 s | 0.86 s | 0.86 s |

Motion replay found four STUCK episodes for robot1 and three for robot2; every
episode had a matching clear event. Rosout replay produced 62 deduplicated
warning records from 5,295 receipts: 21 controller, 9 costmap, 483 process, 8
scan, and 2 TF warning records.

### Cooperation and certificate semantics

- Agreements: 0 publications, 0 unique decisions, 0 unique rounds.
- Continuations: 0; claims: 0.
- Certificate: `NOT_INVOKED`, observation count 0. This is a valid state for this
  run, not missing certificate evidence.
- DNU observations: 1,317 (robot1 657, robot2 660); the preserved time series is
  present.
- Protocol status records: 1,197; cooperation events: 247; generation records:
  240; navigation terminals: 32; failure records: 6.

### Handoff and map quality

- Two accepted structured handoff records, common evidence-set hash
  `920cb8c519c66bcf`.
- Accepted times: robot1 14.58 s; robot2 12.26 s.
- Transform: `[-0.500014597990172, 0.00521945719634177,
  0.0064000603775964384]`.
- Translation error against physical GT: 0.0052194776 m.
- Yaw error: 0.0064000604 rad.
- Confidence: approximately 0.68027013; geometric inlier ratio 0.98244930;
  median residual 0.00948971 m; p95 residual 0.01806930 m.
- Reciprocal verifier: `NO_ACCEPTED_HANDOFF`. Both records have the same
  source-to-target direction; no inverse-direction pair exists. This is an
  explicit absence of reciprocal evidence, not an invented zero.
- Approved cooperative map-quality sidecar is available for all four final maps.
  It reports geometry-based mean/RMS/p95 distances and occupancy/boundary
  quantities. For example, shared-map mean/RMS/p95 distances are robot1
  0.5359503/0.6589929/1.2541927 m and robot2
  0.5361920/0.6590810/1.2543945 m.

### Fairness and scaling windows

- First-seen shares: robot1 0.53355536, robot2 0.46644464.
- Jain fairness over first-seen cells: 0.99551635.
- Component shares are present for first-seen cells, distance, duplicates,
  dispatched goals, and successful goals.

The available half-open windows are:

| Window | Coverage gain | Coverage AUC | Dispatches | Terminals | Avoidable idle (r1/r2) |
|---|---:|---:|---:|---:|---:|
| [0,180) | 52.5419976512 m² | 4,004.5528809825 | 11 | 9 | 5.68 s / 0 s |
| [180,360) | 20.8700990670 m² | 12,148.1501009354 | 13 | 13 | 0 s / 0 s |
| [360,600) | 25.0334988809 m² | 20,951.2430274064 | 10 | 10 | 0 s / 0 s |

The `[600,900)` and `[900,1200)` windows are correctly `NOT_AVAILABLE` because
the artifact horizon is 600 s, not zero-filled.

Shared-map continuity was recovered through the horizon: robot1 shared-map last
received ROS time 599.66 s/header 597.96 s; robot2 last received ROS time 598.66
s/header 595.96 s.

## RTF interpretation

Using the frozen active boundary (ready timestamp to the first horizon shutdown
signal), active wall time was approximately 301.5804088 s for 600.06 s of
simulation, or approximately **1.989718×**. The runner's 406.6466 s lifetime is
not an active RTF because it includes startup and the interrupted finalization.
The recovered active value is therefore diagnostic only for this invalid formal
acceptance run and is not a clean final performance claim.

## Final status

- `FINALIZER_REMEDIATION_COMMITTED`: yes, commit `5241c04`.
- `OFFLINE_SCIENTIFIC_RECOVERY_COMPLETE`: yes, existing raw evidence produced a
  complete offline metric bundle.
- `ORIGINAL_FORMAL_LOGGER_FINALIZATION`: `NOT_FINALIZED` because the historical
  second SIGINT interrupted it before its completion artifact was written.
- `FRESH_WEBOTS_RERUN_REQUIRED_FOR_CLEAN_FINALIZATION`: not required for recovery
  of the scientific metrics; a future clean acceptance run is still required if
  the campaign requires a formally complete logger finalization artifact.

No Webots rerun was performed in this recovery task.
