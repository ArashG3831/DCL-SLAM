# Condition-C 600 s path-evaluation-cache validation

Date: 2026-09-08

Source commit: `9587d8fa0e8556e2357009bd062f27f0d5556684` (`9587d8f`)

Comparison baseline: `6f820e8` (`thesis_condition_C_600s_20260907T234020Z`)

## 1. Run information

Canonical launch command:

```text
python3 src/my_epuck_project/tools/run_thesis_experiment.py --condition C --horizon 600
```

Current artifact root:

`/home/arash/webots_ws_clean_validation_20260823/results/thesis_condition_C_600s_20260908T005703Z/`

Observer artifact:

`/home/arash/webots_ws_clean_validation_20260823/results/thesis_condition_C_600s_20260908T005703Z/fast_trial_20260908T005708Z/observer/fast_trial_20260908T005708Z/`

The canonical launcher preflight passed with the isolated project install,
approved Webots driver prefix, WSL NAT endpoint `172.18.32.1:23420`, CycloneDDS
loopback profile, and ROS domain `230`. The worktree was clean at launch. The
frozen configuration was Condition C, `frontier_cost_only`, `MODE_B`, seed
1001, the canonical close-start 20 ms scan-matching world, full sensors, ideal
encoders, fast/headless Webots, simulated time, scientific raw capture,
deferred synchronized-map frames, and 20 ms GT/contact capture. The exact local
path cache was active with its fixed maximum size of 32 entries.

Both robots and the Supervisor connected. Both robots reached the common
`START_RELEASE` at simulation time `52.70 s`; no pre-release dispatch occurred.
The run reached `SIM_TIME_COMPLETE` at `600.08 s`, without watchdog
termination or horizon overrun.

The native `/clock` bag samples give the active interval:

| Quantity | Value |
|---|---:|
| First advancing simulation sample | `0.02 s` |
| Final horizon sample | `600.08 s` |
| Active simulated interval | `600.06 s` |
| Active wall interval | `255.439949356 s` |
| Authoritative active RTF | **`2.349124×`** |
| Total launch wall time, including startup/finalization | `336.238092 s` |

Acceptance status:

- artifact contract: `COMPLETE`, `missing=[]`;
- clean shutdown: `true`;
- native rosbag export: `complete=true`;
- offline evaluator: complete, with all existing evaluator reports generated;
- shared-map coverage: 291 reconstructed coverage samples through the horizon,
  final shared-stream equivalence difference `0`, final known cells `122077`;
- GT: `30050` samples per robot, `60100` total;
- contact: `30050` samples per robot, `60100` total;
- no required artifact was missing.

The launcher recorded normal SIGINT teardown messages from external controller
processes, but the runner returned `0`, marked `clean_shutdown=true`, and
completed all finalization and replay contracts.

## 2. Cache-specific telemetry

Final allocator health counters were:

| Robot | Hits | Misses | Context invalidations | Evictions |
|---|---:|---:|---:|---:|
| robot1 | 16 | 158 | 13 | 30 |
| robot2 | 1 | 213 | 42 | 29 |
| **Total** | **17** | **371** | **55** | **59** |

There were 388 cache lookups in total, or 2.85 lookups per recorded candidate
batch (136 batches). The exact planner-attribution ledger records 2
`ALLOCATOR_BID.REUSED` results. The 17 lookup hits and 2 planner-reuse
attributions are different telemetry layers; both are reported without treating
the lookup counter as a planner-result count. The baseline predates this
telemetry and therefore has no comparable cache counters; it is not treated as
zero.

The native action replay recorded 1,560 and 1,606 ComputePathToPose status
messages for robot1 and robot2, respectively, and 1,546 reconstructed plan
records. These are recorder/replay counts, not a claim that every status message
was a distinct planner request.

## 3. Latency comparison

### 3.1 Existing evaluator latency outputs

| Measure | `6f820e8` baseline | `9587d8f` cache run | Change |
|---|---:|---:|---:|
| Terminal → next dispatch count | 14 | 24 | more completed cycles |
| Terminal → next dispatch p50 | 21.56 s | **6.44 s** | −15.12 s |
| Terminal → next dispatch p95 | 81.22 s | **63.68 s** | −17.54 s |
| Terminal → next dispatch maximum | 81.22 s | **66.20 s** | −15.02 s |
| Certificate-observation p50 | 17.32 s | **13.22 s** | −4.10 s |
| Certificate-observation p95 | 72.22 s | **61.02 s** | −11.20 s |
| Bid latency p50 | 5.56 s | **2.80 s** | −2.76 s |
| Bid latency p95 | 61.30 s | **11.70 s** | −49.60 s |
| Agreement → navigation-goal p50 | 0.12 s | 0.12 s | unchanged |
| Agreement → navigation-goal p95 | 0.34 s | 0.66 s | +0.32 s |

The finite terminal-to-dispatch certificate-deferral crosswalk is `133.44 s`
for the baseline and `189.74 s` for the cache run. This total is not a
per-request rate: the cache run has 24 finite joins versus 14 in the baseline.
The higher aggregate therefore does not contradict the lower per-join p50/p95.
The crosswalk is the existing archaeology attribution of the finite joins, not
a replacement scientific metric.

For the current run's authoritative p95 terminal-to-dispatch interval, the
recorded attribution was approximately:

| Owner in the p95 interval | Time |
|---|---:|
| Certificate waiting | 47.40 s |
| Peer-bid synchronization | 8.96 s |
| Local planner/query lease | 1.92 s |
| Fresh-snapshot gate | 0.34 s |
| Canonical-round formation | 2.28 s |
| Other allocator coordination | 2.68 s |
| Crosswalk total | 63.58 s |

The 0.10 s difference from the authoritative 63.68 s replay value is the
known event-boundary versus replay-boundary difference. The baseline p95
interval had 27.34 s of peer-bid wait, 11.50 s of query-lease wait, 35.34 s
of certificate attribution, and 3.92 s of round/canonical attribution.

### 3.2 Dispatch → first motion cross-check

This is calculated from the existing timeseries using the established first
nonzero-motion threshold; it is a cross-check, not a new evaluator definition.

| Robot | Baseline p50 / p95 / max | Cache p50 / p95 / max |
|---|---:|---:|
| robot1 | 1.38 / 1.78 / 1.88 s | 1.56 / 2.15 / 2.22 s |
| robot2 | 1.22 / 7.64 / 11.46 s | 1.50 / 1.87 / 2.00 s |

## 4. Idle comparison

| Robot | Baseline idle | Cache idle | Change | Baseline productive | Cache productive | Cache longest idle |
|---|---:|---:|---:|---:|---:|---:|
| robot1 | 133.94 s / 54.0691% | **91.72 s / 27.3660%** | −42.22 s / −26.7031 pp | 113.78 s / 45.9309% | **243.44 s / 72.6340%** | 10.78 s |
| robot2 | 101.40 s / 50.0840% | 102.82 s / **30.6596%** | +1.42 s / −19.4244 pp | 101.06 s / 49.9160% | **232.54 s / 69.3404%** | 10.88 s |
| **Combined** | **235.34 s** | **194.54 s** | **−40.80 s** | 214.84 s | 475.98 s | — |

Robot1 improved in both absolute and fractional avoidable idle. Robot2's
fraction fell because its feasible-work denominator increased substantially;
its absolute avoidable idle rose by 1.42 s. The combined exact avoidable-idle
total fell by 40.80 s.

The existing idle-owner crosswalk for the cache run was:

| Owner | robot1 | robot2 | Combined |
|---|---:|---:|---:|
| Certificate | 43.40 s | 27.64 s | 71.04 s |
| Peer-bid synchronization | 10.68 s | 19.00 s | 29.68 s |
| Fresh snapshots | 10.68 s | 15.08 s | 25.76 s |
| Canonical round formation | 5.74 s | 0.00 s | 5.74 s |
| Other allocator coordination | 21.22 s | 31.86 s | 53.08 s |
| Local query lease | 0.00 s | 9.24 s | 9.24 s |
| **Exact avoidable idle** | **91.72 s** | **102.82 s** | **194.54 s** |

Grouped allocator/coordination is 114.26 s, certificate waiting is 71.04 s,
and planner/query-lease attribution is 9.24 s. Thus planner/query lease is a
smaller remaining owner than allocator coordination.

## 5. Safety and regression checks

| Check | Baseline | Cache run |
|---|---:|---:|
| `DEGRADED_SOLO` | 0 | 0 |
| Traffic waits/grants/conflict clears | 0 explicit events | 0 explicit events |
| Collision-monitor interventions | 0 | 0 |
| StopCircle/collision safety interventions | 0 | 0 |
| Stuck/no-progress/oscillation | 0 / 0 / 0 | 1 stuck episode / 0 / 0 |
| Dispatches | 16 | 26 |
| Navigation successes | 15 | 26 |
| Navigation failures | 1 | 0 |
| Cancellations | 0 | 0 |

The cache run's sole motion anomaly was robot2 `STUCK_STARTED` at `375.10 s`
and `STUCK_CLEARED` at `377.10 s`; no no-progress or oscillation episode was
reported. Collision-monitor lifecycle log entries are not collision
interventions and were not counted as such.

## 6. Exploration and map outputs

| Metric | `6f820e8` baseline | `9587d8f` cache run |
|---|---:|---:|
| Final known area | 109.007995 m² | 109.869295 m² |
| Final known cells | 121120 | 122077 |
| Known-area AUC | 41233.249820 m²·s | 34369.534182 m²·s |
| Known-cell AUC | 45814724.070000 cells·s | 38188373.020000 cells·s |
| Total odometry distance | 34.716756 m | 45.146950 m |
| Shared-map final-cell difference | 0 | 0 |
| Trajectory-overlap IoU | 0 | 0.001863933 |
| Jain fairness | 0.942798046 | **0.995077501** |

Cache-run ownership and workload values:

- first-seen cells: robot1 `65612` (53.5167%), robot2 `56989` (46.4833%);
- later duplicate cells: robot1 `12761`, robot2 `29917`;
- distance: robot1 `24.897598 m`, robot2 `20.249352 m`;
- dispatched/successful goals: `13/13` per robot;
- shared trajectory overlap: 2 intersection bins, 1073 union bins, 0.05 m
  bin size;
- shared final map: 122077 known cells, 114501 free cells, 7576 occupied
  cells, 0.03 m resolution;
- shared-map GT distance quality: mean `0.531955 m`, RMS `0.639179 m`, p95
  `1.182899 m`.

The offline handoff report is `NO_ACCEPTED_HANDOFF`, which is the expected
status for this already-aligned shared-stack run and is not a failed acceptance
gate. The deferred replay contracts for coverage, pair decisions, agreements,
and Nav2 diagnostics are present and marked authoritative; warning replay is
`PARITY_PASS`.

## 7. Answers to the cache questions

1. **Did the cache reduce planner/query-lease contribution?**  Modestly in the
   observed attribution: exact avoidable-idle planner/query ownership fell from
   the baseline's `12.80 s` to `9.24 s`, and the current p95 query-lease share
   is `1.92 s` versus the baseline p95's `11.50 s`. The cache itself had only
   17 lookup hits and 2 exact planner-result reuses, so this run does not claim
   that it removed most planner work. Planner/query lease remains secondary.

2. **Did it reduce terminal-to-dispatch latency?**  Yes in this paired bounded
   observation: p50 fell from `21.56 s` to `6.44 s`, p95 from `81.22 s` to
   `63.68 s`, and maximum from `81.22 s` to `66.20 s`. Agreement-to-dispatch
   remained sub-second, so the improvement is in the upstream round/bid/path
   formation interval rather than a changed dispatch safety gate.

3. **Did it reduce avoidable idle?**  Yes for the exact combined idle total and
   for both percentage values; robot1 also improved in absolute seconds, while
   robot2's absolute idle was 1.42 s higher despite its lower percentage.

4. **Did it expose another dominant bottleneck?**  Yes. After the cache run,
   allocator coordination—fresh peer snapshots, peer-bid synchronization,
   canonical-round formation, commitment/other coordination—was the largest
   non-certificate owner (`114.26 s` of exact avoidable idle and `175.22 s` of
   finite terminal-gap crosswalk attribution). Certificate waiting remains the
   largest component of the current p95 terminal gap (`47.40 s`). No cache,
   certificate, freshness, traffic, or dispatch semantics were changed during
   this validation.

## 8. Final verdict

**PATH_EVALUATION_CACHE_VALIDATED_WITH_MEASURED_BENEFIT**

The single canonical 600 s run passed every requested acceptance gate. The
bounded exact cache was active, produced nonzero reuse telemetry, and coincided
with lower terminal-to-dispatch tail latency, lower combined avoidable idle,
more successful dispatches, and no safety-contract regression. The remaining
latency is still dominated by certificate and allocator round-formation work;
this report does not propose a new fix.
