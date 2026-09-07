# Condition-C 600 s certificate-latency-fix validation

Date: 2026-09-08
Source commit: `6f820e8e882cf1d09ba0b1dcc3f54d6052895dae` (`6f820e8`)
Launch command:

```text
python3 src/my_epuck_project/tools/run_thesis_experiment.py --condition C --horizon 600
```

## 1. Run information

Artifact root:
`/home/arash/webots_ws_clean_validation_20260823/results/thesis_condition_C_600s_20260907T234020Z/`

Observer artifact:
`/home/arash/webots_ws_clean_validation_20260823/results/thesis_condition_C_600s_20260907T234020Z/fast_trial_20260907T234025Z/observer/fast_trial_20260907T234025Z/`

The canonical preflight passed with the clean project install, approved Webots
driver prefix, NAT endpoint `172.18.32.1`, CycloneDDS, ROS domain `230`, and
Webots port `23420`. The frozen configuration was Condition C,
`frontier_cost_only`, `MODE_B`, canonical close-start 20 ms scan-matching world,
seed 1001, full sensors, ideal encoders, fast/headless Webots,
`use_sim_time=true`, scientific raw capture, deferred synchronized-map frames,
20 ms Supervisor GT, and 20 ms contact sampling. The recorded candidate query
budget is 8.

Both robots and the Supervisor connected, both robots reached the common
`START_RELEASE` at simulation time `34.18 s`, and the run reached
`SIM_TIME_COMPLETE` at `600.08 s` without watchdog termination. Shared-map
coverage requests ran from `20.04 s` through `600.08 s` (291 samples); the
canonical shared-map streams were equivalent at every reconstructed coverage
sample.

Finalization status:

- artifact contract: `COMPLETE`, `missing=[]`;
- clean shutdown: `true`;
- offline evaluator: `thin_offline_evaluation_2.0`, `complete=true`;
- raw rosbag export: `complete=true`, recorder return code `0`;
- fixed-horizon mission status: `INCOMPLETE` because frontier work was not
  exhausted by the 600 s horizon; this is distinct from artifact/evaluator
  finalization and does not invalidate the bounded horizon artifact.

## 2. Certificate latency comparison

| Measure | Previous baseline | Query-cap-8 run | Result |
|---|---:|---:|---|
| Terminal → next dispatch p50 | 43.88 s | 21.56 s | lower |
| Terminal → next dispatch p95 | 58.12 s | 81.22 s | higher |
| Certificate-deferred time inside finite terminal→dispatch joins | 206.76 s | 133.44 s | lower |
| Robot1 certificate-deferred time | 55.98 s | 66.44 s | higher |
| Robot2 certificate-deferred time | 150.78 s | 67.00 s | lower |

The deferred-time values use the existing certificate-deferred state-transition
semantics, restricted to finite terminal-to-next-dispatch joins. The current run
also has an unresolved final certificate-deferral episode beginning at `463.10
s` and remaining open through the `600.76 s` evaluator clock; its `137.66 s`
horizon-open span is not a finite terminal-to-dispatch latency and is not added
to the table’s finite-join value.

Certificate observations in the current artifact are complete:

- 99 certificate publications: 77 failed/deferred and 22 certified;
- after collapsing same-time replicated publications: 49 failed evaluation
  instants and 16 certified instants;
- failed-publication reasons: 68 `UNQUERIED_OPTION_CAN_BEAT_EVALUATED_ASSIGNMENT`
  and 9 `MISSING_UNQUERIED_LOWER_BOUNDS`;
- previous baseline: 157 non-certifying and 28 certifying publications, or 99
  failed and 18 successful evaluation instants after collapse.

Query evidence shows the intended cap was active:

- 154 frontier lifecycle cycles selected 1,156 exact candidate queries,
  averaging `7.51` selected queries per cycle and never exceeding 8;
- native action replay recorded 1,127 terminal `ComputePathToPose` requests:
  1,078 succeeded and 49 aborted, averaging `7.32` terminal requests per
  lifecycle cycle;
- 1,078 plan records were replayed;
- previous baseline action replay recorded 1,006 terminal requests, 641
  successful requests, 365 aborted requests, and 641 plan records.

The cap increased exact-query throughput and reduced the finite aggregate
certificate-deferral time, but it did not improve the p95 latency: the long
open final deferral and the robot1 result prevent calling the latency objective
validated.

## 3. Idle comparison

| Robot | Baseline avoidable idle | Query-cap-8 avoidable idle | Baseline productive engagement | Query-cap-8 productive engagement | Baseline longest idle | Query-cap-8 longest idle |
|---|---:|---:|---:|---:|---:|---:|
| robot1 | 56.9988% | 54.0691% | 43.0012% | 45.9309% | 41.12 s | 32.56 s |
| robot2 | 22.8264% | 50.0840% | 77.1736% | 49.9160% | 15.32 s | 10.68 s |

There is no consistent idle reduction correlated with the certificate change.
Robot1 improved only modestly while its finite certificate-deferred component
increased from 55.98 s to 66.44 s. Robot2’s finite certificate-deferred
component fell from 150.78 s to 67.00 s, but its avoidable idle increased from
22.8264% to 50.0840%. The latency fix therefore did not produce a consistent
mission-level idle benefit.

## 4. Safety and regression checks

- `DEGRADED_SOLO`: 0 events and 0 launch-log occurrences.
- Traffic scheduler: 86 `TRAFFIC_AWARE_PAIR_SKIP` decisions and 26
  `TRAFFIC_AWARE_PAIR_FALLBACK` decisions; no explicit
  `TRAFFIC_WAITING`, `TRAFFIC_PRIORITY_GRANTED`, or
  `TRAFFIC_CONFLICT_CLEARED` records were emitted.
- Collision/StopCircle: 0 collision-monitor interventions and 0 collision
  messages; the 12 `StopCircle` log matches are lifecycle create/destroy
  messages only.
- Stuck/no-progress/oscillation: 0 offline motion-anomaly episodes for each
  robot.
- Dispatch: 16 accepted goals, 15 successes, 1 navigation failure, and 0
  cancellations. The failure was robot2’s first goal; all later dispatched
  goals were accounted for by the replay.

## 5. Final verdict

**CERTIFICATE_LATENCY_FIX_NO_EFFECT**

The query-cap change increased query throughput and reduced finite aggregate
certificate-deferral time, but the authoritative terminal→dispatch p95 and
robot2 avoidable-idle fraction regressed. The fail-closed certificate contract
remained observed and complete, and the run pipeline itself passed.
