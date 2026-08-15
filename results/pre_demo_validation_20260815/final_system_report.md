# Final known-transform pre-demo validation

## Scope and result

This report covers the production known-transform cooperative system only: RPP, decentralized local SLAM, source-aware shared-map fusion, canonical frontier tasks, Burgard-style decentralized allocation with `beta=1.0`, and local Nav2 ownership. Unknown-initial-pose C-SLAM and physical traffic/doorway validation were not performed.

Three bounded forensic runs were valid for metrics because each passed readiness, connected the Supervisor, captured complete final artifacts, and ended with a clean observer shutdown. The runner classification is `BOUNDED_DIAGNOSTIC`: the runner intentionally stopped at its 300 s mission bound rather than waiting for both robots to declare mission-complete.

## Production allocator

```text
U_t = 1.0
C_i,t = clamp(L_i,t / 18.0 m, 0, 1)
S(i,t) = U_t - beta*C_i,t,    beta = 1.0
```

`L_i,t` is each robot’s own Euclidean Nav2 ComputePathToPose polyline length. The 18 m denominator is the existing feasibility ceiling and makes the project cost dimensionless; it is a project adaptation, not a Burgard normalization. After selection, remaining task utility is reduced by `P(d)=1-d/11.98` only within lidar range and with clear shared-map LOS. Occupancy values above 50 block the reduction. Canonicalization prevents double assignment before this ranking.

The old seven-weight score is retired from production. `legacy_weighted` remains explicit diagnostic-only mode. Traffic is disabled by default and remains experimental until a doorway world is available.

## Fresh production outcomes

| run | bounded duration (s) | accepted | success | terminal failures | cancellations | recovery changes | known-cell gain | combined GT m |
|---|---:|---:|---:|---:|---:|---:|---:|---:|
| trial_01_attempt_01 | 325.5 | 11 | 10 | 1 | 0 | 0 | 106232 | 31.62 |
| trial_02_attempt_01 | 320.0 | 11 | 10 | 0 | 0 | 0 | 94428 | 28.95 |
| trial_03_attempt_01 | 338.6 | 10 | 9 | 0 | 0 | 0 | 96889 | 32.27 |

Aggregate: **32 accepted**, **29 successful**, **1 failed**, zero cancellations, mean duration **328.0 s**, mean known-cell gain **99183**, and mean known-cell gain per combined GT metre **3207.8**.

## GT→odom

| robot | mean GT m | translation RMSE m | median m | p95 m | final m | yaw RMSE rad | final yaw rad | RMSE per 10 m |
|---|---:|---:|---:|---:|---:|---:|---:|---:|
| robot1 | 19.05 | 0.0197 | 0.0036 | 0.0536 | 0.0695 | 0.0059 | -0.0098 | 0.0115 |
| robot2 | 11.90 | 0.0241 | 0.0053 | 0.0437 | 0.0437 | 0.0058 | -0.0086 | 0.0203 |

The alignment is the same initial GT-to-odom convention used by the prior offline analysis; the known transform is not estimated from these trajectories.

## Direct map accuracy against the large-world static geometry

| product | occupied cells mean | mean dist m | median m | p95 m | p99 m | RMS m | off-geometry area >0.05/>0.10/>0.15/>0.25/>0.50 m² |
|---|---:|---:|---:|---:|---:|---:|---|
| R1 local | 5162 | 0.0388 | 0.0259 | 0.1186 | 0.1672 | 0.0552 | 1.304 / 0.273 / 0.147 / 0.029 / 0.000 |
| R2 local | 4104 | 0.0308 | 0.0254 | 0.0756 | 0.1020 | 0.0398 | 0.809 / 0.042 / 0.006 / 0.001 / 0.000 |
| R1 shared | 7449 | 0.0393 | 0.0317 | 0.1117 | 0.1617 | 0.0538 | 1.975 / 0.300 / 0.154 / 0.030 / 0.000 |
| R2 shared | 7436 | 0.0393 | 0.0317 | 0.1117 | 0.1617 | 0.0538 | 1.968 / 0.296 / 0.152 / 0.030 / 0.000 |

Map transforms apply OccupancyGrid origin position/yaw and the captured map→shared transform once. Direct accuracy is distinct from replica consistency.

## Fusion and shared-map consistency

| metric | mean across runs/replicas |
|---|---:|
| transformed source-local occupied union vs fused occupied IoU, R1 | 0.998989 |
| transformed source-local occupied union vs fused occupied IoU, R2 | 0.995975 |
| shared known IoU | 0.998845 |
| shared free IoU | 0.998581 |
| shared occupied IoU | 0.996707 |
| known-cell agreement | 0.999802 |
| occupied/free conflict rate | 0.000198 |

These are healthy replica/fusion results; consistency is not the same as static-world accuracy.

## Decentralized coordinator evidence

- Complete replicated decisions: **19/19 matched**, semantic agreement **100%** among complete replicas; zero mismatched decision hashes.
- One preliminary local pair-decision publication in Run 1 was superseded by a newer snapshot before the peer published the matching pair; it produced no duplicate dispatch and is reported as incomplete telemetry, not a disagreement.
- Canonical duplicate-assignment attempts: **0**; actual duplicate physical-task dispatches: **0**.
- Mean selected raw Nav2 path length: **3.527 m**; mean normalized cost: **0.196**; mean utility before reduction: **0.951**; mean reduction magnitude: **0.36**; LOS-blocked reduction evaluations: **99**; beta: **1.0**.
- Assignment/snapshot/dispatch latency is not persisted in the current event schema and is not fabricated in this report.

## Fresh terminal failure

One fresh failure occurred in `trial_01_attempt_01` on `robot2` at 217.88 s:

- task `494e1e23c98f2319ebc922ba`, round `457796e0162ec2d8477d21819a9989c71fbd11b41895f8c33c868a56cb8153f5`
- project class `UNKNOWN` / 8; duration 45.481 s; travelled 4.605 m
- raw action result: status **6 (ABORTED)**, accepted `True`, error code `0`, empty message
- recovery-count changes: 0

The supported cause is a Nav2 NavigateToPose ABORTED terminal result. No persisted BT child, planner/controller goal-correlated diagnostic, or Nav2 error message identifies a deeper cause. Zero recovery count does not prove that recovery was never entered; the old/current logger records recovery-count changes rather than every BT transition.

## Run-2 retrospective conclusion

The two original Run-2 failures are documented in `run2_terminal_failure_forensics.md`. Both were robot-1 Nav2 BT/action aborts with project class UNKNOWN, Nav2 error code 0/empty, and no recovery-count changes. The retained Run-2 data cannot distinguish planner, controller, progress-checker, TF, recovery, or map-change causes. The previous forensic harness failure was independently resolved: Webots exposed `ROBOT1`/`ROBOT2` while the source world used lowercase names, and case-sensitive Supervisor lookup aborted before capture.

## Traffic status

Traffic scheduling is explicitly **deferred/experimental** and disabled by default. No doorway world was created and no physical bottleneck validation was run. Pure geometry tests remain retained; they do not constitute Webots traffic evidence.

## Legacy comparison

The archived RPP campaign under `results/rpp_cooperative_validation_20260815/` provides a controller/mapping reference, but it used a different allocator, duration, and capture setup. Fresh mean normalized known-cell yield is 3207.8 cells per combined GT metre versus archived RPP values in the 3035.6–3417.5 range. This supports preserved system health, not a causal claim that Burgard is superior.

## Artifacts

- `fresh_burgard_forensic_metrics.json` — complete per-run metrics
- `cslam_map_accuracy.json`, `fusion_fidelity.json`, `shared_replica_consistency.json` — mapping evidence
- `nav2_benchmark.json`, `allocator_benchmark.json`, `exploration_throughput.json` — system metrics
- `fresh_terminal_failure_forensics.md/.json` — fresh failure breakdown
- `run2_terminal_failure_forensics.md/.json` — original Run-2 forensic breakdown
- `professor_demo_cheatsheet.md` — study guide
- `plots/` — only data-backed fresh-run plots

Unknown initial relative pose remains future work.
