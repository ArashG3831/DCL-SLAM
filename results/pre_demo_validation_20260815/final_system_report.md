# Pre-demo decentralized allocator validation

## Production state

The ordinary launch defaults are `assignment_strategy=burgard`, `beta=1.0`, and `traffic_scheduler_enabled=true`. `legacy_weighted` remains an explicit diagnostic launch option. RPP, SLAM, fusion, NavFn, costmaps, velocity smoother, Collision Monitor, and known-transform map alignment remain unchanged.

The active production assignment is:

```text
U_t = 1
C_i,t = clamp(L_i,t / 18 m, 0, 1)
argmax(i,t) [ U_t - 1.0*C_i,t ]
```

The existing 18 m value is the authoritative feasibility ceiling. The division creates a project-specific dimensionless cost; it is not claimed as a Burgard normalization. After each selection, `P(d)=1-d/11.98` for clear LOS within the actual D500 range, otherwise zero. Canonical task IDs are removed after selection, so duplicate physical assignment is impossible within a round.

## Traffic

Selected bid polylines are checked using continuous segment geometry. The configured safety radius is 0.08 m per robot, derived from the production Collision Monitor stop circle and larger than the 0.055 m Nav2 footprint radius. A conflict is a predicted centerline separation at or below 0.16 m. New conflicting goals are ordered by active status, first-conflict ETA at 0.13 m/s, and robot ID only for a near-equal tie. The losing new goal is held before NavigateToPose and is never blindly resumed after release.

## Evidence inventory

Measured final-run directories discovered: **3**. New runtime benchmark complete (at least 240 s per measured run): **True**. Decision events discovered: **34**.

The deterministic traffic suite has 3 repetitions for one-task/IDLE, shared-doorway, and separated-wide-route cases: **True**. Because the repository has no dedicated doorway Webots world and its existing small world is open-room smoke geometry, these are explicitly labeled pure scheduler tests, not physical doorway evidence.

Archived RPP cooperative artifacts remain under `results/rpp_cooperative_validation_20260815/`. They are not relabeled as Burgard results. A fair throughput comparison requires matched Burgard missions with the same world, duration, and observer configuration.

## Measured production sample

| Run | Simulated seconds | Agreed rounds | Goals accepted | Successes | Failures | Replica agreement | Final known cells |
|---|---:|---:|---:|---:|---:|---:|---:|
| pre_demo_burgard_large_01_final | 336.5 | 6 | 12 | 11 | 0 | 1.0 | 118502 |
| pre_demo_burgard_large_02_final | 296.0 | 5 | 10 | 7 | 2 | 1.0 | 97312 |
| pre_demo_burgard_large_03_final | 315.2 | 6 | 11 | 9 | 0 | 1.0 | 96874 |

Across these runs, the observer recorded zero canonical duplicate-assignment
attempts and zero runtime traffic events. The traffic implementation is covered
by the pure three-repetition geometry suite, but no doorway-specific Webots
world was available for a physical bottleneck run. Run 2's two navigation
failures are retained in the sample and are not treated as allocator crashes.

Map-accuracy-against-static-world, fused-source-union IoU, and ground-truth
trajectory RMSE are not fabricated for these runs because forensic ground-truth
capture was disabled after a diagnostic-only startup failure. The archived RPP
campaign contains those metrics and is referenced separately; the new observer
does provide known-cell growth, map replica state, navigation, motion, and
coordination metrics.

## Acceptance interpretation

Build and focused deterministic tests pass. A full measured production benchmark is only claimable when the saved run directories contain complete observer summaries; absent or startup-only runs are reported as invalid rather than converted into zero-performance results.
