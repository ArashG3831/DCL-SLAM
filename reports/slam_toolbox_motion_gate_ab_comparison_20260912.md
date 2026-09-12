# Slam Toolbox 2.8.5 motion-gate A/B comparison

## Verdict

Both controlled 180 s runs completed and shut down cleanly with the isolated
Slam Toolbox 2.8.5 overlay. The retained artifacts do **not** contain per-scan
`shouldProcessScan()` reasons or Karto-node creation events, so they cannot
directly prove the rotation-only admission question. At the observable output
boundary, B did not produce a material aggregate increase in corrected scans:
292 in A versus 293 in B. The focused motion-gate effect is therefore
**inconclusive from these artifacts**, not a demonstrated pass or failure.

## Provenance and controls

- Source workspace: `/home/arash/webots_peer_lifecycle_validation_20260912`
- Project source commit recorded by the runner: `30033b91c55e74b8d0c82c63a27c9d25c168d4ef`
- Slam Toolbox source: upstream tag `2.8.5`, commit
  `ec8f7635dea317b531c419f798f87d90a336f32e`
- Candidate Slam prefix: `/tmp/slam_toolbox_2.8.5_compat_jK5azQ/install_slam_toolbox/slam_toolbox`
- Candidate wrapper prefix: `/tmp/slam_toolbox_2.8.5_compat_jK5azQ/install_wrapper_clean/reliable_slam_toolbox_wrapper`
- `ldd` verification resolved the Slam Toolbox/Karto libraries from the
  candidate prefix; `/opt/ros/jazzy` supplied non-Slam ROS dependencies only.
- Same world profile, Condition C, seed `1001`, 180 s horizon, and other
  launch parameters were used in both runs. `restamp_tf=false` in both.
- A: `check_min_dist_and_heading_precisely=false`.
- B: `check_min_dist_and_heading_precisely=true`.
- Exactly one A run and one B run were executed; neither was rerun.

## Path-cost lifecycle

| Run | Robot | Requests | Result callbacks | Reachable | Unreachable | Pending at cutoff | Stale records |
|---|---:|---:|---:|---:|---:|---:|---:|
| A | R1 | 59 | 59 | 56 | 3 | 0 | 0 |
| A | R2 | 48 | 47 | 47 | 0 | 1 | 0 |
| B | R1 | 51 | 51 | 50 | 1 | 0 | 0 |
| B | R2 | 27 | 27 | 27 | 0 | 0 | 0 |

All recorded callbacks completed through the existing lifecycle without
`LOCAL_CONTEXT_CHANGED` or other stale rejection records. No stale-only
request segment was recorded. The request lifecycle emitted no
`MINIMUM_TRAVEL`, `shouldProcessScan`, or Karto diagnostic lines, so the true
callback scan count, MINIMUM_TRAVEL rejection count, and Karto-node count are
**not recorded** by these uninstrumented runs.

The corrected-scan stream is the available downstream acceptance proxy:

| Run | R1 corrected scans | R2 corrected scans | Total | R1 map messages | R2 map messages |
|---|---:|---:|---:|---:|---:|
| A | 147 | 145 | 292 | 69 | 70 |
| B | 145 | 148 | 293 | 69 | 69 |

Observed map-publication rates were approximately 0.398/0.397 Hz in A and
0.400/0.400 Hz in B (R1/R2). These are topic-observation rates, not internal
`updateMap()` duration measurements.

## DNU / frontier observations

The artifact reports DNU per candidate-batch observation, not keyed directly
to allocator allocation epochs. The available bounds were:

| Run | R1 batches / max DNU | R2 batches / max DNU | Combined max DNU |
|---|---:|---:|---:|
| A | 27 / 14 | 28 / 15 | 29 |
| B | 31 / 13 | 25 / 14 | 27 |

Candidate batches were A: R1 27, R2 28; B: R1 31, R2 25. Frontier region
observations contained no 2–3-cell regions in either run. The 4–10-cell share
was A 4.67% combined and B 2.08% combined.

## Navigation and mission result

Allocator dispatch logs recorded 9 dispatches in each run. A had 8 terminal
successes, no algorithmic failures/cancellations, and 1 goal active at the
horizon. B had 7 terminal successes, no algorithmic failures/cancellations,
and 2 goals active at the horizon. Native Nav2 status showed the A active goal
being aborted during horizon teardown; this was shutdown behavior, not an
allocator failure. Both runs reached the 180 s simulation cutoff with clean
artifact finalization and runner exit 0; neither naturally terminated before
the horizon.

Mapping/frontier outputs:

- A: 52,892 known-cell gain; final known cells 55,934; 55 candidate batches;
  0.607 duplicated-known fraction.
- B: 51,849 known-cell gain; final known cells 54,891; 56 candidate batches;
  0.852 duplicated-known fraction.

The A/B paths were not identical after startup, so these coverage differences
are not attributable solely to the motion-gate parameter.

## Performance

- Simulation RTF: A `1.999`, B `2.011` simulation seconds per wall second.
- The host watcher sampled the candidate Slam processes sparsely: observed
  `%CPU` ranges were approximately A 23–44% and B 19–55% per process.
- Exact Slam-process RSS was not captured; the watcher recorded `%MEM` only
  (approximately A 0.5–1.1%, B 0.5–1.0%). Observer-process peak RSS was
  122.7 MB (A) and 120.2 MB (B), not Slam Toolbox RSS.

## Final answer to the motion-gate question

The artifacts show that both configurations ran with the intended 2.8.5
parameter values and that the surrounding scan/map/query pipelines remained
healthy. They do **not** show whether rotation-only scans were rejected by
`MINIMUM_TRAVEL`, nor whether a Karto node was created for each such scan.
Therefore the question “does B admit rotation-only scans without translation?”
is **not directly answered** by this A/B run. A dedicated low-overhead internal
Slam Toolbox instrumentation run (or an exposed per-scan gate/node counter) is
required before claiming that result.
