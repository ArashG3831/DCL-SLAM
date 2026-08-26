# Unknown-pose handoff reliability report

Updated 2026-08-27. This report is limited to unknown-pose evidence
acquisition, peer verification, and recording/export. Frontier extraction is
the already-validated upstream `FrontierExplorerCore` adapter and is not
changed here. Map fusion, Nav2, and traffic are reported only at their
handoff boundary.

## Current verdict

`VALIDATION_INCOMPLETE — TERMINAL_EXPLORATION_NOT_REACHED`

The source-level handoff defects are fixed and three fresh dedicated trials
(19--21) independently accepted a peer-confirmed hypothesis in every trial.
A clean 600-second wall-time post-handoff soak completed with continued shared
allocation and navigation. The required 1,200-second campaign also completed
with clean teardown and stable host resources, but it produced no accepted
handoff and no terminal cooperative-exploration state. No acceptance
threshold was lowered; the overall completion claim therefore remains
blocked by terminal exploration.

## Provenance

All changes and tests were performed in
`/home/arash/webots_ws_clean_validation_20260823`. The original dirty checkout
`/home/arash/webots_ws` was not modified. Branch:
`validation/nat-gate-20260826`. Source HEAD after the recorder fix is
`94b103323bd226b3672cbd2bc2b1e1eab092829e` (the handoff-direction fix is
`574dcbd8ac434b78a295e421979bc6cb886b3680`). The isolated build/install used
for all post-fix runs is:

```text
build_unknown_pose_reliability_20260827
install_unknown_pose_reliability_20260827
```

The runner resolved `my_epuck_project` from that install prefix. The world
SHA-256 recorded in each run is
`2bf044aee250be4d2179da89b5f28a6b6cee04bd96f982b03b47b552d86cb84c`.
Runtime was NAT WSL, CycloneDDS `eth0`, subnet discovery, unique domains and
Webots ports, Fast/headless Webots, chunked best-effort raw input, reliable
reconstructed 720-beam scan output, `use_sim_time=true`, scan matching on,
loop closing off, and no rendering/RViz/motion fixture.

## Source changes

### Evidence acquisition and peer verification

* `src/my_epuck_project/my_epuck_project/unknown_pose_frontend.py`
  * `_geometry_rejected_in_active_batch()` (line 1548) scopes a rejected
    geometry key to its acquisition batch instead of suppressing that physical
    footprint forever. This preserves later evidence windows without changing
    geometry thresholds.
  * `_request_next_candidate_verification()` (line 1788) and
    `_verification_worker_busy()` (line 1869) keep a batch open while an
    in-flight worker drains. Budget exhaustion is not reported until queued
    work has completed.
  * `_request_batch_id()` (line 923) binds late worker results and exceptions
    to the batch that requested them, rather than the current batch.
  * `_try_confirm_pending_proposal()` (line 2281) now verifies the responder's
    local direction (target-local keyframe to the received source crop), then
    applies exactly one `invert_se2` to compare with the canonical proposal.
    The old code passed the opposite direction while treating it as reverse;
    this caused valid proposals to be rejected by the responder.

### Recording/export

* `cooperative_experiment_logger.py`: `enable_local_map_capture` defaults to
  true. The passive `ForensicEvidenceWriter` now records local maps, odometry,
  TF, and scan-correction evidence whenever the logger is enabled, without
  starting the optional Supervisor/ground-truth observer. Required-artifact
  validation requires Supervisor output only when that observer was explicitly
  requested.
* `cooperative_map_png_export.py`: direct fast-trial layouts are selectable;
  local final maps are resolved when no shared map exists; local odometry is
  projected through captured `robotN/map <- robotN/odom` TF; no-handoff exports
  write per-robot local PNGs, path overlays, and
  `NO_HANDOFF_SHARED_MAP_UNAVAILABLE.txt`. Shared maps are never synthesized.
* `scripts/validation_host_watch.py`: Windows counters are sampled through the
  absolute PowerShell path and include Pool Nonpaged Bytes, Available MBytes,
  and committed-memory percentage.

## Exact failure diagnosis

The failed pre-fix run 18 (`results/unknown_pose_reliability_trial18_fast_20260827`)
contained enough evidence for both peers to build compatible local clusters,
but the canonical proposal responder rejected it. The proposal sender's
direction was source robot to target robot. The responder's confirmation code
constructed `received_peer_crops[source]` with the local target keyframe but
classified the result as if it were already canonical reverse evidence. The
comparison therefore applied the wrong direction. The log shows proposal hash
`7150def...` rejected with `result_accepted=false` despite the sender's
accepted summary. This is a deterministic transform-direction bug, not weak
geometry and not peer disagreement.

The earlier trials 6--17 had a separate acquisition problem: a geometry key
rejected once remained suppressed across later batches, and the worker could
finish after the batch had been marked exhausted. Those behaviors caused
valid late evidence to be dropped or misattributed. The fixes preserve all
three-inlier, covariance, baseline, residual, temporal, confidence, and margin
gates.

## Successful versus failed evidence timeline

| Run | Result | Evidence | Interpretation |
|---|---|---|---|
| `fast_nat_handoff_delay20_407177b_retry` | both accepted | old build; hash `f8700d12d7ba2dce`; 3 inliers | historical functional baseline only |
| `unknown_pose_reliability_trial12_fast_20260827` | both accepted | old post-acquisition fix; hash `96dbdd23e64aa0b8`; 4 inliers; baseline 1.9200 m | proves protocol can succeed |
| trials 13--17 | no handoff | 0--2 mutually compatible constraints or insufficient spatial diversity | selector correctly withheld handoff |
| trial 18 | no handoff | local clusters reached 3--5 constraints; responder rejected canonical proposal due direction bug | source-level peer-verification defect |
| trial 19 | both accepted | hash `e818c2904ba1bdcb`; 3 inliers; baseline 1.8238 m; residual 0.01937 m; margin 0.05752 | fixed-direction pass |
| trial 20 | both accepted | hash `94587d4c2cd5d93f`; 5 inliers; baseline 2.2163 m; residual 0.02269 m; margin 0.06944 | fixed-direction pass |
| trial 21 | both accepted | hash `3550e7c7940febd3`; 3 inliers; baseline 1.7400 m; residual 0.02306 m; margin 0.05852 | fixed-direction pass |

In trials 19--21 both JSON summaries report `accepted=true`, identical
`evidence_set_hash`, identical margin/confidence and compatible residuals. Each
has an `robot1_accepted_handoff.marker` and
`robot2_accepted_handoff.marker`; launch logs show one handoff start per peer,
local pre-handoff teardown, and `POST_HANDOFF_SHARED`. No second canonical
handoff event occurs.

## Queue and rejection analysis

The pre-fix queue problem was not a DDS drop: the failed runs report zero
registration queue drops in the relevant trials. The measured loss was local
geometry suppression and batch-finalization ordering. The fixed diagnostics
retain batch IDs, candidate age, residual, inlier ratio, baseline, temporal
consistency, peer agreement, selector margin, and rejection reason. The
verification worker drain barrier ensures an in-flight request cannot be
silently counted against a completed batch.

Focused regression coverage includes geometry suppression expiry, late-result
batch attribution, worker-busy budget handling, queue overflow/backpressure,
stale and duplicate candidate rejection, temporal inconsistency, peer
disagreement, verification-budget exhaustion, three-inlier consensus, one
canonical handoff, and no second handoff.

The 20/40-second prehandoff delay is an observation hold, not an estimator
threshold. A 120-second hold kept robots static and failed baseline diversity;
40 seconds allowed both spatial diversity and sufficient overlap. The three
post-fix trials show that this runtime hold is effective, not that a threshold
was weakened.

## Recording/export validation

The first no-handoff recorder run exposed an artifact-contract error: local
maps existed but finalization incorrectly required a Supervisor CSV when
Supervisor capture was disabled. Commit `94b1033` separates optional
Supervisor capture from mandatory local evidence. The rebuilt no-handoff run
`results/no_handoff_recorder_trial_20260827_1787785482` has
`artifact_finalization.json` with `complete=true`, local map NPZs, odom CSVs,
TF CSV, scan-correction JSONL, and all bounded frontend diagnostics.

The exporter produced:

```text
robot1_local_map.png
robot1_local_map_with_paths.png
robot2_local_map.png
robot2_local_map_with_paths.png
final_merged_map.png (robot1 local map, explicitly marked no-handoff)
final_merged_map_with_paths.png (same explicit status)
NO_HANDOFF_SHARED_MAP_UNAVAILABLE.txt
export_manifest.json
```

The manifest contains `handoff_occurred=false`,
`status_label="NO_HANDOFF — SHARED MAP UNAVAILABLE"`, and never writes an
exact shared-map difference image. The exporter regression suite covers
occupancy colors, rotated origins, path overlays, direct fast-trial layouts,
and no-handoff local-map/status output. Accepted-handoff runs retain shared
map outputs and exact-difference artifacts only when both shared maps exist.

## Soak result

`results/unknown_pose_reliability_soak600_fast_retry_20260827` completed with
runner exit reason `mission_timeout`, wall duration 628.70 s, launch return 0,
and complete artifact finalization. The run includes both peer handoff starts,
`POST_HANDOFF_SHARED`, shared fusion/allocation, 44 accepted goals (18
successful), and continued frontier/allocator activity. It recorded 1,993.44
simulated seconds because Fast Webots advances simulation faster than wall
time. There were 13 NavFn warnings; they did not prevent handoff or shared
activation. The run ended with 22 remaining frontiers and is therefore not a
terminal-exploration pass.

Host watcher samples: 89. Pool Nonpaged Bytes ranged from 0.9992 to 1.0375 GiB
and ended at 1.0096 GiB; committed-memory percentage ranged 41.18--47.24%.
No hard resource stop occurred. Post-run process audit found no Webots, ROS,
frontend, logger, or campaign process and no selected-port residue.

## NavFn warning classification

The warning `Failed to create a plan from potential when a legal potential was
found` is emitted by NavFn/global planner attempts associated with individual
frontier goals. In the successful handoff trials it did not prevent evidence
collection, peer agreement, local-stack teardown, or shared activation. The
600-second soak still recorded 25 failed goals and 13 warning occurrences,
while `planner_failed_count` remained zero in its mission artifact. In the
1,200-second campaign there were 86 such warnings, 45 accepted goals, 42
successes, and 3 failed goals (including one controller timeout and one TF
failure). The warning stream is associated with individual frontier planning
attempts; it did not directly reject an evidence set. It is nevertheless an
independent Nav2 planner/goal-feasibility limitation and prevents a claim of
fully reliable navigation until separately resolved. Warnings were not
suppressed.

## Final 1,200-second campaign

`results/unknown_pose_reliability_final1200_fast_20260827` completed with
`exit_reason=mission_timeout`, wall runtime 1219.95 s, launch return code 0,
and `artifact_finalization.complete=true`. The run used the current isolated
install, Fast Webots, NAT CycloneDDS, domain 156, Webots port 25096, and the
same chunked-scan/reliable-reconstructed-720-beam configuration as the soak.

Both frontends exchanged descriptors and crops and independently accepted
geometric registrations, but no mutually compatible three-inlier hypothesis
was formed. Robot 1 accumulated 36 geometric constraints and Robot 2 22;
the final consensus attempts repeatedly had only two and one mutually
consistent constraints respectively. The high-similarity candidates were
spatially/transform-inconsistent (for example, Robot 1 accepted transforms
with y components 0.398, -0.638, and -2.042 m), so the unchanged selector
correctly returned `INSUFFICIENT_EVIDENCE`. The peers did not publish a
matching hypothesis summary, no canonical handoff occurred, and shared
fusion/allocation were never activated. This is evidence of insufficient
overlap/descriptor ambiguity after navigation, not evidence of a map-merge
failure and not a reason to weaken the gates.

The run recorded 548/552 descriptors published, 119/107 crop requests sent,
104/115 registration callbacks, 22/36 accepted geometric candidates, and
64 bounded verification batches per peer. The robots began local navigation
before a compatible evidence set was available; later candidate families
were dominated by repetitive corridor-like false matches. This explains why
the dedicated close-start trials passed while this longer naturally evolving
run did not.

Final host telemetry contained 174 samples: Pool Nonpaged Bytes
1.004--1.005 GiB (pre/peak/post approximately 1.004/1.005/1.001 GiB),
committed memory 41.28--46.71%, and no hard resource stop. Automatic cleanup
left no campaign process, Webots instance, or selected-port residue. These
resource results are a pass for stability, not a handoff or terminal-state
pass.

## Tests and build

The isolated colcon build of `my_epuck_project` and
`reliable_slam_toolbox_wrapper` succeeded after the recorder changes. The
combined focused run passed 124 tests (unknown-pose frontend, two-phase peer
verification, and map PNG exporter). The earlier full unknown-pose focused
set passed 166 tests after the direction and worker fixes. `git diff --check`
passed. Source/build/install are bound to the isolated overlay above.

## Remaining state

The handoff reliability gate is repeatably passed in the three dedicated
trials (3/3), and the 600-second post-handoff soak is complete and
resource-safe. The final campaign completed safely but failed the required
handoff and terminal-exploration gates. Therefore this task remains
`VALIDATION_INCOMPLETE — TERMINAL_EXPLORATION_NOT_REACHED`. Traffic
coordination is outside this task and remains unimplemented/untested.

Evidence paths:

* `results/unknown_pose_reliability_trial19_fast_20260827`
* `results/unknown_pose_reliability_trial20_fast_20260827`
* `results/unknown_pose_reliability_trial21_fast_20260827`
* `results/no_handoff_recorder_trial_20260827_1787785482`
* `results/unknown_pose_reliability_soak600_fast_retry_20260827`
* `results/unknown_pose_reliability_final1200_fast_20260827`
* `build_unknown_pose_reliability_20260827`
* `install_unknown_pose_reliability_20260827`
