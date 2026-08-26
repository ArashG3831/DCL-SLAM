# Unknown-pose handoff reliability report (2026-08-27 update)

## Verdict

`VALIDATION_INCOMPLETE — ROBUST_HANDOFF_NOT_PROVEN`.

The unknown-pose protocol is functional but not yet repeatable under the
current fast NAT runtime. One post-fix trial reached a valid peer-confirmed
handoff; four matched post-fix trials did not. No estimator gate was lowered.
The 600-second soak and 1,200-second campaign remain blocked until three fresh
dedicated handoffs pass.

## Scope and provenance

All work was performed in `/home/arash/webots_ws_clean_validation_20260823`;
`/home/arash/webots_ws` was not modified. Branch:
`validation/nat-gate-20260826`. Starting source was `69bafe3`; the current
uncommitted source includes the changes described below. The rebuilt isolated
overlay is `build_unknown_pose_reliability_20260827` /
`install_unknown_pose_reliability_20260827`; runtime package resolution and
launch logs point to that install.

Runtime configuration for trials 6–14 was fast, headless Webots, NAT
CycloneDDS `eth0` profile, subnet discovery, chunked best-effort raw input,
reliable reconstructed 720-beam output, `use_sim_time=true`, scan matching
enabled, and loop closing disabled. Each run used a unique ROS domain and
Webots port and ended with the runner's automatic process-tree cleanup.

## Evidence comparison

Historical artifacts:

* `results/fast_nat_handoff_delay20_407177b_retry` succeeded, but used source
  `407177b` with the older build/install overlay.
* `results/fast_nat_final_delay20_407177b` and
  `results/fast_nat_handoff_batches200_407177b` failed without handoff and
  carried stale-build provenance relative to their reported source.

Current matched trials:

| Trial | Delay | Peer result | Evidence | Acquisition result |
|---|---:|---|---|---|
| `trial6_fast` | 20 s | both rejected | 3 accepted geometric constraints, inconsistent | 0 queue drops; 42 temporal rejections/peer |
| `trial7_fast` | 20 s | both rejected | 5–6 constraints, dominant two-inlier cluster | 0 queue drops; 44–50 novelty/temporal deferrals |
| `trial8_fast` | 120 s | both rejected | 0–1 accepted constraints | robots remained too static for spatial diversity |
| `trial9_fast` | 40 s | both rejected | two compatible plus aliases | 0 queue drops; selector correctly rejected aliases |
| `trial10_fast` | 20 s | both rejected | 9–10 accepted constraints, dominant two-inlier cluster | geometry suppression greatly reduced |
| `trial11_fast` | 60 s | both rejected | 6–9 accepted constraints | no three-inlier compatible model |
| `trial12_fast` | 40 s | **both accepted** | 4 compatible inliers; hash `96dbdd23e64aa0b8`; baseline 1.9200 m; confidence ~0.870 | shared activation occurred |
| `trial13_fast` | 40 s | both rejected | 0 accepted geometric constraints | descriptor candidates were aliases/poor geometry |
| `trial14_fast` | 40 s | both rejected | robot1 3, robot2 1 accepted constraints | no common three-inlier set |

Trial 12 is a valid functional handoff, not a repeatability proof. Both peers
reported the same evidence hash and accepted four constraints; no second
handoff was observed. The differing hashes across trials are expected because
keyframe IDs differ.

## Root cause and source fix

The source-level acquisition defect was permanent suppression of a
revision-independent crop geometry after one geometric rejection. The same
physical footprint was then excluded from every later verification batch, even
when a later map/keyframe revision could have produced a valid registration.
This was visible in the failed artifacts as thousands of
`physical_geometry_rejections_suppressed` events and exhausted batches despite
zero registration-queue drops.

`unknown_pose_frontend.py` now keeps the rejection set bounded by acquisition
batch: a rejected geometry is suppressed for the current batch only and may be
retried after batch rollover. Exact physical evidence and accepted-evidence
deduplication remain authoritative; no threshold or acceptance gate changed.
The helper `_geometry_rejected_in_active_batch()` implements the rule, and a
regression test covers expiry on batch rollover.

The fast runner also now explicitly starts local Nav2 (`nav2_autostart:=true`),
recognizes already-active lifecycle nodes without issuing a redundant STARTUP,
and exposes `--prehandoff-dispatch-delay-s`. These changes address observed
clock/controller startup behavior and make the overlap hold an explicit
one-variable experiment; they do not alter estimator gates.

## Queue, rejection, and selector analysis

Across trials 6–14, registration queue drops were zero. The worker was receiving
responses; failures arose before consensus because accepted single-crop
registrations formed incompatible transform clusters or no accepted geometry.
Examples from consensus diagnostics:

* trial 6: transforms near `(-2.74, 0.18, 0.028)`,
  `(-2.78, -0.006, 0.008)`, and `(-2.84, 0.006, -0.014)`; only one mutually
  compatible under the unchanged 0.15 m / 1° pairwise gate;
* trial 9: one valid `(+2.78, -0.049, -0.004)` constraint and a false
  `(+3.20, +3.77, 1.57)` alias;
* trial 10: the accumulator retained more evidence, but the best model still
  had only two mutually compatible constraints;
* trial 12: four constraints converged near the accepted `-2.76 m` transform.

Therefore the selector is correctly rejecting insufficient or aliased sets;
the failure is evidence quality/overlap timing, not peer disagreement or
silent worker loss. The 120-second hold prevented motion and therefore failed
the spatial-baseline requirement. The 20–60 second holds permit motion, but
the current fast world can move beyond useful common observation before a
third compatible view is acquired.

## Focused tests and build

The focused suites after the changes passed **113 tests**:

```text
PYTHONPATH=src/my_epuck_project:$PYTHONPATH \
python3 -m pytest -q \
  src/my_epuck_project/test/test_unknown_pose_frontend.py \
  src/my_epuck_project/test/test_unknown_pose_accuracy_upgrade.py \
  src/my_epuck_project/test/test_cooperative_trial_fast.py
113 passed
```

The isolated colcon build of `my_epuck_project` and
`reliable_slam_toolbox_wrapper` succeeded, and `git diff --check` passed.
The test coverage includes incremental hypothesis behavior, historical bad
evidence rejection, compatible three-inlier acceptance, queue/backpressure,
peer verification, canonical handoff suppression, lifecycle readiness, and
the new batch-scoped geometry rejection rule.

## NavFn warnings

The repeated `Failed to create a plan from potential when a legal potential
was found` messages are associated with frontier path-validation requests and
controller/costmap failures for individual goals. They do not appear in the
successful handoff's accepted evidence path and did not produce a DDS or
process failure. They remain a separate Nav2/planner limitation: their exact
impact on long-run exploration is not yet fully classified, so full Nav2
reliability is not claimed.

## Artifact capture

Trials 6–14 used bounded frontend diagnostics but disabled forensic capture, so
they prove handoff behavior and cleanup, not PNG/map export behavior. The
no-handoff local-map/path exporter must still be validated against a dedicated
no-handoff artifact and a successful-handoff artifact before the final report
can claim complete recorder coverage. No shared map is inferred when handoff
was absent.

## Cleanup and runtime status

Every completed trial recorded a graceful launch shutdown and no remaining
campaign-owned process in the runner audit. A few Webots driver PIDs were
observed by the runner during teardown and were terminated by its
campaign-driver cleanup path; this is recorded in each trial summary. No
second handoff, DDS assertion, SIGABRT, or SIGSEGV occurred in these trials.

## Next required work

1. Run at least two more fresh dedicated trials using the batch-scoped fix and
   the 40-second overlap hold; require both peers to accept the same hash and
   three or more compatible inliers in each.
2. Validate local-map/path PNG export for both no-handoff and accepted-handoff
   artifacts.
3. Classify/fix remaining NavFn planner failures if they affect navigation.
4. Only after repeatable handoff and artifact capture pass, run the 600-second
   post-handoff soak, then the 1,200-second terminal campaign.

Traffic coordination remains outside this reliability task and was not run.
