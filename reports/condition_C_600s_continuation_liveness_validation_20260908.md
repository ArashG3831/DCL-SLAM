# Condition-C 600 s continuation-liveness validation

Date: 2026-09-08

## Scope

One canonical Condition-C run was executed after commit `09619c7`
(`fix allocator continuation round liveness`). The change was limited to
preventing the normal-round replacement gate from replacing an active
continuation round; continuation identity, session, commitment, safety, and
certificate validation rules were retained.

No scoring, candidate limits, certificate rules, traffic safety, Nav2, SLAM,
or experiment parameters were changed for this validation.

## Validation inputs

- Focused round-lifecycle tests: `62 passed in 0.51s`.
- Canonical launcher dry-run: `CANONICAL_THESIS_PREFLIGHT_PASS`.
- Canonical command:

  ```text
  python3 src/my_epuck_project/tools/run_thesis_experiment.py --condition C --horizon 600
  ```

- Commit at launch: `09619c7656da61eb084120f86a6addbefe0f9f6a`.
- Result:
  `results/thesis_condition_C_600s_20260908T141244Z/fast_trial_20260908T141249Z/`
- Observer artifact:
  `results/thesis_condition_C_600s_20260908T141244Z/fast_trial_20260908T141249Z/observer/fast_trial_20260908T141249Z-01/`
- Offline evaluator: existing `offline_evidence_replay.evaluate_run()` in a
  clean ROS Jazzy/project-install environment; wall time `49.13 s`.

## Pipeline result

| Gate | Result | Evidence |
|---|---|---|
| Startup | PASS | Both robot stacks, maps, protocol, and navigation became active. |
| 600 s horizon | PASS | `/clock` reached `600.66 s`. |
| Shutdown/finalization | PASS | `artifact_finalization.json: complete=true`; `summary.json` and `mission_result.json` exist. |
| Native evidence | PASS | Native bag export, GT/contact, maps, protocol, warnings, and replay parity artifacts exist. |
| Shared-map continuity | PASS | robot1 shared-map series ends at `600.08 s`; robot2 at `598.76 s`. |
| Offline evaluator | PASS | `thin_metrics.json: complete=true`; all current sidecars were emitted. |

The mission itself is correctly marked `INCOMPLETE` because the imposed horizon
ended with one robot goal still active. This is a horizon accounting result,
not a shutdown or allocator-liveness failure.

## Allocator liveness result

| Measure | Result |
|---|---:|
| Dispatches | 23 |
| Successful navigation terminals | 22 |
| Failed goals | 0 in terminal accounting; 8 diagnostic failure records were retained for rejected/intermediate path/lifecycle cases |
| Active goal at horizon | 1 |
| Unique agreed rounds | 18 |
| Agreement publications | 38 |
| Certificate observations | 43 |
| Certified observations | 42 |
| Certificate observations with `UNQUERIED_OPTION_CAN_BEAT_EVALUATED_ASSIGNMENT` | 0 |
| Certificate observations with `MISSING_UNQUERIED_LOWER_BOUNDS` | 1 |
| Detected unqueried candidates | 0 |
| Continuation rounds started | 49 |
| Round invalidation events | 9 |
| Allocator round replacements | 32 |
| Last allocator health: robot1 | 40 created, 18 replaced, 12 dispatches |
| Last allocator health: robot2 | 35 created, 14 replaced, 11 dispatches |

Compared with the pre-fix liveness failure (3 goals and hundreds of
replacements), both robots continued to receive and complete new dispatches
throughout the horizon. The continuation path reached agreement and dispatch;
it did not remain trapped in the prior self-replacement loop.

## Scientific outputs

| Metric | Result |
|---|---:|
| Final known cells | 203,007 |
| Final known area | 182.7063 m² |
| World coverage | 45.68% of the 400 m² configured extent |
| Known-area AUC | 42,958.5409 m²·s |
| Time to 25% world extent | 474.10 s |
| Time to 50/75/90% | Not reached |
| First-seen cells | robot1 117,428; robot2 86,085 |
| Later duplicate cells | robot1 22,772; robot2 33,165 |
| Duplicated-known fraction | 0.2749 |
| Simultaneously observed cells | 6,238 |
| Odom distance | robot1 33.9743 m; robot2 30.3650 m |
| Shared-frame trajectory overlap IoU | 0.002714 |
| Avoidable idle | robot1 9.90 s / 4.35%; robot2 41.36 s / 16.16% |
| Productive engagement | robot1 95.65%; robot2 83.84% |
| Longest avoidable idle | robot1 9.90 s; robot2 19.56 s |
| Terminal → next dispatch p95 | 23.32 s, 21 samples |
| Agreement → NAV_GOAL_SENT p95 | 0.56 s, 32 samples |
| Handoff → first shared goal | 0.74 s |
| Jain first-seen-cell fairness | 0.97683 |
| Handoff translation error | 0.005219 m |
| Handoff yaw error | 0.006400 rad / 0.3667° |
| Motion anomalies | robot1 one 14.0 s stuck episode; robot2 two short stuck episodes |
| Warnings | available; 52 normalized warning records |
| Map quality | available; approved cooperative analyzer output present |

Certificate payloads were complete. The one lower-bound observation did not
prevent the subsequent valid dispatch sequence; no unqueried-option
certificate starvation was observed.

## Performance note

The run recorded `600.66 s` simulated time and `532.04 s` total runner elapsed
time, which includes lifecycle/finalization. Using the observer event boundary
from START_RELEASE (`45.96 s`) to RUN_END (`600.66 s`), the approximate active
execution RTF was `1.49×`; the first `[0,180)` active segment was approximately
`2.07×`, with later segments approximately `1.32×` and `1.40×`. This was a
liveness validation, not a performance optimization run; no performance change
was attempted.

## Remaining caveats

- Mission completion was not expected at the imposed 600 s horizon and one
  goal remained active.
- Robot2 exceeded the preferred avoidable-idle target, and robot1's longest
  interval exceeded 5 s; these are measured scientific results, not pipeline
  failures.
- The unrelated shutdown-time `teammate_scan_filter` exit code 1 occurred while
  ROS was already shutting down; it did not prevent complete finalization or
  offline evaluation.

## Verdict

`CONDITION_C_600S_CONTINUATION_LIVENESS_PASS`
