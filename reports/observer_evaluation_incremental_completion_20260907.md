# Observer Evaluation Incremental Completion

Baseline commit: `db0f890` — observer evaluation completion specification.

Roadmap source: `OBSERVER_EVALUATION_COMPLETION_SPEC.md`.

Scope is frozen to completion of the offline scientific evaluation and report
layer. The live observer/evidence architecture is not being redesigned, raw
sampling is not being reduced, and robot/control/Nav2/SLAM/allocator behavior
is out of scope.

## Item 1 — Exact avoidable idle and snappiness timeline integration

Status: DONE

Legacy definition/source:

The existing `tools/forensic_exact_avoidable_idle.py` state machine remains the
semantic authority. It classifies intervals from candidate freshness, Nav2/TF
health, readiness, active goals, recovery state, and explicit block state. Its
thresholds, five-second freshness window, interval merge rule, strict
terminal-to-following-dispatch join, percentile rule, and fail-closed handling
were retained.

Current raw evidence:

`offline_protocol_replay.py` parses simulation/header timestamps from the raw
bag's candidate-generation, task-snapshot, bid, distributed-status,
distributed-event, navigation-terminal, and START_RELEASE messages. The
distributed status payload supplies `feasible_work_available`,
`actionable_work_available`, and `work_availability_reason`; these are exposed
as raw observations and are not inferred from dispatch absence.

Existing reusable implementation:

The exact-idle state machine was extracted into
`my_epuck_project.offline_timing_metrics.analyze_event_records`. The command-line
`forensic_exact_avoidable_idle.py` now delegates to that same implementation,
so the audit tool and evaluator cannot drift. `protocol_snappiness` builds the
required timestamp joins from the parsed protocol representation.

Gap identified:

The prior evaluator did not connect protocol replay to the exact-idle analyzer
and did not emit named snappiness/availability artifacts. No additional raw
topic was required for the fields present in the current status/task/event
payloads.

Implementation:

- added `offline_timing_metrics.py` with the shared legacy idle semantics;
- added candidate count, source snapshot epoch, task-generation timestamp,
  candidate-source health, navigation-active state, and protocol identity
  fields to the parsed representation where present in native messages;
- integrated exact idle, raw work-availability observations, and snappiness
  joins into `offline_evidence_replay.evaluate_run`;
- emitted `avoidable_idle.json` and `snappiness.json` beside the evaluator
  result, while retaining the same information in `thin_metrics.json`;
- added focused tests for interval semantics, fail-closed health handling,
  candidate/task, task/bid, agreement/goal, dispatch/terminal, and
  handoff/first-goal joins.

Files changed:

- `src/my_epuck_project/my_epuck_project/offline_timing_metrics.py`
- `src/my_epuck_project/my_epuck_project/offline_protocol_replay.py`
- `src/my_epuck_project/my_epuck_project/offline_evidence_replay.py`
- `src/my_epuck_project/tools/forensic_exact_avoidable_idle.py`
- `src/my_epuck_project/test/test_offline_timing_metrics.py`
- `OBSERVER_EVALUATION_COMPLETION_SPEC.md`

Tests:

- focused pure-Python timing/metric tests: **31 passed** including the
  existing experiment-metric tests;
- Python syntax compilation for all changed Python modules: **PASS**;
- ROS-dependent protocol and legacy logger tests were attempted but could not
  collect because this shell has no `rclpy` installation. This is an external
  test-environment limitation, not a failed semantic assertion.

Validation:

No Webots run was needed. The item is pure offline replay and was validated on
synthetic deterministic records. No live observer or raw-capture process was
changed.

Parity/semantic result:

The extracted classifier preserves the existing legacy state-machine behavior,
including its existing adjacent-feasible interval merge semantics. Missing
health/candidate evidence remains `WORK_UNAVAILABLE`; it is not converted to
zero feasible work. Join values use simulation/header time and are unavailable
when the required identity/timestamp pair is absent. The output is ready for
comparison against preserved run artifacts; a full rosbag replay requires the
ROS Python runtime in the validation environment.

Commit:

`4ab6da3` — `observer: complete offline idle and snappiness replay`.

Remaining caveat:

The preserved raw protocol message set does not guarantee a certificate payload
for every run. Certificate-specific completeness remains item 2 and is not
silently claimed by item 1. The existing legacy idle merge semantics may need a
separate scientific-definition decision if future parity evidence demonstrates
that goal-active transitions must split adjacent feasible intervals; no such
definition change is made here.

## Item 2 — Cooperation, assignment, agreement, certificate, and DNU summaries

Status: DONE

Legacy definition/source:

The legacy observer's distributed message callbacks and final coordination
summary define candidate/task batches, bids, pair decisions, agreement and
continuation events, dispatch/terminal accounting, failure classes, and
cost-only certificate fields. Native raw message payloads are authoritative;
no decision is inferred from a missing message.

Current raw evidence:

`offline_protocol_replay.replay_protocol` reads the current FrontierCandidateArray,
TaskSnapshot, TaskBidArray, PairDecision, DistributedExplorationStatus,
DistributedExplorationEvent, ExplorationFailure, and START_RELEASE streams.
It preserves source robot, simulation/header time, generation/round/session
identity, payload values, and the complete certificate JSON payload when the
certificate event exists.

Existing reusable implementation:

The existing parsed record lists remain the single payload representation.
`protocol_summary` is a compact reduction over those records; it does not
deserialize or store a second copy of full messages. Existing `_state_name`,
`_failure_name`, `_bound_entry_count`, and dispatch/terminal event streams are
reused.

Gap identified:

The prior evaluator exposed raw lists and small robot counters but no final
cooperation summary, per-round reduction, certificate field-completeness
classification, or DNU time-series artifact.

Implementation:

- added `protocol_summary` with candidate/task/bid/pair/agreement/continuation /
  claim/assignment/terminal/failure/certificate/DNU and per-round summaries;
- retained explicit zero counts only where the corresponding raw stream was
  read, and retained `NOT_INVOKED`/`NO_EVIDENCE` distinctions for certificates;
- made certificate payload completeness require the actual audited fields:
  evaluated count, DNU count, blocking count, selected score, optimistic
  score, dispatch certification, and reason;
- integrated `cooperation_summary` into `thin_metrics.json` and emitted the
  compact `cooperation_summary.json` sidecar;
- added focused summary and fail-closed certificate tests.

Files changed:

- `src/my_epuck_project/my_epuck_project/offline_protocol_replay.py`
- `src/my_epuck_project/my_epuck_project/offline_evidence_replay.py`
- `src/my_epuck_project/test/test_offline_protocol_payload.py`
- `src/my_epuck_project/test/test_offline_protocol_summary.py`
- `OBSERVER_EVALUATION_COMPLETION_SPEC.md`

Tests:

- focused offline timing/protocol/metric tests: **33 passed**;
- Python syntax compilation for changed offline modules: **PASS**;
- ROS-dependent native message fixture execution remains unavailable in this
  shell because `rclpy` is not installed; the existing test collection error
  is recorded, not reclassified as a protocol result.

Validation:

No Webots run was needed. Synthetic parsed payloads proved compact summary
counts, per-round identity handling, valid zero pair-decision semantics, and
fail-closed incomplete certificate payload handling.

Parity/semantic result:

The raw parsed records remain available for exact comparison, while summaries
are derived only from those records. Certificate absence is never synthesized
as a certified zero. The final evaluator now exposes the required item-2
summary surfaces; any run lacking a required certificate payload remains
explicitly incomplete under the existing contract.

Commit:

`74e3912` — `observer: complete offline cooperation and certificate summaries`.

The post-commit focused replay check also found and corrected a local reducer
aggregation defect before this checkpoint was finalized; the amended checkpoint
hash is recorded in the spec and git history.

This checkpoint contains only the item-2 evaluator/replay integration, its
focused tests, and the corresponding roadmap/progress status.

Remaining caveat:

The raw source cannot prove a certificate field that was not published by the
allocator. Such fields remain `OBSERVED_INCOMPLETE`/blocked rather than being
reconstructed from bids, pair decisions, or dispatches.

## Item 3 — Motion anomaly replay

Status: DONE

Legacy definition/source:

`experiment_metrics.MotionDetector` is the existing authority for windowed,
edge-triggered `NO_PROGRESS`, `STUCK`, and `OSCILLATION` detection. The live
observer feeds it from each robot's telemetry row, with a near-goal exclusion.

Current raw evidence:

The legacy telemetry streams `robot*_timeseries.csv` retain the same simulation
time, pose, command, navigation-active, and distance-remaining fields supplied
to the detector. A valid empty stream is distinct from an absent or malformed
stream.

Existing reusable implementation:

`offline_motion_metrics.replay_timeseries` calls the existing
`MotionDetector.update` directly. It does not copy the thresholds or implement
a parallel detector. It reconstructs edge events and pairs STARTED/CLEARED
events into open or closed episodes.

Gap identified:

The evaluator had no deferred anomaly artifact; anomaly counts were previously
available only in the live summary/events output.

Implementation:

Added `offline_motion_metrics.py`, integrated `motion_anomalies` into
`thin_metrics.json`, and emitted `motion_anomalies.json`. Missing and valid
zero-event streams remain explicitly different.

Files changed:

- `src/my_epuck_project/my_epuck_project/offline_motion_metrics.py`
- `src/my_epuck_project/my_epuck_project/offline_evidence_replay.py`
- `src/my_epuck_project/test/test_offline_motion_metrics.py`
- `OBSERVER_EVALUATION_COMPLETION_SPEC.md`

Tests:

- focused offline motion/timing/protocol/metric tests: **35 passed**;
- Python syntax compilation: **PASS**.

Validation:

Synthetic telemetry proved parity of detector thresholds and edge transitions,
including clearing an episode and an empty-valid stream.

Parity/semantic result:

The same `MotionDetector` class remains the sole anomaly algorithm. The replay
uses the legacy row order and simulation-time values, preserving threshold,
window, near-goal, and edge-trigger semantics.

Commit:

Pending source checkpoint.

Remaining caveat:

Runs without telemetry CSVs are explicitly unavailable for this metric; no
anomaly is inferred from the absence of a stream.
