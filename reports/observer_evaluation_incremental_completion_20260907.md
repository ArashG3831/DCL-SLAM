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

Pending until the item-1 source/docs checkpoint is created; the final hash is
recorded in the follow-up checkpoint below.

Remaining caveat:

The preserved raw protocol message set does not guarantee a certificate payload
for every run. Certificate-specific completeness remains item 2 and is not
silently claimed by item 1. The existing legacy idle merge semantics may need a
separate scientific-definition decision if future parity evidence demonstrates
that goal-active transitions must split adjacent feasible intervals; no such
definition change is made here.
