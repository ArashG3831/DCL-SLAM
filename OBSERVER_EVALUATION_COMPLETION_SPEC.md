# Observer Evaluation Completion Specification

## 1. Purpose and scope

This specification is the authoritative definition of completion for the
observer and offline-evaluation pipeline. The project uses three deliberately
separated layers:

1. live raw-evidence capture;
2. deferred/offline analysis and reconstruction;
3. final thesis-metric reporting.

The salvaged legacy observer and its deferred paths remain the current
behavioral reference. The thin/raw offline evaluator is being completed so it
reproduces the required scientific outputs from preserved evidence without
changing robot, navigation, mapping, cooperation, or sensing behavior.

This document defines the required outputs, evidence obligations, semantic
rules, implementation order, and completion criteria. It does not authorize a
new observer architecture or any change to production robotics behavior.

## 2. Frozen rules

- Do not reduce lidar, SLAM, mapping, ground-truth, contact, evaluator, or raw
  evidence fidelity.
- Do not run A/B/D campaigns until evaluation completeness is proven.
- Do not run a 1200-second campaign until the 180-second pipeline is complete
  and validated.
- Every implementation step follows: **inspect → implement → test → bounded
  validation → commit**.
- Missing evidence fails closed. It must never be converted silently into zero.
- A valid zero-event result is distinct from an absent, unreadable, malformed,
  or incomplete evidence stream.
- The salvaged legacy path remains the behavioral and semantic reference until
  parity or an explicitly approved replacement definition is recorded.
- Offline processing may derive metrics, but it may not alter recorded robot
  behavior or retroactively reinterpret unresolved evidence as success.

## 3. Complete required metric checklist

The final pipeline must provide the following metric families, with explicit
raw-source and completeness requirements.

### A. Exploration and coverage

- known area over time;
- final known area;
- coverage percentage;
- coverage AUC;
- 25%, 50%, 75%, and 90% milestones;
- duplicate explored cells;
- simultaneous observation;
- ownership / first-seen contribution;
- coverage gained per metre.

### B. Motion and efficiency

- per-robot distance travelled;
- trajectory overlap;
- repeated/revisited distance;
- active time;
- idle time;
- exact avoidable idle;
- productive engagement;
- longest avoidable idle interval.

### C. Coordination

- candidate generation;
- assignment decisions;
- bids;
- agreements;
- continuation decisions;
- certificate, deferred, and certified rounds;
- DNU statistics;
- goal lifecycle;
- failures/recoveries;
- terminal accounting.

### D. Snappiness and timing

- terminal → next-dispatch latency;
- candidate-generation latency;
- bid latency;
- agreement → goal latency;
- handoff → first shared-goal latency;
- planner/action latency;
- timestamp joins, including their source, ordering, and time domain.

### E. Reliability

- stalls;
- no progress;
- oscillation;
- stale topics;
- warnings;
- Nav2 outcomes.

### F. Physical validation

- ground-truth trajectory;
- contact semantics;
- handoff transform residual;
- translation and yaw error;
- handoff timing;
- reciprocal verification when the required evidence exists.

### G. SLAM and map quality

- map-quality metrics;
- localization/reference comparisons;
- physical ground-truth joins.

### H. Performance

- authoritative RTF definition;
- wall-time usage;
- resource attribution required by the thesis.

Observer-internal CPU/RSS history reproduction is **NOT REQUIRED** unless a
future explicit thesis or acceptance decision changes that requirement. Any
resource metric that is retained must state its owner, sampling basis, and
scientific purpose.

### I. Fairness and scaling

- workload contribution;
- an explicit fairness metric definition;
- scaling windows:
  - 0–180 seconds;
  - 180–360 seconds;
  - 360–600 seconds;
  - 600–900 seconds;
  - 900–1200 seconds.

## 4. Current implementation status at commit 9d76768

### Done

- raw evidence capture foundation;
- GT/contact capture at 20 ms;
- provenance and finalization;
- coverage replay foundation;
- odometry replay;
- shared trajectory/overlap replay in the salvaged path;
- pair/agreement deferred path;
- warning/Nav2 deferred path;
- handoff core evaluation;
- artifact validation.

### Missing / to complete

Offline aggregation and final-report integration remain to be completed for:

- exact avoidable idle;
- full snappiness joins;
- complete cooperation summaries;
- certificate/DNU reports;
- full anomaly reports;
- fairness;
- scaling windows;
- complete SLAM-quality reports;
- reciprocal verification where raw evidence permits;
- legacy `summary.json` / `mission_result` equivalence wherever scientifically
  required.

The items above are reporting/evaluation work. They do not authorize changes to
allocator, certificate, navigation, frontier, SLAM, or cooperation semantics.

## 5. Ordered completion roadmap

Each roadmap item is one logically isolated implementation step. It may be
split further if its raw inputs, semantics, or parity evidence are not
independent.

### 1. Exact avoidable idle and snappiness timeline integration

**Status: DONE** — implemented in the current working tree; checkpoint hash is
recorded in `reports/observer_evaluation_incremental_completion_20260907.md`.

**Required raw inputs:** authoritative work-availability/actionability state,
robot identity, state transitions and reasons, simulation timestamps, terminal
events, dispatches, handoff/release events, candidate/bid/agreement events, and
the required navigation/action timestamps.

**Expected artifacts:** structured idle intervals, avoidable-idle totals and
longest interval, productive-engagement intervals, and a snappiness latency
table with explicit join status.

**Validation:** replay preserved complete-evidence artifacts and compare with
the legacy exact-idle and timestamp semantics, including empty/zero/not-invoked
cases.

**Completion criteria:** every interval and latency has a valid source/time
domain or an explicit missing-evidence failure; no dispatch absence is treated
as proof that work was unavailable.

### 2. Cooperation, assignment, agreement, certificate, and DNU summaries

**Required raw inputs:** candidate/task generations, bids, pair decisions,
assignments, agreements, continuations, status/events, certificate reason and
counts, DNU state, dispatches, terminals, source/session/epoch identity, and
payload timestamps.

**Expected artifacts:** per-round and per-robot protocol tables, assignment /
agreement / continuation counts, certificate decision counts, DNU series, and
goal-lifecycle summaries.

**Validation:** deterministic payload replay against the legacy observer's
parsed outputs, preserving ordering, deduplication, generation identity,
zero-vs-missing semantics, and terminal classifications.

**Completion criteria:** every required protocol field is either reconstructed
from a preserved raw source or fails closed with the exact absent-source
diagnosis.

### 3. Motion anomaly replay

**Required raw inputs:** timestamped poses/odometry, velocity/cmd streams,
dispatches, terminals, state transitions, map/context evidence where required,
and the existing anomaly thresholds/definitions.

**Expected artifacts:** stalls, no-progress intervals, oscillation records,
recovery outcomes, and their supporting timestamps.

**Validation:** compare replayed classifications and intervals with the
legacy-derived outputs on the preserved fixture.

**Completion criteria:** anomaly and recovery semantics match, including valid
zero-anomaly runs and incomplete-stream failures.

### 4. Fairness metric definition and implementation

**Required raw inputs:** per-robot assignments, completed work/contribution,
travel and productive exploration measures, and the chosen experiment scope.

**Expected artifacts:** the defined fairness measure, per-robot inputs, and
the denominator/window used for each reported value.

**Validation:** approve the definition against the experiment plan and compare
deterministic replay with the reference where an authoritative historical
definition exists.

**Completion criteria:** the metric has one written definition, no hidden
zero-denominator behavior, and explicit handling of empty or non-invoked work.

### 5. Scaling-window analyzer

**Required raw inputs:** simulation-time boundaries, coverage/motion/protocol /
reliability streams, and the authoritative window cut rules.

**Expected artifacts:** separate 0–180, 180–360, 360–600, 600–900, and
900–1200-second rows, with completeness and boundary metadata.

**Validation:** synthetic boundary fixtures plus replay of a complete run;
verify no post-boundary sample contaminates a window.

**Completion criteria:** every configured window is either complete and
populated or explicitly marked unavailable for a justified evidence reason.

### 6. SLAM and map-quality reports

**Required raw inputs:** complete required map streams/snapshots, map metadata,
TF/odom, ground-truth joins, resolution/origin/frame information, and any
accepted quality thresholds.

**Expected artifacts:** map quality, localization/reference comparison, map
identity/revision, and time-aligned quality summaries.

**Validation:** exact occupancy interpretation and joins against the preserved
legacy reference; fail closed on missing maps or invalid metadata.

**Completion criteria:** required map-quality rows are reproducible with the
same definitions, units, frames, and timestamp semantics.

### 7. Handoff reciprocal verification where feasible

**Required raw inputs:** accepted handoff transform, acceptance timestamp,
evidence-set hash, estimator-quality fields, reciprocal verification evidence,
and physical GT poses.

**Expected artifacts:** residual/error/timing table and reciprocal-verification
status. Missing reciprocal evidence must remain distinct from a verified zero.

**Validation:** compare the offline result with the accepted handoff artifact
and legacy interpretation; test wrong/missing/corrupt evidence fail-closed.

**Completion criteria:** the result states exactly what was verified and what
was not observable from the run.

### 8. Final legacy-versus-current report comparison

**Required raw inputs:** all sources consumed by the completed metric modules,
the preserved legacy reference outputs, manifests, and finalization records.

**Expected artifacts:** requirements-to-source-to-metric matrix, parsed semantic
parity report, compatibility summary, and final thesis metric bundle.

**Validation:** fixture replay, focused negative tests, and one complete 180 s
pipeline validation before any 1200 s campaign.

**Completion criteria:** every required row is `PASS` or has an explicitly
approved replacement definition with documented missing-evidence boundaries;
the pipeline fails closed for incomplete artifacts.

## 6. Evidence and completion semantics

The evaluator must distinguish all of the following:

- channel exists and is readable;
- channel is semantically complete;
- a valid event occurred zero times;
- an event occurred one or more times;
- a mechanism was not invoked;
- a mechanism was invoked;
- a mechanism was invoked but its required payload is missing.

For each metric, the implementation must identify the raw source, message/topic
or artifact, timestamp basis, frame/source identity, minimum completeness
condition, and failure behavior. Derived outputs must not be considered valid
solely because a file exists or a basic topic is nonempty.

The authoritative time basis is simulation time or message/header time as
defined by the metric. Wall time is provenance/performance information unless
the metric definition explicitly requires it. All outputs must be cut at the
configured scientific horizon and must identify their termination and
finalization state.

## 7. Implementation and review rules

- Freeze the observer/evidence architecture while completing this specification.
- Reuse the salvaged legacy deferred primitives and one authoritative parsed raw
  representation wherever practical.
- Do not add a second observer, alter live robot behavior, or change scientific
  definitions to make a metric easier to report.
- Add only the smallest raw-evidence extension when a required metric is
  impossible to reconstruct from currently preserved evidence; document that
  source before implementing the metric.
- Validate each roadmap item with focused tests and preserved artifacts before
  accepting its output.
- Do not begin A/B/D campaigns or a 1200 s campaign until the 180 s pipeline,
  raw completeness, offline reconstruction, and finalization gates pass.

Current commit: `9d76768`

Current state: **Specification created; implementation not started from this
roadmap.**
