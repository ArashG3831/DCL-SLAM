# Legacy Observer Incremental Salvage

## Baseline checkpoint

This report is maintained incrementally.  The current worktree was already
dirty before this salvage task; the pre-existing changes were not cleaned,
reset, or staged.

Authority files and baseline SHA-256 values before the first salvage change:

```text
cooperative_experiment_logger.py       2b3c9eba184ee097d23be253c9085dc96c786b56dad38b02d0c5bc5ebc559e4f
experiment_metrics.py                  4c4f4b550b82151cd2239b9f4a6c2b6a72d913c8bb53006eb38bfefd60710d2d
forensic_evidence.py                   63b1c8f46f528e8143b83481a1c65df4856012012fbda56e709f10fe49706c9b
cooperative_ground_truth_observer.py   09eeb534c646542c9d2f223d7e092378a4d068c90bb1f257fb1d558646ba65d9
passive_rosbag.py                      53254c4642937e58dd7894f6a2f8ae98c7a4147fd9880155756ef4e3e2df4bf9
raw_evidence_contract.py               0e0c04f09865f699430b4d9a7a141906467c6cf8681305f6ad1c5a62e686d99d
```

The focused baseline command was:

```text
PYTHONPATH=src/my_epuck_project:$PYTHONPATH python3 -m pytest -q \
  src/my_epuck_project/test/test_experiment_logger_runtime.py \
  src/my_epuck_project/test/test_cooperative_ground_truth_observer.py \
  src/my_epuck_project/test/test_passive_rosbag_offload.py
```

Result: **79 passed in 24.82 s**.

The current Condition-C configuration template is the legacy observer path:

```text
experiment_condition=C
assignment_strategy=frontier_cost_only
local_path_gate_mode=MODE_B
world_profile=large_unknown_pose_close_start_20ms_scan_matching
webots_mode=fast
rendering=false
sensor_profile=full
observer_architecture=legacy
enable_observer=true
forensic_ground_truth_sample_period_s=0.02
contact_sampling_period_ms=20
use_sim_time=true
```

The exact install, seed, port, horizon, and watchdog are campaign-attempt
values and must be recorded per runtime attempt rather than guessed here.

## Existing deferred mechanisms verified in the baseline source

The old source already contains the following mechanisms and they are not being
reimplemented:

1. `_reconstruct_deferred_synchronized_map_frames()` replays raw TF and odom
   causally and writes the established synchronized-map schema.
2. `export_index_with_bounded_retry()` plus `load_offloaded_timing()` and the
   `_restore_offloaded_artifacts()` / repair functions replay selected
   rosbag-offloaded sensor timing and health fields.
3. `_write_physical_gt_evaluation()` is already post-run.
4. `required_artifact_status()` and `semantic_export_complete()` are already
   finalization/validity paths.

These are treated as existing authority, not replaced by new evaluators.

## Salvage commits

The first implementation checkpoint is the odometry trajectory/distance family,
using the existing `LocalTrajectory` primitive and the already-recorded forensic
odometry CSV.

| Commit | Responsibility | Raw source | Deferred path | Parity | Runtime work removed |
|---|---|---|---|---|---|
| `f9a84d8` | Local odometry trajectory/distance/revisit replay primitive | `forensic/robot*_odom.csv` (`robot_id`, `pose_x`, `pose_y` in callback/file order) | `replay_local_trajectory_csv()` feeds the existing `LocalTrajectory.add()` implementation; live authority unchanged | 82 focused tests pass, including exact summary equality and fail-closed missing/malformed evidence tests | None yet |
| `85821dd` | Finalization parity comparison | Same closed forensic odometry CSVs | Legacy finalization writes `odometry_trajectory_parity.json` containing live/deferred semantic summaries; live remains authoritative | Logger-level fixture added in `f2dbe98`; 83 focused/regression tests pass | Comparison only; no mission-time work removed |
| `a4360db` | Deferred odometry summary authority | Same closed forensic odometry CSVs | On parity pass, finalization swaps the deferred `LocalTrajectory` into the existing summary path; live object retained for audit | Real preserved artifact replay matched `summary.json` exactly for both robots; 84 focused/regression tests pass after cleanup checkpoint | Final full trajectory bins/revisit state no longer remains live when forensic capture exists |
| `426b69b` | Remove full live trajectory state while preserving scalar consumers | Raw forensic odometry remains authoritative; live odom callback remains unchanged | `LiveDistanceAccumulator` retains only cumulative distance/last pose for telemetry and cycle rows; shared `planar_step_distance()` prevents distance-semantic drift; final summary uses deferred full `LocalTrajectory` | 84 focused/regression tests pass; `thesis_C_coverage_set_fastpath_rtf_validation_20260906` replay was exact: both robots' bins, revisit distance, total distance, sample counts, validity, reason and frames matched | Removes live bins/revisit/sample accumulation; retains required low-rate distance scalar |

### Odometry parity evidence

The preserved artifact used for an offline same-input check was:

`results/thesis_C_coverage_set_fastpath_rtf_validation_20260906/fast_trial_20260906T072837Z/observer/fast_trial_20260906T072837Z/`

The legacy `summary.json.motion` and replayed `forensic/robot1_odom.csv` plus
`robot2_odom.csv` summaries were semantically identical. Exact values included:

```text
robot1 bins=157, distance=6.954462368636347,
        repeated=4.968098596901526, samples=8670
robot2 bins=294, distance=12.576388759754769,
        repeated=8.745911202710461, samples=8590
```

The final odometry checkpoint keeps the legacy low-rate `distance_travelled_m`
and cycle-distance fields live, but removes only the full live bins/revisit
state when forensic odometry is available. If deferred odometry is missing or
malformed, finalization records the failure and does not claim a deferred
trajectory result; the existing artifact-contract path remains responsible for
the final validity decision.

## Rejected shared-frame trajectory migration

An offline replay prototype attempted to reuse the legacy TF composition path
with `forensic/raw_tf.csv` and the two odometry CSVs. It was reverted before
commit because parity failed on the preserved same-input artifact:

```text
live shared-frame samples: trajectory summary implied the full odometry path
replay shared-frame samples: 1,285 accepted TF/odom joins
robot1 bins: 55 replay vs 160 live
robot2 bins: 98 replay vs 294 live
cross-robot bins: 0 replay vs 19 live
```

The exact observed mechanism was that the live callback falls through to a
`tf2` lookup when the bounded raw-TF path has no <=0.10 s bracket. The raw CSV
does not preserve enough lookup-cache semantics to reproduce every live tf2
success at the odometry timestamps. Shared-frame trajectory/overlap therefore
remains live until a lossless TF lookup/evidence source is added and parity is
proven. No prototype code remains in the tree.

## Checkpoint f9a84d8

Commit: `f9a84d8` (`observer: add deferred odometry trajectory replay primitive`).

This checkpoint adds one deferred replay entry point and its focused tests. It
does not switch any production result to deferred authority, remove the live
trajectory accumulator, alter sampling, or change any artifact schema. The
replay deliberately consumes forensic odometry in write/callback order and
delegates all metric semantics to the existing `LocalTrajectory` class. Missing
files, missing required columns, robot mismatches, malformed values, and
non-finite poses fail closed.

Validation:

```text
PYTHONPATH=src/my_epuck_project:$PYTHONPATH python3 -m pytest -q \\
  src/my_epuck_project/test/test_legacy_observer_deferred_trajectory.py \\
  src/my_epuck_project/test/test_experiment_logger_runtime.py \\
  src/my_epuck_project/test/test_cooperative_ground_truth_observer.py \\
  src/my_epuck_project/test/test_passive_rosbag_offload.py
82 passed in 23.03s
```

The six authority-file hashes recorded before this checkpoint remain unchanged
for all files except `experiment_metrics.py`, whose change is exactly this
checkpoint. No Webots run or build was performed.

## Raw-evidence boundary for the next family

The next required source-first migration is lossless scientific rosbag capture
for map/coverage and TF-dependent replay. The current worktree contains an
uncommitted passive-bag implementation with an `include_scientific_raw` option,
but that module is not tracked at `HEAD`; the corresponding logger startup and
finalization wiring is also part of a 1,375-line pre-existing dirty diff in
`cooperative_experiment_logger.py`. The salvage protocol requires that an
accepted checkpoint contain only the isolated responsibility and directly
required tests. Staging that wiring would therefore absorb unrelated
pre-existing observer changes and would not be a safe isolated commit.

The preserved artifact confirms the scientific consequence independently: the
existing forensic map snapshots are sparse relative to the live coverage timer,
and the raw TF CSV does not preserve the live `tf2` lookup fallback. The
shared-trajectory/overlap replay was consequently rejected for exact parity.
Coverage/map replay must not be switched until the lossless raw source is
committed separately and its parity is proven.

No source migration was accepted after `426b69b`; no raw-capture code remains
uncommitted from this continuation attempt.

## Inherited observer baseline checkpoint

The current dirty observer/evidence state was pre-existing work from before this
salvage continuation. It was validated before checkpointing with the focused
legacy command:

```text
PYTHONPATH=src/my_epuck_project:$PYTHONPATH python3 -m pytest -q \
  src/my_epuck_project/test/test_experiment_logger_runtime.py \
  src/my_epuck_project/test/test_cooperative_ground_truth_observer.py \
  src/my_epuck_project/test/test_passive_rosbag_offload.py
79 passed in 23.05s
```

The baseline checkpoint contains only these observer/evidence files and this
report:

```text
src/my_epuck_project/my_epuck_project/cooperative_experiment_logger.py
src/my_epuck_project/my_epuck_project/cooperative_ground_truth_observer.py
src/my_epuck_project/my_epuck_project/passive_rosbag.py
src/my_epuck_project/my_epuck_project/raw_evidence_contract.py
src/my_epuck_project/test/test_experiment_logger_runtime.py
src/my_epuck_project/test/test_cooperative_ground_truth_observer.py
src/my_epuck_project/test/test_passive_rosbag_offload.py
reports/legacy_observer_incremental_salvage_20260906.md
```

This is a reference-state checkpoint, not a salvage optimization. It records
the inherited implementation so subsequent raw-evidence and metric migrations
can be isolated and bisected. Unrelated dirty files were deliberately not
staged.

Baseline checkpoint commit: `4429f2c`.

## Lossless scientific raw capture enabling checkpoint

The inherited passive rosbag implementation already provided the authoritative
`scientific_raw_topics()` set and contract validation. This checkpoint exposes
it through the legacy logger as an opt-in parameter:

```text
enable_scientific_raw_capture: false  # default remains unchanged
```

When enabled, the existing native rosbag recorder and its existing finalization
export receive `include_scientific_raw=True`. The selected streams include
`/clock`, `/tf`, `/tf_static`, common START_RELEASE, both robots' odometry,
commands, local/shared maps, frontier/task/cooperation protocol streams and
navigation action status. No live metric, subscription, sampling rate,
classification, or artifact authority was removed in this checkpoint.

Focused validation after the wiring change:

```text
80 passed in 24.59s
```

Commit: `e266623` (`observer: enable lossless scientific raw evidence capture`).

## Shared trajectory / overlap replay checkpoint

The complete raw-bag replay path is now implemented in the isolated module
`src/my_epuck_project/my_epuck_project/deferred_tf_trajectory.py`. It reads the
native `/tf`, `/tf_static`, and per-robot `/odom` streams in rosbag arrival
order, inserts dynamic/static transforms into the same `tf2_ros.Buffer` API
used by the live observer, performs the same zero-timeout timestamped lookup,
and feeds the existing `TrajectoryOverlap` primitive. It therefore does not
introduce a second overlap or distance definition. Missing/mistyped required
streams and malformed/non-finite poses fail closed.

The legacy logger now emits
`shared_trajectory_parity.json` at finalization when opt-in scientific raw
capture is enabled. It records the live summary beside the deferred summary,
accepted/skipped sample counts, deserialization count, errors, and an explicit
`PARITY_PASS`, `PARITY_FAIL`, or fail-closed status. Live trajectory/overlap
remains authoritative; no runtime behavior or output authority was switched in
this checkpoint.

Focused tests after the implementation and finalization-wiring correction:

```text
82 passed in 23.27s
```

Two unrelated pre-existing tests in the broader selected set still fail because
the active installed interface artifacts do not expose fields already present
in dirty source message definitions (`costmap_revision` and
`feasible_work_available`). They are not exercised by this replay checkpoint.

Checkpoint commit: `22bf48f`
(`observer: add rosbag tf2 trajectory parity replay`).

The next required evidence is a fresh run with
`enable_scientific_raw_capture=true`; exact live-vs-bag parity cannot be claimed
from the older sparse forensic artifact. Until that run proves parity, the
authority switch and removal of live shared trajectory/overlap accumulation are
intentionally not performed.

## Continuation checkpoints after the inherited baseline

The raw-capture wiring was isolated from unrelated dirty source and committed
as `55ecf4e` (`observer: expose opt-in scientific raw capture`). The affected
Python package and current interface definitions were rebuilt together into
`install_legacy_salvage_20260906/`. The current interface build was necessary
because the inherited underlay did not contain message fields used by the
current allocator source. Those interface changes remain pre-existing dirty
work and were not staged by the salvage commits.

Shared trajectory replay was corrected in two isolated commits:

* `86b73bf` — the native bag remains the complete payload/integrity source, but
  the exact legacy lookup boundary is replayed from the existing forensic
  callback-order TF/odom rows through the same `tf2_ros.Buffer` semantics;
* `8989e2b` — final `shared_frame_motion` authority switches only when that
  causal replay equals the live `TrajectoryOverlap` summary.

The native-bag replay is retained in the parity artifact as
`native_bag_integrity`; it is not silently treated as equivalent because
rosbag2 inter-topic recording order is not the legacy callback order. Focused
legacy/replay tests passed `106/106` before the map checkpoint.

## Map receipt enabling checkpoint

`0ce7ced` (`observer: record causal map receipt evidence`) adds only an opt-in
`map_receipts.jsonl` ledger and fail-closed artifact requirement when scientific
raw capture is enabled. Each row records robot/key, simulation and receipt
times, map metadata, sequence, and a digest of the OccupancyGrid payload. It
does not change live coverage calculation or authority.

`f8feaef` (`observer: add deferred coverage replay parity`) adds
`deferred_coverage.py`, which joins native rosbag map payloads to the receipt
ledger and reuses the existing `known_counts`, `known_world_cells`, and
`CoverageAttribution` implementations. It emits
`coverage_replay_parity.json`; no coverage authority switch was made.
Focused tests passed `107/107`.

## Fresh map-parity validation and blocker

Fresh validation artifact:

```text
results/legacy_salvage_map_parity_20260906/
  fast_trial_20260906T183007Z/
```

Validity:

* Condition C, `frontier_cost_only`, MODE_B, seed 1001, canonical close-start
  20 ms scan-matching world, full sensor profile, fast/no-rendering mode;
* isolated install `install_legacy_salvage_20260906` plus current interface
  overlay; Webots driver from `webots_ws_close_validation_2eb`;
* simulation `0.02 -> 120.06 s`, termination `SIM_TIME_COMPLETE`, launch return
  code 0, observer finalization complete, raw rosbag export complete;
* GT/contact and all required legacy artifacts finalized; `map_receipts.jsonl`
  contains 145 rows.

Shared trajectory parity passed in this run:

```text
shared_trajectory_parity.json: PARITY_PASS, equal=true
```

Coverage parity did not pass:

```text
coverage_replay_parity.json: PARITY_FAIL
sample_count: 51
difference_count: 197
```

The failure is concrete, not an inferred timing difference. At the first
coverage samples the live legacy `coverage.csv` reports robot1/robot2 local
known counts `2637/2590`, while the exact joined native-bag payloads identified
by the receipt ledger contain `2444/2431`. The bag has no robot1/map payload
with 2637 known cells; the joined receipt digest for the live-visible early
robot1 map is consistently the payload whose replay count is 2444. Later rows
show the same class of divergence (for example live 5927 versus replay 5659).

Therefore the raw payload/receipt contract is not yet sufficient to reproduce
the old observer's live map-count result. The single-family diagnosis must next
resolve the legacy in-memory `map_snapshot()` cache and its `id(message)` key
against the logger/rosbag observation boundary. No authority switch is
justified until the discrepancy is explained. Coverage migration is rejected
at this checkpoint; live map conversion, counting, attribution, and
`coverage.csv` authority remain unchanged. No protocol or later family was
started on top of this unproven state.

## Map-parity correction and deferred coverage authority

The preceding blocker text described the first replay implementation, not the
final result for this responsibility. `d2c9e35` corrected two replay defects
in the deferred implementation itself:

1. local known-cell counts now use `free + occupied`, matching the legacy
   `map_counts()` contract rather than free cells alone;
2. a map receipt with the same simulation timestamp as a coverage timer is
   excluded (`receipt_time < request_time`), matching the observed ROS callback
   order in the legacy logger.

Replaying the preserved
`results/legacy_salvage_map_parity_20260906/fast_trial_20260906T183007Z`
artifact after those corrections produced 51/51 semantic rows with zero
differences. The original `coverage_replay_parity.json` in that artifact is
intentionally not overwritten; it records the pre-correction run state. The
post-correction result is an offline recheck of the same raw bag, map receipts,
and live coverage evidence.

`c177b4a` then switched the coverage summary's final authority to the deferred
result only after that exact parity. This did not yet remove live conversion or
counting from the raw-enabled mission path.

The next isolated migration is the authority-preserving removal of that live
map-derived work. Raw-enabled runs now keep the exact timer request boundary in
`coverage_requests.jsonl`; finalization replays the closed native bag through
the existing map/count/attribution primitives and materializes the established
`coverage.csv` schema. The default non-scientific-raw legacy path still runs
the original live `sample_coverage()` implementation unchanged. The raw
contract requires `coverage_requests.jsonl`, `coverage.csv`, and
`coverage_replay_parity.json`.

Focused validation for the current migration: `107 passed in 24.47s` across
deferred coverage, logger runtime, metrics, passive rosbag, and TF/trajectory
replay tests. No runtime validation has yet been run for this latest removal;
the next required check is a fresh short complete raw-evidence Condition-C run
after rebuilding the isolated salvage install.

## Coverage offload runtime closeout

The first post-`4001ecc` runtime attempt was invalid before the mission because
the runner rejected an unsupported ROS domain, and the corrected shell then
failed preflight because it lacked the established frontier/SLAM underlay. No
Webots process was started by either preflight failure. These attempts are not
scientific runs.

After restoring the exact prior underlay and rebuilding, the clean validation
was:

```text
results/legacy_salvage_coverage_offload_20260906_retry2/
  fast_trial_20260906T185304Z/
```

Configuration matched the prior C validation: canonical close-start 20 ms
scan-matching world, seed 1001, Condition C, `frontier_cost_only`, MODE_B,
full sensors, ideal encoders, fast/no-rendering Webots, forensic/contact
capture, and scientific raw capture. The rebuilt project prefix was
`install_legacy_salvage_20260906`; the Webots driver was
`webots_ws_close_validation_2eb/install/webots_ros2_driver`.

Runtime result:

* simulation `0.02 -> 120.06 s`;
* termination `SIM_TIME_COMPLETE`, no horizon overrun;
* wall duration `100.9646 s` (runner wall duration, including readiness);
* emergency 300 s watchdog armed and not fired;
* launch return code `0` and observer finalization complete;
* artifact contract `COMPLETE`, with no missing artifacts;
* 20 ms GT/contact configuration retained: 12,087 GT data rows and 59,381
  contact data rows after CSV headers;
* 51 request-ledger rows and 51 materialized `coverage.csv` rows;
* `coverage_replay_parity.json`: `DEFERRED_AUTHORITATIVE`, sample count 51,
  `live_rows_during_mission=0`;
* shared trajectory replay remained `PARITY_PASS`, `equal=true`;
* native scientific bag export was complete and included `/clock`, `/tf`,
  `/tf_static`, both odometry streams, local/shared maps, and the existing
  protocol/navigation raw topics.

The final run proves the latest coverage migration's contract. It does not
claim an authoritative active-mission RTF measurement: the saved runner
`wall_runtime_s` includes startup/readiness and this checkpoint was a functional
salvage validation, not a performance acceptance run.

## Salvage checkpoint history

The isolated salvage history, in order, is:

* `4429f2c` — inherited observer/evidence baseline checkpoint. Captured the
  pre-existing authoritative dirty observer state only; explicitly not a
  salvage optimization.
* `e266623` — exposed the existing scientific raw-topic contract as an opt-in
  passive-rosbag mode, without changing live metrics.
* `22bf48f` — added native-bag TF2 trajectory replay and parity artifacts.
* `746d325` — documented that trajectory checkpoint.
* `55ecf4e` — wired the runner/launch opt-in scientific raw capture flag.
* `86b73bf` — preserved exact legacy causal TF2 replay using forensic callback
  ordering, because native rosbag inter-topic ordering alone was not equivalent.
* `8989e2b` — switched shared-frame trajectory summary authority only after
  causal replay matched the live summary.
* `0ce7ced` — added the compact causal `map_receipts.jsonl` ledger, leaving live
  coverage authoritative.
* `f8feaef` — added deferred coverage replay using the existing grid/count and
  `CoverageAttribution` primitives, initially as parity-only.
* `c3ae8e6` — documented the first map-parity result and its then-open blocker.
* `d2c9e35` — corrected deferred coverage to count free+occupied cells and to
  use the strict legacy callback boundary at equal timestamps.
* `c177b4a` — switched the final coverage summary authority to the deferred
  result after the preserved artifact replay was exactly equal (51/51 rows).
* `4001ecc` — removed map conversion/counting/ownership/attribution from the
  scientific-raw mission path; records only coverage timer requests and
  regenerates the established `coverage.csv` schema at finalization.
* `e9b1a4d` — fixed the request ledger to preserve the legacy early return when
  the selected map stream was not yet available; added the focused regression
  test.

The docs-only commits `245ae8d`, `c4853f7`, `426b69b`, `a4360db`, `85821dd`,
and `c82e698` record earlier odometry/raw-evidence checkpoints in the same
salvage history. The odometry work was deliberately staged as replay,
comparison, authority switch, and live-state removal checkpoints; no later
family was started before its parity gates.

## Current state and remaining scope

The old observer remains the behavioral reference and the default legacy path.
Scientific raw capture is opt-in and now owns lossless ROS streams through the
existing passive rosbag infrastructure. Shared trajectory and coverage have
deferred authority only in that raw-enabled path after parity. GT/contact
sampling, START_RELEASE/readiness, provenance, finalization, schemas, and the
robot/coordinator behavior were not changed.

No protocol/event, action-result, warning, health-history, summary, or resource
family was migrated in the initial coverage checkpoint. Subsequent isolated
checkpoints have since enabled raw protocol/action/path/rosout capture, switched
pair-decision and agreement summaries to deferred authority, and added warning
and Nav2-diagnostic semantic replay parity. The live warning and diagnostic
artifacts remain authoritative because receipt wall-time and global event-order
semantics have not yet been replaced. Health-history, summary/resource, and
full action-result authority migrations remain open; they require their own
raw/parity checkpoints rather than being represented as completed here.

## Protocol raw-export and pair-decision continuation

The next protocol checkpoint exposed a contract bug rather than a scientific
parity failure. `dae3528` had correctly recorded optional C protocol streams,
but the scientific export path treated every selected `if_published` stream as
mandatory. A valid run with zero messages on an optional exploration-status/event
topic was therefore marked incomplete. `0b7165f` changed only the export-set
resolution: the strict `required_nonempty_topics()` set is now used for the
required export contract, while optional streams remain selected and reported
without being promoted to missing evidence. The focused passive tests passed
(`17`), and the change was committed separately.

The repaired clean canonical C validation was:

```text
results/legacy_salvage_protocol_exportfix_20260906/
  fast_trial_20260906T192547Z/
```

It reached `SIM_TIME_COMPLETE` at `120.62 s` simulated time, with artifact
finalization `COMPLETE`, no missing artifacts, complete native raw-bag export,
`44` pair-decision messages replayed, and pair-decision replay parity passing.
The fixed-horizon run was intentionally `MISSION_NOT_TERMINATED` in its compact
mission result; that is expected for a 120-second validation horizon and is not
a full mission-completion claim.

`1d3c133` then switched the pair-decision summary authority: finalization keeps
the live/deferred parity artifact, but `summary.json` uses replayed outcomes only
after `PARITY_PASS` (or an explicitly authoritative deferred result), with live
fallback on replay failure or raw capture being disabled. Focused protocol,
passive, and logger tests passed (`49`), plus syntax and diff checks.

`548be61` added the next isolated protocol subfamily, agreement-counter replay.
It replays `/robot*/distributed_event` from the existing native bag and checks
the exact legacy semantics for `DECISION_AGREED` publications, unique rounds,
and unique decision hashes. The logger now emits
`agreement_replay_parity.json` and switches those three summary counters to the
deferred values only after parity. The preserved complete-evidence raw bag was
replayed offline: `81` distributed-event messages were deserialized and the
legacy summary matched exactly (`0` publications, `0` unique rounds, `0` unique
decisions). Focused protocol/passive/logger tests passed (`72`), and the source
was rebuilt in `install_legacy_salvage_protocol_20260906`.

The agreement checkpoint has not yet been given a new Webots runtime run; the
next validation must confirm the new parity artifact is included in the final
artifact contract. No claim is made here that the runtime finalization contract
has already been exercised with this latest commit.

`8f05a15` completed the agreement subfamily's live-work removal. In raw-enabled
legacy C runs, `distributed_event()` still records the complete event payload,
but no longer maintains the redundant live agreement counters/sets; the old
counter path remains unchanged when scientific raw capture is disabled. A
deferred replay failure now fails the raw artifact contract rather than silently
publishing empty agreement statistics.

The clean validation for that authority/removal checkpoint was:

```text
results/legacy_salvage_agreement_authority_20260906/
  fast_trial_20260906T194242Z/
```

It reached `SIM_TIME_COMPLETE` at `120.06 s` simulated time with no horizon
overrun, `94.6743 s` reported wall runtime, clean observer finalization, and
complete native raw export. `agreement_replay_parity.json` was
`DEFERRED_AUTHORITATIVE`; it deserialized `96` distributed-event messages and
reconstructed `0` agreement publications, `0` unique rounds, and `0` unique
decisions. The summary contained those deferred values and the artifact contract
was `COMPLETE`. The fixed-horizon run is a functional checkpoint, not a mission
completion or RTF acceptance run.

`3e20f6c` completed the analogous pair-decision live-work removal. In
raw-enabled runs the callback still computes the per-message outcome needed for
the established `events.jsonl` payload, but no longer maintains the redundant
live `round_outcomes` aggregate. The native-bag replay is now authoritative;
non-raw legacy runs retain the original live counter path. Pair replay failure
is fail-closed through the artifact contract.

Validation:

```text
results/legacy_salvage_pair_authority_20260906/
  fast_trial_20260906T194723Z/
```

The run reached `SIM_TIME_COMPLETE` at `120.06 s`, with no horizon overrun,
`98.7974 s` reported wall runtime, complete raw export/finalization, and
`pair_decision_replay_parity.json` marked `DEFERRED_AUTHORITATIVE`. The bag
replayed zero pair-decision messages in this fixed short horizon and the summary
correctly contained an empty `round_outcomes` mapping. Agreement replay also
remained deferred-authoritative over `55` distributed-event messages. This is a
functional checkpoint, not a mission-completion or RTF acceptance run.

## Action/path raw-evidence enabling checkpoint

`c4d3799` extended the existing native scientific raw contract from version
`1.2` to `1.3` with optional action/path streams needed for a future exact
navigation replay: NavigateToPose feedback, FollowPath status, ComputePathToPose
status, and `/plan` for each robot. This commit intentionally left every live
navigation callback and output authoritative; it only made the raw source
available and updated the contract/topic-set tests.

Validation used the rebuilt project install and the same canonical C world,
seed, MODE_B/frontier-cost-only configuration, 20 ms GT/contact settings, and
120 s simulation horizon:

```text
results/legacy_salvage_action_raw_20260906/
  fast_trial_20260906T195222Z/
```

The run reached `SIM_TIME_COMPLETE` at `120.06 s` with no horizon overrun,
complete observer finalization, and complete native raw export. The newly
captured streams contained: robot1/robot2 NavigateToPose feedback `3043/3401`,
ComputePathToPose status `272/232`, FollowPath status `89/68`, and `/plan`
`134/114` messages. This is an enabling checkpoint only; action/path semantic
replay has not yet been switched to deferred authority.

`d473067` added `/rosout` to the version `1.4` scientific raw contract as an
optional native bag stream. This is an enabling change for a future warning and
diagnostic replay; the live `/rosout` subscription, normalization, warning
artifacts, and validity behavior remain unchanged.

The validation run was:

```text
results/legacy_salvage_rosout_raw_20260906/
  fast_trial_20260906T195542Z/
```

It reached `SIM_TIME_COMPLETE` at `120.06 s`, with no horizon overrun, complete
raw export/finalization, and `1,995` `/rosout` messages in the native bag. This
is raw-source enablement only; no warning/diagnostic authority switch has been
claimed. Exact legacy warning parity still requires preserving the observer's
wall-receipt timestamps and event ordering; the current bag alone does not
prove those fields.

## Rosout receipt-ledger checkpoint

`f9a9116` adds a compact `rosout_receipts.jsonl` ledger to raw-enabled legacy
runs. It preserves the observer-side receipt sequence, Python receipt wall time,
observer ROS time, source message stamp, severity, logger name, and message text.
This is an evidence-enabling change only: the existing live warning normalization,
diagnostic counters, `events.jsonl`, and validity behavior remain authoritative.
The ledger is required by the raw artifact contract so a future warning/diagnostic
replay can compare legacy callback receipt semantics instead of assuming that
native bag timestamps reproduce Python callback receipt time.

The bounded validation used the rebuilt install
`install_legacy_salvage_rosout_receipts_20260906` with the canonical C,
frontier-cost-only, MODE_B, 20 ms, fast/headless configuration and a 120 s
simulation horizon:

```text
results/legacy_salvage_rosout_receipts_20260906/
  fast_trial_20260906T200246Z/
```

It reached `SIM_TIME_COMPLETE` at `120.06 s` with simulation-horizon overrun
`false`, launch return code `0`, observer finalization `COMPLETE`, and no missing
required artifacts. The run produced `1,820` sequential receipt rows
(`receipt_sequence` 0 through 1,819), while the native `/rosout` bag stream
contained `1,882` messages. The difference is expected because the compact
observer ledger intentionally excludes messages filtered by the existing logger
policy (and the native bag remains the complete raw source); it is not used as a
replacement for the bag. Receipt rows covered simulation receipt times from
`0.0` through `120.62 s`, with no recorded receipt-write failure. The native bag
also retained the required `/rosout` payload stream.

`pair_decision_replay_parity.json` and `agreement_replay_parity.json` remained
`DEFERRED_AUTHORITATIVE`, and the final artifact contract was complete. No
warning/diagnostic authority switch is claimed by this checkpoint; the next
isolated work must add and validate warning replay against the bag plus this
receipt ledger before disabling any live warning-derived work.

`6f6708f` adds the adjacent Nav2 diagnostic replay parity path. The diagnostic
category rules and rate-limit boundary are shared with the live rosout callback;
the replay compares the legacy diagnostic fields, including the source stamp
stored in the legacy `ros_time_sec/ros_time_nanosec` columns. It does not switch
the live diagnostic file or planner-query counters to deferred authority.

Focused syntax/logger/protocol/passive tests passed (`80`), and the affected
package was rebuilt as `install_legacy_salvage_nav2_diag_replay_fix_20260906`.
The clean validation was:

```text
results/legacy_salvage_nav2_diag_replay_fix_20260906/
  fast_trial_20260906T202153Z/
```

It reached `SIM_TIME_COMPLETE` at `120.06 s`, with horizon overrun `false`,
launch return code `0`, finalization `COMPLETE`, and no missing artifacts.
Warning replay passed (`81` live/deferred records from `2,165` receipts), and
Nav2 diagnostic replay passed (`227` live/deferred diagnostic records). The
comparison remains semantic-only for receipt wall time and global event
sequence; those legacy timing fields still require live authority until an
explicitly approved replacement is established.

## Warning/rosout semantic replay checkpoint

`76f7516` adds the next isolated parity step. The existing warning-category
classification is now owned by the shared `warning_category()` helper used by
both the live callback and replay. `replay_warning_records_from_receipts()`
feeds the recorded receipt rows through the same `WarningDeduplicator` and
produces a semantic comparison containing node, severity, representative and
normalized message, occurrence count, and category. Wall timestamps are
intentionally not treated as equal yet: the old live callback calls `utc_now()`
when adding the warning, while the receipt ledger records the earlier observer
receipt time. The live warning artifact therefore remains authoritative; this
checkpoint only proves the non-time warning semantics.

Focused syntax/logger/protocol/passive tests passed (`79`). The affected package
was rebuilt as `install_legacy_salvage_warning_replay_20260906`.

The clean canonical validation was:

```text
results/legacy_salvage_warning_replay_20260906/
  fast_trial_20260906T201450Z/
```

It reached `SIM_TIME_COMPLETE` at `120.06 s`, with horizon overrun `false`,
launch return code `0`, observer finalization `COMPLETE`, and no missing required
artifacts. `warning_replay_parity.json` reported `PARITY_PASS` with `59` live
and `59` deferred warning records, from `1,893` receipt rows (`133` warning or
error rows). No live warning/diagnostic work was disabled by this checkpoint.
The next warning migration, if pursued, must separately preserve or explicitly
approve the live warning timestamp/event-order semantics before switching final
warning artifacts to deferred authority.

## Warning/Nav2 final-artifact authority checkpoint

`c81ae5a` completes the warning/Nav2 standalone-artifact authority step.  The
rosout receipt ledger now retains the warning timestamp used by the legacy
deduplicator and the complete diagnostic record metadata (event sequence,
timestamps, source and severity fields).  Replay therefore produces the same
final warning and Nav2 diagnostic records from the preserved receipt stream,
including the fields that were previously excluded from semantic-only parity.

For raw-enabled runs, finalization selects replayed warning records after a
successful warning parity check and rewrites `warnings.jsonl`; the summary's
warning occurrence total uses the same deferred records.  Nav2 diagnostic
records are selected only when replay has all required record metadata and are
written after the live diagnostic handle is closed.  If raw capture is
disabled, or replay is unavailable/incomplete, the established live path
remains in force and the artifact contract fails closed for raw-enabled runs.

The rosout event/category and planner-query counters remain live in this
checkpoint because their global event ordering and query attribution are still
consumers of the legacy callback.  No robot, allocator, certificate, Nav2,
sampling, or protocol behavior changed.  The focused deferred-protocol and
logger tests passed: `64` tests.  No runtime validation was run for this
checkpoint.

## Action/path replay checkpoint

`288357c` adds `deferred_navigation.py`, a bounded replay over the existing
native action/path streams.  It preserves bag order while normalizing status
transitions with the same per-goal duplicate suppression as the legacy
`action_status()` callback, computes planner-path length from finite path
poses, and records only recovery transitions from NavigateToPose feedback
rather than retaining full deserialized messages.  Missing or mistyped
navigation-status streams are reported as incomplete.

Finalization writes `navigation_action_replay.json` for raw-enabled runs and
Condition C requires that artifact through the existing fail-closed artifact
contract.  The existing claim/distributed-event lifecycle remains the
authority for the legacy navigation summary; this replay artifact supplies
the previously missing raw action/path evidence and does not change dispatch,
terminal classification, or Nav2 behavior.

The replay was run offline against the preserved complete-evidence bag from
`legacy_salvage_action_raw_20260906`: `7,377` messages deserialized,
`248` planner paths, `23` recovery transitions, and complete per-robot
NavigateToPose status streams.  Focused replay, protocol, and logger tests
passed: `66` tests.  No Webots runtime was run for this checkpoint.

## Raw-backed health-history checkpoint

`00df2f7` moves the historical `topic_health.csv` row path to finalization for
raw-enabled runs.  The live `sample_health()` callback still performs the
existing TF availability and stale-transition checks, preserving runtime
validity/event semantics; it buffers the same timestamped rows rather than
serializing them during the mission.  Finalization reconstructs ages and
10-second rates from the existing bag-to-`/clock` timing index for odometry,
joint/scans, maps, peer maps, shared maps, frontier/task/status streams,
NavigateToPose feedback, and `cmd_vel`.  Costmap rows remain live because
costmap streams are not in the scientific raw contract.

Headerless raw messages are now retained in the timing index using their
mapped simulation receipt time; headered streams preserve their header stamp.
The legacy CSV schema, row ordering, timer cadence, stale thresholds, and
missing-data behavior are unchanged.  The existing non-raw/passive path is
unchanged.  Focused replay, protocol, passive, navigation, and logger tests
passed: `85` tests.  No runtime validation was run for this checkpoint.

## Current open salvage work

The remaining open families are action/path authority integration beyond the
new normalized evidence artifact, resource-history ownership, and final
summary/finalization cleanup.  The action/path artifact is currently
authoritative for the raw action/path evidence itself, while the established
claim/distributed-event lifecycle remains authoritative for the existing
navigation outcome summary; replacing that summary would change its semantic
owner without a proven parity definition.  CPU/RSS history remains live
observer measurement because post-run replay cannot recreate the observer's
process-resource history; any change requires an explicit measurement-owner
decision and a separate parity/validity checkpoint.

## Warning/action/health build checkpoint

The source state containing `c81ae5a`, `288357c`, and `00df2f7` was rebuilt
without source changes in an isolated prefix:

```text
install_legacy_salvage_warning_action_health_20260907b
```

The initial colcon invocation was rejected before compilation because preserved
result and scratch directories expose duplicate package names.  The successful
invocation explicitly limited package discovery to `src/my_epuck_interfaces`
and `src/my_epuck_project`; it finished both selected packages successfully.
The duplicate preserved copies were not changed.

Installed-package focused validation passed: `85 passed in 28.24s`, covering
deferred navigation, protocol replay, passive-rosbag offload, and experiment
logger runtime behavior.  No Webots run was performed for this checkpoint.

The current salvage boundary is now evidence-defined: warning and Nav2
standalone artifacts use replay authority; the action replay artifact is
complete and authoritative for raw action/path evidence, but the existing
navigation summary remains claim/distributed-event authority because those
events define the coordinator lifecycle.  Health history is replayed for
raw-backed streams while live stale-transition checks remain.  CPU/RSS history
remains live because no exact post-run source can reproduce observer-process
resource samples.  Finalization's two summary/mission-result writes remain
necessary: the first creates the files used by the artifact contract, and the
second records the final contract result after all required artifacts and
replay checks are known.

## Nav2 diagnostic serialization migration

`08bdaa4` removes the duplicate mission-time write path for
`nav2_diagnostics.jsonl` when native raw capture and receipt replay are enabled.
The callback still constructs the same diagnostic metadata row so the receipt
ledger preserves its event sequence, timestamps, source stamp, category and
message.  Replay applies the same 0.25-second/key rate limit and 10,000-record
cap, and finalization writes the same output schema from the replayed records.
Non-raw runs retain the original live rate-limit/write path.

The raw-mode replay status is explicitly `DEFERRED_AUTHORITATIVE`; the legacy
live file is not used as a second copy.  A focused regression verifies that raw
mode leaves the mission-time diagnostic file empty while retaining the receipt
path.  This is a diagnostic-output ownership change only: certificate,
allocator, Nav2 and robot behavior are untouched.

The affected packages were rebuilt successfully in:

```text
install_legacy_salvage_nav2_defer_20260907
```

Installed-package focused tests passed: `84 passed in 27.19s`.  No Webots
runtime was run.

## Action/path live-calculation cleanup

`6b86e80` removes the unused live path-length calculation in `plan()`.  The
callback still marks and retains the received plan message for the legacy
stream semantics, while `deferred_navigation.py` remains the sole producer of
the normalized raw action/path plan lengths in
`navigation_action_replay.json`.  A repository reference check found no
consumer of `latest[r]['path_length']`; no output schema, action lifecycle,
terminal classification, or robot behavior changed.

Focused syntax/deferred-navigation/protocol/logger tests passed: `67 passed`.
The affected packages rebuilt successfully in:

```text
install_legacy_salvage_plan_20260907
```

No Webots runtime was run for this checkpoint.
