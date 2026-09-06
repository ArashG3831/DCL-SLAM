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
