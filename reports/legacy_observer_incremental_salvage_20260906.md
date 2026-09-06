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
