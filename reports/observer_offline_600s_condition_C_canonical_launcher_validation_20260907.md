# Canonical-launcher 600 s Condition-C validation

## Result

This was the one authorized 600 s launch through the canonical launcher. It
failed during startup before the scientific mission became operational. No
retry and no offline evaluator run were performed.

| Field | Result |
|---|---|
| Source HEAD | `a03e609e6b9ac844edd7bc3043a1404024209936` |
| Launcher commit | `7d4ef4b` |
| AGENTS policy commit | `a03e609` |
| Command | `python3 src/my_epuck_project/tools/run_thesis_experiment.py --condition C --horizon 600` |
| Result root | `results/thesis_condition_C_600s_20260907T174921Z/` |
| Attempt | `fast_trial_20260907T174927Z/` |
| Configured horizon | 600 simulated seconds |
| Runner termination | `FAILURE` |
| Failure phase | `cooperation_graph_ready` |
| Runner return code | 1 |
| Scientific horizon reached | NO |
| Offline evaluator | NOT RUN; invalid startup artifact |
| Final verdict | `CONDITION_C_600S_CANONICAL_PIPELINE_FAIL` |

## Launcher and environment

The launcher itself passed its canonical preflight before spawning the existing
runner:

- `CANONICAL_THESIS_PREFLIGHT_PASS` was printed.
- WSL network mode was `nat`.
- Resolved endpoint was `172.18.32.1`.
- ROS domain was 230.
- Driver prefix was `/home/arash/webots_ws_close_validation_2eb/install/webots_ros2_driver`.
- Project prefix was `/home/arash/webots_ws_clean_validation_20260823/install/my_epuck_project`.
- The ROS Jazzy vendor library path, including `.../opt/sdformat_vendor/lib`,
  was retained.
- Existing provenance reported no contaminated original-workspace paths.
- Both requested `ldd` checks passed with no `not found` dependencies.
- The exact environment object used for preflight was passed to the existing
  `run_cooperative_trial_fast.py` child.

The launch therefore did not reproduce the earlier NAT, `LD_LIBRARY_PATH`, or
result-root failures.

## Startup evidence

Webots and the basic robot runtime started successfully:

- Webots external controllers connected for `robot1`, `robot2`,
  `Ros2Supervisor`, and `ForensicGroundTruthSupervisor`.
- Both `robot_state_publisher` processes initialized.
- Both controller managers received `robot_description`.
- The D500 scan transport and scan-fix processes started.
- SLAM, map fusion, Nav2, frontier generators, and observer startup began.

The first scientific runtime failure was the distributed assignment process on
both robots. The relevant log entries are in:

`results/thesis_condition_C_600s_20260907T174921Z/fast_trial_20260907T174927Z/launch.log`

At `distributed_frontier_assignment.py:4861`, both processes attempted to
assign:

```text
message.feasible_work_available = local_feasible
```

and raised:

```text
AttributeError: 'DistributedExplorationStatus' object has no attribute 'feasible_work_available'
```

Both processes exited with code 1. The two unknown-pose frontend processes
subsequently failed at `unknown_pose_frontend.py:6222` while assigning:

```text
message.stationary_witness_scheme_version = ...
```

with:

```text
AttributeError: 'RelativePoseHypothesis' object has no attribute 'stationary_witness_scheme_version'
```

The runner consequently reported:

```text
FAST_TRIAL_FAILURE reason=launch exited during cooperation_graph_ready with code 1
```

## Proven root cause

The selected install contains stale generated Python interface classes.

The current committed source definitions contain the fields required by the
current runtime code:

- `src/my_epuck_interfaces/msg/DistributedExplorationStatus.msg` contains
  `feasible_work_available`, `actionable_work_available`, and
  `work_availability_reason`.
- `src/my_epuck_interfaces/msg/RelativePoseHypothesis.msg` contains the
  stationary-witness fields, including
  `stationary_witness_scheme_version`.

However, the runtime classes loaded from:

`install/my_epuck_interfaces/lib/python3.12/site-packages/my_epuck_interfaces/msg/`

do not expose either field set. Direct inspection in the exact sourced install
reported:

```text
DistributedExplorationStatus ... feasible_work_available=False stationary_witness_scheme_version=False
RelativePoseHypothesis ... feasible_work_available=False stationary_witness_scheme_version=False
```

The generated `DistributedExplorationStatus` Python file in `build/` and the
one in `install/` are byte-identical and both predate the current source
message definitions. The installed project Python code is therefore newer than
the generated interface runtime it imports.

## Causal chain

```text
current distributed_frontier_assignment.py
  -> publishes newly required feasible-work fields
  -> stale DistributedExplorationStatus Python class lacks those fields
  -> both assignment nodes exit code 1
  -> cooperation_graph_ready cannot complete
  -> unknown-pose frontends also hit stale RelativePoseHypothesis class
  -> runner terminates the launch
  -> no valid 600 s mission, complete raw evidence, or scientific metrics
```

The later controller/Webots termination messages and incomplete rosbag export
are cleanup consequences of the launch failure, not the root cause.

## Artifact state

The attempt contains partial startup artifacts, but they are not acceptance
evidence:

- `fast_trial_summary.json` records `termination_reason=FAILURE` and return code
  1.
- `artifact_finalization.json` records `complete=false`.
- Required raw evidence is missing because the mission never became operational.
- `passive_rosbag_export.json` is incomplete.
- `navigation_action_replay.json` was not validly produced.
- The configured 600 s horizon was not reached.
- No offline analysis was run, so no scientific metric values are claimed.

## Minimum correction

Rebuild and reinstall `my_epuck_interfaces` from the current committed message
definitions, then rebuild/reinstall the dependent project packages in one
consistent clean install. The generated Python classes must expose the fields
used by the current installed project code before another launch is considered.

This requires an install/build correction, not a change to observer,
evaluator, Webots, ROS control, SLAM, Nav2, allocator, or scientific metric
source. No source correction was made in this task.

## Stop condition

No second 600 s run, no A/B/D run, no 1200 s run, no performance optimization,
and no offline evaluator execution was performed after the startup failure.

CONDITION_C_600S_CANONICAL_PIPELINE_FAIL
