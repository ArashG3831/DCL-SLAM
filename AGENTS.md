# Validation safety rules

This checkout is the only workspace authorized for Webots/ROS 2 validation.
The original checkout at `/home/arash/webots_ws` is dirty and must never be
sourced, built, or used as the runtime package prefix for this validation.

Before any ROS or Webots launch, read:

`docs/HOST_SAFETY_INCIDENT_2026-08-24.md`

Also read the authoritative threshold policy:

`docs/HOST_RESOURCE_STOP_POLICY.md`

The incident documented there is a hard preflight blocker. Never trust a run
merely because its command contains `--workspace`: `AMENT_PREFIX_PATH`,
`PYTHONPATH`, `CMAKE_PREFIX_PATH`, `COLCON_PREFIX_PATH`,
`RMW_IMPLEMENTATION`, and `CYCLONEDDS_URI` determine which code and middleware
actually run.

Required preflight checks:

```bash
env -i HOME=/home/arash PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin \
  bash --noprofile --norc -c '
    source /opt/ros/jazzy/setup.bash
    source /home/arash/webots_ws_clean_validation_20260823/install/setup.bash
    source /home/arash/webots_ws_clean_validation_20260823/scripts/ros2_wsl_cyclonedds_loopback.sh
    test "$RMW_IMPLEMENTATION" = rmw_cyclonedds_cpp
    test "$CYCLONEDDS_URI" = file:///home/arash/webots_ws_clean_validation_20260823/config/cyclonedds/wsl_loopback.xml
    test "$(ros2 pkg prefix my_epuck_project)" = /home/arash/webots_ws_clean_validation_20260823/install/my_epuck_project
    python3 -c "import my_epuck_project; print(my_epuck_project.__file__)"
  '
```

Do not launch if any provenance, process, port, or telemetry check fails. Do not
use Fast DDS as a silent fallback for this simulation. A full Windows reboot and
a clean baseline are required after a Windows nonpaged-pool incident.

The only automatic in-run resource stops are the explicit thresholds in
`docs/HOST_RESOURCE_STOP_POLICY.md`: Windows Pool Nonpaged Bytes at or above
6 GiB, or Windows RAM usage at or above 99%. WSL/vmmemWSL allocation, Linux
page cache, and a transient Available-MBytes value below an older heuristic are
informational unless one of those Windows thresholds is reached. Never invoke
`wsl --shutdown` merely to reduce a normal WSL cache/VM allocation.

Record WSL memory and Windows counters separately; WSL's reported values do not
replace the Windows counters, and Windows counters do not by themselves identify
a Linux process owner.

## Webots campaign time policy

Keep these two limits separate and record both.

### Emergency wall-clock watchdog

Every Webots run, including diagnostic, validation, and campaign runs, has a
hard maximum real execution time of **20 minutes (1200 wall-clock seconds)**.
This is an emergency safety ceiling for hung, deadlocked, or non-terminating
runs. It is not the scientific mission horizon. Fast mode may reach many
simulated seconds before this ceiling, but a run must never continue past the
wall-clock deadline while waiting for a simulation-time stop.

Enforce the ceiling with a monotonic-clock watchdog started immediately before
the launch wrapper is spawned. The watchdog must supervise the complete launch
process group, including Webots, ROS nodes, controller processes, and cleanup.
At 1200 wall-clock seconds it must mark `watchdog_termination=true`, record
monotonic wall elapsed time, host UTC, the latest Webots simulation time and
ROS `/clock` time, send SIGTERM to the process group, and send SIGKILL after
the documented short grace period if anything remains. The wrapper must not
wait for a simulation timeout or an unbounded post-processing step to terminate
the run. Post-run finalization and report generation are part of the same
wall-clock safety budget.

### Scientific simulation horizon

Every scientific campaign must also define an explicit simulated-time mission
horizon, for example `simulation_horizon_s=1500`. This is the scientific stop
condition, independent of wall-clock duration. The campaign is complete only
when Webots simulation time and ROS `/clock` both verify that the configured
simulation horizon was reached and observer/forensic finalization completed.
Fast mode changes the real duration required to reach the horizon; it does not
change the horizon itself.

When a task specifies a **20-minute simulation limit**, the horizon is exactly
`1200` simulated seconds. Do not interpret that instruction as 1200 wall-clock
seconds, and do not substitute a 1500-second horizon. The runner must stop from
a live Webots simulation clock or live ROS `/clock` at the configured horizon,
before post-processing or observer-file flushing. The configured horizon must
be printed and recorded in the effective command and run manifest before the
launch is accepted.

The normal stop target is the configured horizon `H`. A run is invalid if its
live simulation clock reaches `H + 60` simulated seconds before teardown; this
is an overrun fence, not extra scientific runtime. At that fence the launch
process group must be terminated immediately and the run recorded as
`FAILURE` with detail `SIM_HORIZON_OVERRUN`. Reports must never use samples
past `H`, and a 20-simulated-minute report must never include samples at or
past `1260` simulated seconds.

Cut coverage, travel, milestones, inactivity intervals, traffic metrics,
overlap metrics, and all other scientific measurements at the verified
simulation-time boundary. Never use wall time as a substitute for simulation
time. A clean horizon stop is `SIM_TIME_COMPLETE`; a watchdog stop before the
horizon is not a complete campaign and must be labeled `WALL_WATCHDOG`.

Every run manifest, summary, and report must record:

- simulation start time and simulation end time;
- ROS `/clock` start time and end time;
- observer elapsed time;
- monotonic wall-clock duration;
- host UTC start/end timestamps for provenance;
- configured simulation horizon;
- RTF, with its timebase and interval defined;
- termination reason, exactly one of `SIM_TIME_COMPLETE`, `WALL_WATCHDOG`,
  `FAILURE`, or `MANUAL_ABORT`;
- watchdog armed/deadline status and whether watchdog termination occurred;
- whether observer/forensic finalization completed.

If the watchdog fires, preserve all artifacts but do not claim a clean
full-horizon result. Existing and currently running experiments are not
discarded. For a run that exceeds the wall-clock limit, the usable scientific
prefix is still determined by the configured simulated-time horizon, not by a
wall-clock slice. For a 20-simulated-minute task, only records with verified
simulation time `<= 1200` may be used; any wall-clock prefix used for separate
diagnostics must be labeled non-scientific partial evidence. This rule is not
retroactive and does not delete or invalidate the historical artifacts
themselves.

## Thesis-runtime integrity rules

For thesis-critical runtime claims, never infer active ROS/Webots/Nav2 behavior
from comments, stale YAML files, historical artifacts, or unused configurations.
Prove the active value from the authoritative launch chain, runtime parameter
sources and overrides, live node parameters or live topics, and the exact
production modules used by the run.

Always distinguish production paths from obsolete, historical, and diagnostic
paths; configured defaults from runtime overrides; and physical sensor range
from SLAM usable range and Nav2 costmap processing range.

For experiment reruns, freeze and record the git commit, worktree state, world
hash, install path, and runtime provenance. Before making a thesis conclusion,
check for poisoning or configuration mismatches in active runtime settings.

Prefer existing project code, commands, scripts, reports, and conventions over
new ad hoc tooling. If a new command or feature is needed, inspect whether the
repository already has an appropriate command—especially `webotsreport`—and
extend it instead of creating a parallel top-level pipeline.

For ROS/Nav2/Webots tasks, do not claim that a component or parameter is used
without pointing to its active launch chain and runtime evidence. When multiple
parameter files exist, classify each as active, inactive, obsolete, or
diagnostic-only. For policy comparisons, ensure the only intentional difference
is the policy variable under study.

## Webots controller-network preflight

The Webots controller TCP connection and ROS DDS discovery are separate
validation concerns. Before every Webots launch, explicitly verify and record
the effective Webots networking mode and the resolved controller endpoint.

For the WSL NAT runtime:

- Set `MY_EPUCK_WEBOTS_NETWORK_MODE=nat` explicitly before launch.
- Resolve `WEBOTS_CONTROLLER_ENDPOINT` to the reachable Windows host gateway or
  interface, including its selected TCP port.
- Never use `127.0.0.1` for the Windows Webots server from WSL NAT mode. Linux
  loopback is not proof of reachability to the Windows host.

Do not confuse a CycloneDDS loopback profile with Webots controller
connectivity. `ROS_AUTOMATIC_DISCOVERY_RANGE`, `ROS_LOCALHOST_ONLY`, and the
CycloneDDS URI govern ROS discovery only; they do not prove that the Webots TCP
controller endpoint is reachable.

The launch must fail during preflight, before scientific data collection, if
any of the following holds:

- `WEBOTS_CONTROLLER_ENDPOINT` is missing;
- NAT mode is expected but the resolved endpoint is localhost;
- controller connectivity cannot be actively verified for the robot
  controllers and the Webots supervisor.

Every run manifest, launch provenance record, and final report must record:

- `MY_EPUCK_WEBOTS_NETWORK_MODE`;
- the resolved `WEBOTS_CONTROLLER_ENDPOINT` (host and port);
- the Webots controller connection result, including each required controller;
- whether ROS `/clock` advanced after controller connection.

This is a validation/preflight rule only. Do not add runtime workarounds or
new networking scripts to satisfy it.

## Custom-code skepticism

Do not treat existing custom project code as inherently correct simply because it already exists.

When investigating bugs, distinguish between:

1. established external frameworks/libraries with documented behavior;
2. custom project code written specifically for this project.

Custom code is a valid first suspect when:

- its behavior is undocumented;
- it duplicates responsibility already handled by a mature external system;
- it introduces stricter rules than the external system;
- it creates unexplained failures.

Before preserving or extending custom infrastructure, verify:

- why it exists;
- what documented requirement it satisfies;
- whether the underlying framework already provides the same functionality.

Do not solve bugs in custom layers by adding more custom layers unless the existing architecture and requirements justify it.

## Observer/evaluation completion specification

The single source of truth for observer/evaluation completion requirements is
`OBSERVER_EVALUATION_COMPLETION_SPEC.md`.

## Completed-task report handoff

After completing a full Codex task and providing its summary, if the task
created report/output artifacts such as Markdown, CSV, JSON, or similar files,
create a new uniquely named folder on the user's Windows Desktop and copy the
generated report files into it. Preserve the source files and do not overwrite
an existing Desktop folder. Copy only the task's report/output artifacts;
exclude build, install, log, ROS-log, temporary, raw simulation-output, and
other generated directories unless the user explicitly requests them. If the
Windows Desktop is not accessible, report the exact handoff failure instead of
silently skipping it.

## Legacy Observer Salvage / Incremental Migration Protocol

The old observer remains the behavioral reference while observer salvage is
being proven. The objective is the same scientific observer capability and
outputs with less mission-time work, no silent feature loss, and no duplicate
implementations.

1. Never perform multiple observer responsibility-family migrations in one
   unverified batch.

2. Change one logically isolated responsibility at a time. Examples include:
   synchronized map-frame processing; odometry trajectory/distance;
   coverage/ownership; protocol/event accounting; warning/rosout processing;
   health processing; and map conversion/counting.

3. Before each change:
   - identify the exact old behavior;
   - identify its inputs;
   - identify every output, artifact, and consumer;
   - identify exact timing and missing/zero/error semantics;
   - record the relevant baseline tests and parity evidence.

4. After each change:
   - run the narrowest tests/parity checks that fully exercise that
     responsibility;
   - compare old-versus-new observable outputs;
   - verify that no metric, artifact, evidence source, schema, timestamp
     semantic, fidelity, branch, or validity behavior was lost;
   - verify that production behavior outside that responsibility did not
     change.

5. Do not proceed to the next responsibility while the current one has failing
   tests, unexplained output differences, unproven parity, missing evidence,
   or unresolved semantic differences.

6. Commit each verified logical change separately. Each commit must contain
   only that logical change and its directly required tests. Never combine
   several observer migrations into one commit.

7. If a change breaks parity, diagnose that single change and fix or revert it.
   Do not pile additional changes on top of a broken state.

8. Keep checkpoints fine-grained enough that git history can answer exactly
   which change caused a behavior or performance regression.

9. Do not commit unrelated dirty-worktree files.

10. When a runtime-affecting migration is accepted, record before/after
    runtime evidence when practical so performance regressions can be
    attributed to the exact commit.

11. Prefer extending the old observer's existing passive rosbag capture, raw
    forensic evidence, deferred reconstruction, finalization/replay
    mechanisms, and metric primitives rather than creating a second
    independent evaluator.

12. Never remove live computation until its required raw inputs are preserved
    and its replacement path has demonstrated parity.

13. During migration, maintain one authoritative implementation of each
    scientific calculation wherever practical. Do not maintain separate live
    and offline algorithms that can drift; extract or reuse existing
    calculation logic when necessary.

14. The final goal is the same scientific observer capability and outputs,
    less mission-time work, no silent feature loss, and no duplicate
    implementations.
