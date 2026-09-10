# Validation safety rules

## CANONICAL THESIS EXPERIMENT LAUNCHER

All normal thesis Webots/ROS experiments MUST use:

```bash
python3 src/my_epuck_project/tools/run_thesis_experiment.py --condition C --horizon 180
python3 src/my_epuck_project/tools/run_thesis_experiment.py --condition C --horizon 600
python3 src/my_epuck_project/tools/run_thesis_experiment.py --condition C --horizon 1200
```

Never manually rebuild `env -i` launch commands or manually recreate NAT,
driver, `RUN_OUT`, ROS-library, ROS-domain, port, or experiment-runner
environment values. The launcher constructs one exact child environment,
validates that same environment, records it, and passes that same environment
to the existing runner. It preserves ROS Jazzy vendor libraries, WSL NAT,
`WSL_INTEROP`, the approved driver prefix, and the result root.

Historical manual launches failed because `LD_LIBRARY_PATH`, `RUN_OUT`, and
`MY_EPUCK_WEBOTS_NETWORK_MODE` were omitted in different runs. If the launcher
breaks, fix and test the launcher rather than bypassing it.

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

## ROS domain-range and retry preflight (hard rule)

Never launch a validation run using a ROS domain ID merely because it is
unique. Validate the requested `ROS_DOMAIN_ID` against the active CycloneDDS
profile before Webots or any scientific process starts. For the current
approved WSL loopback profile, the inclusive valid range ends at domain `230`;
domain `231` and any higher value must be rejected during preflight.

The domain check is mandatory for every run and every retry, including a retry
of the same acceptance run. Record the validated domain, active CycloneDDS
profile, and selected Webots port in the attempt provenance before launch.

If domain/port preflight rejects an attempt:

- classify it as a preflight failure, not an algorithm or experiment result;
- do not start Webots, ROS nodes, observers, or evidence collection;
- do not count it as an accepted run and do not overwrite or relabel an
  earlier accepted run;
- preserve the rejected attempt's command, reason, and provenance;
- retry only with a domain accepted by the active profile, a newly validated
  unused Webots port, and a new unique output directory.

Before claiming that a second acceptance run started, verify that the live
manifest/effective command contains the validated in-range domain and fresh
port and that launch/readiness markers and the result directory exist. A
message such as “retrying with domain 230” is not evidence that a simulation
started. The current incident—domain `231` rejected before any simulation or
evidence—must never recur.

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

## Canonical thesis world

For normal thesis development, validation, smoke tests, and experiments, use
the close-start profile `large_unknown_pose_close_start_20ms_scan_matching`
with world `epuck_d500_two_world_unknown_pose_close_start_dynamic_low_slip_20ms_finite.wbt`.
This is the close-start/common-overlap configuration with approximately 0.5 m
initial robot separation. Far-start, delayed-overlap, and legacy worlds must
not be selected by default or by filename/history inference; use another
world only when the USER explicitly requests it. Condition A uses the
robot1-only derived copy of this same environment.

## Authoritative experiment-plan correction (permanent)

These rules supersede any older ABCD, asynchronous-start, D/MRTSP, or 20-run
instructions in this repository, historical reports, or future handoff
messages.

### Synchronized two-robot start

For every two-robot experimental condition, especially B and C, both robots
may initialize while stationary, but neither may dispatch an exploration goal
or intentionally move before both robots are ready. Exploration must begin
from one common simulation-time `START_RELEASE` barrier, and both robots must
receive the exact same release timestamp. This is required because the
canonical starting geometry deliberately creates a symmetric narrow-doorway
traffic scenario; allowing one robot to start early invalidates that
experimental condition.

Record robot1-ready time, robot2-ready time, the authoritative
`START_RELEASE` time, each robot's first goal dispatch, first nonzero
`cmd_vel`, and first measurable motion. Do not fake simultaneity by delaying
logs or rewriting timestamps: gate the actual exploration behavior.

### Main thesis experiment is ABC only

- A = one-robot frontier exploration.
- B = two independent robots.
- C = two cooperative robots using the fixed cost-only strategy.

D/MRTSP is not part of the current main thesis experiment, validation
pipeline, long-horizon stress tests, videos, or final campaign. D may be
revisited only as an optional secondary experiment after ABC is completely
finished. With five seeds, the main final campaign is 15 runs, not 20.

### Validation order

Do not proceed to A/B, videos, or the final campaign while the C RTF gate is
blocked. After the 180-s C RTF gate passes, require full 1200-s long-horizon
validation before final-campaign work.

### Reproducible package closure and provenance

Audit the complete launch/runtime dependency closure in the appropriate ROS
`package.xml`, including the selected interfaces, frontier, cooperative, and
SLAM-wrapper packages. Fresh builds must derive their closure from the ROS/
colcon dependency graph, not from a manually remembered package list.

Before every simulation, source only ROS, the selected fresh thesis install,
and explicitly approved external Webots-driver prefixes. Fail closed if any
required project package resolves from the stale `/home/arash/webots_ws/install`
or another old project overlay. Record and verify every required `ros2 pkg
prefix`, including `my_epuck_interfaces`, `my_epuck_frontier_candidates`,
`my_epuck_cooperative_exploration`, and
`reliable_slam_toolbox_wrapper`.

### Absolute stale-driver fail-closed rule

Codex MUST NEVER source, launch, or accept any Webots/ROS driver or required
runtime package from `/home/arash/webots_ws/install`. Before Webots starts,
preflight MUST fail closed if that prefix appears in any effective environment,
effective command, launch provenance, package provenance, executable
resolution, or planned runtime path. The check must also fail closed if a
process or child-process command line/executable provenance from that prefix is
observed during startup. The only approved external Webots driver prefix is
`/home/arash/webots_ws_close_validation_2eb/install/webots_ros2_driver`;
project packages MUST resolve from the selected fresh thesis install. A direct
nested `ros2 launch` or another wrapper MUST NOT bypass these provenance checks.

### Clean-shell and clean-build contamination gate

Every ROS/Webots build and validation must start from a fresh environment, not
from a shell that previously sourced another ROS workspace. Use an `env -i`
shell with only the required base `PATH`/`HOME`, source `/opt/ros/jazzy`, then
source only the approved external Webots-driver package setup and the selected
fresh project install. Do not inherit or manually retain `AMENT_PREFIX_PATH`,
`CMAKE_PREFIX_PATH`, `COLCON_PREFIX_PATH`, `LD_LIBRARY_PATH`, `PYTHONPATH`, or
similar overlay variables from an earlier run.

Build every selected project dependency into a new uniquely named build and
install prefix. A build directory configured from a contaminated environment
must not be reused; quarantine or replace that exact cache before rebuilding.
The old `/home/arash/webots_ws/install` prefix is forbidden even as a lower
priority underlay. Sourcing the complete setup of an external workspace is
also forbidden when it adds old project packages; source the approved driver
package setup narrowly and verify package resolution explicitly.

Before launch, fail closed unless all of the following are true:

- CMake cache/configuration and generated build metadata contain no
  `/home/arash/webots_ws/install` reference;
- every required project package resolves from the selected fresh install;
- the Webots driver resolves only from the approved external prefix or the
  explicitly approved ROS underlay;
- every relevant executable and shared library passes `ldd` without resolving
  from the stale prefix, and its RPATH/RUNPATH contains no stale prefix;
- the effective environment, command line, launch provenance, and planned
  runtime paths contain no stale prefix.

### Launch-wrapper argument preflight

Before spawning any Webots/ROS process, validate the launch wrapper's own
arguments in the clean shell. In particular:

- Set `MY_EPUCK_INSTALL_PREFIX` explicitly to the selected isolated install and
  verify `ros2 pkg prefix my_epuck_project` equals it.
- Set `MY_EPUCK_WEBOTS_DRIVER_PREFIX` explicitly to the approved driver prefix
  and verify the driver provenance before launch.
- Choose a unique `ROS_DOMAIN_ID` and fail closed unless it is an integer in
  the active CycloneDDS profile's supported range `0..230`.
- Pass the absolute results directory into the `env -i` shell explicitly (for
  example as `RUN_OUT=...`); do not rely on an outer-shell variable that is not
  exported. Create it and verify it is writable before starting the pipeline.
- Require the log destination to be an absolute path below that validated
  directory and verify it can be opened before launch. A failed `tee`, missing
  output variable, invalid domain, or failed preflight is a pre-launch failure;
  it must not start Webots or any experiment process.
- Use `set -u`/equivalent unbound-variable failure in the wrapper and preserve
  the runner return code separately from the logging pipeline (for example via
  `PIPESTATUS`).

The minimum launch preflight must therefore validate, before Webots starts:

```bash
test -n "${RUN_OUT:-}" && test -d "$RUN_OUT" && test -w "$RUN_OUT"
test "${ROS_DOMAIN_ID:?}" -ge 0 && test "$ROS_DOMAIN_ID" -le 230
test "$(ros2 pkg prefix my_epuck_project)" = "$MY_EPUCK_INSTALL_PREFIX"
test -n "${MY_EPUCK_WEBOTS_DRIVER_PREFIX:-}"
```

Do not retry a rejected launch by guessing another argument. Correct the
preflight inputs, record the rejection, and only then launch once.

Record the exact clean-shell recipe, build/install/log prefixes, package
prefixes, executable paths, and dependency-scan result in run provenance. A
successful compile is not sufficient evidence of a clean build: CMake can
cache absolute dependency paths, while an inherited `LD_LIBRARY_PATH` can
override a fresh binary's RUNPATH at runtime. Any such contamination makes the
build/validation attempt invalid and it must not be used for scientific
results.

### Current hard acceptance requirements

The two-robot start barrier and ABC-only scope above are authoritative for the
current thesis stage. A reachable useful task must not be converted into
avoidable software idle because a peer is busy, cooperative evidence is slow,
certificates are pending, or a goal/assignment transition is in progress.
The existing cooperation protocol must establish peer unavailability before
degraded-solo continuation is permitted; it must not bypass cooperative
evaluation, commitments, certificates, duplicate exclusion, traffic, or
safety checks.

Active-simulation RTF is advancing ROS `/clock` divided by wall time while
Webots is running and must be at least 2.0x, with approximately 2.5x as the
target. This may not be achieved by reducing sensors, map/SLAM quality,
ground-truth/evaluator metrics, contact or other evidence fidelity, or
cooperative functionality. Investigate telemetry backlog and observer,
logger, serialization, executor, Nav2, and Webots bottlenecks when the gate
fails.

### Authoritative active RTF measurement

For diagnostic and acceptance reporting, measure RTF only over the active
simulation interval. The authoritative interval begins at the first strictly
advancing live simulation-clock sample after startup (ROS `/clock`, or the
equivalent live Webots Supervisor time) and ends at the final live sample at
the configured simulation horizon `H`. Use monotonic wall time for the same two
boundary events:

```text
active_simulated_seconds = sim_time_at_H - sim_time_at_first_advance
active_wall_seconds = monotonic_wall_at_H - monotonic_wall_at_first_advance
authoritative_RTF = active_simulated_seconds / active_wall_seconds
```

The first advancing-clock boundary excludes launch, controller connection,
readiness, and any period in which Webots has not yet advanced simulation
time. The horizon boundary excludes shutdown, recorder stopping, rosbag export,
offline evaluation, finalization, and other cleanup work. Never obtain this
metric by subtracting an assumed startup duration such as 60 seconds, by using
the whole runner wall duration, by using observer-process lifetime, or by using
clock-message count alone.

The runner must record both boundary timestamps and the clock source, together
with the configured horizon and termination reason. It must cut all scientific
metrics at `H`; samples after `H` may be retained only as teardown diagnostics
and must not enter the RTF or thesis result. Reports may additionally show the
raw observer-lifetime ratio and total launch wall duration, but those are
secondary diagnostics and must not be labeled authoritative active RTF.

## VALIDATED / PROTECTED SUBSYSTEMS — DO NOT REOPEN WITHOUT DIRECT EVIDENCE

The following project areas are validated/frozen. A later failure elsewhere is
not permission to redesign, retune, replace, or simplify them.

Before changing a protected subsystem, Codex must produce direct
runtime/source/artifact evidence that it is defective for the current failure,
state which validated invariant is contradicted, and choose the smallest
semantics-preserving correction. If the current requirement conflicts with a
protected invariant, stop and report the architectural conflict. Focused unit
tests alone do not justify replacing previously validated production behavior.

### Protected canonical simulation and sensing

- Profile: `large_unknown_pose_close_start_20ms_scan_matching`
- World: `src/my_epuck_project/worlds/epuck_d500_two_world_unknown_pose_close_start_dynamic_low_slip_20ms_finite.wbt`
- World SHA-256: `feda86d1c1ba8b3f1b18c8f216a079c61fff222dbda09b0326c68f8e0ef9bb86`
- Environment geometry SHA-256: `40f2a7c1982c0d1983b3e6dc260ebca8f80252efbdf17036299c08bfb0845a36`
- Webots basic timestep: 20 ms.
- Preserve the symmetric close-start narrow-doorway traffic geometry and
  physics/sensor fidelity; do not alter them to make validation easier or RTF
  faster.
- Preserve the D500 lidar, 12.0 m physical range, 720 beams, approximately
  11.98 m usable SLAM/Nav range, and 0.03 m map resolution.

### Protected SLAM, Nav2, frontier, fusion, traffic, and cooperation

- SLAM Toolbox parameters, scan matching, loop-closure policy, and map/sensor
  integration are closed absent direct SLAM evidence; do not retune them for
  unrelated frontier, startup, logging, or RTF failures.
- Nav2 lifecycle/readiness, MODE_B costmap interpretation, current path/goal
  validity, and Nav2 success authority are protected. Do not restore arbitrary
  endpoint-distance rejection or reconfigure active lifecycle nodes without
  direct evidence.
- Stable physical frontier/task identity, canonicalization, Tier-1 duplicate
  suppression, source-aware map fusion, snapshot-time teammate-pose freshness,
  and existing ghost-map corrections are protected.
- Traffic semantics are protected: continuous path-segment conflicts, roughly
  0.16 m minimum separation, roughly 0.13 m/s ETA logic, deterministic tie
  handling, no conflicting goal for a waiting robot, and fresh allocation after
  conflicts clear. Do not weaken traffic coordination to make robots move.
- C remains `frontier_cost_only` + `MODE_B`; preserve two-active cardinality,
  reservations, agreements, certificates, continuation semantics, serialized
  planner safety, traffic, and safety gates. Never dispatch an otherwise
  disallowed local task merely to improve idle metrics.

### Protected unknown-pose estimator

The existing full occupancy-map bounded SE(2) estimator is validated and
frozen. Preserve its refinement, existing quality thresholds, bidirectional
validation, reciprocal peer verification, canonical T21 convention, immutable
accepted transform, and startup-only discovery/quiescence.

Do not replace it or introduce a second transform estimator merely because the
stationary `START_RELEASE` requirement makes evidence accumulation difficult.
The active issue is the evidence/acceptance contract, not evidence that the
validated estimator is defective. If stationary independent evidence is
incompatible with the frozen estimator contract, stop and report that conflict
instead of inventing another estimator or evidence family.

### Validated stationary registration and common release

The stationary `stationary-disjoint-partition-v1` evidence path is validated
and frozen for the canonical close-start C startup: the original whole-map
bounded SE(2) estimator supplies the canonical seed, three genuinely disjoint
support partitions are reconstructed and peer-verified 3/3, and the accepted
handoff is required before shared-stack activation. The common simulation-time
`START_RELEASE` barrier is also frozen: both robots must be ready, traffic
must be ready, exploration dispatch and intentional motion before release are
forbidden, and both robots receive the same release timestamp. Do not lower
the witness count, reuse overlapping evidence, bypass reciprocal verification,
or move the release barrier to compensate for a later failure.

### Protected runtime, time, and evaluation semantics

- Preserve WSL NAT, bounded unique ROS domain IDs, unique Webots ports,
  filtered provenance, and the approved external driver prefix
  `/home/arash/webots_ws_close_validation_2eb/install/webots_ros2_driver`.
- Project packages must resolve from the selected fresh install, never the
  stale `/home/arash/webots_ws/install`; runtime dependency closure belongs in
  `package.xml` and must be derived through colcon.
- Keep simulated time and wall time separate: 180 s smoke, exactly 1200 s final
  mission, 1200 s outer wall watchdog, and normal stop `SIM_TIME_COMPLETE`.
- Preserve GT-as-evaluation-only, authoritative physical-contact evidence,
  coverage/evaluator definitions, artifact schemas, and full evidence fidelity.
  Passive logging implementation may still be optimized for RTF only when
  outputs and semantics remain equivalent.

### Active and not yet frozen

Do not label these complete from a single smoke: current RTF work, full
1200-s C snappiness/scaling, B under the common barrier, or the final ABC
campaign snapshot.

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

## Offline DCL-SLAM renderer

The renderer is:

`src/my_epuck_project/tools/render_cooperative_animation.py`

It is an offline-only visualization tool. It does not launch ROS, Webots, or
the observer. Given a finalized experiment campaign, it renders the shared
map, robot trajectories, recorded frontier regions, candidate/goal overlays,
planned paths, phase and handoff information, and per-robot telemetry. The
sidebar can show position, heading, current goal, linear speed, turn rate,
distance travelled, productivity, traffic state, reachable-candidate count,
candidate-batch age, and issue counts. It also writes a provenance JSON and
optional still frames.

### Input

Pass the finalized `fast_trial` directory with `--campaign`. The renderer
locates the nested observer directory and reads the recorded pose/timeseries,
events, candidate batches, task snapshots, goals, paths, map snapshots, and
frontier geometry. Frontier geometry is taken from
`observer/frontier_regions.jsonl` when present. If that raw geometry is absent,
the renderer falls back to recorded candidate bounds, centroids, and approach
points; it cannot reconstruct raw frontier polygons from event metadata alone.

### Canonical 1x render

From the workspace root:

```bash
python3 src/my_epuck_project/tools/render_cooperative_animation.py \
  --campaign results/thesis_condition_C_180s_20260908T222732Z/fast_trial_20260908T222737Z \
  --output results/thesis_condition_C_180s_20260908T222737Z/render_1x/condition_C_180s_DCL_SLAM_1x.mp4 \
  --start-s 22.04 --duration-s 180 --speedup 1 \
  --title "DCL-SLAM" --policy "Cooperative Cost-Only" \
  --provenance-output results/thesis_condition_C_180s_20260908T222737Z/render_1x/condition_C_180s_DCL_SLAM_1x.provenance.json \
  --stills-dir results/thesis_condition_C_180s_20260908T222737Z/render_1x/stills \
  --still-times 0,30,90,150
```

`--start-s` is the absolute source simulation timestamp at the first frame.
`--duration-s` is the absolute source end timestamp in the current renderer
workflow, so the example covers source time 22.04 through 180 s. `--speedup`
is source simulation seconds per output second: use `1` for real-time video
and `10` for a 10x accelerated presentation. For example, the same source
window at `--speedup 10` is approximately one tenth as long.

Useful presentation options are `--stills-only`, `--hide-frontiers`,
`--hide-goals`, `--hide-candidates`, `--hide-planned-path`, `--width`,
`--height`, `--fps`, and `--still-times`.

### Map snapshot timing

The `MAP SNAPSHOT` value is the timestamp of the latest recorded forensic map
image at or before the current video frame. The renderer does not lazily skip
available data. Raw map receipts can be frequent, but full occupancy-map image
snapshots are recorded less often and are deduplicated in the forensic map
directory. Consequently the displayed map timestamp can advance in roughly
5–15 second jumps. Smoother map animation requires recording more full map
images; receipt metadata alone is not enough to reconstruct the occupancy
image offline.

### Validation

Run the renderer's focused tests with:

```bash
PYTHONPATH=src/my_epuck_project python3 -m pytest -q \
  src/my_epuck_project/test/test_render_cooperative_animation.py
```

The renderer is presentation-only: changing its labels, speedup, frame range,
or visibility options does not change the experiment artifact or scientific
metrics.

## Reusable offline run report

For a finalized `fast_trial` artifact, use
`src/my_epuck_project/tools/generate_offline_run_report.py` to collect the
standard observer, timing, frontier, candidate, query, certificate, allocator,
navigation, map/TF, coverage, cooperation, mission, provenance, and
finalization metrics without ROS or Webots. It reuses
`offline_timing_metrics.analyze_event_records` for the authoritative
feasible/productive/avoidable-idle definition, keeps raw lifecycle records
separate from request-level joins, and emits `null` when a metric is not
recorded rather than guessing.

```bash
python3 src/my_epuck_project/tools/generate_offline_run_report.py \
  --artifact results/.../fast_trial_... \
  --output-json results/.../offline_metrics.json \
  --output-md results/.../offline_report.md
```

Add `--baseline <other-fast-trial>` for the deterministic scalar comparison
table. Use the finalized artifact directory, not a raw live-run directory; the
tool treats the fast-trial simulation end as the measurement cutoff so
post-horizon finalization callbacks are excluded. It is standalone reporting
only and is not invoked automatically by the experiment runner. After each
successful invocation it also creates a new uniquely named folder on the
accessible Windows Desktop and copies only the generated JSON and Markdown
reports into it, then opens that folder in Windows Explorer. Set
`CODEX_WINDOWS_DESKTOP` or pass `--desktop-root` when the Desktop is mounted at
a nonstandard WSL path; an unavailable Desktop is a visible command failure,
not a silent skip. If Explorer cannot be launched, the reports remain copied
and the command reports the handoff/open status explicitly.
