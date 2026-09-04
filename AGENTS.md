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
discarded. For a run that exceeds the wall-clock limit, only its verified first
1200 wall-clock seconds may be used as bounded prefix/partial evidence, with
metrics still cut by their verified simulation timestamps and the result
explicitly labeled partial. This rule is not retroactive and does not delete
or invalidate the historical artifacts themselves.

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

For this WSL/Webots NAT runtime, the external-controller endpoint is not the
DDS loopback endpoint. Any Webots launch must explicitly set
`MY_EPUCK_WEBOTS_NETWORK_MODE=nat` (unless a measured mirrored-network setup is
being used), and the launch log must show the resolved WSL default-route
gateway in `WEBOTS_CONTROLLER_ENDPOINT`. Sourcing the CycloneDDS loopback
profile alone is not sufficient evidence for Webots TCP connectivity.

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
