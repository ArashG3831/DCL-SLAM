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
