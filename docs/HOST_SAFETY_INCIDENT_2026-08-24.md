# Host-safety incident: Windows NETIO pool growth

## Status

This is a recorded incident and a mandatory preflight reference. It is not a
claim that the underlying Windows/WSL network-buffer behavior is fixed.
No further Webots or ROS campaign may start until the host has been rebooted
and the transport provenance checks below pass.

## What happened

During a two-robot ROS 2/Webots campaign, Windows kernel networking counters
grew catastrophically:

| Counter | Observed peak/latest report |
|---|---:|
| Nonpaged pool | approximately 13.4--13.75 GB |
| `Nbuf` | approximately 8.23 GB |
| `Nnbl` | approximately 1.92 GB |
| `Nnbf` | approximately 997 MB |
| Windows available memory | approximately 8--72 MB |

The WSL guest subsequently reported several GiB available. That was not a
healthy-host indication: the allocation was in the Windows kernel pool and
survived WSL restart. Windows counters later measured approximately
13,747,933,184 bytes of nonpaged pool and only 8--186 MB available during a
read-only sample. A full Windows reboot is required to reclaim it.

No Webots or ROS campaign process remained after the observed run, so ordinary
Python process RSS is not the explanation for this allocation.

## Root-cause findings

The earlier changes addressed only one trigger:

* `b2a6d19` constrained the clean runner's domain range to 0--230 using the
  current loopback profile's assumed CycloneDDS port arithmetic.
* `741c703` attempted to fail fast on the exact log text
  `ddsi_udp_conn_write ...:65536`.

They did **not** establish that the actual campaign used that runner or that
it used CycloneDDS.

### Proven workspace/middleware bypass

The shell in which the incident was investigated had all of these paths from
the original dirty checkout:

```text
AMENT_PREFIX_PATH=/home/arash/webots_ws/...
PYTHONPATH=/home/arash/webots_ws/build/...
COLCON_PREFIX_PATH=/home/arash/webots_ws/...
```

It also had no `RMW_IMPLEMENTATION` and no `CYCLONEDDS_URI`. In that shell:

```text
ros2 pkg prefix my_epuck_project
  -> /home/arash/webots_ws/install/my_epuck_project

import my_epuck_project.cooperative_regression
  -> /home/arash/webots_ws/build/my_epuck_project/...

ROS_DOMAIN_MAX = 232
```

The clean, sanitized shell instead resolves:

```text
RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
CYCLONEDDS_URI=file:///home/arash/webots_ws_clean_validation_20260823/config/cyclonedds/wsl_loopback.xml
ros2 pkg prefix my_epuck_project
  -> /home/arash/webots_ws_clean_validation_20260823/install/my_epuck_project
ROS_DOMAIN_MAX = 230
```

Therefore a command containing `--workspace
/home/arash/webots_ws_clean_validation_20260823` could still execute the old
dirty package and old domain guard if it was launched from the contaminated
shell. The workspace argument alone does not repair the process environment.

### Guard gaps still present at the time of the incident

The clean runner had these unsafe gaps:

1. `cooperative_regression.py` silently selected `rmw_fastrtps_cpp` whenever
   `RMW_IMPLEMENTATION` was unset. The simulation was not fail-closed on
   CycloneDDS.
2. The runner did not require or record that `CYCLONEDDS_URI` pointed to the
   clean WSL loopback XML profile.
3. `abnormal_ros_exit_evidence()` polled only the top-level `launch.log`;
   DDS errors written to per-node files under `ros_logs/` could be missed.
4. The invalid-endpoint pattern matched only port `65536`, not every UDP port
   greater than `65535`.
5. The campaign's normal retry policy could relaunch an infrastructure attempt
   unless `--no-infrastructure-retry` was supplied.

These gaps explain why the domain-headroom fix could be correct in source and
still fail to protect an incorrectly sourced or differently configured run.

### Independent remaining mechanism

Even with a safe domain, the earlier small DDS churn test was not equivalent to
the full workload. Accelerated Webots plus two robots and reliable/full sensor
traffic can exercise the WSL2/Hyper-V/Windows NETIO path at a much higher rate.
The bounded churn test therefore could not prove that the full sensor workload
was safe. The invalid RTPS endpoint was a catastrophic amplifier, not proof
that all valid traffic is safe.

The available evidence identifies Windows network buffering as the allocation
site, but does not identify the exact owning NDIS/filter/Cyclone component.
Pool-tag attribution was not captured, so do not claim more specific ownership.

## Mandatory future launch procedure

1. Reboot Windows after any nonpaged-pool incident.
2. Record Windows `LastBootUpTime`, `Pool Nonpaged Bytes`, available memory,
   paging, and WSL boot ID before starting ROS.
3. Start from `env -i`; do not inherit a shell sourced from
   `/home/arash/webots_ws`.
4. Source only `/opt/ros/jazzy`, the clean validation install, and the clean
   loopback Cyclone script.
5. Require and record:

   ```text
   RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
   CYCLONEDDS_URI=file:///home/arash/webots_ws_clean_validation_20260823/config/cyclonedds/wsl_loopback.xml
   ros2 pkg prefix my_epuck_project = clean install prefix
   ```

6. Reject domains above the profile-derived safe bound before any ROS graph
   probe or launch. Record the parsed profile hash and effective maximum port.
7. Treat every UDP endpoint greater than 65535, DDS assertion, SIGABRT, or
   SIGSEGV as fatal. Scan the main launch log and every campaign-owned node log.
8. Disable automatic retries after a fatal transport event.
9. Run no-Webots DDS tests first, then a one-robot realtime transport test.
   Do not run fast two-robot exploration until that test is bounded.
10. Preserve the full command, environment manifest, PID tree, and Windows
    counter timeline in the result directory.

## Evidence limitations

The exact latest catastrophic run's `runner_metadata.json` and Windows pool-tag
capture were not present in this checkout, so it is not possible to prove from
the local artifacts whether that run used the clean or dirty package, or
whether it emitted `:65536` versus another invalid endpoint. The environment
audit above proves that the bypass was possible and active in the investigative
shell. Do not label the latest run as a clean-run regression until its runtime
provenance is captured.

## Follow-up incident: valid-domain clean run (2026-08-25)

The bounded handoff attempt `gate_b_handoff_20260825_0815` was stopped after
the host again entered the catastrophic state.  This attempt has complete
runtime provenance and therefore separates the remaining mechanism from the
old invalid-port trigger:

* source/install commit: `635b66755e43c52b7d8b57204e4ab460c629c056`;
* clean validation install and build tree were used;
* `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`;
* `CYCLONEDDS_URI` was the clean WSL loopback profile;
* ROS domain `46` and Webots port `23092` (both below the safe bound);
* Webots fast mode, two robots, full reliable sensor traffic;
* no `ddsi_udp_conn_write ...:65536` text, DDS assertion, SIGABRT, or
  active-runtime SIGSEGV was found in the attempt logs.

The campaign's `process_metrics.csv` measured WSL process-tree RSS rising to
approximately 3.77 GB while WSL available memory fell from approximately
6.995 GB to 4.779 GB.  After the exact campaign process tree was terminated,
WSL recovered to approximately 6.6 GB available and no campaign process or
Webots port remained.  A Windows counter query after teardown still reported:

```text
Pool Nonpaged Bytes = 13,654,495,232 bytes (about 13.65 GB)
Available MBytes    = 19 MB
Pool Paged Bytes     = 497,905,664 bytes
```

Thus this attempt reproduces the Windows kernel-pool failure with a valid
CycloneDDS domain and no invalid endpoint in the captured logs.  The map-fusion
timestamp change in commit `635b667` cannot account for Windows network-pool
allocation; it changes only ROS-side TF lookup timing.  The remaining trigger
is therefore the accelerated two-robot reliable/full-sensor ROS traffic path
through WSL2/Hyper-V/Windows networking (NETIO/NDIS/filter ownership remains
unidentified).  The earlier domain-headroom and `:65536` fail-fast changes are
still necessary, but they are not a complete fix for this valid-traffic path.

No Windows nonpaged-pool baseline was captured immediately before this
attempt, so the exact byte delta attributable to this individual run cannot be
computed from the saved artifacts.  The post-teardown counter is nevertheless
conclusive that the allocation is outside campaign process RSS and survives
ROS/Webots cleanup.  A full Windows reboot is required before any further
Webots launch; do not use WSL `free` alone as a host-safety signal.

## Required source hardening before resuming validation

The next code change must be narrow and fail-closed:

* reject any non-Cyclone `RMW_IMPLEMENTATION` for this simulation;
* require the clean loopback profile and record its hash;
* scan all campaign-owned logs for any invalid UDP port, not only `65536` in
  `launch.log`;
* stop without infrastructure retry after a fatal transport event.

These changes must be tested offline before any Webots process is started.
