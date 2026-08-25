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

### Leading source-level candidate (not yet runtime-proven)

The failed run's recorded launch command contained, for both Webots drivers:

```text
qos_overrides./scan_d500.publisher.reliability:=reliable
```

The authoritative simulation profile also currently sets
`scan_input_reliability: reliable`, and `d500_scan_fix.py` unconditionally uses
`RAW_SCAN_QOS` with reliable reliability for its raw-scan subscription. This
means the intended best-effort sensor-data path is overridden before the first
720-beam scan crosses the Webots/WSL boundary. The raw stream is then copied
through reliable DDS endpoints before the bounded corrected stream is produced.

This is a credible explanation for why the invalid-port guard did not help:
domain 46 is valid, but reliable fragmented sensor traffic can still exercise a
WSL2/Hyper-V/Windows NETIO buffer/retransmission path. It is not yet proof of
ownership or causality; the required A/B is:

1. reboot Windows and record a healthy nonpaged-pool baseline;
2. run one robot in realtime with the current raw reliable path;
3. run the identical one-robot test with raw `/scan_d500` best-effort, depth 1,
   and a parameter-selected best-effort `d500_scan_fix` subscription;
4. compare Windows nonpaged-pool delta, Nbuf/Nnbl/Nnbf (if PoolMon is
   available), scan receipt/correction counts, and SLAM quality;
5. only then test two robots and fast mode.

The corrected `/scan_d500_fixed` stream can remain reliable at its bounded
1-Hz cadence initially; do not change map, consensus, or estimator semantics
in the same experiment. If the raw-scan A/B is safe but the full campaign is
not, isolate the next largest reliable payload separately (the transient-local
`PeerMap`/occupancy-map stream) rather than changing all QoS policies at once.

## Required source hardening before resuming validation

The next code change must be narrow and fail-closed:

* reject any non-Cyclone `RMW_IMPLEMENTATION` for this simulation;
* require the clean loopback profile and record its hash;
* scan all campaign-owned logs for any invalid UDP port, not only `65536` in
  `launch.log`;
* stop without infrastructure retry after a fatal transport event.

These changes must be tested offline before any Webots process is started.

## Bounded transport diagnostics after Windows reboot (2026-08-25)

After a full Windows reboot, the initial host baseline was healthy:

```text
Windows available memory: approximately 7.1 GB
Pool Nonpaged Bytes:      approximately 868 MB
Pool Paged Bytes:         approximately 411 MB
WSL available memory:     approximately 6.8 GiB
WSL swap:                 unused
```

The raw-scan relay had a real parameterization defect. The node declared
`input_reliability` but always passed the hard-coded reliable `RAW_SCAN_QOS`
to `create_subscription`. Commit `58a2af3` adds an explicit `raw_scan_qos()`
selector and a depth-1 best-effort diagnostic profile; the focused
scan/teardown/profile suite passed 25 tests.

The corrected no-Webots CycloneDDS churn probe completed 120/120 child
processes with zero assertions, aborts, invalid endpoint messages, or leaks.
The monitored run's Windows nonpaged pool rose from approximately 901 MB to
919 MB and stabilized after teardown. This is a small bounded residual, not
the earlier multi-gigabyte slope. The artifacts are preserved under
`results/transport_gate_dds_churn_20260825/`.

The first Webots transport gate was then aborted before Webots startup because
the host watchdog observed only approximately 1.9 GB Windows-available memory
at launch. Inspection showed `vmmemWSL` holding approximately 7.56 GB while
WSL reported approximately 6.7 GiB available internally; the WSL filesystem
page cache had grown to approximately 6.7 GiB after the repeated child-process
probe. Windows nonpaged pool was still approximately 923 MB. This was host
commit pressure from the WSL VM/page cache, not proof of a new NETIO pool leak.

One earlier churn attempt was not valid host-safety evidence because its
sanitized `PATH` could not find PowerShell; its watchdog recorded
`FileNotFoundError` rather than Windows counters. It is retained only as a
harness-failure artifact. The watchdog now uses the absolute Windows
PowerShell path and must check both Windows counters and WSL/vmmem pressure.

Do not launch Webots while Windows available memory is below the measured
safety floor, even if `free -h` inside WSL looks healthy. Reclaim WSL VM/cache
memory (normally by a controlled `wsl --shutdown` or Windows restart), record
a fresh baseline, and rerun the one-robot realtime transport gate before any
dual-relay or fast-mode test.

## WSL fusion launch failure (2026-08-25)

The diagnostic CPU-quota prefix previously wrapped the pre-handoff
`source_aware_map_fusion` processes unconditionally with:

```text
systemd-run --user --scope --quiet -p CPUQuota=30%
```

The validation WSL image provides the `systemd-run` executable but no usable
user systemd bus (`/run/systemd/system` and `DBUS_SESSION_BUS_ADDRESS` are not
available).  Each fusion process therefore exited immediately with:
`Failed to connect to bus: No medium found`.  The rest of the launch continued,
making this look like a handoff/map-fusion failure and leaving shared-map
evidence unavailable.

The launch now probes the exact user scope command before using it.  If the
probe fails, fusion starts without that optional wrapper (the campaign-level
`nice`/CPU affinity still applies); it never makes map-fusion startup depend on
an absent service manager.  The fallback and live-bus paths are covered by
focused tests.  A fresh-install smoke on 2026-08-25 showed both fusion PIDs
alive with `FUSION_PHASE pre_handoff=true`, and both exited cleanly during
teardown.  Future validation must use the fresh install and must check the
fusion PIDs before declaring a handoff/shared-map gate passed.

The external nonpaged-pool guard also has a cross-session cleanup requirement:
the campaign supervisor intentionally creates new sessions for its internal
trial and ROS launch children.  Killing only the guard's original process
group can therefore leave those descendants running.  The guard now
enumerates the exact `psutil` descendant tree and terminates it across
session/process-group boundaries, followed by bounded kill escalation.  A
synthetic `setsid` child test confirmed that no descendant survives.  Every
guarded Webots run must still perform an exact campaign-path process/port audit
after termination; a pool-triggered stop is not considered clean merely
because the top-level PID exited.

## Valid-domain fast-campaign pool evidence (2026-08-25)

The guarded fresh-install run `robust_handoff_fusion_fixed_fresh_20260825_long3`
used CycloneDDS domain 68 (not the invalid 231/232 domains), port 23117, two
robots, Fast Webots mode, and the full reliable scan path.  Windows
`Pool Nonpaged Bytes` rose from 2,236 MB at launch to 3,675 MB before the
guard stopped the process group.  The exact campaign descendants were then
terminated manually because the pre-fix external guard did not cross the
supervisor-created sessions.  A subsequent PoolMon snapshot identified the
retained network-buffer tags:

```text
Nbuf  1,923,020,784 bytes   1,036,111 outstanding allocations
Nnbl    449,729,008 bytes   1,041,030 outstanding allocations
Nnbf    233,194,752 bytes   1,041,029 outstanding allocations
```

The exact PoolMon files are on the Windows desktop as
`pool_snapshot_after_wsl_shutdown_20260825.log` and
`pool_snapshot_idle_after_wsl_shutdown_20260825.log`; after 30 seconds idle,
the three byte totals were unchanged.  This run produced no `:65536` endpoint
messages and no CycloneDDS assertion, so the invalid-port guard was not the
complete fix.  The evidence instead shows that valid accelerated reliable DDS
traffic is sufficient to leave Windows NETIO/WSL networking buffers retained.
The recent traffic increase came from the scan-transport history: commit
`919ec85` introduced a reliable Slam Toolbox scan transport, while `b4d2696`
added a separate reliable Nav2 scan relay; `2248a3a` then adjusted CycloneDDS
fragmentation.  These changes are the correct isolation boundary for the
remaining Windows/WSL kernel-pool behavior.  A Windows reboot is required to
clear this kernel pool before another Webots campaign; WSL shutdown alone did
not reclaim the Nbuf/Nnbl/Nnbf allocations.

## Valid-domain upstream-core gate stop (2026-08-25)

The guarded run `upstream_core_gate_b_fast_20260825` used CycloneDDS domain 37,
Webots port 23143, Fast Webots mode, two robots, reliable full scan transport,
and the upstream-core frontier adapter.  It reached `/clock`, local Nav2
readiness, upstream frontier snapshots on both robots, three compatible
unknown-pose constraints, peer-confirmed evidence hash
`3f8b1c3b0346eadf`, one canonical handoff, and post-handoff shared fusion and
allocation.  It was stopped at 296.66 simulated seconds by the Windows pool
watchdog, not by a DDS assertion or ROS crash.  The watchdog samples crossed
the hard 6,000 MB ceiling:

```text
5831.1, 5856.7, 5881.3, 5905.4, 5932.4, 5953.6, 5973.0,
5995.6, 6021.3 MB (Pool Nonpaged Bytes)
```

No `SIGABRT`, `SIGSEGV`, CycloneDDS assertion, or physics-step failure was
observed.  WSL available memory was approximately 4.3 GB during the run and
returned to approximately 6.8 GB after exact campaign-process termination, but
the Windows nonpaged pool remained above the safety ceiling (6,369.3 MB at a
later read).  The final shared-map arrays were not equal because the watchdog
interrupted active map updates; this is not a valid shared-map pass.

The first watchdog stop exposed a teardown race: sending SIGTERM to the outer
campaign supervisor bypassed the KeyboardInterrupt-only descendant cleanup,
leaving reparented ROS/Webots sessions alive.  Commits `56419aa` and `9da137c`
install a temporary SIGTERM handler that enters the same exact campaign-path
cleanup and add a regression test.  The stopped run used the pre-fix installed
Python package, so its manual cleanup does not count as a normal cleanup pass.
Do not launch another Webots campaign until Windows has been rebooted, the
nonpaged-pool baseline is healthy, and the corrected install is proven to reap
all campaign-owned processes.

## Valid-domain A* smoke residual pool (2026-08-25)

The corrected upstream-core smoke `upstream_core_gate_b_astar_65640f6_20260825`
used the clean validation install, CycloneDDS loopback domain 50, Webots port
23155, the dynamic 20 ms close-start world, reliable full scan transport, and
`use_astar=true` in both active shared-map NavFn profiles.  It completed its
300-second bounded mission with exact process/port/file cleanup and no DDS
assertion, SIGABRT, SIGSEGV, or physics-step failure.

Windows `Pool Nonpaged Bytes` nevertheless rose from approximately 4,396 MB
before launch to 5,611 MB by shutdown.  After the campaign and WSL VM were
stopped, it remained approximately 5,618 MB for at least two minutes while no
campaign process remained.  WSL guest memory returned to approximately 6.9 GB
available, and Windows free memory recovered, but the Windows nonpaged pool did
not.  This is residual host kernel allocation associated with the valid DDS /
Webots transport workload; it is not evidence of an application RSS leak being
fixed by A*.

The agreed 6,000 MB hard ceiling therefore leaves insufficient headroom for a
long-navigation soak.  Do not launch the soak or final campaign from this
baseline.  Reboot Windows, record a clean pool baseline, and repeat the
bounded transport gate before continuing.

## Why the earlier transport fix did not fix the leak (2026-08-25)

The current evidence does not show an active application-RSS leak.  After the
last campaign, the exact campaign process audit was empty and WSL reported
approximately 6.7 GiB available.  Windows `Pool Nonpaged Bytes` nevertheless
remained at approximately 5,616--5,617 MB over idle samples.  This is retained
Windows kernel-pool state, not a currently running ROS process.  WSL shutdown
can reclaim the guest VM and `vmmemWSL`; it does not reclaim these Windows
`Nbuf`/`Nnbl`/`Nnbf` allocations.  A Windows reboot is required for a clean
baseline.

The source-level change in `58a2af3` was narrower than the campaign workload:
it made the raw-scan subscription honor `input_reliability`, but it did not
change the authoritative campaign default.  The current A* campaign manifest
and launch log prove:

```text
scan_input_reliability:=reliable
qos_overrides./scan_d500.publisher.reliability:=reliable
d500_scan_fix input_reliability=reliable
RAW_SCAN_QOS: KEEP_LAST depth=100, RELIABLE
CORRECTED_SCAN_QOS: KEEP_LAST depth=100, RELIABLE
Nav2 /scan_d500_nav: KEEP_LAST depth=1, BEST_EFFORT
RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ROS_DOMAIN_ID=50
CYCLONEDDS_URI=file:///home/arash/webots_ws_clean_validation_20260823/config/cyclonedds/wsl_loopback.xml
```

Thus the fix removed a hard-coded selector bug but the full proof campaign
continued to exercise the same reliable raw/full-sensor transport path.  The
recent transport history shows why this is a real regression boundary:
`919ec85` changed Slam Toolbox scan transport to reliable, `b4d2696` added a
second Nav2 scan relay, and `2248a3a` changed CycloneDDS fragmentation.  These
changes post-date the original project lidar path and increase the number of
reliable fragmented DDS streams; the lidar geometry itself was not changed by
the leak guard.

The available PoolMon snapshots identify the allocation site but not an owning
PID or a unique driver:

```text
Nbuf  1,923,020,784 bytes  (1,036,111 outstanding)  after valid fast traffic
Nnbl    449,729,008 bytes  (1,041,030 outstanding)
Nnbf    233,194,752 bytes  (1,041,029 outstanding)
```

The same tag family remained essentially unchanged after WSL shutdown.  The
Windows binding inventory contains Hyper-V/WSL, VMware, TAP/Outline, and
industrial Ethernet filter drivers, so the evidence cannot honestly name
`CycloneDDS`, `WSL`, or one NDIS filter as the sole owner without a captured
ETW/PoolMon stack trace.  The strongest proven statement is: valid accelerated
ROS 2 UDP traffic from the Webots workload causes Windows NETIO/NDIS buffer
retention; the invalid-domain/port bug was a catastrophic amplifier, not the
complete cause.

Short fresh transport probes are not equivalent to the full workload.  The
recorded one-robot reliable probe increased pool by less than 1 MB, the
one-robot best-effort probe by roughly 2--3 MB, and the short two-robot fast
probe by roughly 10 MB.  Those bounded results neither reproduce nor disprove
the multi-gigabyte slope seen after full SLAM/Nav2/map-fusion traffic.  A
clean post-reboot A/B must therefore isolate raw scan QoS, corrected scan QoS,
and large reliable map/fusion topics one variable at a time before any long
campaign.

## Correction: the realtime handoff incident was not Fast Webots (2026-08-25)

The incident referred to as `upstream_core_gate_b_handoff_20260825` used
`fast_mode=false` and Webots `--mode=realtime`.  Its manifest records source
commit `fb20df3`, CycloneDDS domain 36, the full sensor profile, and the
default `scan_input_reliability:=reliable`.  It ran for approximately 280
wall seconds with 56 campaign processes; WSL process-tree RSS reached about
2.8 GB and WSL available memory fell from about 7.0 GB to 5.6 GB.  That run's
artifact did not capture Windows pool counters, so its individual nonpaged
delta cannot be computed from the repository alone.

The later `upstream_core_gate_b_fast_20260825` and
`upstream_core_gate_b_astar_65640f6_20260825` runs were separate Fast Webots
runs.  Their evidence must not be substituted for the realtime incident.
Therefore the defensible conclusion for the realtime event is not “Fast
Webots caused it.”  The common recent transport boundary is instead the
combination of duplicated raw-scan subscriptions (full-resolution Slam plus a
separate Nav2 relay), reliable DDS delivery, the WSL loopback/fragmentation
profile, and the complete two-robot ROS graph.  The exact contribution of
those mechanisms remains unseparated until a clean post-reboot A/B captures
Windows pool counters for each run.
