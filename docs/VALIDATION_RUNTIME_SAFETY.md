# Validation runtime safety contract

This checkout is the only supported source for Webots/ROS 2 validation. The
original dirty checkout at `/home/arash/webots_ws` must not appear in
`PYTHONPATH`, `AMENT_PREFIX_PATH`, or `COLCON_PREFIX_PATH`.

## Middleware

Simulation uses CycloneDDS over the WSL loopback profile only:

```text
RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
CYCLONEDDS_URI=file:///home/arash/webots_ws_clean_validation_20260823/config/cyclonedds/wsl_loopback.xml
```

The campaign runner rejects Fast DDS, missing `CYCLONEDDS_URI`, inherited
domain mismatches, modules imported outside the selected checkout, and
source/build hash mismatches before creating an attempt directory or ROS
participant.

## Domain-port hazard

The active CycloneDDS profile uses the RTPS participant-port relationship:

```text
PB + DG * domain + d3 + PG * participant_index
```

With the Jazzy profile values and the configured participant bound, domain 230
is the largest safe domain. Domains 231 and 232 can generate UDP port 65536.
The observed `ddsi_udp_conn_write ...:65536 failed` traffic must be treated as
a fatal infrastructure error. It must never be retried as an ordinary launch
failure because reliable DDS traffic can accumulate WSL/Windows network
buffers and trigger nonpaged-pool pressure.

## Evidence-output bound

Physical unknown-pose diagnostic JSONL is capped at 32 MiB and a bounded record
count. Drops and write failures are summarized separately. A bounded output
file is not, by itself, proof that Windows kernel network buffers are safe;
host counters must still be sampled independently.

## Required pre-run gates

1. Record a post-reboot Windows available-memory and nonpaged-pool baseline.
2. Confirm no campaign-owned process or selected port remains.
3. Run the bounded CycloneDDS talker/listener and entity-churn probes.
4. Use a unique safe domain (0--230) and Webots port.
5. Run headless, without RViz or rendering, with one campaign instance.
6. Abort on a DDS assertion, SIGABRT/SIGSEGV, invalid endpoint, clock stall,
   unsafe host pressure, or incomplete process-tree cleanup.

The final report must distinguish WSL process RSS from Windows nonpaged-pool
usage. A single ROS PID cannot be named as the owner of kernel nonpaged pool
without Windows pool-tag/PID evidence.
