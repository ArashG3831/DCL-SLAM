# Host resource stop policy

**Status: authoritative for validation runs (2026-08-26).**

This policy supersedes older notes that treated a low Windows-available-memory
sample, a large `vmmemWSL` working set, or a Linux page-cache increase as an
automatic campaign stop. Those values must still be recorded, but they are not
hard-stop conditions by themselves.

## The only automatic campaign stop conditions

The watchdog must request an orderly campaign shutdown when either condition is
true:

```text
Pool Nonpaged Bytes >= 6 GiB
Windows RAM usage      >= 99.0 percent
```

For the Windows counter used by the runners, 6 GiB is:

```text
6 * 1024^3 = 6,442,450,944 bytes
```

The checks must use Windows-native telemetry (via the absolute PowerShell
executable path), not a Linux estimate. At every sample record the raw pool
bytes, available bytes, and RAM percentage, plus the timestamp and run ID.

When a threshold is reached, do not launch a replacement run. Finish the
bounded, orderly teardown, verify the complete campaign tree and ports are
gone, save the telemetry, and stop further runtime validation until the host
state has been reviewed. Do not keep a run alive deliberately to consume the
remaining memory.

## Values that are informative, not stop triggers

The following are not equivalent to active Windows pool exhaustion and must not
by themselves cause `wsl --shutdown`, a campaign kill, or a failed run:

* the WSL `vmmemWSL` working-set/commit allocation growing toward the WSL VM
  limit;
* Linux `buff/cache` or file-page cache growing after source/artifact searches;
* low Linux `MemFree` while `/proc/meminfo` `MemAvailable` remains healthy;
* a transient Windows Available MBytes fluctuation while RAM usage remains
  below 99% and Pool Nonpaged Bytes remains below 6 GiB;
* normal post-run WSL allocation retention.

Interpret WSL memory with `MemAvailable`, `AnonPages`, process-tree RSS, cache,
swap, and the Windows counters together. A file search can legitimately make
`vmmemWSL` retain several GiB of reclaimable cache; that is not proof that a
ROS process or a kernel leak owns those pages.

## Baseline and telemetry requirements

Before a run, record Windows boot time, Pool Nonpaged Bytes, Windows RAM
percentage, WSL boot ID, WSL `MemAvailable`, process-tree RSS, and campaign
process/port state. A threshold already exceeded before launch is a failed
preflight; do not start Webots.

During and after a run, retain:

* the raw Windows pool/RAM timeline and its within-run slope;
* WSL `MemAvailable`, anonymous pages, cache, swap, and `vmmemWSL` samples;
* per-PID RSS/CPU/command line and role mapping;
* the post-cleanup values and an idle-control slope;
* the exact stop reason (`STOP_POOL_6G` or `STOP_WINDOWS_RAM_99PCT`) when
  applicable.

Do not call a resource problem fixed merely because a process exited. Compare
within-run and post-cleanup deltas against a matched control. If Pool Nonpaged
Bytes remains above 6 GiB after cleanup, require a full Windows reboot before
another WSL/Webots run; `wsl --shutdown` alone does not clear Windows kernel
pool state.

## WSL shutdown rule

`wsl --shutdown` is not a routine response to WSL cache growth and must not be
invoked merely because `vmmemWSL` looks large. Use it only when explicitly
requested, as part of a documented cleanup operation after a threshold stop,
or when the WSL VM itself is the confirmed failed component. A normal WSL
allocation that is reclaimable or stable is allowed to remain in place while
validation continues.

## Attribution boundary

Pool Nonpaged Bytes is Windows kernel memory. A Linux PID or topic cannot be
named as its owner without Windows pool-tag/ETW/NDIS evidence. If those tools
are unavailable or require elevation, report the external owner as unresolved
and distinguish that limitation from application-level RSS and cache data.
