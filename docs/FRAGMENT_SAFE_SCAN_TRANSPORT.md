# Fragment-safe D500 scan transport

## Problem and evidence

On the WSL2 loopback path used by validation, a raw 720-beam `LaserScan`
cannot be delivered as cross-process best-effort DDS traffic. Smaller samples
through 180 beams are delivered, while a reliable raw 720-beam stream causes
the Windows nonpaged pool to grow. A minimal TCP probe delivered 720-beam
messages, but a complete ROS graph has not yet been proven over CycloneDDS
TCP. The transport boundary is therefore treated as a middleware/WSL issue,
not as a SLAM or frontier-algorithm issue.

## Current design

The simulation path has exactly one raw lidar reader per robot:

```text
Webots D500 device
  -> ChunkedLidarPlugin (four <=180-beam ScanChunk messages)
  -> d500_scan_fix ChunkAssembler
  -> reliable /scan_d500_fixed (complete 720-beam LaserScan) -> Slam Toolbox
  -> best-effort depth-1 /scan_d500_nav (180 beams) -> Nav2/collision monitor
```

`ScanChunk` carries the source robot, sequence number, chunk count/index,
original beam count, all LaserScan angle/time/range metadata, frame and stamp,
and an FNV-1a checksum over the range payload. The assembler rejects wrong
robots, invalid metadata, duplicate conflicts, bad checksums, stale chunks and
over-capacity assemblies. It publishes only after every chunk is present and
the reconstructed range count equals the original count. Pending assemblies
are capped and expire after a bounded timeout.

The stock Webots `Ros2Lidar` device block is removed in chunked mode. Leaving
an `enabled=false` block still creates a raw publisher with the installed
driver, so removal is required to preserve the one-reader topology.

The small chunk messages use best-effort depth-eight QoS to avoid reliable
reader-repair buffers in the WSL/Hyper-V path while retaining the four-message
burst. The assembler is atomic: loss
of any chunk discards that complete scan and never produces a partial sample.
Only the reconstructed corrected scan is reliable. No reliable raw 720-beam
DDS sample is used. Slam Toolbox continues to receive the full corrected
stream with `throttle_scans=1` and scan matching enabled.

## Focused evidence

The isolated two-robot namespaced smoke on CycloneDDS loopback delivered, per
robot, 80 chunks representing 20 distinct sequences and 20 complete corrected
720-beam scans. A best-effort subscriber received 20 180-beam Nav2 scans per
robot. `ros2 topic list --no-daemon` exposed only `scan_d500_chunks`,
`scan_d500_fixed`, and `scan_d500_nav`; no raw `scan_d500` topic was present.

The bounded assembler tests cover one-, 180-, 360- and 720-beam payloads,
atomic reconstruction, metadata preservation, missing/duplicate/out-of-order
chunks, wrong robot IDs, metadata mismatch, checksum failure, timeout and
bounded pending storage.

## Validation status

This change is a transport adapter, not a replacement SLAM algorithm. The
full-graph transport gate still requires a clean host baseline, complete ROS
graph discovery, map updates, Slam Toolbox reception, stable Windows pool
telemetry, and complete process/port cleanup. Unknown-pose handoff,
navigation soak and the final campaign must remain blocked until that gate
passes.

## Mirrored-WSL bounded graph validation (2026-08-25)

The raw cross-process diagnostic remains an expected negative control: a
720-beam `LaserScan` publisher emitted 150 samples while its separate
best-effort subscriber received zero under the CycloneDDS loopback profile.
This is not the production path and must not be used as evidence that the
chunked adapter failed.

The supported one-reader project launch was then run in realtime with the
current install, two active robots, `scan_transport:=chunked`, raw input
best-effort, and `use_sim_time:=true` (40 seconds, no Nav2/SLAM campaign).
Both Webots controllers and `Ros2Supervisor` connected. The independent
probe recorded:

```text
                         robot1   robot2
chunk messages              140      140
complete /scan_d500_fixed    35       35   (720 beams each)
/scan_d500_nav               35       35   (180 beams each)
nonzero /clock             1746 messages, 0.02 -> 34.92 s
```

Every received scan stamp was nonzero. Pool Nonpaged Bytes stayed near
`1.00--1.01 GiB`, available Windows memory stayed above `6.9 GiB`, and the
complete launch tree and port `24703` were cleanly released. This proves the
fragment-safe transport and one-reader topology at the sensor/controller
graph boundary; it does not yet prove Slam Toolbox/map/fusion or terminal
exploration.

Mirrored WSL requires the project launch's explicit `127.0.0.1` controller
override. The stock `webots_ros2_driver` helper derives the NAT gateway from
`/etc/resolv.conf`, which is wrong when WSL uses `networkingMode=mirrored`.

The final rebuilt-install smoke (`install_transport_fixed_20260825` plus the
current Python overlay) recorded 112 chunks and 28 complete 720-beam fixed
scans per robot, 28 complete 180-beam Nav2 scans per robot, and 1,367
nonzero `/clock` samples (`0.02` to `27.34` s). A startup sample at exactly
zero simulation time is now dropped by `d500_scan_fix` rather than published
as stale data; all delivered fixed/Nav2 scan stamps in this run were
nonzero. Pool Nonpaged Bytes remained approximately `1.01 GiB` and no
process or port remained after teardown.

The full-resolution SLAM gate was then run from the rebuilt overlay with the
two namespaced reliable Slam Toolbox wrappers (unknown-pose local maps,
scan matching enabled, loop closing disabled, 38 seconds). It recorded:

```text
                         robot1   robot2
chunk messages              120      120
complete /scan_d500_fixed    27       26   (720 beams each)
/scan_d500_nav               27       26   (180 beams each)
local map updates             27       26
nonzero /clock             1482 messages, 0.02 -> 29.64 s
```

Slam Toolbox activated for both robots and no DDS assertion, abort, or active
process crash occurred. Pool Nonpaged Bytes remained approximately
`1.01--1.02 GiB`; available memory remained above `6.45 GiB`. The complete
launch tree and port `24707` were cleanly released. This closes the bounded
full-resolution transport gate for the current WSL validation topology.
