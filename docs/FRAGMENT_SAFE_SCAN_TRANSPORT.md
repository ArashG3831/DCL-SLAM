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

Reliable QoS is used only for the small chunk messages and the reconstructed
corrected scan. No reliable raw 720-beam DDS sample is used. Slam Toolbox
continues to receive the full corrected stream with `throttle_scans=1` and
scan matching enabled.

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
