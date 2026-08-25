# Raw D500 transport forensic result — 2026-08-25

## Verdict

The one-relay architecture is not yet validated. Reliable raw Webots scans
deliver the full 720-beam stream but reproduce substantial Windows nonpaged
pool growth. Best-effort raw transport is resource-stable in the bounded
tests, but CycloneDDS 0.10.5 on the WSL loopback profile drops fragmented
best-effort `LaserScan` samples, so it cannot carry the full-resolution input
required by Slam Toolbox. The issue is therefore **not fixed** and is not a
frontier or SLAM-estimator defect.

## Provenance and safety

* Validation checkout: `/home/arash/webots_ws_clean_validation_20260823`.
* Original dirty checkout was not modified.
* Source HEAD during the test: descendant `c760caecace984219316ff3db82684a6a31af9f4` of the reported `0bce057` baseline.
* ROS 2 Jazzy, `rmw_cyclonedds_cpp`, CycloneDDS `0.10.5-1noble.20260225.142613`.
* Webots was realtime, headless, with no RViz/rendering and no motion fixture.
* `throttle_scans=1`, scan matching enabled, loop closing disabled.
* Every test used a unique ROS domain and Webots port. Campaign-owned
  processes were terminated and verified absent after the probes.

## Full-graph A/B after Windows reboot

Artifacts:

* Best-effort raw: `results/transport_one_relay_best_effort_postreboot_20260825f`.
* Reliable raw: `results/transport_one_relay_reliable_postreboot_20260825a`.

The reliable run used one `d500_scan_fix` per robot and delivered data:

* relay logs: robot 1 received 103/published 100; robot 2 received 203/published 200;
* map rates were about 1.0 Hz and corrected scan records were present;
* Windows Pool Nonpaged Bytes rose from about 933 MB to 1,614 MB (+680 MB)
  in about 228 seconds;
* no DDS assertion, SIGABRT, SIGSEGV, or process leak occurred.

The full-graph best-effort run was resource-stable (pool about 883–946 MB,
about +64 MB), but both relays received zero raw scans, producing zero
corrected scans and zero maps. It therefore failed the transport-readiness
gate and was not a stability pass.

## Minimal cross-process QoS reproducer

The Webots custom plugin was enabled only in a temporary validation URDF for
this probe and then reverted; the active source still uses the stock
`webots_ros2_driver` lidar publisher. The plugin published `/robotN/scan_d500`
with `qos_profile_sensor_data` (BEST_EFFORT), while an external Python
subscriber used BEST_EFFORT, VOLATILE, KEEP_LAST.

With the same CycloneDDS loopback profile and 720-beam `LaserScan` payload:

```text
best-effort, 1 beam:   72 messages / 7 s
best-effort, 100 beams: 71 messages / 7 s
best-effort, 180 beams: 71 messages / 7 s
best-effort, 360 beams: 0 messages / 7 s
best-effort, 720 beams: 0 messages / 7 s
```

A RELIABLE external subscriber reported:

```text
New publisher discovered ... offering incompatible QoS.
Last incompatible policy: RELIABILITY
```

This proves the Webots plugin writer was actually best-effort; the absence of
messages is not a mistaken reliable/best-effort parameter. A same-process
best-effort publisher/subscriber test did receive 79 large messages, so the
failure is specifically the cross-process fragmented transport path.

The source of the observed boundary is the DDSI fragmentation behavior. The
official CycloneDDS 0.10.5 documentation describes samples at or above the
configured fragment size as DDSI fragments and notes a separate
`Internal/DefragUnreliableMaxSamples` path for best-effort writers:

<https://cyclonedds.io/docs/cyclonedds/0.10.5/config/cyclonedds_specifics.html>

Changing temporary `FragmentSize` values (500, 1,025, 4,000, and 14,720 B),
interface address form, multicast-loopback settings, and best-effort
defragmentation limits did not make the 720-beam cross-process probe deliver.

## Why the earlier best-effort conclusion was wrong

The earlier bounded run observed a stable pool while the best-effort graph was
actually receiving no raw samples. Resource stability without data delivery
was incorrectly treated as a transport fix. The corrected interpretation is:

* reliable full-resolution raw transport: functional but unsafe for long
  Windows/WSL runs because of pool growth;
* best-effort full-resolution raw transport: resource-stable but functionally
  broken under this CycloneDDS loopback build;
* best-effort payloads at 180 beams: functional in isolation, but insufficient
  to preserve the required full-resolution Slam Toolbox input.

## Current source state

The temporary URDF/plugin wiring and default-best-effort edits were reverted.
The active launch remains fail-safe reliable for the full-resolution raw path,
and the existing one-relay split remains intact: one raw reader feeds the
reliable full corrected Slam stream and the depth-1 180-beam Nav stream. No
source change is being claimed as a fix for the Windows pool behavior.

## Required blocker before handoff, soak, or final campaign

Do not launch a handoff, long-navigation soak, or 1,200-second campaign until
one of these is proven in a bounded test:

1. a supported CycloneDDS/transport configuration that delivers cross-process
   full-resolution best-effort `LaserScan` samples without pool growth; or
2. a documented transport redesign that preserves all 720-beam SLAM evidence
   without a second raw DDS reader and without unsafe reliable raw traffic.

The current evidence is an infrastructure/transport blocker, not evidence
against the FrontierExplorerCore refactor. Unknown-pose handoff, shared-map
activation, navigation soak, and terminal exploration remain unvalidated on
the corrected transport path.

## TCP transport probe (not a project-wide fix)

Because CycloneDDS 0.10.5 includes its optional TCP transport, a temporary
two-process profile was tested outside the repository. With ROS discovery
environment variables unset (the `ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST`
override prevents this static TCP peer setup), two processes exchanged a
720-beam best-effort `LaserScan` stream at 10 Hz:

```text
published: 601 samples / 60 s
received:  601 samples / 60 s
Pool Nonpaged Bytes: 1612.6–1621.4 MB (+6.6 MB)
WSL available memory: 6749–6796 MB
```

This proves that TCP can carry the large sample in a minimal two-process
experiment, but it is not yet a valid campaign profile. CycloneDDS TCP needs
known peer/listener ports. A fixed TCP port can be shared by the two probe
processes only with the temporary setup; a same-port multi-process ROS probe
received zero samples, and dynamically allocated listener ports cannot be
discovered by the current static loopback peer list. The project launch starts
many independent ROS processes, so adopting TCP would require a separately
designed discovery/port architecture and its own end-to-end validation. It
must not be silently substituted for the existing decentralized UDP profile.

Therefore the TCP result is recorded as a lead, not as evidence that the
one-relay campaign gate has passed. No handoff, soak, or 1,200-second campaign
was launched after the full-resolution best-effort gate failed.
