# Condition-C 600 s End-to-End Validation — 2026-09-07

## Pipeline validity

This was the single authorized fresh 600 s Condition-C validation attempt after
the long-horizon remediation work. It is **invalid as a scientific run** because
the Webots external controllers never connected and the robot stack never
started advancing simulation time. No second 600 s attempt is made in this task.

| Item | Result |
|---|---|
| HEAD | `b08491eca2ab4102f310f8bb6c88604cdb75bd47` |
| Run directory | `results/observer_long_horizon_remediation_600s_20260907/fast_trial_20260907T151030Z/` |
| Configuration | Condition C, `frontier_cost_only`, MODE_B, canonical close-start 20 ms scan-matching world, seed 1001, full sensors, ideal encoders, fast/headless |
| Observer path | Salvaged legacy observer with deferred outputs, native rosbag, 20 ms GT/contact capture |
| Requested horizon | 600 simulated seconds |
| Startup result | FAIL — Webots controllers and paced Supervisor could not connect |
| Scientific horizon | Not reached; `simulation_horizon_reached=false` |
| Finalization | `complete=false`, `MISSING_REQUIRED_ARTIFACTS` |
| Offline evaluator | Not run; the artifact failed the required startup/raw-evidence gate |
| Pipeline verdict | `CONDITION_C_600S_OFFLINE_PIPELINE_FAIL` |

## Preflight

The preflight record passed for the environment prepared before launch:

- clean tracked worktree;
- HEAD `b08491e`;
- ROS 2 Jazzy and the fresh project install;
- approved Webots driver prefix;
- WSL NAT default gateway `172.18.32.1`;
- ROS domain 229;
- `WSL_INTEROP` present;
- Jazzy vendor library path present;
- both required `ldd` checks had zero `not found` dependencies;
- no stale `/home/arash/webots_ws/install` path was selected;
- selected Webots port 23431.

The launch child did not receive the NAT-mode selector, however. The launch
record proves the effective endpoint was:

```text
WEBOTS_CONTROLLER_ENDPOINT host=127.0.0.1 port=23431 network_mode=auto
```

The immediately preceding successful NAT validation used:

```text
WEBOTS_CONTROLLER_ENDPOINT host=172.18.32.1 port=23420 network_mode=nat
```

The attempted command record contains `ROS_DOMAIN_ID=229` but no
`MY_EPUCK_WEBOTS_NETWORK_MODE=nat` export. Thus the preflight’s NAT result was
not propagated into the clean launch child. This is a launch-environment
invocation failure, not a change to observer, evaluator, robotics, SLAM, Nav2,
cooperation, or scientific semantics.

## Startup evidence

Webots itself was spawned, but its extern controllers remained waiting for a
connection on port 23431. The robot and Supervisor clients repeatedly reported
connection failure and then exited:

- robot1 Webots controller: exit code 1 after repeated connection retries;
- robot2 Webots controller: exit code 1 after repeated connection retries;
- paced ROS 2 Supervisor: exit code 1 after repeated connection retries;
- Webots process: exit code 1 after its extern controllers remained unconnected.

The launch log contains the repeated `Cannot connect to Webots instance` lines,
the `Giving up...` lines, and the Webots messages that all four extern
controllers were waiting on port 23431. The observer/logger process started, but
it could not receive a live scientific mission.

Because the external controller endpoint was loopback/automatic rather than the
WSL NAT gateway, the following required startup gates were not satisfied:

| Gate | Result |
|---|---|
| robot1 state publisher/controller operational | FAIL — driver did not connect |
| robot2 state publisher/controller operational | FAIL — driver did not connect |
| robot descriptions/control stack | FAIL as operational robot stack |
| `/robot1/odom`, `/robot2/odom` | absent |
| local maps | absent |
| frontier/protocol/navigation activity | absent |
| START_RELEASE/handoff | absent |
| advancing scientific `/clock` | not established |

The partial bag contains only startup-level evidence and is not a complete raw
scientific bag. Representative contract results are `coverage evidence is
empty`, `MISSING_NAVIGATION_STATUS`, and `passive_rosbag_export.complete=false`.

## Finalization and metrics

The runner was stopped cleanly once the startup failure was established. Its
summary records `exit_reason=interrupt`, `termination_reason=MANUAL_ABORT`,
`launch_return_code=-2`, `simulation_start_time_s=null`,
`simulation_end_time_s=null`, and `observer_finalization_complete=false`.
The generated `artifact_finalization.json` is explicitly incomplete and lists
missing maps, GT/contact files, semantic rosbag completeness, and navigation
replay evidence.

Therefore no authoritative active RTF, coverage, idle, overlap, cooperation,
failure, fairness, handoff, map-quality, warning, or scaling-window result can
be reported for this attempt. Reporting the Webots lifetime ratio or partial
startup wall time as scientific RTF would be invalid.

No offline evaluation was run against this invalid artifact, and no generated
metric sidecars are treated as valid results.

## Source and change status

No production source, observer code, evaluator code, scientific definition,
configuration, or build/install source was changed for this failed attempt. The
earlier remediation commits remain the current source checkpoint:

1. `85048b3` — preserve finalized Condition-C coverage timeline;
2. `ca13345` — bound deferred health finalization with indexed joins;
3. `4a51c1a` — replay warnings from raw rosout receipts;
4. `0e8b94d` — resolve the approved cooperative map-quality tool;
5. `e295022` — document explicit output-root propagation;
6. `b08491e` — record long-horizon attribution and the no-optimization decision.

The minimum correction for a future run is to export
`MY_EPUCK_WEBOTS_NETWORK_MODE=nat` into the same `env -i` child that launches
ROS, while retaining the already-passing ROS/Jazzy library and provenance
environment. This report does not retry it because the task authorized exactly
one 600 s run.

CONDITION_C_600S_OFFLINE_PIPELINE_FAIL
