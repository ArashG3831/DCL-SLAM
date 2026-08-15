# Run 2 terminal-failure forensics

Run: `pre_demo_burgard_large_02_final`
Scope: retrospective analysis of the two terminal `NavigateToPose` failures. No traffic event occurred in this run.

## Executive conclusion

The retained data establish the same outer failure mechanism for both goals:

1. a local ComputePathToPose bid was valid and the assignment was agreed;
2. robot 1 accepted and dispatched the local NavigateToPose goal;
3. `bt_navigator` emitted `[navigate_to_pose] [ActionServer] Aborting handle.` followed by `Goal failed`;
4. the project received a non-success terminal result with no persisted Nav2 error code/message and classified it as `UNKNOWN` (`ExplorationFailure` value 8).

The old run does **not** preserve the Nav2 action result/status, BT child status, or goal-correlated planner/controller diagnostic needed to distinguish planner failure, controller failure, progress-checker failure, TF failure, recovery exhaustion, or map-change invalidation. The scientifically correct retrospective conclusion is therefore `NAV2_BT_ABORT_WITH_INSUFFICIENT_TERMINAL_TELEMETRY`, not a guessed deeper cause.

## Failure 1

Facts:

- Robot: `robot1`
- Round: `063682ad...3150`
- Canonical task: `46a9fdf7c4c2b8fca8b21998`
- Physical task: `d32d014676762cb5a2404081`
- Goal: `(-3.37500007, -0.49500008)` m
- Dispatch: 85.50 s simulated time
- Terminal: 92.80 s simulated time
- Navigation duration: 7.2918 s
- Bid path: 1.18186 m; normalized Burgard cost `1.18186 / 18 = 0.06566`
- Utility before assignment: 1.0
- Travelled distance at terminal: 0.13157 m
- Project failure class: `UNKNOWN` / 8
- Persisted Nav2 result error code/message: `0` / empty
- Recovery count: 0
- Rosout: `bt_navigator` aborted the action and logged `Goal failed`

The nearest retained robot sample after the event was at `(-2.67477, -1.22638)` m with zero commanded velocity. This is useful context, but it is not a Nav2 causal diagnostic.

Supported cause: terminal NavigateToPose BT/action abort.
Unsupported causes: planner no path, controller no valid control, failed-to-make-progress, TF failure, recovery exhaustion, and goal invalidation by a changing map.

## Failure 2

Facts:

- Robot: `robot1`
- Round: `3c5ab512...bfd33a`
- Canonical task: `b2a255a201ef6949c42bfb95`
- Physical task: `f481710d17c3740c51d472d6`
- Goal: `(-2.53500009, 1.93499986)` m
- Dispatch: 176.94 s simulated time
- Terminal: 188.28 s simulated time
- Navigation duration: 11.4119 s
- Bid path: 3.46462 m; normalized Burgard cost `3.46462 / 18 = 0.19248`
- Utility before assignment: 1.0
- Redundancy reduction for this task before assignment: zero because the recorded line of sight was blocked; this is an assignment fact, not a navigation failure cause.
- Travelled distance at terminal: 1.0330 m
- Last retained distance remaining: 2.4362 m
- Last retained commanded forward velocity: 0.10665 m/s
- Project failure class: `UNKNOWN` / 8
- Persisted Nav2 result error code/message: `0` / empty
- Recovery count: 0
- Rosout: `bt_navigator` aborted the action and logged `Goal failed`

Supported cause: terminal NavigateToPose BT/action abort while the robot remained substantially short of the goal.
Unsupported causes: planner no path, controller no valid control, failed-to-make-progress, TF failure, recovery exhaustion, and goal invalidation by a changing map.

## Why recovery count stayed zero

Both terminal project events report `recoveries=0`, and no `RECOVERY_COUNT_CHANGED` event exists for robot 1. The logger records recovery feedback by count changes; it did not retain raw NavigateToPose feedback/status or BT recovery-node transitions. Therefore the evidence supports **zero observed recovery feedback**, but cannot prove whether Nav2 aborted before entering recovery or whether a recovery transition was not captured. It is not valid to claim recovery exhaustion.

## Map and odometry contribution

The second failure window contains shared-map divergence/stale-frontier activity, so map evolution is a plausible mechanism in this architecture. However, Run 2 did not retain dispatch-time and terminal costmaps, Supervisor GT, or a GT-to-odom trajectory. The task was still present in the immediately preceding task snapshot and had a valid local path bid. No goal-correlated obstacle change is available. Map change is therefore **not established** as the cause.

Likewise, no Run-2 GT trajectory exists. Archived trajectories from other campaigns were not used as evidence for these goals.

## Effect on future suppression

Both outcomes were classified `UNKNOWN`, not a typed hard-unreachable/planner/controller/timeout failure. No `FAILURE_SUPPRESSION_CREATED` event was retained for these outcomes. Thus the observed event stream does not show them entering the normal typed hard-failure suppression path. This is safer than inventing a permanent hard gate, but it also explains why the old run cannot answer whether a retry/expiry record was created for these exact tasks.

## Forensic-capture startup failure and repair

The prior diagnostic-only run stopped with:

```text
SUPERVISOR_ROBOT_NOT_FOUND name=ROBOT1
```

The source world names the nodes `robot1` and `robot2`; the Webots Supervisor runtime exposed the names as `ROBOT1` and `ROBOT2`. The observer’s case-sensitive lookup failed. The failure was not caused by launch ordering, ROS domain, RMW, or Webots port availability.

The harness repair is diagnostic-only: case-insensitive Supervisor node lookup, a `supervisor_ready.json` file containing discovered robot definitions and timestep, and logger events for Supervisor ready/exit. No production navigation, mapping, controller, allocator, or world behavior was changed.

## What the fresh runs must capture

The repaired run records raw Supervisor pose, odometry, local/shared maps and metadata, transforms, coverage, allocator events, and rosout. The navigation failure logger now also emits the raw wrapped action status, accepted flag, error code/message, failure class, recovery count, duration, and travelled distance for any future terminal failure. This is intended to make the next failure classifiable at the Nav2 layer rather than only as project `UNKNOWN`.
