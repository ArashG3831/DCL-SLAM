# Two-robot distributed frontier assignment

This package implements decentralized cooperative occupancy-grid mapping and exploration with
known initial relative poses. It does not implement joint pose-graph optimization, inter-robot
loop closure, unrestricted C-SLAM, or centralized allocation.

## Runtime graph

Each robot retains ordinary namespaced Slam Toolbox, local-evidence-only `PeerMap` export,
`source_aware_map_fusion`, a replicated `shared_map`, and its own Nav2 stack. The proposal source
links the checked-out `frontier_exploration_ros2` public core for extraction, costmap-aware
accessible goals, geometry signatures, and lidar-style visible-reveal gain. It never dispatches.

Each `distributed_frontier_assignment` peer consumes both bounded task snapshots and bid arrays,
constructs the same canonical physical union, exhaustively scores ordered non-equivalent task
pairs (including bounded IDLE cases), and publishes the complete decision. Dispatch requires a
matching peer decision hash. Each peer has only relative, namespace-local
`compute_path_to_pose` and `navigate_to_pose` action clients. It has no `cmd_vel` publisher and no
peer action client.

Production assignment is the task-level Burgard adaptation: each eligible canonical task starts
with `U=1`, each robot contributes its locally verified Nav2 path cost
`C=clamp(path_length_m/18.0,0,1)`, and the deterministic selection score is
`U - beta*C` with `beta=1`. After a task is selected, remaining task utility is reduced by
`P(d)=1-d/11.98` when the task representatives are within the configured lidar range and the
shared-map line of sight is clear. The old seven-term score remains available only as explicit
`legacy_weighted` diagnostic mode; its gain, proximity, route, failure, sensing, and workload
fields are retained in messages for historical report compatibility, not used by production
Burgard ranking. Route geometry is not an exploration utility term. Traffic scheduling remains
separate and disabled by default pending physical bottleneck validation.

The retained `cooperative_frontier_coordinator` is the legacy decentralized claim-only baseline.
It is not launched by the final distributed launch.

## Protocol and time domains

- ROS time: map, sensor, TF, task-generation, and path-query provenance.
- Receiver-local steady time: message receipt age, clamped TTL expiry, peer timeout, retries,
  navigation timeout, map-stability grace, and completion confirmation.
- Sender monotonic timestamps are never compared between robots.
- Map revisions are compared only for the same source robot and source session UUID.
- A new session resets source-local revision interpretation; its predecessor is retired so delayed
  old-session snapshots cannot masquerade as another restart.
- Exact immutable snapshot retransmissions refresh liveness without advancing the epoch.

For each fresh pair of source snapshots, both peers calculate:

```text
round_id = hash(robot1_session, robot1_epoch, robot2_session, robot2_epoch)
union_hash = hash(deterministically ordered canonical physical tasks)
```

Bid arrays bind source identity/session/epoch, `round_id`, and `union_hash`. A peer commits only
after both bid fingerprints, both assignments, and the decision hash match. Old rounds cannot
cancel a committed goal. A peer session restart, verified terminal result, bounded timeout, or
explicit material invalidation can start a new round.

In `DEGRADED_SOLO`, a surviving peer may evaluate at most one locally proposed task per local
snapshot epoch and dispatch only through its own Nav2. It does not claim coordinated agreement.
A fresh peer session is required to leave degraded mode.

Operational `COMPLETE` requires an agreed empty assignment, fresh task/status messages, stable map
provenance, no active goals, healthy local and peer candidate/communication state, active local
Nav2 lifecycle evidence, a healthy required transform, matching peer completion candidacy, and a
bounded confirmation interval. Candidate evidence distinguishes no frontiers, only-small
frontiers, out-of-range frontiers, and verified unreachable frontiers. Planner failures or
unclassified candidates cannot masquerade as successful completion; repeated matching planner
infrastructure evidence can produce an explicit abort reason. A mission timeout is also an
explicit abort. Missing health evidence produces `BLOCKED`, not completion.

## Passive evidence

The default final launch starts `cooperative_experiment_logger`, a subscription-only evaluator.
Its `events.jsonl` records bounded task sets, bids, pair score components, decisions, status,
failures, state transitions, path length, recoveries, duration, and odometry-derived travelled
distance. ComputePath evidence is attributed to frontier reachability versus allocator bidding
versus final dispatch validation, including result class and duration where emitted. The compact
summary adds round outcomes, planner attribution, mission terminal reason, and
`mission_result.json` with a recommended exit code. Immutable protocol heartbeats are deduplicated.
Coverage, first-observer attribution, duplicate work, topic health, CPU, and logger RSS retain
the existing report schema. Disabling or crashing the observer cannot alter robot control.

The current physical-task equivalence is deterministic quantized geometry using approach
separation together with bounds and sampled frontier overlap. Predicted sensing overlap is a
bounded geometry/visible-cell approximation; it is not an exact future sensor simulation. There
is no generic fleet abstraction: robot identities and exhaustive assignment are exactly two-robot.
