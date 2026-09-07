# Degraded-solo traffic safety fix analysis

Date: 2026-09-08
Baseline: `66ae5b28abc27e166ff04981e9a6242f09a7462c`

## Scope and conclusion

The existing traffic scheduler is not the defect. In the failed Condition-C run,
cooperative wait states entered the degraded-solo dispatch path while peer
communication was still healthy. That path has only one local route and therefore
cannot call the existing two-route `schedule_traffic()` decision. The smallest
safe correction is to permit cooperative degraded-solo dispatch only after the
existing bounded `PeerLiveness` timeout has established peer unavailability.

No scoring, traffic geometry, Nav2, SLAM, handoff, or map-fusion change is needed.

## Current state machine

1. Fresh task snapshots create a canonical paired round in
   `DistributedFrontierAssignment._tick_impl()`.
2. Both replicas evaluate candidate paths and exchange bid batches.
3. `traffic_compatible()` calls `_traffic_for_bid_pair()`, whose sole scheduler is
   `schedule_traffic()`. Conflicting candidate pairs are excluded before the pair
   decision is committed.
4. The agreed decision is checked again by `_traffic_for_decision()` and dispatch
   applies the existing winner/waiting-robot result.
5. During transient waits for peer snapshots, bids, shared TF, certificate evidence,
   or peer agreement, `_tick_impl()` calls
   `_continue_local_work_while_waiting()`.
6. Before this fix, `_continue_local_work_while_waiting()` immediately called
   `_continue_degraded_solo()` whenever the local robot was free. It did not first
   require `PeerLiveness.evaluate()` to report `DEGRADED_SOLO`.

The existing liveness policy is already bounded: `PeerLiveness` enters
`DEGRADED_SOLO` only after its configured receiver-steady-clock timeout. The class
default is 6 seconds; the canonical shared distributed-assignment launch explicitly
uses 12 seconds. Fresh peer snapshots and status heartbeats refresh that liveness
evidence.

## Exact bypass point

- `distributed_frontier_assignment.py:1804-1836`:
  `_continue_local_work_while_waiting()` directly invokes
  `_continue_degraded_solo()` without proving peer loss.
- `distributed_frontier_assignment.py:3751-3904`:
  `_continue_degraded_solo()` evaluates only the local path and Nav2 dispatch
  preconditions. It has no peer route with which to call `schedule_traffic()`.
- `distributed_frontier_assignment.py:2703-2722`:
  the authoritative traffic compatibility check exists only in the paired
  candidate-selection path.
- `DistributedExplorationStatus.msg:10-40` advertises peer state, active task ID,
  and health, but not the peer route samples. Same-task reservation therefore
  cannot prove geometric route safety.

This is why the bad run could publish `DEGRADED_SOLO` commitments while
`peer_communication_healthy=true`, then independently dispatch paths whose minimum
separation was 0.0124 m. The same existing scheduler rejects those paths when they
reach the paired path.

## Smallest safe fix

Add one guard at the cooperative fallback boundary:

- while peer liveness is healthy, return without entering degraded solo;
- after the existing timeout establishes `DEGRADED_SOLO`, preserve the existing
  degraded-solo implementation unchanged;
- keep the true local-only branch unchanged, because it does not represent a
  two-robot cooperative fallback;
- keep all normal paired-round and `schedule_traffic()` behavior unchanged.

This is preferable to adding traffic logic to degraded solo: exact conflict
checking requires the peer route, which is not present in the status heartbeat.
Adding an approximate check or a second route protocol would enlarge the
architecture and duplicate the proven scheduler.

## Required regression evidence

Focused tests must prove:

1. a geometrically conflicting pair is detected by the existing scheduler and a
   healthy peer prevents the cooperative wait path from dispatching either route
   through degraded solo;
2. degraded solo remains available after the existing peer timeout;
3. the normal paired path still invokes the existing traffic scheduler and all
   existing traffic-scheduler regressions remain green.

## Implemented correction and validation

The implementation adds `_peer_unavailable_for_degraded_solo()` and calls it at
the single cooperative fallback boundary. It checks both status and snapshot
freshness before accepting the existing `PeerLiveness` timeout result. The
local-only branch still calls `_continue_degraded_solo()` directly, and the paired
selection and traffic scheduler are unchanged.

Focused validation used the canonical isolated ROS/interface install and the
checked-out Python source:

- degraded-solo/round-guard, traffic-scheduler, and protocol tests: 66 passed;
- continuation, failure/telemetry, pair-scoring, and task-canonicalization tests:
  93 passed;
- combined focused regression run: 159 passed.

An initial broader test invocation inherited stale generated interfaces from the
workspace `build/` path and produced three unrelated `FrontierCandidateArray`
attribute errors. Re-running in the canonical isolated install proved the current
generated interface contains `candidate_generation_id`; all 159 focused tests then
passed. No Webots run was performed.
