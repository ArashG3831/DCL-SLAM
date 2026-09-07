# Certificate latency fix plan

Date: 2026-09-08

## Current query scheduling path

The active Condition-C frontier generator is configured by
`src/my_epuck_project/launch/two_robots_decentralized_exploration_launch.py`.
Each generator extracts the current frontier set, builds bounded
`FrontierEvaluationRecord` entries, and calls
`fair_frontier_query_order()` in
`src/my_epuck_frontier_candidates/src/frontier_candidate_generator.cpp`.
The order is already fail-closed and deterministic: first-evaluation work is
kept ahead of refresh work, then the optimistic cost lower bound and starvation
age break ties. The selected work is drained asynchronously by `send_next()`;
the certificate itself remains in
`distributed_frontier_assignment.py` and still requires matching lower-bound
provenance plus mathematical domination of every unqueried option.

The separate allocator-side `maximum_path_queries` value bounds the replicated
bid round. It is not the five-query frontier-candidate drain identified by the
600-second archaeology.

## Source of the five-candidate limitation

The frontier generator declares `maximum_path_queries_per_cycle` with default
`5` at `frontier_candidate_generator.cpp:211`. The production launch sets the
same value explicitly at:

- `two_robots_frontier_candidates_launch.py:36`;
- `two_robots_decentralized_exploration_launch.py:366`.

`start_cycle()` passes this value as `query_limit` to
`fair_frontier_query_order()` and `send_next()` stops with
`QUERY_LIMIT_REACHED` once that many exact `ComputePathToPose` requests have
been issued. This is the five-candidate drain observed in the archaeology.

## Smallest safe improvement

Raise only the frontier-candidate per-cycle query cap from 5 to 8, matching the
existing `maximum_candidates_before_path_check` bound. Keep the existing
lower-bound/age ordering, candidate identity deduplication, timeout, planner
lease, and cycle termination behavior unchanged.

This supplies more exact evidence per generation without declaring any
candidate reachable, changing a cost, pruning an option, or changing the
certificate comparison. A certificate still fails closed whenever any
unqueried option can compete or its provenance is unavailable.

## Expected effect

The exact-query service opportunity increases from at most five to at most
eight candidates per generator cycle (a maximum 60% increase in drain
capacity). Because the selected candidates remain the lowest optimistic-bound
first-evaluation candidates, the change should retire competitive unqueried
options sooner and reduce certificate-deferred intervals and avoidable idle.
The 600-second archaeology does not justify a deterministic percentage for
idle reduction: planner latency, lease contention, candidate churn, and
disappearance remain runtime-dependent and will require a later bounded
validation run.
