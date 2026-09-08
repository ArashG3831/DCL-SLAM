# Exact local path-evaluation cache validation

Date: 2026-09-08

## Change

Implemented one isolated allocator change: a bounded cache of successful local
`ComputePathToPose` results across allocator round replacement.

Files changed:

- [distributed_frontier_assignment.py](/home/arash/webots_ws_clean_validation_20260823/src/my_epuck_project/my_epuck_project/distributed_frontier_assignment.py:150)
- [test_distributed_path_evaluation_cache.py](/home/arash/webots_ws_clean_validation_20260823/src/my_epuck_project/test/test_distributed_path_evaluation_cache.py:1)
- This report.

The cache is fixed at 32 entries and uses deterministic FIFO eviction. Compact
health telemetry reports hits, misses, context invalidations, and evictions.

## Exact cache key

The key is:

```text
(
    full frozen CanonicalTask object,
    source snapshot provenance tuple,
)
```

The complete `CanonicalTask` includes every canonical field and every frozen
`PhysicalTask` member, including source robot/session/epoch/map revision,
physical identity, frontier geometry, bounds, target approach and orientation,
visibility/scoring fields, route context, embedded path fields, and generation
timestamp.

The source snapshot tuple additionally includes:

- source robot and session;
- snapshot epoch;
- map revision and map fingerprint;
- snapshot generation timestamp;
- lower-bound context fingerprint;
- costmap revision; and
- candidate-generation ID.

The cache is therefore not keyed only by canonical task ID and does not omit
candidate-generation or certificate-relevant snapshot provenance.

## Reuse and correctness

Only results satisfying all of the following are stored:

- allocator bid caller;
- matching task physical signature;
- `error_code == 0`;
- `FailureClass.UNKNOWN` (the successful planner-result classification);
- valid finite non-empty path;
- finite nonnegative heading cost; and
- nonzero map and costmap stamps.

Failed, aborted, timed-out, invalid, nonfinite, contextless, and non-allocator
results are not stored.

On lookup, the exact key must match and
`LocalNav2.path_context_matches(cached)` must still be true. A stale
map/costmap context removes the entry and follows the existing planner-query
path.

A cache hit supplies only the still-valid path result to `_append_bid()`.
`_append_bid()` constructs a new bid for the current `RoundWork`; the old round,
union, snapshot epoch, peer bid, peer decision, and certificate are not reused.
Normal peer-bid validation, certificate evaluation, freshness rules, canonical
round identity, commitments, traffic, and final dispatch checks are unchanged.
The existing final dispatch path still rechecks path context, dispatch
preconditions, finite path validity, and navigation acceptance.

## Focused tests

Command run from a sanitized test shell sourcing ROS Jazzy and the current
clean install, with the source package prepended so the modified source was
tested:

```text
python3 -m pytest -q \
  src/my_epuck_project/test/test_distributed_path_evaluation_cache.py \
  src/my_epuck_project/test/test_distributed_frontier_round_guard.py \
  src/my_epuck_project/test/test_distributed_protocol.py
```

Result: **71 passed**.

Coverage includes:

- exact task/context hit and new current-round local bid;
- geometry, orientation, source epoch, candidate-generation, and map-revision
  key changes as cache misses;
- changed costmap context invalidation;
- failed, aborted, invalid, nonfinite, and contextless results never stored;
- round replacement rebuilding only the local bid while retaining no old peer
  batch or decision;
- deterministic bounded FIFO eviction; and
- preservation of existing final dispatch safety checks.

No build was run. No Webots or ROS experiment was launched.

## Semantics explicitly untouched

The change does not modify snapshot freshness, `bid_batch_valid()` or peer bid
synchronization, `canonical_round_id()`, certificate code, commitment handling,
traffic scheduling, or dispatch safety behavior.
