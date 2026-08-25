# Upstream frontier-core adapter contract

The production decentralized launch uses
`frontier_exploration_ros2::FrontierExplorerCore` as the frontier backend.
`frontier_candidate_generator` is a ROS adapter, not a second frontier-search
implementation.

## Runtime contract

The adapter constructs `FrontierExplorerCoreParams` and
`FrontierExplorerCoreCallbacks`, forwards occupancy and global-costmap updates
to `occupancyGridCallback()`/`costmapCallback()`, and obtains the complete
candidate set from `get_frontier_snapshot()`.  The upstream core therefore owns
frontier search, clustering, reachable-goal generation, map-generation cache
keys, rotated-origin conversion, and visible-reveal geometry.

The adapter sets `core_->exploration_enabled = false`.  Its dispatch callback is
deliberately a blocked diagnostic callback.  It must never send a Nav2 action;
`distributed_frontier_assignment` remains the only component allowed to send an
agreed `NavigateToPose` goal.  Do not launch the upstream public
`frontier_explorer` node in the decentralized architecture.

Project-specific code remains only for stable physical-task identity, peer
snapshots/bids/decisions, final asynchronous path and costmap validation,
failure suppression, local/shared phase switching, and publication of the
existing project messages.

## Resource contract

`evaluation_cache_` stores only asynchronous path evidence.  It is pruned to
the current upstream snapshot and has the explicit `maximum_evaluation_records`
capacity (default 256).  The `FRONTIER_EVALUATION_CACHE_BOUND` log is required
evidence for each cycle.  Do not restore an unbounded map of path samples or
per-cell diagnostic records.

Full frontier-cell membership is retained in the upstream snapshot and emitted
only in opt-in diagnostic JSON.  Normal task messages retain the established
project protocol and scalar identity fields.

## Provenance

The local `frontier-exploration-ros2` source is the vendored v1.6.0 API with the
project's rotated-origin and exact-cell retention changes documented in
`THIRD_PARTY.md`.  Before a campaign, rebuild both the upstream package and
`my_epuck_frontier_candidates`, source the clean overlay, and verify the runtime
log contains `UPSTREAM_FRONTIER_CORE_ACTIVE` and no
`UPSTREAM_AUTONOMOUS_DISPATCH_BLOCKED` events.

The initial upstream-core migration is intentionally conservative: the final
project path/costmap safety gate remains while runtime parity is measured.  It
must not be removed merely because upstream generated a candidate.
