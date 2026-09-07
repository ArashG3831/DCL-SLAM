# Certificate latency fix validation

Date: 2026-09-08

## Files changed

- `src/my_epuck_frontier_candidates/src/frontier_candidate_generator.cpp`
- `src/my_epuck_project/launch/two_robots_frontier_candidates_launch.py`
- `src/my_epuck_project/launch/two_robots_decentralized_exploration_launch.py`
- `src/my_epuck_frontier_candidates/test/test_frontier_core.cpp`
- `src/my_epuck_project/test/test_distributed_launch_inventory.py`
- `reports/certificate_latency_fix_plan_20260908.md`

## Exact behavior change

The frontier candidate generator's bounded exact `ComputePathToPose` query cap
changed from 5 to 8 candidates per processing cycle. The production C launch
configurations and the generator default now use 8, matching the existing
`maximum_candidates_before_path_check=8` bound.

The existing deterministic order is unchanged: first-evaluation candidates
remain ahead of refresh work, optimistic lower bounds remain the first numeric
priority, starvation age remains a tie-breaker, and duplicate IDs still consume
only one slot. `send_next()` still stops at the configured cap, preserves planner
timeouts/lease handling, and emits the same bounded lifecycle evidence.

## Tests and build

- Clean isolated `colcon build` of `my_epuck_frontier_candidates` with testing
  enabled: **passed**.
- Focused C++ `test_frontier_core`: **32/32 passed**.
- Focused Python allocator/certificate/launch tests:
  `test_distributed_pair_scoring.py`,
  `test_distributed_frontier_round_guard.py`, and
  `test_distributed_launch_inventory.py`: **108 passed**.
- Build/test environment used ROS Jazzy, the approved Webots-driver prefix, and
  clean `install_canonical_thesis_20260907` dependencies. The generated build
  and install prefixes contain no `/home/arash/webots_ws/install` reference.
- No Webots or ROS experiment was launched.

## Guarantees unchanged

- The cost-only certificate comparison and score definitions are unchanged.
- Matching lower-bound provenance remains mandatory.
- Competitive or missing/invalid unqueried evidence still defers the
  certificate; no candidate is pruned as safe by this change.
- Fail-closed behavior, candidate identity deduplication, planner timeout,
  serialized planner lease, path validation, and final dispatch checks remain
  unchanged.
- No SLAM, Nav2, traffic, handoff, robot-control, observer, evidence cadence,
  or scientific semantic behavior changed.

The only intended effect is earlier acquisition of more exact path evidence;
the certificate can still wait when the available evidence does not prove
domination.

## Expected impact on future 600-second runs

The per-cycle exact-query opportunity increases by at most 60% (8 versus 5),
so competitive unqueried candidates should resolve sooner and certificate-
deferred avoidable idle should decrease. The archaeology does not support a
deterministic idle-reduction percentage: planner service time, lease contention,
candidate churn, and disappearance remain runtime-dependent. No 600-second
experiment was run at this checkpoint; future impact remains to be measured.
