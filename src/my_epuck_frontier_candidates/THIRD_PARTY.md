# Frontier engine dependency

The candidate generator now links the exported
`frontier_exploration_ros2::frontier_exploration_ros2_core` target from the checked-out
`src/frontier-exploration-ros2` repository (Apache-2.0, version 1.6.0), vendored at the
reviewed upstream commit `476aaf4` (`Support rotated occupancy-grid origins`). The pinned
source is part of this validation checkout so the build cannot silently resolve the dirty
worktree's installed package. It uses the public APIs for:

- frontier extraction;
- costmap-aware accessible goal generation;
- deterministic frontier signatures; and
- lidar-style visible-reveal gain.

The old files under `third_party/frontier_exploration_ros2/` remain as historical migration
material but are no longer included or compiled. Keeping them avoids rewriting the validated
baseline history; the build has a single active frontier core implementation.

The complete upstream autonomous dispatch node is not launched by this package. Nav2 path validation
remains local to the candidate generator and final dispatch remains in each robot's replicated
assignment node. See `LICENSES/Apache-2.0.txt`.

The pinned dependency provenance is intentionally checked before a runtime campaign: any
`frontier_exploration_ros2` include/library path under `/home/arash/webots_ws` is rejected.
