# Decentralized allocator archaeology

Date: 2026-08-15
Repository: `/home/arash/webots_ws`
Branch inspected: `backup/pre-demo-2026-08-15`
Scope: read-only inspection of the production two-robot allocator; no production runtime code was changed.

## Executive result

The authoritative production path is:

```text
/robotN/frontier_candidates (FrontierCandidateArray)
  -> FrontierProposalAdapter
/robotN/task_snapshot (TaskSnapshot)
  -> distributed_frontier_assignment._snapshot_callback
  -> SnapshotLedger / canonicalize_tasks / union_hash
  -> each robot's local ComputePathToPose queries
/robotN/task_bids (TaskBidArray)
  -> choose_pair_assignment()
/robotN/pair_decision (PairDecision)
  -> peer decision matching / semantic agreement
  -> local final ComputePathToPose validation
  -> local NavigateToPose action
  -> terminal result
  -> ExplorationFailure and bounded suppression when needed
  -> round reset and next fresh-snapshot allocation
```

The exact current pair score is:

```text
S = 3.0 * team_visible_gain
    - 1.0 * combined_path_cost
    - 1.5 * nearby_goal_penalty
    - 3.0 * route_overlap_penalty
    - 5.0 * hard_failure_penalty
    - 2.0 * sensing_overlap_penalty
    - 0.35 * workload_imbalance_penalty
```

The important qualifications are:

* `team_visible_gain` is a sum of two independently selected scalar gains. It is not a visible-cell union.
* The live candidate generator computes `information_gain` as the number of deduplicated ray-hit visible frontier cells multiplied by map resolution. Its units are metres, although it is a length proxy rather than a physical frontier length or area.
* The live ROS message does not carry the actual visible-cell set. `PhysicalTask.visible_cells` is therefore empty in the authoritative adapter path.
* Local path cost is the Euclidean polyline length of the returned `nav_msgs/Path`, in metres. It is not NavFn cost, costmap cost, pose count, or time.
* Hard failures are already filtered by feasibility before scoring. Consequently the nominal `-5.0 * hard_failure_penalty` is normally zero for every assignment that reaches the score function; hard failure behaves as a gate/suppression mechanism, not as a useful soft tradeoff.
* The route-overlap term is a symmetric scalar over bounded XY path samples. It has no path intervals, conflict-zone geometry, direction, ETA, or reservation semantics.
* Canonicalization already prevents the two robots from being assigned the same canonical physical task. The nearby-goal term is not needed for that duplicate-task case.

The detailed machine-readable evidence is in [`current_score_terms.json`](./current_score_terms.json), [`failure_semantics.json`](./failure_semantics.json), [`current_protocol.json`](./current_protocol.json), [`traffic_reuse_assessment.json`](./traffic_reuse_assessment.json), and [`proposed_simplification.json`](./proposed_simplification.json).

## Evidence and safety boundary

The preflight checks were:

```text
git branch --show-current
git status --short
git log -1 --oneline
```

They confirmed branch `backup/pre-demo-2026-08-15` and the checkpoint commit `9f3575a Checkpoint: preserve RPP production promotion and diagnostics`. The worktree already contained unrelated modified/untracked files, including Python bytecode, external source trees, world/project assets, and `AGENTS.md`. Those changes were not reset, cleaned, checked out, or discarded.

This report directory is the only intended change from this archaeology pass. No allocator, launch, message, Nav2, Webots, or controller behavior was edited. No long experiment was run.

## 1. Authoritative allocation call chain

### Candidate publication

The authoritative candidate producer is:

```text
src/my_epuck_frontier_candidates/src/frontier_candidate_generator.cpp
```

`FrontierCandidateGenerator::process()` calls the frontier search, computes a safe approach pose, computes visible reveal gain, performs a coarse truncation, queries `ComputePathToPose`, retains reachable candidates, and publishes `FrontierCandidateArray`. Relevant locations are approximately lines 67-134.

The generator publishes only candidates that pass its local path result checks. It does not dispatch `NavigateToPose` and does not publish `cmd_vel`.

### Candidate-to-task adaptation

```text
src/my_epuck_project/my_epuck_project/frontier_proposal_adapter.py
```

`FrontierProposalAdapter._on_candidates()` converts each candidate into a `PhysicalTask`, preserves the candidate's physical geometry, source provenance, `information_gain`, local-path validity/length, and bounded path samples, then publishes a `TaskSnapshot`. The adapter's physical signature quantizes approach, centroid, and bounds at 0.05 m and ignores the local frontier ID. The adapter publishes at most its configured maximum task count, five by default.

The adapter does not copy a visible-cell set or visible bounds into `PhysicalTask`. It sets `visible_reveal_gain` from `candidate.information_gain`, but the live `FrontierCandidate` message has no visible-cell array.

### Snapshot reception and round formation

```text
src/my_epuck_project/my_epuck_project/distributed_frontier_assignment.py
```

`_snapshot_callback()` validates source/session/epoch behavior through `SnapshotLedger`, refreshes the receiver-local freshness deadline for an unchanged immutable retransmission, and records the latest peer/local snapshot. A new source session retires prior provenance and can cancel an active local navigation as a session-restart safety action.

When both snapshots are fresh, `_tick()` computes a round identity from source sessions and epochs, canonicalizes the combined task set, computes the ordered canonical `union_hash`, clears stale bid/decision state for the new union, and enters bidding. Snapshot validity and round binding are separate from the score.

### Canonicalization

```text
src/my_epuck_project/my_epuck_project/distributed_assignment/canonical.py
```

`canonicalize_tasks()` builds a union-find graph over all local and peer `PhysicalTask` instances. `equivalent_tasks()` uses approach distance, centroid distance, bounds IoU/gap, and sampled frontier-geometry overlap. Canonical IDs are deterministic hashes of quantized physical geometry; the canonical union hash is a deterministic hash of ordered canonical IDs.

Canonical tasks deduplicate frontier geometry and visible cells if visible cells exist. Their visible gain is currently the maximum member gain, not the sum. Because both peers canonicalize the same combined snapshots and bind their decisions to the same union hash, a physical frontier represented by both robots becomes one canonical task.

### Per-robot bid generation

`_continue_bidding()` in `distributed_frontier_assignment.py` asks each robot's local `LocalNav2` wrapper for a path to each canonical task that is not already represented by a valid candidate path. If a canonical member has a valid local candidate path, that path is reused; otherwise the robot queries its own `ComputePathToPose` action. Each robot therefore evaluates its own travel cost. It does not ask the peer to compute a path and does not use the peer's path cost.

The resulting `TaskBidArray` contains one robot's bid for the current round and union. Each bid contains path validity, path length, estimated travel cost, heading, own utility, and bounded XY path samples.

### Pair enumeration, decision, agreement, dispatch

After both valid bid batches arrive, `_tick()` calls `choose_pair_assignment()`. The selected result is serialized into `PairDecision`, including both task IDs, both snapshot epochs/fingerprints, score diagnostics, and a decision hash. Each peer accepts the other decision only when source/session, round, union hash, epochs, fingerprints, task IDs, and decision hash match. This is semantic agreement, not merely receipt of the same topic message.

After a matching decision is observed, the local node commits the round. If its task is non-IDLE, `_start_local_dispatch()` performs a final local path query and dispatch-precondition checks, then calls `LocalNav2.send_navigation()` for `NavigateToPose`. The node itself never commands the other robot.

On terminal navigation success it clears the active goal and waits for the configured post-terminal settle period before a new allocation. On non-success it publishes `ExplorationFailure`, records local suppression state, clears the active goal, invalidates the round, and waits for fresh allocation inputs.

## 2. Exact frontier gain

### Authoritative equation

The authoritative production implementation is:

```text
S(c) = set of unique map cells that are the first unknown cell hit by a ray
       from candidate c's approach pose, where the hit cell also passes
       frontier-point eligibility checks.

G(c) = |S(c)| * map_resolution_m
```

This is implemented by `compute_visible_reveal_gain()` in:

```text
src/frontier-exploration-ros2/src/frontier_search.cpp:802-916
src/frontier-exploration-ros2/include/frontier_exploration_ros2/frontier_search.hpp:209-220
```

The function creates a per-scan cell stamp set. For each ray, it marches from the approach pose up to the configured visible-gain range, using a step no larger than half a map-resolution cell. It stops at map bounds, occupied cells, or the first unknown cell. An unknown cell contributes only if `is_frontier_point()` accepts it, and `mark_visible_frontier_cell_once()` prevents multiple rays from counting that cell more than once. The returned structure contains `visible_reveal_cell_count` and `visible_reveal_length_m = count * resolution`.

The generator calls this function in:

```text
src/my_epuck_frontier_candidates/src/frontier_candidate_generator.cpp:76
```

and assigns `x.gain` from `visible_reveal_length_m`. The candidate message field `information_gain` is then that value.

### Raw inputs

The calculation uses:

* the occupancy grid and its resolution;
* the candidate safe approach pose and heading;
* configured range, field of view, and ray angular step;
* occupied-cell threshold and frontier eligibility logic;
* map bounds and cell coordinates.

It is not computed from frontier cluster length, frontier cell count, total unknown-cell count, expected occupancy area, or a generic Euclidean radius. Frontier cluster `cell_count` and `frontier_length_m` are separate candidate fields.

There is a second helper, `frontier_information_gain(frontier) = frontier.size()`, in `mrtsp_ordering.cpp`. That is a separate MRTSP-related cell-count utility and is not on the current `FrontierCandidateGenerator` publication path, which calls `compute_visible_reveal_gain()` directly. It must not be mistaken for the authoritative allocator gain.

### Units and thresholds

`|S(c)|` is a cell count. The published/current allocator gain is `|S(c)| * resolution`, so its units are metres. The reason is exact: the implementation multiplies the deduplicated integer count by `occupancy_map.info.resolution`; there is no second multiplication by resolution and no area calculation. Semantically, it is a one-dimensional visible-frontier-cell proxy, not a measured line length along a frontier and not square metres.

The following transformations occur before assignment:

1. Generator-level visible-gain geometry validation rejects range below 0.1 m, invalid FOV, or ray step outside its allowed range.
2. Generator coarse truncation normalizes gain over the current candidate batch using `(value - min)/(max - min)` and uses it with Euclidean distance and heading to keep at most the configured pre-path candidate count. This is a truncation ranking, not the distributed pair score.
3. Final generator ordering again normalizes gain over reachable candidates and sorts by coarse/final score, gain descending, path ascending, and stable ID.
4. The adapter copies the scalar without further gain normalization.
5. Canonicalization retains the maximum member scalar gain.
6. Pair scoring divides each canonical gain by `visible_gain_scale_m = 5.0` and clamps to `[0,1]` per selected task.
7. Feasibility rejects a task if its finite gain is below `minimum_visible_gain_m = 0.05`.

### Are visible sets retained?

Internally, the raycast has a deduplicated set-like stamp structure for the duration of one gain computation. The public return value retains only the count and metre-valued length. `FrontierCandidate.msg` has no visible-cell set field. `FrontierProposalAdapter._on_candidates()` therefore leaves `PhysicalTask.visible_cells` empty and does not populate visible bounds from the raycast.

Canonicalization can union `visible_cells` and visible bounds if a future producer supplies them, but in the current live path there are no cells to union. It also keeps only the maximum gain scalar. Therefore:

* exact `|V_A|`, `|V_B|`, intersection, and union are not available from current task snapshots;
* `|visible_cells_A union visible_cells_B|` cannot be computed without reconstructing visibility or extending the producer/message/adapter path;
* the current sensing-overlap fallback cannot recover the missing exact set.

The technically supported future set-based design is therefore possible, but it requires explicit data plumbing first. It is not a property of the current runtime messages.

## 3. Exact path cost and feasibility

### Cost definition

There are two equivalent implementations of the same metric:

```text
path_length(path) = sum over consecutive returned poses p[j-1], p[j]
                   of hypot(p[j].x-p[j-1].x, p[j].y-p[j-1].y)
```

The C++ candidate-side implementation is `path_length()` in:

```text
src/my_epuck_frontier_candidates/src/candidate_utils.cpp:13
```

The Python allocator-side implementation is `path_length()` in:

```text
src/my_epuck_project/my_epuck_project/distributed_assignment/local_nav2.py:83-85
```

The result is a geometric path length in metres. It is not the number of poses, Euclidean start-to-goal distance, accumulated NavFn cost, costmap cost, or time estimate.

### Sources and retained geometry

The candidate generator first uses its own `ComputePathToPose` client. The path is accepted only when the action status is `SUCCEEDED`, the result error is `NONE`, all coordinates are finite, and the final returned pose is within the configured 0.03 m goal tolerance. A nonempty accepted path is measured in full, then at most 16 index-spaced XY samples are retained in the candidate message.

For canonical tasks, the allocator reuses a member's valid local path when possible. Otherwise `LocalNav2.compute_path()` calls `ComputePathToPose`; it measures the complete returned path and retains at most 32 index-spaced XY samples in the `Bid`. The distributed wire does not retain a complete `nav_msgs/Path`: no full pose sequence, orientation sequence, header, timestamps, or frame field survives bidding. The current convention is that the samples are in `shared_map`.

At final dispatch, the allocator queries a fresh local path again and sends the final goal to `NavigateToPose`. Nav2 then owns the complete internal path. Thus the current bid path is useful bounded geometry, but it is not sufficient to reproduce the full Nav2 path exactly.

### Failure/empty endpoint behavior

* Empty path: invalid; no feasible bid.
* One-pose path: valid with length zero only if that pose is already within the goal tolerance; otherwise invalid.
* Endpoint outside the requested goal tolerance: invalid even if the path has poses.
* Non-finite coordinates: invalid.
* Action rejection, timeout, error result, or non-success action status: invalid and mapped to failure evidence by the local wrapper.
* Generator path failures suppress the stable candidate ID temporarily, except `GOAL_OCCUPIED` and `GOAL_OUTSIDE_MAP`, which persist until a different map revision is observed.

### Current limits and normalization

The allocator's hard feasibility limit is `maximum_path_length_m = 18.0` for both `bid.path_length_m` and `bid.estimated_travel_cost`. The local bid query count is capped at eight. Local path-query timeout is 1.5 s by the assignment wrapper's default; the generator has a separate one-second default.

The score uses `clamp(path_length_m / 12.0, 0, 1)` per selected bid. Therefore costs above 12 m saturate the score penalty, while the separate 18 m gate still rejects longer paths. No speed or duration normalization is performed.

Each robot computes only its own path costs. The peer's path cost is transported as a bid and used in the replicated pair computation, but the peer does not compute or validate the other robot's path.

## 4. Exact pair enumeration

For each robot, the solver constructs:

```text
choices(robot) = [IDLE] + sorted(canonical task IDs with a valid feasible bid)
```

It enumerates the Cartesian product of the two choice lists. It skips a pair when both active choices have the same canonical task ID. IDLE is therefore considered for each robot. Both robots cannot receive the same canonical task through this solver, independently of canonicalization.

The actual selection policy is:

```text
all_pairs = []
for robot1_choice in [IDLE] + sorted(robot1_feasible_task_ids):
    for robot2_choice in [IDLE] + sorted(robot2_feasible_task_ids):
        if both choices are the same non-IDLE canonical ID:
            continue
        score = _score_assignment(pair)
        record score and raw path tie-break values

non_idle = pairs with at least one active task
conflict_free_two_active = pairs with both active and
    nearby_goal_penalty == 0 and route_overlap_penalty == 0 and
    sensing_overlap_penalty == 0 and hard_failure_penalty == 0

if conflict_free_two_active is nonempty:
    candidates = conflict_free_two_active
elif non_idle is nonempty:
    candidates = non_idle
else:
    return IDLE/IDLE

return min(candidates, key=(
    -round(total_score, 12),
    round(combined_raw_path_length, 12),
    round(maximum_raw_path_length, 12),
    robot1_task_id,
    robot2_task_id,
))
```

The conflict-free two-active preference is applied before utility maximization. It is not merely a score coefficient. If no pair passes that exact zero-term test, the best non-IDLE pair wins by score and deterministic tie-breaks. There is no general hard preference for two active assignments.

With `K` feasible tasks per robot, the full worst-case enumeration is `(K+1)^2 - K = K^2 + K + 1`, because the `K` same-task active pairs are skipped. The configured canonical union is capped at 10 tasks.

Robot order is fixed as robot 1 then robot 2. Task order does not affect the result because IDs are sorted and maps are keyed by canonical ID. `union_hash` binds the round and prevents decisions from different task unions from matching; it does not affect ranking. Same canonical task assignment is prevented both by canonical task formation and by the solver's explicit pair skip.

## 5. Exact seven score terms

The complete term-by-term record, including source locations, ranges, constants, and removal effects, is in [`current_score_terms.json`](./current_score_terms.json). The essential details are below.

| Term | Exact current computation | Normalized range / coefficient | Semantics | Removing it |
|---|---|---:|---|---|
| Team gain | Sum `clamp(task.visible_reveal_gain / 5.0, 0, 1)` over selected tasks | `[0,2]`, `+3.0`, contribution `[0,6]` | Exploration | Changes ranking only; the separate 0.05 m feasibility gate remains |
| Path cost | Sum `clamp(bid.estimated_travel_cost / 12.0, 0, 1)` | `[0,2]`, `-1.0`, contribution `[-2,0]` | Exploration / travel preference | Changes ranking only; local path feasibility and dispatch remain |
| Nearby goal | `clamp(1 - distance(approach_1, approach_2)/0.60, 0,1)` | `[0,1]`, `-1.5`, `[-1.5,0]` | Mixed separation heuristic; not collision safety | Changes ranking and the conflict-free preselection test; does not enforce safety |
| Route overlap | `clamp(0.5*(one_way(path1,path2)+one_way(path2,path1)),0,1)` with point threshold `0.32 m` | `[0,1]`, `-3.0`, `[-3,0]` | Traffic-like geometry used as exploration ranking | Changes ranking and conflict-free preselection; no Nav2/protocol safety guarantee |
| Hard failure | Count selected canonical IDs present in `hard_failed`, clamped to `[0,1]` | `[0,1]`, `-5.0`, `[-5,0]` | Navigation health / suppression | In current solver selected hard-failed tasks are already infeasible, so normally no ranking effect; removing only the soft term is safe if the gate remains |
| Sensing overlap | Exact quantized-cell Jaccard if both sets nonempty; otherwise `0.65*bounds_IoU + 0.35*approach_proximity` | `[0,1]`, `-2.0`, `[-2,0]` | Information redundancy heuristic | Changes ranking and conflict-free preselection; exact set branch is inactive with current live messages |
| Workload imbalance | `abs(path_length_1-path_length_2) / max(path_length_1+path_length_2, 1e-6)`, clamped | `[0,1]`, `-0.35`, `[-0.35,0]` | Fairness / load balancing | Changes ranking only; no feasibility, protocol, or dispatch-safety role |

More exact observations:

* Gain and path are summed across active tasks; the other terms are pair-level terms.
* `minimum_useful_score = 1e-6` is configured but unused in authoritative pair selection.
* The hard-failure score branch is effectively defensive/diagnostic because `_task_feasible()` rejects hard-failed task IDs before enumeration.
* A score term can affect which already-feasible assignment is selected, but it does not by itself validate paths, establish protocol agreement, or prevent physical collision.

## 6. Route overlap: what is reusable

The implementation is `route_overlap()` in:

```text
src/my_epuck_project/my_epuck_project/distributed_assignment/scoring.py:80-98
```

For every sample in path A it checks whether any sample in path B is within `2 * route_corridor_radius_m`; the default radius is 0.16 m, so the point-to-point threshold is 0.32 m. It computes this in each direction and averages the two fractions.

It compares the ordered bounded XY sample tuples carried in the two bids. Those are not full Nav2 paths and are not guaranteed to be equally spatially sampled: the generator and LocalNav2 use index-spaced downsampling, with maxima of 16 and 32 points respectively. There is no fixed metre sampling interval.

The current scalar cannot distinguish reliably between:

* a single crossing and a long common corridor;
* same-direction and opposite-direction travel;
* an early conflict and a late conflict;
* a conflict that is geometrically close but temporally separated.

It discards matched indices and returns no overlap geometry, path-distance interval, entrance, exit, or conflict-zone object. It uses no ETA, speed, current pose, reservation, occupancy, or time window. A bid timestamp exists but is not used by `route_overlap()`.

The helper is therefore a useful geometric seed for a future `detect_conflict_zone(path1,path2)`, but not a traffic scheduler. A refactor could retain matched sample indices, group contiguous runs, and convert them into path-distance intervals. It would additionally need a common footprint/clearance criterion, common map/costmap evidence, fresh poses, a deterministic ETA model, and reservation/release state. No Nav2 internals need to be changed merely to compute such a zone.

Current local costmap geometry is not enough for the existing route helper to determine that a shared section is too narrow for two robots to pass. The helper receives no costmap. Local costmaps are local to each `LocalNav2`; their geometry and footprint-clearance decisions are not part of the pair-decision agreement. The production Collision Monitor has a 0.08 m stop circle and lidar input, but that is reactive contact prevention, not narrow-doorway ordering or deadlock resolution.

## 7. Sensing overlap and information redundancy

The implementation is `_visible_cell_overlap()` and `sensing_overlap()` in:

```text
src/my_epuck_project/my_epuck_project/distributed_assignment/scoring.py:109-133
```

If both tasks contain visible-cell arrays, each coordinate is quantized to 0.05 m and the score is Jaccard overlap:

```text
J(V_A,V_B) = |Q(V_A) intersection Q(V_B)| / |Q(V_A) union Q(V_B)|
```

If either set is empty, the implementation falls back to:

```text
0.65 * bounds_IoU(visible_bounds_or_task_bounds)
 + 0.35 * clamp(1 - approach_distance / max(1.5 m, 1e-9), 0, 1)
```

The fallback is bounding-box and approach geometry, not actual visible unknown cells, raycast distance, or centroid distance alone. Canonicalization can deduplicate/union visible-cell coordinates that are supplied, but the live candidate adapter does not supply them. Thus the exact Jaccard branch is not active for current production task snapshots; the current sensing term is normally the fallback geometry score.

If exact sets are added, their natural units are cells. Multiplying cardinality by `resolution^2` gives square metres, but the current project gain is not such an area. The best-supported future options are therefore:

1. preserve a Burgard-style visibility reduction over a scalar utility if no set data is added; or
2. add explicit `V_t` data and use set cardinality/union/overlap as a clearly labelled project adaptation.

The source supports option 2 technically only after a producer/message/adapter change. It does not support claiming that the current scalar gains can be unioned exactly.

## 8. Nearby-goal penalty

`nearby_goal_penalty()` is:

```text
clamp(1 - distance(approach_pose_1, approach_pose_2) / 0.60 m, 0, 1)
```

It is nonzero when two simultaneously selected approach poses are within 0.60 m. It is a soft task-separation heuristic. It does not inspect robot footprint, corridor width, occupancy, direction, or timing, so it is not a collision guarantee. It also overlaps conceptually with two mechanisms that already exist or are planned:

* canonicalization merges equivalent physical tasks before pair scoring;
* explicit visible-set redundancy can represent information overlap directly;
* true route exclusion belongs in a traffic layer.

Once canonical merging and explicit information redundancy are authoritative, this term appears redundant as an exploration term. Removing it also removes its contribution to the current conflict-free-two-active preselection; that preselection should then be replaced by an explicit traffic decision rather than silently retained as a score side effect.

## 9. Workload imbalance

The current workload term is:

```text
abs(path_length_robot1 - path_length_robot2)
  / max(path_length_robot1 + path_length_robot2, 1e-6)
```

clamped to `[0,1]`, with a `-0.35` coefficient. It is based only on the two selected raw path lengths. It is not assignment count, historical workload, current active workload, idle duration, task completion count, or estimated navigation time.

It can select a lower-total-gain or higher-total-cost pair when the score difference is small enough for the fairness term to change the ordering. There is no protocol or safety reason requiring it. Removing it changes only ranking and makes the allocation objective less likely to trade information value for visual path-length balance.

## 10. Failure semantics

The complete structured record is in [`failure_semantics.json`](./failure_semantics.json). The production behavior is as follows.

### Action success and status values

The ROS 2 action status values are:

```text
UNKNOWN=0, ACCEPTED=1, EXECUTING=2, CANCELING=3,
SUCCEEDED=4, CANCELED=5, ABORTED=6
```

`ComputePathToPose` succeeds only when status is `SUCCEEDED`, a result exists, the error code is `NONE (0)`, and the path contains a valid pose. `NavigateToPose` succeeds only when status is `SUCCEEDED` and the result error code is `NONE (0)`. The allocator's terminal check is status 4 and error code 0.

`ABORTED` is not automatically a hard failure. The failure classifier uses the available evidence:

* `GOAL_OCCUPIED`, `GOAL_OUTSIDE_MAP`, and `NO_VALID_PATH` map to `HARD_UNREACHABLE`;
* planner error results map to `PLANNER_FAILURE`;
* the local no-progress watchdog maps to `CONTROLLER_NO_PROGRESS`;
* explicit dynamic-obstacle evidence would map to `DYNAMIC_BLOCKAGE`, but current `LocalNav2` does not generate that evidence;
* action timeout maps to `TIMEOUT`;
* explicit cancellation maps to `EXPLICIT_CANCELLATION`;
* rejection maps to `ACTION_REJECTION`;
* generic aborted/exception outcomes map to `UNKNOWN` unless a stronger direct cause exists.

Planner failure, controller no-progress, recovery exhaustion, and dynamic blockage are not all separately observed in current production. Recovery exhaustion is not a distinct current result class. The `failures.py` enum/classifier is broader than the evidence emitted by the current wrapper.

### Association and storage

The allocator associates a failure with:

* the canonical task ID on the wire;
* the physical signature and approach pose;
* the current round/session metadata;
* the selected task's member signatures locally;
* bounded path samples and Nav2 code/message evidence.

Local failures are stored in the receiver-local `_failure_memory`. Peer failure messages are accepted as peer-originated health evidence and contribute to the local hard-failed set. Suppression is effectively both local and replicated as evidence, but there is no central team database.

`_hard_failed_task_ids()` checks exact physical signature membership among canonical-task members and receiver-local expiry. This is not a broad radius-based regional blacklist. The separate `FailureSuppressor` utility can use repeated evidence and alternate-approach logic, but production grep shows the node uses its direct signature/TTL dictionary rather than that utility as the authoritative gate.

### Persistence and retry

Allocator failure records expire using receiver-local monotonic time. Wire validity is 15 s for hard failures and 2 s for non-hard failures; local suppression is bounded and can repeat with durations:

```text
requested 15 s: count 1 -> 15 s, count 2 -> 30 s,
                count 3 -> 60 s, count 4+ -> 120 s
```

The candidate generator has separate suppression: generic path failures/timeouts use the default 7 s; `GOAL_OCCUPIED` and `GOAL_OUTSIDE_MAP` remain suppressed until a different map revision is seen. Its suppression table is capped at 128 entries.

Therefore a failure does not permanently blacklist a physical region: allocator suppression is bounded, and candidate-generator persistent suppression has a map-revision escape. A changed map does not explicitly clear the allocator's dictionary immediately, but the allocator's bounded expiry prevents indefinite suppression.

### Gate versus score

`_task_feasible()` rejects a bid if it is missing/invalid, hard-suppressed, has non-finite or too-small gain, or has a non-finite/negative/over-18 m path/travel cost. The pair solver receives only feasible bids. The nominal hard-failure penalty is thus not reached for a selected hard-failed task in normal production flow.

The safest simplification is to preserve the existing exact-signature bounded gate and retry/expiry semantics, remove hard failure from the exploration score, and retain failure telemetry. A future gate must preserve expiry/map-change re-entry; replacing it with a permanent canonical-ID blacklist could cause a region to be missed after the map changes.

## 11. Decentralized protocol

The exact topic/message map is also in [`current_protocol.json`](./current_protocol.json). The core topics are:

| Topic | Type | Publisher/role |
|---|---|---|
| `/robotN/frontier_candidates` | `FrontierCandidateArray` | Local C++ frontier candidate generator |
| `/robotN/task_snapshot` | `TaskSnapshot` | Local proposal adapter; transient-local snapshot |
| `/robotN/task_bids` | `TaskBidArray` | Local allocator's robot-specific bids |
| `/robotN/pair_decision` | `PairDecision` | Local allocator's complete pair decision |
| `/robotN/distributed_status` | `DistributedExplorationStatus` | Local allocator status/heartbeat |
| `/robotN/exploration_failure` | `ExplorationFailure` | Local allocator failure evidence |
| `/robotN/distributed_event` | `DistributedEvent` | Local transition/diagnostic event stream |

The allocator subscribes to both absolute robot namespaces for snapshots, bids, decisions, status, and failures. It publishes into its own namespace. The peer is never given a command goal; each peer owns only its own `NavigateToPose` action client.

### Identity and freshness

* Source identity identifies the robot/source publisher.
* Session UUID distinguishes a restarted source session.
* Epoch identifies a new snapshot from a session.
* Round ID binds the pair of source sessions and current epochs.
* Map revision/fingerprint and task content fingerprint detect snapshot changes.
* `union_hash` binds the exact canonical task set.
* Bid and decision validity/TTL are clamped by protocol helpers; launch defaults use approximately 8 s for task/bid/decision validity and 12 s peer timeout, while the protocol guards values to a 0.1-10 s range where applicable.

`SnapshotLedger` rejects stale/mismatched source provenance and compares the two snapshots. `bid_batch_valid()` requires fresh local/peer snapshots, correct source/session/epoch, round, union hash, bounded task count, and no duplicate task IDs. Decisions match only when both peers agree on the complete semantic identity described above.

An unchanged immutable snapshot retransmission refreshes receiver-local freshness and does not create needless new work. An unchanged IDLE result has special no-churn handling. A peer session restart is exceptional and can cancel local navigation to avoid mixing rounds.

### Does a score rewrite require protocol changes?

No, not if the replacement solver consumes the same canonical tasks and robot-local bids and emits the same deterministic `PairDecision` fields, task IDs, fingerprints, union hash, and decision hash. The protocol is designed to bind a deterministic assignment result, not to a particular scoring formula.

An exact visible-set union adaptation would require a data-path extension because current live candidates carry no visible sets. That extension can be made without changing the agreement concept, but it would change message/task payloads and fingerprints and therefore must be versioned/tested. A scalar utility rewrite can preserve the proven protocol unchanged.

## 12. Goal lifecycle and reallocation

The production lifecycle is terminal-driven:

```text
matching PairDecision
  -> final local ComputePathToPose
  -> NavigateToPose
  -> terminal result
  -> success: clear active state, settle, next round
  -> failure: publish/store failure, clear active state, invalidate round
  -> fresh snapshots and next allocation
```

An active goal is not preempted by ordinary newer frontier snapshots. `_tick()` returns while dispatch is active or a local navigation is active. The new snapshots are retained, and allocation resumes after terminal handling plus the post-terminal settle period. This prevents constant goal churn.

An active goal is canceled on the exceptional peer source-session restart path. It is not canceled merely because the selected frontier disappears, becomes explored, or becomes redundant due to peer progress. Nav2 is allowed to finish or fail the active goal; then the next round uses current snapshots.

The node does not wait for both robots to finish terminal goals before beginning all future allocation, but it does wait for the local active goal to end and for the peer's fresh snapshot/decision state to be valid. Both peers can be assigned in one pair, and each local peer dispatches only its own assignment.

## 13. Frontier canonicalization and duplicate physical tasks

Canonicalization is not merely a display ID. It is a task-equivalence and agreement mechanism:

1. local frontier IDs are source-local provenance, not authoritative physical identity;
2. physical signatures use quantized approach/centroid/bounds geometry;
3. union-find joins equivalent local/peer tasks using approach, centroid, bounds, and sampled geometry thresholds;
4. canonical IDs are deterministic physical-geometry hashes;
5. canonical tasks combine member geometry and preserve the maximum current gain;
6. `union_hash` binds the ordered canonical task set into the distributed round.

The canonical equivalence thresholds include approximately 0.25 m approach tolerance, 0.60 m centroid tolerance, 0.20 IoU/bounds evidence, 0.35 sampled-geometry overlap, and 0.12 m bounds-gap/geometry tolerances, with a strong-IoU path at approximately 0.45. See `canonical.py` for the exact predicates.

Yes: canonicalization already solves the primary “both robots selected the same physical frontier” problem before scoring. The pair solver additionally skips identical canonical IDs. A future Burgard-style allocator does not need another nearby-goal penalty to prevent duplicate canonical task assignment.

## 14. Classification: what can change later

### Keep as infrastructure

* candidate publication and local candidate-generator path feasibility;
* physical signatures, source sessions, snapshot epochs, TTLs, and fingerprints;
* canonical geometry equivalence, union-find, canonical IDs, and `union_hash`;
* each robot's local Nav2 path computation and query lock;
* bounded bid samples and path fingerprints;
* deterministic `PairDecision` matching and round agreement;
* local NavigateToPose ownership and final path validation;
* lifecycle/TF/map/costmap dispatch preconditions;
* post-terminal settle and fresh-snapshot gating;
* failure evidence, bounded suppression, event/status telemetry.

### Replace as exploration heuristic

* the weighted `AssignmentScore` objective in `scoring.py`;
* the current coefficient interpretation of 5.0 m gain scale and 12.0 m path scale;
* the sensing-overlap weighted penalty, if explicit information redundancy is made available.

### Move to a traffic layer

* route-overlap as an exploration-score penalty;
* path-sample matching, after it returns conflict-zone intervals rather than one scalar;
* occupied-zone state, ETA, priority, hold-point, reservation, and release behavior.

### Likely remove from exploration scoring

* nearby-goal penalty after canonical merging and explicit redundancy are authoritative;
* workload imbalance, because it is path-length fairness only and can override information ranking;
* hard-failure penalty as a soft term, while retaining hard failure as an expiring feasibility/suppression gate;
* route-overlap penalty as an exploration term.

Removing these from the score does not remove canonicalization, path validation, protocol agreement, or local Nav2 safety. Removing route overlap from the score does remove the current special “conflict-free two-active pair first” condition; that behavior must be replaced explicitly by traffic scheduling if it is still required.

## 15. Literature mapping

The implementation is not identical to any of these papers. The table identifies the closest conceptual anchor only.

| Current project mechanism | Closest literature concept | Disposition | Reason |
|---|---|---|---|
| Raycast visible-frontier scalar | Burgard et al. 2005 frontier utility / expected information value | Replace representation, keep candidate infrastructure | Current scalar is a metre-valued ray-hit proxy, not the paper's exact utility definition |
| Robot-specific Nav2 path length | Burgard et al.; Zlot et al. information value minus travel cost | Keep source, replace scaling | Local path geometry is useful, but raw metres are not automatically dimensionless with utility |
| Sensing overlap | Burgard-style utility reduction after assignment | Replace with explicit set/region reduction when data exists | Current live task has no visible-cell set |
| Nearby-goal penalty | Heuristic task separation | Likely remove | Canonical task merging and future traffic handling cover its intended cases more explicitly |
| Route overlap | Chandra et al. / constrained-space traffic scheduling | Move | It is traffic-like geometry, but current scalar lacks zone, ETA, priority, and release |
| Hard failure memory | Zlot-style unreachable-goal abandonment / retry suppression | Keep as gate | It is navigation health, not exploration value; preserve bounded retry/map-change semantics |
| Workload balance | Market/fairness extension, not core information utility | Remove from exploration objective | It is based only on pair path-length imbalance and can distort information choice |
| Canonical same-task prevention | Distributed task identity/negotiation infrastructure | Keep | It prevents duplicate physical assignments before ranking |
| TaskBid/PairDecision agreement | Distributed market/task negotiation | Keep | It provides deterministic replicated selection and semantic matching |

References used as conceptual anchors:

* Burgard, Moors, Stachniss, Schneider, “Coordinated Multi-Robot Exploration,” IEEE Transactions on Robotics, 2005: [paper PDF](https://klara.student.utwente.nl/~stephan/References/CoordinatedMultiRobotExploration.pdf), DOI `10.1109/TRO.2004.839232`.
* Zlot et al., “Market-Driven Multi-Robot Exploration”: [CMU publication page](https://publications.ri.cmu.edu/market-driven-multi-robot-exploration), [PDF](https://publications.ri.cmu.edu/storage/publications/pub_files/pub3/zlot_robert_michael_2002_1/zlot_robert_michael_2002_1.pdf).
* Chandra et al., “Decentralized Social Navigation with Non-Cooperative Robots via Bi-Level Optimization,” 2023: [arXiv](https://arxiv.org/abs/2306.08815).

## 16. Traffic-scheduler feasibility

The current architecture can support a small pre-dispatch traffic gate, but it does not yet contain all inputs needed for a safe independently replicated narrow-zone scheduler.

### Available now

* Both peers retain current-round bid samples for both selected tasks after receiving the peer's bid batch.
* The sampled paths are in the shared-map convention and can support deterministic geometric proximity if both peers use the same samples and thresholds.
* Local Nav2 already has costmap/TF and collision-monitor safety behavior.
* The allocator has a natural point at which to gate dispatch: after semantic pair agreement but before the local `NavigateToPose` send.

### Missing or insufficient now

* There is no peer-pose field, peer-pose topic, pose freshness rule, or protocol-bound current pose in the allocator.
* Bids contain bounded samples, not full `nav_msgs/Path` geometry with frame/timestamps/orientations.
* There is no common footprint/clearance or shared costmap width test in route overlap.
* There is no deterministic ETA model. Path length exists, but start pose, speed, and time are not bound into the decision.
* There is no safe hold-point selector.
* `LocalNav2` exposes send/cancel, not a pause/resume gate. Publishing zero velocity from the allocator would violate the current ownership boundary and could trigger Nav2 progress checking; Collision Monitor is not a reservation API.

### Safe minimum direction

For a first demo-oriented scheduler, delay dispatch of the lower-priority selected goal rather than cancel an active Nav2 goal or inject velocity commands. A separate gate can reserve a known conflict interval, keep the winner's normal goal, and release the loser only after an explicit winner-clear heartbeat. If common pose/clearance evidence is unavailable, conservative serialization of the two conflicting dispatches is safer than guessing a hold point.

The proposed priority remains:

```text
1. robot already occupying the conflict zone;
2. otherwise lower ETA to the conflict-zone entrance;
3. near-equal ETA: lower robot ID.
```

Robot ID must remain only the deterministic tie-break. The existing route scalar cannot establish “already in zone” or ETA. A conflict that emerges after a goal is already active should not be handled by the first version through cancellation or allocator-level zero velocity; that requires a real reservation/active-goal state machine.

## 17. Recommended next implementation architecture

This is a recommendation for the next task, not an implementation performed here.

### A. Leave unchanged

Leave the candidate generator, adapter provenance, snapshot ledger, canonicalization, local path feasibility, per-robot bidding, protocol matching, local dispatch ownership, final path validation, failure telemetry, and bounded retry/expiry behavior unchanged.

### B. Replace

Replace only the exploration-ranking portion of `distributed_assignment/scoring.py` and the deterministic pair-selection policy that consumes it. Keep the same inputs/outputs first so protocol hashes and agreement behavior remain stable.

### C. Burgard literal versus project adaptation

Do not implement the literal expression `U_t - beta * V_t^i` by subtracting the current `information_gain` metres from current path-length metres. Those values happen to both carry metre units, but current gain is a ray-hit count multiplied by resolution, while path is a geometric travel length; equal units do not make equal semantics or a literature-backed beta=1.

The technically best project adaptation is to make the visible-cell/expected-visible region explicit first, then use a dimensionless information utility and reduce the remaining task utilities by the documented expected-visible set/region after each assignment. Until the visible sets are actually transported, use a clearly documented scalar visibility utility and do not claim an exact visible-cell union.

### D. Scaling path cost

Define a dimensionless travel cost explicitly, for example:

```text
C_i,t = path_length_i,t_m / calibrated_distance_scale_m
```

Then `beta=1` has a declared normalization meaning, not an accidental one. A speed-based alternative is `path_length / nominal_speed`, followed by an explicitly declared information-per-time convention. The existing hidden interpretation “gain divided by 5, path divided by 12, with unrelated coefficients” should not be silently relabelled as Burgard beta=1.

### E. Failure gate

Filter active exact-signature hard suppressions before ranking. Retain bounded expiration, repeated evidence escalation, and map-change/alternate-signature re-entry. Remove `-5 * hard_failure_penalty` from the soft exploration objective.

### F. Terms to remove

Remove nearby-goal, route-overlap, sensing-overlap fallback, workload imbalance, and hard-failure weighted terms from the exploration objective as their responsibilities become explicit. Keep sensing information only as a utility/redundancy representation, not as an arbitrary coefficient, and move route conflict to traffic scheduling.

### G. Route reuse

Refactor the current path-sample matching into a helper that returns matched sample runs and path-distance intervals. Do not call that helper a narrow conflict zone until footprint/clearance and common map evidence are available.

### H. Smallest doorway scheduler

Implement a separate post-agreement, pre-dispatch gate that can conservatively serialize two selected goals known to share a conflict interval. Use a single reservation owner, deterministic winner selection, explicit release heartbeat, and no allocator-level `cmd_vel`. For already-active goals, defer the feature until pose freshness, reservation state, and safe stop/hold behavior are specified.

### I. Immediate tests after the rewrite

Run the existing focused tests, then add tests for:

* dimensionless utility/path scaling and beta interpretation;
* one-active, two-active, IDLE, and same-canonical-task cases;
* deterministic result under task-order and retransmission permutations;
* explicit visible-set cardinality/intersection/union and utility reduction;
* hard-failure gate expiry, repeated evidence, and map-change re-entry;
* protocol matching with unchanged message/session/epoch/union semantics;
* route interval extraction independently from exploration score;
* pre-dispatch traffic serialization and reservation release, if the traffic gate is implemented.

The focused current test command is:

```bash
python3 -m pytest -q \
  src/my_epuck_project/test/test_distributed_pair_scoring.py \
  src/my_epuck_project/test/test_distributed_task_canonicalization.py \
  src/my_epuck_project/test/test_distributed_protocol.py \
  src/my_epuck_project/test/test_distributed_failures_and_telemetry.py
```

After production code is changed in the next task, build only the project package:

```bash
source /opt/ros/jazzy/setup.bash
colcon build --packages-select my_epuck_project --symlink-install
source install/setup.bash
```

Then run the focused tests again and perform a short two-robot smoke test using Webots port 23000, not the blocked default 1234. No build or runtime test was needed for this report-only pass because no production code changed.

## Remaining ambiguities

The source evidence resolves the requested allocator behavior. The remaining limitations are architectural rather than unexplained:

1. Exact visible-cell sets are not available in current live task messages; they would need producer/message/adapter plumbing.
2. The distributed bid path is bounded XY samples, not complete `nav_msgs/Path` geometry.
3. Peer pose freshness and common costmap/footprint width are not part of the allocator protocol, so a narrow-zone scheduler cannot yet be independently proven identical at both peers.
4. Generic Nav2 `ABORTED` results are not intrinsically hard failures; the classifier needs direct error evidence to make that distinction.
5. The separate `FailureSuppressor` utility contains richer repeated-evidence logic than the direct production allocator dictionary; it is not currently the authoritative runtime storage path.

These are the only material caveats found. The current production allocation chain, score arithmetic, path metric, failure gate, canonical identity, protocol agreement, and route/sensing limitations are otherwise directly accounted for above.
