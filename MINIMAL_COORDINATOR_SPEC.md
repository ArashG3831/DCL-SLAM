# Minimal decentralized coordinator specification

## 1. Purpose and scope

This is the authoritative implementation contract for a new isolated,
robot-local Condition-C coordinator. It does not modify the legacy allocator,
ROS messages, launch/configuration, frontier generation, SLAM, fusion, Nav2,
traffic geometry, launcher, or evaluation semantics.

## 2. Operating assumptions / fault model

During a valid run ROS, Nav2, both allocators, and both Raspberry Pis remain
alive. Temporary delay, changing frontiers, planner failure, ordinary
navigation failure/cancel, and one robot being busy are in scope. Process or
server restart, crash persistence, exactly-once ownership, and recovery
journals are out of scope; infrastructure failure fails the run. Both robots
run the same coordinator; there is no PC coordinator. Canonical world/profile,
`START_RELEASE`, and protected systems follow `AGENTS.md`.

## 3. End-to-end data flow

```text
R1/R2 FrontierCandidateArray -> compatible current UNION
-> complete local/peer cost vectors -> pure deterministic selector
-> frozen traffic seam or unchanged scheduler adapter
-> LocalNav2 NavigateToPose -> ordinary result -> newest evidence
```

## 4. Frontier identity and union

Use `my_epuck_interfaces.msg.FrontierCandidate.frontier_id` (`uint64`) as the
current task key and `str(frontier_id)` as its selector/message form. No
eternal cross-generation identity is required.

Stage A is the existing pair of `FrontierCandidateArray` messages. They are
compatible when source IDs are `robot1`/`robot2`, `map_revision` and
`header.frame_id` match. `candidate_generation_id` is source-local and need not
match; `costmap_revision` is local path evidence and is not set identity.

```text
union_ids = sorted(set(R1.frontier_id) | set(R2.frontier_id))
union_hash = sha256(canonical_json({frame_id, map_revision, union_ids})).hexdigest()
```

Canonical JSON uses sorted keys and compact separators. A finite reachable
`path_length_m` is the local cost; missing, unreachable, invalid, or nonfinite
candidates are unavailable (`math.inf` internally, `path_valid=False` and
finite zero placeholders on the wire). Valid `Bid.path` is
`local_path_samples`; `Bid.heading_cost` is `heading_change_rad`.

## 5. Current-set / cost-vector exchange

Stage A uses both existing candidate streams; no new message is added. Stage B
publishes a full `TaskBidArray` for every union ID, including unavailable
entries, with `round_id == union_hash`, `union_hash == union_hash`, and
`TaskBid.canonical_task_id == str(frontier_id)`. Both full vectors must contain
the same ID set and digest before selection.

`source_robot_id` identifies the sender. `source_session_id`,
`source_snapshot_epoch`, and `validity` are compatibility fields only, not
freshness or safety proofs. Do not infer provenance from arrival order, local
timestamps, TTL, or zero UUIDs. A changed digest discards EVALUATING data and
starts fresh. Existing `FrontierCandidateArray`, `TaskBid`, and `TaskBidArray`
schemas are sufficient.

## 6. Deterministic selection

Reuse this existing pure signature from `distributed_assignment.scoring`:

```python
choose_pair_assignment(round_id: str, union: CanonicalUnion,
    robot1_bids: BidBatch, robot2_bids: BidBatch,
    hard_failed_tasks: frozenset[str] = frozenset(),
    weights: AssignmentWeights = AssignmentWeights(),
    scoring_mode: str = 'legacy_weighted',
    fixed_robot1_task_id: str = '', fixed_robot2_task_id: str = '',
    traffic_compatibility: Optional[TrafficCompatibility] = None) -> PairDecision
```

The adapter uses `round_id=union.union_hash`, empty hard-failure set, default
weights, and `scoring_mode='frontier_cost_only'`. It returns only normalized
task IDs to the main node; no `PairDecision` exchange/agreement lifecycle is
used. Both idle uses both vectors; one busy excludes its active ID and selects
at most one task for the idle robot; both busy returns no allocation.

## 7. Busy/idle ownership and five states

The main node owns `active_goal`, `active_path`, current goal handle/token,
candidate arrays, peer batches, union hash, and state. One robot owns at most
one NavigateToPose action. Active navigation survives map churn; only the
matching ordinary result/cancel clears it. A stale callback token cannot clear
a newer goal. An idle robot recomputes from newest evidence.

```text
IDLE -> EVALUATING -> WAITING_TRAFFIC -> GOAL_PENDING -> NAVIGATING -> IDLE
```

New union replaces EVALUATING. Matching full vectors completes EVALUATING.
Traffic approval or release exits WAITING_TRAFFIC. A single goal send owns
GOAL_PENDING. Ordinary result/cancel exits NAVIGATING. Infrastructure failure
fails the run rather than entering recovery.

## 8. Traffic

Source inspection found no clean public allocator-independent legacy seam:
`_traffic_for_bid_pair()` depends on legacy round state. Therefore the binding
adapter calls `traffic_scheduler.schedule_traffic()` unchanged. It passes the
two selected `Bid.path` sequences, safe radii, reference speed, `eta_tie_s`,
and active robot IDs; it returns `TrafficDecision`. A matching
`waiting_robot_id` means `WAITING_TRAFFIC`; no winner/waiter means proceed.
The final dispatch uses the same path samples. No traffic protocol or replay
machinery is added.

## 9. Navigation

Reuse `LocalNav2(node: Node, *, phase_gated: bool = False)`,
`check_dispatch_preconditions()`, `send_navigation()`, its existing
`NavigationOutcome` callback, and `cancel_navigation()`. The adapter creates
the existing `PhysicalTask` shape from a candidate for this boundary. A
rejected/failed/canceled ordinary goal clears only its matching token and
recomputes. Unknown infrastructure failure fails the run; no duplicate-send
or crash recovery is designed.

## 10. Mission termination

Reuse `CandidateEvidence`, `TerminalReason`, `classify_empty_frontiers()`, and
`matching_terminal_reason()` from `mission_termination.py`. The main node
supplies the existing `map_stability_grace_s`; no new timer is invented.
Terminal requires both robots idle, matching union hash, no GOAL_PENDING or
WAITING_TRAFFIC, fresh repeated evidence, and no unresolved/unclassified/
pending or actionable work.

The current classifier intentionally returns no success for an unreachable
count. The termination adapter may add only this required minimal rule: after
the existing stability interval, if no actionable/unqueried/unclassified or
planner-pending work remains and remaining evidence is conclusively
unreachable, return `TerminalReason.NO_REACHABLE_FRONTIERS`. Stable empty or
no-actionable evidence uses the existing classifier. No terminal ACK/proof is
added.

## 10a. Completion propagation

Use the existing `DistributedExplorationEvent` on `/{robot}/distributed_event`;
do not add a message, topic, ACK, journal, or replay store. A local successful
goal adds its frontier ID to `completed_ids`. The set is pruned to IDs present
in the current union whenever current union/bid state is rebuilt.

Whenever current bid state is published, advertise every locally completed ID
still present in the current union as `NAVIGATION_SUCCEEDED` with
`canonical_task_id`, `source_robot_id`, `union_hash`, and `round_id` equal to
the current union hash. This is repeated current state, not event history.

Accept a peer completion only when the source is the expected peer, the event
type is `NAVIGATION_SUCCEEDED`, the task ID is in the current union, and the
event union hash equals the current union hash. Insertion is idempotent. An
old event is rejected after the task or its union context has retired. Peer
learned IDs suppress local bids but are not re-advertised as locally completed
navigation. No restart recovery is required.

## 11. Six module boundaries

| Production file | Responsibility | Physical-line cap |
|---|---|---:|
| `src/my_epuck_project/my_epuck_project/minimal_frontier_sets.py` | union, digest, existing-model conversion, costs | 100 |
| `src/my_epuck_project/my_epuck_project/minimal_frontier_selection.py` | pure selector adapter and busy/idle normalization | 100 |
| `src/my_epuck_project/my_epuck_project/minimal_frontier_protocol.py` | existing-message conversion and full-vector checks | 100 |
| `src/my_epuck_project/my_epuck_project/minimal_frontier_navigation.py` | LocalNav2 and goal-token adapter | 120 |
| `src/my_epuck_project/my_epuck_project/minimal_frontier_traffic.py` | unchanged scheduler adapter | 100 |
| `src/my_epuck_project/my_epuck_project/minimal_frontier_termination.py` | termination/stability adapter | 100 |

The future main `minimal_frontier_allocator.py` composes these modules, owns
ROS I/O and the five states, and is not created in this task. No seventh helper
module is required.

## 12. Exact public interfaces

Types are imported from existing ROS/project modules; no shared types module or
new elaborate dataclass is permitted.

```python
# minimal_frontier_sets.py
compatible_context(r1: FrontierCandidateArray, r2: FrontierCandidateArray) -> bool
build_union(r1: FrontierCandidateArray, r2: FrontierCandidateArray) -> CanonicalUnion | None
materialize_costs(array: FrontierCandidateArray, union: CanonicalUnion) -> Mapping[str, float]
materialize_bids(array: FrontierCandidateArray, union: CanonicalUnion) -> tuple[Bid, ...]
# minimal_frontier_selection.py
choose_assignment(union: CanonicalUnion, r1: BidBatch, r2: BidBatch, *, active_robot1_id: str = '', active_robot2_id: str = '') -> tuple[str, str] | None
# minimal_frontier_protocol.py
make_batch(source_robot_id: str, source_session_id: str, source_snapshot_epoch: int, union: CanonicalUnion, bids: tuple[Bid, ...], validity_s: float) -> BidBatch
to_msg(batch: BidBatch, stamp: builtin_interfaces.msg.Time) -> TaskBidArray
from_msg(message: TaskBidArray) -> BidBatch
complete_pair(union: CanonicalUnion, r1: BidBatch, r2: BidBatch) -> bool
# minimal_frontier_navigation.py
to_physical_task(candidate: FrontierCandidate, source_robot_id: str) -> PhysicalTask
check_preconditions(nav: LocalNav2, task: PhysicalTask, final_path_valid: bool, callback: Callable[[DispatchPreconditions], None], path_samples: tuple[Point, ...] = (), path_frame_id: str = '') -> None
send(nav: LocalNav2, task: PhysicalTask, callback: Callable[[NavigationOutcome], None], diagnostic_path: tuple[Point, ...] = ()) -> bool
cancel(nav: LocalNav2) -> bool
# minimal_frontier_traffic.py
decide(r1_path: Sequence[Point], r2_path: Sequence[Point], *, robot1_safe_radius_m: float, robot2_safe_radius_m: float, reference_speed_mps: float, eta_tie_s: float = 0.05, active_robots: frozenset[str] = frozenset()) -> TrafficDecision
should_wait(robot_id: str, decision: TrafficDecision) -> bool
# minimal_frontier_termination.py
classify(r1: CandidateEvidence, r2: CandidateEvidence, *, stable_for_s: float, stability_grace_s: float) -> TerminalReason | None
matching(local_terminal: bool, local_reason: str, peer_terminal: bool, peer_reason: str) -> str | None
```

`build_union()` creates existing `CanonicalUnion`/`CanonicalTask` shapes only
for the pure selector; it does not invoke persistent physical equivalence.
`to_msg()`/`from_msg()` may call existing `ros_conversion.bid_batch_to_msg()`
and `bid_batch_from_msg()` unchanged. `classify()` calls the existing terminal
classifier and owns only the explicitly stated stable-no-reachable adapter.

## 13. Agent/test ownership and forbidden dependencies

Every production file begins with a concise 5–10 line docstring stating what
it does and does not do. Each production agent owns exactly one listed file,
may inspect anything, may edit only that file, and may not create helpers or
modify existing production source. If a frozen signature is impossible, stop
and report the mismatch. The second wave owns exactly one corresponding test
file; tests do not modify production code.

The new files must not import or recreate legacy RoundWork, continuation,
certificates, lower bounds, candidate history, provenance rebase, selector
agreement lifecycle, commitment journals, restart/source-session recovery,
terminal proof/ACK, or hard-failure expiry. The legacy allocator is untouched.

## 14. LOC budget and build order

Caps are physical production lines; helper-file splitting may not evade them,
and concise comments/docstrings count. Reused code is not new LOC. Add no
mechanism without a concrete normal-runtime failure it prevents. Crash recovery
is out of scope. If a module exceeds its cap, its agent stops and reports.

Expected total new Python: **450–800 lines**; warning **over 1,000**; hard
design stop **approaching 1,200**.

Build order: review this spec and signatures; implement the six files in
parallel; run import/compile checks and the six-file test wave; then have main
Codex implement the composition node. Compare offline before ROS/Webots.
Rollback remains the old allocator entry point.
