# Distributed allocator/coordinator code architecture inventory

Date: 2026-09-08

Scope: canonical production allocator/coordinator runtime only. No source, build, ROS, Webots, or test execution was performed.

Included source files are the live allocator node, its `distributed_assignment` runtime helpers (excluding `burgard_assignment.py`), and the direct termination/actionability/round-lifecycle helpers listed in the task. Tests, reports, generated files, install trees, and the legacy Burgard implementation are excluded.

LOC convention: physical LOC includes blank/comment lines; nonblank LOC excludes only blank lines. Symbol ranges are inclusive physical source lines.

## 1. File inventory

| File | Physical LOC | Nonblank LOC | Purpose |
|---|---:|---:|---|
| `src/my_epuck_project/my_epuck_project/distributed_frontier_assignment.py` | 5076 | 4897 | ROS node implementing one replicated two-robot frontier-assignment peer, including round orchestration, peer callbacks, certificate gating, traffic handling, dispatch, terminal handling, and runtime telemetry. |
| `src/my_epuck_project/my_epuck_project/distributed_assignment/__init__.py` | 15 | 13 | Package re-exports for canonical task, model, and pair-scoring symbols. |
| `src/my_epuck_project/my_epuck_project/distributed_assignment/canonical.py` | 274 | 236 | Deterministic task identity, geometry equivalence, canonical union, and round/task fingerprint helpers. |
| `src/my_epuck_project/my_epuck_project/distributed_assignment/failures.py` | 125 | 108 | Failure classification and bounded suppression state for observed task/navigation failures. |
| `src/my_epuck_project/my_epuck_project/distributed_assignment/local_nav2.py` | 1820 | 1685 | Local Nav2/action, map/costmap/TF/odometry/scan, path-evaluation, dispatch-precondition, navigation-result, and bounded diagnostic integration used by the allocator. |
| `src/my_epuck_project/my_epuck_project/distributed_assignment/models.py` | 305 | 252 | Dataclasses and enums representing physical/canonical tasks, snapshots, bids, decisions, failures, lifecycle state, completion inputs, and travel distance. |
| `src/my_epuck_project/my_epuck_project/distributed_assignment/protocol.py` | 216 | 180 | Receipt freshness, source-version/snapshot acceptance, bid validation, committed-round protection, peer liveness, and completion-state rules. |
| `src/my_epuck_project/my_epuck_project/distributed_assignment/ros_conversion.py` | 228 | 198 | Conversions between ROS message representations and the internal task, snapshot, bid, and decision models. |
| `src/my_epuck_project/my_epuck_project/distributed_assignment/scoring.py` | 1062 | 986 | Assignment weights, motion-cost scoring, cost-only certificate evaluation, pair selection, MRTSP route selection, solo ranking, overlap, and decision fingerprints. |
| `src/my_epuck_project/my_epuck_project/distributed_assignment/traffic_scheduler.py` | 248 | 219 | Continuous sampled-path conflict geometry, progress projection, and deterministic traffic ordering for a new conflicting dispatch. |
| `src/my_epuck_project/my_epuck_project/mission_termination.py` | 309 | 271 | Terminal-reason types and conservative physical-frontier evidence summarization/classification. |
| `src/my_epuck_project/my_epuck_project/frontier_actionability.py` | 33 | 24 | Small eligibility predicates for frontier gain and reachable task actionability. |
| `src/my_epuck_project/my_epuck_project/round_lifecycle.py` | 39 | 29 | Monotonic round-generation leases for rejecting stale asynchronous work. |

**Included total: 9,750 physical LOC and 9,098 nonblank LOC across 13 files.**

### `src/my_epuck_project/my_epuck_project/distributed_frontier_assignment.py`

Purpose: ROS node implementing one replicated two-robot frontier-assignment peer, including round orchestration, peer callbacks, certificate gating, traffic handling, dispatch, terminal handling, and runtime telemetry.

Imported modules: `from collections import deque`, `from dataclasses import dataclass, field, replace`, `import hashlib`, `import json`, `import math`, `import os`, `import time`, `from typing import Optional`, `from my_epuck_interfaces.msg import DistributedExplorationEvent, DistributedExplorationStatus, ExplorationFailure, FrontierCandidateArray, PairDecision as PairDecisionMsg, TaskBidArray as TaskBidArrayMsg, TaskSnapshot as TaskSnapshotMsg, RelativePoseHypothesis`, `from std_msgs.msg import Bool, String`, `import rclpy`, `from rclpy.executors import MultiThreadedExecutor, SingleThreadedExecutor`, `from rclpy.node import Node`, `from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy`, `from .distributed_assignment.canonical import build_canonical_union, canonical_round_id, equivalent_tasks, TaskIdentity`, `from .distributed_assignment.failures import HARD_FAILURES, bounded_suppression_duration`, `from .distributed_assignment.local_nav2 import classify_dispatch_precondition_failure, DispatchPreconditions, LocalNav2, NavigationOutcome, PathEvaluation, path_is_valid_finite`, `from .distributed_assignment.models import Bid, BidBatch, CanonicalTask, CanonicalUnion, CoordinatorState, FailureClass, PairDecision, PhysicalTask, TaskSnapshot`, `from .distributed_assignment.protocol import bid_batch_valid, CommittedRound, PeerLiveness, receive, Received, SnapshotLedger`, `from .distributed_assignment.ros_conversion import bid_batch_from_msg, bid_batch_to_msg, decision_to_msg, duration_to_seconds, seconds_to_duration, snapshot_from_msg, text_to_uuid, uuid_to_text`, `from .distributed_assignment.scoring import AssignmentWeights, choose_pair_assignment, choose_mrtsp_route_assignment, cost_only_dispatch_certificate, nominal_motion_cost_s, route_overlap, rank_solo_tasks`, `from .distributed_assignment.traffic_scheduler import TrafficDecision, project_path_progress, schedule_traffic`, `from .round_lifecycle import RoundGeneration`, `from .mission_termination import CandidateEvidence, FrontierRegionEvidence, TerminalReason, classify_empty_frontiers, summarize_frontier_regions, credible_planner_infrastructure_failure, terminal_reason_is_success, all_physical_tasks_suppressed`

Imported internal project modules: `.distributed_assignment.canonical`, `.distributed_assignment.failures`, `.distributed_assignment.local_nav2`, `.distributed_assignment.models`, `.distributed_assignment.protocol`, `.distributed_assignment.ros_conversion`, `.distributed_assignment.scoring`, `.distributed_assignment.traffic_scheduler`, `.mission_termination`, `.round_lifecycle`

Classes:
- `InitialExplorationBarrier` (266-309): Replicated mutual-readiness gate for the two-robot local phase. This is deliberately a small state machine, independent of ROS transport. Each local allocator owns one instance and observes the same two readiness facts from its own local candidate pipeline and the peer's readiness announcement. Handoff supersedes the gate while it is still closed.
- `RoundWork` (313-331): Mutable bounded work for one uncommitted canonical round.
- `ActiveCommitment` (335-348): Replicated immutable description of one dispatched cooperative goal.
- `ContinuationContext` (352-359): One free robot plus one still-active immutable peer commitment.
- `TrafficHold` (368-380): A local deferred dispatch bound to one agreed traffic reservation.
- `DistributedFrontierAssignment` (443-5050): Compute a complete pair decision independently and dispatch only locally.

Top-level functions:
- `dispatch_delay_elapsed()` (102-110): Return whether the optional pre-handoff dispatch hold has elapsed. The hold is a bounded evidence-acquisition aid for unknown-pose runs. It is evaluated against monotonic wall time so a zero simulation clock or a paused Webots startup cannot accidentally release navigation early.
- `evidence_hold_active()` (113-117): Return whether a live evidence-opportunity lease blocks new goals.
- `solo_retry_delay_s()` (120-123): Return bounded exponential delay for retryable local failures.
- `lower_bound_context_matches()` (126-138): Return whether compact bounds belong to this exact task snapshot.
- `classify_lower_bound_evidence()` (153-262): Classify certificate evidence without changing certificate behavior. This is deliberately diagnostic-only. The production certificate still uses its existing ``lower_bound_context_matches`` gate; this helper merely records which existing input made that gate conservative.
- `round_pass_is_current()` (362-364): Return whether a threaded tick still owns its round snapshot.
- `eligible_solo_tasks()` (410-440): Return locally dispatchable tasks after bounded physical suppression. A successful finite Nav2 path is feasible at any distance. Path length remains available to the local/distributed preference logic and is not a hard eligibility condition.
- `main()` (5053-5076): Run one namespaced replicated assignment peer.

Methods:
- `InitialExplorationBarrier.__post_init__()` (281-284): No function-level docstring; implementation is at the listed source range.
- `InitialExplorationBarrier.observe_local_ready()` (286-288): Record local readiness idempotently.
- `InitialExplorationBarrier.observe_peer_ready()` (290-292): Record peer readiness idempotently.
- `InitialExplorationBarrier.observe_handoff()` (294-296): Make an accepted canonical handoff supersede local exploration.
- `InitialExplorationBarrier.maybe_release()` (298-304): Release exactly once when both ready and handoff has not won.
- `InitialExplorationBarrier.dispatch_allowed()` (307-309): Return whether local pre-handoff goal dispatch is permitted.
- `DistributedFrontierAssignment.__init__()` (446-980): Configure one equal peer with identity-bound topic and Nav2 interfaces.
- `DistributedFrontierAssignment._sim_time_s()` (982-991): Return current simulation/ROS time for authoritative startup logs.
- `DistributedFrontierAssignment._startup_event()` (993-1003): Emit one compact startup milestone to ROS logs and the observer.
- `DistributedFrontierAssignment._publish_initial_local_ready()` (1005-1031): Publish one peer-readable local exploration readiness fact.
- `DistributedFrontierAssignment._initial_peer_ready_callback()` (1033-1063): Observe the peer's one-shot readiness announcement idempotently.
- `DistributedFrontierAssignment._maybe_release_initial_exploration_barrier()` (1065-1090): Release local dispatch only after both replicas are ready.
- `DistributedFrontierAssignment._traffic_test_release_callback()` (1092-1109): Record a per-round simulated dispatch boundary from the test barrier.
- `DistributedFrontierAssignment._publish_traffic_test_ready()` (1111-1132): Advertise one agreed round to the test-only common barrier.
- `DistributedFrontierAssignment._activate_protocol_inputs()` (1134-1181): Subscribe to task/bid traffic only in the active shared phase.
- `DistributedFrontierAssignment._activate_assignment_timers()` (1183-1190): Start assignment work only after the canonical handoff.
- `DistributedFrontierAssignment._activate_shared_phase()` (1192-1216): No function-level docstring; implementation is at the listed source range.
- `DistributedFrontierAssignment._start_release_callback()` (1218-1236): No function-level docstring; implementation is at the listed source range.
- `DistributedFrontierAssignment._exploration_dispatch_allowed()` (1238-1241): Keep every exploration send behind the common C release barrier.
- `DistributedFrontierAssignment._shared_nav2_ready_callback()` (1243-1249): No function-level docstring; implementation is at the listed source range.
- `DistributedFrontierAssignment._handoff_callback()` (1251-1283): Switch local-only dispatch off only after canonical acceptance.
- `DistributedFrontierAssignment._stop_local_phase()` (1285-1301): Stop pre-handoff allocation work after canonical handoff.
- `DistributedFrontierAssignment._candidate_callback()` (1303-1469): Keep bounded generator evidence separate from reachable task bids.
- `DistributedFrontierAssignment._decode_frontier_regions()` (1472-1531): Decode compact diagnostic region geometry; malformed data is ignored.
- `DistributedFrontierAssignment._snapshot_callback()` (1532-1604): Accept only bounded monotonic source-local snapshot provenance.
- `DistributedFrontierAssignment._bid_callback()` (1606-1616): No function-level docstring; implementation is at the listed source range.
- `DistributedFrontierAssignment._decision_callback()` (1618-1623): No function-level docstring; implementation is at the listed source range.
- `DistributedFrontierAssignment._status_callback()` (1625-1654): No function-level docstring; implementation is at the listed source range.
- `DistributedFrontierAssignment._failure_callback()` (1656-1668): No function-level docstring; implementation is at the listed source range.
- `DistributedFrontierAssignment._peer_event_callback()` (1670-1704): Replicate event-driven traffic release without a coordinator.
- `DistributedFrontierAssignment._record_hard_failure()` (1706-1734): Record bounded suppression for directly observed hard evidence.
- `DistributedFrontierAssignment._fresh_snapshot()` (1736-1738): No function-level docstring; implementation is at the listed source range.
- `DistributedFrontierAssignment._local_fallback_snapshot()` (1740-1767): Return a current or safely revalidated local task seed. ``Received.fresh`` remains the authority for cooperative/peer evidence. Local degraded-solo work has a different lifetime: an expired local snapshot may be retained only when the latest local candidate batch still advertises every task selected from it. The final dispatch path then performs the existing current TF, map, costmap, Nav2 path, traffic, reservation, and failure checks. An absent/empty current candidate set never authorizes a stale task.
- `DistributedFrontierAssignment._consume_local_fallback_trigger()` (1769-1792): Start existing fallback before another cooperative wait. This is a scheduler hint only. The fallback still performs current source-local identity, peer-liveness, TF, path, reservation, failure, and final dispatch checks; no retained path is sent.
- `DistributedFrontierAssignment._temporary_peer_reservation_ids()` (1794-1813): Return task IDs advertised by the peer's degraded-solo fallback. A degraded-solo goal is a temporary local commitment, not a normal cooperative agreement. It is nevertheless advertised through the existing distributed status heartbeat so the peer cannot select the same canonical task while cooperative evidence is catching up. Normal cooperative commitments use their ordinary decision hash and are deliberately not included here.
- `DistributedFrontierAssignment._peer_unavailable_for_degraded_solo()` (1815-1823): Require bounded peer-loss evidence before cooperative solo dispatch.
- `DistributedFrontierAssignment._continue_local_work_while_waiting()` (1825-1861): Use local fallback only after bounded peer unavailability. This does not manufacture a pair decision or relax the certificate. A responsive peer keeps selection on the paired path, where the authoritative traffic scheduler has both routes. After the existing peer timeout, the temporary task is published through the normal status heartbeat and excluded by ``_temporary_peer_reservation_ids`` on the peer.
- `DistributedFrontierAssignment._start_immediate_fallback_after_terminal()` (1863-1881): Start peer-loss continuation without waiting for the next tick. A terminal navigation callback already establishes that this robot is free. A local candidate that arrived while this robot was busy is already a scheduler trigger; consuming it here avoids waiting for another cooperative tick only when bounded liveness evidence has established peer unavailability. A responsive peer resumes through the normal paired traffic path instead.
- `DistributedFrontierAssignment._finite_path_samples()` (1884-1889): Return whether a retained commitment has usable traffic geometry.
- `DistributedFrontierAssignment._remember_active_commitments()` (1891-1946): Retain only the agreed task/path needed by future continuation rounds.
- `DistributedFrontierAssignment._clear_active_commitment()` (1948-1957): Drop one task commitment and invalidate any continuation using it.
- `DistributedFrontierAssignment._continuation_context()` (1959-2015): Return a safe one-free/one-busy context, without using busy proposals.
- `DistributedFrontierAssignment._continuation_round_id()` (2017-2027): Hash only free proposal content and the immutable busy commitment.
- `DistributedFrontierAssignment._activate_continuation_round()` (2029-2099): Install a continuation round or retain its unchanged generation.
- `DistributedFrontierAssignment._continuation_busy_batch()` (2101-2130): Create a deterministic one-task bid for the fixed active commitment.
- `DistributedFrontierAssignment._continuation_bid_batches()` (2132-2160): Validate only the free robot's exchanged batch plus fixed peer state.
- `DistributedFrontierAssignment._snapshot_content_fingerprint()` (2163-2193): Fingerprint task content while ignoring epoch-only heartbeats.
- `DistributedFrontierAssignment._round_is_current()` (2195-2201): Check both object identity and generation before committing work.
- `DistributedFrontierAssignment._discard_stale_tick()` (2203-2220): Account for asynchronous work that no longer owns the round.
- `DistributedFrontierAssignment._log_round_lifecycle()` (2222-2236): Emit compact lifecycle telemetry for post-run race auditing.
- `DistributedFrontierAssignment._activate_round()` (2238-2253): Install a round and return its generation token.
- `DistributedFrontierAssignment._publish_health_diagnostic()` (2255-2281): Publish liveness counters without changing allocator behavior.
- `DistributedFrontierAssignment._evidence_status_callback()` (2283-2305): Refresh or clear the bounded local evidence-opportunity lease.
- `DistributedFrontierAssignment._allocator_timing_bucket()` (2309-2318): Return the requested simulation-time attribution window.
- `DistributedFrontierAssignment._allocator_timing_begin()` (2320-2325): Begin one aggregate diagnostic section measurement.
- `DistributedFrontierAssignment._allocator_timing_record()` (2327-2336): Accumulate one section measurement without per-call logging.
- `DistributedFrontierAssignment._allocator_timing_add_inputs()` (2338-2353): Record call-site input cardinalities for the same two windows.
- `DistributedFrontierAssignment._allocator_timing_timed_call()` (2355-2362): Time an existing allocator call while preserving its return value.
- `DistributedFrontierAssignment._allocator_timing_maybe_log()` (2364-2381): Emit one compact cumulative snapshot periodically when enabled.
- `DistributedFrontierAssignment._tick()` (2383-2392): Run one allocator tick and optionally attribute its wall time.
- `DistributedFrontierAssignment._tick_impl()` (2394-2915): No function-level docstring; implementation is at the listed source range.
- `DistributedFrontierAssignment._cost_only_certificate_blocker_diagnostics()` (2917-3118): Describe certificate blockers without changing certificate behavior.
- `DistributedFrontierAssignment._cost_only_dispatch_certificate()` (3120-3251): Prevent dispatch before an unqueried cost-only option is dominated.
- `DistributedFrontierAssignment._traffic_for_bid_pair()` (3253-3286): Run the one authoritative traffic model for candidate pair paths.
- `DistributedFrontierAssignment._traffic_for_decision()` (3288-3311): Derive the same bounded traffic result from agreed bid geometry.
- `DistributedFrontierAssignment._log_traffic_aware_selection()` (3313-3367): Log only policy choices skipped by the exact traffic scheduler.
- `DistributedFrontierAssignment._select_traffic_test_conflict_pair()` (3369-3463): Select a deterministic conflicting pair from real Nav2 bid paths. This is strictly a synchronized-test fixture operation. It does not invent coordinates, use physical truth, or alter production Burgard allocation. Every candidate is an actually valid bid for the owning robot, and conflict is evaluated by the same continuous scheduler used at dispatch time. Sorting by strongest measured conflict and then canonical IDs makes both replicas choose the same pair.
- `DistributedFrontierAssignment._traffic_reason()` (3465-3467): Keep event/status evidence compact, structured, and deterministic.
- `DistributedFrontierAssignment._continue_traffic_hold()` (3469-3548): Release on conflict clearance, or terminal evidence as fallback.
- `DistributedFrontierAssignment._begin_traffic_wait()` (3550-3605): Defer only this new local goal; never inject a Nav2 velocity hold.
- `DistributedFrontierAssignment._consider_completion()` (3607-3717): Require matching healthy empty-round persistence before COMPLETE.
- `DistributedFrontierAssignment._set_terminal()` (3719-3771): Freeze local dispatch after a replicated terminal semantic result.
- `DistributedFrontierAssignment._continue_degraded_solo()` (3773-3926): Dispatch at most one locally proposed task per epoch without team claims.
- `DistributedFrontierAssignment._continue_bidding()` (3928-3933): Continue bounded bidding and attribute the whole call if enabled.
- `DistributedFrontierAssignment._local_path_execution_key()` (3936-3962): Return the exact task/provenance key for a reusable path result. The frozen ``CanonicalTask`` contains every source member and every path-relevant task field, rather than only its quantized canonical ID. The source snapshot adds candidate-generation and lower-bound provenance that is not carried by ``PhysicalTask`` itself.
- `DistributedFrontierAssignment._local_path_result_cacheable()` (3965-3979): Accept only a successful finite allocator path result.
- `DistributedFrontierAssignment._remember_local_path_evaluation()` (3981-3993): Store one successful result with deterministic bounded eviction.
- `DistributedFrontierAssignment._cached_local_path_evaluation()` (3995-4015): Return a current-context exact cached path, if one exists.
- `DistributedFrontierAssignment._continue_bidding_impl()` (4017-4084): No function-level docstring; implementation is at the listed source range.
- `DistributedFrontierAssignment._append_bid()` (4086-4122): No function-level docstring; implementation is at the listed source range.
- `DistributedFrontierAssignment._finish_bids()` (4124-4150): No function-level docstring; implementation is at the listed source range.
- `DistributedFrontierAssignment._publish_local_bid_batch()` (4152-4173): Refresh the local bid heartbeat without changing round semantics.
- `DistributedFrontierAssignment._both_bid_batches_valid()` (4175-4188): No function-level docstring; implementation is at the listed source range.
- `DistributedFrontierAssignment._publish_decision()` (4190-4329): No function-level docstring; implementation is at the listed source range.
- `DistributedFrontierAssignment._matching_peer_decision()` (4331-4358): No function-level docstring; implementation is at the listed source range.
- `DistributedFrontierAssignment._start_local_dispatch()` (4360-4491): No function-level docstring; implementation is at the listed source range.
- `DistributedFrontierAssignment._dispatch_after_checks()` (4493-4620): No function-level docstring; implementation is at the listed source range.
- `DistributedFrontierAssignment._navigation_finished()` (4622-4746): No function-level docstring; implementation is at the listed source range.
- `DistributedFrontierAssignment._invalidate_round()` (4748-4763): No function-level docstring; implementation is at the listed source range.
- `DistributedFrontierAssignment._publish_failure()` (4765-4830): No function-level docstring; implementation is at the listed source range.
- `DistributedFrontierAssignment._hard_failed_task_ids()` (4832-4837): No function-level docstring; implementation is at the listed source range.
- `DistributedFrontierAssignment._expire_failures()` (4839-4843): No function-level docstring; implementation is at the listed source range.
- `DistributedFrontierAssignment._reset_round()` (4845-4864): No function-level docstring; implementation is at the listed source range.
- `DistributedFrontierAssignment._transition()` (4866-4892): No function-level docstring; implementation is at the listed source range.
- `DistributedFrontierAssignment._receipt_age()` (4895-4896): No function-level docstring; implementation is at the listed source range.
- `DistributedFrontierAssignment._local_session_text()` (4898-4900): No function-level docstring; implementation is at the listed source range.
- `DistributedFrontierAssignment._current_round_id()` (4902-4904): No function-level docstring; implementation is at the listed source range.
- `DistributedFrontierAssignment._peer_state_text()` (4906-4909): No function-level docstring; implementation is at the listed source range.
- `DistributedFrontierAssignment._publish_status()` (4911-4987): No function-level docstring; implementation is at the listed source range.
- `DistributedFrontierAssignment._emit_event()` (4989-5042): No function-level docstring; implementation is at the listed source range.
- `DistributedFrontierAssignment._log_union()` (5044-5050): No function-level docstring; implementation is at the listed source range.

### `src/my_epuck_project/my_epuck_project/distributed_assignment/__init__.py`

Purpose: Package re-exports for canonical task, model, and pair-scoring symbols.

Imported modules: `from .canonical import build_canonical_union, canonical_round_id`, `from .models import Bid, PhysicalTask, TaskSnapshot`, `from .scoring import AssignmentWeights, choose_pair_assignment`

Imported internal project modules: `.canonical`, `.models`, `.scoring`

Classes:
- None.

Top-level functions:
- None.

Methods:
- None.

### `src/my_epuck_project/my_epuck_project/distributed_assignment/canonical.py`

Purpose: Deterministic task identity, geometry equivalence, canonical union, and round/task fingerprint helpers.

Imported modules: `from __future__ import annotations`, `from dataclasses import dataclass`, `import hashlib`, `import json`, `import math`, `from typing import Iterable, List, Sequence, Tuple`, `from .models import Bounds, CanonicalTask, CanonicalUnion, PhysicalTask, Point`

Imported internal project modules: `.models`

Classes:
- `EquivalenceTolerances` (15-26): Documented geometry tolerances for e-puck frontier matching.
- `TaskIdentity` (47-52): Source identity fields used by canonical round hashing.

Top-level functions:
- `_stable_hash()` (29-33): No function-level docstring; implementation is at the listed source range.
- `canonical_round_id()` (36-43): Hash Robot 1 then Robot 2 source sessions and snapshot epochs.
- `bounds_iou()` (55-69): Return bounded intersection-over-union of two task rectangles.
- `bounds_gap()` (72-82): Return Euclidean separation between rectangles, zero on overlap.
- `geometry_overlap()` (85-99): Estimate symmetric sample overlap without depending on sample order.
- `equivalent_tasks()` (102-133): Match practical observation tasks using several geometry signals.
- `_quantize()` (136-137): No function-level docstring; implementation is at the listed source range.
- `_quantized_points()` (140-144): No function-level docstring; implementation is at the listed source range.
- `_make_canonical_task()` (147-222): No function-level docstring; implementation is at the listed source range.
- `build_canonical_union()` (225-261): Cluster equivalent proposals and return deterministic bounded ordering.
- `world_from_rotated_grid_cell()` (264-274): Convert a cell center through an occupancy-grid origin rotation.

Methods:
- None.

### `src/my_epuck_project/my_epuck_project/distributed_assignment/failures.py`

Purpose: Failure classification and bounded suppression state for observed task/navigation failures.

Imported modules: `from dataclasses import dataclass`, `from typing import Iterable`, `from .models import FailureClass, FailureEvidence, FailureRecord, PhysicalTask`

Imported internal project modules: `.models`

Classes:
- `_Suppression` (69-71): Class body defines the type/state structure; no class-level docstring is present.
- `FailureSuppressor` (74-125): Bounded escalating suppression keyed by physical signature.

Top-level functions:
- `classify_compute_path_result()` (9-27): Classify only Nav2 ComputePath evidence exposed by its action result.
- `classify_failure()` (30-48): Choose the most specific class supported by direct observable evidence.
- `bounded_suppression_duration()` (59-65): Return an escalating but bounded TTL for hard evidence.

Methods:
- `FailureSuppressor.__init__()` (77-87): Configure escalation timing and the bounded record capacity.
- `FailureSuppressor.observe()` (89-111): Create suppression only from hard, repeated evidence.
- `FailureSuppressor.suppressed()` (113-125): Check signature suppression, permitting documented alternate approaches.

### `src/my_epuck_project/my_epuck_project/distributed_assignment/local_nav2.py`

Purpose: Local Nav2/action, map/costmap/TF/odometry/scan, path-evaluation, dispatch-precondition, navigation-result, and bounded diagnostic integration used by the allocator.

Imported modules: `from dataclasses import dataclass`, `from collections import deque`, `import fcntl`, `import hashlib`, `import json`, `import math`, `import os`, `import time`, `from typing import Callable, Optional`, `from action_msgs.msg import GoalStatus`, `from lifecycle_msgs.srv import GetState`, `from nav2_msgs.action import ComputePathToPose, NavigateToPose`, `from nav_msgs.msg import OccupancyGrid, Odometry`, `from sensor_msgs.msg import LaserScan`, `from rclpy.action import ActionClient`, `from rclpy.duration import Duration`, `from rclpy.node import Node`, `from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy`, `from rclpy.time import Time`, `from tf2_ros import Buffer, TransformException, TransformListener`, `from .failures import classify_failure`, `from .models import FailureClass, FailureEvidence, PhysicalTask, Point, TravelDistance`

Imported internal project modules: `.failures`, `.models`

Classes:
- `PathEvaluation` (125-145): Observable result of one local ComputePathToPose request.
- `DispatchPreconditions` (149-183): Auditable final local dispatch preconditions.
- `NavigationOutcome` (187-204): Terminal local NavigateToPose evidence and measured motion.
- `LocalPathClearance` (438-449): Bounded evidence for the contiguous path prefix in the local grid.
- `LocalNav2` (518-1820): Own only this node namespace's planner, navigator, state, TF, and odometry.

Top-level functions:
- `initial_path_heading_cost()` (50-79): Measure initial planned-path direction mismatch in radians. The path's first meaningful segment is used instead of the approach-pose bearing. Nav2 commonly repeats the first pose, so nearly coincident samples are skipped. A path without a meaningful segment has no heading evidence and contributes zero rather than inventing a turn cost.
- `classify_follow_path_controller_error()` (82-88): Classify propagated FollowPath failures without hiding TF faults.
- `follow_path_controller_error_name()` (91-93): Return the installed Nav2 FollowPath meaning without flattening it.
- `classify_dispatch_precondition_failure()` (96-121): Classify a final dispatch rejection by its first-order evidence. Geometry can be rejected before lifecycle queries complete. In that case ``lifecycle_active`` is deliberately still false and must not turn a stale/unknown goal into a TF/lifecycle failure.
- `local_path_gate_threshold()` (222-231): Return the selected local-path acceptance threshold.
- `upstream_point_validation()` (237-258): Mirror v1.6.0 ``world_point_cost``/``is_world_point_blocked``. The upstream public point helper treats absent and out-of-bounds points as not blocked, and blocks only a present occupancy value strictly greater than ``OCC_THRESHOLD``. Keep the raw value and the reason separate so this diagnostic cannot be mistaken for a dispatch gate.
- `_grid_cell()` (261-276): Return a world point's integer cell without exposing an unbounded grid.
- `bounded_grid_crop()` (279-305): Return a bounded row-major occupancy crop for failure diagnostics.
- `_transform_translation()` (308-320): Return the latest source-frame origin expressed in target_frame.
- `_transform_point()` (323-342): Return a source-frame point expressed in target_frame, if available.
- `_stamp_ns_static()` (345-347): Convert a ROS builtin time stamp without requiring a node instance.
- `execution_geometry_signature()` (350-362): Hash execution-relevant local geometry, excluding timestamps.
- `path_length()` (365-367): Measure a path polyline in its declared frame.
- `path_samples_digest()` (370-377): Return a compact deterministic digest for planner/gate path identity.
- `path_is_valid_finite()` (380-394): Return whether a successful path is structurally safe to consume. Path length is deliberately not bounded here. A finite, otherwise valid Nav2 path remains eligible regardless of distance; distance is a scoring and navigation-cost input, not an artificial reachability gate.
- `downsample_path()` (397-407): Keep deterministic endpoints and bounded evenly spaced path samples.
- `occupancy_value()` (410-434): Read a world point from a grid with a possibly rotated origin.
- `local_path_clearance()` (452-505): Check the portion of a global path visible in the rolling local grid. Global NavFn is intentionally allowed to traverse unknown space so it can reach a frontier. That makes a valid global path insufficient at the dispatch boundary: its first metres can still enter an inflated obstacle that the rolling local controller will stop for. Only the contiguous prefix that lies inside the rolling window is an execution-horizon gate. Once the path leaves that window, the farther segment is covered by the global path validation and is not re-entered if the path later loops back. The transform is injected so this geometry rule remains deterministic and unit-testable without a ROS TF graph.
- `local_path_clear()` (508-515): Boolean compatibility wrapper for the local path safety predicate.

Methods:
- `DispatchPreconditions.ready()` (175-183): Return whether every required dispatch condition passed.
- `LocalNav2.__init__()` (521-632): Create relative interfaces that resolve inside the local robot namespace.
- `LocalNav2._ensure_compute_client()` (634-637): No function-level docstring; implementation is at the listed source range.
- `LocalNav2._ensure_navigate_client()` (639-642): No function-level docstring; implementation is at the listed source range.
- `LocalNav2._ensure_lifecycle_clients()` (644-651): No function-level docstring; implementation is at the listed source range.
- `LocalNav2._activate_phase_inputs()` (653-679): Start map/health callbacks for a post-handoff shared phase.
- `LocalNav2.local_goal_active()` (682-687): Return whether this wrapper owns an unresolved local navigation goal.
- `LocalNav2.shared_map()` (690-692): Expose the local shared-map replica for allocator LOS evaluation.
- `LocalNav2.travelled_distance_m()` (695-697): Return measured cumulative local odometry displacement.
- `LocalNav2.interface_names()` (699-706): Expose resolved local names for static integration auditing.
- `LocalNav2.refresh_health()` (708-734): Refresh managed-node state without blocking the coordinator timer.
- `LocalNav2.health_flags()` (736-747): Return conservative current Nav2 and required-transform health flags.
- `LocalNav2.shared_tf_status()` (749-771): Return the exact shared-map/base TF readiness used by dispatch. The distributed allocator uses this as a *pre-decision* gate. It deliberately applies the same frame pair and freshness limit as the final dispatch health check; it does not synthesize or fall back to a remembered/ground-truth pose.
- `LocalNav2.synchronized_test_inputs_ready()` (773-787): Return the local inputs needed before the test barrier advertises a round. The shared phase manager has already established lifecycle readiness. This test-only predicate deliberately avoids depending on the asynchronous diagnostic lifecycle cache, while still requiring both action servers and the current map/costmap samples.
- `LocalNav2.lookup_pose_in_global()` (789-821): Return a fresh target-frame pose expressed in this Nav2 global frame. The traffic gate uses this only for an already committed peer path. A zero-time lookup asks tf2 for the newest available transform; the returned age is measured against the node's ROS clock and callers must reject an unavailable/stale result rather than masking a remembered location.
- `LocalNav2._on_map()` (823-824): No function-level docstring; implementation is at the listed source range.
- `LocalNav2._on_costmap()` (826-827): No function-level docstring; implementation is at the listed source range.
- `LocalNav2._on_local_costmap()` (829-831): Retain only the latest rolling costmap for failure diagnostics.
- `LocalNav2._grid_stamp()` (834-839): Return the source timestamp used for bounded path freshness.
- `LocalNav2.path_context_matches()` (841-848): Reject bid reuse when either local planning input has advanced.
- `LocalNav2._on_odom()` (850-868): No function-level docstring; implementation is at the listed source range.
- `LocalNav2._stamp_ns()` (871-873): Convert a ROS builtin time stamp to nanoseconds.
- `LocalNav2._on_scan()` (875-886): No function-level docstring; implementation is at the listed source range.
- `LocalNav2._age_s()` (888-892): Return a non-negative source age, or None for missing time.
- `LocalNav2._point_validation_sample()` (894-944): Capture point-level upstream validation without affecting dispatch.
- `LocalNav2._last_tf_stamp_ns()` (946-955): Read the latest shared-map-to-base transform stamp for telemetry.
- `LocalNav2._record_point_validation_sample()` (957-980): Append only meaningful bounded target-cost transitions.
- `LocalNav2._emit_point_validation()` (982-1016): Publish compact diagnostic-only dispatch/transition lifecycle data.
- `LocalNav2._failure_snapshot()` (1018-1115): Serialize one bounded synchronized failure snapshot.
- `LocalNav2._pose()` (1117-1126): No function-level docstring; implementation is at the listed source range.
- `LocalNav2.evaluate_path()` (1128-1164): Start one bounded local path request; return false if busy/unavailable.
- `LocalNav2.path_start_failure_reason()` (1166-1168): Return why the most recent path request could not start.
- `LocalNav2.clear_path_query_priority()` (1170-1172): Drop an obsolete fallback priority request.
- `LocalNav2._request_path_priority()` (1174-1185): Publish a one-shot priority hint for the shared planner lease.
- `LocalNav2._clear_path_priority()` (1187-1193): No function-level docstring; implementation is at the listed source range.
- `LocalNav2._path_goal_response()` (1195-1209): No function-level docstring; implementation is at the listed source range.
- `LocalNav2._path_result()` (1211-1281): No function-level docstring; implementation is at the listed source range.
- `LocalNav2._finish_path()` (1283-1312): No function-level docstring; implementation is at the listed source range.
- `LocalNav2._acquire_path_query_lock()` (1314-1328): Serialize this robot's planner action with the C++ candidate node.
- `LocalNav2._release_path_query_lock()` (1330-1338): Release the bounded per-robot planner lease, if held.
- `LocalNav2.check_dispatch_preconditions()` (1340-1380): Asynchronously confirm local lifecycle plus map, costmap, and TF context.
- `LocalNav2._basic_preconditions()` (1382-1473): No function-level docstring; implementation is at the listed source range.
- `LocalNav2._log_blocked_local_path()` (1475-1613): Emit bounded point-level evidence for a local-path rejection. This is intentionally rejection-only telemetry. It repeats the read-only transform/grid lookup used by ``local_path_clearance`` and does not feed any value back into the dispatch decision.
- `LocalNav2.send_navigation()` (1615-1656): Send one already validated goal to this namespace's navigator only.
- `LocalNav2._navigation_goal_response()` (1658-1686): No function-level docstring; implementation is at the listed source range.
- `LocalNav2._navigation_feedback()` (1688-1692): No function-level docstring; implementation is at the listed source range.
- `LocalNav2.cancel_navigation()` (1694-1701): Request explicit cancellation of this namespace's active goal.
- `LocalNav2._navigation_result()` (1703-1775): No function-level docstring; implementation is at the listed source range.
- `LocalNav2._finish_navigation()` (1777-1789): No function-level docstring; implementation is at the listed source range.
- `LocalNav2._check_timeouts()` (1791-1820): No function-level docstring; implementation is at the listed source range.

### `src/my_epuck_project/my_epuck_project/distributed_assignment/models.py`

Purpose: Dataclasses and enums representing physical/canonical tasks, snapshots, bids, decisions, failures, lifecycle state, completion inputs, and travel distance.

Imported modules: `from dataclasses import dataclass, field`, `from enum import Enum`, `from typing import Any, Optional, Tuple`

Imported internal project modules: None.

Classes:
- `Bounds` (12-16): Axis-aligned world bounds for compact frontier geometry.
- `PhysicalTask` (20-48): One source robot's physical observation-task proposal.
- `TaskSnapshot` (52-70): Bounded task snapshot with ROS and lower-bound provenance.
- `CanonicalTask` (74-86): Deterministic union task produced from equivalent source proposals.
- `CanonicalUnion` (90-94): Canonical ordered physical task set for one coordination round.
- `Bid` (98-109): One robot's bounded local Nav2 evaluation of a canonical task.
- `BidBatch` (113-122): Round-bound bid array from one robot.
- `AssignmentScore` (126-142): Bounded, individually auditable pair-score terms.
- `AssignmentDiagnostics` (146-187): Deterministic explanation of a pair decision, including IDLE cases.
- `PairDecision` (191-204): Complete replicated assignment for Robot 1 and Robot 2.
- `FailureClass` (207-218): Evidence-conservative navigation failure classes.
- `FailureEvidence` (222-234): Only directly observed evidence used for classification.
- `FailureRecord` (238-250): Bounded team-visible failure evidence for one physical task.
- `CoordinatorState` (253-263): Small public coordinator lifecycle.
- `CompletionInputs` (267-285): Health and stability evidence required for operational completion.
- `TravelDistance` (289-305): Accumulate odometric distance while rejecting discontinuous jumps.

Top-level functions:
- None.

Methods:
- `TravelDistance.observe()` (296-305): Add one valid odometry displacement and return the total.

### `src/my_epuck_project/my_epuck_project/distributed_assignment/protocol.py`

Purpose: Receipt freshness, source-version/snapshot acceptance, bid validation, committed-round protection, peer liveness, and completion-state rules.

Imported modules: `from dataclasses import dataclass`, `from typing import Generic, Optional, TypeVar`, `from .models import BidBatch, CompletionInputs, CoordinatorState, PairDecision, TaskSnapshot`

Imported internal project modules: `.models`

Classes:
- `Received` (26-36): Message receipt whose age uses only the receiver's steady clock.
- `SourceVersion` (45-50): Last accepted source-local session, epoch, and map revision.
- `SnapshotLedger` (53-105): Reject stale snapshots without comparing revisions across sources.
- `CommittedRound` (127-158): Protect a committed navigation goal from delayed protocol messages.
- `PeerLiveness` (162-186): Steady-clock peer timeout and fresh-session recovery policy.

Top-level functions:
- `clamp_ttl()` (18-22): Clamp sender-advertised validity before local steady-clock use.
- `receive()` (39-41): Record a value with receiver-local expiry state.
- `bid_batch_valid()` (108-123): Validate all protocol bindings required before a bid can participate.
- `completion_state()` (189-216): Return operational completion only with fresh healthy matching evidence.

Methods:
- `Received.fresh()` (33-36): Return whether the locally measured receipt age is within TTL.
- `SnapshotLedger.__init__()` (56-60): Configure the per-source snapshot task bound.
- `SnapshotLedger.accept()` (62-95): Validate identity, bounds, session, epoch, and source-local revision.
- `SnapshotLedger.version()` (97-99): Return one source's accepted provenance state.
- `SnapshotLedger.compare_revisions()` (101-105): Compare only versions from the same source and same session.
- `CommittedRound.commit()` (133-136): Record positive peer agreement before dispatch.
- `CommittedRound.delayed_message_can_cancel()` (138-142): Old or unrelated rounds never cancel a committed local goal.
- `CommittedRound.explicit_reauction_allowed()` (144-158): Whitelist meaningful events that may invalidate committed navigation.
- `PeerLiveness.observe()` (170-178): Accept a peer heartbeat, requiring a new session after degraded mode.
- `PeerLiveness.evaluate()` (180-186): Enter degraded solo after a bounded receiver-local timeout.

### `src/my_epuck_project/my_epuck_project/distributed_assignment/ros_conversion.py`

Purpose: Conversions between ROS message representations and the internal task, snapshot, bid, and decision models.

Imported modules: `from dataclasses import asdict`, `import json`, `import math`, `from typing import Iterable`, `from builtin_interfaces.msg import Duration`, `from geometry_msgs.msg import Point`, `from my_epuck_interfaces.msg import PairDecision as PairDecisionMsg, PhysicalTask as PhysicalTaskMsg, TaskBid as TaskBidMsg, TaskBidArray as TaskBidArrayMsg, TaskSnapshot as TaskSnapshotMsg`, `from unique_identifier_msgs.msg import UUID`, `from .models import Bid, BidBatch, Bounds, PairDecision, PhysicalTask, TaskSnapshot`

Imported internal project modules: `.models`

Classes:
- None.

Top-level functions:
- `uuid_to_text()` (32-34): Convert a ROS UUID byte array to lowercase hexadecimal text.
- `text_to_uuid()` (37-44): Convert 32 hexadecimal characters to a ROS UUID.
- `duration_to_seconds()` (47-49): Convert a ROS duration without mixing it with a clock epoch.
- `seconds_to_duration()` (52-56): Convert bounded positive seconds to a ROS duration.
- `_point()` (59-60): No function-level docstring; implementation is at the listed source range.
- `_points()` (63-64): No function-level docstring; implementation is at the listed source range.
- `task_from_msg()` (67-111): Build one immutable task from its wire representation.
- `snapshot_from_msg()` (114-133): Build a snapshot while retaining source-local ROS provenance.
- `bid_from_msg()` (136-150): Convert one bounded path bid.
- `bid_batch_from_msg()` (153-163): Convert one round-bound bid array.
- `bid_batch_to_msg()` (166-191): Serialize deterministic bids with bounded path samples.
- `decision_to_msg()` (194-228): Serialize all fields required for positive replicated agreement.

Methods:
- None.

### `src/my_epuck_project/my_epuck_project/distributed_assignment/scoring.py`

Purpose: Assignment weights, motion-cost scoring, cost-only certificate evaluation, pair selection, MRTSP route selection, solo ranking, overlap, and decision fingerprints.

Imported modules: `from dataclasses import dataclass`, `import hashlib`, `import json`, `import math`, `from typing import Callable, Iterable, Mapping, Optional, Sequence`, `from .canonical import bounds_iou`, `from .models import AssignmentScore, AssignmentDiagnostics, Bid, BidBatch, CanonicalTask, CanonicalUnion, PairDecision, Point`

Imported internal project modules: `.canonical`, `.models`

Classes:
- `AssignmentWeights` (32-57): Pair-utility weights, geometry scales, and physical motion references. ``path`` and ``heading`` remain available for the historical scoring modes. ``frontier_cost_only`` deliberately does not use either weight: its path/heading tradeoff is expressed in seconds by the explicit motion references below.

Top-level functions:
- `_bounded()` (60-61): No function-level docstring; implementation is at the listed source range.
- `nominal_motion_cost_s()` (64-86): Return nominal translation plus initial reorientation time. This is a physically scaled frontier preference, not a Nav2 ETA and not an RPP trajectory simulation. Invalid values are rejected instead of silently becoming an attractive candidate.
- `cost_only_dispatch_certificate()` (89-168): Certify that unevaluated cost-only options cannot improve a decision. The unknown option bounds are optimistic motion costs. Pair penalties are deliberately set to zero here, making every unknown combination at least as attractive as it could be under the real ``_score_assignment``. A missing bound set is not certified. MRTSP never calls this helper.
- `_hash()` (171-175): No function-level docstring; implementation is at the listed source range.
- `bid_fingerprint()` (178-207): Fingerprint semantic bid data independent of message arrival order.
- `_path_overlap_one_way()` (210-217): No function-level docstring; implementation is at the listed source range.
- `route_overlap()` (220-228): Return bounded geometric corridor overlap, without traffic timing claims.
- `rank_solo_tasks()` (231-301): Order local tasks by policy, with bounded route reuse. The historical modes retain their generator-score ordering and bounded route reuse. ``frontier_cost_only`` uses only candidate-local actual path length and initial path-heading mismatch using the physical nominal motion cost; gain is absent from both the score and all tie-breaks.
- `nearby_goal_penalty()` (304-309): Penalize distinct goals that occupy the same local work region.
- `_visible_cell_overlap()` (312-319): No function-level docstring; implementation is at the listed source range.
- `sensing_overlap_estimate()` (322-336): Use visible cells when present, otherwise a bounded geometry approximation.
- `_score_assignment()` (339-456): No function-level docstring; implementation is at the listed source range.
- `_bid_map()` (459-477): No function-level docstring; implementation is at the listed source range.
- `_task_feasible()` (480-507): Apply explicit assignment safety/utility feasibility gates. Soft pair utility is deliberately not part of this predicate. A useful frontier may have negative absolute score after travel and overlap terms, but it must still be assignable when it is the best feasible work.
- `_assignment_rank()` (510-517): Keep the deterministic pair ranking independent of selection policy.
- `_is_conflict_free_two_active_assignment()` (520-533): Return whether a feasible pair has no cooperative-conflict signal. Feasibility is established before this predicate is evaluated. Exact zero is intentional: the geometry helpers clamp genuinely separated work to 0.0, while any nonzero value represents an existing soft conflict signal.
- `choose_pair_assignment()` (536-770): Exhaustively evaluate bounded ordered task pairs and idle cases. ``legacy_weighted`` and ``frontier_gain`` remain callable for historical fixture compatibility. Runtime policy selection uses only ``frontier_cost_only`` and ``frontier_mrtsp``; the latter is implemented by ``choose_mrtsp_route_assignment`` below. Cost-only uses physically scaled actual path and heading terms and never information gain.
- `_source_route_rank()` (773-787): Return the source-local upstream route rank for one robot/task pair. A bounded upstream DP route intentionally only ranks its selected horizon. Tasks outside that horizon are not silently promoted by the old local min-max scalar; they remain unavailable to the route-aware resolver until a fresh upstream route is published.
- `choose_mrtsp_route_assignment()` (790-1049): Resolve two distinct tasks from two upstream DP-ordered preferences. This intentionally uses ordinal route evidence, not the project generator's batch-normalized ``local_ordering_score``. Each peer's candidate generator has already run the vendored upstream MRTSP/DP solver over its local candidate set. If their first tasks collide physically, the lower *raw Nav2 path cost* owns that shared first preference and the other peer advances through its existing route order. A robot ID tie keeps replicas deterministic. No central state and no new scalar team objective are introduced here.
- `decisions_match()` (1052-1062): Require positive agreement on every decision-binding fingerprint.

Methods:
- None.

### `src/my_epuck_project/my_epuck_project/distributed_assignment/traffic_scheduler.py`

Purpose: Continuous sampled-path conflict geometry, progress projection, and deterministic traffic ordering for a new conflicting dispatch.

Imported modules: `from dataclasses import asdict, dataclass`, `import math`, `from typing import Sequence`, `from .models import Point`

Imported internal project modules: `.models`

Classes:
- `TrafficDecision` (15-44): Replicated geometry and priority evidence for one agreed task pair.

Top-level functions:
- `_sub()` (47-48): No function-level docstring; implementation is at the listed source range.
- `_dot()` (51-52): No function-level docstring; implementation is at the listed source range.
- `_add()` (55-56): No function-level docstring; implementation is at the listed source range.
- `_scale()` (59-60): No function-level docstring; implementation is at the listed source range.
- `_segment_distance()` (63-92): Closest distance and normalized positions on two finite 2-D segments.
- `_segments()` (95-105): No function-level docstring; implementation is at the listed source range.
- `detect_path_conflict()` (108-117): Compatibility wrapper returning the first joint conflict evidence.
- `detect_path_conflict_interval()` (120-166): Return conservative first/last distances for a continuous path conflict. Every segment pair whose minimum separation is within the required center separation contributes its closest-point distance along each route. The returned interval is the deterministic min/max envelope of those points. If separate conflict regions exist, the envelope intentionally covers the gap between them; that is conservative and avoids releasing a waiter while a later conflict on the same committed pair remains ahead.
- `project_path_progress()` (169-194): Project a point onto a sampled path. Returns ``(distance_along_path_m, lateral_distance_m)`` for the nearest point on any finite segment. It is deliberately small and stateless: the traffic hold owns the committed path and calls this on each timer tick.
- `schedule_traffic()` (197-248): Order one new conflicting dispatch by active state, ETA, then robot ID.

Methods:
- `TrafficDecision.as_dict()` (34-44): Return JSON-friendly bounded telemetry.

### `src/my_epuck_project/my_epuck_project/mission_termination.py`

Purpose: Terminal-reason types and conservative physical-frontier evidence summarization/classification.

Imported modules: `from dataclasses import dataclass`, `from enum import Enum`, `import math`, `from typing import Iterable, Mapping`, `from .frontier_actionability import is_actionable_reachable`

Imported internal project modules: `.frontier_actionability`

Classes:
- `TerminalReason` (13-26): Stable semantic reasons written to replicated status messages.
- `CandidateEvidence` (30-55): Bounded generator evidence needed to classify an empty task union.
- `FrontierRegionEvidence` (59-79): Compact geometry/status evidence for one physical frontier region.

Top-level functions:
- `credible_planner_infrastructure_failure()` (82-100): Require planner-query evidence plus an observed unhealthy stack. Candidate-specific NO_VALID_PATH and similar responses are useful evidence that a particular frontier is not executable, but they do not prove that the planner infrastructure is dead. The replicated abort path therefore requires both peers to have observed query failures and at least one peer to report unhealthy Nav2 infrastructure.
- `summarize_frontier_regions()` (109-198): Summarize unique physical regions without counting peer replicas twice. A physical ID can be present in both replicas with different local classifications. The summary is conservative: geometry uses the larger observed size and a blocking status wins over a terminal status.
- `evidence_from_mapping()` (201-223): Decode message-like evidence without trusting missing fields.
- `classify_empty_frontiers()` (226-263): Return success only when all observed frontier evidence is classified.
- `all_physical_tasks_suppressed()` (266-281): Return true only when every current task has hard evidence. A canonical task may contain equivalent source observations from both robots. Requiring every member signature to be suppressed prevents one robot's failed local approach from incorrectly declaring a task globally unreachable when the peer still has an executable view.
- `terminal_reason_is_success()` (284-286): Return whether a reason represents successful map exhaustion.
- `terminal_reason_is_abort()` (289-291): Return whether a reason represents abnormal mission termination.
- `matching_terminal_reason()` (294-300): Return a terminal result only when both replicas carry the same reason.
- `recommended_exit_code()` (303-309): Map terminal semantics to the compact runner result convention.

Methods:
- `CandidateEvidence.classified()` (52-55): Return the number of regions with a defensible terminal class.

### `src/my_epuck_project/my_epuck_project/frontier_actionability.py`

Purpose: Small eligibility predicates for frontier gain and reachable task actionability.

Imported modules: `from __future__ import annotations`, `import math`

Imported internal project modules: None.

Classes:
- None.

Top-level functions:
- `gain_meets_minimum()` (16-18): Return the production eligibility test: finite gain >= threshold.
- `is_actionable_reachable()` (21-33): Return whether a planner-reachable frontier is worth assigning.

Methods:
- None.

### `src/my_epuck_project/my_epuck_project/round_lifecycle.py`

Purpose: Monotonic round-generation leases for rejecting stale asynchronous work.

Imported modules: `from dataclasses import dataclass`, `from typing import Optional`

Imported internal project modules: None.

Classes:
- `RoundLease` (8-12): Immutable identity captured by work that may complete later.
- `RoundGeneration` (15-39): Monotonic round generation independent of ROS callback timing.

Top-level functions:
- None.

Methods:
- `RoundGeneration.__init__()` (18-20): No function-level docstring; implementation is at the listed source range.
- `RoundGeneration.activate()` (22-26): Start or replace a round and return its immutable lease.
- `RoundGeneration.invalidate()` (28-32): Invalidate the current round and return the new generation.
- `RoundGeneration.is_current()` (34-39): Return whether asynchronous work still belongs to this round.

## 2. Class/function index

The static index contains 23 top-level classes, 94 top-level functions, and 171 class methods. The per-file sections above provide the complete class/method/function index with inclusive line ranges and source-derived descriptions.

No runtime behavior was invoked to produce this index.

## 3. Responsibility grouping

### Candidate generation / filtering

`DistributedFrontierAssignment._candidate_callback()`, `_decode_frontier_regions()`, `_snapshot_callback()`; `frontier_actionability.gain_meets_minimum()`, `is_actionable_reachable()`; `TaskSnapshot`, `PhysicalTask`.

### Task canonicalization and identity

`canonical.py`: `TaskIdentity`, `canonical_round_id()`, `build_canonical_union()`, equivalence/geometry helpers; `models.py`: `PhysicalTask`, `CanonicalTask`, `CanonicalUnion`; allocator `_snapshot_content_fingerprint()` and `_log_union()`.

### Scoring and assignment

`scoring.py`: `AssignmentWeights`, `nominal_motion_cost_s()`, `rank_solo_tasks()`, `_score_assignment()`, `choose_pair_assignment()`, `choose_mrtsp_route_assignment()`, `decisions_match()`; allocator `_continue_bidding_impl()`, `_append_bid()`, `_finish_bids()`, `_publish_decision()`.

### Bid generation and path evaluation

Allocator `_continue_bidding()`, `_continue_bidding_impl()`, `_append_bid()`, `_finish_bids()`; `LocalNav2.evaluate_path()`, `_path_goal_response()`, `_path_result()`, `_finish_path()`; `Bid`, `BidBatch`, `PathEvaluation`.

### Peer communication and wire conversion

Allocator `_activate_protocol_inputs()`, `_candidate_callback()`, `_snapshot_callback()`, `_bid_callback()`, `_decision_callback()`, `_status_callback()`, `_failure_callback()`, `_peer_event_callback()`; `protocol.py`; `ros_conversion.py`; ROS publishers/subscribers configured in `__init__()`.

### Round lifecycle and stale-work protection

`RoundGeneration`, `RoundLease`; allocator `RoundWork`, `_activate_round()`, `_round_is_current()`, `_discard_stale_tick()`, `_reset_round()`, `_transition()`; `SnapshotLedger`, `CommittedRound`.

### Certificate logic

Allocator `classify_lower_bound_evidence()`, `_cost_only_certificate_blocker_diagnostics()`, `_cost_only_dispatch_certificate()`; scoring `cost_only_dispatch_certificate()`; task/bid provenance fields in `TaskSnapshot` and `BidBatch`.

### Commitment and continuation handling

`ActiveCommitment`, `ContinuationContext`, `TrafficHold`; allocator `_remember_active_commitments()`, `_clear_active_commitment()`, `_continuation_context()`, `_continuation_round_id()`, `_activate_continuation_round()`, `_continuation_busy_batch()`, `_continuation_bid_batches()`.

### Traffic / conflict handling

`TrafficDecision`, `detect_path_conflict()`, `detect_path_conflict_interval()`, `project_path_progress()`, `schedule_traffic()`; allocator `_traffic_for_bid_pair()`, `_traffic_for_decision()`, `_begin_traffic_wait()`, `_continue_traffic_hold()`, `_select_traffic_test_conflict_pair()`.

### Dispatch and Nav2 interaction

`LocalNav2.check_dispatch_preconditions()`, `_basic_preconditions()`, `send_navigation()`, `_navigation_goal_response()`, `_navigation_feedback()`, `_navigation_result()`, `_finish_navigation()`; allocator `_start_local_dispatch()`, `_dispatch_after_checks()`, `_navigation_finished()`.

### Logging, status, and telemetry

Allocator `_startup_event()`, `_publish_status()`, `_emit_event()`, `_log_round_lifecycle()`, `_publish_health_diagnostic()`, allocator timing helpers, `_log_traffic_aware_selection()`, and `LocalNav2` point/failure snapshot methods.

### Recovery, failure, and termination

`failures.py`: `FailureClass`, `FailureSuppressor`, `classify_failure()`, `bounded_suppression_duration()`; allocator `_record_hard_failure()`, `_publish_failure()`, `_invalidate_round()`, `_expire_failures()`, `_consider_completion()`, `_set_terminal()`; `mission_termination.py` terminal/evidence functions; `PeerLiveness` and `completion_state()`.

## 4. Runtime flow reconstruction

The included allocator is a replicated peer: each robot runs `DistributedFrontierAssignment`; there is no central coordinator class in this scope. The same logical decision is formed from source-local snapshots, exchanged bids, and positive peer agreement.

| Stage | Responsible implementation |
|---|---|
| Frontier discovered | The upstream candidate source is outside this scope. The allocator receives `FrontierCandidateArray` in `_candidate_callback()`. |
| Candidate created/filtered | `_candidate_callback()` decodes candidate and region evidence, applies available actionability/path metadata, records candidate evidence, and prepares source-local snapshot state. `_decode_frontier_regions()` handles compact region evidence. |
| Task snapshot accepted | `_snapshot_callback()` converts/accepts source-local task provenance and uses `SnapshotLedger`/`Received` freshness rules. `TaskSnapshot` and `PhysicalTask` carry the bounded task representation. |
| Canonical task union formed | `_tick_impl()` obtains fresh snapshots, computes a canonical round ID with `canonical_round_id()`, and calls `build_canonical_union()` to produce `CanonicalUnion`/`CanonicalTask` values. |
| Local task selected for evaluation | `_continue_bidding()` / `_continue_bidding_impl()` iterate bounded union tasks. `rank_solo_tasks()` is used for local policy paths; `LocalNav2.evaluate_path()` obtains path evidence. |
| Bid generated | `_append_bid()` constructs `Bid`; `_finish_bids()` builds/validates the `BidBatch`; `_publish_local_bid_batch()` publishes the round-bound bid array. `LocalNav2._path_result()` and `_finish_path()` complete asynchronous path evaluation. |
| Peer exchange | `_bid_callback()` receives peer bids; `bid_batch_from_msg()` converts them; `bid_batch_valid()` and `_both_bid_batches_valid()` enforce round, snapshot, and freshness bindings. |
| Round formed | `_activate_round()` installs `RoundWork` and a `RoundGeneration` lease. Continuation rounds use `_continuation_context()`, `_continuation_round_id()`, and `_activate_continuation_round()`. |
| Pair assignment selected | `choose_pair_assignment()` or `choose_mrtsp_route_assignment()` evaluates the bounded bid union. The allocator attaches traffic diagnostics through `_traffic_for_decision()`. |
| Certificate evaluated | `_cost_only_dispatch_certificate()` in the allocator gathers provenance and blocker diagnostics, then calls pure `scoring.cost_only_dispatch_certificate()`. A failed certificate transitions to `WAITING_FOR_INPUTS` and may only do bounded fallback work; it does not dispatch the pair. |
| Positive peer agreement/commitment | `_publish_decision()` publishes the decision; `_decision_callback()` and `_matching_peer_decision()` verify replicated agreement; `CommittedRound.commit()` protects the accepted commitment from stale protocol messages. |
| Traffic checked | `_traffic_for_bid_pair()` / `_traffic_for_decision()` call `schedule_traffic()`. Conflict results may enter `_begin_traffic_wait()` and `TrafficHold`; `_continue_traffic_hold()` releases or re-evaluates. |
| Final local dispatch checks | `_start_local_dispatch()` requests/reuses final path evidence and calls `LocalNav2.check_dispatch_preconditions()`. `_dispatch_after_checks()` validates TF/map/costmap/action state and sends only a valid local goal. |
| Nav2 execution | `LocalNav2.send_navigation()` and its action callbacks own the local goal lifecycle; allocator status/events reflect the commitment. |
| Terminal callback | `_navigation_finished()` emits success/failure, clears the active commitment, resets the round, applies suppression bookkeeping, and calls `_start_immediate_fallback_after_terminal()`. |

Where the frontier generator itself discovers regions and runs upstream path candidate production is outside the included allocator files; only its ROS input boundary is inventoried here.

## 5. Complexity hotspots

| Component | Size/shape | Why it is structurally complex |
|---|---|---|
| `DistributedFrontierAssignment` | 4,608-line class range; 95 methods | Owns ROS lifecycle, replicated state, snapshots, bidding, certificates, continuations, traffic, dispatch, terminal behavior, and telemetry. |
| `distributed_frontier_assignment.py` orchestration | 5,076 LOC; `_tick_impl()` 522 LOC; `__init__()` 535 LOC | The main timer and constructor cross most responsibility families and many asynchronous callbacks. |
| `LocalNav2` | 1,303-line class range; 49 methods | Combines action clients, map/costmap/TF/odom/scan caches, path leases, preconditions, failure snapshots, navigation callbacks, and timeouts. |
| `scoring.py` | 1,062 LOC; pair selectors span 235 and 260 lines | Contains several selectable scoring modes, exhaustive pair/idle selection, route-aware selection, motion models, overlap, and certificate predicate. |
| Certificate path | allocator diagnostics 202 LOC + certificate gate 132 LOC + pure scoring certificate 80 LOC | Combines candidate-bound provenance, blocker history, optimistic bounds, and dispatch safety. |
| Async round/dispatch path | `_continue_bidding_impl()`, `_publish_decision()`, `_start_local_dispatch()`, `_dispatch_after_checks()`, `_navigation_finished()` | Uses callback state, generation checks, cached path context, peer messages, and final rechecks across one logical dispatch. |

Static symbol counts: 13 files; 23 top-level classes; 94 top-level functions; 171 class methods. These counts include small dataclasses/enums and private helpers, not tests.

## 6. Potential architectural concerns (observation only)

- `DistributedFrontierAssignment` is the dominant ownership boundary: it contains transport callbacks, protocol state, scoring orchestration, certificate policy, traffic policy, dispatch gates, terminal semantics, and telemetry in one class.
- `LocalNav2` is a second large ownership boundary that mixes ROS action integration, current sensor/map state, path-query scheduling, final preconditions, navigation outcomes, and diagnostic serialization.
- Scoring policy and certificate policy are in the same module (`scoring.py`) even though certificate evaluation is a safety gate rather than ordinary assignment ranking.
- There are multiple lifecycle concepts in parallel: `RoundGeneration`, `RoundWork`, `CommittedRound`, `ActiveCommitment`, `ContinuationContext`, and `TrafficHold`. Their source-level names show distinct semantics, but their transitions are coordinated by the main allocator class.
- ROS conversion and protocol freshness are separated into helpers, while source provenance is also carried inside model dataclasses and checked in allocator callbacks; ownership is distributed across these boundaries.
- Telemetry is interleaved with decision code. The allocator timing instrumentation is opt-in, but startup/status/event/health reporting is part of the same class as scheduling.
- `mission_termination.py` and `frontier_actionability.py` are small and comparatively focused; they are direct dependencies of terminal classification and eligibility rather than alternate allocators.
- `burgard_assignment.py` is intentionally excluded here because it is not imported by the canonical live allocator path; it is a separate legacy/pre-demo/test-supported assignment implementation.

These are structural observations only. This inventory does not classify any boundary as incorrect and does not propose a refactor.

## 7. Final summary

### 1. Five largest components

By physical source size within the included scope:

1. `distributed_frontier_assignment.py` — 5,076 LOC.
2. `distributed_assignment/local_nav2.py` — 1,820 LOC.
3. `distributed_assignment/scoring.py` — 1,062 LOC.
4. `mission_termination.py` — 309 LOC.
5. `distributed_assignment/models.py` — 305 LOC.

### 2. Where a frontier assignment travels

`FrontierCandidateArray` → `_candidate_callback()` → source `TaskSnapshot`/`PhysicalTask` → `_tick_impl()` → `build_canonical_union()` → `_continue_bidding_impl()` and `LocalNav2.evaluate_path()` → `Bid`/`BidBatch` → `_publish_local_bid_batch()` and peer `_bid_callback()` → `choose_pair_assignment()`/`choose_mrtsp_route_assignment()` → `_cost_only_dispatch_certificate()` → `_publish_decision()`/`_matching_peer_decision()` → `schedule_traffic()`/`TrafficHold` → `_start_local_dispatch()` → `check_dispatch_preconditions()` → `send_navigation()` → `_navigation_finished()` → round reset and next scheduling trigger.

### 3. Files to study first for reducing idle latency

This is a study order, not a recommendation to change them:

1. `distributed_frontier_assignment.py`, especially `_tick_impl()`, `_continue_bidding_impl()`, `_cost_only_dispatch_certificate()`, `_publish_decision()`, `_start_local_dispatch()`, and `_navigation_finished()`.
2. `distributed_assignment/scoring.py`, especially `cost_only_dispatch_certificate()` and `choose_pair_assignment()`.
3. `distributed_assignment/local_nav2.py`, especially path-query lease/result methods and `check_dispatch_preconditions()`.
4. `distributed_assignment/protocol.py`, for snapshot/bid freshness and peer-liveness waits.
5. `distributed_assignment/canonical.py` and `models.py`, for candidate identity, generation, and canonical-union changes that affect evidence reuse.

### 4. Files not to touch casually because they are safety/protocol-critical

`distributed_assignment/protocol.py`, `distributed_assignment/traffic_scheduler.py`, `distributed_assignment/scoring.py` certificate logic, `distributed_frontier_assignment.py` commitment/dispatch/terminal methods, `distributed_assignment/local_nav2.py` final preconditions and navigation result handling, `round_lifecycle.py`, and `mission_termination.py`. These components define freshness, stale-work rejection, traffic conflict handling, final dispatch checks, terminal semantics, or failure/termination evidence.

## Measurement conclusion

The canonical allocator/coordinator runtime in this scope is **9,750 physical LOC / 9,098 nonblank LOC**. The live center of gravity is the 5,076-line allocator node plus the 1,820-line local Nav2 integration and 1,062-line scoring module; the remaining files provide typed models, protocol, conversion, geometry, traffic, failure, termination, and round-validity support.

