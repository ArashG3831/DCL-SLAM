# Allocator round-formation fix architecture archaeology

Date: 2026-09-08  
Scope: read-only allocator/cooperation/protocol archaeology; no source, build,
ROS, Webots, or runtime changes.

## Final verdict

**MINIMAL_SAFE_ALLOCATOR_FIX_IDENTIFIED**

The smallest defensible change is a bounded, local cache of successful
`ComputePathToPose` evaluations across allocator round replacement. The cache
must be used only when the complete task execution key is identical and
`LocalNav2.path_context_matches()` still proves the current map and costmap
context is the one used for the cached path. The bid must still be rebuilt and
bound to the current round, union, source session, and snapshot epoch.

This removes repeat local planner work caused solely by round churn without
reusing a peer bid, accepting a stale snapshot, relaxing a certificate, or
altering a commitment. The current evidence identifies planner/query lease
waiting as a real secondary contributor, including an 11.50 s overlap in the
longest terminal-to-dispatch interval; it does not justify weakening the
freshness or agreement gates.

## Evidence and boundaries

The preserved query-cap-8 archaeology report establishes that successful
same-round certificate-to-dispatch latency is only 0.12--0.34 s. The remaining
delay is upstream: fresh peer snapshots, exact peer-bid synchronization,
epoch-derived canonical round replacement, commitment validity, and local
planner-lease waits. This report maps those observations to the current source
and identifies a source change that can be isolated from the frozen protocol.

Relevant implementation references use the current checkout:

- [distributed_frontier_assignment.py](/home/arash/webots_ws_clean_validation_20260823/src/my_epuck_project/my_epuck_project/distributed_frontier_assignment.py)
- [protocol.py](/home/arash/webots_ws_clean_validation_20260823/src/my_epuck_project/my_epuck_project/distributed_assignment/protocol.py)
- [canonical.py](/home/arash/webots_ws_clean_validation_20260823/src/my_epuck_project/my_epuck_project/distributed_assignment/canonical.py)
- [frontier_proposal_adapter.py](/home/arash/webots_ws_clean_validation_20260823/src/my_epuck_project/my_epuck_project/frontier_proposal_adapter.py)
- [local_nav2.py](/home/arash/webots_ws_clean_validation_20260823/src/my_epuck_project/my_epuck_project/distributed_assignment/local_nav2.py)

## Current mechanism

```text
terminal / robot becomes available
        |
        v
frontier candidates -- FrontierProposalAdapter --> TaskSnapshot
        |                                             |
        |                                             +--> freshness/session/epoch gate
        v
bounded local path queries                         |
        |                                             v
        +--> local BidBatch <---- peer BidBatch --- canonical union and round
                                                        |
                                                        v
                                              pair decision + exact agreement
                                                        |
                                                        v
                                             cost-only certificate
                                                        |
                                                        v
                                         final path/TF/Nav2/traffic gates
                                                        |
                                                        v
                                                    dispatch
```

The runtime order is implemented by `_tick_impl()`:

1. It obtains both fresh snapshots before normal cooperative work. If either
   is absent or expired, it enters the existing local-wait/degraded-solo path
   and does not form a normal pair ([distributed_frontier_assignment.py:2493-2511](/home/arash/webots_ws_clean_validation_20260823/src/my_epuck_project/my_epuck_project/distributed_frontier_assignment.py:2493)).
2. It hashes the two source sessions and snapshot epochs into the canonical
   round ID ([distributed_frontier_assignment.py:2532-2536](/home/arash/webots_ws_clean_validation_20260823/src/my_epuck_project/my_epuck_project/distributed_frontier_assignment.py:2532); [canonical.py:36-43](/home/arash/webots_ws_clean_validation_20260823/src/my_epuck_project/my_epuck_project/distributed_assignment/canonical.py:36)).
3. A changed round ID creates a new `RoundWork`, caps the query list at eight,
   clears exchanged bid batches and peer decision state, and enters `BIDDING`
   ([distributed_frontier_assignment.py:2577-2598](/home/arash/webots_ws_clean_validation_20260823/src/my_epuck_project/my_epuck_project/distributed_frontier_assignment.py:2577)).
4. Each replica builds its local bid batch. Stored reachable candidate paths
   are reused, otherwise `evaluate_path()` acquires the per-robot serialized
   planner lease and starts one asynchronous query
   ([distributed_frontier_assignment.py:3920-3980](/home/arash/webots_ws_clean_validation_20260823/src/my_epuck_project/my_epuck_project/distributed_frontier_assignment.py:3920); [local_nav2.py:1128-1164](/home/arash/webots_ws_clean_validation_20260823/src/my_epuck_project/my_epuck_project/distributed_assignment/local_nav2.py:1128)).
5. The pair cannot score until both batches are fresh and match the current
   source session, snapshot epoch, round ID, and union hash
   ([distributed_frontier_assignment.py:4070-4083](/home/arash/webots_ws_clean_validation_20260823/src/my_epuck_project/my_epuck_project/distributed_frontier_assignment.py:4070); [protocol.py:108-123](/home/arash/webots_ws_clean_validation_20260823/src/my_epuck_project/my_epuck_project/distributed_assignment/protocol.py:108)).
6. The selected decision must match the peer on the same round, union, source
   epochs, bid fingerprints, task IDs, and decision hash before local dispatch
   ([distributed_frontier_assignment.py:4226-4253](/home/arash/webots_ws_clean_validation_20260823/src/my_epuck_project/my_epuck_project/distributed_frontier_assignment.py:4226)).
7. Final dispatch still performs the current path, TF, map, costmap, lifecycle,
   traffic, and active-goal checks; an allocator bid path is reused only when
   its current path context matches ([distributed_frontier_assignment.py:4358-4386](/home/arash/webots_ws_clean_validation_20260823/src/my_epuck_project/my_epuck_project/distributed_frontier_assignment.py:4358); [local_nav2.py:841-848](/home/arash/webots_ws_clean_validation_20260823/src/my_epuck_project/my_epuck_project/distributed_assignment/local_nav2.py:841)).

## 1. Peer snapshot freshness gate

### Where it is enforced

`Received.fresh()` uses only the receiver's monotonic receipt time and the
accepted sender TTL ([protocol.py:25-41](/home/arash/webots_ws_clean_validation_20260823/src/my_epuck_project/my_epuck_project/distributed_assignment/protocol.py:25)). The proposal adapter publishes snapshots with an 8.0 s default validity
([frontier_proposal_adapter.py:48-58](/home/arash/webots_ws_clean_validation_20260823/src/my_epuck_project/my_epuck_project/frontier_proposal_adapter.py:48)). The allocator reads only a fresh value in `_fresh_snapshot()`
([distributed_frontier_assignment.py:1725-1727](/home/arash/webots_ws_clean_validation_20260823/src/my_epuck_project/my_epuck_project/distributed_frontier_assignment.py:1725)). Normal cooperative round formation requires both values fresh.

`SnapshotLedger.accept()` separately rejects invalid source identity, missing
session/epoch, oversized snapshots, task/source mismatches, duplicate or older
epochs, map-revision regression, and messages from a retired session
([protocol.py:53-95](/home/arash/webots_ws_clean_validation_20260823/src/my_epuck_project/my_epuck_project/distributed_assignment/protocol.py:53)). This is not an arbitrary timeout: a stale proposal cannot stand in for current peer work, source identity, or source-local map/candidate provenance.

### Are unchanged snapshots unnecessarily invalidated?

There is already one narrow exemption. An exact immutable retransmission that
the ledger rejects is received as a heartbeat and refreshes the receipt time
([distributed_frontier_assignment.py:1529-1537](/home/arash/webots_ws_clean_validation_20260823/src/my_epuck_project/my_epuck_project/distributed_frontier_assignment.py:1529)). Status messages independently refresh peer liveness, so a navigating peer does not have to continue publishing proposals ([distributed_frontier_assignment.py:1614-1643](/home/arash/webots_ws_clean_validation_20260823/src/my_epuck_project/my_epuck_project/distributed_frontier_assignment.py:1614)).

The normal proposal path is different. Every candidate-array callback
increments the adapter epoch, even when the visible task set appears similar
([frontier_proposal_adapter.py:118-148](/home/arash/webots_ws_clean_validation_20260823/src/my_epuck_project/my_epuck_project/frontier_proposal_adapter.py:118)). The allocator's semantic fingerprint intentionally ignores epoch-only changes, but it includes rounded task fields and does not include every certificate/provenance field, such as `candidate_generation_id` and `generation_ros_ns` ([distributed_frontier_assignment.py:2151-2182](/home/arash/webots_ws_clean_validation_20260823/src/my_epuck_project/my_epuck_project/distributed_frontier_assignment.py:2151)). It is therefore not safe to treat this fingerprint alone as proof that a new snapshot is equivalent for certificate purposes.

The certificate path explicitly checks lower-bound map revision, context
fingerprint, candidate-generation ID, and related provenance; a mismatch
conservatively produces no usable bound ([distributed_frontier_assignment.py:132-137](/home/arash/webots_ws_clean_validation_20260823/src/my_epuck_project/my_epuck_project/distributed_frontier_assignment.py:132); [distributed_frontier_assignment.py:3135-3160](/home/arash/webots_ws_clean_validation_20260823/src/my_epuck_project/my_epuck_project/distributed_frontier_assignment.py:3135)). That rules out a safe change that merely ignores newer epochs or extends snapshot TTL.

### Delay conclusion

Fresh-snapshot waiting is a confirmed upstream allocator gate. The current
code has no proof that the newer proposal is only a heartbeat unless the full
certificate-relevant evidence is identical. The exact-immutable retransmission
path is already the safe no-change case.

## 2. Peer bid synchronization

`_bid_callback()` stores the most recently received batch with receiver-local
freshness ([distributed_frontier_assignment.py:1595-1605](/home/arash/webots_ws_clean_validation_20260823/src/my_epuck_project/my_epuck_project/distributed_frontier_assignment.py:1595)). It does not independently decide that a batch belongs to the active round. `_both_bid_batches_valid()` delegates that decision to `bid_batch_valid()`, which requires:

- receiver-local bid freshness;
- the expected source robot and session;
- the exact snapshot epoch;
- the exact canonical round ID;
- the exact union hash; and
- unique task IDs.

The current allocator therefore waits for a real peer batch whenever the
current round changed. A locally valid bid from the previous round is not
enough: using it would mix proposals, path costs, or source epochs from
different evidence sets. The local batch is refreshed as a heartbeat only
within its existing round ([distributed_frontier_assignment.py:4047-4062](/home/arash/webots_ws_clean_validation_20260823/src/my_epuck_project/my_epuck_project/distributed_frontier_assignment.py:4047)).

The local side also may be waiting for actual computation rather than peer
transport. `_continue_bidding_impl()` processes the bounded query list one
task at a time. A source-provided valid path is reused immediately; otherwise
the call can fail with `PATH_QUERY_LEASE_BUSY`, leaves the round alive, and
retries on a later tick. The lease serializes allocator queries with the same
robot's C++ frontier-candidate planner work ([local_nav2.py:1314-1328](/home/arash/webots_ws_clean_validation_20260823/src/my_epuck_project/my_epuck_project/distributed_assignment/local_nav2.py:1314)).

### Continuation exception

Continuation is the existing safe reuse mechanism. When one robot is
navigating, `_continuation_context()` requires fresh peer status, a finite
immutable active commitment, valid session/task identity, and a fresh snapshot
only for the free robot ([distributed_frontier_assignment.py:1948-2005](/home/arash/webots_ws_clean_validation_20260823/src/my_epuck_project/my_epuck_project/distributed_frontier_assignment.py:1948)). The busy side then gets a synthetic bid from the immutable commitment path; only the free side supplies a new bid ([distributed_frontier_assignment.py:2090-2149](/home/arash/webots_ws_clean_validation_20260823/src/my_epuck_project/my_epuck_project/distributed_frontier_assignment.py:2090)). Existing tests explicitly cover this, while a normal pair still requires both batches ([test_distributed_continuation_allocation.py:189-238](/home/arash/webots_ws_clean_validation_20260823/src/my_epuck_project/test/test_distributed_continuation_allocation.py:189); [test_distributed_continuation_allocation.py:358-363](/home/arash/webots_ws_clean_validation_20260823/src/my_epuck_project/test/test_distributed_continuation_allocation.py:358)).

### Delay conclusion

The dominant peer-bid delay is strict synchronization to a new current round,
not an unexplained recomputation of an already valid same-round batch. Local
planner-lease wait is a separate, measured secondary source. Reusing a normal
peer bid across a changed epoch would be a protocol change, not a harmless
cache optimization.

## 3. Canonical round churn

### Required causes

The canonical round ID contains both source sessions and both snapshot epochs
([canonical.py:36-43](/home/arash/webots_ws_clean_validation_20260823/src/my_epuck_project/my_epuck_project/distributed_assignment/canonical.py:36)). Consequently, any accepted newer proposal from either source changes the normal round ID. The adapter increments the source epoch on each candidate callback, and the allocator creates a replacement when the computed ID differs.

The following invalidations are required correctness boundaries:

| Event | Why reset/replacement is required |
|---|---|
| New source session | Old messages and commitments must not cross a source restart. |
| Changed snapshot epoch/provenance | Existing bids and decisions are bound to the old evidence. |
| Navigation terminal result | Robot availability and completed-task suppression change. |
| Accepted dispatch/path/TF failure | The feasible set changes and failure suppression must affect the next round. |
| Commitment ended, session changed, or task identity changed | Continuation must not retain a goal that is no longer active or owned. |
| Traffic conflict cleared/released | The old deferred goal is explicitly discarded and fresh proposals are required. |

`_activate_round()` increments the generation and records replacement, while
the normal tick clears bid batches, peer decision, and committed state when a
new canonical round is installed ([distributed_frontier_assignment.py:2227-2242](/home/arash/webots_ws_clean_validation_20260823/src/my_epuck_project/my_epuck_project/distributed_frontier_assignment.py:2227); [distributed_frontier_assignment.py:2577-2598](/home/arash/webots_ws_clean_validation_20260823/src/my_epuck_project/my_epuck_project/distributed_frontier_assignment.py:2577)). The generation guard then discards asynchronous completions belonging to the replaced round ([round_lifecycle.py:15-39](/home/arash/webots_ws_clean_validation_20260823/src/my_epuck_project/my_epuck_project/round_lifecycle.py:15)).

### Potentially avoidable cause

An epoch-only update whose complete certificate-relevant proposal evidence is
identical to the current in-progress round could theoretically avoid replacing
that round. The current semantic fingerprint is not sufficient to authorize
this: it rounds values, omits candidate-generation provenance, and does not
bind the exact lower-bound evidence. A future semantic-coalescing change would
need a new exact equivalence predicate and a carefully defined rule for which
received snapshot remains the authoritative round snapshot. Without that, the
round churn is conservative and required.

The existing special cases do not solve actionable-round churn. They suppress
work only for an already IDLE decision with unchanged content, or after the
round has been cleared following a matching decision
([distributed_frontier_assignment.py:2537-2575](/home/arash/webots_ws_clean_validation_20260823/src/my_epuck_project/my_epuck_project/distributed_frontier_assignment.py:2537)). They do not preserve an in-progress pair whose bid arrays or certificate are still pending.

## 4. Commitment handling

After agreement, `_remember_active_commitments()` retains only the selected
task/path and source session/epoch needed for continuation; it deliberately
does not retain the old frontier bid vector ([distributed_frontier_assignment.py:1880-1935](/home/arash/webots_ws_clean_validation_20260823/src/my_epuck_project/my_epuck_project/distributed_frontier_assignment.py:1880)). This is the correct boundary: an active goal can reserve its exact task and traffic path, but its old unselected proposals are not current evidence.

The peer commitment is cleared when the peer becomes inactive, changes source
session, or advertises a non-empty different task. An empty optional task field
does not clear an otherwise fresh active commitment ([distributed_frontier_assignment.py:1614-1643](/home/arash/webots_ws_clean_validation_20260823/src/my_epuck_project/my_epuck_project/distributed_frontier_assignment.py:1614)). Continuation then requires fresh peer status and identity checks, and invalidates the continuation round if the commitment disappears ([distributed_frontier_assignment.py:1937-2005](/home/arash/webots_ws_clean_validation_20260823/src/my_epuck_project/my_epuck_project/distributed_frontier_assignment.py:1937)).

These invalidations protect real failure modes: a delayed status or goal
completion must not cause a free robot to reserve a task that the peer no
longer owns; a session restart must not reuse old source evidence; and an
active navigation failure must not remain an immutable successful commitment.
The existing continuation path already survives harmless candidate-epoch churn
on the busy side; the dedicated test
`test_continuation_round_id_ignores_busy_candidate_epoch_changes()` covers that
behavior ([test_distributed_continuation_allocation.py:306-319](/home/arash/webots_ws_clean_validation_20260823/src/my_epuck_project/test/test_distributed_continuation_allocation.py:306)). No further commitment relaxation is justified by the source evidence.

## 5. Exact delay-source summary

| Delay source | Source mechanism | Status | Safe to relax now? |
|---|---|---|---|
| Fresh peer snapshot | Receiver-local TTL plus both-fresh normal cooperative gate | Confirmed upstream gate | No; freshness protects current source evidence. |
| Peer-bid synchronization | Exact round/union/session/epoch/freshness binding | Confirmed dominant non-certificate gate in the long tail | No; cross-round reuse would change protocol semantics. |
| Canonical round churn | Source session/epoch in round hash; replacement clears batches/decision | Confirmed; epoch-only churn may be avoidable only with full provenance proof | Not with the current rounded content fingerprint. |
| Local planner/query lease | One serialized per-robot planner lease; retry preserves round | Confirmed secondary source | Yes, by bounded exact path-result reuse only. |
| Commitment validity | Active task/path/session identity and fresh status checks | Required correctness gate; continuation already reuses safe state | No relaxation justified. |
| Traffic | Scheduler runs after pair selection and before dispatch | Not the long pre-certificate owner in the preserved query-cap-8 evidence | No change indicated. |
| Certificate | Conservative unqueried-candidate bound checks | Earlier dominant source, not the proposed target here | Frozen. |

Increasing snapshot or bid TTLs would not be a defensible fix. It would allow
older evidence to participate longer while leaving the round/epoch mismatch
logic intact, and it would weaken the fail-closed freshness contract rather
than reduce the measured work.

## Ranked candidate fixes

### 1. Safest/minimal: bounded exact local path-evaluation cache

**Location:** the allocator's local bid path around
`_continue_bidding_impl()`; no protocol message or certificate function needs
to change.

**Change shape:** retain a small bounded cache of successful finite
`PathEvaluation` values after a round is replaced. The key must include the
complete canonical task execution input, including the physical/source
provenance and target pose/path-relevant fields, not only `canonical_id`. A
cached value is eligible only when:

1. the current task key is exact;
2. the cached result is valid, finite, and was produced by an allocator path
   query;
3. `LocalNav2.path_context_matches(cached)` is true for the current map and
   costmap; and
4. the result is converted into a newly constructed bid bound to the current
   `RoundWork`, current union, current snapshot epoch, and current round ID.

Invalid results are not cached as durable evidence. A cache miss keeps the
existing serialized planner request and retry behavior. The final dispatch
path remains unchanged and continues to recheck TF, map, costmap, lifecycle,
traffic, active-goal state, and the final path.

**Expected benefit:** eliminates repeated local `ComputePathToPose` calls and
their lease waits when a newer peer proposal replaces a round without changing
the local task/path context. This is expected to reduce the measured planner
portion of round formation, not the separate wait for a genuinely missing
peer snapshot or peer bid. The preserved archaeology reports 12.80 s combined
planner/query overlap in the long terminal rows and an 11.50 s single overlap;
these are an upper-bound indication of the opportunity, not a guaranteed
speedup.

**Correctness risk:** low if the key and context checks are exact. The cache
must never rebind a path to changed task geometry, source provenance, map, or
costmap state. It must not be used to accept a bid or certificate from an old
round.

**Required focused tests:**

- same exact task plus matching current map/costmap reuses the path and emits a
  new current-round bid;
- changed approach, orientation, geometry, source epoch, map revision,
  candidate-generation provenance, or task generation misses the cache;
- changed map/costmap stamp misses the cache and invokes the existing lease;
- invalid/nonfinite results are never cached;
- a replaced round still requires the peer's current exact bid and current
  `bid_batch_valid()` bindings;
- final dispatch still executes the existing current-context and safety gates;
- cache size/eviction is bounded and does not alter selection ordering.

The existing lease tests in
[test_distributed_frontier_round_guard.py:577-625](/home/arash/webots_ws_clean_validation_20260823/src/my_epuck_project/test/test_distributed_frontier_round_guard.py:577)
and exact bid tests in
[test_distributed_protocol.py:82-105](/home/arash/webots_ws_clean_validation_20260823/src/my_epuck_project/test/test_distributed_protocol.py:82)
are the nearest regression base.

### 2. Moderate: exact certificate-aware semantic round coalescing

**Change shape:** add a new exact equivalence predicate for an in-progress
round. It would have to include source session, map revision/fingerprint,
lower-bound context, candidate-generation ID, generation stamp, all exact task
execution fields, and any other field consumed by certificate or pair scoring.
Only a proven equivalent update could avoid replacing the current round; all
other updates would retain today's behavior.

The difficult part is not the hash. A normal bid and decision are explicitly
bound to source snapshot epochs and round ID. A safe implementation would need
to specify whether the old round remains authoritative while a newer received
snapshot is kept only as liveness, or whether all bid/certificate provenance
is reissued under a new round. Rebinding old peer messages is not safe by
default.

**Expected benefit:** potentially removes round replacement, bid rebuilding,
and peer synchronization for genuinely evidence-identical candidate
callbacks. The benefit could be meaningful if telemetry proves that many
replacements are epoch-only; source inspection alone does not establish that
rate.

**Correctness risk:** moderate to high. The current content fingerprint is not
an adequate predicate, and the `TaskSnapshot` model explicitly separates
proposal evidence from heartbeat/liveness. A bug here could accept a bid or
certificate against a changed candidate-generation or lower-bound context.

**Required focused tests:**

- exact equivalence accepts only all certificate-relevant fields;
- any candidate-generation, map, lower-bound, task, session, or path-context
  change replaces the round;
- newer epoch with exact equivalent evidence cannot mix old and new bid
  provenance;
- concurrent snapshot callbacks cannot invalidate a current asynchronous bid;
- both replicas reach the same round identity and decision hash;
- all certificate mismatch tests remain fail-closed;
- peer loss/session restart and continuation commitment tests remain unchanged.

The existing canonical-hash, snapshot-ledger, certificate-provenance, and
round-generation tests are necessary but not sufficient for this change:
[test_distributed_task_canonicalization.py:90-118](/home/arash/webots_ws_clean_validation_20260823/src/my_epuck_project/test/test_distributed_task_canonicalization.py:90),
[test_distributed_pair_scoring.py:1188-1565](/home/arash/webots_ws_clean_validation_20260823/src/my_epuck_project/test/test_distributed_pair_scoring.py:1188),
and [test_distributed_continuation_allocation.py:306-363](/home/arash/webots_ws_clean_validation_20260823/src/my_epuck_project/test/test_distributed_continuation_allocation.py:306).

### 3. Risky: reuse normal peer bids/decisions or relax snapshot/commitment gates

Examples include accepting a prior bid after a snapshot epoch changes,
allowing a bid without a fresh peer snapshot, keeping a decision across a new
union, or retaining a commitment after session/task/activity evidence no
longer matches.

**Expected benefit:** could remove much of the peer-bid and round-formation
wait.

**Correctness risk:** unacceptable under the frozen contract. These changes
would let distributed replicas score or certify different evidence sets,
permit delayed messages to participate, or reuse a goal after ownership or
navigation state changed. They would require a protocol redesign and are not
authorized by this archaeology task.

**Required tests:** full cross-replica message-order, delayed-message,
session-restart, peer-loss, certificate-provenance, traffic, commitment, and
dispatch-safety suites, plus new distributed integration evidence. This is not
a minimal allocator fix and should not be pursued.

## Existing test coverage and remaining gaps

| Responsibility | Existing coverage |
|---|---|
| Snapshot epoch/session/revision/TTL | `test_distributed_protocol.py` covers duplicate/older epochs, revision regression, session restart, retired sessions, and receiver-local TTL. |
| Bid bindings | `test_bid_must_bind_round_union_identity_epoch_and_fresh_peer()` covers round, union, identity, epoch, freshness, and expiry. |
| Round generation/race guards | `test_distributed_frontier_round_guard.py` covers asynchronous round replacement, stale callbacks, common release, fallback safety, and planner lease retry. |
| Canonical union/round identity | `test_distributed_task_canonicalization.py` covers arrival-order independence and strict robot ordering. |
| Continuation/commitments | `test_distributed_continuation_allocation.py` covers fresh free snapshots, busy commitment reuse, session changes, task identity omission, goal termination, traffic hold, and the normal two-bid requirement. |
| Certificates | `test_distributed_pair_scoring.py` covers competitive unqueried options, dominated options, missing bounds, context mismatch, candidate-generation mismatch, source-local evidence, and continuation bounds. |
| Traffic | `test_traffic_scheduler.py` and pair/continuation scoring tests cover continuous path conflict, deterministic priority, active-goal behavior, and waiting-state semantics. |

There is no existing focused test for a cross-round successful path cache, so
candidate 1 needs a narrow new test set. There is also no test proving that an
in-progress normal round may retain its exact bid/certificate inputs across a
newer epoch with unchanged full provenance; that gap is why candidate 2 is not
the first change.

## Recommended checkpoint

If implementation is authorized later, implement candidate 1 as one isolated
commit, run only its focused tests plus the existing distributed round/protocol
tests, and compare planner-lease wait and round-formation telemetry. Do not
change snapshot TTLs, bid validity, canonical round identity, certificate
definitions, commitment invalidation, traffic rules, or the continuation
contract in that checkpoint.

No fix was implemented in this task. No tests, build, Webots launch, or ROS
launch were run.

**MINIMAL_SAFE_ALLOCATOR_FIX_IDENTIFIED**
