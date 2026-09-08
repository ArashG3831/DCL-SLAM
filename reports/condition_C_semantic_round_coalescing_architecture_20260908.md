# Condition-C semantic round-coalescing architecture archaeology

Date: 2026-09-08
Analysis commit: `a294b92`
Scope: read-only allocator/protocol/certificate archaeology. No source, test,
build, ROS, Webots, or experiment changes were made.

## Final conclusion

**MINIMAL_SAFE_SEMANTIC_ROUND_COALESCING_NOT_PROVEN**

Semantic coalescing is technically possible only in a very narrow sense: an
update may be coalesced when it is proven to be an exact retransmission or when
the state is already represented by an immutable active commitment. Those two
cases already exist in the current implementation. A new coalescing rule that
keeps an actionable normal round alive across a newer source snapshot is not
currently proven safe.

The reason is not the canonical task ID alone. Normal bid, peer-bid, and pair
decision protocol objects are bound to source session/epoch, the canonical
round ID, the union hash, and fresh receiver-local evidence; the local path and
cost-only certificate additionally depend on current task, map/costmap, and
lower-bound provenance. Reusing the old round across a changed epoch would
require a new exact evidence-equivalence contract and a deterministic
authority rule for the newer snapshot. The current rounded semantic fingerprint
does not establish that contract.

The current architecture therefore treats most normal-round churn as a
correctness boundary. The measured opportunity for a future improvement is
limited to genuinely evidence-identical updates; the preserved 600-second
cache run does not prove that this subset is large enough to remove a material
fraction of the remaining `114.26 s` allocator-coordination crosswalk.

## Evidence and references

Primary runtime artifact:

`results/thesis_condition_C_600s_20260908T005703Z/fast_trial_20260908T005708Z/observer/fast_trial_20260908T005708Z/`

The run was the validated exact local path-cache run. The earlier allocator
architecture report and post-cache archaeology are preserved in:

- [allocator round-formation archaeology](allocator_round_formation_fix_architecture_20260908.md)
- [post-cache coordination archaeology](condition_C_post_cache_allocator_coordination_archaeology_20260908.md)

Relevant implementation:

- [`distributed_frontier_assignment.py`](../src/my_epuck_project/my_epuck_project/distributed_frontier_assignment.py)
- [`canonical.py`](../src/my_epuck_project/my_epuck_project/distributed_assignment/canonical.py)
- [`protocol.py`](../src/my_epuck_project/my_epuck_project/distributed_assignment/protocol.py)
- [`models.py`](../src/my_epuck_project/my_epuck_project/distributed_assignment/models.py)
- [`frontier_proposal_adapter.py`](../src/my_epuck_project/my_epuck_project/frontier_proposal_adapter.py)
- [`round_lifecycle.py`](../src/my_epuck_project/my_epuck_project/round_lifecycle.py)

Relevant focused tests:

- [`test_allocator_round_lifecycle.py`](../src/my_epuck_project/test/test_allocator_round_lifecycle.py)
- [`test_distributed_protocol.py`](../src/my_epuck_project/test/test_distributed_protocol.py)
- [`test_distributed_continuation_allocation.py`](../src/my_epuck_project/test/test_distributed_continuation_allocation.py)
- [`test_distributed_frontier_round_guard.py`](../src/my_epuck_project/test/test_distributed_frontier_round_guard.py)
- certificate tests in [`test_distributed_pair_scoring.py`](../src/my_epuck_project/test/test_distributed_pair_scoring.py)

## Current mechanism

```text
terminal / candidate update
        |
        v
source TaskSnapshot + candidate evidence
        |
        +-- freshness/session/epoch/map/provenance checks
        v
canonical union + source-epoch round ID
        |
        v
local BidBatch <---- exact current-round peer BidBatch
        |
        v
pair decision <---- exact current-round peer decision
        |
        v
cost-only certificate over current bids and current lower-bound evidence
        |
        v
final path / TF / map / costmap / lifecycle / traffic gates
        |
        v
dispatch and immutable active commitment
```

The normal path is implemented in `_tick_impl()`:

1. Both source snapshots must be fresh before normal cooperative work.
2. `canonical_round_id()` hashes both source sessions and snapshot epochs.
3. A different ID creates a new `RoundWork`, clears bid/decision/commitment
   state for that uncommitted round, and enters bidding.
4. Each local bid is produced or safely reused from the path cache; the path
   cache creates a new current-round bid and does not reuse protocol evidence.
5. Both bid batches must match current source session, snapshot epoch, round ID,
   union hash, and receiver-local freshness.
6. Peer agreement must match the same round, union, epochs, bid fingerprints,
   selected task IDs, and decision hash.
7. The cost-only certificate is recomputed from the current decision/batches
   and current lower-bound evidence before dispatch.

This sequence is deliberately stronger than “same physical task.” A physical
task can persist while its available path, map, candidate generation, lower
bound, peer availability, or assignment competitors change.

## 1. Round lifecycle and measured churn

### Creation and replacement

`_activate_round()` obtains a new `RoundGeneration` lease and installs the
`RoundWork`. The generation guard requires object identity, generation, and
active round ID to match before asynchronous work can commit. A replacement
therefore invalidates in-flight work from the old round rather than allowing a
late callback to publish stale bids or decisions.

The normal round ID is:

```text
H(robot1 source session, robot1 snapshot epoch,
  robot2 source session, robot2 snapshot epoch)
```

This is in [`canonical_round_id()`](../src/my_epuck_project/my_epuck_project/distributed_assignment/canonical.py#L36-L43)
and is used by `_tick_impl()` before creating the new `RoundWork`.

For the primary cache-run launch log:

| Lifecycle record | Count | Interpretation |
|---|---:|---|
| First round creations | 51 | A node installed a round while no round was active |
| Round replacements | 112 | 53 robot1, 59 robot2 |
| Total activation records | 163 | First creations plus replacements |
| Replacement reason: canonical | 57 | 25 robot1, 32 robot2 |
| Replacement reason: continuation | 55 | 28 robot1, 27 robot2 |
| Unique agreement rounds | 18 | Offline cooperation summary |
| Dispatches | 26 | Offline cooperation summary |

The launch log and offline summary count different protocol boundaries, so
activation records must not be divided by dispatches to infer a unique-round
rate. They do establish that most activated rounds do not become a dispatched
agreement before they are superseded, cleared, left idle, or deferred by the
certificate.

The same run recorded 87 `new continuation round` state transitions, 76 `new
canonical round` transitions, 52 waits for valid peer bids, 58 waits for valid
free-robot continuation bids, 11 fresh-snapshot waits, 29 continuation
commitment-invalid states, 6 peer commitment changes, 8 local commitment
terminations, and 7 explicit round-invalidated transitions. These are state
telemetry counts, not a substitute for the 112 replacement records.

### Required replacement causes

The following replacement/invalidation causes protect a concrete invariant:

| Cause | Why the old round cannot simply survive |
|---|---|
| New source session | Old session evidence and commitments must not cross a peer restart. |
| New snapshot epoch/provenance | The old bid and certificate describe a different source evidence set. |
| Changed union | The set of competitors and the assignment domain changed. |
| Navigation terminal result | Robot availability, completed-task suppression, and continuation state changed. |
| Path/TF/map/costmap failure | The feasible assignment set changed and failure suppression must affect the next round. |
| Commitment ended or changed | A busy side can no longer be represented by the old immutable commitment. |
| Traffic reservation release | The old deferred assignment is intentionally discarded and a fresh allocation is required. |

The code clears `_bid_batches`, `_peer_decision`, and `_committed` on a normal
replacement/reset. That is necessary because these objects carry the old round
identity, not merely a task ID. `RoundGeneration` separately invalidates late
callbacks.

### Potentially avoidable replacement

An epoch-only callback whose complete content and certificate evidence are
identical could, in principle, avoid rebuilding the old in-progress round.
However, the current `_snapshot_content_fingerprint()` is not an exact
certificate-equivalence predicate. It rounds several fields and intentionally
does not include all provenance used by the certificate, including the source
candidate-generation identity and generation timestamp. It also does not
serve as a proof that the old lower-bound evidence remains authoritative.

The existing code already avoids two narrower classes of unnecessary work:

- an exact immutable snapshot retransmission refreshes its receipt as a
  heartbeat rather than becoming a new epoch;
- unchanged IDLE semantic content is not repeatedly re-planned;
- continuation uses the immutable active commitment on the busy side and only
  requires a fresh free-side proposal.

Those are safe coalescing cases because their authority rules are explicit.
They do not authorize coalescing an actionable normal round with a newer
source snapshot.

## 2. Exact relationship between rounds and certificates

### What the certificate consumes

`_cost_only_dispatch_certificate()` receives:

- the current `RoundWork`;
- the current `PairDecision`;
- both current `BidBatch` objects;
- source-local lower-bound evidence;
- the source snapshots used by that round.

The certificate is evaluated after pair selection and before the decision can
be committed for dispatch. Its diagnostic/event key includes the round ID,
bid cardinalities, DNU count, blocker count, scores, result/reason, and
provenance comparison. It is not a standalone certificate cache keyed only by
the physical task.

For each required source, the production lower-bound gate requires the bound
provenance to match the current snapshot's map revision, lower-bound context
fingerprint, and candidate-generation identity. The conservative branches
return no usable bound when evidence is absent, non-finite, incomplete, or
from a mismatched context. The diagnostic classifier additionally records
generation, timestamp, session, epoch, map, and costmap mismatches.

### Why a successful certificate cannot be rebound by task ID

A certificate for task `T` proves that the selected assignment beats the
unqueried competitors represented by the *current* source evidence and the
*current* evaluated bid batches. If a round changes, any of the following may
also have changed:

- the competing union members;
- the selected pair or pair score;
- the source snapshot epoch/session;
- the candidate generation and lower-bound context;
- the map/costmap used to establish bounds and paths;
- the peer's bid or current active commitment;
- the traffic compatibility of the pair.

Therefore a successful certificate can survive a round replacement only if
all of those bindings are proven equivalent and the certificate contract
explicitly authorizes the rebinding. The current implementation does not have
such a certificate object or rebind operation. It recomputes the certificate
for the current `RoundWork` instead.

The safe state already preserved across replacement is narrower:

1. The exact successful local path result can be reused if the full execution
   key matches and `LocalNav2.path_context_matches()` proves the current map and
   costmap context. `_append_bid()` then creates a new bid for the current
   round.
2. An active navigation commitment can be represented in continuation mode
   using its immutable task/path/session identity, fresh peer status, and a new
   current continuation round. The busy side receives a synthetic current-round
   bid; the old peer bid and old certificate are not reused.

These are computation/state reuse, not certificate or peer-evidence reuse.

## 3. Candidate identity: physical sameness is not evidence sameness

The project uses multiple identities with deliberately different meanings:

| Identity | Meaning | Sufficient for certificate reuse? |
|---|---|---|
| `local_frontier_id` | Source-local transient frontier identifier | No |
| `physical_signature` | Quantized approach/centroid/bounds geometry; excludes transient frontier identity | No |
| `canonical_id` | Quantized canonical union geometry over one or more equivalent members | No |
| source session/epoch | Source-provenance version | Required protocol binding |
| map/costmap/lower-bound/candidate-generation fields | Evidence context | Required certificate binding |
| round ID/union hash | Current decentralized assignment domain | Required bid/decision binding |

`physical_signature()` deliberately treats a geometrically persistent
opportunity as the same physical opportunity across observations. The
`equivalent_tasks()` tolerances can also merge source proposals that are close
or geometrically overlapping. Conversely, a split or merged frontier can
change the canonical union even when one member's physical signature remains
recognizable.

Thus repeated candidate appearances can be:

- the same physical opportunity with a new map/candidate-generation snapshot;
- a changed approach or orientation for that opportunity;
- a split/merged canonical task;
- a stale candidate that has disappeared and later reappeared; or
- a genuinely new physical opportunity.

The current cache-run evidence shows all of these lifecycle pressures without
proving that they are simultaneous duplicates: 3,420 new candidate identities,
5,687 reused identities, 3,353 absent/pruned identities, no frontier-generator
capacity evictions, and maximum candidate-cache sizes of 281 robot1 and 290
robot2. Certificate history had 638 physical IDs in its latest per-ID records;
612 were absent at their latest record, with 617 disappearance and 5
reappearance counts. These history values are not a simultaneous live-set
count, but they prove candidate churn.

The current certificate observations reinforce the distinction:

- 129 complete observations;
- 38 certified and 91 deferred;
- 78 deferred because an unqueried option could still beat the assignment;
- 13 deferred because required lower bounds were missing;
- 13 generation-mismatch evidence observations and 2 no-bound-summary
  observations;
- maximum detected-not-queried count 145 and maximum blocker count 107.

A physical signature or canonical ID alone cannot distinguish the evidence
contexts in those cases.

## 4. Peer coordination and obsolete work

### Fresh snapshots

`Received.fresh()` uses receiver-local monotonic receipt age and the accepted
sender TTL. Normal cooperative work requires both source snapshots to be
fresh. `SnapshotLedger.accept()` rejects duplicate/older epochs, source
identity mismatches, map-revision regressions, and retired sessions. Exact
immutable retransmission is separately treated as a heartbeat.

The cache-run state telemetry recorded 11 direct fresh-snapshot waits. The
existing idle crosswalk assigned 25.76 seconds to fresh-snapshot ownership.
This proves freshness is a measured coordination owner, but not that every
fresh wait invalidated an otherwise usable certificate. The artifact does not
record a counter named “fresh snapshot invalidated a valid old round,” so that
stronger quantity remains unproven.

### Peer bid synchronization

`bid_batch_valid()` requires all of:

- receiver-local batch freshness;
- fresh peer snapshot;
- expected source robot and session;
- exact source snapshot epoch;
- exact current round ID;
- exact union hash; and
- unique canonical task IDs.

The matching peer decision adds current epochs, bid fingerprints, task IDs, and
decision hash. This means a valid bid can become obsolete before dispatch for a
real protocol reason: the round identity or evidence domain changed. The
current artifacts record 52 waits for a valid peer bid and 58 waits for a valid
free-robot continuation bid. The post-cache exact idle crosswalk attributes
29.68 seconds to peer-bid waiting.

The artifact does not contain a direct per-bid “invalidated before dispatch”
field. It does contain 366 replayed bid batches/2,071 bid records, 2,059 valid
records, 18 unique agreement rounds, and 26 dispatches. The difference is not
itself an obsolete-bid count because batches include repeated heartbeats,
IDLE decisions, certificate-deferred rounds, and continuation work. The exact
number of peer batches rendered obsolete before dispatch is therefore not
proven by the preserved schema.

### Is peer coordination unavoidable?

Some coordination delay is fundamental to this decentralized contract. Each
robot must establish that it is solving the same source-versioned assignment
problem before either can use the pair decision. A peer cannot safely infer
that a missing or older bid remains valid merely because the physical task
looks unchanged.

The delay is not all fundamental in the abstract. If an exact, symmetric,
certificate-relevant equivalence proof existed, a new snapshot could be
classified as an evidence-identical heartbeat. The current code and artifacts
do not provide that proof, and the current fingerprint is intentionally weaker
than the certificate provenance check.

## 5. Safety audit of possible coalescing changes

### Safe under the frozen contract

These are safe because they preserve current authoritative evidence and still
construct current protocol objects:

1. **Exact immutable snapshot retransmission handling.** Refresh receipt
   freshness only when the entire snapshot value is equal. This is already
   implemented in `_snapshot_callback()`.
2. **Exact successful local path reuse.** Retain only a finite successful path
   under the full task/provenance key and current path-context match. Create a
   new current-round `Bid`; never reuse the old batch, decision, or certificate.
3. **Continuation commitment reuse.** Reuse only the immutable active task/path
   for the busy side, require fresh status and identity checks, and build a
   current continuation round and free-side bid. This is already implemented
   and tested.
4. **A future exact-evidence predicate**, but only if it proves every field
   consumed by scoring, bid validation, certificate bounds, traffic, and
   dispatch is unchanged, preserves old evidence only while its original
   freshness remains valid, and never rebinding old peer/certificate messages.
   This is a design requirement, not a currently authorized implementation.

### Unsafe

The following would weaken the contract and must not be attempted:

- reuse a peer bid, decision, or certificate by physical/canonical task ID;
- ignore source snapshot epoch or candidate-generation ID;
- replace the round hash with a geometry-only/content-only hash;
- extend snapshot or bid TTL to hide coordination delay;
- accept a newer snapshot while treating older lower bounds as current without
  an exact provenance proof;
- bypass `bid_batch_valid()`, matching-decision checks, certificate checks, or
  final path/TF/map/costmap/traffic gates;
- preserve an active commitment after peer session, task identity,
  navigation, or communication validity changes;
- treat status heartbeat liveness as a substitute for fresh task evidence;
- use the current rounded `_snapshot_content_fingerprint()` as a certificate
  authorization key;
- let a local robot dispatch while the peer's current protocol evidence is
  missing merely because the physical task appears unchanged.

## 6. Smallest possible future boundary, if pursued later

The smallest defensible boundary would be before `_activate_round()` in the
normal `_tick_impl()` path, not in certificate scoring and not in
`bid_batch_valid()`:

1. Compute an exact, versioned equivalence record for both source snapshots.
2. Require exact equality of source session, all task members and execution
   geometry, source map/costmap revisions and fingerprints, candidate
   generation, generation timestamp, lower-bound context and complete bound
   set, canonical union, and any traffic/commitment state consumed by the
   decision.
3. Require the old round's snapshots, batches, peer decision, and lower-bound
   evidence to remain fresh and valid at the receiver.
4. Define which snapshot remains authoritative; never silently mix old bids
   with fields from the new snapshot.
5. Keep the current round/union/bid/decision/certificate bindings unchanged,
   or explicitly reissue every protocol object under a new round. Do not
   rebind an old certificate by task ID.
6. Preserve the generation guard and all final dispatch checks.

If any one of these conditions cannot be proven, the current behavior must
remain: replace the round, clear old protocol state, and rebuild using the
existing freshness and certificate gates.

This boundary would be a new protocol/evidence contract, not a cache-only
optimization. It requires focused tests for source epoch changes, map and
costmap changes, candidate-generation changes, split/merged canonical tasks,
peer bid mismatch, certificate-bound mismatch, freshness expiry, commitment
termination, and deterministic behavior when the two decentralized replicas
observe callbacks in different orders.

## 7. Expected benefit against the measured `114.26 s`

The cache-run post-cache crosswalk is:

| Owner | Exact avoidable-idle crosswalk |
|---|---:|
| Certificate waiting | 71.04 s |
| Allocator coordination (peer + fresh + round + other) | 114.26 s |
| Planner/query lease | 9.24 s |
| Total avoidable idle | 194.54 s |

The 114.26-second coordination total is not all round-rebuild cost. It
contains peer-bid waits, fresh-snapshot waits, continuation/commitment state,
terminal transitions, and other coordination labels. A semantic coalescer
could only remove the subset caused by evidence-identical round replacement.
It could not remove:

- a real missing or stale peer snapshot;
- a genuinely changed candidate-generation/lower-bound context;
- a changed competitor set or union;
- a commitment ending or changing;
- a required current peer bid or matching decision;
- a certificate that is still correctly deferred.

The current run has no direct count of evidence-identical actionable-round
replacements. Therefore no defensible percentage of the `114.26 s` can be
claimed. The likely direct benefit is small or unknown and would overlap with
the already-measured path-cache benefit. The path cache already removes the
safe repeated planner work while retaining new current-round bids. A new
semantic coalescing rule would need to demonstrate a materially larger exact
equivalence population before it could justify protocol changes.

## Answers to the requested questions

### A) Is semantic round coalescing technically possible without weakening correctness?

Only for exact evidence-equivalent updates and immutable commitment/heartbeat
cases. Those safe cases already exist. A general new rule for newer normal
snapshots is not proven possible under the current contract because no exact
equivalence predicate or authority rule exists for rebinding the current round
to the newer source evidence.

### B) If yes, what could be reused and where?

Reusable state is limited to:

- finite successful local path computation under the existing exact key and
  context check;
- immutable active commitment task/path in continuation mode;
- exact immutable snapshot receipt/heartbeat state.

Any future normal-round coalescer would have to live at the round-activation
boundary and prove full snapshot, task, union, map/costmap, lower-bound,
candidate-generation, freshness, peer-state, traffic, and commitment
equivalence. It must generate current protocol objects or keep the old round
authoritative without mixing versions. It may not reuse a certificate merely
because a physical task matches.

### C) If not, why is current round churn necessary?

Under the current implementation, accepted source epoch/session changes define
a new protocol version. Peer bids and decisions are exact-version objects, and
the cost-only certificate is recomputed from current-round assignment and
source-local evidence. Clearing the old round prevents late callbacks and
stale decentralized evidence from being committed. Until a stronger exact
equivalence contract is added and verified, that churn is necessary for the
frozen correctness guarantees.

### D) Could it reduce the remaining `114.26 s`?

It could reduce only an unmeasured, likely limited subset of that total:
round replacement that is fully equivalent in every certificate-relevant
field. It cannot safely remove the measured peer/freshness/commitment waits or
the fail-closed certificate deferrals. The preserved evidence is therefore
insufficient to claim that semantic coalescing would materially reduce the
remaining allocator coordination.

## Final decision

Do not implement semantic round coalescing at this checkpoint. Preserve the
current round identity, freshness, peer-bid validation, certificate evidence
matching, commitment validity, traffic safety, and dispatch gates. The only
currently proven safe reuse boundaries are the existing exact retransmission,
continuation commitment, and path-evaluation cache paths.
