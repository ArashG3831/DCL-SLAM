# Condition-C post-certificate allocator bottleneck archaeology

## Verdict

**ALLOCATOR_COORDINATION_IS_PRIMARY_REMAINING_BOTTLENECK**
This conclusion is about the remaining non-certificate delay in the query-cap-8
artifact. A strict same-robot, same-round join shows that dispatch follows the
matching successful certificate almost immediately: the eight long finite
terminal-to-dispatch joins have certificate-success-to-dispatch delays of
0.12–0.34 s (median 0.22 s, mean 0.25 s). Therefore valid work is not sitting
for tens of seconds after its own certificate succeeds. It is waiting for the
allocator to produce a complete, current, dispatchable round and certificate.

No source, build, or runtime artifact was changed. No Webots/ROS process was
launched.

## Scope and method

Only the query-cap-8 artifact was analyzed:

`results/thesis_condition_C_600s_20260907T234020Z/fast_trial_20260907T234025Z/`

Inputs were the preserved `events.jsonl`, `launch.log`,
`goal_decision_ledger.jsonl`, `cooperation_summary.json`, `avoidable_idle.json`,
`navigation_action_replay.json`, `snappiness.json`, `summary.json`, and
`run_manifest.json`.

The existing scientific avoidable-idle intervals were retained. The owner
breakdown below is an archaeology crosswalk that intersects those exact
intervals with recorded `STATE_TRANSITION` reasons; it is not a new metric.
The terminal-to-dispatch analysis matches the successful certificate,
agreement, and dispatch by robot and `round_id`. This prevents a certificate
for the other robot or an earlier round from being counted as the waiting
robot’s certificate.

## 1. Terminal-to-dispatch decomposition

The table includes every finite terminal-to-dispatch interval longer than 20 s.
`F/U` is the overlap of the exact candidate-feasible and candidate-unavailable
states. Bid times list the first valid local bid observed for each side of the
matching round. A continuation row can legally reuse the peer commitment, so a
same-round peer bid is shown as not required where applicable.

| Robot | Terminal → dispatch | Matching certificate evaluation | First valid bids | Agreement | Dispatch | F/U (s) | Recorded pre-certificate owner / reason |
|---|---:|---|---|---:|---:|---:|---|
| robot1 | 58.50 → 98.62 (40.12) | failed `68.74–98.18`; success 98.40, `NO_UNQUERIED_OPTIONS` | r1 98.30 (3 valid), r2 98.30 (3 valid) | 98.52 | 98.62 | 28.38 / 11.74 | certificate 24.30; peer-bid wait 3.66; local pair 7.02; terminal/round 3.00; canonical/other 1.24 |
| robot2 | 69.52 → 101.96 (32.44) | continuation success 101.74, `NO_UNQUERIED_OPTIONS`; prior failed episode ended at 98.40 | r2 101.62 (2 valid); peer bid reused under active commitment | 101.84 | 101.96 | 24.64 / 7.80 | certificate 22.64; round invalidation 3.00; terminal/commitment 3.12; peer-bid wait 0.76; query lease 0.68 |
| robot2 | 128.98 → 165.86 (36.88) | failed `132.08–160.86`; success 165.74, `ALL_UNQUERIED_OPTIONS_DOMINATED` | r2 165.42 (6 valid), r1 165.64 (6 valid) | 165.74 | 165.86 | 34.68 / 2.20 | certificate 25.36; peer-bid wait 4.88; terminal/commitment 2.90; query lease 1.92 |
| robot1 | 134.52 → 215.86 (81.34 raw; 81.22 authoritative replay) | failed `197.08–213.86`; success 215.42/215.52, `ALL_UNQUERIED_OPTIONS_DOMINATED` | r2 214.74 (6 valid), r1 215.42 (6 valid) | 215.64 | 215.86 | 72.24 / 9.10 | peer-bid wait 27.34; query lease 11.50; certificate 35.34; canonical/round 3.92 |
| robot2 | 194.08 → 215.86 (21.78) | same 215.42/215.52 successful round | r2 214.74 (6 valid), r1 215.42 (6 valid) | 215.64 | 215.86 | 16.68 / 5.10 | certificate 12.66; peer-bid wait 4.24; terminal/commitment 2.78; query/other 0.24 |
| robot1 | 241.98 → 263.88 (21.90) | failed 244.86; success 263.54/263.66, `ALL_UNQUERIED_OPTIONS_DOMINATED` | r2 263.44 (5 valid), r1 263.54 (5 valid) | 263.78 | 263.88 | 6.78 / 15.12 | fresh-snapshot/round gate 11.58; certificate 6.56; terminal/commitment 2.76; query lease 0.44 |
| robot2 | 366.34 → 387.88 (21.54) | successful 387.54/387.66, `ALL_UNQUERIED_OPTIONS_DOMINATED` | r1 387.10 (8 valid), r2 387.34 (8 valid) | 387.78 | 387.88 | 1.54 / 20.00 | fresh-snapshot/round gate 17.66; terminal/commitment 2.44; query lease 1.00 |
| robot1 | 367.34 → 387.98 (20.64) | same 387.54/387.66 successful round | r1 387.10 (8 valid), r2 387.34 (8 valid) | 387.78 | 387.98 | 7.56 / 13.08 | fresh-snapshot/round gate 16.78; query lease 0.44; peer-bid wait 0.44; terminal/commitment 2.22 |

The raw 81.34 s event interval is reported as 81.22 s by the authoritative
`snappiness.json` replay because its terminal boundary is 0.12 s earlier. The
reported p95 is therefore the preserved 81.22 s value.

### What actually waited

The 81.22 s p95 event was robot1’s 134.52 s terminal followed by the 215.86 s
dispatch. There were 72.24 s of exact feasible work in that interval, so lack
of candidate availability is not the explanation. The final matching round had
six valid bids on each side. The long part before certificate success was
dominated by repeated allocator rounds that could not form a current valid
peer-bid pair, plus 11.50 s of local path-query lease wait. Once the matching
certificate and agreement existed, dispatch followed in 0.34 s.

The same distinction matters for the apparent “after certificate” observation:
the 165.74 s successful certificate belongs to the `cd0a7fb7` round and the
robot2 assignment. It is not robot1’s certificate for the later `79147fe2`
dispatch. Counting that global event as robot1’s post-certificate wait would
mix rounds and overstate a post-certificate dispatch gate.

### Gate classification

| Gate | Evidence in this artifact | Classification |
|---|---|---|
| Missing/fresh peer snapshots | Repeated `fresh task snapshots from both source sessions required`; late waits show snapshot ages above the freshness window. | Confirmed upstream allocator gate |
| Missing valid peer bids | `waiting for valid peer bids` with `peer_bid_age=None`; local valid bids often existed first. | Dominant allocator gate in the p95 interval |
| Stale bids | Stale snapshot ages occur in earlier failed rounds, but no separate stale-bid expiration field is recorded. | Contributing possibility, not independently quantified |
| Round synchronization | Repeated `new canonical round`; the p95 interval contains 34 distinct non-empty recorded round IDs across both robots. | Confirmed round churn/gate |
| Commitment rules | `continuation commitment no longer valid`, `peer active commitment ended or changed`, and two `ROUND_INVALIDATED` records: active navigation and final ComputePathToPose failure. | Confirmed secondary gate |
| Duplicate/canonicalization | Pair diagnostics include normal equivalence rejections, but no long wait is labeled duplicate or canonicalization failure. | Not shown as a delay owner |
| Traffic | No `WAITING_FOR_TRAFFIC`, `TRAFFIC_CONFLICT_CLEARED`, or degraded-solo event in the run. | Not the cause |
| Other allocator gate | Complete pair publication and agreement occur only after the current snapshots/bids/commitments are valid. | Confirmed, but not separately attributable beyond the recorded reasons |

Thus the valid candidates were not simply ignored after certification. A valid
candidate’s local path result was insufficient to dispatch: the distributed
allocator still required the current peer snapshot, the matching peer bid or a
valid continuation commitment, a complete pair decision, and a certificate for
that same round.

## 2. Candidate backlog after certificate

The preserved frontier lifecycle and candidate records show a persistent
competitive backlog rather than a one-time five/eight-query burst:

| Recorded quantity | Query-cap-8 value |
|---|---:|
| Candidate-batch rows | 87 |
| Candidate records in batches | 1,708 |
| Detected-not-queried row-observations | 4,648; mean 53.43, maximum 252 |
| Maximum frontier snapshot | 263 |
| Maximum cache before pruning | 352 |
| Maximum cache after pruning | 256 |
| Capacity evictions | 84 |
| Tier-1 selected / sent | 1,156 / 710 |
| Frontier lifecycle cycles | 154 |
| Certificate publications / failed / successful | 99 / 77 / 22 |
| Maximum blocker `certificate_rounds_waiting` | 13 |

The lifecycle log shows that the cap was being exercised, not bypassed. In the
late tail, DNU counts were 252, 244, 236, 228, 220, 212, 204, 196, 188, and
180 across successive cycles; most cycles selected and sent eight candidates.
The backlog therefore remained while queries were being processed. This is a
row-observation count, not a claim that 4,648 unique candidates were all live
simultaneously.

The certificate payloads explain why this matters. Of 77 failed publications,
68 were `UNQUERIED_OPTION_CAN_BEAT_EVALUATED_ASSIGNMENT`, 12 were
`ALL_UNQUERIED_OPTIONS_DOMINATED`, 10 were `NO_UNQUERIED_OPTIONS`, and 9 were
`MISSING_UNQUERIED_LOWER_BOUNDS` (publication counts can include paired robot
observations). The direct blocker diagnostics show candidates persisting,
disappearing, and reappearing across generations; they do not show a relaxed
certificate. The late open certificate episode from 463.10 s through 536.08 s
also never produced a successful dispatch before the horizon.

For the p95 terminal gap specifically, the exact idle crosswalk reports
72.24 s of feasible work and only 9.10 s unavailable. The final valid bid
arrays prove that the allocator eventually had valid work; the delay was the
time needed to obtain a mutually current, certificate-valid round, not absence
of all work.

The distinct-round audit count between terminal and dispatch was 14, 13, 15,
34, 11, 3, 2, and 2 for the eight rows above. This count includes mirrored
robot/continuation rounds and is diagnostic rather than a project metric. It
shows the p95 round churn directly; it does not mean 34 assignments were
eligible for dispatch.

No artifact proves that cache eviction alone caused a particular dispatch
delay, and the action replay lacks a lossless candidate-ID join for every path
request. Cache churn and repeated candidate generations are therefore
identified as plausible contributors to the allocator backlog, not as a proven
independent causal duration.

## 3. ComputePathToPose lease delay

The authoritative navigation replay contains:

- 1,127 `ComputePathToPose` executing requests;
- 1,078 successful and 49 aborted path outcomes;
- 1,078 reconstructed plan records;
- 600 robot1 and 478 robot2 plan-topic records;
- no missing required action-status topics.

The recorded `STATE_TRANSITION` reason `waiting for local
ComputePathToPose query lease` is present in the long gaps. Its largest
single overlap is 11.50 s in the 81.22 s p95 row. The other long-row overlaps
are 0.10, 0.68, 1.92, 0.24, 0.44, 1.00, and 0.44 s. In the exact avoidable
idle crosswalk, planner/query ownership totals 7.78 s for robot1 and 5.02 s
for robot2.

Both robots are recorded waiting for the local lease in some rounds, including
the 263.22 s and 386.66 s round starts. That proves lease wait is a real
allocator input delay, but the preserved artifact does not record lease
acquire/release timestamps, owner identity, queue depth, or concurrent lease
count. It cannot prove whether one robot blocked itself, whether the two
robots contended for one shared resource, or the exact lease ownership
duration.

The available evidence does not make planner throughput the primary limiter:
the largest p95 planner wait (11.50 s) is smaller than the peer-bid wait
(27.34 s), successful path outcomes dominate aborted outcomes, and
agreement-to-dispatch p95 is only 0.34 s. Planner/query lease is a secondary
contributor that prolongs round formation, not the primary post-certificate
dispatch gate.

## 4. Idle impact

The exact artifact values are 133.94 s avoidable idle for robot1 and 101.40 s
for robot2. Intersecting those exact idle segments with recorded state reasons
gives this crosswalk:

| Robot | Avoidable idle | Allocator/coordination | Certificate | Planner/query | Navigation/terminal |
|---|---:|---:|---:|---:|---:|
| robot1 | 133.94 s (54.0691%) | 67.12 s | 56.60 s | 7.78 s | 2.44 s |
| robot2 | 101.40 s (50.0840%) | 13.12 s | 78.22 s | 5.02 s | 5.04 s |
| Combined | 235.34 s | 80.24 s (34.1%) | 134.82 s (57.3%) | 12.80 s (5.4%) | 7.48 s (3.2%) |

Certificate state remains the largest total owner of exact avoidable idle,
especially for robot2. That does not contradict the verdict: this task is
isolating the *remaining post-certificate/round-formation bottleneck*. Among
the non-certificate remainder, allocator/coordination is substantially larger
than planner/query (80.24 s versus 12.80 s), and it owns the p95 tail.

Robot1’s long p95 allocator/peer-bid episode explains why its idle did not fall
in proportion to certificate improvement. Robot2 still has a larger residual
certificate crosswalk, so its idle cannot be explained by allocator delay
alone. The robots therefore have different mixes, but the common long-tail
owner after certificate work is reduced is allocator round formation.

## Final finding

The query-cap-8 artifact does not support a claim that dispatch is blocked for
long periods after a successful same-round certificate. It supports the more
precise finding that:

1. valid local work exists;
2. the allocator repeatedly waits for fresh peer snapshots, valid peer bids,
   query-lease availability, and valid commitment/round state;
3. a successful certificate is emitted only for the completed current round;
4. agreement and dispatch then follow within sub-second time.

The primary remaining non-certificate bottleneck is therefore:

**ALLOCATOR_COORDINATION_IS_PRIMARY_REMAINING_BOTTLENECK**
