# Condition-C post-cache allocator-coordination archaeology

Date: 2026-09-08

Current validated commit: `f9aef48` (report commit; runtime under analysis:
`9587d8f`)

Primary artifact:

`/home/arash/webots_ws_clean_validation_20260823/results/thesis_condition_C_600s_20260908T005703Z/fast_trial_20260908T005708Z/observer/fast_trial_20260908T005708Z/`

Comparison artifact:

`/home/arash/webots_ws_clean_validation_20260823/results/thesis_condition_C_600s_20260907T234020Z/fast_trial_20260907T234025Z/observer/fast_trial_20260907T234025Z/`

No source, test, build, ROS, Webots, or runtime artifact was changed or run for
this archaeology pass. The existing exact avoidable-idle definition and
existing offline evaluator outputs were retained.

## Verdict

**ALLOCATOR_COORDINATION_IS_PRIMARY_REMAINING_BOTTLENECK**

At the high-level owner grouping, allocator coordination is the largest
remaining avoidable-idle owner after the path cache:

- allocator coordination: `114.26 s` (`58.73%` of `194.54 s` exact avoidable
  idle);
- certificate waiting: `71.04 s` (`36.52%`);
- planner/query lease: `9.24 s` (`4.75%`).

Certificate waiting remains the largest single atomic label, but peer-bid,
fresh-snapshot, round, commitment, and other allocator coordination combined
are larger. The cache run therefore exposed, rather than removed, a
peer/round-formation bottleneck. Candidate backlog amplifies this bottleneck,
but the artifact does not prove that raw candidate count alone is its cause.

## 1. Scope and method

The primary evidence is the cache validation's `events.jsonl`,
`cooperation_summary.json`, `avoidable_idle.json`, `snappiness.json`,
`navigation_action_replay.json`, and launch log. The comparison uses the
preserved `6f820e8` validation and the earlier certificate/allocator archaeology
reports.

The exact avoidable-idle intervals are the existing evaluator segments with:

```text
classification = FEASIBLE_WORK_AVAILABLE
goal_active = false
```

Those intervals were intersected with the latest recorded allocator
`STATE_TRANSITION` reason until the next transition. This is a diagnostic
crosswalk, not a replacement metric. `WORK_UNAVAILABLE` intervals are not
reclassified as avoidable idle.

Reason codes used below:

| Code | Recorded state-reason family |
|---|---|
| `CERT` | cost-only dispatch certificate deferred |
| `PEER` | waiting for valid peer bids or free-robot continuation bid |
| `FRESH` | fresh task snapshots from both source sessions required |
| `ROUND` | new canonical or continuation round |
| `QUERY` | waiting for local `ComputePathToPose` query lease |
| `COMMIT` | continuation commitment no longer valid or peer commitment changed |
| `TERM` | navigation terminal-result state persisted during the interval |
| `PAIR` | local pair-decision publication/decision transition |
| `TF` | shared-frame TF was not yet usable |
| `OTHER` | another recorded allocator/transition state |

The `TERM` code does not mean that navigation was still executing; it means the
allocator's recorded state reason remained the terminal transition while the
idle segment was being closed.

## 2. Current versus pre-cache ownership

| Owner crosswalk | `6f820e8` baseline | `9587d8f` cache run | Change |
|---|---:|---:|---:|
| Exact avoidable idle | 235.34 s | 194.54 s | −40.80 s |
| Certificate | 134.82 s | 71.04 s | −63.78 s |
| Allocator/coordination | 80.24 s | 114.26 s | +34.02 s |
| Planner/query lease | 12.80 s | 9.24 s | −3.56 s |
| Navigation/terminal crosswalk | 7.48 s | included in coordination crosswalk | not independently comparable |

The baseline's allocator/coordination total is the preserved earlier crosswalk
(`67.12 s` for robot1 plus `13.12 s` for robot2). The current allocator total
is the sum of peer bids, fresh snapshots, round formation, and other recorded
coordination states. The increase is consistent with the current run's higher
round-replacement count and does not show that the cache weakened any protocol
rule.

Current robot-level ownership remains:

| Robot | Exact avoidable idle | Certificate | Peer bid | Fresh snapshot | Round | Other coordination | Query lease |
|---|---:|---:|---:|---:|---:|---:|---:|
| robot1 | 91.72 s | 43.40 s | 10.68 s | 10.68 s | 5.74 s | 21.22 s | 0.00 s |
| robot2 | 102.82 s | 27.64 s | 19.00 s | 15.08 s | 0.00 s | 31.86 s | 9.24 s |
| **Total** | **194.54 s** | **71.04 s** | **29.68 s** | **25.76 s** | **5.74 s** | **53.08 s** | **9.24 s** |

The current robot-level idle values are `27.3660%` for robot1 and `30.6596%`
for robot2. Robot1's longest interval is `172.86–183.64 s` (`10.78 s`);
robot2's longest intervals are `216.98–227.86 s` and `256.78–267.66 s`
(`10.88 s` each).

## 3. Certificate-side evidence

The current run contains 129 complete certificate observations:

| Certificate result | Count | Meaning in the preserved contract |
|---|---:|---|
| Deferred: `UNQUERIED_OPTION_CAN_BEAT_EVALUATED_ASSIGNMENT` | 78 | an unqueried candidate could still beat the evaluated assignment |
| Deferred: `MISSING_UNQUERIED_LOWER_BOUNDS` | 13 | required lower-bound evidence was absent |
| Certified: `ALL_UNQUERIED_OPTIONS_DOMINATED` | 34 | all relevant unqueried options were proven dominated |
| Certified: `NO_UNQUERIED_OPTIONS` | 4 | no unqueried option remained |
| **Total** | **129** | payload complete |

Thus 91 observations were non-certifying/deferred and 38 were certifying. The
deferred state-transition records are exactly 78 plus 13, matching the two
deferred reasons above.

Certificate evidence diagnostics recorded:

- evidence reason `OK`: 114 observations;
- candidate-generation mismatch: 13 observations;
- no candidate-bound summary: 2 observations;
- maximum observed detected-not-queried count: 145;
- median detected-not-queried count: 20;
- maximum blocker count: 107;
- maximum `certificate_rounds_waiting`: 13.

These are not evidence of an over-strict definition. They show the fail-closed
contract refusing to certify while current lower-bound/provenance evidence is
incomplete or while an unqueried option remains potentially competitive.

Time attribution depends on the join being measured:

- certificate overlap inside exact avoidable-idle intervals: `71.04 s`;
- certificate overlap across finite terminal-to-next-dispatch joins:
  `189.74 s`;
- current authoritative terminal-to-dispatch p95: `63.68 s`, with
  approximately `47.40 s` certificate overlap.

These scopes overlap neither as simple totals nor as a new metric. The finite
join value includes work that was not necessarily classified as exact idle,
while the avoidable-idle value includes only feasible work with no active goal.

The certificate delay is therefore driven by candidate/evidence churn and
missing current bounds, not by missing peer status alone. Peer synchronization
then delays formation of the current round in which a valid certificate can be
used.

## 4. Allocator coordination and round formation

### Recorded state-reason counts

| State reason | Count |
|---|---:|
| New continuation round | 87 |
| New canonical round | 76 |
| Waiting for valid free-robot continuation bid | 58 |
| Waiting for valid peer bids | 52 |
| Waiting for local `ComputePathToPose` query lease | 56 |
| Fresh snapshots required | 11 |
| Continuation commitment no longer valid | 29 |
| Peer active commitment ended or changed | 6 |
| Local navigation commitment terminated | 8 |
| Round invalidated | 7 |
| Local complete pair decision published | 38 |
| Required shared-frame TF not usable | 2 |

The launch log records 112 `ALLOCATOR_ROUND_REPLACED` records:

| Robot | Total replacements | Canonical | Continuation |
|---|---:|---:|---:|
| robot1 | 53 | 25 | 28 |
| robot2 | 59 | 32 | 27 |
| **Total** | **112** | **57** | **55** |

The `6f820e8` artifact recorded 98 replacements: 72 canonical and 26
continuation. The cache run therefore had 14 more replacement records, with a
large shift toward continuation-round replacement. This is consistent with
more completed dispatch/terminal cycles and does not by itself prove an
unnecessary round reset.

The seven explicit invalidations were caused by real local dispatch-state
changes:

- three `another local navigation goal is active` records;
- four `local NavigateToPose send precondition changed` records.

They are not evidence that the cache invalidated a valid certificate.

### Terminal-to-dispatch long-tail decomposition

The existing authoritative p95 is robot2's `83.84–147.42 s` interval. The
event crosswalk is:

| Interval | Gap | Certificate | Peer | Fresh | Round | Query | Other |
|---|---:|---:|---:|---:|---:|---:|---:|
| robot2 83.84–147.42 | 63.58 s | 48.50 | 7.86 | 0.34 | 1.94 | 1.92 | 3.02 |

The current maximum raw event gap is robot1 `486.66–552.96 s` (`66.30 s`),
dominated by `38.68 s` certificate and `23.54 s` fresh-snapshot overlap. Other
long finite gaps are:

| Robot interval | Gap | Main recorded components |
|---|---:|---|
| robot1 110.30–147.52 | 37.22 s | certificate 24.72, peer 6.52 |
| robot1 357.44–376.10 | 18.66 s | certificate 15.00 |
| robot1 437.54–463.22 | 25.68 s | certificate 13.04, peer 4.22, query 4.46 |
| robot2 189.98–199.98 | 10.00 s | certificate 6.20 |
| robot2 259.44–273.54 | 14.10 s | certificate 8.12, peer 1.44 |
| robot2 325.44–357.44 | 32.00 s | certificate 21.42, fresh 7.56 |
| robot2 416.10–463.34 | 47.24 s | peer 17.14, certificate 12.94, query 6.58 |
| robot2 539.32–553.08 | 13.76 s | fresh 10.00 |

Once the allocator has a matching current agreement and certificate, the
existing agreement-to-goal handoff remains sub-second (`p50 0.12 s`, `p95
0.66 s`). The long tail is consequently upstream of final dispatch, not a
post-certificate dispatch-safety delay.

## 5. Planner/query contribution

The cache was active and produced:

- 17 lookup hits;
- 371 misses;
- 55 context invalidations;
- 59 deterministic evictions;
- 388 total lookups;
- 2 exact `ALLOCATOR_BID.REUSED` planner-result attributions.

The hit rate was therefore only `4.38%` of cache lookups, and exact reused
planner-result attribution was lower still. The current exact idle crosswalk
assigns `9.24 s` to query-lease waiting, versus `12.80 s` in the preserved
baseline crosswalk. The current p95 terminal gap assigns `1.92 s` to query
lease, while the prior p95 assigned `11.50 s`.

This is a measured reduction, but not a primary remaining cause. The 56
query-lease state transitions are mostly short retry/serialization waits; the
largest current query overlap in the listed long gaps is `6.58 s` for robot2's
`416.10–463.34 s` interval. The artifact does not show an independent planner
lease ownership timeline, so it cannot prove that one robot blocked itself or
that planner throughput, rather than allocator scheduling around the lease,
was responsible for each wait.

## 6. Candidate lifecycle and backlog

The current cooperation and frontier artifacts record:

| Candidate/lifecycle quantity | Current cache run |
|---|---:|
| Candidate batches | 136 (`67` robot1, `69` robot2) |
| Bid batches / bid records | 366 / 2071 |
| Valid bid records | 2059 |
| Frontier lifecycle rows | 223 (`111` robot1, `112` robot2) |
| Tier-1 candidates selected | 1610 |
| Tier-1 candidates submitted | 1057 |
| Tier-1 reachable / unreachable / aborted | 940 / 30 / 1 |
| Tier-1 timeout | 0 |
| New candidate identities in lifecycle rows | 3420 |
| Reused candidate identities in lifecycle rows | 5687 |
| Candidates pruned as absent | 3353 |
| Frontier-generator capacity evictions | 0 |
| Maximum candidate cache before pruning | 290 (robot2) |
| Maximum candidate snapshot after pruning | 166 (robot2) |

The query cap was reached frequently: selected candidates averaged `7.22` per
lifecycle row and never exceeded eight; submitted candidates averaged `4.74`
per row. The backlog varied over time:

| Window | robot1 max DNU / max cache-before | robot2 max DNU / max cache-before |
|---|---:|---:|
| 0–180 s | 61 / 281 | 101 / 290 |
| 180–360 s | 33 / 125 | 43 / 89 |
| 360–600 s | 54 / 132 | 35 / 152 |

At the end of the lifecycle log both robots still reported DNU `13`, although
the maximum certificate-payload DNU was `145`. The two values come from
different streams: lifecycle snapshots and certificate observations.

Certificate-history diagnostics add evidence of churn rather than a static
backlog. They contain 638 unique physical IDs in the latest per-ID diagnostic
records; 612 had `last_present=false`, 617 disappearance counts were recorded,
and 5 reappearance counts were recorded. The current artifact therefore shows
many candidates appearing, disappearing, and being reintroduced while the
allocator is waiting for a current evidence set. It does not show that all
638 were simultaneously live or that all were independently blocking.

The backlog is a real upstream load on certificate and round formation. It is
not sufficient evidence for the stronger claim that "too many candidates" is
the sole cause: the current p95 has both certificate and peer/fresh-snapshot
components, and many short intervals remain after DNU falls.

## 7. Exact avoidable-idle interval reason trace

The following table lists every current exact avoidable-idle interval. The
reason trace is the recorded state-reason sequence intersected with that
interval; durations are shown to make the attribution auditable.

### robot1

| Interval (sim s) | Duration | Recorded reason trace |
|---|---:|---|
| 74.30–79.62 | 5.32 s | `COMMIT 3.44; ROUND 0.44; TERM 1.44` |
| 118.40–123.62 | 5.22 s | `ROUND 0.22; QUERY 0.22; PEER 4.78` |
| 123.96–134.64 | 10.68 s | `PEER 0.10; ROUND 0.12; QUERY 0.22; PEER 0.12; CERT 0.10; ROUND 0.12; PEER 0.10; CERT 3.46; ROUND 0.12; PEER 0.10; CERT 0.90; ROUND 0.10; PEER 0.12; CERT 4.34; ROUND 0.10; QUERY 0.12; CERT 0.44` |
| 135.20–140.20 | 5.00 s | `CERT 0.10; ROUND 0.12; PEER 0.10; CERT 4.56; ROUND 0.12` |
| 141.08–146.20 | 5.12 s | `CERT 0.12; ROUND 0.10; PEER 0.12; CERT 4.44; ROUND 0.12; PEER 0.22` |
| 172.86–183.64 | 10.78 s | `COMMIT 9.78; TERM 1.00` |
| 212.52–217.64 | 5.12 s | `COMMIT 0.22; ROUND 2.12; PAIR 0.12; TERM 2.66` |
| 262.98–273.54 | 10.56 s | `CERT 4.00; ROUND 0.12; TERM 2.68; ROUND 0.20; QUERY 0.12; PEER 0.68; CERT 2.00; ROUND 0.20; QUERY 0.12; PEER 0.34; PAIR 0.10` |
| 353.88–359.66 | 5.78 s | `CERT 0.10; ROUND 0.12; CERT 3.00; ROUND 0.12; PAIR 0.12; OTHER 0.20; TERM 2.12` |
| 440.66–446.66 | 6.00 s | `CERT 0.12; ROUND 0.10; PEER 0.22; CERT 2.24; ROUND 0.32; QUERY 0.32; CERT 2.68` |
| 500.44–505.66 | 5.22 s | `FRESH 0.10; ROUND 0.12; CERT 5.00` |
| 513.44–518.68 | 5.24 s | `CERT 0.12; ROUND 0.22; CERT 4.90` |
| 526.98–532.68 | 5.70 s | `CERT 0.22; ROUND 0.12; CERT 5.36` |
| 592.20–597.66 | 5.46 s | `FRESH 3.36; ROUND 0.20; QUERY 0.56; PEER 0.34; CERT 1.00` |
| 600.44–600.96 | 0.52 s | `ROUND 0.12; QUERY 0.20; PEER 0.20` |

### robot2

| Interval (sim s) | Duration | Recorded reason trace |
|---|---:|---|
| 84.52–94.62 | 10.10 s | `TERM 1.78; ROUND 0.10; CERT 3.22; ROUND 0.12; CERT 4.88` |
| 95.18–100.62 | 5.44 s | `CERT 0.12; ROUND 0.10; CERT 5.22` |
| 101.30–106.62 | 5.32 s | `CERT 0.10; ROUND 0.12; CERT 5.10` |
| 112.84–122.62 | 9.78 s | `PEER 0.12; ROUND 0.10; QUERY 0.34; PEER 0.22; CERT 3.68; ROUND 0.10; QUERY 0.44; CERT 0.34; FRESH 0.34; ROUND 0.10; QUERY 0.22; PEER 3.78` |
| 124.40–133.64 | 9.24 s | `QUERY 0.12; ROUND 0.10; PEER 0.22; CERT 3.46; ROUND 0.22; CERT 1.00; ROUND 0.22; CERT 3.90` |
| 133.86–139.64 | 5.78 s | `CERT 0.12; ROUND 0.22; PEER 1.10; ROUND 0.12; PEER 0.10; CERT 4.12` |
| 139.98–145.64 | 5.66 s | `CERT 0.10; ROUND 0.12; PEER 0.10; CERT 0.90; ROUND 0.10; PEER 0.12; CERT 4.22` |
| 216.98–227.86 | 10.88 s | `TERM 2.10; ROUND 0.12; QUERY 0.32; PEER 0.22; PAIR 0.12; OTHER 0.12; ROUND 0.76; COMMIT 3.90; TERM 3.22` |
| 256.78–267.66 | 10.88 s | `COMMIT 2.66; TERM 2.66; ROUND 0.12; CERT 5.44` |
| 335.10–344.66 | 9.56 s | `FRESH 0.12; ROUND 0.12; CERT 4.32; ROUND 0.12; CERT 4.88` |
| 348.22–353.66 | 5.44 s | `CERT 0.12; ROUND 0.10; CERT 5.22` |
| 422.44–431.66 | 9.22 s | `PEER 0.10; ROUND 0.24; QUERY 0.44; PEER 3.32; FRESH 5.12` |
| 595.44–600.96 | 5.52 s | `FRESH 0.12; ROUND 0.30; QUERY 0.70; CERT 2.88; ROUND 0.32; QUERY 0.68; PEER 0.12; ROUND 0.20; QUERY 0.20` |

The final `600.44–600.96 s` and `595.44–600.96 s` tails are present in the
existing evaluator artifact, whose `end_sim_s` is `600.96 s`; they are retained
here for traceability and are not evidence of a new mission horizon.

## 8. Answers to the required questions

### A) Largest remaining cause of avoidable idle

The largest high-level cause is **allocator coordination** at `114.26 s`,
made up of peer-bid synchronization, fresh snapshots, round formation, and
other coordination/commitment states. Certificate waiting is the largest
single atomic cause at `71.04 s`. Planner/query lease is not primary.

### B) What is causing the delay?

The evidence supports this ordering:

1. **Peer coordination and round synchronization are the primary remaining
   non-certificate bottleneck.** The allocator repeatedly waits for a current
   peer bid, free-robot continuation bid, fresh snapshots, and a matching
   decision bound to the current round.
2. **Candidate backlog/churn is an upstream amplifier.** The query cap is
   frequently reached, DNU counts and candidate caches become large, and many
   candidates disappear/reappear. This keeps certification and current-round
   formation active.
3. **Certificate evidence remains substantial but is fail-closed and
   justified.** The 78 unqueried-competitive and 13 missing-bound deferrals
   are not evidence that the certificate requirement is incorrectly strict.
4. **Planner lease contention is secondary.** The cache reduced its measured
   overlap, and only a small portion of current idle is attributed to it.

Therefore the remaining delay is not proven to be caused by "too many
candidates" alone, nor by a certificate-definition defect. It is best
classified as **peer coordination and excessive current-round synchronization,
amplified by candidate/evidence churn**.

### C) Minimal safe future improvement

This pass does not authorize or implement a fix. The only defensible future
direction identified without relaxing any listed safety rule is an exact,
certificate-proven semantic-coalescing optimization for an in-progress round:
it could avoid replacement only when the complete certificate-relevant source
session, epoch/provenance, candidate generation, map/costmap context, task
content, and lower-bound evidence are proven identical, while retaining the
current authoritative snapshot and rebinding any new decision to the current
protocol context.

The current rounded semantic fingerprint is not enough to prove that property,
and the artifact does not establish that a particular round replacement was
unnecessary. This remains a design candidate requiring source-level proof and
focused protocol/certificate tests, not an approved change. The existing exact
path cache is the only change validated so far.

### D) Unsafe fixes that must not be attempted

- increasing snapshot/bid TTLs or ignoring stale snapshot epochs;
- reusing peer bids across a changed session, snapshot epoch, union hash, or
  canonical round;
- suppressing round replacement based only on rounded task geometry or task
  identity;
- treating missing lower bounds or missing candidate provenance as zero,
  dominated, or `NOT_INVOKED` evidence;
- dispatching from a local bid without the current matching peer decision and
  certificate;
- weakening commitment invalidation after peer status, task identity, session,
  navigation, or send-precondition changes;
- bypassing traffic, final path/TF/costmap/lifecycle checks, or planner lease
  serialization to hide coordination idle;
- raising the query cap as a substitute for proving exact candidate and
  certificate semantics.

## Limitations

The artifact does not contain a first-class per-second ownership field for
certificate, peer, snapshot, commitment, and planner causes. The reason traces
and owner totals are therefore deterministic crosswalks from existing exact
idle intervals and recorded state transitions. The action replay also does not
provide a lossless physical-candidate-ID join for every planner request.

No claim is made that all certificate-history IDs were simultaneously live or
that every candidate disappearance independently caused a delay. The evidence
does establish the dominant owner grouping and the current long-tail sequence
without relaxing any scientific or safety contract.
