# Condition-C cost-only dispatch certificate latency archaeology

Date: 2026-09-08  
Repository HEAD inspected: `6580f51e831d23b64e011d66d73869bdfbdca0d4`  
Artifact:
`results/thesis_condition_C_600s_20260907T215420Z/fast_trial_20260907T215426Z/`  
Observer run:
`observer/fast_trial_20260907T215426Z-01/`

## Scope and method

This was a bounded, read-only artifact investigation. No source was changed, no
build was run, and no ROS/Webots process was launched. Scientific time below is
the recorded `elapsed_s`/ROS simulation time.

Evidence used:

- `events.jsonl`: candidate batches, task snapshots, bid arrays, state
  transitions, and every `COST_ONLY_DISPATCH_CERTIFICATE` payload;
- `navigation_action_replay.json`: recorded `ComputePathToPose` action and plan
  throughput;
- `cooperation_summary.json`: certificate contract completeness/counts;
- `snappiness.json`: terminal-to-next-dispatch distribution;
- `avoidable_idle.json`: idle-analysis context.

The certificate topic produced 185 publications: 157 non-certifying and 28
certifying. Some global certificate decisions are published by both robots at
the same simulation timestamp. Collapsing those paired publications gives 117
evaluation instants: 99 failed and 18 successful. The earlier count of 193 was
an attribution count inside overlapping per-robot terminal gaps, not the number
of distinct certificate publications.

An unqueried blocker is identified exactly by `(robot_id, physical_id)`. In the
per-certificate ledger below, the event sequence numbers are exact primary keys
into `events.jsonl`; the exhaustive candidate list is the JSON-decoded
`message.blocker_diagnostics`, filtered where `is_blocking=true` or
`certificate_evidence_reason != "OK"`. This preserves an exact pointer to every
candidate without copying thousands of repeated 64-bit IDs into this report.
For example, events 518 and 521 are blocked by exactly
`robot1:6192150531719941946` and `robot1:8712261504026781609`.

## Direct finding

The root cause is a large, frequently changing unqueried-candidate backlog being
retired by bounded serial path-cost evaluation. The certificate is generally
not waiting for consensus, traffic, or a missing certificate algorithm:

1. A candidate generation exposes many detected frontiers, but only a bounded
   subset has exact path-cost evidence.
2. Unqueried candidates receive conservative optimistic lower bounds.
3. At 142 of 157 failed publications, at least one optimistic unqueried option
   can strictly beat the best currently evaluated assignment.
4. Exact `ComputePathToPose` work then drains predominantly five candidates per
   evaluation cycle. Forty-five of 54 within-episode decreases were exactly
   five candidates; those cycles were median 2.24 s, mean 2.68 s, p95 5.12 s.
5. New candidate generations can add candidates faster than that drain, and
   many candidates disappear before ever receiving an exact query.
6. Certification occurs only when every competitive candidate has been queried
   or disappeared, or when its bound becomes dominated.

Seven blocking episodes span 200.26 s from first failed evaluation to the next
successful certificate. This independently accounts for nearly all of the
previously attributed 206.76 s certificate-deferred delay; the small difference
is boundary/state-transition accounting around those episodes.

## Complete evidence lifecycle

| Stage | Recorded evidence | Result |
|---|---|---|
| Candidate generation | 148 `CANDIDATE_BATCH_RECEIVED` records | 9,362 detected-frontier observations; 3,253 evaluated candidates; 2,191 explicitly detected-but-not-queried observations. |
| Candidate metadata | Candidate rows carry frontier ID, geometry, local score, exact path length, heading cost, and path samples | Exact metadata exists for queried candidates. Unqueried candidates are represented in certificate diagnostics by physical ID, generation/revision provenance, and conservative bound; their full geometry is not repeated in the certificate event. |
| Task publication | 148 `DISTRIBUTED_TASK_SNAPSHOT` records | Carries the bounded/evaluated physical tasks and generation/map provenance. |
| Query scheduling | State transitions plus action replay | Query work is active, not dead: 1,006 `ComputePathToPose` requests reached terminal status (641 succeeded, 365 aborted). Recorded plan count is 641. |
| Lower-bound generation | Certificate `provenance_comparison` and `blocker_diagnostics` | Usually current and complete. Only 15 failed publications (12 instants) were freshness mismatches. |
| Certificate evaluation | 185 complete payloads | 142 competitive-bound failures, 15 freshness failures, 20 `NO_UNQUERIED_OPTIONS` successes, and 8 `ALL_UNQUERIED_OPTIONS_DOMINATED` successes. |
| Dispatch | Successful certificate followed by agreement/traffic/goal path | Final post-certificate agreement/dispatch is not the dominant delay in the preceding latency audit. |

## Candidate volume and planner throughput

| Measure | Robot 1 | Robot 2 | Combined |
|---|---:|---:|---:|
| Candidate batches | 77 | 71 | 148 |
| Detected frontier observations | 4,899 | 4,463 | 9,362 |
| Exact evaluated candidates in batches | 416 | 2,837 | 3,253 |
| Detected-not-queried observations | 1,109 | 1,082 | 2,191 |
| Unreachable observations | 3,359 | 530 | 3,889 |
| Aggregate evaluated/detected ratio | 8.49% | 63.57% | 34.75% |
| Mean batch interval | 7.12 s | 7.57 s | — |
| Time attributed to `waiting for local ComputePathToPose query lease` | 15.84 s | 1.24 s | 17.08 robot-s |

The large robot asymmetry is real in the recorded artifact. Robot 1 produced far
fewer exact candidate evaluations and far more unreachable observations. The
artifact proves the asymmetry and the query backlog, but it does not by itself
prove why the lease/query scheduler allocated work asymmetrically.

For failed certificate publications, detected-not-queried count was mean 29.53,
median 25, p95 67, maximum 105. Across all 185 publications (including success),
the corresponding values were mean 27.87, median 22, p95 72, maximum 129.
`blocking_unqueried_candidates` is a count of feasible pair/assignment options,
not physical frontiers: mean 247.54, median 24, p95 1,668, maximum 2,368 on failed
publications. The nonlinear expansion therefore magnifies a moderate physical
backlog into many potentially better pair assignments.

## Blocking episodes

| Episode | Failed interval (s) | Failed evaluation instants | Failed publications | Reasons | Mean/max DNU | Unique physical blockers | Certified at (s) | Resolution of unique blockers by certification |
|---:|---:|---:|---:|---|---:|---:|---:|---|
| 1 | 39.20–50.34 | 6 | 12 | 5 can-beat, 1 missing-bound | 4.00 / 9 | 9 | 51.96 | 8 queried, 1 disappeared |
| 2 | 79.40–115.74 | 21 | 33 | 19 can-beat, 2 missing-bound | 39.38 / 98 | 134 | 119.52 | 62 queried, 72 disappeared |
| 3 | 148.74–172.52 | 13 | 19 | 12 can-beat, 1 missing-bound | 24.38 / 39 | 82 | 172.98 | 25 queried, 54 disappeared, 3 became bound-dominated |
| 4 | 204.08–211.74 | 3 | 4 | 3 can-beat | 73.00 / 105 | 45 | 214.52 | 35 disappeared, 10 became bound-dominated |
| 5 | 299.98–339.22 | 31 | 47 | 26 can-beat, 5 missing-bound | 28.26 / 72 | 72 | 339.78 | 36 queried, 36 disappeared |
| 6 | 384.54–410.98 | 11 | 17 | 9 can-beat, 2 missing-bound | 17.82 / 32 | 32 | 413.54 | 17 queried, 15 disappeared |
| 7 | 499.54–540.56 | 14 | 25 | 13 can-beat, 1 missing-bound | 33.93 / 60 | 60 | 543.44 | 28 queried, 32 disappeared |

Candidate count alone is not sufficient to prevent dispatch. Successful
certificates at 214.52 s and 260.22 s retained respectively 129 and 122
unqueried candidates, because every optimistic bound was already worse than the
evaluated assignment. The bottleneck is specifically the number of *competitive*
unqueried candidates and how quickly exact evidence or disappearance resolves
them.

## Candidate lifetime and churn

There were 429 unique physical candidates that actually blocked at least one
failed evaluation:

- 180 (41.96%) eventually received an exact candidate/path result;
- 249 (58.04%) disappeared without ever being queried;
- none remained unresolved in the final certificate history;
- only 3 candidates reappeared after disappearing.

From first recorded unqueried appearance, blocker-history lifetime was mean
26.12 s, median 9.00 s, p95 106.70 s, maximum 180.56 s. Candidates were carried
through mean 2.86 generations (median 2, p95 8, maximum 15) and prevented mean
4.60 certification rounds (median 2, p95 14, maximum 23).

For the 180 eventually queried candidates, first unqueried appearance to exact
query was mean 32.73 s, median 19.00 s, p95 115.14 s, maximum 134.78 s. Measured
from first *blocking certificate* rather than first appearance, resolution by
query was mean 18.66 s, median 15.00 s, p95 39.00 s, maximum 111.14 s. The 249
never-queried candidates resolved by disappearance after mean 8.95 s, median
4.78 s, p95 31.56 s, maximum 48.48 s from first blocking certificate.

This proves substantial churn. It also shows why work can be scientifically
wasted: most unique blockers vanish before obtaining exact path evidence, while
the remaining competitive candidates wait tens of seconds for serial queries.

## Missing lower-bound evidence

All 15 `MISSING_UNQUERIED_LOWER_BOUNDS` publications were caused by
`TASK_CANDIDATE_GENERATION_MISMATCH`, not by permanently absent bound machinery.
They collapse to 12 evaluation instants. Matching bound provenance returned in
0.20–0.56 s (mean 0.358 s, median 0.35 s), totalling about 4.30 s if the isolated
freshness intervals are summed.

At 339.22 s the DNU count was already zero, but robot 2's task/candidate
fingerprint was from a different generation; the fail-closed gate correctly
waited 0.56 s for matching provenance. This is a genuine generation-freshness
check, but it is not the 206.76 s bottleneck.

## Repeated evaluation

- The 157 failed publications correspond to 99 failed evaluation instants and
  72 distinct round IDs.
- 24 round IDs were evaluated at more than one simulation timestamp; maximum
  was three timestamps for one round.
- Using reason, evaluated/DNU counts, scores, provenance reasons, and exact
  blocker set as the state fingerprint, 99 failed instants yielded 78 distinct
  states. Seventeen state fingerprints repeated; 38 failed instants belonged to
  a repeated fingerprint; the maximum exact repetition was four.
- The longest exact repeated-state sequence was four evaluations from 335.34 to
  338.22 s with the same four competitive blockers.
- The dominant reason repeats in long runs: for example 16 consecutive
  `UNQUERIED_OPTION_CAN_BEAT_EVALUATED_ASSIGNMENT` instants from 109.30 to
  171.52 s (across decision episodes), and 12 from 303.78 to 321.88 s.

Repeated attempts therefore reflect persistent unresolved evidence, not an
accept/reject oscillation after proof was already sufficient.

## Could any failed certificate safely have passed earlier?

No, not under the recorded cost-only certificate definition and evidence
available at each timestamp.

- All 142 competitive-bound failures had
  `best_optimistic_unqueried_score > current_evaluated_assignment_score`.
  The advantage margin was minimum 2.1617, median 79.7796, mean 75.5437, maximum
  169.8264 score units.
- The other 15 failures lacked generation-matching lower-bound provenance, so
  dominance could not be proven at that instant.
- Zero failed publications had a finite best optimistic score less than or equal
  to the evaluated assignment score.
- The implementation demonstrably does certify without exhaustive querying:
  eight publications used `ALL_UNQUERIED_OPTIONS_DOMINATED`, including paired
  successes with 129 and 122 DNU candidates.

Retrospective knowledge that a candidate later disappeared or received an
unfavourable exact cost is not evidence that existed at the earlier timestamp.
The artifact therefore provides no safe earlier-accept case and no evidence that
the comparison rule itself is excessively conservative.

## Per-certificate ledger

Each row is one failed evaluation instant. `Event sequence(s)` enumerates every
one of the 157 failed publications; paired values are the two robot publications
of the same global evaluation instant. `Blocking physical IDs` is the exact
number of unique `(robot_id, physical_id)` entries in those event payloads.
`Resolved later` classifies those exact IDs over the remainder of the run.

| Sim s | Event sequence(s) | Reason | Eval. | DNU | Blocking physical IDs | Blocking pair options | Next certified Δ s | Resolved later: queried / disappeared |
|---:|---|---|---:|---:|---:|---:|---:|---:|
| 39.20 | 518,521 | CAN_BEAT | 4 | 2 | 2 | 3 | 12.76 | 2 / 0 |
| 40.60 | 571,572 | CAN_BEAT | 4 | 9 | 9 | 48 | 11.36 | 8 / 1 |
| 44.56 | 595,596 | CAN_BEAT | 4 | 7 | 7 | 19 | 7.40 | 6 / 1 |
| 46.18 | 640,641 | MISSING_BOUND | 4 | 2 | 2 | 0 | 5.78 | 1 / 1 |
| 46.38 | 648,649 | CAN_BEAT | 4 | 2 | 2 | 4 | 5.58 | 1 / 1 |
| 50.34 | 701,702 | CAN_BEAT | 4 | 2 | 2 | 4 | 1.62 | 1 / 1 |
| 79.40 | 1041 | CAN_BEAT | 12 | 70 | 68 | 77 | 40.12 | 0 / 68 |
| 80.52 | 1094,1096 | CAN_BEAT | 14 | 98 | 97 | 155 | 39.00 | 30 / 67 |
| 83.62 | 1118 | CAN_BEAT | 6 | 64 | 64 | 154 | 35.90 | 62 / 2 |
| 84.18 | 1125 | CAN_BEAT | 6 | 64 | 64 | 154 | 35.34 | 62 / 2 |
| 85.06 | 1175 | CAN_BEAT | 6 | 59 | 59 | 78 | 34.46 | 58 / 1 |
| 85.18 | 1178 | CAN_BEAT | 6 | 59 | 59 | 78 | 34.34 | 58 / 1 |
| 88.30 | 1194 | MISSING_BOUND | 6 | 54 | 54 | 0 | 31.22 | 53 / 1 |
| 88.52 | 1200,1201 | CAN_BEAT | 6 | 54 | 54 | 62 | 31.00 | 53 / 1 |
| 90.18 | 1250,1251 | CAN_BEAT | 6 | 49 | 49 | 52 | 29.34 | 48 / 1 |
| 93.74 | 1271,1272 | CAN_BEAT | 6 | 44 | 44 | 46 | 25.78 | 43 / 1 |
| 95.40 | 1322,1323 | CAN_BEAT | 6 | 39 | 39 | 39 | 24.12 | 38 / 1 |
| 98.84 | 1344,1345 | CAN_BEAT | 6 | 34 | 34 | 34 | 20.68 | 33 / 1 |
| 100.52 | 1393,1395 | CAN_BEAT | 6 | 29 | 29 | 29 | 19.00 | 28 / 1 |
| 103.96 | 1414 | CAN_BEAT | 6 | 24 | 24 | 24 | 15.56 | 23 / 1 |
| 104.18 | 1422 | CAN_BEAT | 6 | 24 | 24 | 24 | 15.34 | 23 / 1 |
| 105.62 | 1464,1465 | CAN_BEAT | 6 | 19 | 19 | 19 | 13.90 | 18 / 1 |
| 109.06 | 1486 | MISSING_BOUND | 6 | 14 | 14 | 0 | 10.46 | 14 / 0 |
| 109.30 | 1493,1494 | CAN_BEAT | 6 | 14 | 14 | 14 | 10.22 | 14 / 0 |
| 110.96 | 1539,1540 | CAN_BEAT | 6 | 9 | 9 | 9 | 8.56 | 9 / 0 |
| 114.30 | 1562,1563 | CAN_BEAT | 6 | 4 | 4 | 4 | 5.22 | 4 / 0 |
| 115.74 | 1607,1608 | CAN_BEAT | 6 | 2 | 2 | 2 | 3.78 | 2 / 0 |
| 148.74 | 1962,1963 | CAN_BEAT | 5 | 39 | 22 | 22 | 24.24 | 1 / 21 |
| 153.64 | 2022,2023 | CAN_BEAT | 4 | 36 | 27 | 27 | 19.34 | 6 / 21 |
| 158.42 | 2102,2104 | CAN_BEAT | 16 | 32 | 22 | 22 | 14.56 | 11 / 11 |
| 159.86 | 2123,2124 | CAN_BEAT | 14 | 28 | 17 | 17 | 13.12 | 13 / 4 |
| 162.64 | 2179 | CAN_BEAT | 14 | 29 | 18 | 18 | 10.34 | 17 / 1 |
| 162.74 | 2181 | CAN_BEAT | 14 | 29 | 18 | 18 | 10.24 | 17 / 1 |
| 163.86 | 2197 | CAN_BEAT | 10 | 27 | 18 | 18 | 9.12 | 18 / 0 |
| 163.98 | 2199 | CAN_BEAT | 10 | 27 | 18 | 18 | 9.00 | 18 / 0 |
| 167.30 | 2259,2261 | CAN_BEAT | 10 | 22 | 13 | 13 | 5.68 | 13 / 0 |
| 168.52 | 2277,2278 | CAN_BEAT | 10 | 17 | 8 | 8 | 4.46 | 8 / 0 |
| 171.42 | 2331 | CAN_BEAT | 10 | 12 | 4 | 4 | 1.56 | 4 / 0 |
| 171.52 | 2333 | CAN_BEAT | 10 | 12 | 4 | 4 | 1.46 | 4 / 0 |
| 172.52 | 2343 | MISSING_BOUND | 10 | 7 | 3 | 0 | 0.46 | 0 / 3 |
| 204.08 | 2762 | CAN_BEAT | 12 | 57 | 5 | 5 | 10.44 | 0 / 5 |
| 204.20 | 2766 | CAN_BEAT | 12 | 57 | 5 | 5 | 10.32 | 0 / 5 |
| 211.74 | 2879,2881 | CAN_BEAT | 12 | 105 | 45 | 45 | 2.78 | 9 / 36 |
| 299.98 | 4011,4012 | CAN_BEAT | 8 | 72 | 72 | 2368 | 39.80 | 36 / 36 |
| 302.54 | 4095,4097 | CAN_BEAT | 8 | 67 | 67 | 1853 | 37.24 | 36 / 31 |
| 303.44 | 4114 | MISSING_BOUND | 8 | 62 | 62 | 0 | 36.34 | 32 / 30 |
| 303.78 | 4128 | CAN_BEAT | 8 | 62 | 62 | 1668 | 36.00 | 32 / 30 |
| 303.88 | 4130 | CAN_BEAT | 8 | 62 | 62 | 1668 | 35.90 | 32 / 30 |
| 305.88 | 4204,4205 | CAN_BEAT | 8 | 57 | 57 | 1203 | 33.90 | 32 / 25 |
| 307.44 | 4248,4250 | CAN_BEAT | 8 | 52 | 52 | 1068 | 32.34 | 31 / 21 |
| 307.54 | 4252 | CAN_BEAT | 8 | 52 | 52 | 1068 | 32.24 | 31 / 21 |
| 310.10 | 4327,4328 | CAN_BEAT | 8 | 47 | 47 | 653 | 29.68 | 31 / 16 |
| 311.66 | 4363,4365 | CAN_BEAT | 8 | 42 | 42 | 568 | 28.12 | 29 / 13 |
| 313.66 | 4408,4410 | CAN_BEAT | 8 | 37 | 37 | 203 | 26.12 | 29 / 8 |
| 314.88 | 4448,4449 | CAN_BEAT | 8 | 32 | 32 | 168 | 24.90 | 28 / 4 |
| 317.34 | 4518,4519 | CAN_BEAT | 8 | 29 | 29 | 29 | 22.44 | 27 / 2 |
| 318.88 | 4553,4554 | CAN_BEAT | 8 | 24 | 24 | 24 | 20.90 | 24 / 0 |
| 321.88 | 4614,4616 | CAN_BEAT | 8 | 24 | 24 | 24 | 17.90 | 24 / 0 |
| 322.98 | 4631 | MISSING_BOUND | 8 | 19 | 19 | 0 | 16.80 | 19 / 0 |
| 323.22 | 4648,4650 | CAN_BEAT | 8 | 19 | 19 | 19 | 16.56 | 19 / 0 |
| 325.78 | 4707,4709 | CAN_BEAT | 8 | 19 | 19 | 19 | 14.00 | 19 / 0 |
| 327.22 | 4740,4741 | CAN_BEAT | 8 | 14 | 14 | 14 | 12.56 | 14 / 0 |
| 329.66 | 4769 | CAN_BEAT | 8 | 14 | 14 | 14 | 10.12 | 14 / 0 |
| 329.78 | 4771 | CAN_BEAT | 8 | 14 | 14 | 14 | 10.00 | 14 / 0 |
| 331.10 | 4832,4833 | CAN_BEAT | 10 | 9 | 9 | 9 | 8.68 | 9 / 0 |
| 333.44 | 4845,4847 | MISSING_BOUND | 10 | 9 | 9 | 0 | 6.34 | 9 / 0 |
| 333.88 | 4871 | CAN_BEAT | 10 | 9 | 9 | 9 | 5.90 | 9 / 0 |
| 333.98 | 4873 | CAN_BEAT | 10 | 9 | 9 | 9 | 5.80 | 9 / 0 |
| 335.10 | 4924 | MISSING_BOUND | 10 | 4 | 4 | 0 | 4.68 | 4 / 0 |
| 335.34 | 4939 | CAN_BEAT | 10 | 4 | 4 | 4 | 4.44 | 4 / 0 |
| 335.54 | 4941 | CAN_BEAT | 10 | 4 | 4 | 4 | 4.24 | 4 / 0 |
| 338.10 | 4975 | CAN_BEAT | 10 | 4 | 4 | 4 | 1.68 | 4 / 0 |
| 338.22 | 4978 | CAN_BEAT | 10 | 4 | 4 | 4 | 1.56 | 4 / 0 |
| 339.22 | 4988 | MISSING_BOUND | 10 | 0 | 0 | 0 | 0.56 | 0 / 0 |
| 384.54 | 5540,5542 | CAN_BEAT | 10 | 32 | 32 | 71 | 29.00 | 17 / 15 |
| 388.88 | 5593 | MISSING_BOUND | 10 | 27 | 27 | 0 | 24.66 | 12 / 15 |
| 389.44 | 5617,5618 | CAN_BEAT | 10 | 27 | 27 | 62 | 24.10 | 12 / 15 |
| 391.44 | 5696 | CAN_BEAT | 10 | 22 | 22 | 41 | 22.10 | 11 / 11 |
| 391.54 | 5698 | CAN_BEAT | 10 | 22 | 22 | 41 | 22.00 | 11 / 11 |
| 396.22 | 5750,5752 | MISSING_BOUND | 10 | 17 | 17 | 0 | 17.32 | 6 / 11 |
| 396.66 | 5775,5776 | CAN_BEAT | 10 | 17 | 17 | 36 | 16.88 | 6 / 11 |
| 399.44 | 5826 | CAN_BEAT | 10 | 12 | 12 | 21 | 14.10 | 6 / 6 |
| 399.54 | 5828 | CAN_BEAT | 10 | 12 | 12 | 21 | 14.00 | 6 / 6 |
| 403.66 | 5896,5897 | CAN_BEAT | 10 | 7 | 7 | 16 | 9.88 | 1 / 6 |
| 410.98 | 6047,6048 | CAN_BEAT | 10 | 1 | 1 | 1 | 2.56 | 0 / 1 |
| 499.54 | 7572,7574 | CAN_BEAT | 8 | 60 | 60 | 1738 | 43.90 | 28 / 32 |
| 503.66 | 7643,7645 | CAN_BEAT | 8 | 55 | 55 | 1451 | 39.78 | 24 / 31 |
| 506.78 | 7721,7723 | CAN_BEAT | 8 | 50 | 50 | 1196 | 36.66 | 24 / 26 |
| 510.98 | 7777 | MISSING_BOUND | 8 | 45 | 45 | 0 | 32.46 | 20 / 25 |
| 511.34 | 7798,7799 | CAN_BEAT | 8 | 45 | 45 | 961 | 32.10 | 20 / 25 |
| 513.56 | 7842 | CAN_BEAT | 8 | 40 | 40 | 756 | 29.88 | 20 / 20 |
| 513.68 | 7844 | CAN_BEAT | 8 | 40 | 40 | 756 | 29.76 | 20 / 20 |
| 518.88 | 7912,7913 | CAN_BEAT | 8 | 35 | 35 | 571 | 24.56 | 15 / 20 |
| 521.20 | 7991,7992 | CAN_BEAT | 8 | 30 | 30 | 416 | 22.24 | 15 / 15 |
| 526.32 | 8063,8065 | CAN_BEAT | 8 | 25 | 25 | 281 | 17.12 | 10 / 15 |
| 528.44 | 8107,8108 | CAN_BEAT | 8 | 20 | 20 | 176 | 15.00 | 10 / 10 |
| 533.20 | 8177,8179 | CAN_BEAT | 8 | 15 | 15 | 91 | 10.24 | 5 / 10 |
| 535.44 | 8252,8253 | CAN_BEAT | 8 | 10 | 10 | 36 | 8.00 | 5 / 5 |
| 540.56 | 8329,8330 | CAN_BEAT | 8 | 5 | 5 | 19 | 2.88 | 1 / 4 |

For a `MISSING_BOUND` row, the physical IDs are the candidates whose diagnostic
provenance is stale. Matching bounds returned at the next evaluation in every
case with DNU candidates. The 339.22 s row has no physical blocker because DNU
was zero; the stale robot-2 generation fingerprint itself was the blocker.

## Cause classification

| Candidate explanation | Finding | Confidence |
|---|---|---|
| Too many candidates | **Proven contributor.** Failed evaluations average 29.53 DNU; peaks are 105 physical DNU and 2,368 blocking pair options. | High |
| Candidate churn invalidating evidence | **Proven contributor.** 249/429 blockers disappear unqueried; new generations also create sharp backlog increases. Only 15 publications are direct provenance mismatches, so mismatch invalidation is secondary. | High |
| Path-planner throughput | **Proven dominant rate limiter.** Query terminal throughput is about 1.68 requests/s over 600 s, matching five-candidate drain cycles at about 1.87 candidates/s; exact blocker queries commonly take tens of simulation seconds. | High |
| Missing lower bounds | **Real but small/transient.** Twelve instants, 0.20–0.56 s recovery, about 4.30 s total isolated waiting. | High |
| Query scheduling/leases | **Secondary and asymmetric.** Explicit lease-wait state totals 17.08 robot-s, concentrated on robot 1. The artifact does not prove the scheduler policy cause of that asymmetry. | High for magnitude; uncertain for policy cause |
| Conservative certificate logic | **Not supported as a defect.** Every failed score comparison was unsafe; dominated-unqueried success already works, including DNU 129/122. | High |
| Consensus/agreement | **Not the certificate bottleneck.** The long waits precede certification; prior stage timing showed the post-final-bid/decision path was small. | High |
| Traffic scheduling | **Not the certificate bottleneck.** Traffic occurs after a certifying decision and is a separate latency family. | High |

## Proven causes and unknowns

Proven:

- The single largest mechanism is competitive DNU backlog versus bounded exact
  path-query throughput.
- Candidate churn is substantial and causes both backlog replacement and wasted
  exact-query opportunity: 58.04% of unique blockers vanish before query.
- Lower-bound generation usually works; generation mismatch is brief and cannot
  explain the large latency.
- The certificate comparison behaves consistently with its safety contract and
  did not reject any state already proven safe by its own recorded evidence.

Not proven from this artifact:

- why robot 1 receives far fewer exact candidate evaluations than robot 2;
- whether the five-candidate drain is an explicit configured budget, an emergent
  service/lease limit, or both;
- whether the 365 aborted `ComputePathToPose` requests are intentional
  cancellation/churn or avoidable planner failures;
- geometry/centroid of unqueried physical IDs, because the certificate payload
  records stable identity and bound provenance but not full frontier geometry;
- whether an alternative lower-bound formula could be tighter while retaining
  the same safety proof. That would be a design question, not established by
  this read-only audit.

## Final verdict

The certificate cannot become valid quickly because each decision episode often
starts with tens of unqueried physical candidates whose conservative optimistic
bounds genuinely beat the currently evaluated pair assignment. Exact path-cost
work retires predominantly five candidates every roughly 2–5 simulation seconds,
while candidate generations can add or replace work. Certification therefore
waits until exact queries, disappearance, or bound domination resolves every
competitive option. Transient lower-bound provenance mismatch and query-lease
waiting add small secondary delays; the certificate acceptance rule itself is
not shown to be wrong or unnecessarily exhaustive.

`CERTIFICATE_PIPELINE_ROOT_CAUSE_IDENTIFIED`
