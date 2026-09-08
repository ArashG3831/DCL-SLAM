# Condition-C query-cap-8 latency bottleneck archaeology

## Verdict

**CERTIFICATE_WAS_TRUE_BOTTLENECK_BUT_SECONDARY_LIMIT_REMAINS**

The certificate was a real baseline bottleneck: certificate-deferred time
inside finite terminal-to-next-dispatch joins fell from **206.76 s** to
**133.44 s** after the query cap changed from 5 to 8.  That improvement did not
translate into lower total avoidable idle because the cap-8 run had a much
larger unresolved candidate backlog and spent its long tail waiting for valid
peer bids, allocator rounds, and local `ComputePathToPose` query leases.  The
new long-tail owner is therefore downstream of, and partly exposed by, the
certificate drain.

This is a read-only comparison of the two preserved artifacts.  No source was
changed, no build was run, and no Webots/ROS process was launched.

## Scope and authoritative inputs

| Run | Artifact | Relevant configuration evidence |
|---|---|---|
| Baseline | `results/thesis_condition_C_600s_20260907T215420Z/fast_trial_20260907T215426Z/observer/fast_trial_20260907T215426Z-01/` | `run_manifest.json`, `events.jsonl`, `goal_decision_ledger.jsonl`, `snappiness.json`, `avoidable_idle.json`, `navigation_action_replay.json`; maximum path queries 5 |
| Query-cap-8 | `results/thesis_condition_C_600s_20260907T234020Z/fast_trial_20260907T234025Z/observer/fast_trial_20260907T234025Z/` | Same artifact families; commit `6f820e8`; maximum path queries 8 |

Both runs are 600 s Condition C runs using the canonical close-start world,
`frontier_cost_only`, MODE_B, full sensors, ideal encoders, and the same
recorded seed 1001.  `snappiness.json` is used for the reported latency
percentiles; raw event timestamps and state transitions are used for the
decomposition.  The exact idle definition is the existing
`offline_timing_metrics.py` state machine: a segment is feasible only when
fresh candidate evidence and the required health evidence are present, and
avoidable idle is feasible work with no active navigation goal.

The reason-overlap values below are an archaeology crosswalk, not a new
scientific metric.  They intersect those existing exact idle segments with the
existing `STATE_TRANSITION` messages.  Every idle second is assigned to the
recorded message active over that interval; `other` means an explicit recorded
round/commitment/terminal state, not an inferred failure.

## Headline comparison

| Measure | Baseline | Query-cap-8 | Change |
|---|---:|---:|---:|
| Certificate-deferred time inside finite terminal→dispatch joins | 206.76 s | 133.44 s | −73.32 s (−35.5%) |
| Finite terminal→next-dispatch joins | 10 | 14 | +4 |
| Terminal→dispatch p50 | 43.88 s | 21.56 s | −22.32 s |
| Terminal→dispatch p95 | 58.12 s | 81.22 s | +23.10 s |
| Robot 1 avoidable idle | 145.86 s (56.9988%) | 133.94 s (54.0691%) | −11.92 s |
| Robot 2 avoidable idle | 75.98 s (22.8264%) | 101.40 s (50.0840%) | +25.42 s |
| Combined avoidable idle | 221.84 s | 235.34 s | +13.50 s |
| Certificate publications / failed / certified | 185 / 157 / 28 | 99 / 77 / 22 | fewer observed cycles, not zero blocking |
| Unique failed certificate evaluation instants | 99 | 49 | −50 |

The lower p50 shows a genuine central-latency improvement.  The higher p95
shows that the improvement did not control the tail.

## Certificate evaluation and defer periods

Unique same-timestamp paired publications are collapsed to evaluation instants.
The interval is from the first failed evaluation in an episode to the first
successful certificate, inclusive of the recorded failed-evaluation period.

### Baseline

| Episode | Failed evaluation interval (sim s) | Failed instants | First successful certificate (sim s) | Episode span |
|---:|---:|---:|---:|---:|
| 1 | 39.20–50.34 | 6 | 51.96 | 12.76 s |
| 2 | 79.40–115.74 | 21 | 119.52 | 40.12 s |
| 3 | 148.74–172.52 | 13 | 172.98 | 24.24 s |
| 4 | 204.08–211.74 | 3 | 214.52 | 10.44 s |
| 5 | 299.98–339.22 | 31 | 339.78 | 39.80 s |
| 6 | 384.54–410.98 | 11 | 413.54 | 29.00 s |
| 7 | 499.54–540.56 | 14 | 543.44 | 43.90 s |

The failed reason was predominantly
`UNQUERIED_OPTION_CAN_BEAT_EVALUATED_ASSIGNMENT` (142 of 157 publications);
the remaining failures were 15 missing-lower-bound publications.  The
certificate did not wait for traffic or Nav2 execution in these episodes; it
waited on conservative evidence for unqueried candidates.

### Query-cap-8

| Episode | Failed evaluation interval (sim s) | Failed instants | First successful certificate (sim s) | Result |
|---:|---:|---:|---:|---|
| 1 | 38.68–38.68 | 1 | 44.78 | 6.10 s |
| 2 | 68.74–98.18 | 16 | 98.40 | 29.66 s |
| 3 | 132.08–160.86 | 14 | 165.74 | 33.66 s |
| 4 | 197.08–213.86 | 7 | 215.42 | 18.34 s |
| 5 | 244.86–244.86 | 1 | 263.54 | 18.68 s |
| 6 | 463.10–536.08 | 10 | none before the 600 s horizon | open at the end |

The finite episode span is shorter in aggregate than baseline, but the late
episode never certified and therefore never produced a subsequent dispatch.
That open deferral is absent from the finite terminal→dispatch statistic by
construction.  It is a direct reason that reducing the finite 206.76 s number
does not imply that the mission became continuously productive.

## Terminal-to-dispatch decomposition

The table lists every recorded navigation terminal.  `dispatch` is the next
same-robot `NAV_GOAL_SENT`; `OPEN` means no later dispatch exists.  `candidate
feasible` and `candidate unavailable` are overlaps with the existing exact
idle classifier.  `cert`, `query`, `coord`, and `traffic` are overlaps with
the recorded state-transition reason text.  The raw event gap for the cap-8
p95 row is 81.34 s (134.52→215.86); the authoritative replay join in
`snappiness.json` is 81.22 s because its bag/header boundary is 0.12 s
earlier.  The requested p95 uses the authoritative 81.22 s value.

| Run/robot | Terminal (sim s) | Next dispatch (sim s) | Raw gap | Candidate feasible / unavailable | cert | coord | query | traffic | terminal/other |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| Base r1 | 75.74 S | 119.74 | 44.00 | 39.56 / 4.44 | 36.28 | 4.72 | 0.22 | 0 | 2.78 |
| Base r1 | 146.86 S | 173.20 | 26.34 | 24.90 / 1.44 | 19.58 | 2.96 | 2.02 | 0 | 1.78 |
| Base r1 | 192.20 S | OPEN | 407.88 | 138.54 / 269.34 | 99.60 | 95.76 | 13.60 | 91.72 | 2.44 / 104.76 |
| Base r2 | 74.40 S | 119.74 | 45.34 | 39.88 / 5.46 | 37.08 | 4.90 | 0.80 | 0 | 2.56 |
| Base r2 | 155.42 S | 173.20 | 17.78 | 17.78 / 0 | 12.12 | 2.76 | 0.44 | 0 | 2.46 |
| Base r2 | 201.74 S | 214.74 | 13.00 | 3.44 / 9.56 | 7.00 | 3.76 | 0 | 0 | 2.24 |
| Base r2 | 251.42 S | 260.54 | 9.12 | 0.88 / 8.24 | 0 | 6.80 | 0 | 0 | 2.32 |
| Base r2 | 291.88 S | 340.54 | 48.66 | 41.00 / 7.66 | 31.50 | 15.16 | 0 | 0 | 2.00 |
| Base r2 | 367.66 S | 413.78 | 46.12 | 25.12 / 21.00 | 20.80 | 23.54 | 0 | 0 | 1.78 |
| Base r2 | 433.34 S | 450.66 | 17.32 | 5.76 / 11.56 | 0 | 15.54 | 0 | 0 | 1.78 |
| Base r2 | 485.44 S | 543.56 | 58.12 | 36.70 / 21.42 | 39.90 | 16.44 | 0 | 0 | 1.78 |
| Base r2 | 589.32 S | OPEN | 10.76 | 0 / 10.76 | 0 | 8.76 | 0 | 0 | 2.00 |
| Cap8 r1 | 58.50 S | 98.62 | 40.12 | 28.38 / 11.74 | 24.54 | 12.48 | 0.10 | 0 | 3.00 |
| Cap8 r1 | 134.52 S | 215.86 | 81.34 | 72.24 / 9.10 | 35.34 | 31.72 | 11.50 | 0 | 2.78 |
| Cap8 r1 | 241.98 S | 263.88 | 21.90 | 6.78 / 15.12 | 6.56 | 12.14 | 0.44 | 0 | 2.76 |
| Cap8 r1 | 291.66 S | 304.22 | 12.56 | 2.00 / 10.56 | 0 | 9.12 | 0.66 | 0 | 2.78 |
| Cap8 r1 | 327.10 S | 344.54 | 17.44 | 1.88 / 15.56 | 0 | 14.54 | 0.90 | 0 | 2.00 |
| Cap8 r1 | 367.34 S | 387.98 | 20.64 | 7.56 / 13.08 | 0 | 17.98 | 0.44 | 0 | 2.22 |
| Cap8 r1 | 401.22 S | OPEN | 198.86 | 43.74 / 155.12 | 58.44 | 136.42 | 1.78 | 0 | 2.22 |
| Cap8 r2 | 35.78 F | 45.10 | 9.32 | 8.26 / 1.06 | 6.00 | 0.44 | 0 | 0 | 2.88 |
| Cap8 r2 | 69.52 S | 101.96 | 32.44 | 24.64 / 7.80 | 22.88 | 5.76 | 0.68 | 0 | 3.12 |
| Cap8 r2 | 128.98 S | 165.86 | 36.88 | 34.68 / 2.20 | 25.46 | 6.60 | 1.92 | 0 | 2.90 |
| Cap8 r2 | 194.08 S | 215.86 | 21.78 | 16.68 / 5.10 | 12.66 | 6.10 | 0.24 | 0 | 2.78 |
| Cap8 r2 | 253.30 S | 263.88 | 10.58 | 1.00 / 9.58 | 0 | 7.80 | 0.22 | 0 | 2.56 |
| Cap8 r2 | 294.98 S | 304.34 | 9.36 | 1.24 / 8.12 | 0 | 6.24 | 0.44 | 0 | 2.68 |
| Cap8 r2 | 332.44 S | 344.54 | 12.10 | 1.44 / 10.66 | 0 | 9.54 | 0.44 | 0 | 2.12 |
| Cap8 r2 | 366.34 S | 387.88 | 21.54 | 1.54 / 20.00 | 0 | 18.10 | 1.00 | 0 | 2.44 |
| Cap8 r2 | 431.88 S | OPEN | 168.20 | 20.78 / 147.42 | 59.36 | 104.64 | 4.20 | 0 | 0 |

The finite cap-8 p95 event is therefore not a traffic event.  Its 81.22 s
authoritative join is the robot-1 terminal at 134.52 s followed by the next
dispatch at 215.86 s.  After the certificate succeeds at 165.74 s, the
recorded state remains dominated by allocator/coordination and query-lease
waits until dispatch.  The baseline p95 event is robot 2 at 485.44→543.56 s;
there certificate contributes 39.90 s of the 58.12 s gap and coordination
contributes 16.44 s.

## Robot-by-robot idle explanation

### Exact idle and availability outputs

| Robot/run | Feasible work | Avoidable idle | Idle fraction | Productive engagement | Work unavailable | Longest idle | Idle after last terminal |
|---|---:|---:|---:|---:|---:|---:|---:|
| Baseline r1 | 255.90 | 145.86 | 56.9988% | 110.04 | 307.58 | 41.12 | 138.54 |
| Cap8 r1 | 247.72 | 133.94 | 54.0691% | 113.78 | 318.86 | 32.56 | 43.74 |
| Baseline r2 | 332.86 | 75.98 | 22.8264% | 256.88 | 230.62 | 15.32 | 0 |
| Cap8 r2 | 202.46 | 101.40 | 50.0840% | 101.06 | 364.12 | 10.68 | 20.78 |

The existing metric has no separate scientific fields for “certificate idle,”
“assignment idle,” or “navigation idle.”  The following crosswalk is the
intersection of its avoidable-idle segments with the recorded state-reason
messages, so it explains the artifact without replacing the established idle
definition.

| Robot | Run | Certificate state overlap | Allocator/coordination overlap | Planner/query overlap | Traffic overlap | Navigation/terminal overlap |
|---|---|---:|---:|---:|---:|---:|
| r1 | Baseline | 56.36 | 26.18 | 6.88 | 51.22 | 5.22 |
| r1 | Cap8 | 56.60 | 67.12 | 7.78 | 0 | 2.44 |
| r2 | Baseline | 52.38 | 23.14 | 0.46 | 0 | 0 |
| r2 | Cap8 | 78.22 | 13.12 | 5.02 | 0 | 5.04 |

These columns sum to the exact avoidable-idle total for each row; they are not
double-counted.  `WORK_UNAVAILABLE` is outside avoidable idle by definition,
and its increase is shown in the exact table above.

Robot 1 improved only slightly because its idle certificate overlap was
unchanged (56.36→56.60 s).  Its reduction came mainly from less post-final
idle and no recorded traffic-wait overlap in cap 8, while allocator/coordination
overlap increased from 26.18→67.12 s.

Robot 2 became worse for the opposite reason.  Its finite terminal-gap
certificate overlap fell from 150.78 s to 67.00 s, but certificate-state
overlap *inside exact avoidable-idle segments* rose from 52.38→78.22 s.  Those
are different denominators: much of the baseline certificate time was during
intervals not classified as feasible idle, while the residual cap-8 certificate
wait occurred while feasible work was available and no goal was active.  At the
same time robot 2's feasible-work window fell by 130.40 s and productive
engagement fell by 155.82 s.  The resulting 364.12 s of work-unavailable time
and the still-open late certificate episode explain why lower finite certificate
latency did not lower robot-2 mission idle.

There was no separate navigation-execution stall causing the p95 regression:
baseline had 12 successful navigation terminals; cap 8 had 15 successes and one
failure, and `dispatch_to_terminal` p95 improved from 45.76 s to 44.10 s.  The
cap-8 failed goal was robot 2 at 35.78 s, retried at 45.10 s, and is not the
81.22 s p95 event.

## Candidate backlog and query throughput

| Measure | Baseline cap 5 | Query-cap-8 | Interpretation |
|---|---:|---:|---|
| Candidate batches | 148 | 87 | Fewer batch arrivals in this run |
| Detected-frontier observations | 9,362 | 6,614 | Fewer observations overall |
| Exact candidate records in batches | 3,253 | 1,708 | Fewer evaluated candidates in the source batches |
| Detected-not-queried observations | 2,191 | 4,648 | Backlog more than doubled |
| Unreachable observations | 3,889 | 201 | More cap-8 candidates remained unresolved rather than being classified unreachable |
| Frontier lifecycle cycles | 213 | 154 | Fewer cycles did not imply a smaller backlog |
| Tier-1 candidates selected | 825 | 1,156 | More selected query work under cap 8 |
| Tier-1 candidates sent | 575 | 710 | More query submissions |
| `ComputePathToPose` terminal requests | 1,006 | 1,127 | +121 requests (+12.0%) |
| Successful path plans | 641 | 1,078 | More successful plans |
| Aborted path plans | 365 | 49 | Fewer planner aborts |
| Maximum frontier snapshot | 159 | 263 | Much larger cap-8 candidate population |
| Candidate-cache evictions | 0 | 84 | Cap-8 run hit the cache-capacity path |
| Unique blocking physical IDs in failed certificate diagnostics | 429 | 707 | More distinct blockers appeared |
| Failed evaluation instants | 99 | 49 | Fewer failed evaluation times, but not an empty backlog |
| Mean DNU count at failed instants | 29.64 | 122.35 | Failed cap-8 evaluations saw a much larger DNU population |

The late cap-8 lifecycle is direct evidence of the exposed secondary limit:
the robot-1/robot-2 frontier snapshots reached 263 records, the cache retained
256 with capacity evictions, and successive cycles still reported DNU values
of 252, 244, 236, 228, 220, 212, 204, 196, 188, and 180.  Each cycle selected
eight candidates and commonly sent eight queries, but the backlog remained
large through the horizon.  Baseline late cycles instead drained DNU from 30
to 0 before entering `WORK_EXHAUSTED` cycles.

The certificate diagnostic histories show the same direction: among physical
IDs that appeared as blockers, cap 8 had 707 unique identities, with 407 still
present in the final history and 334 marked disappeared; baseline had 429
identities and all had resolved by the final certificate history.  These are
certificate-history identities and are not substituted for the exact action
query count above.

The query cap therefore increased query throughput, but the run presented a
larger and more persistent competitive candidate population.  It did not
remove the conservative certificate contract; it moved the limiting wait into
the subsequent evidence/allocator path for the long tail.

## Exact p95 regression

### Baseline p95: 58.12 s

- Robot 2 terminal: **485.44 s**.
- Next dispatch: **543.56 s**.
- Certificate overlap: **39.90 s**.
- Allocator/coordination overlap: **16.44 s**, primarily fresh peer snapshots
  and valid peer-bid waits.
- Candidate-feasible / unavailable: **36.70 / 21.42 s**.
- No planner/query-lease or traffic interval was the dominant owner.

### Query-cap-8 p95: 81.22 s

- Robot 1 terminal: **134.52 s**, task `0641eca4...`.
- Next dispatch: **215.86 s**, task `f29aaa5e...`.
- Certificate overlap: **35.34 s**, ending with the successful certificate at
  165.74 s.
- Allocator/coordination overlap: **31.72 s**; the largest single recorded
  state was **27.34 s waiting for valid peer bids**.
- Planner/query overlap: **11.50 s waiting for the local
  `ComputePathToPose` query lease**.
- Candidate-feasible / unavailable: **72.24 / 9.10 s**.  Thus candidates
  existed; lack of candidate availability was not the main owner of this
  p95 event.
- Traffic overlap: **0 s**; the cap-8 launch log has no
  `WAITING_FOR_TRAFFIC` or `TRAFFIC_CONFLICT_CLEARED` records.
- `agreement_to_nav_goal_sent` p95 remained sub-second (0.34 s), so the final
  agreement-to-dispatch handoff was not the source of the tail.

The cap-8 p95 event is therefore classified as:

1. certificate waiting, real but shorter than the baseline p95 certificate
   component;
2. then allocator/peer-bid delay;
3. then local query-lease/planner delay;
4. not traffic, navigation execution, or no-candidate availability.

The explicit `new canonical round`/commitment intervals account for the small
remaining `other` portion; no materially unexplained interval remains in this
event.

## Final answer to the bottleneck question

The cap was fixing a real baseline certificate bottleneck, as shown by the
35.5% reduction in finite terminal-gap certificate time and the lower p50.
It was not sufficient to reduce mission idle because:

1. the cap-8 run accumulated a larger DNU/competitive candidate backlog;
2. an unresolved certificate episode remained open to the horizon;
3. robot 1's p95 tail became peer-bid/allocator/query-lease dominated after
   certificate success;
4. robot 2's certificate-state overlap inside the exact avoidable-idle windows
   increased even while its terminal-gap certificate total decreased; and
5. the exact idle metric counts feasible work with no active goal, not only
   certificate-deferred seconds.

Accordingly, the evidence supports **CERTIFICATE_WAS_TRUE_BOTTLENECK_BUT_SECONDARY_LIMIT_REMAINS**, not “certificate was the only bottleneck” and not
“the cap had no effect.”

## Limitations

The two preserved artifacts do not contain a first-class per-idle-second label
for certificate versus allocator versus planner ownership.  The reason table
is therefore a deterministic crosswalk from the exact idle intervals and the
recorded state-transition messages.  Also, the action replay records exact
query outcomes but does not provide a lossless physical-candidate ID join for
every planner request; candidate-level claims are consequently limited to the
certificate diagnostics and the lifecycle/query counters shown above.
