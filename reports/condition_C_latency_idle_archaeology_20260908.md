# Condition-C task-latency and avoidable-idle archaeology

Date: 2026-09-08
Repository HEAD during analysis: `7351816a8e681cbb5bcd6ce7c7499d3a8f007ee5`

## Scope

This is a bounded, read-only analysis of exactly this completed canonical Condition-C artifact:

```text
/home/arash/webots_ws_clean_validation_20260823/results/
  thesis_condition_C_600s_20260907T215420Z/
  fast_trial_20260907T215426Z/
```

The observer artifact is:

```text
fast_trial_20260907T215426Z/observer/fast_trial_20260907T215426Z-01/
```

No source, configuration, build, or runtime state was changed. No Webots or ROS process was launched. The analysis used the finalized `events.jsonl`, `goal_decision_ledger.jsonl`, forensic odometry CSVs, native-bag export index, `navigation_action_replay.json`, `avoidable_idle.json`, `snappiness.json`, and `cooperation_summary.json` from this run.

All times below are simulation time. `NAVIGATION_SUCCEEDED` is the scientific terminal timestamp. Native NavigateToPose status messages have a zero ROS stamp; mapping their bag-receipt wall timestamps to the nearest `/clock` samples places them within approximately one `/clock` sample of the distributed terminal event. Small apparent negative offsets are clock-sampling interpolation, not evidence that the terminal record preceded Nav2 success. The allocator's explicit `navigation terminal result` state reset occurred within 0.12 s of every terminal event, so terminal publication/receipt is not the long delay.

“First motion” means the first forensic odometry sample after dispatch with linear speed greater than 0.001 m/s or angular speed greater than 0.01 rad/s. This rejects floating-point stationary noise while retaining actual turning or translation.

## Executive finding

The 58.12 s tail is caused by repeated allocator/certificate cycling before a dispatchable round is formed. It is not caused by final consensus, traffic scheduling, terminal publication, or Nav2 startup.

Across the ten successful terminal-to-next-dispatch joins:

- terminal-to-dispatch time totals **325.68 s**;
- first post-terminal candidate arrival accounts for 77.54 s, or **23.81%**;
- first candidate to the eventual dispatch-round bid accounts for 242.20 s, or **74.37%**;
- the entire final bid → certificate → decision → agreement → dispatch chain accounts for only 5.94 s, or **1.82%**;
- allocator-state integration attributes **206.76 s (63.49%)** directly to `cost-only dispatch certificate deferred`;
- 193 certificate observations in these gaps were non-certifying: 177 `UNQUERIED_OPTION_CAN_BEAT_EVALUATED_ASSIGNMENT` and 16 `MISSING_UNQUERIED_LOWER_BOUNDS`.

The same mechanism is the largest avoidable-idle owner: certificate deferral accounts for **110.66 of 221.84 avoidable-idle seconds (49.88%)**. Traffic waiting is second at 51.46 s (23.20%), all on robot1 after it had completed its last dispatched goal.

## Complete timestamp pipeline

The run produced 12 successful goals. Ten have a later dispatch for the same robot and therefore form finite terminal→next-dispatch joins. The final success for robot1 at 192.08 s and robot2 at 589.32 s has no subsequent same-robot dispatch before the 600 s horizon.

`Candidate` is the first new candidate batch for that robot after terminal. `Bid` is the first bid publication belonging to the round that eventually dispatched. Consequently, candidate→bid includes all intervening candidate revisions, bidding attempts, certificate deferrals, and peer-evidence waits; it is not the execution time of one bid callback.

| Robot | Terminal success / terminal record | Allocator terminal reset | Next candidate | Dispatch-round bid | Certified / pair decision | Agreement | Traffic event affecting final dispatch | NAV_GOAL_SENT | First motion |
|---|---:|---:|---:|---:|---:|---:|---|---:|---:|
| robot1 | 75.74 | 75.84 | 83.06 | 119.40 | 119.52 / 119.52 | 119.62 | none | 119.62 | 120.74 |
| robot1 | 146.86 | 146.86 | 148.30 | 172.74 | 172.98 / 172.98 | 173.08 | none | 173.20 | 173.42 |
| robot1 | 192.08 | 192.08 | 195.42 | — | — | — | later traffic waits; no own dispatch | — | — |
| robot2 | 74.40 | 74.40 | 79.74 | 119.40 | 119.52 / 119.52 | 119.62 | none | 119.62 | 120.84 |
| robot2 | 155.42 | 155.52 | 159.30 | 172.74 | 172.98 / 172.98 | 173.08 | none | 173.20 | 173.42 |
| robot2 | 201.74 | 201.74 | 211.30 | 214.08 | 214.52 / 214.52 | 214.64 | none | 214.74 | 214.98 |
| robot2 | 251.42 | 251.42 | 259.66 | 259.78 | 260.22 / 260.22 | 260.34 | none | 260.54 | 261.54 |
| robot2 | 291.88 | 291.88 | 299.54 | 339.10 | 339.66 / 339.66 | 340.44 | wait 340.22; priority 340.44 | 340.54 | 341.66 |
| robot2 | 367.54 | 367.66 | 380.98 | 413.10 | 413.54 / 413.54 | 413.66 | stale release 383.78; wait/priority 413.66 | 413.78 | 413.98 |
| robot2 | 433.34 | 433.44 | 443.34 | 450.10 | 450.44 / 450.44 | 450.54 | none | 450.66 | 451.66 |
| robot2 | 485.44 | 485.44 | 496.10 | 543.08 | 543.32 / 543.32 | 543.44 | wait/priority 543.44 | 543.56 | 544.68 |
| robot2 | 589.32 | 589.32 | none before horizon | — | — | — | — | — | — |

The terminal record is the `NAVIGATION_SUCCEEDED` distributed event, so its publication timestamp is the terminal timestamp shown above. The allocator is also the producer/consumer of this lifecycle transition; the separate state-reset column shows the observable receipt/processing consequence.

## Individual stage latencies

| Robot | Terminal→dispatch | Terminal→candidate | Candidate→dispatch-round bid | Bid→certificate | Certificate→decision | Decision→agreement | Agreement→dispatch | Dispatch→motion |
|---|---:|---:|---:|---:|---:|---:|---:|---:|
| robot1 | 43.88 | 7.32 | 36.34 | 0.12 | 0.00 | 0.10 | 0.00 | 1.12 |
| robot1 | 26.34 | 1.44 | 24.44 | 0.24 | 0.00 | 0.10 | 0.12 | 0.22 |
| robot2 | 45.22 | 5.34 | 39.66 | 0.12 | 0.00 | 0.10 | 0.00 | 1.22 |
| robot2 | 17.78 | 3.88 | 13.44 | 0.24 | 0.00 | 0.10 | 0.12 | 0.22 |
| robot2 | 13.00 | 9.56 | 2.78 | 0.44 | 0.00 | 0.12 | 0.10 | 0.24 |
| robot2 | 9.12 | 8.24 | 0.12 | 0.44 | 0.00 | 0.12 | 0.20 | 1.00 |
| robot2 | 48.66 | 7.66 | 39.56 | 0.56 | 0.00 | 0.78 | 0.10 | 1.12 |
| robot2 | 46.24 | 13.44 | 32.12 | 0.44 | 0.00 | 0.12 | 0.12 | 0.20 |
| robot2 | 17.32 | 10.00 | 6.76 | 0.34 | 0.00 | 0.10 | 0.12 | 1.00 |
| robot2 | 58.12 | 10.66 | 46.98 | 0.24 | 0.00 | 0.12 | 0.12 | 1.12 |

All values are seconds. The table decomposition sums to terminal→dispatch; dispatch→motion is additional.

## Latency distributions

| Stage | Count | Total | Median | p95 | Maximum |
|---|---:|---:|---:|---:|---:|
| Terminal→candidate | 10 | 77.54 | 7.95 | 13.44 | 13.44 |
| Candidate→dispatch-round bid | 10 | 242.20 | 28.28 | 46.98 | 46.98 |
| Bid→certificate | 10 | 3.18 | 0.29 | 0.56 | 0.56 |
| Certificate→pair decision | 10 | 0.00 | 0.00 | 0.00 | 0.00 |
| Pair decision→agreement | 10 | 1.76 | 0.11 | 0.78 | 0.78 |
| Agreement→NAV_GOAL_SENT | 10 | 1.00 | 0.12 | 0.20 | 0.20 |
| NAV_GOAL_SENT→first motion | 10 | 7.46 | 1.00 | 1.22 | 1.22 |
| Terminal→NAV_GOAL_SENT | 10 | 325.68 | 35.11 | **58.12** | **58.12** |
| Terminal→first motion | 10 | 333.14 | 35.78 | 59.24 | 59.24 |

Per-robot terminal→dispatch distributions:

| Robot | Count | Total | Median | p95 / max |
|---|---:|---:|---:|---:|
| robot1 | 2 | 70.22 | 35.11 | 43.88 |
| robot2 | 8 | 255.46 | 31.50 | 58.12 |

Robot1's candidate delay median/p95 is 4.38/7.32 s and candidate→dispatch-round-bid median/p95 is 30.39/36.34 s. Robot2's corresponding values are 8.90/13.44 s and 22.78/46.98 s. Final dispatch and Nav2 activation remain short for both robots.

## What happened inside the terminal gaps

Integrating the observable allocator state/reason from each terminal to its next dispatch gives:

| State/reason family | Robot1 | Robot2 | Total | Share of 325.68 s |
|---|---:|---:|---:|---:|
| Certificate deferred | 55.98 | 150.78 | **206.76** | **63.49%** |
| Waiting for fresh task snapshots | 0.00 | 52.92 | 52.92 | 16.25% |
| Waiting for peer/continuation bids | 3.76 | 21.26 | 25.02 | 7.68% |
| Terminal/reset propagation state | 4.56 | 16.92 | 21.48 | 6.60% |
| Round evaluation | 3.20 | 8.34 | 11.54 | 3.54% |
| Waiting for matching decision | 0.32 | 3.90 | 4.22 | 1.30% |
| Waiting for local planner-query lease | 2.28 | 1.22 | 3.50 | 1.07% |
| Commitment changed | 0.12 | 0.12 | 0.24 | 0.07% |
| Traffic-wait state of the robot awaiting its next dispatch | 0.00 | 0.00 | **0.00** | **0.00%** |

The longest individual join, robot2 485.44→543.56 s, spent 40.46 s in certificate deferral, 11.88 s waiting for fresh snapshots, 2.30 s waiting for peer bids, and only 0.48 s from final bid through dispatch. It had 25 failed certificates: 24 `UNQUERIED_OPTION_CAN_BEAT_EVALUATED_ASSIGNMENT` and one `MISSING_UNQUERIED_LOWER_BOUNDS`.

The 291.88→340.54 s join similarly spent 32.26 s certificate-deferred and had 47 failed certificates. Traffic selected robot2 at the end, but wait and priority events were only 0.22 s apart; traffic did not create the preceding 48 s delay.

## Handoff→first shared goal

`START_RELEASE` occurred at 37.18 s. The first goals were sent at 52.28 s and 52.38 s, yielding the reported 15.10 s handoff→first-shared-goal latency.

Candidate/task evidence already existed before release and refreshed at 38.78, 40.38, 44.34, 46.06, 50.14, and 51.74 s. Between release and the successful certificate at 51.96 s, there were 12 paired non-certifying certificate records:

- ten records reported `UNQUERIED_OPTION_CAN_BEAT_EVALUATED_ASSIGNMENT`;
- two reported `MISSING_UNQUERIED_LOWER_BOUNDS`.

The final dispatch round then took only 0.10 s to agreement and another 0.22/0.32 s to robot1/robot2 dispatch. Thus the 15.10 s startup latency has the same proximate cause as the terminal tail: certificate completeness/dominance was not established until 51.96 s.

## Avoidable-idle attribution

The authoritative offline result is:

| Robot | Feasible-work time | Avoidable idle | Avoidable fraction | Productive engagement | Work unavailable |
|---|---:|---:|---:|---:|---:|
| robot1 | 255.90 | 145.86 | 56.9988% | 110.04 | 307.58 |
| robot2 | 332.86 | 75.98 | 22.8264% | 256.88 | 230.62 |
| **Total** | **588.76** | **221.84** | — | **366.92** | **538.20** |

Every avoidable-idle interval has `goal_active=false`. Therefore, by the frozen metric definition:

- **221.84 s (100%)** is feasible work while no Nav goal is assigned/active;
- **0 s** is an assigned active Nav goal that merely failed to start moving;
- dispatch→first-motion is only 7.46 s over all ten joins, p95 1.22 s;
- frontier/work shortage contributes 538.20 s of separate `WORK_UNAVAILABLE` time, but none of it is counted as avoidable idle.

Splitting every avoidable interval by the allocator state/reason active at that time gives:

| Avoidable-idle owner | Robot1 | Robot2 | Total | Share of all avoidable idle |
|---|---:|---:|---:|---:|
| Certificate deferred | 57.32 | 53.34 | **110.66** | **49.88%** |
| Traffic/reallocation | 51.46 | 0.00 | **51.46** | **23.20%** |
| Waiting for peer/continuation bids | 11.52 | 7.92 | 19.44 | 8.76% |
| Waiting for fresh task snapshots | 0.12 | 11.56 | 11.68 | 5.27% |
| Round evaluation | 5.62 | 2.72 | 8.34 | 3.76% |
| Waiting for local planner-query lease | 5.78 | 0.44 | 6.22 | 2.80% |
| Terminal/reset state | 5.34 | 0.00 | 5.34 | 2.41% |
| Waiting for matching decision | 4.94 | 0.00 | 4.94 | 2.23% |
| Commitment changed | 3.76 | 0.00 | 3.76 | 1.69% |
| **Total** | **145.86** | **75.98** | **221.84** | **100%** |

Finite terminal→next-dispatch intervals overlap 78.86 s, or 35.55%, of total avoidable idle:

- robot2: all 75.98 avoidable seconds lie inside its terminal→next-dispatch gaps;
- robot1: only 2.88 of 145.86 seconds lies inside a finite join;
- the other 142.98 robot1 seconds occur after robot1's last dispatch/terminal and are absent from the terminal→next-dispatch statistic because robot1 never receives another goal.

This explains why the 58.12 s latency and 56.9988% robot1 idle are related but are not the same statistic. The latency metric fully explains robot2's measured avoidable idle. Robot1's larger problem is the absence of any later own dispatch after 192.08 s despite continuing protocol activity.

After robot1's final success, the artifact contains 49 further robot1 candidate snapshots through 567.66 s, 55 bid arrays, 56 certificate records, 38 agreement publications, 35 traffic-wait events, 35 traffic releases, 35 continuation-round starts, and 32 continuation agreements. Robot1 candidates alternate between zero and small feasible sets (typically one or two). The pipeline is active; robot1 is not idle because the allocator process stopped.

## Correlation with DNU, candidates, agreements, traffic, and motion

- DNU series: 1,304 observations, 827 nonzero, maximum 103. Certificate payloads inside finite terminal gaps reached `detected_not_queried_count=129`.
- Finite terminal gaps contain 215 certificate records; 193 are non-certifying and 202 report nonzero DNU.
- Avoidable intervals contain 117 certificate records. Of these, 105 are non-certifying and 104 report nonzero DNU.
- Avoidable intervals still contain 51 candidate/task snapshots and 112 bid-array publications. Candidate generation and bidding are occurring; certification repeatedly rejects the incomplete evaluated set.
- Robot1 avoidable intervals contain 36 agreement events, 50 continuation-round starts, and 30 continuation agreements. Agreement machinery is active but often assigns/continues robot2 or reaches a traffic conflict rather than dispatching robot1.
- Robot1's 51.46 traffic-attributed avoidable seconds occur after its final goal. They are real safety waits, but they do not explain the 58.12 s robot2 terminal→dispatch p95.
- No avoidable interval is attributed to an active Nav goal. No no-progress, stuck, or oscillation episode was recorded, and dispatch→motion is consistently short.

## Direct answers

### 1. What causes the 58.12 s terminal→dispatch latency?

- Candidate generation delay: **secondary**, p95 13.44 s and 23.81% of aggregate terminal gaps.
- Allocator waiting: **yes**, specifically repeated round/certificate cycling.
- Consensus/agreement delay: **no** as the dominant cause; pair decision→agreement p95 is 0.78 s and agreement→dispatch p95 is 0.20 s.
- Traffic scheduler: **no** for the 58.12 s tail. The eventual dispatching robot received immediate priority at 543.44 s and dispatched at 543.56 s.
- Waiting for peer evidence: **secondary**—fresh snapshots and peer bids account for 77.94 s (23.93%) of terminal gaps.
- No feasible work: **not for the measured avoidable intervals**; those intervals explicitly have feasible work. Work-unavailable time is tracked separately.
- Something else: **the primary cause is the fail-closed cost-only certificate gate**, chiefly unqueried candidates that could beat the evaluated assignment, with occasional missing unqueried lower bounds.

### 2. What does the avoidable-idle metric represent here?

It is entirely feasible work with no active assigned Nav goal. It is not assigned-but-stationary Nav2 behavior. Certificate deferral owns 49.88%, traffic/reallocation owns 23.20%, and peer/snapshot evidence waits own another 14.03%. Frontier shortage creates substantial additional physical inactivity but is explicitly excluded from avoidable idle.

### 3. How do idle intervals compare with traffic, candidates, agreements, DNU, continuations, and motion?

The intervals contain abundant candidate, bid, agreement, DNU, and continuation activity. Certificate deferrals correlate directly with nonzero DNU and dominate the time. Traffic is a substantial robot1-only secondary contributor. Motion starts within 1.22 s p95 once a goal is finally dispatched.

### 4. What is the single largest bottleneck?

**The cost-only dispatch certificate remains non-certifying while potentially superior unqueried candidates or missing lower-bound evidence remain.** This accounts for 206.76 s of finite terminal gaps and 110.66 s of avoidable idle—larger than every other attributable family.

### 5. Is a deeper implementation cause proven?

The artifact proves the proximate state-machine/evidence cause. It does not by itself prove why the candidate-query/lower-bound pipeline leaves so many options unqueried for so long—possible planner-query throughput, candidate churn, lease policy, or evidence invalidation mechanisms require separate source/runtime analysis. It also does not prove that a safe non-conflicting robot1 task existed during every traffic wait. Those remain hypotheses, and no fix is proposed here.

## Proven causes

1. Terminal publication and allocator terminal receipt are prompt; they are not the bottleneck.
2. Candidate refresh has a measurable delay but cannot explain the 46–58 s tail alone.
3. Repeated fail-closed certificate deferral is the dominant terminal-gap and avoidable-idle owner.
4. Peer snapshots/bids are secondary contributors.
5. Final consensus, dispatch publication, and Nav2 movement startup are fast.
6. Traffic is not responsible for the terminal→dispatch p95, but it is the second-largest total avoidable-idle contributor because robot1 remains free after its last goal.
7. Robot1's no-next-dispatch period is why terminal latency alone explains only 35.55% of aggregate avoidable idle.

## Unproven hypotheses

- Whether serialized ComputePathToPose throughput, candidate-set churn, lower-bound publication cadence, or another evidence invalidation rule is the underlying reason for prolonged DNU/certificate deferral.
- Whether robot1 could safely have received a different task during each traffic wait without violating the protected traffic and certificate semantics.
- Whether the observed assignment imbalance is avoidable without changing the frozen cost-only policy.

No source correction is justified by this read-only artifact audit alone.

## Final verdict

`PIPELINE_LATENCY_ROOT_CAUSE_IDENTIFIED`
