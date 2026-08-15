# Professor demo cheatsheet — known-transform system

## What is a frontier?
A frontier is the boundary between the currently known free part of an occupancy map and adjacent unknown space. The frontier generator groups frontier cells into bounded local candidates and supplies an approach pose that Nav2 can try.

## From detections to tasks
Each robot publishes a bounded candidate snapshot. The peers exchange candidate geometry, approach poses, local feasibility, frontier samples, session/epoch freshness, and map fingerprints. Canonicalization merges detections representing the same physical frontier into one canonical task ID. That canonical task union is what the allocator sees.

## Why decentralized?
Both robots receive the same task/bid evidence and independently run the same deterministic calculation. They exchange snapshots, local bids, decisions, status, and failure evidence, then verify session, round, union-hash, bid-fingerprint, and decision-hash agreement. There is no central brain, and neither robot sends an action to the other.

## Why each robot computes only its own path cost
A robot’s Nav2 planner and costmap are local to that robot. Each robot therefore computes its own `ComputePathToPose` bid; the peer receives the bounded bid result and path samples, not a peer Nav2 action request.

## New assignment equation
```text
U_t = 1.0
C_i,t = clamp(L_i,t / 18.0 m, 0, 1)
S(i,t) = U_t - beta*C_i,t       beta = 1.0
```

`U_t` is the remaining utility of canonical task `t`. It starts equal for every eligible task. `L_i,t` is the Euclidean length in metres of robot `i`’s own Nav2 path. `C_i,t` is dimensionless because it is scaled by the existing 18 m admissible path ceiling. `beta=1` is the Burgard baseline; the 18 m normalization is our project adaptation because raw Nav2 metres and the paper’s travel-cost representation are not numerically identical.

## Redundancy reduction
After a robot/task is selected, each remaining task is compared to the selected physical frontier representative. Within the D500 range `R=11.98 m`, clear LOS gives `P(d)=1-d/R`, and utility is reduced by `P(d)`. Occupied grid cells above 50 block the reduction; unknown cells alone do not. This is the Burgard-inspired sensor-overlap adaptation. The old visible-frontier gain was boundary length in metres and is no longer production utility.

## Same task, one frontier, and failures
Canonical IDs are removed after selection, so the same physical frontier cannot be assigned twice. If only one canonical task exists, one robot receives it and the other can be IDLE. A failure is a typed navigation/feasibility event where the existing system supports it; failure memory and expiry gate robot/task eligibility. It is not a magic `-5` ranking penalty. An evolving occupancy map can invalidate the execution context even in a static physical world: walls are initially unknown, costmaps change, localization/odom is imperfect, and closed-loop Nav2 can abort after an initially feasible path.

## What the fresh evidence says
Three bounded known-transform forensic runs used RPP + Burgard beta=1. They captured Supervisor GT, odometry, local/shared maps, transforms, fusion evidence, coverage, allocator events, and Nav2 results. They produced 32 accepted goals, 29 successes, 1 terminal abort, zero cancellations, zero recovery-count changes, zero duplicate canonical dispatches, and 19/19 complete replicated decisions matching. One local preliminary decision was superseded before peer completion; it was not a disagreement.

## Traffic tonight
Traffic is a separate scheduling layer, not part of frontier utility. It is implemented and pure-tested but **experimental/deferred and disabled by default** because no doorway world exists yet. Do not claim doorway or physical bottleneck validation. When later enabled, the intended policy is pre-dispatch conflict scheduling: an already active robot keeps priority; otherwise lower ETA to the first conflict wins; robot ID is only the exact-tie breaker. The lower-priority new NavigateToPose goal waits and is not sent; after the winner terminates, stale work is discarded and a fresh allocation is requested. No zero-velocity injection or healthy-goal cancellation is used.

## Mapping and future work
Known initial relative transform remains enabled. Each robot keeps local SLAM, source-aware map fusion, local Nav2, and its own action client. Unknown initial relative pose, place recognition, map registration, pose-graph optimization, and ghost cleanup are future work.

## Text architecture
```text
Robot 1 local SLAM ----                        -> evidence exchange -> replicated shared maps
Robot 2 local SLAM ----/                               |
                                                       v
                                              frontier candidates
                                                       |
                                              canonical physical tasks
                                                       |
                                      local Nav2 feasibility / bids
                                                       |
                                      replicated U - beta*C allocator
                                                       |
                                                peer agreement
                                                       |
                                      local NavigateToPose on each robot
                                                       |
                                                     RPP/Nav2
```
