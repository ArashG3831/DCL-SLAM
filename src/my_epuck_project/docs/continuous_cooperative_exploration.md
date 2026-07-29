# Continuous decentralized cooperative exploration

Each namespaced coordinator consumes only its robot's reachable frontier batch
and shared map, the configured peer's claim and exploration status, and its own
`NavigateToPose` action. It publishes only its own claims, status, and
observation events. It has no peer action client, peer candidate subscription,
map/costmap writer, `ComputePathToPose` client, or velocity publisher.

## Modes and cycle lifecycle

`operating_mode` defaults to `single_goal`. The committed single-goal launch
also explicitly sets `one_goal_only=true`; a process can accept at most one
goal. The continuous observed launch explicitly sets `operating_mode=continuous`
and `one_goal_only=false`.

Continuous cycles follow:

`WAITING_FOR_INPUTS -> IDLE/SELECTING_NEXT -> ARBITRATING -> NAVIGATING ->
TERMINAL_BROADCAST -> COOLDOWN -> SELECTING_NEXT`.

Every proposal increments the process-lifetime claim ID and cycle number.
Every publication increments its session's message revision. Every dispatched
goal captures a unique generation and claim ID; response, feedback, result, and
cancel callbacks must match both and the expected state.

After terminal broadcast and the bounded cooldown, selection requires a
candidate callback received after the terminal cycle. An unchanged positive map
revision is acceptable only on that new batch: the generator republishes at
0.5 Hz, so this avoids indefinite waiting while still providing a post-motion
refresh. A disappearing candidate can release an uncommitted proposal, but
never preempts accepted navigation.

## Claims, arbitration, and status

Claims and statuses use reliable, volatile KeepLast(10). Both contain source
robot ID, a random process UUID, a monotonic revision, and bounded TTL. TTL
expiry is measured from receiver-local monotonic receipt time. A new UUID is
accepted only after the active session expires; the old UUID is retired and
delayed packets are rejected.

Spatial conflicts use the established frontier ID/centroid/bounds equivalence.
Lower frozen path cost wins outside `path_cost_tie_tolerance_m`; differences
inside the tolerance use robot ID. Accepted goals remain committed despite
ranking or candidate changes. If delayed messages produce two accepted
equivalent goals, both coordinators apply the same frozen arbitration; the
loser cancels once and receives a short spatial arbitration cooldown.

Exploration status is peer-to-peer liveness and completion evidence only. It
does not allocate goals or command a peer. States are `STARTING`, `ACTIVE`,
`NAVIGATING`, `NO_ELIGIBLE_CANDIDATES`, `COMPLETE`, `STOPPED`, and `ERROR`.

## Suppression

Successful regions receive a 25-second cooldown and additionally require a
newer candidate map revision before reuse. Local failures use bounded spatial
backoff parameters: 20 seconds for the first failure, 60 seconds for the second,
and 180 seconds thereafter. The configured maximum is always enforced.
Arbitration loss is not a navigation failure. A peer failure creates only the
short configured caution cooldown. Storage is bounded and expiry is observable.

An aborted or otherwise non-successful Nav2 action with result error code zero
is `UNKNOWN_NAV2_FAILURE`; action terminal status, error code, message, and
coordinator category remain separate in the terminal reason. Only a
coordinator-measured goal timeout is classified as `NAVIGATION_TIMEOUT`.

## Conservative completion

Local exhaustion requires no active goal, fresh positive-revision candidates,
a fresh shared map, no eligible frontier, the no-candidate grace, and stable
known-cell coverage over the configured window. Cooperative completion then
requires a fresh exhausted/complete peer status, no fresh reserving peer claim,
neither peer navigating, and the consensus grace. Silence cannot complete a
mission. A new candidate cancels exhaustion/consensus and resumes selection.
Final coordinators keep the stack alive and issue no further goals.
