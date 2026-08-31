# Final Runtime Reliability Report

## Scope and provenance

This bounded task did not run a final long campaign. The preserved failed artifacts remain in place and are explicitly classified below. The three formal short validations are the same-configuration runs `validation2_final`, `validation3_final`, and `validation5_final`; `validation1_final` and `validation4_final` reached cooperative operation but were excluded from the formal set because their evaluation-only fixed-anchor GT artifact lacked one pre-motion anchor.

## Validation 1 shared-stack failure

The preserved diagnostic at `results/final_18m_idle_validations/validation1/` reached handoff at 55.82 simulation seconds. The shared stack launch was requested, the child processes were spawned, lifecycle managers were present, and lifecycle STARTUP requests were logged at approximately 61.70 s and 64.18 s. No shared lifecycle ACTIVE or costmap barrier event followed; the phase manager repeatedly remained in `shared_nav2_waiting_for_lifecycle_manager`. The first causal divergence is therefore **STARTUP REQUEST SENT -> NO COMPLETED ACTIVATION RESPONSE**, with processes alive but inactive. There is no evidence that local teardown killed the shared stack, and no dependency path from the peer-active assignment fix to this startup path.

The bounded fix was to make the runner launch local Nav2 with `nav2_autostart:=false`, leaving the existing readiness probe as the single gated startup owner. This removes automatic lifecycle bringup racing the explicit request. It does not bypass the local-control safety boundary or shared costmap/TF gates.

## Webots no-clock failures

Preserved validations 2 and 3 from `final_18m_idle_validations` both timed out waiting for `/clock`, then recorded Webots exit code 1 during SIGINT cleanup. Validation 2 only initialized Robot 2's lidar controller; validation 3 only initialized Robot 1's. The common immediate cause is therefore an external Webots ROS controller stalling before initialization, which prevented Ros2Supervisor from producing advancing `/clock`. The lower-level controller/Webots error was not captured, so it is reported as unavailable rather than guessed. These failures occurred before meaningful algorithm execution and are not attributed to assignment, MRTSP, unknown-pose registration, or 18 m semantics.

The corrected harness uses the intended Webots driver and reliable SLAM wrapper provenance, unique ports/domains, stale-workspace filtering, explicit `nav2_autostart:=false`, and post-run orphan verification.

## Three genuine short validations

| run | both ready | handoff | shared active | costmap ready | first valid pair | first cooperative goal | handoff->active wall | handoff->goal wall | GT translation | GT yaw |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| fast_trial_20260831T025852Z | True | True | True | True | yes | True | 11.42 s | 12.89 s | 0.580 cm | 0.12215° |
| fast_trial_20260831T030251Z | True | True | True | True | yes | True | 13.10 s | 14.05 s | 0.744 cm | 0.14312° |
| fast_trial_20260831T031615Z | True | True | True | True | yes | True | 14.17 s | 14.85 s | 0.272 cm | 0.00656° |

All three selected runs had advancing `/clock`, mutual readiness, successful handoff, shared lifecycle/costmap activation, valid pair decisions, cooperative dispatch, no early unilateral goals, no first-pair TF invalidation, and valid evaluation-only GT with `estimator_input_connection=false`. Focused tests passed: **153 passed**. The fresh selected-package build passed.

The exact safe-independent-idle live case was not naturally exercised in these short maps. Live evidence did include active/IDLE and traffic-deferred cases, while the focused integration test `test_peer_activity_is_not_a_global_assignment_barrier` passed. Per the allowed fallback, this is reported as unit/integration proof rather than claimed as a natural live event.

## 0.4275-degree yaw follow-up

The 0.4274528° artifact is most consistent with **registration estimate variation**, not a GT anchor artifact: both fixed anchors were pre-motion with negligible displacement, translation error was only 0.284 cm, the GT yaw was effectively zero, and the accepted estimated yaw itself was 0.42745°. Nearby runs using the same fixed-anchor evaluator produced near-zero yaw. Matcher geometry was not modified.

## Preserved scope boundaries

- The 18 m removal remains in force; no replacement ordinary path cap was added.
- The peer-active assignment fix remains in force; a busy peer is not preempted.
- MRTSP/scoring, traffic, overlap logic, unknown-pose geometry, max speed, and frontier granularity were not modified.
- GT remains evaluation-only.
- No central coordinator was introduced.
- No full campaign was run.

## Final decision

**READY FOR OVERLAP AUDIT — RUNTIME STARTUP IS RELIABLE, COOPERATIVE DISPATCH SUCCEEDS IN THREE SHORT RUNS, AND IDLE-PEER CONCURRENCY IS VALIDATED**

The short runs establish operational reliability. They do not claim that stochastic Nav2 startup variance has vanished, and the lower-level cause of the two old pre-clock controller stalls remains unavailable in their preserved logs.
