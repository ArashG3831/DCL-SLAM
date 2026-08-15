# Offline RPP production decision

## Dataset and validity

Six retained bounded 600 s cooperative missions were analyzed: three DWB and three frozen-RPP. The DWB startup retry was excluded. RPP trial 3 reached normal timeout shutdown despite its runner crash label and was retained as valid bounded evidence.

## Ground-truth geometry and transforms

The saved `epuck_d500_two_world_large.wbt` was parsed into the RectangleArena perimeter and all 23 lidar-plane SolidBox rectangle boundaries. Robot/camera/light/diagnostic objects were excluded. Occupancy cell centers use map origin translation and yaw, then the saved `shared_map <- robotN/map` transform and the WORLD_DERIVED robot1-initial-to-world transform. Shared replicas are not offset a second time.

## Direct map accuracy

| map | DWB mean m | RPP mean m | DWB p95 m | RPP p95 m | DWB off-area >0.15 m² | RPP off-area >0.15 m² |
|---|---:|---:|---:|---:|---:|---:|
| R1 local | 0.417 | 0.235 | 1.552 | 0.693 | 5.634 | 4.178 |
| R2 local | 0.323 | 0.107 | 1.102 | 0.343 | 4.494 | 2.208 |
| R1 shared | 0.411 | 0.194 | 1.628 | 0.620 | 10.061 | 6.287 |
| R2 shared | 0.410 | 0.194 | 1.625 | 0.620 | 10.039 | 6.282 |

RPP is closer to the parsed static geometry for all four map products. This is direct geometric accuracy, not merely repeated-run consistency. Boundary samples are reported in the JSON; unknown boundary samples are not counted as false-free. Exact free-erosion interpretation remains limited by partial observability.

## Fusion and shared-replica consistency

The saved within-trial shared replicas and source-union comparisons are in `shared_replica_metrics.json` and `fusion_fidelity.json`. Fusion remains high-fidelity: the transformed source-local union and fused occupied cells have near-unity agreement in the retained artifacts; RPP's improved shared maps therefore reflect cleaner source evidence rather than a new fusion artifact.

## Normalized throughput

| metric | DWB mean | RPP mean |
|---|---:|---:|
| known-cell gain / combined GT metre | 2953.0 | 3228.7 |
| known-cell gain / 100 s | 37849.7 | 35848.4 |
| mean known cells over 0-600 s | 135450.6 | 146181.5 |
| successful goals / 100 s | 1.8 | 2.3 |

RPP gains approximately 9% more known cells per combined metre and has a higher mean known-cell AUC. Its raw known-cell gain per 100 s is approximately 5% lower, while it is ahead at 120–480 s and approximately tied at 600 s. The controller is therefore not a throughput collapse; it trades some instantaneous speed for better per-metre mapping and fewer failed goals.

## Recovery semantics

The logger's `RECOVERY_COUNT_CHANGED` event is a transition in NavigateToPose feedback `number_of_recoveries`; it is not a typed Spin/BackUp/ClearCostmap event. The saved data cannot identify the individual behavior plugin or recovery duration. Reconstructing terminal-goal feedback counts shows RPP recoveries are concentrated in failed goals: successful goals averaged 0.05 recovery actions, while the seven failed RPP goals averaged 14.86; DWB successful goals averaged 0.09 and its 18 failed goals averaged 2.39. Thus RPP has fewer failed goals and fewer successful goals needing recovery, but a small number of RPP failures enter much deeper recovery churn. This explains the higher recovery count without treating it as benign in every case.

## Odometry and navigation

RPP's GT-to-odom advantage remains large after distance normalization. It also has 42 successes versus 33 for DWB and 7 failures versus 18. No RPP run shows a hidden catastrophic mission failure; trial-to-trial variation is lower for RPP trajectory error than DWB.

| robot | DWB RMSE / 10 m | RPP RMSE / 10 m | DWB final error / distance | RPP final error / distance |
|---|---:|---:|---:|---:|
| robot1 | 0.139 | 0.104 | 0.0357 | 0.0144 |
| robot2 | 0.205 | 0.038 | 0.0473 | 0.0068 |

Mean transformed-source-union versus fused occupied IoU is DWB 0.986 and RPP 0.999; fusion remains faithful in both campaigns.

## Plain-language answers

**Q1. Are RPP maps closer to Webots geometry?** Yes, for the parsed static boundary reference, in all local and shared products.
**Q2. Is normalized exploration acceptable?** Yes. RPP is better per metre and in coverage AUC, approximately tied at 600 s, though raw cells/second is slightly lower.
**Q3. Why more recoveries?** The event is a feedback recovery-count change, not a type. RPP's extra count is concentrated in a few failed goals with many repeated recovery actions.
**Q4. Is the navigation-success problem improved?** Yes: 42 versus 33 successes and 7 versus 18 failures across the campaigns.
**Q5. Should RPP replace DWB?** Yes, the combined controlled, direct-map, normalized-throughput, odometry, and navigation evidence supports promoting the frozen candidate. Keep recovery-churn monitoring as a validation guardrail.

## Artifacts

Analysis root: `results/rpp_cooperative_validation_20260815/offline_decision`
Production YAML and runtime configuration remain unchanged pending an explicit promotion task.

RPP_PRODUCTION_PROMOTION_SUPPORTED
