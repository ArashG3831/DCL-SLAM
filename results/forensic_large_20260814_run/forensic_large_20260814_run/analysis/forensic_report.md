# Forensic local-SLAM versus fusion report

Run: `forensic_large_20260814_run`  
Commit recorded by run: `84586dab1056810a43101671ac7f65fa985b7221`  
World SHA-256: `4caa7e7deba011ae88a0a952ac2b7a8680ad6dedc65e0610e03f6a2ea08c8415`  
Simulation time captured: 0.1–708.2 s; logger clean shutdown: `True`.  
The source world and production parameters were not edited. The diagnostic launch used a temporary runtime copy only to attach the read-only Supervisor observer.

## Evidence inventory

- Periodic compressed map snapshots: 129 (15.0 s requested; content changes only).
- Explicit final map files in this artifact are reconstructed from the last complete periodic snapshot because the run predates the logger final-writer correction. The correction is now in the working tree for future runs.
- Raw odometry: robot1 25842 rows; robot2 25630 rows.
- Supervisor ground truth: 14164 rows (0.1 s sampling, both robots).
- Transform samples: 288 rows; all six requested transform pairs had available samples.
- PeerMap records: 1397 accepted local-evidence records.

| robot | received | accepted | revision range | local_evidence_only |
|---|---:|---:|---|---|
| robot1 | 698 | 698 | 1–698 | ['True'] |
| robot2 | 699 | 699 | 1–699 | ['True'] |

## Alignment

The persisted production transforms were used. `shared_map→robot1/map` was `(0, 0, 0)` and `shared_map→robot2/map` was approximately `(0, -2.5 m, 0)`. Local occupied cell centres were transformed into the final shared grid at 0.03 m resolution. For trajectory comparison, Webots world coordinates were converted once into each robot's initial frame; odometry and map-frame poses were each anchored once at their first common sample. Supervisor data was never published to ROS or used by control, TF, SLAM, Nav2, or fusion.

## Occupancy comparison

- Robot 1 transformed local occupied cells: **14663**.
- Robot 2 transformed local occupied cells: **6100**.
- Source union/intersection: **19687 / 1076** cells; Jaccard **0.0547**. The low overlap is primarily different explored regions, not by itself a registration error.
- Robot 1 shared output: **19588** occupied cells; Robot 2 shared output: **19511**.
- Fused shared versus transformed-source union: only **10** extra occupied cells and **109** source-union cells absent from the selected fused output; shared-vs-union Jaccard was **0.9940** (robot1 replica) and **0.9886** (robot2 replica).
- Source-nearest disagreement distances were asymmetric because coverage differed: Robot-2 evidence to the nearest Robot-1 occupied cell had median **0.095 m**, 95th percentile **2.160 m**.

## Geometry against Webots reference

Reference geometry is the source world's SolidBox rectangles plus a thin RectangleArena wall proxy, transformed into shared-map coordinates. This is a geometric diagnostic, not a visibility-complete map-accuracy proof.

| evidence | mean distance (m) | median (m) | p95 (m) | >0.15 m ghost area (m²) |
|---|---:|---:|---:|---:|
| Robot 1 local | 0.107 | 0.027 | 0.547 | 2.358 |
| Robot 2 local | 0.059 | 0.008 | 0.252 | 0.815 |
| source union | 0.093 | 0.024 | 0.397 | 3.173 |
| fused shared | 0.099 | 0.035 | 0.405 | 3.543 |

The wall-thickness proxy was 0.06 m median and 0.12 m at the 95th percentile for both source union and fused output; this proxy is resolution/occupied-region based and should not be read as a metrology-grade wall width.

## Trajectory drift

| robot | GT→odom RMSE (m) | GT→odom yaw RMSE (rad) | final GT→odom (m / rad) | GT→SLAM RMSE (m) | GT→SLAM yaw RMSE (rad) | final GT→SLAM (m / rad) |
|---|---:|---:|---|---:|---:|---|
| robot1 | 0.525 | 0.037 | 1.320 / 0.066 | 0.541 | 0.055 | 1.190 / 0.060 |
| robot2 | 0.248 | 0.037 | 0.337 / 0.034 | 0.249 | 0.050 | 0.337 / 0.024 |

SLAM pose samples are the 48 persisted transform-query epochs; odometry comparisons use dense raw odometry interpolation. The close GT→SLAM and GT→odom values, together with `use_scan_matching=false` in production, are consistent with little active scan-based correction in this run.

## Conclusion

**LOCAL_SLAM_IS_PRIMARY_SOURCE_OF_UGLINESS**. Both local maps contain substantial off-reference occupied geometry, while source-aware fusion is nearly a set-union/max-evidence operation in this run: the fused output adds only 10 cells beyond the transformed source union and has >0.98 Jaccard agreement with each replica's source union. The low source-to-source overlap is dominated by different explored extents; it is not evidence of a large common-region transform mismatch. Fusion may expose/combine local errors, but it did not materially create the observed ugly geometry here.

## Artifacts

- `maps/robot1_map_final.npz`, `maps/robot2_map_final.npz`: last periodic local-map snapshots, explicitly marked reconstructed final.
- `maps/robot1_shared_map_final.npz`, `maps/robot2_shared_map_final.npz`: last periodic fused snapshots, explicitly marked reconstructed final.
- `analysis/transformed_local_maps_shared_grid.npz`
- `analysis/source_separated_overlay.png`
- `analysis/local_vs_fused_panels.png`
- `analysis/trajectory_and_tf.png`
- `analysis/forensic_metrics.json`
- `supervisor_ground_truth.csv`, `robot1_odom.csv`, `robot2_odom.csv`, `transforms.csv`, `peer_map_records.csv`
