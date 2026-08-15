# Controlled DWB versus RPP wheel–chassis residual test

Diagnostic-only one-robot fixed-route comparison. Production Nav2, SLAM, wheel geometry, world, and cooperative configuration were not changed.

## 1. Software and configuration

- ROS 2 Jazzy; installed Nav2 DWB and Regulated Pure Pursuit packages: 1.3.10.
- Installed `webots_ros2_driver` package: 2025.0.1 (the external Windows Webots executable was not modified).
- Wheel model: `r=0.020 m`, `L_eff=0.052*1.095=0.05694 m`.
- Fixed route controller: `nav2_msgs/action/FollowPath` on `/robot1/follow_path`.
- Final command chain and wheel feedback were logged; Supervisor GT was external/read-only.

**DWB:** `dwb_core::DWBLocalPlanner`; max vx 0.13, max theta 0.35, acc/decel x +0.20/-0.20, 6 vx samples, 21 theta samples, sim_time 1.7 s, unchanged production critics.
**RPP:** `nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController`; desired 0.13, lookahead 0.20 (min 0.12, max 0.30, time 1.5), velocity-scaled lookahead, regulated scaling enabled, min radius 0.50 m, min speed 0.026, rotate-to-heading enabled at 0.785 rad, rotate speed 0.35, max angular accel 0.20, no reversing, collision detection and stateful enabled.

## 2. Route and validity

The route definition was generated once and replayed identically. It contains 14 FollowPath segments (one full rounded-rectangle lap plus the first five segments of a second lap), approximately 16.88 m, with straight and medium/tight rounded arcs. The implemented rounded rectangle uses counter-clockwise (left) arcs; right-turn symmetry was not separately sampled in this campaign and is a follow-up limitation. Route signatures (labels and all path points) compare equal across all valid runs.

| controller | repetition | valid | route time (sim s) | route distance (m) | GT→odom RMSE (m) | final error (m) | yaw RMSE (rad) | final yaw (rad) |
|---|---:|---|---:|---:|---:|---:|---:|---:|
| DWB | controller_residual_dwb_rep01_rt | True | 159.6 | 16.8781 | 0.025631 | 0.0515263 | 0.0171342 | -0.023831 |
| DWB | controller_residual_dwb_rep02_rt | True | 163.4 | 16.89 | 0.0254908 | 0.0566896 | 0.0168908 | -0.0269339 |
| DWB | controller_residual_dwb_rep03_rt | True | 161.4 | 16.8348 | 0.0241035 | 0.0532767 | 0.0158098 | -0.0234945 |
| RPP | controller_residual_rpp_rep01_rt | True | 202.4 | 16.8731 | 0.00918904 | 0.0166725 | 0.00606042 | -0.0068111 |
| RPP | controller_residual_rpp_rep02_rt | True | 200 | 16.8771 | 0.0086606 | 0.0176679 | 0.00554658 | -0.00937443 |
| RPP | controller_residual_rpp_rep03_rt | True | 202.2 | 16.8758 | 0.0090777 | 0.0176564 | 0.00594242 | -0.00900887 |

All six runs are valid route completions. Earlier fast-mode, shared-library, lifecycle, and incomplete-route attempts are excluded as infrastructure-invalid or route-failure and are not counted.

## 3. Curvature-conditioned primary results

| bin (1/m) | controller | distance mean±sd (m) | command vx mean±sd (m/s) | lateral residual/m mean±sd | yaw residual/m mean±sd |
|---|---|---:|---:|---:|---:|
| 0-0.5 | DWB | 13.1473±0.0436 | 0.120744±0.00229 | 4.28449e-05±1.03e-06 | 0.390252±0.0175 |
| 0-0.5 | RPP | 13.4039±0.0383 | 0.099508±0.00102 | 4.3441e-05±5.97e-06 | 0.36919±0.00821 |
| 0.5-2 | DWB | 1.01778±0.0127 | 0.0929657±0.00417 | 0.00126748±6.36e-05 | 0.778039±0.0368 |
| 0.5-2 | RPP | 0.791439±0.0827 | 0.107078±0.00204 | 0.00133468±0.000211 | 0.659433±0.109 |
| 2-5 | DWB | 2.50005±0.0277 | 0.0941905±0.001 | 0.00346018±0.000129 | 1.49006±0.115 |
| 2-5 | RPP | 2.67999±0.0439 | 0.0545501±0.000307 | 0.0020544±0.000305 | 1.79893±0.375 |
| 5-20 | DWB | 0.202415±0.032 | 0.0391937±0.00108 | 0.00354674±6.66e-05 | 3.28955±0.136 |
| 5-20 | RPP | 0±0 | n/a | n/a | n/a |

The high-curvature comparison is the 2–5 1/m bin (the RPP route had no 5–20 1/m samples, so that bin is not interpreted).

- Pooled 2–5 1/m command speed reduction: **42.1%** (0.09419 → 0.05455 m/s).
- Pooled 2–5 1/m absolute lateral residual per metre reduction: **40.7%** (0.003460 → 0.002053 m/m).
- Pooled 2–5 1/m absolute yaw residual per metre **increased 20.6%** (1.490269 → 1.796779 rad/m); RPP does not improve every residual component.
- The directional lateral-residual reduction occurs in all three DWB/RPP repetition pairs: RPP values 0.001703, 0.002251, 0.002209 m/m versus DWB 0.003350, 0.003429, 0.003602 m/m.

## 4. Motion categories and supporting results

| controller | category | distance (m) | lateral residual/m mean±sd across repetitions |
|---|---|---:|---:|
| DWB | STRAIGHT | see per-run summaries | 1.29514e-05±1.23e-06 |
| DWB | ARC | see per-run summaries | 0.00222666±8.15e-05 |
| RPP | STRAIGHT | see per-run summaries | 2.07353e-05±3.5e-06 |
| RPP | ARC | see per-run summaries | 0.00163821±0.000252 |

DWB arc residual is consistently much larger than straight residual. RPP reduces arc residual while retaining the same route geometry. RPP also spends more route distance in low-curvature motion and has lower low-curvature command speed (~0.0995 m/s pooled versus DWB ~0.1207 m/s); this throughput tradeoff is measured, not hidden.

## 5. Interpretation and limitations

- The measured quantity is the discrepancy between wheel-position-implied differential-drive motion and external Webots chassis motion. It is not proof of literal tire slip or a specific contact mechanism.
- No invasive contact instrumentation was required; contact identity remains unresolved. The route was open and collision-monitor chain was retained, but contact telemetry is explanatory rather than a validity gate.
- Initial GT/odom alignment used one SE(2) transform per run: `theta = yaw_odom(t0)-yaw_GT(t0)` with translation chosen to coincide initial positions. No later realignment was used.
- This test isolates controller-generated motion on one robot. It does not establish cooperative exploration, map-quality, or production-controller superiority.
- The local diagnostic costmap used the same controller/smoother/collision chain and an open-arena inflation-only costmap; no frontier/cooperative stack was launched.

## 6. Resource and file status

- Realtime headless runs were used; fast mode was rejected for validity because it produced CPU/load-sensitive route failures. One-robot realtime process samples remained materially lighter than the previous two-robot forensic campaign; no RViz, rosout collection, maps, or cooperative telemetry were enabled.
- Per-run artifacts include controller YAML snapshot, route definition/status, GT CSV, wheel/odom/command telemetry, reconstructed per-step residual CSV, and summary JSON.
- Diagnostic source additions: `fixed_follow_path_driver.py`, `controller_residual_test_launch.py`, `analyze_controller_residual.py`, `pool_controller_residual.py`, `write_controller_residual_report.py`; `setup.py` adds only the diagnostic driver entry point; `turn_motion_diagnostics.py` adds buffered flushing. Production YAML files were not edited.

## 7. Engineering conclusion

RPP reduced commanded speed in the empirically problematic curvature regime and reduced the dominant lateral wheel-implied-versus-Webots-chassis residual by approximately 41% in pooled 2–5 1/m motion, with the same lateral-residual direction in all three repetition pairs. The yaw-residual-per-metre component increased by approximately 21%, so this supports reduced lateral translational mismatch rather than improvement of every residual component. This supports a subsequent full cooperative validation, but does not by itself authorize changing production configuration.

RPP_CURVATURE_DRIFT_REDUCTION_SUPPORTED
