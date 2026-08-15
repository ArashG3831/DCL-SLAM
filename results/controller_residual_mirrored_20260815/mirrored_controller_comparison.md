# Clockwise/right-turn RPP residual validation

Diagnostic-only one-robot fixed-route comparison. Production parameters and cooperative architecture were unchanged. The original validated route was reflected across the x axis: `(x,y,yaw) -> (x,-y,-yaw)`.

## Route equivalence

- Original geometric route length: **16.925950541 m**.
- Mirrored geometric route length: **16.925950541 m**.
- Difference: **0 m**.
- Original path curvature signs: 84 positive, 0 negative arc increments.
- Mirrored path curvature signs: 0 positive, 84 negative arc increments.
- The command-level signed-curvature telemetry confirms the original high-curvature samples are predominantly positive (CCW) and mirrored samples negative (CW).

## Valid repetitions

| group | repetitions | high-curvature samples | distance (m) | high-k speed (m/s) | lateral residual/m | yaw residual/m | GT→odom RMSE (m) |
|---|---:|---:|---:|---:|---:|---:|---:|
| CCW DWB | 3 | 3975 | 7.500 | 0.09418 | 0.003460 | 1.490269 | 0.02508 |
| CW DWB | 3 | 3953 | 7.690 | 0.09643 | 0.003253 | 1.287728 | 0.02835 |
| CCW RPP | 3 | 7361 | 8.040 | 0.05455 | 0.002053 | 1.796779 | 0.00898 |
| CW RPP | 3 | 7374 | 8.066 | 0.05458 | 0.001926 | 1.623716 | 0.00924 |

## Individual mirrored repetitions

| controller | run | high-k distance (m) | speed (m/s) | lateral residual/m | yaw residual/m |
|---|---|---:|---:|---:|---:|
| DWB | controller_residual_mirrored_dwb_rep01_rt | 2.596 | 0.09632 | 0.003305 | 1.269489 |
| DWB | controller_residual_mirrored_dwb_rep02_rt | 2.541 | 0.09749 | 0.003571 | 1.551343 |
| DWB | controller_residual_mirrored_dwb_rep03_rt | 2.553 | 0.09550 | 0.002884 | 1.043893 |
| RPP | controller_residual_mirrored_rpp_rep01_rt | 2.710 | 0.05489 | 0.001875 | 1.540285 |
| RPP | controller_residual_mirrored_rpp_rep02_rt | 2.657 | 0.05426 | 0.002204 | 1.970509 |
| RPP | controller_residual_mirrored_rpp_rep03_rt | 2.699 | 0.05460 | 0.001704 | 1.366051 |

## Mirrored route: DWB versus RPP

- High-curvature speed: 0.09643 → 0.05458 m/s (**43.4% reduction**).
- High-curvature lateral residual/m: 0.003253 → 0.001926 (**40.8% reduction**).
- High-curvature yaw residual/m: 1.287728 → 1.623716 (**+26.1% change**).
The lateral reduction is present in all three mirrored DWB/RPP repetition pairs.

## Turn-direction symmetry

| controller | CCW lateral residual/m | CW lateral residual/m | CW versus CCW | CCW yaw residual/m | CW yaw residual/m |
|---|---:|---:|---:|---:|---:|
| DWB | 0.003460 | 0.003253 | -6.0% | 1.490269 | 1.287728 |
| RPP | 0.002053 | 0.001926 | -6.2% | 1.796779 | 1.623716 |

Both controllers show approximately 6% lower lateral residual on the mirrored clockwise route than on the original counter-clockwise route; this is a small directional difference relative to the DWB→RPP reduction. RPP therefore retains its lateral benefit in both curvature signs. RPP yaw residual increases on both routes rather than improving.

## Validity and limitations

- Three valid realtime DWB and three valid realtime RPP mirrored-route repetitions completed.
- The exact prior controller YAML snapshots and ros2_control chain were reused; contact identity remains unresolved.
- This tests one robot and fixed paths only. It is not a cooperative or production-controller validation.
- No production YAML files were modified.

## Conclusion

RPP reduces the dominant lateral wheel-implied-versus-Webots-chassis residual during clockwise/right-turn high-curvature motion by approximately 40.8%, with consistent direction across repetitions. The effect is therefore approximately symmetric with respect to turn direction. Yaw residual per metre increases under RPP and remains a separate tradeoff.

RPP_BIDIRECTIONAL_CURVATURE_REDUCTION_SUPPORTED
