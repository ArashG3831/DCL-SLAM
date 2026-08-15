# Offline autonomous odometry-drift forensic analysis

Artifact: `forensic_large_20260814_run` (HEAD recorded by the run as
`84586dab1056810a43101671ac7f65fa985b7221`). No Webots process was started and
no project source/configuration was changed for this analysis.

## Evidence and alignment

The run persisted Supervisor truth at 0.1 s, raw odometry, 1 Hz logger
timeseries, structured events, TF samples, and warnings. It did **not** persist
wheel/joint states, a raw `/cmd_vel` stream, a rosbag, or collision/contact
signals.

All calculations use simulation time: Supervisor `sim_time_s` and odometry
`header_stamp`. Supervisor x/y/yaw is converted once to each robot's own
initial frame from its recorded world start pose. At the first common sample,
one rigid transform `T = GT(t0) * inverse(odom(t0))` is applied to odometry;
there is no later re-alignment. x/y and unwrapped yaw are linearly
interpolated. Repeated odometry header stamps are de-duplicated by retaining
the last received sample for the timestamp.

The source run's published report gives Robot 1 GT→odom RMSE 0.525 m and final
1.320 m. A corrected dense raw-odometry recomputation gives RMSE 0.445 m and
final 1.332 m; the 1 Hz logger pose series reproduces RMSE 0.529 m. The final
error agrees, but the RMSE discrepancy means the old aggregate RMSE depends on
sampling/series selection and should not be quoted without this qualification.

## Recomputed trajectory summary

| robot | GT distance | GT→odom translation RMSE | final translation error | yaw RMSE | final yaw error |
|---|---:|---:|---:|---:|---:|
| robot1 | 59.03 m | 0.445 m (0.529 m at 1 Hz) | 1.332 m | 0.0338 rad | 0.0632 rad |
| robot2 | 27.74 m | 0.233 m | 0.337 m | 0.0362 rad | 0.0336 rad |

Robot 2 stopped moving at about 459 s, which limits its final distance and
explains why its later error is flat.

## Robot 1 error growth by phase

| interval | error change | GT distance change | motion interpretation |
|---|---:|---:|---|
| 0–130 s | 0.000→0.157 m | 0→13.2 m | small early accumulation during ordinary navigation and turns |
| 130–220 s | 0.157→0.182 m | 13.2→22.8 m | slow growth |
| 220–380 s | 0.182→0.063 m | 22.8→35.4 m | error partially cancels; no monotone distance-only law |
| 419–425 s | 0.063→0.066 m; yaw error 0.016→0.050 rad | 0.111 m | in-place/tight turn and goal-start transient; GT yaw −2.055 rad, odom yaw −2.012 rad |
| 425–445 s | 0.066→0.198 m | 2.600 m | forward travel after the yaw offset; path disagreement only −3.6 mm |
| 445–470 s | 0.198→0.345 m | 3.009 m | continued navigation/turning; path disagreement −4.0 mm |
| 470–520 s | 0.345→0.444 m | 2.137 m | slower travel; persistent offset remains |
| 520–580 s | 0.444→0.797 m | 6.620 m | largest sustained late growth during mostly forward travel |
| 580–620 s | 0.802→1.002 m | 3.275 m | continued growth |
| 620–660 s | 1.002→1.158 m | 2.617 m | continued growth |
| 660–708 s | 1.158→1.332 m | 3.180 m | final growth; late turn at 702–705 s |

The main identifiable mechanism is therefore **an orientation disagreement
introduced during a tight/start turn, followed by cross-track position-error
growth during subsequent translation**. Distance alone is insufficient: error
falls from 0.182 m to 0.063 m over 22.8–35.4 m, then rises sharply after the
419–425 s turn sequence.

## Short-window motion disagreement

The direct 419–425 s comparison is the strongest diagnostic:

* GT travelled 0.111 m; odometry travelled 0.116 m (only +4.4 mm).
* GT yaw changed −2.055 rad; odometry changed −2.012 rad (−0.043 rad
  disagreement, about 2.5 degrees).
* GT→odom yaw error increased by about 0.035 rad.

Immediately afterward, 425–445 s had GT distance 2.600 m versus odometry
2.596 m and yaw changes −0.654 versus −0.651 rad, yet position error grew by
0.132 m. This is consistent with a small heading offset being integrated into
position, not with the wheels claiming substantial motion while the body made
none.

Across the full run, Robot 1 cumulative odom path was 58.975 m versus 59.026 m
GT (−0.051 m); Robot 2 was 27.732 m versus 27.740 m (−0.008 m). The maximum
absolute cumulative path disagreement was 0.054 m for Robot 1 and 0.015 m for
Robot 2. The largest five-second distance disagreement was 10.9 mm for Robot 1
and 4.4 mm for Robot 2. No short window showed metres of odometric translation
with negligible Supervisor motion.

## Correlation with available runtime evidence

* Robot 1's 419–425 s sequence begins immediately after a goal dispatch. The
  1 Hz command snapshots show zero linear velocity and approximately −0.35
  rad/s angular command, followed by acceleration to 0.13 m/s and further
  turns. A single `NO_PROGRESS_STARTED` event occurs at 420 s, then clears at
  425 s. There is no Robot 1 recovery, stuck, or corner-trap event in this
  interval.
* Robot 1 has six later `NAVIGATION_FAILED` events (about 506, 558, 585, 618,
  650, 687 s), but each ±5 s window has only roughly 0.9–3.5 mm path-distance
  disagreement and small yaw disagreement. They coincide with continuing
  motion and do not look like physical obstruction events. They may reflect
  goal/planner churn, but causality is not established.
* Robot 1 has no persisted wheel velocities, wheel encoder integration,
  collision/contact flags, DWB critic values, or high-rate command stream. Slip,
  scrubbing, or contact therefore cannot be proven or excluded directly.
* Robot 1's error-growth rate correlates more with being in high linear motion
  than with instantaneous angular rate after the 419–425 s trigger (integer
  second samples: error-increment correlation ≈0.42 with GT speed, ≈−0.01
  with |GT angular rate|). This is expected if translation amplifies an
  existing heading offset; it is not evidence that speed independently caused
  the initial offset.
* Robot 2 experienced substantially more no-progress/stuck/recovery behavior
  (33 no-progress starts, 7 stuck starts, 13 recovery-count changes) but ended
  at 0.337 m error after 27.74 m and stopped at about 459 s. Robot 1 had one
  no-progress start, no stuck starts, no recoveries, but travelled 59.03 m.
  Thus the robot difference is primarily trajectory/distance and one
  Robot-1-specific late heading discrepancy, not a generic recovery mechanism.
* Supervisor separation was at least 2.499 m (initial) and about 4.38 m at the
  Robot 1 419–425 s trigger, increasing thereafter. Proximity/interference is
  not supported by the persisted geometry. No contact sensor was recorded.
* Four TF warnings and repeated map-fusion footprint extrapolation warnings
  exist, but none is Robot-1-specific at the trigger and no persisted TF jump
  coincides with the 0.043 rad odometry/GT turn disagreement. They remain a
  possible system-load confounder, not a demonstrated cause.

## Largest-interval contribution

Using non-overlapping five-second windows, Robot 1's largest ten positive error
increments sum to 0.424 m, 11.8% of the total positive variation (3.591 m); the
largest five sum to 0.216 m, 6.0%. This means the final 1.332 m is not a single
catastrophic jump: it is mostly accumulated after the late heading-offset
trigger. The specific sustained phases 420–470 s (+0.283 m), 520–580 s
(+0.353 m), 580–620 s (+0.200 m), and 620–708 s (+0.330 m) account for most of
the net late growth, with some earlier error cancellation.

## Replayability

The run has 708 one-Hz `robot1_timeseries.csv` rows containing sampled
`commanded_linear_mps`, `commanded_angular_radps`, navigation state, goal
activity, and distance fields. It does **not** have the raw command topic at
controller frequency, wheel states, or a bag. This is enough to reconstruct a
coarse piecewise-constant approximation, not the exact autonomous command
sequence deterministically.

A future replay should use the same world/start pose and a single robot, feed a
timestamped high-rate recording of the final command-chain topic (preferably
the last command before the Webots controller, with at least the controller
period), and run the same external Supervisor observer. Supervisor must remain
read-only; it must not publish TF, odometry, goals, or corrections. The replay
should compare the same SE(2) error and short-window distance/yaw disagreement
metrics.

## Generated artifacts

* `offline_drift_summary.json`
* `robot1_drift_samples.csv`, `robot2_drift_samples.csv`
* `robot1_drift_intervals.csv`, `robot2_drift_intervals.csv`
* `robot1_short_window_disagreement.csv`, `robot2_short_window_disagreement.csv`
* `robot1_drift_analysis.png`, `robot2_drift_analysis.png`
* `robot1_drift_events.png`, `robot2_drift_events.png`

## Conclusion

The evidence localizes the onset to a Robot 1 tight/start-turn sequence around
419–425 s and shows subsequent translation amplifying a small heading
disagreement. It rules out travelled distance alone and does not show gross
wheel-vs-ground motion disagreement. However, because wheel states, contacts,
and high-rate commands were not persisted, the physical/controller-level cause
of the turn disagreement remains unproven.
