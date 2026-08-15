# Simulation D500 free-space policy

The authoritative simulation pipeline is:

```text
raw Webots scan
-> fixed scan
-> finite teammate-return masking
-> natural +inf completion
-> SLAM
```

Webots publishes a natural no-return as `+inf`. Karto's installed Jazzy path
does not retain infinity as a usable ray, so the simulation-only SLAM branch
converts that value to the finite free-space cap `11.98 m`. Nav2 costmaps and
Collision Monitor continue to consume `scan_d500_fixed`; they never consume
the teammate-filtered or completed scan.

Teammate masking is an observation test, not a silhouette deletion. The
filter computes the first ray intersection with the teammate's circular
Pi-puck footprint, but classifies a hit only when the original reading is finite,
strictly between `range_min` and `range_max`, and within `0.005 m` of that
first surface. A predicted intersection without a finite return is not a
detection. Walls before the first surface, returns deeper in the circle chord,
and returns behind the teammate remain unchanged.

A finite teammate hit is changed to NaN before completion. Completion does not
alter NaN, so the hit remains unknown to SLAM and cannot clear through the
teammate. Conversely, an original `+inf` is never classified as a teammate hit;
it remains eligible for conversion to the free-space cap even when its beam
crosses the predicted teammate circle.

The relative pose is independent of SLAM and uses this exact-time composition:

```text
own_lidar <- own_odom <- peer_odom <- peer_base
```

`own_odom <- peer_odom` is the known initial transform from the validated
Webots world metadata. A FIFO holds at most four scans while the exact
odometry transforms arrive, retries every `0.02 s`, and drops a scan after
`0.20 s`. No unfiltered degraded publication exists. Physical filtering stays
disabled until a trustworthy physical relative-pose source replaces the
simulation's fixed initial odometry alignment.

The D500 `LaserScan.range_max` remains `12.0 m`, while SLAM Toolbox
`max_laser_range` is `11.98 m`. The cap is derived as:

```text
12.0 - max(2 * 1e-6 Karto tolerance, 2 * 0.01 m map cells, 0.01 m margin)
= 11.98 m
```

Karto ray-traces readings at or above the threshold after clipping their
endpoint to the threshold, but marks an endpoint occupied only when the
reading is below `threshold - 1e-6`. The completed cap is therefore a
free-only ray and cannot create an occupied cap ring. Finite obstacle returns
below the cap remain occupied endpoints. NaN, negative infinity, zero,
negative, malformed, and exact `range_max` readings remain ignored.
