# Simulation D500 free-space policy

Webots publishes a natural no-return as `+inf`. Karto’s installed Jazzy path
does not retain infinity as a usable ray, so the simulation SLAM branch
normalizes only that value to the finite free-space cap `11.98 m`.

The D500 `LaserScan.range_max` remains `12.0 m`, which is Karto’s absolute
sensor maximum. `max_laser_range` is the Karto threshold and is set to the
same `11.98 m` cap. The cap is derived as:

`12.0 - max(2 * 1e-6 Karto tolerance, 2 * 0.01 m map cells, 0.01 m margin) = 11.98 m`.

Karto ray-traces readings at or above the threshold after clipping their
endpoint to the threshold, but marks an endpoint occupied only when the
reading is below `threshold - 1e-6`. Therefore the completed cap is a
free-only ray and cannot create an occupied cap ring. Finite obstacle returns
below the cap remain occupied endpoints. Finite returns above the cap and
below `12.0 m` are consistently clipped to the free-only threshold. NaN,
negative infinity, zero, negative, malformed, and absolute-range readings are
ignored by Karto.

The order is fixed: raw Webots scan → basic fixed scan → finite `+inf`
completion → teammate masking → SLAM. A teammate hit becomes NaN after
completion, so it remains unknown and cannot clear through the teammate. A
missing or stale exact transform drops the pending scan conservatively.

Nav2 costmaps, Collision Monitor, raw logging, and physical D500 processing
continue to consume the fixed scan. The completion controls are explicit:
simulation mode enables `simulation_free_space_completion`; physical mode
defaults `physical_free_space_completion` to false. The policy is not enabled
for physical no-returns because they can also represent optical loss or
dropout.
