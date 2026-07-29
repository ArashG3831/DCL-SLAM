# Cooperative experiment logging

`/cooperative_experiment_logger` is a centralized **passive evaluation tool**, not a coordinator. It has only subscriptions and wall/ROS timers. It has no publisher, action client, service client, control callback, or lifecycle dependency. Removing or crashing it cannot change candidate selection, claims, Nav2, maps, filtering, or robot motion. The observed launch includes the committed cooperative launch without modifying it.

## Live interfaces

The 2026-07-29 preflight confirmed `/robotN/odom` (`nav_msgs/Odometry`), `/robotN/map`, `/robotN/shared_map`, local/global `costmap` (`nav_msgs/OccupancyGrid`), `/robotN/scan_d500_fixed` and `/robotN/scan_d500_slam` (`sensor_msgs/LaserScan`), `/robotN/frontier_candidates` (`FrontierCandidateArray`), `/cslam/robotN/local_map` (`PeerMap`), `/cslam/robotN/exploration_claim` (`ExplorationClaim`), `/robotN/plan` (`nav_msgs/Path`), `/robotN/cmd_vel_nav` (`Twist`), final `/robotN/cmd_vel` (`TwistStamped`), NavigateToPose feedback/status, and `/rosout` (`rcl_interfaces/Log`). Maps/costmaps and peer maps are reliable transient-local; candidates and claims are reliable volatile (production depths 1 and 10); sensor subscriptions are best-effort-compatible. Frames are `robotN/odom`, `robotN/map`, robot base frames, and fused `shared_map`; the fixed map alignments are `(0,0,0)` and `(-0.299999998712,-0.000027796077,-3.1415)`.

## Files and timestamps

Each collision-safe `results/<run_id>/` contains `run_manifest.json`, `events.jsonl`, two robot CSV files, `coverage.csv`, `topic_health.csv`, deduplicated `warnings.jsonl`, atomic `summary.json`, and `README.txt`. Every event/sample contains the run ID, UTC ISO-8601 wall time, ROS seconds/nanoseconds, process-monotonic elapsed seconds, source/robot where applicable, and one process-wide increasing sequence. Message stamps are retained when present; monotonic time drives ordering, rates, freshness, windows, and durations. Non-finite JSON values become `null`.

Event schema version `1.0.0` has common fields plus only observed optional values. Candidate batches, claim state transitions, inferred arbitration/dispatch, action feedback/recoveries, navigation terminal claims, topic edges, detector edges, and warning anomalies are events. Heartbeats and feedback samples are not events. A transition to `NAVIGATING` is documented as the coordinator-supported evidence for goal send/accept; the claim protocol does not expose the action request UUID, explicit non-conflicting arbitration rationale, or Nav2 result payload. The logger therefore does not invent them. `state_reason` is retained verbatim and failure classification prefers its structured coordinator meaning; unknowns remain `UNKNOWN_NAV2_FAILURE`.

## Derived metrics

No-progress requires an active accepted goal and a complete 10 s window, less than 0.03 m improvement in minimum feedback distance remaining, and less than 0.02 m endpoint displacement. Stuck requires an active non-terminal goal, a complete 6 s window where every sample commands either at least 0.02 m/s linear or 0.15 rad/s angular motion, and less than 0.015 m displacement. Near-goal states are excluded. Oscillation requires an active goal, a complete 10 s window, at least four angular-command sign changes above 0.05 rad/s, less than 0.04 m displacement, and less than 0.03 m feedback improvement. Events are emitted once on entry and once on recovery. These are conservative diagnoses and never trigger control.

Coverage counts all occupancy values `>=0`; free is `[0,49]`, occupied is `>=50`, and unknown is `<0`. Area is known cells times resolution squared. For duplicated sensing, known local-map cell centers are rotated by map origin, transformed by the committed fixed local-to-shared transform, and floored onto a 0.01 m shared grid. The first observer is retained permanently; another robot within 2 s is simultaneous, otherwise later duplicate. Unknown never overwrites history. The fraction is cells known by both divided by the union. This estimates sensing duplication; rounding, resolution, SLAM corrections, and observation angle limit physical interpretation.

Trajectory positions are floored into 0.05 m shared-frame bins. Cross-robot overlap is intersection over union; same-robot travel in an already visited bin is repeated distance. Initial 0.15 m neighborhoods are excluded. Frontier equivalence uses the production centroid tolerance (0.15 m) OR expanded bounding-box overlap (0.05 m margin); goal duplication uses 0.15 m Euclidean tolerance.

Topic health uses a rolling 10 s observed-rate window and configurable monotonic ages. Stale and recovered events occur only on edges. Console output is immediate for lifecycle, claims, arbitration, navigation, new/recovered anomalies and errors, plus at most one five-second line per robot. `/rosout` WARN/ERROR keys are node, severity, and conservatively normalized message; only the first occurrence becomes an event and final counts retain first/last time.

Buffered files flush every five seconds and at shutdown; manifest/summary use same-directory atomic replacement. Write failures are throttled and do not propagate to robot control. SIGINT writes a clean final summary; unexpected exceptions attempt an interrupted summary. Rosbag is deliberately not required or enabled.

Subscription and timer entry points have named exception boundaries. A malformed
diagnostic input or optional metric emits a throttled `LOGGER_INTERNAL_ERROR`,
increments its subsystem-specific count, and leaves unrelated observation
running. Internal-error reporting has a recursion guard. Initialization failures
for the output directory remain fatal because the node cannot provide useful
logging without its required files. Finalization has a one-way guard, cancels
observer timers, blocks late writes, preserves each robot's last terminal or
active state, atomically updates summary/manifest, and safely ignores repeated
calls.

Occupancy data is converted to a compact NumPy view once for each latest map
object. Counts and checksums are cached; known-cell counting and coordinate
transforms are vectorized; local-map attribution runs only once per new map;
and first-observer/duplicate totals are maintained incrementally. Sampling
rates and all coverage definitions above are unchanged.

Logger CPU is measured from `/proc/self/stat` for the logger PID only, sampled
once per second after a ten-second warm-up. Percentages are deltas of user plus
system CPU time divided by monotonic wall time, normalized so one fully used
CPU core is 100%. The summary records PID, interval, warm-up, sample count,
mean, median, p95, and peak. RSS is sampled from `/proc/self/statm`; neither
measurement includes the launch process, Webots, or other ROS processes.

The node uses Jazzy's event-driven rclpy executor. With this observer's many
passive subscriptions, the default single-threaded executor repeatedly rebuilt
and scanned a large wait set for every incoming message. The event executor
preserves the same mutually exclusive callback behavior and callback bodies
while avoiding that measured scheduling overhead.
