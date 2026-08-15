# RPP production promotion — 2026-08-15

## Change

The active production FollowPath controller was changed from
`dwb_core::DWBLocalPlanner` to the frozen validated
`nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController` in:

- `src/my_epuck_project/resource/nav2_robot1_shared_map.yaml`
- `src/my_epuck_project/resource/nav2_robot2_shared_map.yaml`

The exact frozen FollowPath values are `desired_linear_vel=0.13`, lookahead
`0.20/0.12/0.30` with `lookahead_time=1.5`, velocity-scaled lookahead,
regulated scaling enabled with minimum radius `0.50` m and minimum speed
`0.026` m/s, rotate-to-heading enabled at `0.785` rad with `0.35` rad/s and
`0.20` rad/s², no reversing, collision detection enabled, and stateful mode.
The existing `transform_tolerance=1.5` is retained. Progress/goal checkers,
smoother, Collision Monitor, costmaps, SLAM, fusion, allocator, teammate
filter, lidar, wheel geometry, and motion limits were not changed.

## Diagnostic compatibility

`two_robots_teammate_filtered_stack_launch.py` now treats RPP as the normal
source YAML and keeps an explicit `controller_variant:=dwb` or
`controller_variant:=rotation_shim_dwb` path by generating an isolated
diagnostic parameter copy containing the historical DWB block. The normal
launch does not require a generated temporary RPP file. The regression runner's
default controller variant is now `rpp`; the three cooperative wrapper launches
also default to `rpp`, so an ordinary launch cannot silently reselect DWB.
Explicit diagnostic selection remains available.

## Static verification and tests

Both production YAML files parse with the RPP plugin and all frozen critical
parameters. The focused navigation parameter test passed **9/9**. Package build:

```text
colcon build --packages-select my_epuck_project --symlink-install
  Summary: 1 package finished
```

The complete dirty-workspace test collection produced 420 passed, 1 skipped and
7 pre-existing unrelated failures (plus the repository-wide lint/pep checks
which scan generated and unrelated Webots sources). Those failures were not
modified as part of this controller promotion.

The focused promotion/regression suite passed **60/60**:
`test_navigation_parameter_parity.py`, `test_cooperative_regression.py`, and
`test_distributed_launch_inventory.py`.

## Focused production regression

Command path: `run_cooperative_regression.py`, with no `--controller-variant`
override, large world, headless, full sensors, 240 s simulation bound, and
forensic map/GT capture enabled. The run passed readiness, live clock/sensors,
odometry, local/shared maps, TF, Nav2 lifecycle, controller servers, allocator,
frontier generation, and distributed agreement gates.

Runtime controller-server logs explicitly report for both robots:

```text
Created controller : FollowPath of type nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController
Activating controller: FollowPath of type regulated_pure_pursuit_controller::RegulatedPurePursuitController
```

The regression ran 242.98 simulated seconds (311.31 wall seconds), reached 5
unique agreed rounds and 9 decision agreements, accepted 9 goals, and produced
8 terminal successes with no terminal navigation failures and no recovery-count
changes. Robot 1 travelled 17.237 m and Robot 2 11.480 m. Corrected one-time
GT/odom alignment gave translation RMSE/final error of 0.016/0.040 m (Robot 1)
and 0.0046/0.0114 m (Robot 2). Final local/shared known-cell counts were
106,880/122,586 (Robot 1) and 70,764/119,360 (Robot 2).

The runner labels the attempt `BOUNDED_DIAGNOSTIC` because the requested
simulation bound ended while a final goal was active; this is a normal bounded
shutdown, not an infrastructure-invalid run. Cleanup was clean, all processes
were released, and port 23000 was free afterward.

## Recovery-churn monitoring

The existing event logger remains in place. It records goal identity, start and
terminal events, terminal result, and recovery-count feedback changes. No
recovery churn occurred in this focused regression. The previously established
caveat remains documented: a small subset of failed RPP goals can accumulate a
large Nav2 `number_of_recoveries` feedback count; this task does not retune or
redesign the recovery tree.

## Files changed by this promotion

Promotion-specific edits:

- production FollowPath blocks in the two `nav2_*_shared_map.yaml` files;
- production-default/diagnostic variant handling in
  `two_robots_teammate_filtered_stack_launch.py`;
- RPP defaults in `two_robots_frontier_candidates_launch.py`,
  `two_robots_decentralized_exploration_launch.py`, and
  `two_robots_distributed_assignment_launch.py`;
- production-default runner argument and help text in
  `my_epuck_project/cooperative_regression.py`;
- focused production RPP assertion in `test_navigation_parameter_parity.py`.

The working tree contained many unrelated pre-existing dirty and untracked
diagnostic/world files; they were preserved. No SLAM, wheel, world, fusion,
allocator, frontier, teammate-filter, lidar, or costmap production settings
were changed.

## Engineering decision

The frozen RPP candidate is now the production FollowPath controller. The
controlled mechanism tests and six-mission cooperative evidence support the
promotion, and the normal production launch loaded RPP for both robots without
diagnostic overrides. The remaining deep-recovery behavior is a separately
monitored follow-up issue, not a reason to retain DWB.

RPP_PRODUCTION_PROMOTION_COMPLETE
