# Distributed assignment validation record — 2026-07-31

## Reproducibility state

- Integration branch: `thesis/distributed-pair-assignment`
- Starting parent commit: `518022439d3b94e4a3d7b6065baf0729be6ec641`
- Frontier repository branch/patch: `thesis/rotated-grid-origin`, commit
  `476aaf45a9e863efa1ba1900f4ebcf37325f379c`
- Saved world: `src/my_epuck_project/worlds/epuck_d500_two_world_large.wbt`
- SHA-256: `93a920db40f206073c857b60f92915e6c5253a02963a4d91b147c63ab245f0fe`
- Starts: Robot 1 `(16.0, 0.0)`, Robot 2 `(18.5, 0.0)`, separation `2.5 m`
- Webots transport: Windows Webots R2025a, WSL address `127.0.0.1`, port `23000`

The saved-world edits predated this branch and were intentionally not staged or modified.

## Offline evidence

- Upstream `frontier_exploration_ros2`: 142 tests, 0 failures. This includes the new 90-degree
  occupancy-origin map/world round trip.
- Upstream-backed proposal generator: 12 reported tests, 0 failures (11 gtest cases plus CTest
  report), no compiler warnings after correction.
- Distributed algorithm/protocol/ROS-boundary/logger suite: 49 tests, 0 failures.
- Earlier scoped distributed suite before the lifecycle/telemetry additions: 41 tests, 0 failures.
- `my_epuck_project` symlink build: passed. Its existing package-index marker warning remains.
- All eight required launch entry points passed `ros2 launch ... --show-args` before the final
  observer addition; the final launch is rechecked in the closing validation pass.

The deterministic DDS fixed-task run produced matching values at both peers:

```text
round_id     d68191cc6563c7c7f9d86589a83435924502e1a336c3bd0845da22bcdb49c69c
union_hash   0dbd57bf39c05fae631c30ca0e573c27a57e1e9411cf337bf02ddbb22cd6e6fe
decision     49d495103d23abfa18d1491cd64279ba7e54f8b2fdf422f10707fb6e01a8dc3f
robot1 task  cb62fa1ce3190e48a9fe9a1f
robot2 task  8d8515debbb988d90ab5de0b (alternate east branch)
team score   4.443512
route term   0.142857
```

Both peers logged positive replicated agreement with dispatch disabled. The bounded harness ended
with timeout status 124 by design; this is not a mission result. ROS logs are under
`~/.ros/log/2026-07-31-22-00-38-817990-DESKTOP-OVP2LHB-349279`.

## Historical artifacts inspected

`regression_20260730T220449Z` used the 35.94 m start revision. It reported duplicate fraction
0.095, distances 15.71/42.76 m, 68 accepted goals, 8 successes, 53 failures, 6 cancellations, and
123 recoveries; it did not complete.

`regression_20260730T235432Z` used the current 2.5 m saved-world revision. It reported duplicate
fraction 0.6441, distances 5.43/4.04 m, 6 accepted goals, 4 successes, 2 failures, no cancellation,
and 3 recoveries; it did not complete.

The quoted 90.86% duplicated-known result was not located in the inspected artifacts and is not
used as measured evidence here.

## Commands

Build the required packages without rebuilding unrelated workspace packages:

```bash
cd /home/arash/webots_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select frontier_exploration_ros2 my_epuck_interfaces my_epuck_frontier_candidates my_epuck_project --symlink-install --allow-overriding frontier_exploration_ros2 my_epuck_frontier_candidates
source install/setup.bash
```

Open the exact source world manually from WSL if Webots is not already open:

```bash
"/mnt/c/Program Files/Webots/msys64/mingw64/bin/webots.exe" --port=23000 "$(wslpath -w /home/arash/webots_ws/src/my_epuck_project/worlds/epuck_d500_two_world_large.wbt)"
```

Use a dedicated domain in every ROS terminal:

```bash
export ROS_DOMAIN_ID=224
export WEBOTS_CONTROLLER_URL=tcp://127.0.0.1:23000
source /opt/ros/jazzy/setup.bash
source /home/arash/webots_ws/install/setup.bash
```

Launches:

```bash
ros2 launch my_epuck_project two_robots_decentralized_mapping_only_launch.py webots_port:=23000 world_profile:=large
ros2 launch my_epuck_project two_robots_mapping_plus_nav2_launch.py webots_port:=23000 world_profile:=large
ros2 launch my_epuck_project two_robots_assignment_dry_run_launch.py webots_port:=23000 world_profile:=large
ros2 launch my_epuck_project two_robots_decentralized_exploration_launch.py webots_port:=23000 world_profile:=large output_root:=/home/arash/webots_ws/results
ros2 launch my_epuck_project fixed_task_distributed_assignment_launch.py
ros2 launch my_epuck_project single_robot_frontier_baseline_launch.py webots_port:=23000 world_profile:=large
ros2 launch my_epuck_project two_robots_legacy_claim_baseline_launch.py webots_port:=23000 world_profile:=large
```

RViz is passive and may run separately:

```bash
rviz2 -d /home/arash/webots_ws/src/my_epuck_project/resource/cooperative_manual_exploration.rviz --ros-args -p use_sim_time:=true
```

Checks:

```bash
ros2 topic hz /robot1/scan_d500_fixed /robot1/scan_d500_slam /robot1/odom /robot1/map /robot1/shared_map
ros2 topic hz /robot2/scan_d500_fixed /robot2/scan_d500_slam /robot2/odom /robot2/map /robot2/shared_map
ros2 topic echo /cslam/robot1/local_map --once
ros2 topic echo /cslam/robot2/local_map --once
ros2 topic echo /robot1/task_snapshot --once
ros2 topic echo /robot2/task_snapshot --once
ros2 topic echo /robot1/task_bids --once
ros2 topic echo /robot2/task_bids --once
ros2 topic echo /robot1/pair_decision --once
ros2 topic echo /robot2/pair_decision --once
ros2 topic echo /robot1/distributed_status --once
ros2 topic echo /robot2/distributed_status --once
ros2 action info /robot1/compute_path_to_pose
ros2 action info /robot1/navigate_to_pose
ros2 action info /robot2/compute_path_to_pose
ros2 action info /robot2/navigate_to_pose
ros2 lifecycle get /robot1/planner_server
ros2 lifecycle get /robot1/controller_server
ros2 lifecycle get /robot1/bt_navigator
ros2 lifecycle get /robot2/planner_server
ros2 lifecycle get /robot2/controller_server
ros2 lifecycle get /robot2/bt_navigator
ros2 run tf2_ros tf2_echo shared_map robot1/base_footprint
ros2 run tf2_ros tf2_echo shared_map robot2/base_footprint
```

Stop the foreground launch with `Ctrl+C`. If a prior launch did not exit cleanly, inspect first,
then stop only this project's ROS processes:

```bash
pgrep -af 'ros2 launch|distributed_frontier_assignment|frontier_candidate_generator|webots-controller|cooperative_experiment_logger'
pkill -INT -f 'ros2 launch my_epuck_project'
```

Reports are created under `/home/arash/webots_ws/results/<run_id>/`; ROS process logs are under
`~/.ros/log/<launch-timestamp>/`.

## Unmet live acceptance evidence

No claim is made yet for manual-goal reliability, live two-branch motion, failure injection, full
coverage comparison, CPU/RSS of the whole ROS/Webots graph, or DDS bandwidth. These require a
reachable Windows Webots instance on port 23000 and bounded physical runs. Until the documented
manual reachable-goal set reaches at least 80% success for each robot, autonomous performance
results are not accepted. The two full end-to-end run allowance remains unused by this branch.
At the close of this offline pass, the direct `127.0.0.1:23000` TCP probe timed out and Windows
`netstat` showed no listener, so starting a simulation was not possible without external Webots
state.
