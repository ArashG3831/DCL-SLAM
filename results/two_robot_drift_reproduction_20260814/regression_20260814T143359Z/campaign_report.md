# Cooperative exploration regression campaign

## Verdict

- Trials completed: 1/1
- Trials passed: 0
- Mission-completion rate: 0.0%
- Crashes: 0
- Timeouts: 0
- Final-map repeatability: UNAVAILABLE
- Worst map pair: unavailable / unavailable
- Major warning counts: {"CONTROLLER_WARNING": 79, "COSTMAP_WARNING": 23, "PROCESS_WARNING": 225, "SCAN_WARNING": 10, "TF_WARNING": 5}
- Selected concurrency: 1
- Total campaign wall time: 405.040807 s

## 1. Configuration

```json
{
  "active_execution_wall_time_s": 403.25208901600126,
  "adaptive_reductions": [],
  "calibration_result": {
    "attempt_id": "trial_01_attempt_01",
    "classification": "BOUNDED_DIAGNOSTIC",
    "cleanup_complete": true,
    "maximum_process_count": 55,
    "process_exit_codes": {
      "collector": 0,
      "launch": 0,
      "rviz": null
    },
    "process_tree_peak_cpu_percent": 105.4,
    "process_tree_peak_rss_bytes": 4857069568,
    "readiness_elapsed_s": 82.85374403100286,
    "wall_time_s": 399.7314956459959
  },
  "campaign_id": "regression_20260814T143359Z",
  "chosen_concurrency": 1,
  "chosen_concurrency_reason": "bounded by requested maximum, normal cap, calibration peak RSS, and isolation pilot outcome",
  "clean_shutdown": true,
  "concurrency_policy": {
    "hard_default_cap": 4,
    "minimum_available_memory_fraction": 0.2,
    "normal_cap": 3,
    "overload_cpu_percent": 90,
    "requested_maximum": 1
  },
  "cpu_model": "12th Gen Intel(R) Core(TM) i7-12650H",
  "dirty_worktree": true,
  "dirty_worktree_status": [
    "M src/my_epuck_project/docs/simulated_lidar_slam_policy.md",
    " M src/my_epuck_project/launch/motion_characterization_launch.py",
    " M src/my_epuck_project/launch/two_robots_decentralized_exploration_launch.py",
    " M src/my_epuck_project/my_epuck_project/__pycache__/__init__.cpython-312.pyc",
    " M src/my_epuck_project/my_epuck_project/__pycache__/d500_scan_fix.cpython-312.pyc",
    " M src/my_epuck_project/my_epuck_project/__pycache__/imu_webots_plugin.cpython-312.pyc",
    " M src/my_epuck_project/my_epuck_project/__pycache__/twist_stamper.cpython-312.pyc",
    " M src/my_epuck_project/my_epuck_project/cooperative_experiment_logger.py",
    " M src/my_epuck_project/my_epuck_project/cooperative_regression.py",
    " M src/my_epuck_project/my_epuck_project/cooperative_regression_report.py",
    " M src/my_epuck_project/my_epuck_project/cooperative_trial_collector.py",
    " M src/my_epuck_project/my_epuck_project/distributed_assignment/canonical.py",
    " M src/my_epuck_project/my_epuck_project/motion_slam_recorder.py",
    " M src/my_epuck_project/resource/cooperative_manual_exploration.rviz",
    " M src/my_epuck_project/setup.py",
    " M src/my_epuck_project/test/test_cooperative_manual_rviz.py",
    " M src/my_epuck_project/test/test_cooperative_regression.py",
    " M src/my_epuck_project/test/test_cooperative_world_profiles.py",
    " M src/my_epuck_project/test/test_distributed_launch_inventory.py",
    " M src/my_epuck_project/test/test_distributed_task_canonicalization.py",
    " M src/my_epuck_project/test/test_slam_pipeline_separation.py",
    " M src/my_epuck_project/test/test_slam_range_policy.py",
    " M src/my_epuck_project/test/test_teammate_scan_filter_geometry.py",
    " M src/my_epuck_project/worlds/.epuck_d500_two_world_large.wbproj",
    " M src/my_epuck_project/worlds/epuck_d500_two_world.wbt",
    " M src/my_epuck_project/worlds/epuck_d500_two_world_both_active.wbt",
    " M src/my_epuck_project/worlds/epuck_d500_two_world_large.wbt",
    " M src/my_epuck_project/worlds/epuck_d500_two_world_robot1_active.wbt",
    " M src/my_epuck_project/worlds/epuck_d500_two_world_robot2_active.wbt",
    " M src/my_epuck_project/worlds/epuck_d500_two_world_teammate_visible.wbt",
    "?? check_slam_stack.sh",
    "?? src/frontier-exploration-ros2/",
    "?? src/m-explore-ros2/",
    "?? src/my_epuck_project/FILTERING_ARCHITECTURE.md",
    "?? src/my_epuck_project/my_epuck_project/cooperative_ground_truth_observer.py",
    "?? src/my_epuck_project/my_epuck_project/cooperative_map_png_export.py",
    "?? src/my_epuck_project/my_epuck_project/cooperative_trial_fast.py",
    "?? src/my_epuck_project/my_epuck_project/forensic_evidence.py",
    "?? src/my_epuck_project/my_epuck_project/motion_course_node.py",
    "?? src/my_epuck_project/my_epuck_project/motion_course_spec.py",
    "?? src/my_epuck_project/my_epuck_project/motion_course_supervisor.py",
    "?? src/my_epuck_project/my_epuck_project/motion_scan_branch_relay.py",
    "?? src/my_epuck_project/test/test_cooperative_map_png_export.py",
    "?? src/my_epuck_project/test/test_cooperative_trial_fast.py",
    "?? src/my_epuck_project/test/test_motion_course_methodology.py",
    "?? src/my_epuck_project/test/test_webots_robot_windows.py",
    "?? src/my_epuck_project/tools/analyze_motion_trajectory.py",
    "?? src/my_epuck_project/tools/analyze_slam_map_quality.py",
    "?? src/my_epuck_project/tools/analyze_turn_motion.py",
    "?? src/my_epuck_project/tools/export_cooperative_maps_png.py",
    "?? src/my_epuck_project/tools/forensic_runtime_efficiency.py",
    "?? src/my_epuck_project/tools/run_cooperative_trial_fast.py",
    "?? src/my_epuck_project/tools/run_nav2_frontier_diagnostic.py",
    "?? src/my_epuck_project/tools/turn_motion_diagnostics.py",
    "?? src/my_epuck_project/worlds/.epuck_d500_two_world.wbproj",
    "?? src/my_epuck_project/worlds/.epuck_d500_two_world_both_active.wbproj",
    "?? src/my_epuck_project/worlds/.epuck_d500_two_world_large.jpg",
    "?? src/my_epuck_project/worlds/.epuck_d500_two_world_robot1_active.wbproj",
    "?? src/my_epuck_project/worlds/.epuck_d500_two_world_robot2_active.wbproj",
    "?? src/my_epuck_project/worlds/.epuck_d500_two_world_teammate_visible.wbproj",
    "?? src/scan_sanitize/",
    "?? src/webots_ros2/"
  ],
  "exact_webots_options": "--port=<trial-port> --batch --mode=fast --no-rendering --stdout --stderr --minimize",
  "execution_profile": "headless",
  "fast_mode": true,
  "final_cleanup_audit": {
    "clean": true,
    "occupied_selected_ports": [],
    "open_result_files": [],
    "remaining_campaign_processes": [],
    "utc": "2026-08-14T14:40:45.657453Z"
  },
  "fusion_resolution": 0.03,
  "git_commit": "84586dab1056810a43101671ac7f65fa985b7221",
  "global_costmap_resolution": 0.03,
  "hostname": "DESKTOP-OVP2LHB",
  "initial_map_costmap_configuration": {
    "coverage_attribution_resolution": 0.03,
    "map_comparison_shift_window": 1,
    "minimum_frontier_cells": 2,
    "minimum_known_cell_gain_for_activity": 1
  },
  "initial_separation_m": 2.5,
  "installed_world_path": "/home/arash/webots_ws/install/my_epuck_project/share/my_epuck_project/worlds/epuck_d500_two_world_large.wbt",
  "installed_world_sha256": "4caa7e7deba011ae88a0a952ac2b7a8680ad6dedc65e0610e03f6a2ea08c8415",
  "isolation_pilot_passed": false,
  "kernel": "6.6.87.2-microsoft-standard-WSL2",
  "known_initial_relative_transform": [
    1.5308084989341916e-16,
    -2.5,
    0.0
  ],
  "launch_arguments": {
    "assignment_mode": "replicated_two_robot_pair",
    "dispatch_enabled": true,
    "hold_open_after_completion": false,
    "launch_rviz": false,
    "no_mission_timeout": false,
    "time_mode": "sim",
    "use_sim_time": true,
    "world_profile": "large"
  },
  "launch_file": "two_robots_decentralized_exploration_launch.py",
  "lidar_maximum_range": 12.0,
  "limitations": [
    "Process launch, startup, clock-stall, and emergency limits use wall time; mission timers use ROS simulation time when time_mode=sim."
  ],
  "local_costmap_resolution": 0.02,
  "logging_mode": {
    "console_status": false,
    "observer": true
  },
  "logical_cpu_count": 16,
  "map_thresholds": {
    "free_max": 25,
    "occupied_min": 65,
    "uncertain": "25 < value < 65",
    "unknown": "value < 0"
  },
  "offline_map_benchmark": {
    "all_45_pair_comparisons_time_s": 2.3984427350005717,
    "cached_alignment_time_s": 0.16027415699500125,
    "common_grid_time_s": 0.00017752800340531394,
    "consensus_generation_time_s": 0.206270319999021,
    "height": 1120,
    "one_pair_comparison_time_s": 0.05060869700537296,
    "resolution": 0.029999999329447746,
    "rss_increase_bytes": 12935168,
    "source": "calibration final map geometry",
    "width": 391
  },
  "os": "Linux-6.6.87.2-microsoft-standard-WSL2-x86_64-with-glibc2.39",
  "peer_export_resolution": 0.03,
  "random_seeds": {
    "controlled": false,
    "reason": "no project/Webots seed is exposed by committed launch"
  },
  "rendering_mode": "disabled",
  "reproduction_command": "src/my_epuck_project/tools/run_cooperative_regression.py --trials 1 --maximum-concurrency 1 --output-root results/two_robot_drift_reproduction_20260814 --world-profile large --execution-profile headless --sensor-profile full --time-mode sim --fast-mode true --rendering false --rviz false --diagnostic-mode true --enable-forensic-capture true --forensic-snapshot-interval-s 15 --mission-timeout 600 --emergency-wall-runtime 900 --no-infrastructure-retry --skip-build --skip-tests --ros-domain-base 228 --webots-port-base 23660",
  "rmw_implementation": "rmw_fastrtps_cpp",
  "robot_start_poses": {
    "robot1": {
      "controller": "<extern>",
      "planar_yaw": 1.5707963267948966,
      "rotation": [
        0.0,
        0.0,
        1.0,
        1.5707963267948966
      ],
      "translation": [
        16.0,
        0.0,
        0.001
      ],
      "window": "<none>"
    },
    "robot2": {
      "controller": "<extern>",
      "planar_yaw": 1.5707963267948966,
      "rotation": [
        0.0,
        0.0,
        1.0,
        1.5707963267948966
      ],
      "translation": [
        18.5,
        0.0,
        0.001
      ],
      "window": "<none>"
    }
  },
  "ros_distribution": "jazzy",
  "ros_domain_id_range": [
    228,
    228
  ],
  "rviz": false,
  "schema_version": "1.0.0",
  "sensor_profile": "full",
  "simulation_time_measurement": "collector /clock telemetry",
  "slam_resolution": 0.03,
  "source_world_path": "/home/arash/webots_ws/src/my_epuck_project/worlds/epuck_d500_two_world_large.wbt",
  "time_mode": "sim",
  "timeouts": {
    "emergency_wall_runtime_s": 900.0,
    "graceful_shutdown_s": 20.0,
    "hard_shutdown_s": 10.0,
    "mission_s": 600.0,
    "mission_timeout_domain": "sim",
    "settling_s": 4.0,
    "startup_s": 300.0
  },
  "total_ram_bytes": 8128757760,
  "total_wall_time_s": 405.040807,
  "transform_source": "WORLD_DERIVED",
  "trial_count_requested": 1,
  "use_sim_time": true,
  "utc_end": "2026-08-14T14:40:45.263345Z",
  "utc_start": "2026-08-14T14:34:00.222538Z",
  "webots_executable": "/mnt/c/Program Files/Webots/msys64/mingw64/bin/webots.exe",
  "webots_mode": "fast",
  "webots_port_range": [
    23660,
    23660
  ],
  "webots_version": "R2025a",
  "world": "epuck_d500_two_world_large.wbt",
  "world_dimensions_m": [
    40.0,
    10.0
  ],
  "world_profile": "large",
  "world_sha256": "4caa7e7deba011ae88a0a952ac2b7a8680ad6dedc65e0610e03f6a2ea08c8415"
}
```

## 2. Parallel-execution methodology

Each complete two-robot stack used a distinct ROS domain, Webots port, process group, ROS log directory, temporary directory, and attempt directory. Calibration and the two-instance pilot gate adaptive bounded scheduling.

## 3. Trial outcomes

| Trial | Result | Robot 1 | Robot 2 | Claims (r1/r2) | Time (s) |
|---|---|---|---|---|---:|
| trial_01 | BOUNDED_DIAGNOSTIC | NAVIGATING | BIDDING | None/None | None |

## 4. Final robot statuses

The table above records the exact final status and claim snapshot.

## 5. Errors and warnings

{"CONTROLLER_WARNING": 79, "COSTMAP_WARNING": 23, "PROCESS_WARNING": 225, "SCAN_WARNING": 10, "TF_WARNING": 5}

## 6. Navigation reliability

{"cancellations": 0, "distance_travelled_m": 54.694371279041775, "failures": 5, "goals": 20, "goals_accepted": 20, "recoveries": 21, "rejections": 0, "successes": 15, "timeouts": 0}

## 7. Mission-completion reliability

0 of 1 valid trials met every pass condition.

## 8. Within-trial robot map agreement

{
  "trial_01": {
    "best_shift_correlation": 1.0,
    "best_shift_dx_cells": 0,
    "best_shift_dx_m": 0.0,
    "best_shift_dy_cells": 0,
    "best_shift_dy_m": 0.0,
    "boundary_tolerant_occupied_agreement": 1.0,
    "common_grid": {
      "height": 1120,
      "origin_x": -5.759999871253967,
      "origin_y": -4.799999892711639,
      "resolution": 0.029999999329447746,
      "width": 391
    },
    "comparison_resolution": 0.029999999329447746,
    "correlation_improvement": 0.0,
    "coverage_area_a_m2": 197.81459115699838,
    "coverage_area_b_m2": 197.81459115699838,
    "coverage_area_difference_m2": 0.0,
    "coverage_area_relative_difference": 0.0,
    "dimensions_a": [
      390,
      1119
    ],
    "dimensions_b": [
      390,
      1119
    ],
    "free_area_difference_m2": 0.0,
    "free_area_relative_difference": 0.0,
    "free_iou": 1.0,
    "geometry_equal": true,
    "jointly_known_cell_count": 219794,
    "known_cell_agreement": 1.0,
    "known_iou": 1.0,
    "known_union_count": 219794,
    "occupied_area_difference_m2": 0.0,
    "occupied_area_relative_difference": 0.0,
    "occupied_free_conflict_rate": 0.0,
    "occupied_iou": 1.0,
    "origin_position_difference_m": 0.0,
    "origin_yaw_difference_rad": 0.0,
    "raw_occupancy_correlation": 1.0,
    "resolution_equal": true,
    "semantic_confusion_matrix": {
      "free/free": 202964,
      "free/occupied": 0,
      "free/uncertain": 0,
      "free/unknown": 0,
      "occupied/free": 0,
      "occupied/occupied": 16830,
      "occupied/uncertain": 0,
      "occupied/unknown": 0,
      "uncertain/free": 0,
      "uncertain/occupied": 0,
      "uncertain/uncertain": 0,
      "uncertain/unknown": 0,
      "unknown/free": 0,
      "unknown/occupied": 0,
      "unknown/uncertain": 0,
      "unknown/unknown": 218126
    },
    "unknown_mismatch_rate": 0.0,
    "zero_shift_correlation": 1.0
  }
}

## 9. Across-trial map repeatability

{
  "best_shift_correlation": {
    "maximum": null,
    "mean": null,
    "median": null,
    "minimum": null,
    "p10": null,
    "p90": null,
    "standard_deviation": null
  },
  "free_iou": {
    "maximum": null,
    "mean": null,
    "median": null,
    "minimum": null,
    "p10": null,
    "p90": null,
    "standard_deviation": null
  },
  "known_cell_agreement": {
    "maximum": null,
    "mean": null,
    "median": null,
    "minimum": null,
    "p10": null,
    "p90": null,
    "standard_deviation": null
  },
  "known_iou": {
    "maximum": null,
    "mean": null,
    "median": null,
    "minimum": null,
    "p10": null,
    "p90": null,
    "standard_deviation": null
  },
  "occupied_free_conflict_rate": {
    "maximum": null,
    "mean": null,
    "median": null,
    "minimum": null,
    "p10": null,
    "p90": null,
    "standard_deviation": null
  },
  "occupied_iou": {
    "maximum": null,
    "mean": null,
    "median": null,
    "minimum": null,
    "p10": null,
    "p90": null,
    "standard_deviation": null
  },
  "raw_occupancy_correlation": {
    "maximum": null,
    "mean": null,
    "median": null,
    "minimum": null,
    "p10": null,
    "p90": null,
    "standard_deviation": null
  },
  "unknown_mismatch_rate": {
    "maximum": null,
    "mean": null,
    "median": null,
    "minimum": null,
    "p10": null,
    "p90": null,
    "standard_deviation": null
  }
}

## 10. Cross-correlation diagnostics

0 unique pairs retain zero-shift authoritative metrics; best small shifts are supplementary only.

```json
{
  "material_improvement_pair_count": 0,
  "maximum_correlation_improvement": 0.0,
  "nonzero_best_shift_pair_count": 0,
  "possible_systematic_alignment_problem": false,
  "repeated_nonzero_shift_patterns": []
}
```

## 11. Consensus and disagreement maps

Artifacts: /home/arash/webots_ws/results/two_robot_drift_reproduction_20260814/regression_20260814T143359Z/aggregate/consensus_map.npz and /home/arash/webots_ws/results/two_robot_drift_reproduction_20260814/regression_20260814T143359Z/aggregate/disagreement_frequency.npz.

```json
{
  "cells_known_in_all_trials": 219794,
  "cells_with_any_disagreement": 0,
  "cells_with_disagreement_at_least_0_2": 0,
  "majority_semantic_cell_counts": {
    "free": 202964,
    "occupied": 16830,
    "uncertain": 0,
    "unknown": 218126
  },
  "maximum_disagreement_frequency": 0.0,
  "mean_disagreement_frequency": 0.0,
  "mean_known_frequency": 0.501904457435148
}
```

## 12. Performance and resource usage

Campaign wall time: 405.040807 s.

## 13. Outliers

All trials remain in the primary aggregates. Flagged trials and the explicitly labeled sensitivity analysis follow.

```json
{}
```

```json
{
  "excluded_trials": [],
  "metrics": {
    "best_shift_correlation": {
      "maximum": null,
      "mean": null,
      "median": null,
      "minimum": null,
      "p10": null,
      "p90": null,
      "standard_deviation": null
    },
    "free_iou": {
      "maximum": null,
      "mean": null,
      "median": null,
      "minimum": null,
      "p10": null,
      "p90": null,
      "standard_deviation": null
    },
    "known_cell_agreement": {
      "maximum": null,
      "mean": null,
      "median": null,
      "minimum": null,
      "p10": null,
      "p90": null,
      "standard_deviation": null
    },
    "known_iou": {
      "maximum": null,
      "mean": null,
      "median": null,
      "minimum": null,
      "p10": null,
      "p90": null,
      "standard_deviation": null
    },
    "occupied_free_conflict_rate": {
      "maximum": null,
      "mean": null,
      "median": null,
      "minimum": null,
      "p10": null,
      "p90": null,
      "standard_deviation": null
    },
    "occupied_iou": {
      "maximum": null,
      "mean": null,
      "median": null,
      "minimum": null,
      "p10": null,
      "p90": null,
      "standard_deviation": null
    },
    "raw_occupancy_correlation": {
      "maximum": null,
      "mean": null,
      "median": null,
      "minimum": null,
      "p10": null,
      "p90": null,
      "standard_deviation": null
    },
    "unknown_mismatch_rate": {
      "maximum": null,
      "mean": null,
      "median": null,
      "minimum": null,
      "p10": null,
      "p90": null,
      "standard_deviation": null
    }
  },
  "remaining_pair_count": 0
}
```

## 14. Limitations

[
  "Process launch, startup, clock-stall, and emergency limits use wall time; mission timers use ROS simulation time when time_mode=sim."
]

## 15. Exact reproduction command

`src/my_epuck_project/tools/run_cooperative_regression.py --trials 1 --maximum-concurrency 1 --output-root results/two_robot_drift_reproduction_20260814 --world-profile large --execution-profile headless --sensor-profile full --time-mode sim --fast-mode true --rendering false --rviz false --diagnostic-mode true --enable-forensic-capture true --forensic-snapshot-interval-s 15 --mission-timeout 600 --emergency-wall-runtime 900 --no-infrastructure-retry --skip-build --skip-tests --ros-domain-base 228 --webots-port-base 23660`
