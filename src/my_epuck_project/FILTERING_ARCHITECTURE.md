# Filtering architecture audit

The authoritative teammate-filtering pipeline is owned only by
`my_epuck_project` in `/home/arash/webots_ws`. Its data path is:

```text
/robotN/scan_d500
-> d500_scan_fix
-> /robotN/scan_d500_fixed
-> teammate_scan_filter
-> /robotN/scan_d500_slam
-> /robotN/slam_toolbox
```

`two_robots_teammate_filtered_dual_slam_launch.py` starts both fixed-scan
producers (through the namespaced robot launch), both teammate filters, and
both SLAM Toolbox instances. `two_robots_teammate_filtered_stack_launch.py`
includes that launch and adds the cooperative stack. The two authoritative
SLAM configurations are
`slam_toolbox_robot1_teammate_filtered.yaml` and
`slam_toolbox_robot2_teammate_filtered.yaml`; they consume only the matching
`/robotN/scan_d500_slam` topic. Nav2 and Collision Monitor consume
`/robotN/scan_d500_fixed` and are outside the SLAM-only filtering branch.

## Reference classification

- `ACTIVE_RUNTIME`: `my_epuck_project/d500_scan_fix.py`,
  `my_epuck_project/teammate_scan_filter.py`,
  `my_epuck_project/slam_range_policy.py`, the two teammate-filtered launch
  files, the two `slam_toolbox_robotN_teammate_filtered.yaml` files, and the
  `setup.py` console entry point.
- `ACTIVE_TEST`: `test_teammate_scan_filter_geometry.py`,
  `test_slam_range_policy.py`, `test_slam_pipeline_separation.py`,
  `test_robot_identity_mapping.py`, `test_controller_pipeline_diagnostics.py`,
  and the scan-topic checks in `test_cooperative_regression.py`.
- `ACTIVE_DIAGNOSTIC`: `tools/analyze_slam_filter_liveness.py`,
  `controller_pipeline_diagnostics.py`, `cooperative_experiment_logger.py`,
  and `cooperative_regression.py`. These observe or assess the authoritative
  topics; they do not filter scans.
- `ACTIVE_DOCUMENTATION`: this file,
  `docs/simulated_lidar_slam_policy.md`,
  `docs/cooperative_experiment_logging.md`, and the topic-liveness commands in
  `docs/distributed_assignment_validation_20260731.md`.
- `LEGACY_OR_ALTERNATIVE`: the entire `src/scan_sanitize` package, generic
  `/scan_filtered` users including `resource/nav2_params.yaml` and
  `resource/slam_toolbox_epuck_filtered.yaml`, generic D500 SLAM profiles,
  `slam_toolbox_robot1_two_robots.yaml`,
  `slam_toolbox_robot2_two_robots.yaml`, and the unrelated upstream
  `webots_ros2_tests` `/scan_filtered` test. None is launched by the current
  teammate-filtered stack.
- `GENERATED_COPY`: `build/my_epuck_project`, `install/my_epuck_project`,
  Python `__pycache__` directories, and `.pytest_cache`. They are build/test
  products, never authoritative edit targets.

The validated Webots world mounts both a Pi-puck and the custom D500 housing.
The official Webots `Pi-puck.proto` bounding object is a centered cylinder
with radius `0.035 m`, larger than the D500 housing's `0.025 m` radius. Live
fixed scans at a peer-center distance of about `0.2905 m` return about
`0.2567 m` on the center beam, independently confirming a visible radius of
about `0.034 m`. The authoritative ray-intersection radius is therefore
`0.035 m`; the lower stock e-puck body cylinder is not added to it.

Only the authoritative source files listed above should be changed for this
pipeline. Do not update legacy `/scan_filtered` paths to mirror its API.
