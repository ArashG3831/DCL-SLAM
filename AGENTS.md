# Repository instructions

## Workspaces and provenance

- **Main analysis workspace:** `/home/arash/webots_ws_clean_validation_20260823`.
  It may contain legitimate dirty source, tests, reports, and experiments. It
  is not automatically a runtime source.
- **Persistent clean validation worktree:** the reusable worktree selected by
  `git worktree list`. The latest canonical run used
  `/home/arash/webots_peer_lifecycle_validation_20260912`. Verify this pointer
  before a run; intentionally rotate it only when necessary and record the new
  path in the run provenance.
- Do not create a new worktree for every patch or run. Do not reconstruct the
  validation environment from scratch for every run.
- Do not run a canonical experiment from a tracked-dirty worktree. The
  launcher’s clean-tree gate is mandatory and must not be bypassed.
- Record source path, commit, changed files, install prefixes, command, world,
  profile, domain, port, and termination reason in the artifact.

## Normal patch → run workflow

1. Bring only the authorized source change into the existing validation
   worktree; preserve unrelated analysis-workspace changes.
2. Review `git status`, changed files, and `git diff --check`, then commit the
   intended validation change in that worktree so the canonical clean-tree gate
   passes.
3. Rebuild only the affected package(s), preserving the existing isolated
   `--symlink-install` layout. Never use `--merge-install` for canonical runs.
4. Source ROS Jazzy, the approved dependency overlay, and that worktree’s
   existing install; do not manually substitute another install prefix.
5. Verify the changed package, executable, or Python import resolves from the
   validation worktree/install.
6. Run the canonical launcher:

   ```bash
   python3 src/my_epuck_project/tools/run_thesis_experiment.py \
     --condition C --horizon <N>
   ```

The established layout is:

```text
<validation-worktree>/build_canonical_thesis_20260907_symlink
<validation-worktree>/install_canonical_thesis_20260907_symlink
<validation-worktree>/dependency_overlay_rclcpp_action_2798_20260909
```

For Python-only `my_epuck_project` changes, rebuild/install only
`my_epuck_project`; do not rebuild seven unrelated packages. Build additional
packages only when an actual dependency or interface change requires them.
If a launcher failure occurs before ROS/Webots starts (missing package,
executable, import, or provenance), repair the existing validation
worktree/build/install in place; do not create another worktree.

## Canonical safety and runtime setup

- Never source, build, launch, or accept packages from
  `/home/arash/webots_ws/install`.
- Use the canonical launcher and all of its clean-environment, provenance,
  readiness, resource, and termination gates. Do not manually reconstruct or
  bypass them.
- Before ROS/Webots, read
  `docs/HOST_SAFETY_INCIDENT_2026-08-24.md` and
  `docs/HOST_RESOURCE_STOP_POLICY.md`.
- Use ROS Jazzy with `rmw_cyclonedds_cpp` and the worktree’s
  `config/cyclonedds/wsl_loopback.xml`; never silently fall back to Fast DDS.
  Use WSL NAT (`MY_EPUCK_WEBOTS_NETWORK_MODE=nat`), let the launcher resolve
  the Windows endpoint, and never use a WSL-NAT loopback endpoint.
- Use the approved external prefixes already selected by the launcher:
  `webots_ros2_driver` at
  `/home/arash/webots_ws_close_validation_2eb/install/webots_ros2_driver` and
  the validated `rclcpp_action` dependency overlay in the worktree.
- Preserve the default profile
  `large_unknown_pose_close_start_20ms_scan_matching`, canonical world,
  synchronized `START_RELEASE`, sensors, physics, maps, and experiment
  definitions unless explicitly authorized otherwise.

## Minimal decentralized coordinator

- `MINIMAL_COORDINATOR_SPEC.md` exists and is authoritative; read it before
  minimal-coordinator work. The minimal allocator is implemented and active.
- There is no PC/RViz coordinator. Preserve decentralized robot-local
  decisions, existing messages, traffic, completion, termination, and Nav2
  boundaries. Do not add protocols, ACKs, certificates, histories, or custom
  messages without explicit authorization.
- **Invariant:** current frontier-union membership controls task
  selectability, not active-goal ownership. A local or peer active navigation
  goal survives frontier disappearance/union churn until authoritative
  terminal, inactive, supersession, or expiry lifecycle evidence.
- Preserve continuous frontier/map/TF processing, committed active goals,
  existing Nav2 internal replanning, and event-driven alternative-frontier
  costing only when a robot needs a new goal.

## Protected scope and change discipline

- Treat SLAM/fusion, frontier detection/clustering, costing, Nav2 core,
  traffic safety, launch/configuration, evaluator semantics, observer schemas,
  and experiment provenance as protected. Before changing one, identify the
  contradicted invariant and smallest semantics-preserving correction.
- Keep diffs focused; preserve unrelated dirty changes. Do not claim runtime
  validation from source inspection or offline tests alone. Do not launch ROS,
  Webots, or another experiment unless explicitly requested and all gates pass.

## Offline reports and video

- For a finalized `fast_trial`, use the standard offline-only generator:
  `src/my_epuck_project/tools/generate_offline_run_report.py`.
- Render with
  `src/my_epuck_project/tools/render_cooperative_animation.py --campaign
  <fast_trial>`. Use documented renderer options; serial OpenCV is the
  fallback, while `--workers 4` requires external FFmpeg with `libx264`.
- After creating report/video outputs, copy only those outputs to a new
  uniquely named accessible Windows Desktop folder and open it in Explorer.
  Do not copy raw logs, build/install trees, or temporary directories.
