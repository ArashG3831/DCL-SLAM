# Repository instructions

## Scope and workspace

- This is the only authorized validation workspace:
  `/home/arash/webots_ws_clean_validation_20260823`.
- Never source, build, launch, or accept runtime packages from the stale
  workspace prefix `/home/arash/webots_ws/install`.
- The older checkout `/home/arash/webots_ws` is dirty and is not a runtime
  workspace for this repository.
- Inspect the existing implementation before editing. Prefer a small,
  isolated, semantics-preserving change over a new custom layer.
- Do not claim runtime validation from source inspection alone. Runtime claims
  require the appropriate artifact, live evidence, or experiment result.

## Canonical experiments and preflight

- Use the canonical launcher for thesis experiments:

  ```bash
  python3 src/my_epuck_project/tools/run_thesis_experiment.py --condition C --horizon 180
  python3 src/my_epuck_project/tools/run_thesis_experiment.py --condition C --horizon 600
  python3 src/my_epuck_project/tools/run_thesis_experiment.py --condition C --horizon 1200
  ```

- Do not manually reconstruct the launcher environment, overlays, ports,
  domains, result paths, NAT settings, or preflight checks. Do not bypass a
  dirty-tree, provenance, or readiness gate.
- Before any ROS/Webots launch, read:
  `docs/HOST_SAFETY_INCIDENT_2026-08-24.md` and
  `docs/HOST_RESOURCE_STOP_POLICY.md`.
- Use a clean `env -i`-style runtime environment. Source ROS Jazzy, the
  approved external driver prefix, and this checkout's fresh install only.
  Verify package prefixes and executable/library provenance before launch.
- Use `rmw_cyclonedds_cpp` with the approved WSL loopback profile:
  `config/cyclonedds/wsl_loopback.xml`. Validate `ROS_DOMAIN_ID` against the
  active profile; the approved range is `0..230`. Never silently fall back to
  Fast DDS.
- The selected install must resolve project packages, including interfaces,
  frontier candidates, cooperative exploration, and the SLAM wrapper. No
  effective environment, command, cache, RPATH/RUNPATH, or process may contain
  `/home/arash/webots_ws/install`.
- For WSL NAT, set `MY_EPUCK_WEBOTS_NETWORK_MODE=nat`, resolve and record the
  Windows Webots controller endpoint, and never use `127.0.0.1` as the Windows
  host endpoint from WSL NAT. The approved external driver prefix is
  `/home/arash/webots_ws_close_validation_2eb/install/webots_ros2_driver`.
- Record the selected domain, port, world/profile, install prefixes, command,
  termination reason, and runtime provenance in every experiment artifact.
- Respect the host resource stop policy. The 1200-second wall watchdog is an
  emergency ceiling; the scientific horizon is independent. For the final
  thesis horizon use 1200 simulated seconds, cut metrics at the verified
  horizon, and report active RTF using live clock and monotonic-wall boundaries.

## Canonical thesis setup

- Default profile:
  `large_unknown_pose_close_start_20ms_scan_matching`.
- Default world:
  `src/my_epuck_project/worlds/epuck_d500_two_world_unknown_pose_close_start_dynamic_low_slip_20ms_finite.wbt`.
- Preserve the canonical world, sensors, physics, map resolution, and
  symmetric close-start geometry. Use another world only when explicitly
  requested.
- Both robots must pass readiness and the common simulation-time
  `START_RELEASE` barrier before exploration dispatch or intentional motion.
  Record both ready times, the release time, first goals, and first motion.
- There is no PC/RViz central coordinator or brain. Robot decisions remain
  decentralized; PC/RViz is visualization/evaluation only.
- The main thesis scope is ABC. Preserve the synchronized start and current
  scientific experiment definitions.

## Protected subsystems

Preserve these absent direct contradictory evidence from the active runtime:

- SLAM and unknown-pose registration;
- source-aware map fusion and shared-map semantics;
- frontier generator and decision-map semantics;
- Nav2 core, lifecycle, path/goal validity, and success authority;
- traffic conflict geometry, deterministic winner/loser semantics, and safety;
- canonical world, sensors, physics, and experiment launcher/provenance;
- evaluator, observer, artifact schemas, and scientific metric definitions;
- validated action ownership and stale-callback behavior.

Before changing a protected subsystem, identify the active source path, the
contradicted invariant, and the smallest semantics-preserving correction.
Focused tests or historical artifacts alone do not authorize a redesign.

## Minimal decentralized coordinator

- Before implementing or auditing the new isolated minimal coordinator, read
  `MINIMAL_COORDINATOR_SPEC.md`. That authoritative spec does not exist yet
  and must be created before implementation.
- The minimal coordinator is a narrowly scoped exception to legacy allocator
  lifecycle assumptions. It must not inherit the legacy allocator's rounds,
  continuation, certificate/lower-bound/history, commitment-journal,
  restart-recovery, source-session-recovery, or hard-failure lifecycle.
- This exception applies only to the new minimal-coordinator files. The
  existing allocator remains untouched unless separately authorized.
- Frontier generation, SLAM, fusion, Nav2 core behavior, traffic semantics,
  launcher safety, and evaluation remain protected for the minimal copy too.
- Do not change ROS messages, launch/configuration, or runtime behavior as
  part of design-only work. Use established pure project/framework functions
  instead of duplicating them.

## Observer and reports

- The authoritative observer/evaluation completion requirements are in
  `OBSERVER_EVALUATION_COMPLETION_SPEC.md`.
- Reuse existing observer, offline analyzer, replay, and metric definitions.
  Do not create a second scientific evaluator whose semantics can drift.
- For a finalized `fast_trial`, use
  `src/my_epuck_project/tools/generate_offline_run_report.py` for standard
  offline metrics. It does not run ROS or Webots.
- After a task creates report/output artifacts, create a new uniquely named
  folder on the accessible Windows Desktop, copy only those artifacts, and
  open the folder in Explorer. Preserve source files; do not copy raw logs,
  build/install trees, or temporary directories. If Desktop handoff or
  Explorer opening fails, report the exact status.

## Change discipline

- Do not modify production, tests, messages, launch/configuration, or runtime
  files unless the user explicitly authorizes that scope.
- Keep changes reviewable and focused. Do not repair one custom layer by
  adding another custom layer without proving the underlying requirement.
- Run only checks proportionate to the task and use authoritative repository
  commands. Keep unrelated dirty-worktree changes intact.
- Point to detailed safety, host, observer, launcher, and subsystem documents
  instead of duplicating their procedures here.
