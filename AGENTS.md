minimal_frontier_allocator is the only true decentralized task allocator, all else are either old or broken.
use the verified files in /home/robot1/cooperative_migration_source;
cooperative_experiment_logger.py is the correct observer, working, and verified. not thin_experiment_recorder.py or others.
/home/arash/webots_ws sucks ass and must never be used.

# Authoritative WSL workspace

The actual correct WSL folder for the canonical simulator, offline reports, and
offline videos is:

```text
/home/arash/webots_peer_lifecycle_validation_20260912_clean_20260915
```

Use that folder only for the canonical simulation and its post-processing.
It is an isolated validation worktree on the local branch
`validation-clean-report-20260915`. The original
`/home/arash/webots_peer_lifecycle_validation_20260912` is the dirty source
worktree and must not be used for canonical runs. The old
`/home/arash/webots_ws` tree must never be sourced, built, launched, or used
for reports.

The clean workspace contains:

```text
<workspace>/src
<workspace>/build_canonical_thesis_20260907_symlink
<workspace>/install_canonical_thesis_20260907_symlink
<workspace>/dependency_overlay_rclcpp_action_2798_20260909
<workspace>/results
```

The dependency overlay is a required real directory inside this workspace and
is ignored as generated support data. Do not delete or replace it with the
old workspace's overlay. The build and install use `--symlink-install`; their
Python package resolution must point back into this workspace. The validated
Webots ROS 2 driver is the approved external dependency at:

```text
/home/arash/webots_ws_close_validation_2eb/install/webots_ros2_driver
```

That external driver prefix is allowed by the canonical launcher. No other
workspace install prefix is allowed.

The simulator runtime source is based on migration commit
`9ae5d0846a7d85448b931fb950e8dea75465d6cf`. The only local commits after that
in this isolated worktree are the verified offline-report closure and renderer
tools; they do not change the simulator allocator/runtime path. Do not push
the local validation branch or alter the original worktree.

# Clean and isolation requirements

Before every canonical run:

```bash
WORKSPACE=/home/arash/webots_peer_lifecycle_validation_20260912_clean_20260915
git -C "$WORKSPACE" diff --quiet HEAD --
git -C "$WORKSPACE" status --short --branch
```

The tracked-tree diff must be empty. The dependency overlay may be present as
ignored generated support data. Build, install, results, and report/video
outputs are expected local generated data. Never bypass the launcher's clean
tree gate.

Do not create a new worktree for each run. Preserve this isolated workspace,
its build/install layout, its dependency overlay, and its provenance. If a
runtime creates tracked `__pycache__` changes, restore only those generated
files and recheck the clean gate before the next run.

For migration work on the Pi, use only the verified source at
`/home/robot1/cooperative_migration_source`. Do not use a Git checkout or an
old WSL workspace as the migration source. Pi artifacts may be copied back to
this WSL workspace after finalization; report generation and video rendering
remain offline on WSL.

# Safety and environment

Before launching ROS or Webots, read:

```text
<workspace>/docs/HOST_SAFETY_INCIDENT_2026-08-24.md
<workspace>/docs/HOST_RESOURCE_STOP_POLICY.md
```

Use the canonical launcher and its clean-environment, provenance, readiness,
resource, watchdog, and termination gates. Do not reconstruct the launch by
hand or bypass a failed gate. Use ROS Jazzy, CycloneDDS, WSL NAT networking,
and the launcher's resolved Windows endpoint. Never source
`/home/arash/webots_ws/install` and never silently fall back to Fast DDS.

# Canonical simulator run

The documented launcher is:

```text
<workspace>/src/my_epuck_project/tools/run_thesis_experiment.py
```

The verified standard run is condition C for 180 seconds. Run it from the
clean environment below so inherited paths cannot select an old workspace:

```bash
WORKSPACE=/home/arash/webots_peer_lifecycle_validation_20260912_clean_20260915
env -i HOME=/home/arash \
  PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin \
  WSL_INTEROP="${WSL_INTEROP-}" \
  bash --noprofile --norc -c '
    set -e
    source /opt/ros/jazzy/setup.bash
    source "$1/dependency_overlay_rclcpp_action_2798_20260909/setup.bash"
    source "$1/install_canonical_thesis_20260907_symlink/setup.bash"
    cd "$1"
    exec python3 src/my_epuck_project/tools/run_thesis_experiment.py \
      --condition C --horizon 180
  ' bash "$WORKSPACE"
```

The launcher accepts conditions `A`, `B`, `C`, and `D`, a required
`--horizon`, optional `--ros-domain-id`, `--webots-port`, `--results-root`,
`--record-frontier-geometry`, and `--dry-run`. Use the existing launcher
defaults unless a different condition or horizon is explicitly required.

The canonical condition-C launch uses the existing
`two_robots_decentralized_exploration_launch.py` chain with the recorded
unknown-pose profile, full sensors, ideal simulated encoders, fast Webots
mode, `frontier_cost_only`, synchronized start release, and the legacy
observer. It launches `minimal_frontier_allocator` as the decentralized task
allocator. Do not add `distributed_frontier_assignment`, `frontier_explorer`,
or `thin_experiment_recorder` to this chain.

For source changes, rebuild only the affected package with the existing
symlink layout. Do not use `--merge-install`. Verify that the installed
executable/import resolves to this workspace before running.

# Observer and artifact contract

The correct observer is the proven legacy
`cooperative_experiment_logger.py`. It is not
`thin_experiment_recorder.py`. The canonical simulator enables:

```text
enable_observer=true
observer_architecture=legacy
```

The finalized observer artifact is under:

```text
<fast_trial>/observer/<run_id>/
```

It contains the event ledger, robot timeseries, frontier-region records, task
and bid/status evidence, maps/TF/odometry forensic data, coverage data,
navigation replay, rosbag export, `summary.json`, `mission_result.json`, and
`artifact_finalization.json`. Use only finalized artifacts. A clean shutdown
and complete observer finalization do not by themselves mean the mission
completed; inspect `mission_result.json` and the report's mission status.

# Offline report generation

The correct report generator is in the same authoritative workspace:

```text
<workspace>/src/my_epuck_project/tools/generate_offline_run_report.py
<workspace>/src/my_epuck_project/my_epuck_project/offline_timing_metrics.py
```

The timing module is part of the required report-tool closure. Do not use a
report generator from `/home/arash/webots_ws` or an older checkout. Generate
reports only after observer finalization:

```bash
WORKSPACE=/home/arash/webots_peer_lifecycle_validation_20260912_clean_20260915
RUN="$WORKSPACE/results/<campaign>/fast_trial_<run_id>"
PYTHONDONTWRITEBYTECODE=1 PYTHONPATH="$WORKSPACE/src/my_epuck_project" \
  python3 -B \
  "$WORKSPACE/src/my_epuck_project/tools/generate_offline_run_report.py" \
  --artifact "$RUN" \
  --output-json "$RUN/offline_report.json" \
  --output-md "$RUN/offline_report.md"
```

This generator is offline-only: it reads observer JSON/JSONL/CSV artifacts,
does not connect to ROS or Webots, and writes only the requested report
outputs. It may perform the existing Windows Desktop handoff. The generated
JSON is the machine-readable source for the Markdown report. Productivity,
navigation, coverage, idle attribution, and mission status must be taken from
this verified generator's output, not inferred from a partial log.

# Offline video rendering

The correct renderer is also in the same workspace:

```text
<workspace>/src/my_epuck_project/tools/render_cooperative_animation.py
```

It consumes finalized observer artifacts only; it never connects to ROS or
Webots and does not require a camera. It renders the saved map, transformed
robot trajectories, frontiers, candidates, selected goals, and planned paths:

```bash
WORKSPACE=/home/arash/webots_peer_lifecycle_validation_20260912_clean_20260915
RUN="$WORKSPACE/results/<campaign>/fast_trial_<run_id>"
mkdir -p /tmp/robot_render
PYTHONDONTWRITEBYTECODE=1 PYTHONPATH="$WORKSPACE/src/my_epuck_project" \
  python3 -B \
  "$WORKSPACE/src/my_epuck_project/tools/render_cooperative_animation.py" \
  --campaign "$RUN" \
  --output /tmp/robot_render/cooperative_animation.mp4 \
  --provenance-output /tmp/robot_render/cooperative_animation.provenance.json \
  --fps 30 --speedup 10 --style thesis
```

The verified serial OpenCV renderer produces a 1920x1080 H.264-compatible
video and a provenance JSON. Keep raw artifacts in the workspace; copy only
the final report JSON/Markdown, video, and video provenance to a uniquely
named accessible Desktop handoff folder.

# Latest verified run

The latest canonical run from this workspace is:

```text
/home/arash/webots_peer_lifecycle_validation_20260912_clean_20260915/results/thesis_condition_C_180s_20260914T215916Z/fast_trial_20260914T215921Z
```

Its observer finalized cleanly. The corrected report and report-tool closure
were verified from this same workspace, and the renderer was verified against
the same artifact. Its mission reached the 180-second horizon while goals
were still active, so its mission status is `INCOMPLETE`; do not call that a
reporting failure or a completed exploration mission.

# Architecture invariants

- `minimal_frontier_allocator` is the only task owner for this architecture.
- Simulator cooperative behavior must remain peer-based; physical solo mode
  is an explicit minimal-allocator extension that listens for Robot 1 and
  resumes peer coordination when Robot 1 appears.
- Preserve the existing candidate centroid/approach-pose distinction.
- Preserve Nav2 planner, exact path, footprint, TF, and safety gates.
- Preserve the legacy observer schema, event names, metric formulas, and
  finalization contract.
- Do not add a second allocator, fake peer, new protocol, duplicate rosbag,
  camera dependency, or replacement observer.
- Never claim movement, productivity, completion, or report compatibility from
  static inspection alone; use the finalized artifact and the verified
  offline consumers.
