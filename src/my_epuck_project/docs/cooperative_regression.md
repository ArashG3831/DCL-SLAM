# Cooperative exploration simulation regression

This utility runs isolated, passive, repeatable two-robot Webots simulation
campaigns. It does not publish commands, navigation goals, claims, statuses,
maps, or candidate data. The committed decentralized coordinators remain the
only exploration decision makers.

## Prerequisites

- Ubuntu under WSL with ROS 2 Jazzy and this workspace overlay.
- Windows Webots R2025a under `/mnt/c/Program Files/Webots`.
- Windows Webots must be reachable from WSL on the selected per-trial ports.
- Python packages `numpy` and `psutil`.
- The workspace dependencies already installed. The command builds only
  `my_epuck_project`; it does not build or modify unrelated source trees.
- Selected ROS domain IDs and Webots ports must be unused.

Webots R2025a was locally verified to support `--mode=fast`,
`--no-rendering`, `--batch`, and `--port=<port>`. The official
`WebotsLauncher` maps `webots_mode:=fast` and `webots_gui:=false` to:

```text
webots.exe --port=<trial-port> --no-rendering --stdout --stderr --minimize \
  <temporary-world-copy> --batch --mode=fast
```

Every concurrent attempt gets a unique ROS domain, Webots port, result
directory, ROS log directory, temporary directory, and process group. The
committed `use_sim_time=false` policy is preserved. If Webots simulation time
is not genuinely available, reports say `unavailable` and do not invent a
real-time factor.

## Commands

Run these from `/home/arash/webots_ws` after sourcing the ROS installation and
workspace overlay:

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
python3 src/my_epuck_project/tools/run_cooperative_regression.py \
  --trials 10 --maximum-concurrency 3 \
  --execution-profile headless --sensor-profile full \
  --world-profile large \
  --ros-domain-base 100 --webots-port-base 23000 \
  --startup-timeout 180 --mission-timeout 1800 \
  --fast-mode true --rendering false
```

`headless` is the validated full-sensor campaign profile: rendering and RViz
are disabled while the standard sensor, physics, SLAM, Nav2, fusion, and
cooperative settings remain enabled. The `throughput` profile is retained only
for explicitly controlled experiments; it selects the reduced-sensor profile
and is not a recommended benchmark mode.

This is the normal top-level command. It validates prerequisites, builds only
`my_epuck_project`, runs deterministic package tests, performs calibration and
the two-instance isolation pilot, selects bounded concurrency, runs the
remaining valid trials, performs all map analysis, and writes the reports.

One-trial quick test:

```bash
python3 src/my_epuck_project/tools/run_cooperative_regression.py \
  --trials 1 --maximum-concurrency 1 \
  --campaign-id regression_quick \
  --calibration true --isolation-pilot false
```

The default for new continuous/manual campaigns is `--world-profile large`,
which selects `epuck_d500_two_world_large.wbt`, 0.03 m local/shared/global-map
resolution, and 0.02 m local-costmap resolution. The proven 1.5 m small world
and its existing resolutions remain explicitly available with
`--world-profile small`.

One normal-speed rendered large-world run with the passive RViz view:

```bash
cd ~/webots_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash

python3 src/my_epuck_project/tools/run_cooperative_regression.py \
  --trials 1 \
  --maximum-concurrency 1 \
  --world-profile large \
  --ros-domain-base 151 \
  --webots-port-base 23101 \
  --startup-timeout 180 \
  --mission-timeout 1800 \
  --fast-mode false --rendering true --rviz true \
  --hold-open-after-completion true \
  --skip-build \
  --skip-tests
```

The manual RViz preset initially shows both shared maps, the grid, and one pose
Axes display on each robot's namespaced `base_link`. The complete TF tree
remains available but starts disabled, as do both LaserScan displays, robot
models, plans, markers, and both robots' local and global costmaps. Both
profiles retain an Orbit camera and normal 3D controls. The large profile
starts with a 45 m camera distance centered along the long arena. The runner
starts the optional RViz window with the stack so startup is visible; RViz is
not a readiness dependency.

`--hold-open-after-completion true` requires one trial, concurrency one, and
rendering. After both robots complete and the settled final status, claims,
and maps are captured, the runner disables the remaining mission deadline and
keeps Webots, RViz, and the ROS stack open. Press Ctrl+C once to finalize the
observer, perform scoped graceful shutdown, audit the port/process cleanup,
and generate the campaign reports. The option defaults to false, so automated
regression campaigns retain their existing automatic shutdown behavior.

Sequential fallback:

```bash
python3 src/my_epuck_project/tools/run_cooperative_regression.py \
  --trials 10 --maximum-concurrency 1 \
  --calibration true --isolation-pilot false
```

Resume without overwriting completed valid trials:

```bash
python3 src/my_epuck_project/tools/run_cooperative_regression.py \
  --campaign-id regression_YYYYMMDDTHHMMSSZ --resume
```

Regenerate analysis and reports without starting ROS or Webots:

```bash
python3 src/my_epuck_project/tools/run_cooperative_regression.py \
  --campaign-id regression_YYYYMMDDTHHMMSSZ --analysis-only
```

The configurable operational options include `--output-root`,
`--startup-timeout`, `--mission-timeout`, `--settling-period`,
`--graceful-shutdown-timeout`, `--hard-shutdown-timeout`,
`--free-threshold`, `--occupied-threshold`, and `--shift-window`. Use
`--world-profile` selects `large` or `small`; `--help` lists the complete
interface. No source edit is needed between campaigns.

## Safe interruption and resume

Press Ctrl+C once. Each active attempt receives scoped SIGINT, followed by
bounded SIGTERM and SIGKILL only for remaining processes belonging to that
attempt. Progress and every attempt directory are retained. Do not use global
`pkill`, `killall`, or Windows `taskkill` commands. Resume with the command
above.

## Results and exit codes

Results are written below
`results/regression_<UTC timestamp>/`. Each attempt remains under `attempts/`,
including an infrastructure attempt retried once. Inspect a failed trial using
its `runner_metadata.json`, `final_state.json`, `launch.log`, `collector.log`,
and `observer/` directory.

Exit codes:

- `0`: all requested trials were valid, passed, analyzed, and cleaned up.
- `2`: the campaign and analysis completed, but at least one valid trial
  failed/timed out/crashed or cleanup was incomplete.
- `3`: prerequisite, build, test, resource-isolation, or analysis
  infrastructure failure.
- `130`: user interruption.

At exit the command prints the exact paths to `campaign_report.md`,
`campaign_summary.json`, `trials.csv`, and `pairwise_map_metrics.csv`.
Webots profiles default to `--time-mode sim`: the Webots ROS 2 driver is the
single `/clock` publisher, and mission TTLs, Nav2, SLAM, coordination and
observer timestamps follow simulation time. Startup, process shutdown,
clock-stall and emergency limits remain wall-clock safeguards. Use
`--no-mission-timeout` for an unbounded simulated mission and
`--emergency-wall-runtime SECONDS` for unattended protection. Physical-robot
launches remain wall-time profiles.
