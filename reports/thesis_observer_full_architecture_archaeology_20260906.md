# Observer / Evidence Architecture Archaeology Audit

Date: 2026-09-06
Scope: read-only source, launch, test, artifact, report, and Git-history audit.
No Webots, ROS mission, profiler, experiment, production edit, launch edit, or install edit was performed for this audit.

## Executive verdict

The observer subsystem is not one homogeneous “logger”. It is a mixed architecture containing: (1) a passive live ROS subscriber that currently performs substantial derived work; (2) a Webots Supervisor-side ground-truth/contact recorder; (3) a forensic writer and raw/indexed TF/odometry state; (4) a native passive rosbag recorder; (5) optional diagnostic nodes; and (6) a large offline/rendering surface. The direct observer/evidence production scope is **7,550 LOC**, consisting of **6,219 runtime/direct observer LOC** and **1,331 offline/render LOC**. A broader adjacent-support closure adds **13,603 LOC**, but that broader number includes the runner, generic regression runner, report generator, preflight, termination, and diagnostics and is not counted as observer production LOC.

The evidence supports an architectural redesign toward a thin raw recorder plus deterministic offline evaluator. It does **not** authorize an immediate deletion or live-topic removal: the current source still couples several custom streams to live event, readiness, health, and finalization behavior, and the proposed replacement requires an explicit raw-stream completeness contract and replay/parity tests. The defensible conclusion is:

> **ARCHITECTURAL_THIN_RECORDER_REDESIGN_JUSTIFIED_PARITY_FIRST; NO_IMMEDIATE_LIVE_REMOVAL_AUTHORIZED**

This is a design/audit conclusion, not a performance result and not permission to change production behavior.

## 1. Method and source scope

The audit started from the canonical C launch and followed imports, launch flags, package entry points, artifact writers, finalizers, evaluators, tests, offline exporters, and preserved reports. Static searches used `grep`, `find`, Python AST inspection, line-count scripts, and Git history/blame. `rg` was unavailable in the environment. Existing preserved reports and artifacts were read offline; no new mission was launched.

The current repository is on branch `validation/nat-gate-20260826` at `79ad86bbf1f4fbbf391a257173784e9b82a75e2a` (“checkpoint: current validated state”), with a heavily dirty worktree containing existing user changes, installs, results, and reports. Source line ranges below describe the current worktree, not an assumption that the worktree is clean.

The direct scope is:

| Scope | Files / role | LOC |
|---|---|---:|
| Live/direct observer and evidence | logger, forensic writer, GT/Supervisor, passive rosbag, paced supervisor, motion observer, DWB capture, zero-event capture, debug observer | **6,219** |
| Offline/rendering evidence | `cooperative_map_png_export.py`, `map_exporter.py` | **1,331** |
| Direct observer/evidence total | above two rows | **7,550** |
| Adjacent transitive support, excluded from direct total | `cooperative_trial_fast.py`, regression runner/report, metrics, pipeline telemetry, termination, preflight, Nav2 diagnostic | **13,603** |
| Broad source surface if all adjacent support is included | direct + adjacent | **21,153** |

The direct file counts are:

| File | Role | LOC |
|---|---|---:|
| `cooperative_experiment_logger.py` | central passive logger/finalizer | 4,092 |
| `forensic_evidence.py` | forensic files, raw/indexed evidence, map/TF/odom writer | 484 |
| `cooperative_ground_truth_observer.py` | external Webots GT/contact recorder | 393 |
| `debug_observer_node.cpp` | optional frontier/map/TF visualization observer | 449 |
| `passive_rosbag.py` | selected native rosbag recorder/exporter | 298 |
| `dwb_geometry_capture.py` | optional DWB geometry capture | 203 |
| `dwb_zero_event_capture.py` | optional zero-event capture | 114 |
| `motion_supervisor_observer.py` | noncanonical motion characterization | 100 |
| `paced_ros2_supervisor.py` | fast-mode `/clock` publisher proxy | 86 |
| `map_exporter.py` | separate map publisher/export helper | 121 |
| `cooperative_map_png_export.py` | offline map/figure rendering | 1,210 |
| **Total** |  | **7,550** |

The counts include comments, blank lines, and executable Python/C++ source lines according to the repository line-count script. They are therefore an architecture sizing measure, not a cyclomatic-complexity measure.

### 1.1 Evidence set used

The source/launch pass included the canonical launch files, `cooperative_trial_fast.py`, `cooperative_experiment_logger.py`, `forensic_evidence.py`, `cooperative_ground_truth_observer.py`, `passive_rosbag.py`, `paced_ros2_supervisor.py`, the debug/DWB/motion observers, map exporters, evaluator/analyzer modules, related tests, and message definitions where topic semantics were relevant. The report pass included the accepted full-observer baseline, host/observer attribution, direct RCL attribution, executor wait-set audit, passive-split audit, raw-TF/odom regression recoveries, and the existing Persian/English evaluation-limitations closeout reports. `docs/THESIS_EXPERIMENT_PLAN.md` supplied the current ABC/15-run/START_RELEASE/RTF policy.

No standalone `thesis_blueprint.md`, Persian thesis source, or unambiguous `thesis_campaign_readiness_freeze_20260905.md` was found in the scoped WSL workspace. The provenance reconciliation report records that the freeze file was not reconstructable and requires manual authority review. This audit therefore does not use an absent or conflicting freeze file to promote any artifact or campaign claim.

## 2. Canonical launch and runtime topology

The canonical C path is:

```text
run_cooperative_trial_fast.py
  -> filtered runtime environment / provenance checks
  -> two_robots_decentralized_exploration_launch.py
       -> two_robots_namespaced_launch.py
            -> Webots + robot controllers + Nav2/SLAM/fusion/cooperation
            -> paced_ros2_supervisor.py (/clock fast-mode proxy)
       -> cooperative_experiment_logger.py (passive ROS observer)
       -> cooperative_ground_truth_observer.py (external Webots Supervisor)
       -> passive_rosbag.py (native rosbag2 recorder)
  -> common readiness / handoff / START_RELEASE validation
  -> SIM_TIME_COMPLETE and finalization
       -> rosbag export and deferred reconstruction
       -> artifact_finalization.json, summary, mission result, evaluator inputs
```

The launch wires the logger at approximately lines 583–684 of the current decentralized launch and exposes observer/forensic/contact/passive-bag flags around lines 817–870. The C runner supplies the canonical close-start configuration, `frontier_cost_only`, `MODE_B`, full observer/evidence settings, and a common simulation-time `START_RELEASE`. The canonical path is not asynchronous-start: both robots must be ready and handed off before the common release, and no exploration dispatch or intentional motion is valid before it.

The logger has no control publisher, action client, service client, lifecycle client, or direct Webots control path in the canonical configuration. It reads ROS state and writes observations. `paced_ros2_supervisor.py` is different: it owns the authoritative fast-mode clock behavior and therefore belongs to runtime/acceptance infrastructure even though it is not an exploration controller. The external ground-truth observer reads Webots Supervisor state and writes GT/contact records; it does not command robots.

## 3. Quantitative architecture accounting

The following values are qualified static classifications. They are deliberately ranges where code responsibility crosses categories inside a method or where reachability depends on launch flags.

| Dimension | Direct static result | Interpretation |
|---|---:|---|
| Direct observer/evidence LOC | 7,550 | Includes 6,219 runtime/direct and 1,331 offline/rendering. |
| Canonical-C live/runtime reachable | approximately **4,361–4,761** | Static range; launch-gated branches and helpers make exact executed LOC unavailable without runtime coverage. |
| `cooperative_experiment_logger.py` canonical-C active | approximately **3,100–3,500 of 4,092** | Core initialization, 41 subscriptions, 9 timers, callbacks, TF/odom/maps/events, finalization; optional frontier diagnostics excluded. |
| Control-critical logger LOC | approximately **0 direct** | Logger does not command control. Readiness/finalization checks around it are acceptance-critical but are not robot-control semantics. |
| Runtime/acceptance infrastructure outside logger | approximately **86–400** | Clock proxy, Supervisor readiness/contact, startup/finalization hooks; exact assignment depends on whether GT/contact is treated as live acceptance or evidence. |
| Scientifically necessary live/raw capture | approximately **1,800–2,700** | Raw streams and minimum live validity/GT/contact capture; not every currently derived calculation. |
| Scientifically necessary live derived computation | approximately **0–700** | Only computations proven necessary before shutdown; most metrics can be reconstructed. |
| Acceptance/provenance live work | approximately **700–1,100** | Manifest, horizon, release, recorder/finalization state, required-stream integrity. |
| Legacy/diagnostic/offline candidate surface | approximately **3,000–4,000** | Historical schemas, live derived metrics, optional diagnostics, rendering, compatibility and duplicate representations. |

These ranges are not additive per row because the same function can read raw data, update an acceptance state, and emit a legacy artifact. They are an architectural estimate, not a claim of exact dead-code reachability.

### 3.1 Responsibility LOC decomposition

The direct 7,550 LOC can be coherently partitioned as follows:

| Responsibility group | Approx. LOC | Canonical C status | Notes |
|---|---:|---|---|
| Raw ROS intake, callback dispatch, state updates | 1,050–1,350 | live | 41 subscriptions; includes odom, maps, peer maps, cooperation, cmd velocity, TF, release, descriptors, rosout. |
| Raw/indexed TF and odometry support | 450–650 | live | `tf_buffer`, direct TF arrays, `RawTFSeriesIndex`, `_odom_samples`, exact lookup/fallback. |
| Forensic raw/map/TF/odom writing | 500–750 | live | CSV/JSONL/NPZ writes; some can be replaced by native raw capture plus offline writer. |
| Ground truth/contact/runtime metrics | 393 | live/external | Supervisor-side per-step sampling and contact capture. |
| Live derived telemetry, coverage, health, idle, event ledgers | 750–1,100 | live but mostly evaluation | Important outputs exist, but most can be deterministically derived after raw capture. |
| Synchronized-map/deferred reconstruction | 300–500 | mixed | Live snapshot/index state plus finalization reconstruction; major offline candidate. |
| Finalization/manifest/summary/mission result/passive-bag glue | 700–950 | live acceptance | A thin contract should retain validity checks while moving metric production offline. |
| Optional frontier/debug/DWB/motion observers | 1,000–1,500 | noncanonical or diagnostic | Separate launch paths and flags; not part of canonical C unless explicitly enabled. |
| Offline map/export/rendering | 1,331 | offline | Useful for thesis figures/evaluation; not mission-critical. |

The groups overlap at method boundaries; they are a decomposition of responsibilities, not a second line count to add blindly. The count that is stable and directly measurable is the 7,550-file total above.

## 4. What the central logger actually contains

The 4,092 lines are distributed across the following meaningful blocks. Ranges are current worktree line ranges; the block LOC is approximate and includes helper code shared by adjacent blocks.

| Lines | Block | What it does | Canonical C |
|---|---|---|---|
| 69–179 | utility/configuration helpers | safe calls, time conversion, atomic writes, locks, process/format helpers | yes, selected helpers |
| 182–216 | supervisor/runtime indexing | runtime metric and external-observer integration state | startup/finalization conditional |
| 219–283 | rolling/index state | time-series/index structures and bounded/latest state | yes |
| 286–360 | `RawTFSeriesIndex` | ordered raw TF edge samples, lookup and registration support | yes |
| 363–593 | initialization | parameters, paths, writers, optional child observers, timers, state | yes |
| 595–740 | file/QoS/GT orchestration | directories, evidence flags, GT/contact process setup | yes conditional |
| 742–909 | TF intake | TF buffer/static TF/direct raw TF, edge guard, indexed cache | yes |
| 911–1345 | TF lookup and pose support | exact-time/indexed lookup, fallback, shared pose/trajectory helpers | yes |
| 1387–1662 | maps and GT | latest maps/costmaps, map digests, ground-truth-derived state | yes |
| 1664–1945 | synchronized maps | snapshot requests, map/TF/odom assembly, deferred reconstruction | yes/conditional |
| 1948–2030 | descriptors/subscriptions | evidence descriptor, 41 canonical subscriptions | yes |
| 2032–2199 | release/events | START_RELEASE and event normalization/state transitions | yes |
| 2200–2304 | marks/odom | `mark()`, odom indexing, local trajectory, forensic odom, pose fallback | yes |
| 2305–2552 | cooperation protocol | claims, bids, decisions, distributed status/events/failures | yes |
| 2555–2879 | frontier/diagnostic queries | candidate/assignment diagnostics and optional live analysis | canonical C mostly disabled |
| 2881–3058 | rosout/warnings | warning normalization, planner/costmap health and diagnostic ledger | yes |
| 3060–3269 | sampling timers | telemetry, coverage, health, process resources | yes |
| 3270–3363 | flushing/diagnostics | periodic flush and optional diagnostics | yes/conditional |
| 3365–3713 | finalization | required artifacts, passive export, repaired offloaded timing/health, checks | yes |
| 3716–3788 | manifest | provenance and run manifest | yes |
| 3789–3924 | summary/result | summary and mission-result artifacts | yes |
| 3925–4021 | finalize | shutdown ordering, child-process and artifact finalization | yes |
| 4023–4092 | executor/main | single-threaded executor, signal/shutdown, entry point | yes |

The largest architectural observation is not that one of these blocks is “bad”; it is that raw intake, live derived analysis, diagnostic interpretation, artifact formatting, and fail-closed finalization all share one process and one normal `SingleThreadedExecutor`.

## 5. Classes, functions, ownership, and reachability

### 5.1 Direct production symbols

| File | Lines | Symbol/block | Canonical C reachability | Runtime phase | Main responsibility | Proposed disposition |
|---|---:|---|---|---|---|---|
| `cooperative_experiment_logger.py` | 69–179 | utility/time/atomic-write/safe-call helpers | conditional | runtime/finalization | shared safety, conversion, I/O and timestamps | KEEP LIVE where used; MOVE OFFLINE for derived-only helpers |
| same | 182–216 | supervisor/runtime state | conditional | startup/finalization | external observer state and resource fields | KEEP LIVE for validity; move derived fields offline |
| same | 219–283 | rolling/index structures | yes | runtime | latest/time-series state | RECORD RAW or replace with replay index |
| same | 286–360 | `RawTFSeriesIndex` | yes | runtime/offline bridge | exact TF edge indexing | RECORD RAW; offline equivalent can replace live derived use |
| same | 363–593 | `CooperativeExperimentLogger.__init__` | yes | startup | reads flags, creates state/writers/timers | KEEP LIVE, thin version only |
| same | 595–740 | setup/GT/child process helpers | conditional | startup/shutdown | evidence process orchestration | KEEP LIVE for enabled acceptance streams |
| same | 742–909 | TF callbacks and direct edge functions | yes | runtime | raw TF capture, cache/index, exact lookup | RECORD RAW; retain minimal live lookup only if acceptance requires it |
| same | 911–1345 | TF/pose lookup helpers | yes | runtime | direct/indexed/fallback pose computation | MOVE OFFLINE for metrics; KEEP any live validity lookup proven necessary |
| same | 1387–1662 | map, costmap, GT, pose helpers | yes | runtime | latest maps, digests, state | RECORD RAW; move map analysis offline |
| same | 1664–1945 | sync-map capture/reconstruction | yes | runtime/finalization | map/TF/odom snapshot assembly | MOVE OFFLINE after raw/parity proof |
| same | 1948–2030 | descriptor/subscription creation | yes | startup | 41 subscriptions and QoS | REDESIGN boundary; no immediate removals |
| same | 2032–2199 | START_RELEASE and event callbacks | yes | runtime | readiness/release/event state | KEEP LIVE for acceptance; derived event tables can be replayed |
| same | 2200–2304 | mark/odom callbacks | yes | runtime | odom/raw row and trajectory state | RECORD RAW; keep only minimum live state |
| same | 2305–2552 | cooperation telemetry callbacks | yes | runtime | claims, bids, decisions, status | RECORD RAW; keep live only if termination/acceptance consumes it |
| same | 2555–2879 | optional query/frontier diagnostics | disabled in canonical C | diagnostic | candidate/assignment inspections | NONCANONICAL / DELETE CANDIDATE from canonical logger |
| same | 2881–3058 | rosout/warning handlers | yes by default | runtime | warning/health ledger | DIAGNOSTIC or offline; retain only explicit acceptance health fields |
| same | 3060–3269 | telemetry/coverage/health/process timers | yes | runtime | derived sampling and resource history | MOVE OFFLINE where raw streams suffice |
| same | 3270–3363 | periodic flush/diagnostic callbacks | yes | runtime | writes and flushes | KEEP bounded raw flush; move derived writes offline |
| same | 3365–3713 | required status/export/finalizer | yes | finalization | fail-closed checks and export | KEEP a reduced raw-validity finalizer |
| same | 3716–4021 | manifest/summary/result/finalize | yes | finalization | artifacts and shutdown | KEEP manifest/validity; move thesis metrics |
| same | 4023–4092 | executor/main | yes | runtime | SingleThreadedExecutor and shutdown | KEEP architecture until redesign is validated |
| `forensic_evidence.py` | 84–484 | `ForensicEvidenceWriter` and methods | conditional | runtime/finalization | raw odom/TF/maps/sync/scan files and manifest | RECORD RAW; replace live derived parts offline |
| same | 200–246 | map save/capture/final maps | conditional | runtime/finalization | NPZ/digest/map files | RECORD RAW or finalization-only; no live map analysis required proven |
| same | 248–362 | peer/odom/TF/sync records | conditional | runtime | row writers | KEEP raw capture; assess native bag parity |
| same | 372–452 | scan correction records | optional | runtime | correction evidence | NONCANONICAL/conditional; retain only if thesis metric uses it |
| same | 454–484 | flush/manifest | conditional | runtime/shutdown | durable close | KEEP raw validity |
| `cooperative_ground_truth_observer.py` | 21–100 | connection, timing, contact setup | yes when enabled | startup/runtime | Supervisor connection and 20-ms timing | KEEP LIVE for GT/contact capture |
| same | 101–389 | `main`, `save_metrics`, shutdown | yes when enabled | runtime/shutdown | GT/contact CSV and ready/runtime metrics | KEEP raw GT/contact; move analysis offline |
| `passive_rosbag.py` | 14–49 | topic selection | yes when enabled | startup | selected hidden/raw topic list | KEEP; source of raw evidence |
| same | 52–118 | QoS/command/start/stop | yes | runtime/shutdown | native recorder lifecycle | KEEP |
| same | 121–241 | export/index/retry | finalization | shutdown | export completeness | KEEP raw-validity finalizer |
| same | 244–298 | timing load/semantic completion | finalization | shutdown | required stream completeness | KEEP, but reduce derived duplication |
| `paced_ros2_supervisor.py` | 23–45 | `_ClockPublisherProxy` | yes | runtime | BEST_EFFORT `/clock` | KEEP |
| same | 48–82 | supervisor/main | yes | startup/runtime | clock proxy and pacing | KEEP |
| `debug_observer_node.cpp` | 68–306 | node setup/parameters/pose/warnings | noncanonical | optional runtime | visualization/debug analysis | NONCANONICAL / D candidate |
| same | 319–441 | analysis timer, markers, main | noncanonical | optional runtime | RViz/debug outputs | NONCANONICAL / D candidate |
| `dwb_geometry_capture.py` | all | capture node | optional/noncanonical | runtime | DWB geometry diagnostics | D/diagnostic |
| `dwb_zero_event_capture.py` | all | zero-event capture | optional/noncanonical | runtime | DWB diagnostics | D/diagnostic |
| `motion_supervisor_observer.py` | all | Webots motion observer | noncanonical | runtime | characterization | NONCANONICAL |
| `map_exporter.py` | all | map export/publisher | noncanonical | optional/offline | map helper | MOVE OFFLINE / D |
| `cooperative_map_png_export.py` | all | map image/figure exporters | offline | post-run | visualization | KEEP OFFLINE |

The table groups small helper methods rather than pretending every setter or trivial wrapper is an independent architectural responsibility. The complete function-by-function appendix below lists the remaining meaningful grouped blocks and their dispositions.

## 6. Canonical-C reachability

### 6.1 Reachability classes

| Class | Approx. direct LOC | Evidence |
|---|---:|---|
| Canonical C startup/runtime | 3,400–3,900 | Logger initialization, selected subscriptions/timers, TF/odom/maps/events, GT/contact, passive bag, clock, release. |
| Canonical C finalization | 700–900 | Bag stop/export, semantic completeness, manifest, summary/result, deferred reconstruction and shutdown. |
| Canonical C conditional | 300–600 | Forensic map/sync paths and optional fields enabled by full evidence/contact flags. |
| Exists but disabled by canonical flags | 500–900 | Frontier query/debug paths, diagnostic candidate analysis, optional captures. |
| A/B-specific or strategy-specific | not reliably separable statically | Same logger callbacks receive condition-dependent streams; not all are C-only. |
| D/MRTSP/noncanonical | 400–900 | DWB/MRTSP and legacy diagnostic branches/launches; D is excluded by permanent policy. |
| Offline/render only | 1,331 | Map PNG/export and generic map helper. |
| Test-only/compatibility/dead candidate | 500–1,200 | Branches identified by flags/history/tests; exact deadness needs coverage or removal proof. |

The broad ranges overlap because a function can be loaded in C but only execute a branch conditionally. The strongest static conclusion is that **the 4,092-line logger is not equivalent to 4,092 lines of C runtime work**. Its canonical path reaches the central intake/finalization architecture, but optional query and diagnostic blocks do not execute under the normal C flags.

### 6.2 What is not canonical C

- The C path does not enable `diagnostic_frontier_capture`; the frontier query and candidate-analysis region is therefore not a canonical C hot path.
- `debug_observer_node.cpp` is a separate visualization/debug observer and is not wired into the normal canonical C launch.
- DWB geometry and zero-event capture are optional diagnostic paths, not the ABC C evidence core.
- `motion_supervisor_observer.py` and the generic map exporter are noncanonical characterization/helper paths.
- PNG/figure export is post-run/offline and should not be treated as mission-time observer load.
- The canonical path does enable the central logger, full forensic/contact settings, the external GT observer, passive rosbag, and fast-mode `/clock` proxy.

## 7. Live subscriptions, timers, and processes

### 7.1 Canonical logger entities

The source creates **41 subscriptions and 9 timers** in the normal canonical C logger executor. The current canonical grouping is:

| Family | Count | Live callback purpose | Raw/native equivalent | First-principles status |
|---|---:|---|---|---|
| Odometry | 2 | pose/speed, index, trajectory, forensic row, fallback pose | bag/offline raw odom | raw required; live derivation not generally required |
| Maps/costmaps/shared/local/global | 8 | latest maps, peer-map state, digests, sync support | bag/offline map streams where captured | raw/acceptance subset required; live derived map analysis not established |
| Local/peer map handoff | 2 | peer registration/map evidence | custom/live messages and artifacts | acceptance/scientific; replay candidate |
| Frontier candidates | 2 | candidate/event/evaluation evidence | topic/raw bag if selected | scientific supporting; runtime control is elsewhere |
| Claim/status/event | 6 | cooperation event ledger and state | hidden selected/action/status streams plus custom events | raw cooperation evidence required; live interpretation can move offline if termination does not use it |
| Snapshot/bids/decision/status/event/failure | 12 | distributed task/cooperation telemetry | selected raw streams, custom raw streams | raw required for cooperation analysis; minimum live validity only |
| cmd_vel/cmd_vel_nav | 4 | motion/velocity evidence and idle | raw topic streams | raw required for motion/idle; live analysis not proven required |
| TF / TF static | 2 entities | raw TF, cache/index, pose lookup | raw TF evidence | raw required; live tf2 feed/derived lookups are major redesign candidates but not yet removable |
| release/descriptors/rosout | 3 | release contract, provenance descriptor, warnings | some raw topics/manifest | release/manifest live; rosout mostly diagnostic |
| **Total** | **41** |  |  |  |

Joint-state, scan, selected action/status, and other high-rate streams are omitted from the Python subscription set when the native passive rosbag offload is enabled. This already provides a concrete separation between raw recording and live Python processing, but it does not prove that every remaining custom subscription is scientifically necessary live.

### 7.2 Canonical timers

The source has **9 canonical timers**. Their configured responsibilities are approximately:

| Timer family | Default period | Work | Proposed status |
|---|---:|---|---|
| Telemetry | 1 s | summary/telemetry rows | move derived calculation offline where raw streams suffice |
| Coverage | 0.5 s | known/occupied/free or coverage summary | move offline; raw maps/poses are the source |
| Health | 0.2 s | topic/health state | retain only minimum acceptance health; move diagnostic health offline |
| Console/status | 5 s | operator/status output | diagnostic/optional |
| Process resources | 1 s | resource history | diagnostic unless explicitly a thesis metric |
| Forensic snapshot | 5 s | periodic evidence snapshot | retain raw snapshot only if required; otherwise offline reconstruction |
| Synchronized-map | approximately 50 Hz | snapshot/map/TF/odom work | highest architectural move-offline candidate; preserve raw inputs first |
| Flush | 5 s | file flushing | retain bounded durability, but avoid derived serialization in callback path |
| Maintenance/diagnostic | conditional | cleanup/health/final state | reduce to raw-validity/shutdown obligations |

The exact timer count and periods come from the current logger defaults and launch configuration. The previous timer-consolidation experiment regressed authoritative RTF and is rejected; this audit does not recommend timer consolidation. The architectural question is whether the derived work needs to run live at all.

### 7.3 Minimum live process set

The smallest defensible current live process set is:

1. Webots/controllers/Nav2/SLAM/fusion/cooperation (robot-control stack, not observer work).
2. `paced_ros2_supervisor.py` for authoritative fast-mode `/clock` behavior.
3. A minimal acceptance coordinator for readiness, common `START_RELEASE`, clean horizon, recorder status, and fail-closed raw-stream integrity.
4. Native `rosbag2` recording of the selected raw ROS streams, including hidden action/status topics, migrated joint/scan streams, and `/clock` with the proven BEST_EFFORT/KEEP_LAST-depth-1/VOLATILE profile.
5. External GT/contact capture at the required simulator cadence, or an equivalent raw source whose completeness is independently checked.
6. A bounded shutdown/finalization process that verifies raw recorder completion and writes provenance.

The current Python logger is doing more than this minimum. Whether custom local maps, peer-map evidence, and cooperation events can all be recorded by native rosbag without losing required QoS/metadata must be proved before migration.

## 8. Artifact and field consumer audit

The following inventory is derived from the current writers, finalizer, evaluators, tests, and preserved canonical-C result roots. “Runtime consumer” means a consumer that affects mission execution or an in-mission acceptance decision; a file being written during runtime does not by itself make its content runtime-critical.

| Artifact family | Producer | Consumers found | Runtime consumer? | Thesis/acceptance role | Duplicate/raw source | Proposed status |
|---|---|---|---|---|---|---|
| `run_manifest.json` | logger/runner finalization | finalizer, reports, provenance checks | yes for validity | acceptance/provenance | command/env/source metadata | KEEP LIVE |
| `artifact_finalization.json` | runner/logger finalization | runner, evaluator gate, reports | yes at shutdown | acceptance | status derived from many checks | KEEP REDUCED RAW VALIDITY |
| `summary.json` | logger | reports/evaluator | no during exploration | supporting metrics | overlaps timeseries/events | MOVE OFFLINE |
| `mission_result.json` | logger/runner | final result/evaluator | shutdown only | acceptance/result summary | derives from events and horizon | KEEP SCHEMA OR REPLACE WITH OFFLINE RESULT |
| `events.jsonl` | logger | evaluator/reports/tests | no control consumer found | cooperation/navigation analysis | protocol topics are raw source | RECORD RAW; RECONSTRUCT |
| `goal_decision_ledger.jsonl` | logger | cooperation/assignment analysis | no control consumer found | supporting cooperation metric | overlaps task/status/bid streams | RECORD RAW; RECONSTRUCT |
| `robot1_timeseries.csv`, `robot2_timeseries.csv` | logger | idle/trajectory/coverage evaluators | no control consumer found | motion/utilization/coverage support | odom/cmd_vel/maps/bag | MOVE OFFLINE |
| `coverage.csv` | logger | coverage evaluator/reports | no | primary/supporting outcome depending plan | map/odom/GT raw streams | MOVE OFFLINE |
| `topic_health.csv` | logger | health reports/finalization tests | no control consumer found | diagnostic/acceptance evidence | raw timestamps/bag metadata | REDUCE / MOVE OFFLINE |
| `warnings.jsonl` | logger | warning reports/tests | no control consumer found | diagnostic | `/rosout` raw source | DIAGNOSTIC / MOVE OFFLINE |
| `nav2_diagnostics.jsonl` | logger | reports/tests | no control consumer found | supporting/diagnostic | Nav2/raw rosout/status | MOVE OFFLINE or optional |
| `passive_rosbag_export.json` | passive rosbag | finalizer/evaluator | yes at shutdown | raw evidence completeness | recorder metadata and bag | KEEP |
| `passive_rosbag_export.jsonl` | passive rosbag | diagnostics/reports | no control consumer found | export trace | export summary | OPTIONAL/DIAGNOSTIC |
| QoS override YAML | passive rosbag | recorder | yes at startup | recording compatibility | no duplicate scientific metric | KEEP |
| recorder log | rosbag process | finalizer/reports | status only | acceptance diagnostics | process status | KEEP STATUS, not thesis metric |
| forensic odom CSVs | `ForensicEvidenceWriter` | offline/reports/tests | no control consumer found | raw trajectory source | rosbag odom may overlap | RECORD RAW, choose one canonical source |
| `raw_tf.csv` / `transforms.csv` | forensic writer | TF reconstruction/evaluators/tests | direct live cache only, file is post-run | pose/TF support | rosbag `/tf` may overlap | RECORD RAW; remove duplicate derived file only after parity |
| synchronized-map JSONL | forensic/logger | deferred reconstruction/evaluator | no control consumer found | synchronized-map support | maps/TF/odom raw inputs | MOVE OFFLINE |
| peer-map records | forensic/logger | registration/cooperation reports | acceptance/scientific | peer handoff evidence | local/peer map raw topics | KEEP RAW; derived fields offline |
| map NPZ/digests | forensic writer | map analysis/figures/tests | no control consumer found | map evidence | raw map topic | MOVE OFFLINE or finalization-only |
| supervisor GT CSV | GT observer | coverage/trajectory/error/evaluator | capture process must run | scientific/acceptance | Webots Supervisor source, not ROS bag | KEEP RAW LIVE |
| contact CSV | GT observer | collision/contact evaluator | capture process must run | scientific safety metric | Supervisor source | KEEP RAW LIVE |
| supervisor ready/runtime metrics | GT observer | startup/finalizer/reports | readiness status | acceptance/diagnostic | process state | KEEP MINIMUM |
| scan correction files | forensic writer | scan/SLAM diagnostics | no control consumer found | conditional diagnostic | raw scans/SLAM diagnostics | OPTIONAL / D |
| process resource history | logger/observer | performance reports | no | diagnostic | external sampler can replace | DIAGNOSTIC ONLY |
| figures/PNG/map exports | offline exporter | thesis/reporting | no | presentation | map/coverage raw source | KEEP OFFLINE |

### 8.0 Artifact counts and consumer counts

The preserved canonical result root exposes approximately **20 top-level artifact families**. The expanded field/family inventory in Appendix 21 contains **26 rows** because raw TF/transforms, GT/contact/ready/runtime, recorder metadata/logs, and several derived families need separate ownership analysis. Repository-wide source search found direct evaluator/report consumers for approximately **13 of the 20 top-level families**; approximately **5 families** are primarily acceptance/provenance or recorder-status inputs; and approximately **8 families** are diagnostic, presentation, duplicate, or legacy-contract candidates. These categories overlap where a family has both an acceptance and a reporting consumer, so they are not a claim that 13+5+8 are disjoint files.

At the field level, an exact normalized count is not defensible because schemas are split across CSV headers, JSON objects, JSONL records, NPZ arrays, bag metadata, and versioned finalizer checks. The robust count is therefore: 20 top-level families, 26 audited field/family rows, at least 13 with direct evaluator/report reads, and no evidence that every legacy field is a primary thesis variable. This is the reason the future redesign must begin with a machine-readable raw-contract inventory rather than delete files by name.

### 8.1 Important field-level conclusions

The current code preserves fields such as `received_wall_elapsed_s`, callback-local timestamps, internal cache age, warning text normalization, digest values, and event-ledger ordering. Some are useful for diagnosing executor pressure, but no current thesis metric was found that requires callback arrival ordering or wall-time receipt order as a scientific observable. Simulation timestamps, message headers, source identity, and deterministic event ordering reconstructed from raw records are materially different and should not be conflated with callback timing.

The `artifact_finalization.json` contract currently treats many derived files as required because the current finalizer and tests expect them. That establishes an implementation dependency, not automatically a first-principles scientific requirement. A future contract should distinguish raw-evidence integrity from successful offline derivation and from optional diagnostics.

## 9. Data structures and cache duplication

| Structure | Producer | Consumers | Lifetime/bounds | Duplicate of raw data? | Runtime necessity | Future disposition |
|---|---|---|---|---|---|---|
| `tf_buffer` / listener state | logger TF intake | live lookup/fallback | runtime | yes, overlaps raw `/tf` | only if a live consumer requires lookup | eliminate or minimize after consumer proof |
| direct TF sample/cache state | logger | exact pose/trajectory and forensic rows | runtime/indexed | partly | raw index useful for exact reconstruction | RECORD RAW; offline index |
| `RawTFSeriesIndex` | logger | exact edge lookup/live fast path | runtime | yes, derived index | current fast path accepted | preserve behavior; move index offline eventually |
| `_odom_samples` | logger | exact odom lookup, trajectory, fallback | runtime | yes, raw odom overlap | no control consumer found | offline index candidate |
| latest map/costmap arrays | logger | digest, peer/sync, coverage/health | latest plus copies | yes, raw map overlap | peer/acceptance subset uncertain | raw record; offline analysis |
| NumPy occupancy arrays/digests | forensic/logger | map export/sync/evaluation | per snapshot | derived copy | not control | MOVE OFFLINE |
| trajectory caches | logger | idle/coverage/overlap | runtime timeseries | derived from odom/cmd_vel | evaluation only | MOVE OFFLINE |
| event/state ledgers | logger | summary/mission/evaluator | bounded/unbounded JSONL | derived from protocol topics | live release/finalization subset only | record raw; replay |
| cooperation/task caches | logger | event interpretation and reports | latest/history | overlaps task/status streams | control is in cooperation nodes, not logger | raw record; offline |
| frontier candidate state | logger diagnostic path | candidate reports | conditional | duplicates frontier node outputs | disabled in canonical C | DELETE FROM CANONICAL LOGGER CANDIDATE |
| sync-map request state | logger | sync-map JSONL | timer/runtime | derived snapshot assembly | no control consumer found | MOVE OFFLINE |
| descriptor state | logger | manifest/finalization | run lifetime | provenance not raw robot data | acceptance | KEEP |
| warning/rosout state | logger | health/warnings | history | raw `/rosout` overlap | diagnostic | MOVE OFFLINE/optional |
| health timestamps | logger | topic health/finalization | per-topic latest | raw headers overlap | only required-stream validity needs a reduced form | REDUCE |
| process-resource history | logger | performance report | 1-Hz history | external sampling overlap | no thesis-control need | DELETE from mission contract |

The largest architecture debt is duplicated representation: a raw message is received, interpreted, copied into a latest-state cache, converted into an index, converted into a forensic row, sometimes converted into a derived timer sample, and then the raw stream is also recorded by rosbag. This may be scientifically defensible for a debugging run, but it is not automatically the minimum thesis architecture.

## 10. Serialization, copy, and I/O audit

### 10.1 Live-runtime operations

- Odom callbacks update latest state, append/index samples, update local trajectory, call forensic odometry recording, and can perform direct pose/TF fallback.
- Raw TF callbacks update the direct cache/index, optionally feed the TF buffer, and record raw rows/static transforms.
- Map/costmap callbacks retain latest messages, digest or reshape occupancy data, and support peer/synchronized-map state.
- Ground-truth/contact capture performs per-step Supervisor reads and writes CSV fields; this is external process work rather than Python logger executor work.
- `ForensicEvidenceWriter.save_map()` reshapes NumPy data, computes digests, writes compressed NPZ atomically, and uses flush/fsync semantics for selected evidence.
- `record_odom`, `record_transform`, `record_raw_tf`, `record_sync`, and peer-map methods serialize rows/JSONL during the runtime path.
- Timers serialize derived telemetry, coverage, health, and process-resource records; a five-second flush timer writes buffered files.
- `rosout` is parsed and normalized into warnings/diagnostics; it is not a control input.
- Native rosbag2 performs DDS serialization/IPC independently of Python live callbacks for offloaded joint, scan, hidden action/status, and `/clock` streams.
- Finalization stops/export-checks the bag, repairs or reconstructs offloaded timing/health files, writes summary/mission/provenance, and closes evidence.

The preserved attribution report measured high cumulative time in wait-set/entity eligibility, odometry paths, raw TF, synchronized-map work, coverage and writers. It also demonstrated high WSL logger and ROS-stack pressure and a no-observer diagnostic around 3.16x versus full-observer runs roughly 1.3–1.75x. These are system-level observations, not additive CPU budgets and not proof that one serialization path causes the entire gap.

### 10.2 Duplicated serialization

The strongest concrete duplication candidates are:

1. High-rate raw TF is both retained by the logger/forensic path and recorded through ROS topic infrastructure.
2. Odom is both placed in `_odom_samples`/trajectory state and emitted as forensic rows, while the ROS stream may also be available in the bag.
3. Maps are stored as latest ROS messages, NumPy/NPZ snapshots, digests, synchronized-map records, and sometimes bag/raw topic records.
4. Cooperation/status/action streams are kept as interpreted event ledgers while selected hidden raw topics are also recorded.
5. `/rosout` is preserved as warning summaries while its raw messages could be retained or parsed offline.

These are candidates for a future raw-source ownership decision. They are not authorization to delete any stream in this audit.

## 11. Scientific contract from first principles

The current `docs/THESIS_EXPERIMENT_PLAN.md` defines the active methodology as ABC only, with five seeds and 15 planned primary runs, common synchronized start semantics, C using fixed `frontier_cost_only`, and RTF/idle gates. It does not make every current debug artifact a primary outcome. The following minimum raw evidence is derived from the plan, evaluator source, and current metric/report consumers.

| Thesis question / metric | Minimum raw information | Live derived computation required? |
|---|---|---|
| Explored/known area and coverage curve | time-stamped maps or equivalent source-aware occupancy, simulation time, robot/world frame | no; compute offline |
| Coverage AUC/threshold/completion time | coverage curve inputs and valid `/clock` | no |
| Distance/path length | time-stamped odom/pose or authoritative trajectory | no |
| Coverage per distance | coverage + pose/odom | no |
| Navigation success/failure | goal/terminal/status events and timestamps, Nav2 result/status source | no, except immediate run validity if required |
| Task allocation/cooperation | raw claims, bids, decisions, commits, status/failure events and robot identity | no, except START_RELEASE/readiness safety |
| Duplicate/overlap/path overlap | time-stamped trajectories, map/discovered-cell provenance, task identity | no |
| Collision/contact | complete Supervisor/contact stream with simulation timestamps | capture must remain live or equivalent raw source |
| Idle/productive engagement | odom/cmd_vel, task/goal terminal/dispatch, work-availability and safety state | no; offline interval analyzer |
| Startup/START_RELEASE validity | readiness markers, handoff acceptance, one release timestamp, first dispatch/cmd_vel/motion, `/clock` | minimal live gate and final proof |
| Reproducibility/provenance | world/profile/hash, seed, condition, strategy/gate, install/driver/domain/port, horizon, recorder status | yes at launch/finalization |
| Runtime throughput | `/clock` and wall timing plus clean horizon | live/final gate; detailed attribution is diagnostic |

The plan’s primary scientific comparison is not a claim about arbitrary-start CSLAM, inter-robot loop closure, joint pose-graph optimization, optimal task allocation, or universal scalability. Those claims require separate evidence and remain outside the validated current scope.

The raw evidence requirement is therefore broad but conceptually simple: preserve authoritative robot/simulator streams and provenance; calculate derived metrics deterministically after the mission; keep only the live checks needed to prevent an invalid run from being accepted.

## 12. Fail-closed contract audit

The current finalizer is stronger than “file exists”: it checks recorder completion, required topic counts, `/clock`, exporter `complete=true`, reconstruction and final artifact state. That is an improvement over the earlier generic file-presence finalizer. However, it still conflates three different questions:

1. **Raw run validity:** Did the intended world/condition/seed/horizon run, did the common release contract hold, and are the required raw streams complete and readable?
2. **Offline reconstruction validity:** Can the deterministic evaluator reconstruct the primary metrics from those raw streams?
3. **Presentation/debug completeness:** Were every current summary, warning, diagnostic, ledger, figure, and legacy file generated?

The current contract treats much of (2) and (3) as part of final artifact success. This is safe but expensive and circular: a live logger-derived file is required because finalization expects it, while the file’s scientific necessity may not have been independently established.

### 12.1 Future contract proposed, not implemented

The minimum future fail-closed design should be two-stage:

**Stage R — runtime/raw validity**

- exact manifest/provenance and selected source/install/driver checks;
- valid world/profile/condition/strategy/seed/horizon;
- both-robot readiness/handoff and exactly one common `START_RELEASE`;
- zero pre-release exploration dispatch/motion;
- authoritative `/clock` and clean `SIM_TIME_COMPLETE`;
- recorder exited cleanly;
- `/clock` and every required raw stream present, readable, and internally timestamp-valid;
- GT/contact raw capture complete when enabled;
- no forbidden stale prefix or provenance violation.

**Stage E — offline evaluation validity**

- deterministic replay succeeds;
- required primary metrics are present and schema-valid;
- evaluator output is complete and internally consistent;
- optional diagnostics are reported separately;
- a missing optional/debug artifact cannot turn an otherwise scientifically valid raw run into a false “complete” result.

This proposal does not change the current contract. It identifies the smallest future acceptance redesign that avoids making all live-derived artifacts mandatory.

## 13. Test-suite fossilization audit

The direct observer/evidence tests contain legitimate scientific/acceptance tests and implementation-fossil tests. The existence of a test is evidence of a dependency, not proof of scientific necessity.

| Test/file group | What it protects | Control | Science | Acceptance | Legacy schema | Diagnostic | Thin architecture impact |
|---|---|---:|---:|---:|---:|---:|---|
| `test_experiment_logger_runtime.py` (52 tests) | logger callbacks, TF/odom indexes, sync reconstruction, finalization, CSV/schema, contact, coverage, shutdown | some | some | many | many | some | split into raw replay/parity and acceptance tests; drop live-implementation assumptions |
| `test_cooperative_ground_truth_observer.py` (7) | GT/contact sampling, timing, fields | no | yes | yes | some | some | retain raw capture and timing tests; move derived checks offline |
| `test_passive_rosbag_offload.py` (10) | selected topics, QoS, hidden topics, `/clock`, export completeness, runtime dependency | no | yes | yes | no | some | retain nearly all, add raw replay completeness tests |
| `test_cooperative_regression.py` (66) | cooperative behavior/regression artifacts | yes | yes | some | some | some | retain control/metric tests; detach from logger-specific files |
| `test_cooperative_trial_fast.py` (59) | runner, horizon, provenance, finalization and commands | no | no | yes | some | some | retain, revise required artifact list after contract review |
| `test_cooperative_regression_artifacts.py` (2) | artifact presence/shape | no | some | yes | yes | no | replace file-list assumptions with raw/evaluator contract |
| `test_distributed_logger_integration.py` (2) | integration/logging | no | some | some | yes | yes | replay raw protocol events rather than live logger internals |
| `test_cooperative_map_png_export.py` (12) | offline figures | no | supporting | no | no | no | retain offline |
| `test_paced_ros2_supervisor.py` (2) | `/clock` QoS/proxy | no | yes | yes | no | no | retain |
| DWB/Nav2/scan/unknown-pose diagnostic groups | optional diagnostics/protected robot behavior | yes/diagnostic | conditional | no | no | yes | remain separate from thin recorder |

The narrow direct observer groups total approximately 84 named tests (52+7+10+2+2+5+4+2); the broader relevant test surface is approximately 318 tests. The numbers are AST-discovered test-function counts and may include parameterized/helper cases differently from a test runner’s collected count.

Tests that most clearly fossilize the current architecture include exact live callback ordering, exact CSV headers/row placement, assumptions that derived files exist before finalization, and direct construction of logger internals. Those tests should be rewritten only after a replacement raw/evaluator contract exists. Tests for START_RELEASE, raw stream completeness, contact completeness, `/clock`, provenance, finalization fail-closed behavior, and deterministic metric parity legitimately remain.

## 14. Git archaeology

`cooperative_experiment_logger.py` grew incrementally. Git history does not always record an explicit requirement, so motives are reported only where the commit subject or source evolution supports them.

| Commit / date | Observed growth or theme | Evidence-supported interpretation | Current architectural status |
|---|---|---|---|
| `2bbc8df` 2026-07-29 | +309 structured cooperative observability | initial central logger/observability expansion | core, but later responsibilities accumulated |
| `0d8858a` | continuous cooperative telemetry | ongoing runtime observation | mixed core/derived |
| `34bc8f5`, `5fb216f`, `1591de2` | timing, shutdown, idempotence, teardown | reliability/finalization hardening | acceptance-relevant subset remains |
| `5f2eaa7` | bounded diagnostics/teardown | diagnostic and cleanup additions | partly optional |
| `02341b5`, `40730ea` | distributed assignment and agreement/navigation outcomes | cooperation reporting | raw streams remain scientific support; live interpretation questionable |
| `0bf7faa`, `6373a08` | idle explanation and allocator validation | utilization/evaluation reporting | offline candidates |
| `2b59311` | literature-backed allocation reporting | task/allocation observability | not evidence that logger controls allocation |
| `23c16e2`, `1d9cfce`, `0efcb1e` | Burgard/unknown-pose forensic and accuracy reporting | unknown-pose evidence additions | preserve raw/acceptance fields; derived analysis candidate |
| `94b1033` | local maps | map/evidence expansion | raw source needed; derived work moveable |
| `0c97d46`, `5a9c0c5` | scan and scan-forensic additions | forensic/diagnostic evidence | conditional; native bag/offline candidate |
| `692b4f0` | frontend shutdown | lifecycle/finalization | keep shutdown semantics, simplify derived output |
| `cbd04d7` | unknown-pose changes | registration/forensic support | protected estimator is outside this audit |
| `e4018bb` 2026-09-02 | +1,558 logger / +53 forensic | largest visible growth phase; source checkpoint combines many evidence/finalization additions | strongest refactoring target, but exact motive not fully recoverable |

`passive_rosbag.py` is present in the current worktree but has no comparable long Git history in the inspected path; its provenance is in reports/source changes rather than a mature commit lineage. This is a maintenance risk: the offload path should become the explicit raw-source ownership layer before more live evidence is added.

## 15. Cross-module duplication

| Concept | Live logger | Forensic writer | Offline/evaluator | Duplication finding |
|---|---|---|---|---|
| Trajectory | odom samples/local trajectory/timeseries | odom CSV | idle/coverage/overlap analyzers | same pose history represented multiple times |
| TF lookup/index | tf2 buffer, direct cache, `RawTFSeriesIndex` | raw TF/transforms | deferred sync/pose reconstruction | raw stream plus live and offline indexes |
| Map counting | latest maps/digests/coverage timers | NPZ/map records | map exporter/evaluator | live derived map work overlaps offline analysis |
| Event interpretation | callback state/event ledger/summary | some event files | cooperation evaluator/report | raw protocol and derived ledger both retained |
| Terminal state | callbacks/mission result | no separate source | runner/evaluator | live interpretation may be reduced to raw status + finalizer |
| Idle | live telemetry/idle bookkeeping | timeseries | exact feasible-work analyzer | direct duplication; offline analyzer is authoritative for later reports |
| Health/warnings | rosout parser/topic health | no raw duplicate in current logger path | warning reports | diagnostic interpretation can be offline from raw rosout/status |
| Coverage | timer/sampling | map evidence | offline coverage evaluator | strongest move-offline candidate |

The source justifies an architectural simplification even though no single callback explains 100% of the RTF deficit: the no-observer control is materially faster, the live logger is a large CPU consumer, and the logger performs overlapping raw capture and derived analysis. This is evidence for reducing live derived responsibility, not proof that any one topic or callback may be removed without a new contract.

## 16. Candidate sets

### Set 1 — Move offline

| Candidate | Exact current work | Approx. affected direct LOC | Why it is a candidate | Required proof |
|---|---|---:|---|---|
| coverage/known-cell curves | logger coverage timer and map-derived state | 150–300 | raw maps/poses are sufficient | deterministic curve parity, timestamp policy, map-source parity |
| idle/productive-work analysis | telemetry/mark/trajectory interpretation | 200–450 | exact analyzer already exists offline | parity for terminal/dispatch/work-availability rules |
| synchronized-map derived frames | live sync timer/reconstruction | 250–500 | raw maps/TF/odom can be replayed | same snapshot selection, frame, transform, ordering and missing-data behavior |
| warnings/topic health | rosout parser and health timer | 150–300 | raw messages/timestamps exist | classify acceptance-critical health separately; no lost fail-closed signal |
| cooperation ledgers | callback interpretation and summary | 250–500 | raw bids/status/events exist | deterministic event replay and duplicate handling |
| trajectory/distance/overlap | local trajectory/timeseries | 200–450 | odom/cmd/map raw sources | exact metric parity and frame/pose convention |
| map NPZ/digest/figure preparation | live map copies and exporters | 250–600 | raw map topics are sufficient | map resolution/source identity and offline export parity |
| process-resource history | 1-Hz timer | 50–120 | external sampling/diagnostic only | remove from scientific contract; report separately |

### Set 2 — Delete from final experiment, subject to contract review

These are not “safe to delete now”; they are deletion candidates because no primary scientific or acceptance consumer was found in the current source:

- optional frontier query/candidate diagnostics disabled in canonical C;
- `debug_observer_node.cpp` RViz markers and decision-map publishing;
- DWB geometry and zero-event diagnostics outside the ABC C path;
- motion characterization observer;
- process-resource history if no approved thesis metric retains it;
- warning normalization duplicates if raw `/rosout` and required runtime health checks remain;
- duplicate derived summaries whose values are fully reproducible from the authoritative offline evaluator;
- duplicate raw TF/transforms/NPZ representations after one canonical raw source is selected and parity is demonstrated.

### Set 3 — Must remain live now

- accepted Webots/controllers/SLAM/Nav2/fusion/cooperation behavior;
- fast-mode `/clock` proxy and valid simulation-time progression;
- readiness/handoff/common `START_RELEASE` gate and pre-release zero-dispatch/zero-motion check;
- provenance and stale-overlay fail-closed checks;
- a raw recorder for every selected required stream, including hidden action/status streams and `/clock` QoS compatibility;
- GT/contact raw capture at the required cadence or an independently equivalent raw source;
- bounded recorder lifecycle, shutdown flush, and raw completeness/finalization checks;
- any custom map/peer/registration raw stream not yet proven reproducible from native bag input;
- the accepted robot-control and protected registration/SLAM/Nav2/fusion semantics.

## 17. Thin-recorder target architecture

The target is a two-stage architecture, not a new concurrent executor.

```text
ROBOT CONTROL STACK
  Webots/controllers + SLAM + Nav2 + fusion + frontier/cooperation/traffic
                              |
                              v
LIVE VALIDITY / RAW CAPTURE
  /clock + native rosbag2 selected streams + GT/contact raw capture
  minimal START_RELEASE/readiness/horizon/provenance coordinator
  bounded flush/shutdown + recorder/raw-stream integrity checks
                              |
                              v
OFFLINE REPLAY / EVALUATOR
  maps/TF/odom synchronization -> trajectories -> coverage/distance/idle/overlap
  cooperation/task reconstruction -> navigation/health/warnings
  artifact schemas -> thesis_metrics.json / tables / figures
```

### 17.1 Proposed live components

1. Existing robot-control nodes, unchanged.
2. Existing `paced_ros2_supervisor.py`, unchanged.
3. Native `passive_rosbag.py` extended only as needed to own all raw streams that are proven recordable with required QoS and hidden-topic behavior.
4. GT/contact raw capture and ready marker, with per-simulation-time sample integrity.
5. Minimal coordinator that does not calculate coverage, idle, trajectory, synchronized maps, warnings, or cooperation summaries; it only verifies the start/horizon/recording contract.
6. A reduced finalizer that checks raw data and launches the offline evaluator.

### 17.2 Proposed offline components

- deterministic TF/odom/map index and synchronized-map replay;
- coverage/known-area/AUC/threshold calculation;
- trajectory/distance/overlap and idle/productive-work analysis;
- cooperation/task/bid/decision reconstruction;
- navigation/terminal-state and health/warning analysis;
- contact/collision analysis;
- final metrics, CSV/JSON/NPZ/figures, and thesis tables.

Existing evaluator/analyzer code can be reused where it consumes raw inputs without logger-specific receive-time assumptions. Code should not be reused unchanged where it assumes a live callback order, a particular intermediate file being available before finalization, or the old central logger’s mutable caches.

### 17.3 Rough size target

No exact post-redesign LOC is claimed. A defensible architectural target is approximately **800–1,500 live Python observer/acceptance LOC**, plus the native recorder, GT/contact recorder, and existing offline evaluator surface. The target is a range because custom map/peer/registration raw capture and GT/contact integration may remain project-specific. The current 4,092-line logger should not be mechanically shrunk to a target without parity tests.

## 18. Existing performance evidence and limits

| Evidence | Measured result | What it supports | What it does not prove |
|---|---|---|---|
| Observer-disabled matched control | approximately 3.164636x | observer/evidence workload is materially associated with wall-time cost | no single observer path is causal; control is not complete-evidence valid |
| Accepted full-observer baseline | 1.7527587419627202x | complete-evidence path below hard gate | stable mean, exact decomposition, or one root cause |
| Accepted-source repeat | 1.4720904744951158x | substantial run-to-run variance exists | a statistically estimated distribution |
| Host observer attribution | latest sampled full-observer run 1.3212411212239743x | Webots and WSL/observer pressure overlap | sampler run is a production RTF baseline; causality |
| Webots engine attribution | `webots-bin.exe` measured, e.g. 92.984 CPU s / 288.11 MiB in authoritative attribution record | wrapper-only measurement was invalid; engine is real work | Webots alone explains full gap |
| Logger/RCL attribution | logger near one normalized core; high wait-set/entity and callback work | live Python observer is a large consumer | logger alone causes deficit |
| Prior micro-optimizations | raw-TF edge/live-edge accepted improvements; serializer, coverage fast path, timer consolidation, TF dedupe/single-pass and odom scalar reuse rejected after regressions | local changes are risky and require full parity | no architecture-wide answer |

The evidence is sufficient to justify a first-principles observer simplification audit and a parity-first redesign plan. It is not sufficient to name a single production optimization, and this audit makes no such change.

## 19. Required quantitative answers

### Q1. Approximately how much of the 6,000+ lines is truly part of robot runtime control?

Approximately **0 direct logger LOC** command the robots. The actual robot-control stack is outside the observer subsystem. `paced_ros2_supervisor.py` contributes approximately 86 LOC of clock/fast-mode runtime infrastructure, and the readiness/termination boundary in the runner contributes additional acceptance logic. The safe statement is therefore **0–200 observer-adjacent LOC for control/clock infrastructure**, not thousands of logger lines.

### Q2. Approximately how much exists purely for observation/evaluation/forensics?

Approximately **5,500–7,000 of the direct 7,550 LOC** is observation, forensic capture, derived evaluation, diagnostics, or offline rendering. The lower bound excludes raw acceptance/provenance glue; the upper bound includes the full offline/rendering surface. The central logger is overwhelmingly passive/evaluation-oriented even though it currently runs live.

### Q3. How much of `cooperative_experiment_logger.py` actually executes in canonical C?

Static launch/flag analysis supports approximately **3,100–3,500 of 4,092 LOC** as canonical-C-reachable, with approximately 500–900 LOC in optional/diagnostic/query branches not enabled by normal C flags. Exact executed LOC is unknown without runtime coverage, which this task was not authorized to collect.

### Q4. What are the largest responsibility groups inside its 4,092 lines?

The largest groups are central initialization/subscription/state setup; TF/pose/cache/index work; map/GT/synchronized-map processing; cooperation/event interpretation; live telemetry/coverage/health timers; and finalization/manifest/summary/result generation. The optional frontier diagnostic region is also large but is not canonical-C runtime work.

### Q5. How much is noncanonical, diagnostic, legacy, compatibility, or dead?

For the direct scope, approximately **1,000–2,000 LOC** is clearly optional/noncanonical/diagnostic across the logger and separate observer files; approximately **1,500–3,000 LOC** is a plausible legacy/derived/offline-removal candidate. Exact dead-code LOC is **unknown** because static reachability cannot prove that a branch has no historical or alternate launch consumer. No code is labeled dead solely because it is not used by C.

### Q6. Which current artifacts are genuinely required to answer the thesis questions?

Raw simulation time and provenance; robot odometry/pose and motion streams; maps/occupancy or equivalent coverage source; task/cooperation protocol streams; navigation outcomes; complete GT/contact records; start/release evidence; and valid recorder/horizon status. Derived coverage, distance, idle, overlap, cooperation statistics, warning analysis, synchronized-map tables, and figures can be generated offline if raw inputs and conventions are preserved.

### Q7. Which artifacts are only required because old tests/finalization expect them?

The strongest candidates are duplicate summary/timeseries/health/warning/diagnostic files, live synchronized-map JSONL, process-resource history, exact internal callback receive-time fields, and intermediate ledger formats whose values are reproducible from raw records. The exact set must be confirmed by a replacement contract; “only tests expect it” is not by itself permission to remove it.

### Q8. Which live observer responsibilities could disappear completely rather than merely be optimized?

Live coverage computation, live idle/productive-work calculation, live trajectory/overlap calculation, live warning normalization, process-resource history, most live synchronized-map derivation, optional frontier/debug/DWB analysis, and live summary/figure generation are deletion-from-live candidates. They cannot disappear from the experiment until raw replay parity and acceptance design are implemented.

### Q9. Which information must remain recorded raw?

At minimum: `/clock`; exact provenance/configuration; common START_RELEASE/readiness markers; odometry/pose and relevant velocity; command/goal/terminal events; maps/costmaps and source identity needed for coverage/fusion interpretation; cooperation/task/bid/decision/status/failure events; TF/static TF; GT/contact; hidden action/status streams selected by the current evidence contract; and recorder completeness metadata. Exact custom stream coverage must be finalized from topic/QoS inventory before redesign.

### Q10. Which derived information can be generated after the mission?

Coverage curves/AUC/threshold times; distance and trajectory; overlap/duplicate analysis; idle/productive-work intervals; synchronized-map frames; TF/pose joins; cooperation summaries; warning/topic health; navigation summaries; figures and tables; and final thesis metrics can be offline-derived, subject to deterministic conventions and raw data completeness.

### Q11. Does exact callback receive ordering / `received_wall_elapsed_s` have scientific significance?

No scientific significance was established in the inspected plan, evaluator, or reports for exact Python callback receive ordering or `received_wall_elapsed_s` as a primary thesis metric. Simulation/header timestamps, source identity, event semantics, and deterministic replay order are significant. Receive-wall ordering remains useful for performance/diagnostic analysis and might be needed by a narrowly defined acceptance check, so it should be classified as optional until explicitly removed by contract review.

### Q12. Does the source justify retaining a 4,092-line central live Python logger?

No. The source justifies retaining a passive raw/acceptance observer, but not the current central live mixture of raw capture, derived metrics, diagnostics, and final artifact formatting. The source also does not justify deleting the logger immediately; the replacement must preserve raw/evaluator parity.

### Q13. What is the smallest defensible live observer architecture?

A native/raw recorder for required ROS streams; a minimal readiness/START_RELEASE/horizon/provenance coordinator; GT/contact raw capture; bounded flush/shutdown; and a raw completeness finalizer. Derived metrics and reports should be offline. Custom streams whose native recording equivalence is not proven remain live/raw until parity exists.

### Q14. Is a thin-recorder/offline-evaluator redesign justified by existing evidence?

**Yes, as a parity-first architecture task.** The observer-disabled control is much faster, the full observer is CPU-heavy, and the source contains duplicated raw/derived representations. No single callback optimization has reliably crossed the gate. This supports redesign analysis, not an immediate production migration or a claim that the redesign will reach 2.0x.

### Q15. What should be the first implementation step if redesign is authorized?

Create a read-only/raw-contract fixture from one preserved complete C bag and enumerate every required raw stream, QoS, timestamp, source identity, and acceptance marker. Then implement one offline replay of coverage, trajectory, idle, and synchronized-map artifacts against that fixture while retaining the current runtime path for comparison. Do not remove a live subscription until fixture parity and fail-closed raw completeness pass.

### Q16. What should explicitly not be done again?

Do not repeat rejected timer consolidation, raw-TF buffer dedupe, TF single-pass materialization, odometry scalar reuse, raw-TF serializer changes, coverage-set fast paths, speculative executor concurrency, observer disabling as a production fix, evidence downsampling, sensor/map reduction, or changes to protected SLAM/Nav2/fusion/cooperation/traffic/START_RELEASE behavior.

## 20. Function-by-function appendix

The table below is the required source-level ownership map. Trivial getters and one-line wrappers are grouped with their parent block; every meaningful production block in the direct transitive scope is represented.

| File | Lines | Symbol/block | Canonical C reachability | Runtime phase | Responsibility | Reads | Writes | Artifact/output | Scientific necessity | Candidate disposition |
|---|---:|---|---|---|---|---|---|---|---|---|
| `cooperative_experiment_logger.py` | 69–179 | safe-call/time/atomic I/O/process helpers | conditional | all | common utility behavior | params/state | files/diagnostic state | atomic JSON/CSV support | only selected safety/format rules | KEEP LIVE / MOVE OFFLINE |
| same | 182–360 | supervisor state, rolling indexes, `RawTFSeriesIndex` | yes | runtime | indexed timing/TF state | TF/clock | caches/indexes | raw TF/derived lookup | raw timestamps; live index not yet proven | RECORD RAW |
| same | 363–593 | logger initialization | yes | startup | flags, writers, timers, child processes | launch/env | state/files/processes | directories/descriptors | acceptance setup | KEEP THIN |
| same | 595–740 | evidence/GT/contact orchestration | conditional | startup/shutdown | starts external evidence processes | flags/paths | process handles | GT/contact markers | contact/GT raw capture | KEEP LIVE |
| same | 742–909 | TF callbacks/static/direct edge guard | yes | runtime | raw TF and cache/index | `/tf`, `/tf_static` | buffer/cache/files | raw TF/transforms | raw TF/pose reconstruction | RECORD RAW; live lookup review |
| same | 911–1345 | TF lookup, fallback, pose helpers | yes | runtime | exact/indexed pose | indexes/odom/TF | state | trajectories/sync inputs | derived pose offline candidate | MOVE OFFLINE after parity |
| same | 1387–1662 | map/costmap/GT helpers | yes | runtime | latest maps, digest, source state | map/GT | caches/files | map/peer/coverage support | raw maps; derived analysis no | RECORD RAW |
| same | 1664–1945 | sync request/reconstruction | yes | runtime/finalization | map/TF/odom joins | caches/raw state | JSONL/state | sync frames | offline candidate | MOVE OFFLINE |
| same | 1948–2030 | descriptor/subscription builder | yes | startup | QoS/subscription ownership | flags/topics | subscriptions/manifest | descriptor | acceptance/raw capture | KEEP/REDESIGN |
| same | 2032–2199 | release/event normalization | yes | runtime | START_RELEASE/events | protocol/release | event state/JSONL | events/mission state | release and cooperation raw | KEEP MINIMUM/REPLAY |
| same | 2200–2304 | mark/odom callbacks | yes | runtime | samples/index/forensic rows | odom/clock | indexes/trajectory/files | odom/timeseries | raw odom; derived offline | RECORD RAW |
| same | 2305–2552 | cooperation callbacks | yes | runtime | claims/bids/decisions/status | protocol topics | ledgers/state | events/goal ledger | raw cooperation | RECORD RAW |
| same | 2555–2879 | frontier/candidate diagnostics | no in C flags | optional runtime | candidate analysis | frontier/nav | diagnostic state | diagnostic files | not established for C | DELETE CANDIDATE |
| same | 2881–3058 | rosout/warning/health interpretation | yes by default | runtime | warning and diagnostics | `/rosout`, status | warning/health files | warnings/nav2 diagnostics | mostly supporting | MOVE OFFLINE |
| same | 3060–3269 | telemetry/coverage/health/resource timers | yes | runtime | derived sampling | caches/topics | CSV/health/resource | timeseries/coverage/health | metrics offline | MOVE OFFLINE |
| same | 3270–3363 | flush/maintenance | yes | runtime/shutdown | buffer/file flushing | writers/state | files | durability | raw durability | KEEP MINIMUM |
| same | 3365–3713 | artifact status/passive export/reconstruction | yes | shutdown | fail-closed finalization | files/export | status/repaired files | finalization/export | acceptance | KEEP REDUCED |
| same | 3716–4021 | manifest/summary/result/finalize | yes | shutdown | provenance and summaries | all status | JSON/summary/result | final artifacts | manifest/result; summaries offline | KEEP MANIFEST |
| same | 4023–4092 | executor/main/signal shutdown | yes | runtime/shutdown | single-threaded dispatch | callbacks/signals | process exit | status/log | runtime lifecycle | KEEP until replacement |
| `forensic_evidence.py` | 84–199 | `ForensicEvidenceWriter` init/path state | conditional | startup | evidence paths/files | flags | writers | manifest/path state | setup | KEEP/THIN |
| same | 200–246 | map save/capture/final map | conditional | runtime/shutdown | reshape/digest/NPZ | OccupancyGrid/arrays | NPZ/files | map artifacts | raw map source; derived NPZ offline | MOVE OFFLINE |
| same | 248–362 | peer/odom/TF/sync record methods | conditional | runtime | row serialization | messages/state | CSV/JSONL | raw records | raw records, but native bag parity needed | RECORD RAW |
| same | 372–452 | scan correction record methods | optional | runtime | correction JSONL | scan/diagnostic | files | correction artifacts | conditional diagnostic | D/OPTIONAL |
| same | 454–484 | flush/manifest | conditional | shutdown | durability/metadata | writers | files | forensic manifest | acceptance/raw validity | KEEP |
| `cooperative_ground_truth_observer.py` | 21–100 | timing/connection/contact setup | enabled C | startup | Supervisor connection and sample timing | Webots | handles/files | ready marker | GT/contact validity | KEEP LIVE |
| same | 101–389 | `main`, sample/write/shutdown | enabled C | runtime/shutdown | GT/contact raw capture | Supervisor state | CSV/JSON | GT/contact/runtime metrics | collision/coverage support | KEEP RAW; analyze offline |
| `passive_rosbag.py` | 14–49 | topic lists/recorded topics | enabled C | startup | selected raw topic set | config | command data | topic manifest | raw evidence | KEEP |
| same | 52–118 | QoS/command/start/stop | enabled C | runtime/shutdown | recorder lifecycle | graph/config | process/files | recorder logs | raw/acceptance | KEEP |
| same | 121–241 | export/index/retry | enabled C | shutdown | bag export | bag/process | JSON | export status | acceptance | KEEP |
| same | 244–298 | offloaded timing/completion | enabled C | shutdown | semantic raw checks | bag/index | JSON/derived timing | export metadata | acceptance; reduce duplicates | KEEP REDUCED |
| `paced_ros2_supervisor.py` | 23–45 | clock proxy | enabled C | runtime | BEST_EFFORT clock | ROS time | `/clock` | clock stream | fundamental timing | KEEP |
| same | 48–82 | supervisor/main | enabled C | startup/runtime | fast clock operation | params | publisher/process | clock status | timing | KEEP |
| `debug_observer_node.cpp` | 68–306 | node setup/params/pose/warnings | not canonical C | optional runtime | debug subscriptions/TF | maps/TF | markers/warnings | RViz/debug | no primary C necessity | D |
| same | 319–441 | analysis timer/markers/main | not canonical C | optional runtime | visualization | caches | markers | debug output | diagnostic | D |
| `dwb_geometry_capture.py` | all | capture callbacks/writers | optional | diagnostic | DWB geometry | Nav2/DWB | files | DWB artifacts | not ABC core | D |
| `dwb_zero_event_capture.py` | all | zero-event capture | optional | diagnostic | negative evidence | Nav2 | files | zero-event artifacts | not ABC core | D |
| `motion_supervisor_observer.py` | all | motion characterization | noncanonical | diagnostic | Webots motion | Supervisor | CSV | characterization | unfinished/optional | D |
| `map_exporter.py` | all | map helper/export | noncanonical/offline | post-run/optional | map output | maps | output | map artifact | supporting | MOVE OFFLINE |
| `cooperative_map_png_export.py` | all | map/figure rendering | offline | post-run | thesis figures | NPZ/maps | PNG/CSV | figures | presentation | KEEP OFFLINE |

## 21. Artifact/field appendix

The following expands the artifact table into the fields most relevant to the contract. Exact field names vary across artifact versions; the status is therefore tied to the current writer/evaluator families rather than an invented universal schema.

| Artifact / field | Producer | Consumer(s) | Runtime consumer? | Thesis metric? | Acceptance gate? | Duplicate source? | Exact legacy semantics necessary? | Proposed future status |
|---|---|---|---|---|---|---|---|---|
| condition/seed/strategy/gate | manifest/runner | finalizer/reports | yes | reproducibility | yes | command/env duplicates | value necessary, file shape not necessarily | KEEP provenance |
| world/profile/hash/install/driver | runner/manifest | provenance/finalizer | yes | reproducibility | yes | env/command | content necessary | KEEP |
| robot readiness/handoff | logger/runner | release gate | yes | startup validity | yes | events/launch logs | exact common release proof necessary | KEEP minimal |
| `START_RELEASE` sim timestamp | logger/runner | evaluator/reports | yes | startup validity | yes | events/clock | semantic event necessary | KEEP |
| pre-release goal/cmd/motion markers | logger/evaluator | finalizer/reports | yes at gate | safety validity | yes with raw topics | exact derived file not necessary | raw + offline check |
| `/clock` rows/count/profile | rosbag/paced supervisor | exporter/finalizer/evaluator | yes at final gate | RTF/time | yes | raw authoritative stream necessary | profile compatibility necessary | KEEP |
| odom timestamp/pose/twist/frame | logger/bag | trajectory/idle/coverage | no control | yes | duplicate cache/index | header/source values necessary | RECORD RAW |
| TF timestamp/frame/transform/static | logger/bag | pose/sync/map | no control | supporting | sometimes | direct cache/index/bag | raw values necessary | RECORD ONE RAW SOURCE |
| occupancy/map metadata/cells | logger/bag/forensic | coverage/overlap/sync | no control | primary/support | sometimes | NPZ/digest/latest cache | map semantics necessary | RECORD RAW |
| task/bid/decision/status/failure | logger/bag | cooperation evaluator | no control | supporting/possibly primary | sometimes | event ledger | protocol values/order necessary | RECORD RAW |
| goal/terminal/navigation status | logger/bag/Nav2 | navigation metric | no control | primary/support | yes if gate | ledger/status duplicate | semantic terminal event necessary | RECORD RAW |
| GT pose/velocity | Supervisor | trajectory/error/coverage | no control | support | yes when enabled | not necessarily ROS raw | sample time/value necessary | KEEP RAW |
| contact points/events | Supervisor | collision metric | no control | primary safety | yes | no equivalent inferred source | complete raw contact necessary | KEEP RAW |
| coverage rows | logger | evaluator/reports | no | support/primary depending plan | current finalizer may require | map/odom duplicate | numerical output not raw requirement | MOVE OFFLINE |
| timeseries fields | logger | idle/trajectory reports | no | support | current finalizer/tests | odom/cmd/state | exact file layout not established as scientific | MOVE OFFLINE |
| health/topic age | logger | diagnostics/finalizer | no except required raw stream check | diagnostic | current finalizer subset | headers/bag metadata | exact callback age not established | REDUCE |
| warning text/category | logger | warning reports | no | diagnostic | no primary gate found | raw `/rosout` | category useful, arrival wall time not | MOVE OFFLINE |
| resource CPU/RSS history | logger/sampler | attribution reports | no | diagnostic | no | external sampler | no | DELETE FROM SCIENTIFIC CONTRACT |
| synchronized-map frame | logger/forensic | map/pose evaluator | no | support | current artifact | raw map/TF/odom | derived frame can be regenerated | MOVE OFFLINE |
| NPZ/digest/PNG | forensic/exporter | figures/map audit | no | presentation/support | no primary | raw map | rendered bytes not | OFFLINE |
| bag export `complete`/counts | passive rosbag | finalizer | yes at shutdown | evidence validity | yes | recorder log partly duplicates | semantic status necessary | KEEP |
| final `summary`/`mission_result` | logger/runner | reports/evaluator | shutdown only | summary | yes current | offline evaluator can produce | semantic result needed, old writer not | KEEP REDUCED |

## 22. Test appendix

| Test/file | What it protects | Control | Science | Acceptance | Legacy schema | Diagnostic | Would need change under thin architecture? |
|---|---|---:|---:|---:|---:|---:|---|
| `test_experiment_logger_runtime.py` | logger callback/index/TF/odom/sync/finalizer behavior | partial | partial | yes | yes | partial | yes; split raw replay from live callback tests |
| `test_cooperative_ground_truth_observer.py` | Supervisor timing, GT/contact fields, shutdown | no | yes | yes | partial | partial | retain raw capture tests; move metric tests offline |
| `test_passive_rosbag_offload.py` | topics, QoS, hidden topics, `/clock`, export, missing data, runtime dependency | no | yes | yes | no | partial | retain and extend with all custom raw stream fixtures |
| `test_cooperative_trial_fast.py` | command, provenance, horizon, shutdown, finalization | no | no | yes | partial | partial | update required-artifact contract only after design |
| `test_cooperative_regression.py` | cooperative protocol/regression behavior and artifacts | yes | yes | partial | partial | partial | retain metric semantics, decouple file ownership |
| `test_cooperative_regression_artifacts.py` | artifact presence and schema | no | partial | yes | yes | no | replace file-list checks with raw/evaluator contract |
| `test_distributed_logger_integration.py` | distributed logging integration | no | partial | partial | yes | yes | replay raw events and verify parity |
| `test_cooperative_map_png_export.py` | offline rendering | no | support | no | no | no | retain unchanged as offline |
| `test_paced_ros2_supervisor.py` | `/clock` QoS/clock proxy | no | yes | yes | no | no | retain unchanged |
| `test_dwb_geometry_capture.py` | optional DWB debug capture | no | no | no | no | yes | remain separate/noncanonical |
| `test_dwb_zero_event_capture.py` | optional negative-event capture | no | no | no | no | yes | remain separate/noncanonical |
| `test_nav2_frontier_diagnostic.py` | diagnostic Nav2/frontier analysis | no | conditional | no | no | yes | remain diagnostic |
| `test_unknown_pose_launch.py` / scan/profile tests | protected launch/sensor/registration behavior | yes | yes | yes | no | partial | not observer redesign tests; retain |

The test suite currently protects both real invariants and historical file/implementation shape. Under a thin architecture, tests for exact callback invocation, logger-private caches, and intermediate file timing should be rewritten as fixture-based replay/parity tests. Tests for raw values, timestamp semantics, common release, complete contact/GT streams, `/clock`, provenance, finalization, and primary metric equivalence should remain mandatory.

## 23. Uncertainties and evidence boundaries

The following remain unknown from static evidence and must not be silently promoted to facts:

- Exact line-by-line executed coverage of canonical C; no runtime coverage was collected in this audit.
- Whether every custom map/peer/cooperation topic can be recorded by native rosbag2 with the same QoS/source identity and required hidden-topic semantics.
- Whether an exact raw replay can reproduce every current synchronized-map and registration artifact without retaining a live callback-time decision.
- Whether any finalizer consumer outside the searched repository expects a legacy file or field.
- Whether any external thesis/report tooling reads `received_wall_elapsed_s`, warning arrival order, process-resource history, or exact JSONL ordering as an unstated contract.
- Exact dead-code count; a branch not reached by canonical C is not necessarily dead in an alternate diagnostic launch.
- Exact CPU attribution of each derived callback; preserved profiler numbers overlap and are not additive budgets.

These uncertainties are why the verdict is “redesign justified, parity first” rather than “delete the logger”.

## 24. Recommended first implementation step — not implemented

If ChatGPT/project management authorizes a redesign, the first bounded implementation task should be:

> Build an offline raw-contract fixture from one preserved complete canonical-C bag and its GT/contact/forensic raw files; enumerate and checksum every required stream and provenance marker; implement replay-only reconstruction for coverage, trajectory/distance, idle, and synchronized-map outputs; compare values, simulation timestamps, source identity, ordering, and final schemas against the preserved artifacts; do not remove any live subscription or alter the runner until the replay and raw-completeness gates pass.

This step should be performed as a separate, explicitly authorized change with focused parity tests and a fresh complete-evidence validation. It should not begin with executor threading, timer consolidation, topic deletion, sensor reduction, or host-priority changes.

## 25. Final answers in compact form

| Question | Answer |
|---|---|
| Q1 | Approximately 0 direct logger LOC control robot behavior; roughly 0–200 observer-adjacent LOC belongs to clock/acceptance infrastructure. |
| Q2 | Approximately 5,500–7,000 direct LOC is observation/evaluation/forensics/offline rendering. |
| Q3 | Approximately 3,100–3,500 of 4,092 logger LOC is canonical-C reachable; exact executed LOC unknown. |
| Q4 | Initialization/subscriptions; TF/pose/index; maps/GT/sync; cooperation/events; timers/health/coverage; finalization; optional diagnostics. |
| Q5 | Approximately 1,000–2,000 clearly optional/noncanonical and 1,500–3,000 plausible legacy/derived-removal candidates; exact dead LOC unknown. |
| Q6 | Raw time/provenance, motion/odom, maps, cooperation/task events, navigation outcomes, GT/contact, release, recorder/horizon status. |
| Q7 | Duplicate summaries/timeseries/health/warnings/diagnostics, live sync frames, process history, internal callback receive-time fields, and schemas supported only by old finalization/tests are candidates. |
| Q8 | Live coverage, idle, trajectory/overlap, warning analysis, resource history, most sync-map derivation, optional debug/frontier/DWB analysis. |
| Q9 | `/clock`, provenance, release/readiness, odom, maps, TF, task/cooperation, nav outcomes, GT/contact, hidden selected streams, completeness metadata. |
| Q10 | Coverage, distance, overlap, idle, sync maps, TF joins, cooperation summaries, warnings/health, figures, final metrics. |
| Q11 | Exact callback receive ordering and `received_wall_elapsed_s` have not been shown scientifically necessary. |
| Q12 | No; source justifies a smaller raw/acceptance observer, not a 4,092-line central live derived logger. |
| Q13 | Thin raw recorder + minimal acceptance coordinator + GT/contact + bounded finalizer; offline evaluator for derived metrics. |
| Q14 | Yes for a parity-first redesign; no immediate deletion or migration is authorized by this audit. |
| Q15 | Fixture-based raw contract and offline replay/parity for coverage/trajectory/idle/sync-map, before live removal. |
| Q16 | Do not repeat rejected micro-optimizations, speculative threading, evidence reduction, or protected-subsystem changes. |

## 26. Scope and non-claims

This audit does not claim that the logger alone caused the RTF deficit, that Webots alone caused it, or that native rosbag can already replace every custom live stream. It does not claim arbitrary-start C-SLAM, inter-robot loop closure, joint pose-graph optimization, optimal allocation, zero duplicate exploration, guaranteed collision avoidance, universal scalability, Raspberry Pi/PID validation, or final motion limits. It does not authorize the final 15-run ABC campaign, a 1,200-s C run, A/B/D, videos, or any production change. The current hard C RTF gate remains blocked in the authoritative reports, independently of this architecture audit.

## Final summary

VERDICT:
`ARCHITECTURAL_THIN_RECORDER_REDESIGN_JUSTIFIED_PARITY_FIRST; NO_IMMEDIATE_LIVE_REMOVAL_AUTHORIZED`

TRUE_TRANSITIVE_OBSERVER_LOC:
`7,550 direct observer/evidence LOC (6,219 runtime/direct + 1,331 offline/render); 21,153 including adjacent runner/diagnostic support, with the latter excluded from the direct total.`

CANONICAL_C_LIVE_OBSERVER_LOC:
`Approximately 4,361–4,761 static direct LOC reachable/conditional in canonical C; approximately 3,100–3,500 in cooperative_experiment_logger.py.`

CONTROL_CRITICAL_LOGGER_LOC:
`Approximately 0 direct logger LOC; approximately 0–200 observer-adjacent clock/acceptance infrastructure LOC.`

SCIENTIFICALLY_NECESSARY_LIVE_LOC:
`Approximately 1,800–2,700 for raw capture and minimum live validity/GT/contact obligations; exact value requires the authorized raw-contract redesign.`

LEGACY_DIAGNOSTIC_OR_OFFLINE_CANDIDATE_LOC:
`Approximately 3,000–4,000 direct LOC, including derived live work, optional diagnostics, duplicate representations, and offline/rendering; qualified range, not a deletion count.`

THIN_RECORDER_REDESIGN_JUSTIFIED:
`YES — as a parity-first architecture task; NO immediate production migration authorized.`

FIRST_RECOMMENDED_IMPLEMENTATION_STEP:
`Create a preserved-bag/raw-contract fixture and deterministic offline replay for coverage, trajectory/distance, idle, and synchronized-map outputs; prove artifact/schema/timestamp parity and raw completeness before removing any live observer work.`
