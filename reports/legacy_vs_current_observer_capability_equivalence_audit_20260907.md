# Legacy vs current observer capability-equivalence audit

Date: 2026-09-07  
Workspace: `/home/arash/webots_ws_clean_validation_20260823`  
Scope: read-only source and preserved-artifact review. No source, configuration,
build, or experiment was changed.

## Executive result

The repository currently contains two selectable observer paths:

1. **Salvaged legacy path** — `cooperative_experiment_logger.py` remains the
   complete behavioral reference and now has deferred-authority switches for
   synchronized map frames, odometry trajectory, shared trajectory/overlap,
   coverage, pair/agreement summaries, warnings, and Nav2 diagnostics.
2. **Thin raw/offline path** — `ThinEvidenceSession` plus native rosbag2,
   `ThinSupervisorCapture`, `offline_protocol_replay.py`, and
   `offline_evidence_replay.py`.

The salvaged legacy path preserves more of the old observable output contract.
The thin path preserves the raw sources and a substantial set of derived
metrics, but it does **not** yet reproduce every legacy `summary.json` and
`mission_result.json` field. The missing fields are concentrated in exact
avoidable-idle/snappiness, full certificate payload statistics, fairness,
scaling windows, map-quality/localization joins, several anomaly fields, and
resource attribution.

Overall classification for the current thin path: **D — partially preserved
with missing information**, with several individual families at **B** (moved to
offline replay) and **C** (raw evidence exists but final aggregation is absent).
No evidence supports calling the thin path fully equivalent to the legacy
observer. The salvaged legacy deferred path is the stronger current equivalence
path for the families whose parity artifacts report `PARITY_PASS` or
`DEFERRED_AUTHORITATIVE`.

## Classification and evidence rules

* **A — identical capability preserved:** same externally useful capability and
  semantics are available in the current path; storage location may differ.
* **B — moved live computation to offline replay:** the capability remains, but
  the current implementation computes it after the mission from preserved raw
  evidence. This does not by itself prove numerical parity; the parity status is
  stated separately.
* **C — raw evidence exists, final analyzer/report field missing:** the source
  data is captured, but the current evaluator does not emit the required final
  metric or summary field.
* **D — partially preserved with missing information:** some values or cases are
  available, but the current source, join, definition, or artifact is not
  sufficient for the complete legacy capability.
* **E — genuinely lost:** neither the required capability nor sufficient raw
  evidence/reconstruction exists in the current path. This is used only where
  the old capability is not present at all, not merely where a report key has
  moved.

“Raw evidence exists” means the needed source is actually recorded by the
current path, not merely listed in a contract. “Analyzer exists” means current
production finalization/evaluation invokes a reconstruction that emits the
metric; a standalone diagnostic script or a parity artifact alone is not
counted as a final analyzer.

## Source and artifact basis

| Area | Legacy implementation | Current deferred/evidence implementation |
|---|---|---|
| Live legacy observer | `src/my_epuck_project/my_epuck_project/cooperative_experiment_logger.py`, especially `odom()` 2360–2474, protocol callbacks 2503–2710, `sample_telemetry()` 3325–3337, `sample_coverage()` 3338–3451, `sample_health()` 3452–3488, `summary()` 4585–4646, `write_mission_result()` 4648–4742 | Same logger’s deferred/replay finalization 3727–4944; current salvaged legacy path remains selectable and is the reference path |
| Thin lifecycle | — | `thin_experiment_recorder.py:55–408`; raw recorder lifecycle, contract validation, boundaries and offline entry |
| Thin GT/contact | `cooperative_ground_truth_observer.py:137–300` | `thin_supervisor_capture.py:80–255`; integrated paced Supervisor capture, 20 ms fidelity retained |
| Raw ROS source | Legacy callbacks and optional `passive_rosbag.py` capture | `passive_rosbag.py:45–65` scientific raw topic set; native rosbag2 is the raw owner |
| Contract | Legacy artifact checks in logger finalization | `raw_evidence_contract.py:28–229`; topic and condition-aware semantic checks |
| Protocol replay | Legacy event/counter/ledger callbacks | `offline_protocol_replay.py:131–423`; compact payload parser and dispatch-terminal latency |
| General replay | Legacy live metric accumulators and CSVs | `offline_evidence_replay.py:38–859`; bag, GT, maps, coverage, ownership, handoff, contact and warning reconstruction |
| Current runtime selection | Legacy is the runner default unless explicitly overridden | `cooperative_trial_fast.py:712–723, 1748–1770`; thin C starts `ThinEvidenceSession` and disables the legacy logger |

Representative artifacts used as evidence were the preserved thin complete-
evidence run under `results/thin_complete_evidence_120_20260906_run4/...` and
the current salvaged-legacy run under
`results/observer_metric_completeness_smoke_20260907_final2/...`. The former
has `thin_metrics.json` with `complete=true`, raw finalization complete, and
the thin schema; the latter has legacy `summary.json`, `mission_result.json`,
and deferred parity artifacts. A complete raw contract is not equivalent to
complete legacy metric parity.

## Capability equivalence matrix

| Responsibility / thesis metric | Legacy capability | Current raw evidence | Current analyzer/artifact | Status | Exact gap and whether observer changes are required |
|---|---|---|---|---|---|
| Exact avoidable idle | Live event/status/command state and the existing exact idle logic/diagnostic path | Thin raw `distributed_status`, `distributed_event`, candidate/task streams, odom and command streams; status payload fields are parsed by `offline_protocol_replay.py` | `offline_evidence_replay.py` checks only `work_availability_complete`; it does not emit feasible intervals, avoidable-idle seconds, longest idle, or productive engagement. Standalone `tools/forensic_exact_avoidable_idle.py` exists but is not the thin finalizer | **C** | Raw sources are present in a valid C bag, but final aggregation is missing. No new observer source is required if those payload fields remain complete; an offline analyzer/report integration is required. Missing work-state payload would require a future raw contract change. |
| Snappiness / timestamp joins | Legacy event sequence and source timestamps support goal/terminal and cooperation timing fields | Bag header times, bag order, task/bid/decision/status/event IDs and navigation status are present; protocol replay preserves some sim times and round/task IDs | `_dispatch_latencies()` emits dispatch→terminal latency only. It does not emit terminal→next dispatch, agreement→goal, handoff→first shared goal, candidate-generation, bid, or certificate latency | **D** | Raw time and identity inputs are partly present, but the full join semantics and final fields are missing. Offline analyzer work is required; observer changes are needed only if a particular join lacks source identity or invocation timestamps. |
| Cooperation protocol payloads | Legacy records candidates, task snapshots, bids, pair decisions, statuses, events, failures and normalized ledgers | Native bag records all listed C protocol topics when published; `offline_protocol_replay.py` deserializes payloads into compact records | Current replay emits payload records, per-robot counts, states and event types; it does not reproduce every legacy normalized ledger/counter field in `summary.json` | **D** | Capability is substantially present but not output-equivalent. Offline aggregation/adapters are missing; raw observer changes are not required for the fields already in the messages. |
| Assignment counts and task statistics | Legacy task/bid/decision callbacks and goal ledger | Task snapshots, bids, pair decisions, distributed events/status and candidate batches | Counts and payload lists exist in replay; legacy round outcomes, unique decision semantics and complete goal ledger are not all emitted by thin evaluation | **D** | Need exact offline normalization and report aggregation. Raw additions are needed only for any missing identity/payload field discovered during parity. |
| Agreement / continuation statistics | Legacy `distributed_decision`, `distributed_status`, `distributed_event` and counters | Pair decisions, distributed events and statuses are bagged if published | Current replay parses pair decisions and events; current `thin_metrics.json` does not emit the complete legacy agreement/continuation summary. Salvaged legacy finalization has deferred agreement/pair artifacts | **B** for salvaged legacy; **D** for thin | The deferred legacy path has the capability; thin needs final aggregation and parity mapping. No control change is required. |
| Cost-only certificate decisions | Legacy certificate/deferral state is represented in logger event payloads and certificate-related counters | Distributed event stream is recorded; replay recognizes `COST_ONLY_DISPATCH_CERTIFICATE` and parses its JSON reason payload | `_certificate_status()` distinguishes `OBSERVED`, `MISSING_OBSERVATIONS`, `NOT_INVOKED`, and `NO_EVIDENCE`, but no complete certificate metric table is emitted; the raw contract’s C `certificate_event_stream` predicate is weaker than payload completeness | **D** | Raw source exists, but detailed certificate observations and final report fields are incomplete/contract-gated. Offline replay/contract strengthening is required; do not infer missing certificate values. |
| DNU / deferred / certified rounds | Legacy events/counters and candidate/task data are recorded live | Frontier candidate metadata and distributed status/events are bagged; generation and DNU fields are parsed where present | Current replay retains generation records and status DNU counts but does not emit the complete legacy DNU/deferred/certified time series and reason breakdown | **C** | Raw evidence exists for the current message payloads; final time-series aggregation is missing. No observer change unless a required DNU field was not published into the raw message. |
| Coverage source and final coverage | Legacy `sample_coverage()` and `CoverageAttribution`; C uses shared map, A local map, B local-map union | Thin bag has local/shared OccupancyGrid streams; map request/receipt evidence exists in the salvaged path | `_coverage()` is condition-aware: A local, B transformed local union, C canonical shared stream; final known cells/area are emitted | **B** | Live map-derived work moved to offline replay. Current source semantics are correct at the source-selection level. Exact legacy numerical parity is not uniformly demonstrated for all preserved fixtures; retain parity gating. No observer change if map/request evidence remains complete. |
| Coverage AUC and milestones | Legacy coverage rows contain the sampled curve but not a complete modern AUC/milestone contract | Raw map snapshots plus `/clock`; coverage request timestamps exist in the salvaged path | `_auc()` and `_coverage_milestones()` emit AUC and 25/50/75/90% threshold times | **B** | Deferred computation exists. The remaining issue is reconciliation of sampling/threshold definitions with the authoritative thesis definition, not missing raw capture. |
| Duplicate explored cells and ownership | Legacy attribution maintains first-seen, duplicate, later and simultaneous sets | Thin raw local maps and initial transform provenance are available; C shared maps are separately available | `_local_ownership()` and `_stream_local_ownership()` reconstruct local first-seen/later ownership; current thin output does not guarantee legacy C shared-map attribution fields | **D** | A replay exists, but map source/transform policy is not yet shown identical to every legacy ownership result. Offline parity work is required; raw additions only if exact source-map identity is missing. |
| Simultaneous observation / overlap of explored cells | Legacy `CoverageAttribution` retains simultaneous cells under its configured window | Local map sequence and timestamps exist | Current coverage replay reconstructs basic ownership but does not expose the full legacy simultaneous-window output in the thin final schema | **D** | Partial raw and implementation support; final field and exact window parity are missing. Offline completion is required. |
| Per-robot trajectory and odometry distance | Legacy `LocalTrajectory`/`LiveDistanceAccumulator` and odom callback | Raw `/odom` bag and GT CSV | `offline_evidence_replay._read_bag()` computes odom distance; `experiment_metrics.replay_local_trajectory_csv()` and salvaged logger deferred odometry replay use the existing `LocalTrajectory` primitive | **B** | Capability moved offline in the salvaged path with recorded parity evidence. Thin evaluator currently emits distance but not all legacy bins/revisit fields, so thin schema equivalence remains partial. |
| Shared trajectory / cross-robot overlap | Legacy `TrajectoryOverlap` uses TF-composed global samples and overlap bins | Thin raw `/tf`, `/tf_static`, `/odom` | Salvaged legacy deferred path replays causal TF and reports `shared_trajectory_parity.json`; preserved smoke reports `PARITY_PASS`, `equal=true`. Thin `offline_evidence_replay.py` does not reproduce this full TF trajectory path | **B** for salvaged legacy; **E** for the current thin evaluator’s full shared-frame metric | The capability remains in the salvaged deferred path. It is absent from the thin evaluator, although raw TF/odom exists; implementing thin replay would be required before claiming thin equivalence. |
| Stalls / no-progress / stuck / oscillation | Legacy `MotionDetector`, `sample_telemetry()`, event counters and anomaly summary | Thin bag has odom, commands, navigation feedback, statuses and distributed events | Current thin evaluator has no `MotionDetector` replay and no `anomalies`/stuck/no-progress/oscillation output | **C** | Required raw ingredients are mostly present, but the final analyzer/report field is missing. Offline reuse/extraction of the legacy detector semantics is required; observer changes are not required if command and feedback provenance is sufficient. |
| Navigation goals and terminal outcomes | Legacy action callbacks, goal ledger, `summary.navigation`, `mission_result.goal_accounting` | Hidden NavigateToPose status, feedback and related action status/plan topics are requested by thin bag; distributed terminal events are also captured | `navigation_action_replay.json` and `_read_bag()` count terminal status; protocol replay parses navigation terminal events | **B** for basic outcomes; **D** for full legacy taxonomy | Basic outcome capability moved to raw/offline replay. Full goal-ledger identity, cancellation/failure taxonomy, recovery and exact legacy categories are not fully reproduced by thin output. Offline join work is needed. |
| Planner/action/path diagnostics | Legacy `action_status`, `plan`, rosout and optional frontier forensic callbacks | Thin bag records planner/controller action status, feedback, plans and `/rosout` when published | `navigation_action_replay.json` and `nav2_diagnostic_replay.json` exist; detailed frontier-query forensic capture is noncanonical thin | **B** for raw action/path status; **D** for forensic detail | Replay covers the normal action/path streams, but not all legacy forensic crops/query diagnostics. Optional diagnostic capability is not part of the canonical thin thesis contract; retaining it would require explicit diagnostic capture, not a control change. |
| Physical GT pose/velocity | Legacy external Supervisor records 20 ms GT pose/orientation/velocity | Thin integrated Supervisor writes `supervisor_ground_truth.csv` with 20 ms sampling | `_ground_truth_metrics()` computes GT distance, active and idle time; GT remains a live raw acquisition responsibility | **A** for raw GT capture; **B** for derived GT motion | The raw capability and cadence are preserved. Derived distance/active/idle are moved offline, but exact parity of every old field still needs explicit comparison. |
| Contact semantics / collision episodes | Legacy Supervisor contact polling and contact CSV; legacy final summaries/counters | Thin Supervisor polls contacts at the configured 20 ms period and writes contact evidence | `_contact_metrics()` reads contact CSV and clusters rows into episodes | **B** | Contact detection fidelity is retained and episode derivation is offline. The exact legacy filtering/episode definition must be compared; no sampling reduction is evidenced. |
| SLAM/map quality | Legacy forensic/map artifacts and physical-GT evaluation support map evidence; quality-specific output is not uniformly in `summary.json` | Raw local/shared maps, TF and handoff/GT artifacts exist; optional physical GT JSON may exist | Thin evaluator only loads `forensic/physical_gt_evaluation.json` as `map_quality`; it does not calculate a general SLAM quality suite | **D** | Inputs are partly present and one artifact can be consumed, but required map-quality fields and reference joins are missing. Offline analyzer/report work is needed; observer changes only if a claimed quality metric needs a raw source not currently captured. |
| Handoff transform residual, error and timing | Legacy handoff/frontend artifacts plus `_write_physical_gt_evaluation()` compute physical residuals | Structured frontend JSON, accepted timestamp, evidence-set hash, quality fields and manifest transform are preserved in current thin runs | `_handoff_metrics()` emits accepted records, transform, quality and translation/yaw error against the manifest transform | **B** for the available core; **D** for full physical/reciprocal semantics | The core moved offline. Current evaluator does not prove every legacy physical reference, reciprocal check, or all quality/error fields. Offline joins/aggregation are needed; structured handoff capture must remain. |
| Reciprocal verification | Legacy frontend/coordinator evidence can contain source/target and reciprocal diagnostics | Thin bag contains protocol/front-end related streams; structured frontend JSON is captured where present | `_handoff_metrics()` records source/target and quality but does not perform reciprocal verification | **D** | Raw evidence is partial and the final reciprocal metric is absent. If reciprocal payloads are not in the structured artifact, a minimal future structured artifact addition is required; no inferred value is valid. |
| RTF | Legacy run/runner reports record simulation and wall boundaries; logger summary has elapsed time but is not the authoritative RTF owner | Thin runtime boundary/manifest and runner summary carry simulation time, active wall interval and termination reason | `offline_evidence_replay.py` does not emit authoritative RTF; runner summary is the current owner | **B** | The capability is preserved in the runner/evidence boundary, not in the thin metric JSON. No observer metric change is required if the runner artifact remains authoritative; final report aggregation is missing from the evaluator. |
| CPU/RSS/resource attribution | Legacy logger samples its own CPU/RSS and writes `summary.system`; runner may collect external process data | Thin raw path does not collect an equivalent logger CPU/RSS history; runtime/runner provenance is available in some campaign outputs | No current thin offline reconstruction of the legacy logger resource series | **E** for legacy logger-specific history; **D** for external run-level attribution | The old logger-specific measurement is genuinely absent from thin. Decide whether it is a thesis requirement; if retained, use an explicitly scoped external sampler/raw artifact, not fabricated post-run values. |
| Fairness / workload contribution | Legacy fields provide contribution, goals and task-related counters but no uniformly complete fairness index | Raw bids, decisions, status, events, maps, GT and goals are present | No fairness aggregate in current thin evaluator | **C** | Raw ingredients exist for several fairness definitions; final report aggregation and an approved definition are missing. No observer change for available fields. |
| Scaling windows | Legacy reports can contain run-window/runtime summaries, but no complete current thin window evaluator | `/clock`, boundaries, status/events and raw streams exist | No scaling-window analyzer or final fields in current thin evaluator | **C** | Raw time series exist, but window definitions and output aggregation are absent. Offline implementation/definition approval is required. |
| Provenance / run boundaries / termination | Legacy manifest, run events, finalization and campaign fields | Thin manifest, raw validation, finalization, runtime boundary, controller endpoint and provenance | Thin session records start/horizon/end metadata and fail-closed raw status; runner records final boundary and termination reason | **A** for core acceptance provenance; **B** for deferred report composition | The acceptance capability is preserved, with ownership moved to the coordinator/manifest rather than the logger. This is not a mission metric and must remain fail-closed. |
| START_RELEASE / readiness / pre-release validity | Legacy live release/readiness events and pre-release checks | Raw `/cslam/unknown_pose/start_release`, structured handoff, runner readiness and GT readiness | Thin protocol replay reconstructs release records and checks exactly one accepted common release and pre-release dispatch count | **A** | Current contract preserves the acceptance semantics for the evidence it records. Keep this live/structured validity layer. |
| Artifact completeness / finalization | Legacy logger closes files, exports bags, checks required artifacts and writes final status | Thin session stops GT/rosbag, validates raw bag and invokes offline evaluation | `raw_evidence_finalization.json` and `thin_metrics.json`; raw complete is distinct from metric complete | **A** for raw finalization; **D** for single final scientific certificate | The raw lifecycle is preserved, but raw `complete=true` can coexist with incomplete metric reconstruction. Final scientific completeness needs a stricter final gate. |

## Legacy `summary.json` field equivalence

The following is an exhaustive compact inventory of the field families present
in the representative legacy `summary.json`. `[robot]` means both `robot1` and
`robot2`; `[event_type]` means every event key emitted in that run. A field is
marked by the status of the current **thin** output; the salvaged legacy
deferred path often still writes the original field.

| Legacy field family | Current thin equivalent | Status | Finding |
|---|---|---|---|
| `schema_version` | `thin_offline_evaluation_2.0` (current source; preserved older run is 1.0) | **D** | Versioned schema exists, but it is not the legacy schema. Consumers must not assume byte/schema identity. |
| `run.run_id`, `run.start_time`, `run.end_time`, `run.elapsed_duration_s`, `run.clean_shutdown` | `raw_finalization`, manifest, runtime boundary, runner summary | **B** | Boundary/provenance is preserved, but not reproduced under the same `summary.run` object by the thin evaluator. |
| `frames.global_frame`, `trajectory_source_frame`, `shared_trajectory_source_frame`, `coverage_source_frame`, `coverage_target_frame`, `initial_transform_source`, `known_initial_relative_transform` | manifest provenance, `coverage_contract`, handoff artifact | **D** | Some frame and transform values exist; all legacy frame/report fields are not emitted in one equivalent object. |
| `mapping.available`, `source`, `reason` | `mapping` result and coverage contract | **B** | Availability/source is represented; exact legacy reasons are not guaranteed text-identical. |
| `mapping.initial_known_cells`, `final_known_cells`, `coverage_gain_cells`, `coverage_gain_per_metre_travelled` | `mapping` final cells/area/AUC plus `efficiency` | **B** for final/gain; **C** for exact legacy aggregate field set | The calculations are available or derivable, but the exact legacy field grouping is not fully emitted by thin. |
| `mapping.unique_first_seen_cells.[robot]`, `later_duplicated_by_robot.[robot]`, `total_known_union_cells`, `duplicated_known_fraction` | `mapping.ownership` where local raw maps permit; no guaranteed legacy C aggregate | **D** | Partial ownership replay exists; source-map and parity semantics are not fully closed. |
| `mapping.simultaneously_observed_cells` | no final thin field | **C** | Raw map series exists; simultaneous attribution aggregation is missing. |
| `motion.valid`, `motion.reason`, `source_frame_by_robot.[robot]`, `sample_count_by_robot.[robot]` | `motion.[robot].samples`, source bag topic; no same validity/reason object | **D** | Basic data is present, but the legacy validity/reason semantics are not fully preserved in the thin schema. |
| `motion.distance_travelled_m.[robot]` | `motion.[robot].travel_distance_m` and GT distance | **B** | Odom distance is replayed; current thin does not expose the complete legacy distinction in the same motion object. |
| `motion.repeated_visit_distance_m.[robot]`, `motion.bins_per_robot.[robot]` | no equivalent in current thin `motion` | **C** | Raw odom exists; full `LocalTrajectory` replay/report is not wired into thin output. |
| `shared_frame_motion.bins_per_robot.[robot]`, `cross_robot_bins`, `cross_robot_overlap_fraction`, `distance_travelled_m.[robot]`, `repeated_visit_distance_m.[robot]` | no equivalent in current thin evaluator; salvaged legacy deferred parity artifact exists | **E** in thin; **B** in salvaged legacy | This is a path-specific difference, not a claim that raw TF/odom is absent. |
| `events.[event_type]` | `cooperation.events`/`robots.[robot].event_types` and raw bag event stream | **D** | Event payload/count sources exist, but the flattened legacy event-counter map is not emitted equivalently. |
| `coordination.agreement_publications`, `unique_agreed_rounds`, `unique_agreed_decisions`, `dispatch_attempts`, `goals_terminal` | protocol replay records/dispatches/terminals; no complete legacy coordination object | **D** | Components exist, but deduplication and normalized goal-accounting semantics need offline parity aggregation. |
| `coordination.goal_accounting.by_robot.[robot].active_at_mission_end`, `categories`, `dispatched`, `terminal`, `unknown_or_unaccounted` | status/terminal/event records | **D** | Raw identity and terminal evidence exist, but the legacy ledger classification is not fully emitted by thin. |
| `coordination.goal_accounting.categories`, `.dispatched`, `.records`, `.terminal` | protocol dispatch/terminal records | **D** | Reconstructable in principle, not currently reported with the legacy schema and semantics. |
| `coordination.round_outcomes` | task/bid/pair/event payload records | **C** | Raw round IDs and payloads exist; final round outcome aggregation is absent. |
| `coordination.planner_query_attribution` and `planner_query_duration_s` | action/path status and event payloads | **D** | Some raw planner evidence exists, but attribution categories/durations are not fully replayed into the thin result. |
| `continuous_exploration.[robot].average_cycle_duration_s`, `completed_goals`, `exploration_cycles`, `failed_goals`, `locally_exhausted_duration_s`, `maximum_equivalent_region_attempt_count`, `repeated_region_attempts`, `suppression_creations` | status/events/candidate streams, partial terminal counts | **D** | Inputs are partial and no equivalent final aggregator exists. Exact suppression/region-attempt semantics are not safely inferable. |
| `mission.terminal`, `terminal_reason`, `terminal_time_s`, `shutdown_clean`, `mission_completion_time_s` | protocol status and raw finalization; no equivalent `mission` object in thin metrics | **D** | Some state is available, but terminal and completion semantics are not fully normalized into the current evaluator output. |
| `navigation.goals_sent`, `goals_accepted`, `successes`, `failures`, `cancellations`, `recoveries`, `timeouts` | per-robot action status counts and event terminals | **B** for sent/success/abort/cancel; **D** for recoveries/timeouts/full taxonomy | Basic Nav2 outcome replay exists; the legacy event/goal-ledger classification is incomplete. |
| `robot_terminal_state.[robot].claim_id`, `claim_state`, `frontier_id`, `navigation_active` | status records with state/active task ID; no final per-robot terminal object | **C** | Raw status payload exists; final-state selection/reporting is missing. |
| `anomalies.no_progress_episodes`, `stale_topic_episodes`, `stuck_episodes`, `warning_occurrences` | raw statuses/events/rosout/commands | **C** for warning occurrences; **C/D** for motion/stale anomalies | Warning replay exists in the salvaged legacy path; thin has raw inputs but no complete anomaly aggregator. |
| `system.callback_timing_enabled`, `callback_timing`, `cpu_measurement.*`, `logger_cpu_percent`, `logger_rss_bytes`, `rss_mean_bytes`, `rss_peak_bytes`, `dropped_logger_samples`, `write_failures`, `internal_logger_error_count`, `internal_logger_errors` | raw finalization/runner status; no logger resource history in thin | **E** for logger-specific resource series; **D** for error/finalization subset | These were implementation-scoped legacy logger measurements. They are not reconstructable from a thin bag. |
| `system.output_file_sizes.*` | raw finalization/manifest file inventory, not full thin metrics | **C** | Finalization can know files and sizes; the current thin evaluator does not emit the complete legacy map. |
| `artifact_finalization.complete/status/missing/required/frontend_directory` | `raw_evidence_finalization` and manifest | **A** | The raw acceptance contract is preserved, though the artifact set/schema is versioned differently. |

## Legacy `mission_result.json` field equivalence

| Legacy field family | Current thin equivalent | Status | Finding |
|---|---|---|---|
| `accepted_goals`, `successful_goals`, `failed_goals`, `cancelled_goals` | `navigation` terminal counts and protocol dispatch/terminal records | **B/D** | Basic counts are replayable; exact legacy accepted/terminal accounting is not fully emitted. |
| `actionable_reachable_count`, `detected_not_queried_count`, `below_minimum_gain_count` | distributed status/candidate payloads | **C/D** | Raw status/candidate values exist where published, but no final mission-result aggregation is present. |
| `planner_failed_count` | exploration failure/event payloads and action status | **D** | Raw failure classes can be present; exact legacy counter semantics are not reproduced by the thin evaluator. |
| `final_allocator_epoch`, `final_map_revisions.[robot]` | task/candidate payloads and map metadata | **C** | Raw revisions/epochs may be present, but final field selection is not emitted. |
| `final_known_cells`, `final_known_cells_available`, `mapping_metric_reason` | `mapping.final_known_cells`, `mapping` availability/source | **B/D** | Final map values are replayed for supported map sources; legacy mission-result grouping and unavailable semantics are not identical in all cases. |
| `remaining_frontier_count`, `remaining_out_of_range_count`, `remaining_small_frontier_count`, `remaining_unreachable_count` | frontier candidate batches/status | **C/D** | Candidate snapshots are raw, but final classification at the horizon is not computed by thin evaluation. |
| `robot1_final_state`, `robot2_final_state`, `robot_final_states.[robot]` | distributed status records | **C** | Raw states exist; final-state reduction is missing. |
| `semantic_agreement`, `terminal_agreement` | pair decisions/events and protocol replay | **D** | Agreement payloads exist where published, but the exact terminal semantics are not emitted. |
| `shutdown_clean`, `simulated_duration_s`, `wall_duration_s`, `terminal_reason`, `terminal_time_s` | runtime boundary, manifest and raw finalization | **B** | Boundary and termination evidence is preserved outside the thin mission result. |
| `terminal_small_frontier_length_m` | run configuration/provenance | **C** | Configuration is known, but the current thin evaluator does not re-emit this mission-result field. |
| `recommended_exit_code`, `mission_status` | raw finalization `complete`; runner exit status | **D** | Raw validity and process outcome exist, but mission-status/exit-code policy is not reproduced as the legacy mission result. |
| `goal_accounting` and all nested per-robot/category/record fields | protocol raw records, dispatches, terminals, failure records | **D** | The source data is substantially present, but exact legacy ledger normalization is missing. |
| `artifact_finalization` | `raw_evidence_finalization` | **A** for acceptance status; **D** for artifact schema | Same acceptance concept, different artifact contract and field nesting. |

## Missing-item disposition

| Missing or incomplete item | Raw evidence exists? | Offline analyzer already exists? | Only report aggregation missing? | Observer change required? |
|---|---|---|---|---|
| Exact avoidable-idle seconds/fraction/longest interval/productive engagement | Yes, when `distributed_status` carries feasible/actionable fields and command/status streams are complete | Partial: standalone `forensic_exact_avoidable_idle.py`; not integrated into current thin evaluator | Mostly yes | No, unless payload fields or explicit invocation state are absent in a future run |
| Full snappiness join family | Partial/yes for sim timestamps and IDs; certificate/agreement identity coverage is not guaranteed for every event | Only dispatch→terminal exists in `offline_protocol_replay.py` | No: join logic and output fields are missing | Only for missing source identity/timestamps |
| Full certificate observation table | Event source exists; only actual certificate payloads are authoritative | Partial `_certificate_status()` and event parser | No: payload completeness/aggregation and final artifact are missing | Possibly, if allocator does not publish all required certificate fields; do not infer |
| Complete DNU/deferred/certified time series | Candidate/status/event raw fields exist | No complete current time-series report | Yes, for fields already carried | No for existing payloads |
| Legacy coverage aggregate fields | Maps, `/clock`, request/receipt evidence exist | Yes, `_coverage()`/`_coverage_milestones()` | Mostly; exact parity definition still needs closure | No if request/map source is complete |
| Full ownership/duplicate/simultaneous semantics | Local maps and transform provenance exist; shared C source is separate | Partial `_local_ownership()` and streaming fallback | Not entirely; source-policy parity remains | Only if map identity/transform/request evidence cannot establish old semantics |
| Shared-frame trajectory/overlap in thin path | `/tf`, `/tf_static`, `/odom` exist | Yes in salvaged legacy deferred module/path, not in thin evaluator | No for thin; replay adapter is missing | No raw addition, but offline replay implementation is required |
| Stalls/no-progress/oscillation | Odom, commands, feedback, status/events exist | Legacy live detector exists; no thin replay | Yes if its inputs are sufficient | No, subject to input completeness |
| General SLAM/map quality | Maps/TF/GT partly exist; exact reference inputs depend on metric | Only physical GT JSON loading; no full evaluator | No, metric implementation/report missing | Maybe, for any reference source absent from raw evidence |
| Reciprocal handoff verification | Partial structured source/target/quality fields | No | No, verifier and report field missing | Yes only if reciprocal payload is not preserved |
| Fairness index and workload balance | Bids/decisions/status/maps/goals exist | No current fairness aggregator | Yes for selected approved definition | No for current fields |
| Scaling-window statistics | Time-stamped raw streams and boundaries exist | No | Yes, window definitions/aggregation missing | No |
| Logger CPU/RSS history in thin mode | No equivalent thin raw source | No | No | Yes only if this remains a required thesis metric; otherwise classify non-required rather than fabricate it |
| Full legacy `summary.json` / `mission_result.json` schema | Individual raw pieces exist, but not all fields | Partial | No, several semantic reducers are missing | Only where raw evidence itself is absent; most gaps are offline aggregation |

## What is preserved versus what is not

### Preserved or successfully deferred

* Native raw bag capture, 20 ms GT/contact acquisition, common START_RELEASE,
  provenance, horizon/finalization status and fail-closed raw validation.
* Condition-specific coverage source selection and basic final coverage/AUC
  reconstruction.
* Odom distance and trajectory replay using the existing `LocalTrajectory`
  semantics in the salvaged legacy path.
* Shared TF/odom trajectory and overlap in the salvaged legacy deferred path;
  the preserved parity artifact reports `PARITY_PASS` with equal summaries.
* Basic navigation terminal status replay, structured handoff extraction and
  warning/Nav2 diagnostic replay in the salvaged path.
* Pair-decision/agreement deferred artifacts in the salvaged legacy path,
  subject to their explicit parity/authority status.

### Raw evidence present but current thin final output incomplete

* Exact avoidable idle and the full snappiness family.
* Full cooperation/assignment/round/goal-ledger summaries.
* Certificate observation details and DNU/deferred/certified time series.
* Ownership/duplicate/simultaneous attribution at legacy schema/parity level.
* Motion anomaly and stale/no-progress/oscillation summaries.
* Full map quality/localization error and reciprocal handoff verification.
* Fairness and scaling-window reports.
* Many legacy `summary.json` and `mission_result.json` reducers.

### Genuinely absent from the thin path

* The old logger’s own CPU/RSS history and callback timing history are not
  reproducible from the thin bag. This is an implementation-specific diagnostic
  capability, not automatically a thesis requirement.
* The optional legacy frontier-query forensic crops and some diagnostic-only
  normalized outputs are not part of canonical thin capture. They should not be
  counted as thesis loss unless the experiment plan explicitly requires them.

## Direct verdict

**The current thin observer is not capability-equivalent to the old observer.**
It has a valid raw/evidence foundation and several successful deferred
capabilities, but raw contract completeness currently outruns final metric
completeness. The salvaged legacy path is closer to full observable equivalence
because it retains the old summary/mission-result authority while selectively
using deferred replay.

The safest conclusion is:

> **Current thin path: D — partially preserved with missing information; not a
> thesis-final replacement. Current salvaged legacy deferred path: B for the
> migrated families, A for the acceptance/raw-capture foundations, and still
> the behavioral reference for all remaining fields.**

No observer change is required merely because a final report field is absent
when its raw input is already recorded. Observer changes are required only for
specific evidence that cannot be recovered from current raw bags/artifacts—for
example reciprocal handoff payloads or a deliberately retained resource
measurement. The next work should therefore be an offline parity/aggregation
pass against the legacy outputs, with explicit fail-closed handling for missing
payloads, not a new live observer redesign.
