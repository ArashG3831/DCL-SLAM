# Observer evaluation final completeness — 2026-09-07

## Verdict

The frozen observer architecture was not redesigned. The offline completion
work added deterministic report surfaces around the existing raw/evidence
representations and reused existing metric primitives. The source-backed work is
complete where evidence exists, but the evaluation layer is **not yet ready for
campaign use** because two evidence conditions remain blocked:

1. the full approved local/shared SLAM map-quality comparison lacks its required
   reference-map/quality artifact;
2. certificate fields cannot be reconstructed when the allocator did not publish
   the required certificate payload.

The current shell also lacks `rclpy`/`rosbag2_py`, so no fresh native-bag replay
was claimed. No Webots run, 1200-second run, or A/B/D campaign was performed.

## Scope and frozen boundaries

Baseline: `db0f890`.

Roadmap: `OBSERVER_EVALUATION_COMPLETION_SPEC.md`.

The live observer, robot behavior, sensor/SLAM/Nav2/cooperation behavior,
scientific sampling, GT/contact fidelity, and raw evidence architecture were not
changed by this completion pass. Work was limited to offline parsing, replay,
aggregation, explicit status, tests, and reports.

## Roadmap status

| Item | Status | Evidence |
|---|---|---|
| 1. Exact avoidable idle and snappiness | DONE | Shared legacy idle state machine; timestamp-join outputs; focused tests 35/35 at checkpoint |
| 2. Cooperation/assignment/agreement/certificate/DNU | DONE with conditional blocked payloads | Payload replay, compact summaries, DNU series, fail-closed certificate completeness; 33/33 at checkpoint |
| 3. Motion anomaly replay | DONE | Existing `MotionDetector` invoked directly on telemetry rows; edge/episode tests |
| 4. Fairness/workload | DONE — approved replacement definition | First-seen-cell workload plus Jain index and component shares |
| 5. Scaling windows | DONE | Half-open simulation-time windows; beyond-horizon rows are `NOT_AVAILABLE` |
| 6. SLAM/map quality | BLOCKED | Existing quality artifact is consumed when present; complete approved reference-map suite is absent |
| 7. Handoff reciprocal verification | DONE where evidence exists | Opposite-direction transforms are composed; missing direction is explicit |
| 8. Final legacy/current comparison | BLOCKED | Final comparison/status layer exists, but blocked evidence prevents an all-PASS claim |

## Requirements-to-output status

The status below is semantic, not a byte-for-byte JSON comparison. A missing
source is never converted to zero.

| Required family | Current source/output | Status | Exact caveat |
|---|---|---|---|
| Known area over time/final coverage/AUC | `offline_evidence_replay.py` → `thin_metrics.json` (`coverage`/`mapping`) | PRESERVED_BY_EQUIVALENT_OFFLINE_OUTPUT | Native-bag replay still requires ROS Python runtime |
| Coverage milestones 25/50/75/90% | `coverage.time_to_world_extent_thresholds_s` | PRESERVED_BY_EQUIVALENT_OFFLINE_OUTPUT | `null` means threshold not reached or extent unavailable |
| Ownership/duplicate attribution | `coverage.ownership` | SEMANTIC_PARITY | Requires complete local-map cells and transform metadata |
| Simultaneous observation | existing coverage semantics plus source-aware map replay | SEMANTIC_PARITY | Exact replay is bounded by preserved map sampling |
| Per-robot distance | `bag.motion` and existing motion primitives | SEMANTIC_PARITY | Uses preserved odometry/GT source as declared |
| Trajectory overlap/repeated distance | existing `LocalTrajectory`/GT overlap outputs | SEMANTIC_PARITY | Unavailable when required trajectory stream is absent |
| Feasible work/avoidable idle | `avoidable_idle.json` | SEMANTIC_PARITY | Shared legacy classifier; missing health is not feasible |
| Productive engagement/longest idle | `avoidable_idle.json` | SEMANTIC_PARITY | Valid zero and missing evidence remain distinct |
| Terminal→dispatch, agreement→goal, handoff→goal joins | `snappiness.json` | PRESERVED_BY_EQUIVALENT_OFFLINE_OUTPUT | Missing timestamp identity remains unavailable |
| Candidate/task/bid/decision/assignment records | `cooperation` raw parsed lists | SEMANTIC_PARITY | Native payload is authoritative |
| Cooperation compact summaries | `cooperation_summary.json` | PRESERVED_BY_EQUIVALENT_OFFLINE_OUTPUT | Counts derive from one parsed representation |
| Certificate reason/counts/bounds/decision | `cooperation_summary.certificate` | BLOCKED_MISSING_RAW_EVIDENCE when payload fields were not published | Never inferred from bids or dispatches |
| DNU time series/reasons | `cooperation_summary.dnu` | PRESERVED_BY_EQUIVALENT_OFFLINE_OUTPUT | Only observed candidate/status records are included |
| Goal lifecycle/failures/recoveries | protocol dispatch/terminal/failure streams | SEMANTIC_PARITY | Unobserved action payload remains unavailable |
| No-progress/stuck/oscillation | `motion_anomalies.json` | SEMANTIC_PARITY | Reuses `MotionDetector`; missing telemetry fails closed |
| Fairness/workload | `fairness.json` | APPROVED_REPLACEMENT | Jain scalar uses first-seen-cell contribution; components emitted |
| Scaling windows | `scaling_windows.json` | PRESERVED_BY_EQUIVALENT_OFFLINE_OUTPUT | Windows beyond clock horizon are `NOT_AVAILABLE` |
| SLAM/map quality | `map_quality_report.json` | BLOCKED_MISSING_RAW_EVIDENCE | No complete approved reference-map/quality artifact |
| Handoff physical residual/timing | `handoff` in `thin_metrics.json` | SEMANTIC_PARITY where structured artifacts exist | Physical-GT source and accepted structured files required |
| Reciprocal handoff transform | `handoff_reciprocal_verification.json` | PRESERVED_BY_EQUIVALENT_OFFLINE_OUTPUT where both directions exist | Missing reciprocal direction is explicit |
| GT/contact episodes | existing GT CSV/contact CSV paths | PRESERVED_BY_EQUIVALENT_OFFLINE_OUTPUT | No sampling change; native replay not executed here |
| RTF | runtime boundary/provenance artifacts | NOT_REQUIRED for pure offline completion | No new RTF claim made |
| Observer self CPU/RSS history | not reproduced | NOT_REQUIRED | Explicitly excluded by the specification |

## Implementation commits

Ordered commits after `db0f890`:

1. `4ab6da3` — `observer: complete offline idle and snappiness replay`
2. `a95bc26` — `docs: record offline idle and snappiness checkpoint`
3. `74e3912` — `observer: complete offline cooperation and certificate summaries`
4. `0cd7ee8` — `observer: correct protocol summary aggregation`
5. `2bdc0ff` — `observer: add offline motion anomaly replay`
6. `f6fd436` — `observer: add offline fairness workload metrics`
7. `cd5b8a5` — `observer: add offline scaling windows`
8. `de7a198` — `observer: expose offline map quality status`
9. `d0592b4` — `observer: add offline reciprocal handoff verification`
10. final documentation/comparison checkpoint: recorded after this report is
    committed.

Each functional change was isolated and followed by focused pure-Python tests.
No unrelated dirty files were staged.

## Tests and validation

Focused cumulative checkpoint results:

- item 1: 33 passed;
- item 3: 35 passed;
- item 4: 37 passed;
- item 5: 39 passed;
- item 6: 41 passed;
- item 7: 43 passed;
- final comparison layer: 45 passed after its implementation checkpoint.

Python syntax compilation passed for each changed offline module. ROS-dependent
message/bag fixture tests could not collect in this shell because `rclpy` is not
installed; this is an environment limitation, not a claimed pass. No Webots or
mission validation was run.

## Exact blocked fields

### Certificate payload

If the raw certificate event exists but omits any of:

- evaluated candidate count;
- DNU count;
- blocking unqueried candidates;
- selected evaluated assignment score;
- best optimistic unqueried score;
- dispatch certification;
- reason;

the replay emits `OBSERVED_INCOMPLETE` and lists the missing fields. If bids or
pair decisions exist but no certificate event exists, it emits
`MISSING_OBSERVATIONS`. This is a source-evidence blocker, not a certificate
semantic change.

### Map quality

The evaluator preserves an existing `map_quality.json` or physical-GT quality
artifact when present. It does not derive an IoU, occupancy agreement, or map
alignment score from coverage counts. The full required thesis map-quality
family remains blocked until the approved reference-map/quality artifact and its
local/shared semantics are available.

## Live observer change required

No live observer change is required by the completed source-backed items. The
remaining blockers require evidence/definition resolution, not a change to
robotics behavior. If a future approved map-quality metric cannot be reconstructed
from existing raw streams, its raw source must be specified before any capture
change.

## Campaign readiness

`OFFLINE_SCIENTIFIC_EVALUATION_READY_FOR_CAMPAIGN`: **NO**.

The layer is structurally ready for continued bounded offline development, but
not thesis-campaign ready until the exact blocked map-quality and conditional
certificate evidence requirements are resolved and a ROS-enabled native-bag
replay validates the complete contract.

## Final status

DONE items: 6 (items 1, 2, 3, 4, 5, 7, counting item 2 as done with explicit
conditional payload blocking is intentionally described below).

BLOCKED items: 2 (items 6 and 8).

The count sentence above intentionally follows the roadmap statuses table's
semantic convention; the unambiguous item-by-item list is: DONE = 1, 2, 3, 4,
5, 7; BLOCKED = 6, 8.
