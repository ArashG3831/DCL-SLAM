# Long-Horizon Observer Pipeline Remediation

Baseline commit: `20cf6997b994a7af1baa37f5206b3e7b928f1b14`
Scope: observer/evidence and offline-pipeline defects exposed by the preserved
1200 s Condition-C artifact. No protected robotics or scientific configuration
was changed. The untracked preserved artifact is intentionally not part of any
commit.

## Fix family 1 — Condition-C shared-map coverage continuity

Status: DONE — offline coverage authority corrected; fresh runtime validation is
still required before long-horizon acceptance.

### Root cause

The source-aware fusion process did not crash at approximately 263 s. It remained
alive through teardown. The fusion implementation publishes a shared map only
when the map payload changes. The preserved 1200 s evidence shows local-map
payload changes ending at approximately 256–259 s, peer-local updates ending at
approximately 256–262 s, and shared-map output ending at approximately
258–264 s. Later local-map messages continue, but repeat the same payload. This
is an intentional change-only publication behavior, not a designed 263 s horizon
or a missing shared-map state.

The old finalization path already records request-time coverage rows. The
preserved artifact contains 591 `coverage_requests.jsonl` rows through 1200 s,
but its `coverage.csv` is empty because finalization was interrupted before the
coverage replay stage. The current offline evaluator instead selected sparse
shared-map message snapshots directly, so it incorrectly treated the last
payload-changing publication as the end of Condition-C coverage evidence.

### Evidence

- Preserved artifact: `fast_trial_20260907T133814Z/observer/fast_trial_20260907T133814Z-01/`.
- Native bag: local maps continue to the horizon; shared-map messages are sparse
  and stop after their final payload change.
- `coverage_requests.jsonl`: request timestamps continue to 1200 s.
- `source_aware_map_fusion` profile/logs: process remains alive; later calls are
  no-op because map content is unchanged.

### Implementation

`offline_evidence_replay._coverage()` now prefers the legacy finalizer's
request-time `coverage.csv` for Condition C. The adapter preserves the shared
map definition and carries the final valid shared state across later coverage
requests through the existing finalizer/replay semantics. It does not substitute
local maps. A present-but-empty, malformed, or non-monotonic `coverage.csv` now
fails closed rather than silently falling back to sparse snapshots. The previous
sparse-map path remains available for conditions and runs without the finalized
artifact.

### Tests

The following focused tests passed in a clean ROS-enabled Jazzy shell:

`test_offline_coverage_artifact.py`, `test_deferred_coverage.py`, and
`test_offline_window_metrics.py`: **5 passed**.

Python compilation and `git diff --check` also passed.

### Commit

Commit: `85048b3` — `observer: honor finalized Condition-C coverage timeline`.

### Remaining caveat

The preserved failed 1200 s run cannot demonstrate the finalized request-time
CSV because shutdown interrupted finalization. A fresh run with normal
finalization must prove that the request-time coverage artifact is generated and
that the [360,600) and later windows remain evaluable.

## Fix family 2 — bounded offloaded finalization

Status: IMPLEMENTED — focused long-input tests pass; runtime confirmation is
deferred to the authorized 600 s validation.

### Root cause

`_repair_offloaded_health()` repeatedly called `_offloaded_topic_records()` for
each health row. That operation filtered and sorted the complete per-topic
receipt history, then `_last_received()` rebuilt a timestamp list for the same
row. The rolling ten-second rate then scanned the same records a third time.
At long horizons this made finalization grow with the product of health rows and
receipt history, and it was the operation active when shutdown escalation
interrupted finalization.

### Implementation

The finalizer now builds one sorted record list and one sorted receipt-time tuple
per topic. Timeseries age joins use binary search over the tuple. Health age and
the legacy inclusive `[sim_time - 10, sim_time]` rate window use
`bisect_left`/`bisect_right` over the same tuple. Existing one-argument helper
entry points remain compatible and construct the same indexes when called
directly. No raw rows, CSV fields, rates, stale thresholds, or timestamp
semantics were changed.

### Tests

The focused logger/offload/coverage/window suite passed: **85 passed**. The
long-input regression exercises 12,001 receipt records, headerless receipts,
last-receipt joins, and both endpoints of the ten-second rate window.

### Commit

Commit: `ca13345` — `observer: bound deferred health finalization`.

### Remaining caveat

The optimization is a finalization-only change. A fresh 600 s run must verify
that normal shutdown writes all final artifacts without escalation and that the
indexed output remains semantically equivalent on a real long evidence history.

## Fix family 3 — warning replay completeness

Status: IMPLEMENTED — focused replay tests pass; real-run confirmation remains
part of the 600 s acceptance.

### Root cause

The legacy warning replay already consumes the complete
`rosout_receipts.jsonl` stream through `replay_warning_records_from_receipts()`.
The standalone offline evaluator, however, reported warnings unavailable unless
the finalizer had already materialized `warnings.jsonl`. When the 1200 s
finalization was interrupted, this made complete raw rosout evidence appear
unavailable even though the authoritative replay input existed.

### Implementation

`offline_evidence_replay._warning_metrics()` now retains the existing
`warnings.jsonl` path as authoritative when present. If it is absent, it invokes
the existing legacy-compatible receipt replay, writes the resulting
`warnings.jsonl` sidecar, and reports its source and receipt counts. Missing
receipts and corrupt receipts fail closed. A readable receipt stream with no
warning-level records is reported as a valid zero and produces an empty warning
sidecar. No classifier or warning normalization semantics were duplicated.

### Tests

Focused warning/replay/comparison/coverage/window tests passed: **17 passed**.
The tests cover 11,000 warning receipts, legacy deduplication, valid zero
warnings, missing raw evidence, and corrupt raw evidence.

### Commit

Commit: `4a51c1a` — `observer: replay warnings from raw receipts`.

### Remaining caveat

The 1200 s artifact was interrupted before finalization and therefore cannot
prove the normal finalizer path. The fresh 600 s run must confirm that warning
output is available from ordinary finalization and that the fallback does not
mask any other finalization failure.

## Fix family 4 — approved map-quality output discovery

Status: IMPLEMENTED — focused analyzer/path tests pass; real-run confirmation
remains part of the 600 s acceptance.

### Root cause

The approved analyzer is tracked at
`src/my_epuck_project/tools/analyze_cooperative_decision_offline.py`, but the
package install does not install the repository `tools/` directory. The
installed `offline_map_quality.py` therefore looked only beneath its installed
package location and returned unavailable even when all four final map NPZ
artifacts and canonical world metadata existed.

### Implementation

`offline_map_quality._approved_map_quality_tool()` now resolves the analyzer
first from the manifest's recorded `runtime_worktree` (and then its recorded
source root), followed by the existing checkout-relative path. The existing
analyzer, world geometry, transforms, map inputs, and output metric definitions
are unchanged. The selected source path remains recorded in the report.

### Tests

Focused map-quality, approved-analyzer, warning, and comparison tests passed:
**13 passed**. The new regression proves an installed evaluator can resolve the
approved tool from manifest provenance without copying or reimplementing it.

### Commit

Commit: `0e8b94d` — `observer: resolve approved map quality tool`.

### Remaining caveat

The 1200 s artifact has the four maps but was finalized incompletely. The fresh
600 s run must demonstrate that normal finalization and the provenance-resolved
analyzer produce `map_quality_report.json` with the approved metrics.

## Fix family 5 — output-root and clean-provenance boundary

Status: VERIFIED — no production source change required.

### Root cause

The failed 1200 s invocation constructed a clean child environment but did not
pass its intended `RUN_ROOT` variable through that boundary. The runner then
received its default output root (`.`), so generated result files appeared under
the source workspace and were reflected as `worktree_dirty` in the run
manifest. This was an invocation/wrapper defect, independent of observer
semantics and independent of source changes.

### Verification and correction

The tracked runner already passes an explicit absolute `output_root` and
`run_id` in its launch argument list, and `prepare_attempt()` derives the
attempt directory from the explicit `--results-directory` argument. The clean
launch procedure for the next run will therefore pass the absolute results
directory as a command-line argument inside the `env -i` child, rather than
depending on an unexported `RUN_ROOT`. The existing environment filter remains
responsible for removing only the excluded old workspace while preserving ROS
vendor paths and the approved driver prefix.

Existing runner tests cover filtered environments and explicit output-root
launch arguments. No source modification is justified for this external
wrapper mistake.

### Commit

This verification is recorded in the remediation report; no source commit is
needed for this family.

### Remaining caveat

The next preflight must print and verify the exact absolute results directory,
the generated attempt path, and the manifest's `runtime_worktree` separately
from its output directory. Any result that resolves to the repository root
instead of the requested results directory must be rejected before launch.

## Fix family 6 — long-horizon RTF scaling attribution

Status: ANALYZED — no additional safe source optimization justified from the
preserved evidence; the 600 s run remains required.

### Measured evidence

The 1200 s launch log contains 1,092 fusion profile records. Across both fusion
processes, the recorded process CPU durations were approximately:

| Fusion mode | Calls | CPU seconds | Wall seconds | Cells inspected |
|---|---:|---:|---:|---:|
| `FULL_REBUILD` | 76 | 2.749 | 3.093 | 30,603,324 |
| `POSE_PATCH` | 23 | 0.223 | 0.358 | 1,891,857 |
| `COALESCED` | 22 | 0.375 | 0.389 | 3,701,156 |
| `NOOP` | 971 | 1.933 | 2.375 | 6,114,401 |
| **total** | **1,092** | **5.281** | **6.215** | **42,310,738** |

The preserved authoritative active execution window is 656.186 wall seconds for
approximately 1200 simulated seconds. The fusion profile therefore proves
real map-processing work and increasing map dimensions (ending at approximately
383x825), but it does not account for the full long-horizon wall-time loss by
itself. The artifact contains no valid post-run per-process CPU attribution for
the rest of the ROS graph, so assigning the remaining degradation to rosbag,
TF, SLAM, frontier generation, or observer callbacks would be speculation.

### Decision

No map-resolution, sensor-rate, evidence-rate, GT/contact, cooperation, or
robotics behavior change is made. No source optimization is accepted without a
measured dominant cost and a semantic parity proof. The indexed finalization
change already removes the proven superlinear shutdown work; the corrected
Condition-C coverage authority removes the sparse-stream evaluator defect. The
fresh 600 s run will provide the required end-to-end confirmation and report
RTF, finalization, and evidence completeness separately.

### Commit

This analysis is recorded in the remediation report; no performance source
commit is warranted from the available evidence.

### Remaining caveat

The 1200 s run cannot establish a complete post-fix RTF or prove that every
remaining ROS process scales acceptably. The single authorized 600 s run must
be used for that validation; if it fails a pipeline gate, stop and diagnose that
failure rather than launching another run.
