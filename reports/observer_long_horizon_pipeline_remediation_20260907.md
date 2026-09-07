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

Pending in this working checkpoint; the source, focused regression test, and
this report section are to be committed together as one isolated fix-family
commit.

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

Pending in this working checkpoint; this family will be committed separately
after the report update.

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

Pending in this working checkpoint; this warning-output change and its focused
tests will be committed separately.

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

Pending in this working checkpoint; this map-quality discovery fix and its
focused test will be committed separately.

### Remaining caveat

The 1200 s artifact has the four maps but was finalized incompletely. The fresh
600 s run must demonstrate that normal finalization and the provenance-resolved
analyzer produce `map_quality_report.json` with the approved metrics.
