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
