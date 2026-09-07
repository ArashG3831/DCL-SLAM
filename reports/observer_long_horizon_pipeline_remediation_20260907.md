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
