# Condition C observer-enabled diagnostic rerun: rosbag export fix

## Result

The rosbag export failure was fixed without changing mission-time observer
workload. The authoritative rerun reached the requested 180.50 simulated
seconds, stopped the rosbag recorder cleanly, exported the bag successfully,
and passed artifact finalization.

The rerun's active Supervisor-clock RTF was **1.8415798319x**, calculated as:

```text
180.50 simulated seconds / 98.013671128 monotonic active seconds
```

This is lower than the earlier 2.3154867231x diagnostic and therefore does not
claim to reproduce that performance. It does establish complete evidence after
the rosbag fix.

## Fix

Commits:

- `225ce51` — bounded retry for the transient rosbag2 conversion exception;
- `9c5b19d` — signal the recorder's private process group and allow a 30 s
  normal SQLite/cache shutdown before escalation.

The second change addresses the observed failure mode in which finalization
opened the database while the recorder was still shutting down, or the recorder
was force-killed after the previous 8 s grace period. No subscription, timer,
sampling rate, topic selection, observer callback, or robot behavior changed.

## Validation of the fix

Focused tests: **74 passed** covering passive rosbag export/offload and runtime
logger behavior, including private-process-group shutdown and bounded retry.

Build:

```text
build_rosbag_stop_fix_v2_20260907
install_rosbag_stop_fix_v2_20260907
```

The interface and observer package were built together in this prefix so the
current `DistributedExplorationStatus` definition and coordinator executable
were compatible.

## Authoritative run

Result directory:

```text
results/thesis_C_rtf_normal_evidence_export_fix_authoritative_20260907/
```

Runtime observer directory:

```text
results/thesis_C_rtf_normal_evidence_export_fix_authoritative_20260907/observer/normal_evidence_c_export_fix_authoritative_20260907/
```

Configuration matched the prior diagnostic:

- Condition C;
- `frontier_cost_only`;
- `MODE_B`;
- `large_unknown_pose_close_start_20ms_scan_matching`;
- full sensor profile, ideal encoders;
- Webots `fast`, headless/no rendering;
- `use_sim_time=true`;
- seed 1001;
- native rosbag2 plus 20 ms Supervisor GT/contact capture.

Provenance:

- branch: `validation/nat-gate-20260826`;
- recorded source commit: `9c5b19d791bfb1b12b6c9cd7bbb296005f51ef4a`;
- ROS domain: 226;
- RMW: `rmw_cyclonedds_cpp`;
- Webots endpoint: `172.18.32.1:23367`;
- base world SHA-256:
  `feda86d1c1ba8b3f1b18c8f216a079c61fff222dbda09b0326c68f8e0ef9bb86`;
- derived run world SHA-256:
  `1936ed314ee12494969148783d03d72dd7757c4f7ff06db39b771bb6539bede3`.

Clock probe:

- status: `SIM_TIME_COMPLETE`;
- simulation interval: `0.02` to `180.52` s;
- active simulated interval: `180.50` s;
- active monotonic wall interval: `98.013671128` s;
- clock samples: 1,633;
- probe QoS: BEST_EFFORT/VOLATILE.

## Evidence and finalization

- `artifact_finalization.json`: `complete: true`, missing: `[]`;
- `passive_rosbag_export.json`: `complete: true`;
- recorder return code: 0;
- rosbag `/clock` messages: 1,632;
- total indexed rosbag messages: 35,441;
- GT rows: 18,051 data rows at the requested 20 ms cadence;
- contact rows: 88,489;
- `forensic/runtime/runtime_metrics.json`: finalized, 9,026 Supervisor step
  calls, 18,050 contact queries, 20 ms contact sampling.

The launch wrapper remained alive briefly after scientific finalization because
the Webots driver shutdown tail did not exit. It was terminated only after the
recorder export and artifact finalization had completed. The run manifest records
`clean_shutdown: true` and `shutdown_status: clean`; the outer wrapper returned
143 from that post-finalization cleanup signal.

## Invalid attempts retained separately

- `results/thesis_C_rtf_normal_evidence_export_fix_20260907/`: invalid because
  the first probe used incompatible `/clock` QoS, overshot the target, and the
  recorder was force-killed before metadata creation.
- `results/thesis_C_rtf_normal_evidence_export_fix_rerun_20260907/`: invalid
  because its probe was corrected but its package overlay was built against an
  older interface; both assignment nodes exited on missing
  `feasible_work_available` before dispatch.
- `results/thesis_C_rtf_normal_evidence_export_fix_final_20260907/`: complete
  storage export but invalid for behavioral comparison for the same interface
  mismatch; it recorded zero dispatches.

Only the `authoritative` result above is used for the final comparison.

## Comparison

| Run | Active sim s | Active wall s | Active RTF | Complete evidence | Notes |
|---|---:|---:|---:|---|---|
| Earlier diagnostic | 180.50 | 77.953373 | 2.3154867231x | No (export conversion failure) | Reference diagnostic |
| Authoritative rerun | 180.50 | 98.013671128 | 1.8415798319x | Yes | Export fix validated |

The endpoint-tolerance redesign was not changed during this task. The only
production changes are the post-run rosbag shutdown/export robustness changes
listed above.
