# Condition-C 600 s final acceptance

## Run

- Exact HEAD: `ba6e5beacfc13bda873bbf4f72264ef57ffc1ff9`
- Run: `/home/arash/webots_ws_clean_validation_20260823/results/thesis_condition_C_600s_20260907T191800Z/fast_trial_20260907T191805Z/`
- Launcher: `python3 src/my_epuck_project/tools/run_thesis_experiment.py --condition C --horizon 600`
- Configuration: Condition C, `frontier_cost_only`, MODE_B, seed 1001, canonical close-start 20 ms scan-matching world, full sensors, ideal encoders, fast/headless, scientific raw capture, GT/contact at 20 ms, profiling disabled.

## Gate results

| Gate | Result |
|---|---|
| Canonical launcher preflight | PASS |
| Robot startup | PASS |
| Scientific clock / 600 s horizon | PASS (`SIM_TIME_COMPLETE`, end 600.08 s) |
| Clean shutdown | PASS |
| Finalizer fix | PASS; observer exited cleanly |
| `artifact_finalization.json` | PASS; `complete=true`, `missing=[]` |
| `summary.json` / `mission_result.json` | PRESENT |
| Native rosbag export | PASS; `complete=true`, recorder return code 0 |
| GT/contact capture | PRESENT; 20 ms capture artifacts present |
| Shared-map continuity through 600 s | **FAIL** |
| Offline evaluator | NOT RUN (required stop condition) |

## Blocking failure

The native bag and `map_receipts.jsonl` show that shared-map publication stopped
far before the scientific horizon:

- `/robot1/shared_map`: 12 messages; last receipt 71.40 s, header 69.96 s.
- `/robot2/shared_map`: 14 messages; last receipt 72.40 s, header 69.96 s.
- Local maps continued to the horizon: robot1 last receipt 599.66 s and robot2
  last receipt 599.44 s.
- Final local and shared map files were written, but the existence of final
  snapshots does not prove continuous shared-map stream evidence.

This violates the acceptance requirement that shared-map evidence support the
full 0–600 coverage windows. The failure is earlier than the previously observed
~263 s stopping point and must be diagnosed separately; no local-map substitution
was made.

## Finalization conclusion

The committed finalizer correction was effective for this run. The observer
completed its long finalization path without the earlier second-SIGINT
`KeyboardInterrupt`, and all required finalization artifacts were produced.
The remaining failure is shared-map continuity, not finalizer interruption.

Because a required scientific raw stream failed its continuity gate, the current
offline evaluator was deliberately not run and no final metric values or RTF
acceptance claim are reported from this run.

No source, observer, evaluator, or scientific-definition changes were made during
this validation.

CONDITION_C_600S_FINAL_ACCEPTANCE_FAIL
