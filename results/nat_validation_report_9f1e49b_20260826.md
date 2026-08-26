# NAT cooperative-exploration validation — 2026-08-26

## Verdict

`VALIDATION_INCOMPLETE — TERMINAL_EXPLORATION_NOT_REACHED`

The NAT/mode-aware Webots controller workaround is operational and resource-stable in the bounded runs below. The dedicated unknown-pose handoff gate passed, including independent peer acceptance and matching evidence hash. The 600-second soak completed under the hard host limits. The 1,200-second campaign completed its full timebox with clean automatic teardown, but this stochastic run did not reach a handoff and therefore did not reach shared fusion, shared allocation, or terminal cooperative exploration. No traffic-coordination validation was run.

This is not a memory-leak-fixed claim. It is evidence for a practical NAT transport workaround and a remaining unknown-pose/evidence-acquisition reliability gap.

## Provenance

* Validation checkout: `/home/arash/webots_ws_clean_validation_20260823`
* Original checkout was not modified: `/home/arash/webots_ws`
* Source commit: `9f1e49b1d02c1b64aadc55ad3e773814c005e398` (detached HEAD)
* Build: `build_nat_controller_fix_9f1e49b_20260826`
* Install: `install_nat_controller_fix_9f1e49b_20260826`
* Launch source/build/install SHA-256: `cd4fb4ae2e966bd3e842bea6258bfa57371ec33b66a155b94faf0a1a9900fe66`
* World: `src/my_epuck_project/worlds/epuck_d500_two_world_unknown_pose_close_start_dynamic_low_slip_20ms_finite.wbt`
* Middleware: `rmw_cyclonedds_cpp`, CycloneDDS NAT `eth0` profile, `ParticipantIndex=auto`, `MaxAutoParticipantIndex=120`
* Networking: WSL NAT, `eth0=172.18.45.108`, gateway `172.18.32.1`, `ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET`, `ROS_LOCALHOST_ONLY` unset
* Runtime: realtime/headless Webots, no RViz/rendering, no motion fixture, `use_sim_time=true`, chunked scan input, reliable reconstructed scan, scan matching enabled, `throttle_scans=1`, loop closing disabled, traffic scheduler disabled.

The NAT profile used was `/tmp/cyclonedds_nat_eth0_20260826.xml` (archived at `results/nat_bootstrap_domain90_20260826/cyclonedds_nat_eth0.xml`). The controller endpoint fix is in `two_robots_namespaced_launch.py`; in NAT/SUBNET mode it derives the default route gateway and logs `WEBOTS_CONTROLLER_ENDPOINT host=172.18.32.1`.

## Host safety and cleanup

Preflight after Windows reboot found no campaign processes or selected-port listeners. Pool Nonpaged Bytes was approximately 0.87 GiB and Windows RAM usage approximately 56–57%. The harness hard stops were exactly Pool Nonpaged Bytes >= 6 GiB or Windows RAM >= 99%; WSL cache/Available-MBytes fluctuations were not stop conditions.

All valid runs used unique ROS domains and Webots ports and ended with the harness descendant cleanup. Each valid artifact has `remaining_campaign_matches=` empty and a post-run port audit with no selected port occupied. The forensic-observer rerun was explicitly excluded as invalid because it produced no ROS data after enabling the supervisor extension; it was not used as a workload result.

## Gate 1 — dedicated unknown-pose handoff

Artifact: [`nat_handoff_gate_9f1e49b_20260826`](nat_handoff_gate_9f1e49b_20260826)

Command (via the validated NAT harness):

```bash
VERIFICATION_LIFETIME_S=1100.0 TIMEBOX_SECONDS=180 \
  /tmp/run_dispatch_resource_nat_eth0_20260826.sh \
  96 24878 results/nat_handoff_gate_9f1e49b_20260826 true
```

Functional evidence:

* Robot 1 and Robot 2 each independently accepted a compatible three-inlier hypothesis.
* Both frontend summaries contain the identical evidence-set hash: `85fcee00193145a8`.
* Robot 1 selected three constraints; Robot 2 independently verified the same three among five candidates. Both reported the same transform convention and inverse-compatible handoff.
* Transform installed by the canonical handoff: approximately `(-2.756245, -0.007152, 0.00334654 rad)`.
* Baseline: Robot 1 `1.79959595 m`; Robot 2 `1.79959595 m`.
* Projected registration error: `0.00113076 m`.
* Registration residuals: Robot 1 `0.0258986 m`, Robot 2 `0.0200371 m`.
* Geometric inlier ratios: Robot 1 `0.785602`, Robot 2 `0.935442`.
* Descriptor similarity `0.940401`; overlap approximately `0.947–0.948`.
* Exactly one shared-activation sequence occurred. Both local stacks logged `local_process_teardown ... remaining=0` before `STARTING_SHARED`/`POST_HANDOFF_SHARED`.
* Shared fusion and distributed allocation activated only after the handoff. Both allocators produced the same decision hash (`d9509612...`) for the shared round.
* No second handoff marker or active DDS/ROS crash occurred.

Resource evidence:

* Pool: `933,474,304` bytes pre, `940,699,648` peak/post (`+6.89 MiB`).
* Windows RAM: approximately 55.8% pre to 73.3% peak, 59.7% at the final sample.
* `/clock`: 8,104 messages; nonzero final simulation time about 162.32 s.
* Full corrected scans: Robot 1 131 and Robot 2 133, all 720 beams; Nav scans 131/133 at 180 beams.
* Maps: 167/168; global costmaps 84/86; local costmaps 269/269; TF 23,914.
* Cleanup: `remaining_campaign_matches=` empty; selected port released.

Shared-map evidence was captured in a second observer-enabled run (`nat_handoff_gate_9f1e49b_20260826_observed`). Its `coverage.csv` computed byte-level map equivalence (`sha256` over occupancy arrays plus geometry) at synchronized samples at simulated times 150 s and 160 s. Later asynchronous map updates diverged again, so the result proves synchronized equivalent snapshots, not persistent equality at every publication. The run itself remained functional and clean. The subsequent forensic-supervisor variant was invalid (zero `/clock`/scan/map data and zero forensic snapshots) and is excluded from gate scoring.

## Gate 2 — long-navigation soak

Artifact: [`nat_long_navigation_soak_9f1e49b_20260826`](nat_long_navigation_soak_9f1e49b_20260826)

Command:

```bash
MY_EPUCK_WEBOTS_NETWORK_MODE=nat VERIFICATION_LIFETIME_S=1100.0 \
TIMEBOX_SECONDS=600 /tmp/run_dispatch_resource_nat_eth0_20260826.sh \
  99 24881 results/nat_long_navigation_soak_9f1e49b_20260826 true
```

The full 600-second bound completed (`TIMEBOX_600S`) without a hard stop. Pool was `938,020,864` bytes pre, `947,105,792` peak/post (`+8.66 MiB`); Windows RAM was 55.5% pre, 70.4% peak, 60.4% post. `/clock` reached simulation second 566; both corrected 720-beam scan streams, 180-beam Nav streams, maps, costmaps, TF, odometry, and `cmd_vel_nav` were active. Cleanup and port audits were clean.

This soak did not independently reach handoff (`accepted=false` in both final frontend summaries). It therefore validates long-run NAT/navigation/resource behavior, not post-handoff exploration. It also exposed repeated NavFn messages, “Failed to create a plan from potential when a legal potential was found”, and two goal failures. These are recorded as an unresolved planner limitation/diagnostic, not suppressed or reclassified as a clean Nav2 pass. No DDS assertion or active SIGABRT/SIGSEGV occurred.

## Gate 3 — final 1,200-second campaign

Artifact: [`nat_final_campaign_9f1e49b_20260826`](nat_final_campaign_9f1e49b_20260826)

Command:

```bash
MY_EPUCK_WEBOTS_NETWORK_MODE=nat VERIFICATION_LIFETIME_S=1300.0 \
TIMEBOX_SECONDS=1200 /tmp/run_dispatch_resource_nat_eth0_20260826.sh \
  100 24882 results/nat_final_campaign_9f1e49b_20260826 true
```

The timebox completed (`TIMEBOX_1200S`) and automatic cleanup was clean. Functional transport remained healthy:

* Pool: `940,511,232` bytes pre, `967,569,408` peak/post (`+25.80 MiB`).
* Windows RAM: 60.7% pre, 74.3% peak, 61.1% post.
* `/clock`: 56,971 messages; nonzero final simulation time about 1,139.42 s.
* Corrected scans: Robot 1 1,020 and Robot 2 931, all 720 beams; Nav scans 1,020/931 at 180 beams.
* Maps: 1,186/1,185; global costmaps 606/610; local costmaps 1,950/1,951; TF 157,617; odometry about 56,600 per robot; `cmd_vel_nav` 22,629/21,908; action feedback 115,157/112,324.
* No active DDS assertion, SIGABRT, or SIGSEGV; no selected port or campaign process remained after cleanup.

The final gate failed functionally:

* `UNKNOWN_POSE_SHARED_ACTIVATION` remained in its waiting state; no accepted handoff, shared fusion, or shared allocation occurred.
* Both frontends exhausted 64 verification batches. Each accumulated only three constraints, with no multi-constraint attempt accepted.
* Robot 1 recorded 429 registration-worker queue drops, 507 crop requests, 552 descriptors published, 554 received, 23,834 cheap candidates, and 4,177 candidate selections. Its best margin was only `0.11744`, with temporal-gate rejections and repeated novelty/budget exhaustion.
* Both final frontend summaries have `accepted=false`; no canonical evidence hash was installed and no terminal cooperative state was reached.
* The run therefore cannot prove shared-map equality, post-handoff navigation, terminal frontier exhaustion, or mission completion.

This is evidence of a stochastic/evidence-acquisition reliability gap, not permission to lower selector gates. The successful dedicated gate shows the selector and peer protocol can pass under favorable evidence; the final run shows that the current launch can exhaust its bounded registration budget before such evidence is formed.

## Comparison with prior mirrored mode

The valid NAT runs are materially different from the earlier mirrored-mode failures. The dedicated and soak NAT runs stayed below 1 GiB pool with only single-digit-MiB within-run deltas; the 1,200-second run ended at about 0.90 GiB with a +25.8 MiB peak delta. Earlier mirrored full-graph runs crossed approximately 6 GiB with hundreds of MiB growth. This supports the statement “NAT/mode-aware Webots transport workaround validated for these bounded runs.” It does not identify or eliminate an external Windows NETIO/NDIS owner, nor does it prove arbitrary long-run safety.

## Remaining blockers

1. Make evidence acquisition deterministic enough that a normal final campaign reaches the already-proven peer-confirmed handoff without lowering the three-inlier/margin gates. The current final run shows registration queue drops and verification-budget exhaustion while robots navigate away from the initial overlap.
2. Re-run a post-fix dedicated handoff with saved shared-map arrays and verify persistent equality at a defined synchronized snapshot.
3. Eliminate or precisely classify the repeated NavFn potential-generation failures and goal failures before calling Nav2 reliability complete.
4. Re-run the full final campaign only after the above, and require terminal completion and no active-goal residue.
5. Traffic reservation/priority remains separate and was not implemented or validated in these gates.

## Artifact index

* `results/nat_bootstrap_controller_fix_9f1e49b_20260826_valid`
* `results/nat_resource_controller_fix_9f1e49b_20260826`
* `results/nat_handoff_gate_9f1e49b_20260826`
* `results/nat_handoff_gate_9f1e49b_20260826_observed`
* `results/nat_handoff_gate_9f1e49b_20260826_forensic` (invalid zero-data observer variant; excluded)
* `results/nat_long_navigation_soak_9f1e49b_20260826`
* `results/nat_final_campaign_9f1e49b_20260826`
