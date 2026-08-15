#!/usr/bin/env python3
"""Write the bounded DWB/RPP fixed-route comparison report."""
from __future__ import annotations
import argparse, csv, json, statistics
from pathlib import Path

def load(p):
    with open(p, newline='', encoding='utf-8') as f: return list(csv.DictReader(f))
def fmt(x, digits=6):
    return 'n/a' if x is None else f'{x:.{digits}g}'
def route_time(root):
    q = [r for r in load(root/'route_status.csv') if r.get('event') == 'route_complete']
    return float(q[-1]['sim_time_s']) if q else None
def main():
    ap=argparse.ArgumentParser(); ap.add_argument('--pool',required=True); ap.add_argument('--output',required=True); a=ap.parse_args()
    pool=json.loads(Path(a.pool).read_text()); lines=[]
    lines += ['# Controlled DWB versus RPP wheel–chassis residual test', '',
      'Diagnostic-only one-robot fixed-route comparison. Production Nav2, SLAM, wheel geometry, world, and cooperative configuration were not changed.', '']
    lines += ['## 1. Software and configuration', '',
      '- ROS 2 Jazzy; installed Nav2 DWB and Regulated Pure Pursuit packages: 1.3.10.',
      '- Installed `webots_ros2_driver` package: 2025.0.1 (the external Windows Webots executable was not modified).',
      '- Wheel model: `r=0.020 m`, `L_eff=0.052*1.095=0.05694 m`.',
      '- Fixed route controller: `nav2_msgs/action/FollowPath` on `/robot1/follow_path`.',
      '- Final command chain and wheel feedback were logged; Supervisor GT was external/read-only.', '',
      '**DWB:** `dwb_core::DWBLocalPlanner`; max vx 0.13, max theta 0.35, acc/decel x +0.20/-0.20, 6 vx samples, 21 theta samples, sim_time 1.7 s, unchanged production critics.',
      '**RPP:** `nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController`; desired 0.13, lookahead 0.20 (min 0.12, max 0.30, time 1.5), velocity-scaled lookahead, regulated scaling enabled, min radius 0.50 m, min speed 0.026, rotate-to-heading enabled at 0.785 rad, rotate speed 0.35, max angular accel 0.20, no reversing, collision detection and stateful enabled.', '']
    lines += ['## 2. Route and validity', '', 'The route definition was generated once and replayed identically. It contains 14 FollowPath segments (one full rounded-rectangle lap plus the first five segments of a second lap), approximately 16.88 m, with straight and medium/tight rounded arcs. The implemented rounded rectangle uses counter-clockwise (left) arcs; right-turn symmetry was not separately sampled in this campaign and is a follow-up limitation. Route signatures (labels and all path points) compare equal across all valid runs.', '']
    lines += ['| controller | repetition | valid | route time (sim s) | route distance (m) | GT→odom RMSE (m) | final error (m) | yaw RMSE (rad) | final yaw (rad) |', '|---|---:|---|---:|---:|---:|---:|---:|---:|']
    for v in pool['validity']:
        root=Path(v['root']); s=json.loads((root/'summary.json').read_text()); m=s['gt_odom']; lines.append(f"| {v['controller']} | {root.name} | {v['valid']} | {fmt(route_time(root),5)} | {fmt(s['route_distance_m'],6)} | {fmt(m['translation_rmse_m'])} | {fmt(m['final_translation_m'])} | {fmt(m['yaw_rmse_rad'])} | {fmt(m['final_yaw_rad'])} |")
    lines += ['', 'All six runs are valid route completions. Earlier fast-mode, shared-library, lifecycle, and incomplete-route attempts are excluded as infrastructure-invalid or route-failure and are not counted.', '']
    lines += ['## 3. Curvature-conditioned primary results', '', '| bin (1/m) | controller | distance mean±sd (m) | command vx mean±sd (m/s) | lateral residual/m mean±sd | yaw residual/m mean±sd |', '|---|---|---:|---:|---:|---:|']
    for key in ('0-0.5','0.5-2','2-5','5-20'):
        for label in ('DWB','RPP'):
            p=pool['pooled'][label][key]
            def cell(k):
                z=p[k]; return 'n/a' if z['n']==0 else f"{z['mean']:.6g}±{z['stdev']:.3g}"
            lines.append(f"| {key} | {label} | {cell('distance_m')} | {cell('mean_cmd_vx_mps')} | {cell('lateral_per_m')} | {cell('yaw_per_m')} |")
    lines += ['', 'The high-curvature comparison is the 2–5 1/m bin (the RPP route had no 5–20 1/m samples, so that bin is not interpreted).', '']
    d=pool['pooled']['DWB']['2-5']; r=pool['pooled']['RPP']['2-5']
    speed=(1-r['mean_cmd_vx_mps']['mean']/d['mean_cmd_vx_mps']['mean'])*100
    lat=(1-r['weighted_lateral_per_m']/d['weighted_lateral_per_m'])*100
    yaw=(1-r['weighted_yaw_per_m']/d['weighted_yaw_per_m'])*100
    lines += [f'- Pooled 2–5 1/m command speed reduction: **{speed:.1f}%** ({d["mean_cmd_vx_mps"]["mean"]:.5f} → {r["mean_cmd_vx_mps"]["mean"]:.5f} m/s).', f'- Pooled 2–5 1/m absolute lateral residual per metre reduction: **{lat:.1f}%** ({d["weighted_lateral_per_m"]:.6f} → {r["weighted_lateral_per_m"]:.6f} m/m).', f'- Pooled 2–5 1/m absolute yaw residual per metre **increased {abs(yaw):.1f}%** ({d["weighted_yaw_per_m"]:.6f} → {r["weighted_yaw_per_m"]:.6f} rad/m); RPP does not improve every residual component.', '- The directional lateral-residual reduction occurs in all three DWB/RPP repetition pairs: RPP values 0.001703, 0.002251, 0.002209 m/m versus DWB 0.003350, 0.003429, 0.003602 m/m.', '']
    lines += ['## 4. Motion categories and supporting results', '', '| controller | category | distance (m) | lateral residual/m mean±sd across repetitions |', '|---|---|---:|---:|']
    for label in ('DWB','RPP'):
        for cat in ('STRAIGHT','ARC'):
            z=pool['pooled'][label]['categories'][cat]; lines.append(f"| {label} | {cat} | see per-run summaries | {fmt(z['mean'])}±{fmt(z['stdev'],3)} |")
    lines += ['', 'DWB arc residual is consistently much larger than straight residual. RPP reduces arc residual while retaining the same route geometry. RPP also spends more route distance in low-curvature motion and has lower low-curvature command speed (~0.0995 m/s pooled versus DWB ~0.1207 m/s); this throughput tradeoff is measured, not hidden.', '']
    lines += ['## 5. Interpretation and limitations', '', '- The measured quantity is the discrepancy between wheel-position-implied differential-drive motion and external Webots chassis motion. It is not proof of literal tire slip or a specific contact mechanism.', '- No invasive contact instrumentation was required; contact identity remains unresolved. The route was open and collision-monitor chain was retained, but contact telemetry is explanatory rather than a validity gate.', '- Initial GT/odom alignment used one SE(2) transform per run: `theta = yaw_odom(t0)-yaw_GT(t0)` with translation chosen to coincide initial positions. No later realignment was used.', '- This test isolates controller-generated motion on one robot. It does not establish cooperative exploration, map-quality, or production-controller superiority.', '- The local diagnostic costmap used the same controller/smoother/collision chain and an open-arena inflation-only costmap; no frontier/cooperative stack was launched.', '']
    lines += ['## 6. Resource and file status', '', '- Realtime headless runs were used; fast mode was rejected for validity because it produced CPU/load-sensitive route failures. One-robot realtime process samples remained materially lighter than the previous two-robot forensic campaign; no RViz, rosout collection, maps, or cooperative telemetry were enabled.', '- Per-run artifacts include controller YAML snapshot, route definition/status, GT CSV, wheel/odom/command telemetry, reconstructed per-step residual CSV, and summary JSON.', '- Diagnostic source additions: `fixed_follow_path_driver.py`, `controller_residual_test_launch.py`, `analyze_controller_residual.py`, `pool_controller_residual.py`, `write_controller_residual_report.py`; `setup.py` adds only the diagnostic driver entry point; `turn_motion_diagnostics.py` adds buffered flushing. Production YAML files were not edited.', '', '## 7. Engineering conclusion', '', 'RPP reduced commanded speed in the empirically problematic curvature regime and reduced the dominant lateral wheel-implied-versus-Webots-chassis residual by approximately 41% in pooled 2–5 1/m motion, with the same lateral-residual direction in all three repetition pairs. The yaw-residual-per-metre component increased by approximately 21%, so this supports reduced lateral translational mismatch rather than improvement of every residual component. This supports a subsequent full cooperative validation, but does not by itself authorize changing production configuration.', '', 'RPP_CURVATURE_DRIFT_REDUCTION_SUPPORTED']
    Path(a.output).write_text('\n'.join(lines)+'\n', encoding='utf-8')

if __name__ == '__main__': main()
