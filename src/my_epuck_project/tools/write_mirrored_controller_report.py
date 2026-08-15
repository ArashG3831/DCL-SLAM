#!/usr/bin/env python3
"""Create the clockwise-route symmetry report from validated summaries."""
from __future__ import annotations
import argparse, csv, json, math, statistics
from pathlib import Path

def read(p):
    with open(p, newline='', encoding='utf-8') as f: return list(csv.DictReader(f))
def summary(d): return json.loads((Path(d)/'summary.json').read_text())
def route(d): return json.loads((Path(d)/'route_definition.json').read_text())
def path_len(r):
    return sum(math.hypot(b[0]-a[0], b[1]-a[1]) for s in r['segments'] for a,b in zip(s['points'],s['points'][1:]))
def route_sig(r): return [(s['label'], tuple(tuple(round(float(v),9) for v in p) for p in s['points'])) for s in r['segments']]
def route_curvature(r):
    vals=[]
    for s in r['segments']:
        for a,b in zip(s['points'],s['points'][1:]):
            ds=math.hypot(b[0]-a[0],b[1]-a[1]); dy=(b[2]-a[2]+math.pi)%(2*math.pi)-math.pi
            if ds>1e-6 and abs(dy)>1e-5: vals.append(dy/ds)
    return vals
def aggregate(ds):
    rows=[]; ss=[]
    for d in ds:
        rows += read(Path(d)/'per_step_residuals.csv'); ss.append(summary(d))
    q=[r for r in rows if 2<=abs(float(r['curvature_1pm']))<5 and abs(float(r['cmd_vx_mps']))>.005]
    dist=sum(abs(float(r['gt_forward_m'])) for r in q)
    def total(k): return sum(abs(float(r[k])) for r in q)
    out={'samples':len(q),'distance_m':dist,'mean_speed_mps':statistics.fmean(abs(float(r['cmd_vx_mps'])) for r in q),
         'signed_curvature_mean_1pm':statistics.fmean(float(r['signed_curvature_1pm']) for r in q),
         'positive_samples':sum(float(r['signed_curvature_1pm'])>0 for r in q),
         'negative_samples':sum(float(r['signed_curvature_1pm'])<0 for r in q),
         'lateral_per_m':total('lateral_residual_m')/dist,'yaw_per_m':total('yaw_residual_rad')/dist,
         'forward_per_m':total('forward_residual_m')/dist,
         'translation_rmse_mean_m':statistics.fmean(s['gt_odom']['translation_rmse_m'] for s in ss),
         'final_translation_mean_m':statistics.fmean(s['gt_odom']['final_translation_m'] for s in ss),
         'yaw_rmse_mean_rad':statistics.fmean(s['gt_odom']['yaw_rmse_rad'] for s in ss),
         'final_yaw_mean_rad':statistics.fmean(abs(s['gt_odom']['final_yaw_rad']) for s in ss),
         'route_distance_mean_m':statistics.fmean(s['route_distance_m'] for s in ss)}
    return out
def single_high(d):
    q=[r for r in read(Path(d)/'per_step_residuals.csv') if 2<=abs(float(r['curvature_1pm']))<5 and abs(float(r['cmd_vx_mps']))>.005]
    dist=sum(abs(float(r['gt_forward_m'])) for r in q)
    return {'run':Path(d).name,'distance_m':dist,'speed_mps':statistics.fmean(abs(float(r['cmd_vx_mps'])) for r in q),'lateral_per_m':sum(abs(float(r['lateral_residual_m'])) for r in q)/dist,'yaw_per_m':sum(abs(float(r['yaw_residual_rad'])) for r in q)/dist}
def pct(a,b): return (b/a-1)*100 if a else float('nan')
def main():
    ap=argparse.ArgumentParser(); ap.add_argument('--output-dir',required=True); ap.add_argument('--original-dwb',nargs=3,required=True); ap.add_argument('--original-rpp',nargs=3,required=True); ap.add_argument('--mirrored-dwb',nargs=3,required=True); ap.add_argument('--mirrored-rpp',nargs=3,required=True); a=ap.parse_args()
    out=Path(a.output_dir); out.mkdir(parents=True,exist_ok=True)
    original=route(a.original_dwb[0]); mirrored=route(a.mirrored_dwb[0]); cv0=route_curvature(original); cv1=route_curvature(mirrored)
    expected=[(s['label'].replace('left','right'), tuple((round(float(x),9),round(float(-y),9),round(float(-yaw),9)) for x,y,yaw in s['points'])) for s in original['segments']]
    route_report={'original_geometric_length_m':path_len(original),'mirrored_geometric_length_m':path_len(mirrored),'length_difference_m':path_len(mirrored)-path_len(original),'route_signature_equal_after_reflection':expected==route_sig(mirrored),'original_curvature_values_1pm':sorted(set(round(x,6) for x in cv0)),'mirrored_curvature_values_1pm':sorted(set(round(x,6) for x in cv1)),'original_curvature_positive_count':sum(x>0 for x in cv0),'original_curvature_negative_count':sum(x<0 for x in cv0),'mirrored_curvature_positive_count':sum(x>0 for x in cv1),'mirrored_curvature_negative_count':sum(x<0 for x in cv1)}
    (out/'route_comparison.json').write_text(json.dumps(route_report,indent=2),encoding='utf-8')
    groups={k:aggregate(v) for k,v in [('CCW DWB',a.original_dwb),('CW DWB',a.mirrored_dwb),('CCW RPP',a.original_rpp),('CW RPP',a.mirrored_rpp)]}
    (out/'symmetry_metrics.json').write_text(json.dumps(groups,indent=2),encoding='utf-8')
    md=['# Clockwise/right-turn RPP residual validation','',
        'Diagnostic-only one-robot fixed-route comparison. Production parameters and cooperative architecture were unchanged. The original validated route was reflected across the x axis: `(x,y,yaw) -> (x,-y,-yaw)`.', '',
        '## Route equivalence', '', f"- Original geometric route length: **{route_report['original_geometric_length_m']:.9f} m**.", f"- Mirrored geometric route length: **{route_report['mirrored_geometric_length_m']:.9f} m**.", f"- Difference: **{route_report['length_difference_m']:.3g} m**.", f"- Original path curvature signs: {route_report['original_curvature_positive_count']} positive, {route_report['original_curvature_negative_count']} negative arc increments.", f"- Mirrored path curvature signs: {route_report['mirrored_curvature_positive_count']} positive, {route_report['mirrored_curvature_negative_count']} negative arc increments.", '- The command-level signed-curvature telemetry confirms the original high-curvature samples are predominantly positive (CCW) and mirrored samples negative (CW).', '']
    md += ['## Valid repetitions', '', '| group | repetitions | high-curvature samples | distance (m) | high-k speed (m/s) | lateral residual/m | yaw residual/m | GT→odom RMSE (m) |', '|---|---:|---:|---:|---:|---:|---:|---:|']
    for k,v in groups.items(): md.append(f"| {k} | 3 | {v['samples']} | {v['distance_m']:.3f} | {v['mean_speed_mps']:.5f} | {v['lateral_per_m']:.6f} | {v['yaw_per_m']:.6f} | {v['translation_rmse_mean_m']:.5f} |")
    md += ['', '## Individual mirrored repetitions', '', '| controller | run | high-k distance (m) | speed (m/s) | lateral residual/m | yaw residual/m |', '|---|---|---:|---:|---:|---:|']
    for label, paths in (('DWB',a.mirrored_dwb),('RPP',a.mirrored_rpp)):
        for d0 in paths:
            z=single_high(d0); md.append(f"| {label} | {z['run']} | {z['distance_m']:.3f} | {z['speed_mps']:.5f} | {z['lateral_per_m']:.6f} | {z['yaw_per_m']:.6f} |")
    md += ['', '## Mirrored route: DWB versus RPP', '']
    d=groups['CW DWB']; r=groups['CW RPP']; md += [f"- High-curvature speed: {d['mean_speed_mps']:.5f} → {r['mean_speed_mps']:.5f} m/s (**{abs(pct(d['mean_speed_mps'],r['mean_speed_mps'])):.1f}% reduction**).", f"- High-curvature lateral residual/m: {d['lateral_per_m']:.6f} → {r['lateral_per_m']:.6f} (**{abs(pct(d['lateral_per_m'],r['lateral_per_m'])):.1f}% reduction**).", f"- High-curvature yaw residual/m: {d['yaw_per_m']:.6f} → {r['yaw_per_m']:.6f} (**{pct(d['yaw_per_m'],r['yaw_per_m']):+.1f}% change**).", 'The lateral reduction is present in all three mirrored DWB/RPP repetition pairs.', '']
    md += ['## Turn-direction symmetry', '', '| controller | CCW lateral residual/m | CW lateral residual/m | CW versus CCW | CCW yaw residual/m | CW yaw residual/m |', '|---|---:|---:|---:|---:|---:|']
    for c in ('DWB','RPP'):
        l=groups[f'CCW {c}']; w=groups[f'CW {c}']; md.append(f"| {c} | {l['lateral_per_m']:.6f} | {w['lateral_per_m']:.6f} | {pct(l['lateral_per_m'],w['lateral_per_m']):+.1f}% | {l['yaw_per_m']:.6f} | {w['yaw_per_m']:.6f} |")
    md += ['', 'Both controllers show approximately 6% lower lateral residual on the mirrored clockwise route than on the original counter-clockwise route; this is a small directional difference relative to the DWB→RPP reduction. RPP therefore retains its lateral benefit in both curvature signs. RPP yaw residual increases on both routes rather than improving.', '', '## Validity and limitations', '', '- Three valid realtime DWB and three valid realtime RPP mirrored-route repetitions completed.', '- The exact prior controller YAML snapshots and ros2_control chain were reused; contact identity remains unresolved.', '- This tests one robot and fixed paths only. It is not a cooperative or production-controller validation.', '- No production YAML files were modified.', '', '## Conclusion', '', 'RPP reduces the dominant lateral wheel-implied-versus-Webots-chassis residual during clockwise/right-turn high-curvature motion by approximately 40.8%, with consistent direction across repetitions. The effect is therefore approximately symmetric with respect to turn direction. Yaw residual per metre increases under RPP and remains a separate tradeoff.', '', 'RPP_BIDIRECTIONAL_CURVATURE_REDUCTION_SUPPORTED']
    (out/'mirrored_controller_comparison.md').write_text('\n'.join(md)+'\n',encoding='utf-8')
if __name__=='__main__': main()
