#!/usr/bin/env python3
"""Pool fixed-route DWB/RPP residual runs without touching production config."""
from __future__ import annotations
import argparse, csv, json, math, statistics
from pathlib import Path

BINS = [(0.0, 0.5), (0.5, 2.0), (2.0, 5.0), (5.0, 20.0)]

def rows(path):
    with open(path, newline='', encoding='utf-8') as f:
        return list(csv.DictReader(f))

def num(v):
    try: return float(v)
    except (TypeError, ValueError): return float('nan')

def valid_route(root):
    p = root / 'route_status.csv'
    if not p.exists(): return False, 'missing route_status.csv'
    rs = rows(p)
    if not rs: return False, 'empty route_status.csv'
    if any(r.get('event') == 'route_complete' and r.get('status') == 'SUCCEEDED' for r in rs):
        return True, 'route_complete'
    failures = [r for r in rs if r.get('event') == 'goal_result' and r.get('status') not in ('4', 'SUCCEEDED')]
    return False, 'route incomplete' + (f'; result={failures[-1].get("status")}' if failures else '')

def route_signature(root):
    p = root / 'route_definition.json'
    data = json.loads(p.read_text())
    return [(s.get('name', s.get('label', '')), tuple(tuple(round(float(v), 9) for v in pt) for pt in s['points'])) for s in data['segments']]

def stats(values):
    v = [x for x in values if x is not None and math.isfinite(x)]
    if not v: return {'n': 0, 'mean': None, 'median': None, 'stdev': None, 'min': None, 'max': None}
    return {'n': len(v), 'mean': statistics.fmean(v), 'median': statistics.median(v),
            'stdev': statistics.stdev(v) if len(v) > 1 else 0.0, 'min': min(v), 'max': max(v)}

def run_metrics(root):
    data = json.loads((root / 'summary.json').read_text())
    step = rows(root / 'per_step_residuals.csv')
    out = {'root': str(root), 'summary': data, 'bins': {}, 'categories': {}}
    for lo, hi in BINS:
        q = [r for r in step if lo <= num(r['curvature_1pm']) < hi]
        d = sum(abs(num(r['gt_forward_m'])) for r in q)
        lat = [abs(num(r['lateral_residual_m'])) for r in q]
        yaw = [abs(num(r['yaw_residual_rad'])) for r in q]
        fwd = [abs(num(r['forward_residual_m'])) for r in q]
        v = [abs(num(r['cmd_vx_mps'])) for r in q]
        out['bins'][f'{lo:g}-{hi:g}'] = {
            'samples': len(q), 'distance_m': d,
            'mean_cmd_vx_mps': statistics.fmean(v) if v else None,
            'lateral_total_m': sum(lat), 'lateral_per_m': sum(lat) / d if d else None,
            'yaw_total_rad': sum(yaw), 'yaw_per_m': sum(yaw) / d if d else None,
            'forward_total_m': sum(fwd), 'forward_per_m': sum(fwd) / d if d else None,
        }
    for cat in ('STOP','STRAIGHT','ARC','NEAR_PLACE_ROTATION'):
        q = [r for r in step if r['category'] == cat]
        d = sum(abs(num(r['gt_forward_m'])) for r in q)
        out['categories'][cat] = {'samples': len(q), 'distance_m': d,
            'lateral_total_m': sum(abs(num(r['lateral_residual_m'])) for r in q),
            'lateral_per_m': sum(abs(num(r['lateral_residual_m'])) for r in q)/d if d else None}
    return out

def main():
    ap = argparse.ArgumentParser(); ap.add_argument('--output', required=True); ap.add_argument('--dwb', nargs='+', required=True); ap.add_argument('--rpp', nargs='+', required=True)
    a = ap.parse_args(); out = {'validity': [], 'groups': {}, 'route_equal': True}
    signatures = []
    for label, paths in (('DWB', a.dwb), ('RPP', a.rpp)):
        vals = []
        for p in paths:
            root = Path(p); ok, reason = valid_route(root); out['validity'].append({'controller': label, 'root': str(root), 'valid': ok, 'reason': reason})
            if ok:
                signatures.append(route_signature(root)); vals.append(run_metrics(root))
        out['groups'][label] = vals
    out['route_equal'] = bool(signatures) and all(s == signatures[0] for s in signatures)
    pooled = {}
    for label, vals in out['groups'].items():
        pooled[label] = {}
        for key in (f'{lo:g}-{hi:g}' for lo, hi in BINS):
            entries = [v['bins'][key] for v in vals]
            pooled[label][key] = {k: stats([e[k] for e in entries]) for k in ('distance_m','mean_cmd_vx_mps','lateral_per_m','yaw_per_m','forward_per_m')}
            d = sum(e['distance_m'] for e in entries)
            pooled[label][key]['weighted_lateral_per_m'] = sum(e['lateral_total_m'] for e in entries)/d if d else None
            pooled[label][key]['weighted_yaw_per_m'] = sum(e['yaw_total_rad'] for e in entries)/d if d else None
        pooled[label]['categories'] = {cat: stats([v['categories'][cat]['lateral_per_m'] for v in vals]) for cat in ('STRAIGHT','ARC')}
    out['pooled'] = pooled
    Path(a.output).write_text(json.dumps(out, indent=2), encoding='utf-8')
    print(json.dumps({'route_equal': out['route_equal'], 'validity': out['validity'], 'pooled': pooled}, indent=2))

if __name__ == '__main__': main()
