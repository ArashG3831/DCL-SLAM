#!/usr/bin/env python3
"""Offline curvature-conditioned wheel-to-Supervisor residual analysis."""
from __future__ import annotations
import argparse, csv, json, math, statistics
from pathlib import Path
import numpy as np

R = 0.020
L = 0.052 * 1.095
BINS = [(0.0, 0.5), (0.5, 2.0), (2.0, 5.0), (5.0, 20.0)]

def load(path):
    with open(path, newline='', encoding='utf-8') as f: return list(csv.DictReader(f))
def f(row, key):
    try: return float(row[key]) if row.get(key, '') not in ('', None) else np.nan
    except (ValueError, TypeError): return np.nan
def yaw(row):
    return f(row, 'planar_yaw_rad')
def unwrap(x): return np.unwrap(np.asarray(x, dtype=float))
def interp(t, x, q): return np.interp(q, t, x)

def analyze(root):
    root = Path(root); m = load(root / 'telemetry.csv'); g = load(root / 'supervisor_ground_truth.csv')
    t = np.array([f(r,'sim_time_s') for r in m]); keep=np.isfinite(t)
    m=[r for r,k in zip(m,keep) if k]; t=t[keep]
    gt_t=np.array([f(r,'sim_time_s') for r in g]); good=np.isfinite(gt_t)
    g=[r for r,k in zip(g,good) if k]; gt_t=gt_t[good]
    order=np.argsort(gt_t); gt_t=gt_t[order]; g=[g[i] for i in order]
    a=max(t[0],gt_t[0]); b=min(t[-1],gt_t[-1]); mask=(t>=a)&(t<=b); t=t[mask]; m=[r for r,k in zip(m,mask) if k]
    gx=interp(gt_t,np.array([f(r,'world_x_m') for r in g]),t); gy=interp(gt_t,np.array([f(r,'world_y_m') for r in g]),t)
    gyaw=unwrap(interp(gt_t,unwrap([yaw(r) for r in g]),t))
    ox=np.array([f(r,'odom_x_m') for r in m]); oy=np.array([f(r,'odom_y_m') for r in m]); raw_oyaw=np.array([f(r,'odom_yaw_rad') for r in m])
    finite_yaw=np.isfinite(raw_oyaw)
    if not np.any(finite_yaw):
        raise RuntimeError('no odometry yaw samples')
    oyaw=unwrap(np.interp(np.arange(len(raw_oyaw)), np.flatnonzero(finite_yaw), raw_oyaw[finite_yaw]))
    valid=np.isfinite(ox)&np.isfinite(oy)&np.isfinite(oyaw); t=t[valid]; gx=gx[valid]; gy=gy[valid]; gyaw=gyaw[valid]; ox=ox[valid]; oy=oy[valid]; oyaw=oyaw[valid]; m=[r for r,k in zip(m,valid) if k]
    th=oyaw[0]-gyaw[0]; c,s=math.cos(th),math.sin(th); agx=c*gx-s*gy; agy=s*gx+c*gy
    agx += ox[0]-(c*gx[0]-s*gy[0]); agy += oy[0]-(s*gx[0]+c*gy[0]); agyaw=gyaw+th
    terr=np.hypot(ox-agx,oy-agy); yerr=np.unwrap(oyaw-agyaw)
    dgt=np.hypot(np.diff(agx),np.diff(agy)); dist=np.r_[0,np.cumsum(dgt)]
    lp=np.array([f(r,'left_position_rad') for r in m]); rp=np.array([f(r,'right_position_rad') for r in m]);
    ds=R*((np.diff(lp)+np.diff(rp))/2.0); dw=R*(np.diff(rp)-np.diff(lp))/L
    wf=np.zeros(len(ds)); wl=np.zeros(len(ds));
    nz=np.abs(dw)>1e-9; wf[~nz]=ds[~nz]; wf[nz]=ds[nz]*np.sin(dw[nz])/dw[nz]; wl[nz]=ds[nz]*(1-np.cos(dw[nz]))/dw[nz]
    dx=np.diff(agx); dy=np.diff(agy); prev=agyaw[:-1]; cf=np.cos(prev); sf=np.sin(prev)
    gf=cf*dx+sf*dy; gl=-sf*dx+cf*dy; gyinc=np.unwrap(agyaw)[1:]-np.unwrap(agyaw)[:-1]
    cmdv=np.array([f(r,'final_command_vx') for r in m]); cmdw=np.array([f(r,'final_command_wz') for r in m]); cmdv=np.nan_to_num(cmdv); cmdw=np.nan_to_num(cmdw)
    signed_k=cmdw[:-1]/np.maximum(np.abs(cmdv[:-1]),1e-6); k=np.abs(signed_k); moving=np.abs(cmdv[:-1])>0.005
    cat=np.full(len(ds),'STOP',dtype=object); cat[moving & (k<0.15)]='STRAIGHT'; cat[moving & (k>=0.15)]='ARC'; cat[(~moving)&(np.abs(cmdw[:-1])>0.03)]='NEAR_PLACE_ROTATION';
    rows=[]
    for i in range(len(ds)):
        rows.append({'sim_time_s':float(t[i+1]),'distance_m':float(dist[i+1]),'category':cat[i], 'curvature_1pm':float(k[i]),'signed_curvature_1pm':float(signed_k[i]),'cmd_vx_mps':float(cmdv[i]),'cmd_wz_radps':float(cmdw[i]),'gt_forward_m':float(gf[i]),'gt_lateral_m':float(gl[i]),'wheel_forward_m':float(wf[i]),'wheel_lateral_m':0.0,'gt_yaw_increment_rad':float(gyinc[i]),'wheel_yaw_increment_rad':float(dw[i]),'forward_residual_m':float(gf[i]-wf[i]),'lateral_residual_m':float(gl[i]),'yaw_residual_rad':float(gyinc[i]-dw[i]),'gt_x_m':float(agx[i+1]),'gt_y_m':float(agy[i+1]),'odom_x_m':float(ox[i+1]),'odom_y_m':float(oy[i+1]),'translation_error_m':float(terr[i+1]),'yaw_error_rad':float(yerr[i+1])})
    with open(root/'per_step_residuals.csv','w',newline='',encoding='utf-8') as out:
        w=csv.DictWriter(out,fieldnames=list(rows[0])); w.writeheader(); w.writerows(rows)
    summary={'initial_alignment':{'theta_align_rad':th,'initial_gt':[float(gx[0]),float(gy[0]),float(gyaw[0])],'initial_odom':[float(ox[0]),float(oy[0]),float(oyaw[0])]},'gt_odom':{'translation_rmse_m':float(np.sqrt(np.mean(terr**2))),'final_translation_m':float(terr[-1]),'max_translation_m':float(np.max(terr)),'yaw_rmse_rad':float(np.sqrt(np.mean(yerr**2))),'final_yaw_rad':float(yerr[-1]),'max_yaw_rad':float(np.max(np.abs(yerr)))},'route_distance_m':float(dist[-1]),'samples':len(rows),'bins':[],'categories':{}}
    for lo,hi in BINS:
        sel=moving & (k>=lo)&(k<hi); dsum=float(np.sum(np.abs(np.array([r['gt_forward_m'] for r in rows])[sel]))); lat=np.array([r['lateral_residual_m'] for r in rows])[sel]; yr=np.array([r['yaw_residual_rad'] for r in rows])[sel]
        sk=np.array([r['signed_curvature_1pm'] for r in rows])[sel]
        summary['bins'].append({'lo':lo,'hi':hi,'samples':int(np.sum(sel)),'distance_m':dsum,'mean_abs_lateral_m':float(np.mean(np.abs(lat))) if len(lat) else None,'lateral_per_m':float(np.sum(np.abs(lat))/max(dsum,1e-9)) if len(lat) else None,'p95_abs_lateral_m':float(np.percentile(np.abs(lat),95)) if len(lat) else None,'yaw_per_m':float(np.sum(np.abs(yr))/max(dsum,1e-9)) if len(yr) else None,'mean_cmd_vx_mps':float(np.mean(np.abs(cmdv[:-1][sel]))) if np.any(sel) else None,'mean_signed_curvature_1pm':float(np.mean(sk)) if len(sk) else None,'median_signed_curvature_1pm':float(np.median(sk)) if len(sk) else None})
    for name in ('STOP','STRAIGHT','ARC','NEAR_PLACE_ROTATION'):
        sel=cat==name; lat=np.array([r['lateral_residual_m'] for r in rows])[sel]; dsum=float(np.sum(np.abs(np.array([r['gt_forward_m'] for r in rows])[sel]))); summary['categories'][name]={'samples':int(np.sum(sel)),'distance_m':dsum,'abs_lateral_total_m':float(np.sum(np.abs(lat))),'lateral_per_m':float(np.sum(np.abs(lat))/max(dsum,1e-9))}
    with open(root/'summary.json','w',encoding='utf-8') as out: json.dump(summary,out,indent=2)
    return summary

def main():
    p=argparse.ArgumentParser(); p.add_argument('--root',required=True); args=p.parse_args(); print(json.dumps(analyze(args.root),indent=2))
if __name__=='__main__': main()
