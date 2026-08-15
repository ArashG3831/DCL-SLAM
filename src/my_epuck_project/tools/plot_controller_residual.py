#!/usr/bin/env python3
"""Make compact diagnostic plots from fixed-route residual CSVs."""
from __future__ import annotations
import argparse, csv
from pathlib import Path
import matplotlib.pyplot as plt

def read(p):
    with open(p, newline='', encoding='utf-8') as f: return list(csv.DictReader(f))
def f(r,k): return float(r[k])
def main():
    ap=argparse.ArgumentParser(); ap.add_argument('--output',required=True); ap.add_argument('--dwb',nargs='+',required=True); ap.add_argument('--rpp',nargs='+',required=True); a=ap.parse_args()
    fig, ax=plt.subplots(2,2,figsize=(12,8),constrained_layout=True)
    colors={'DWB':'tab:blue','RPP':'tab:orange'}
    for label, paths in (('DWB',a.dwb),('RPP',a.rpp)):
        first=True
        for p in paths:
            q=read(Path(p)/'per_step_residuals.csv'); q=[r for r in q if abs(f(r,'cmd_vx_mps'))>0.005 and abs(f(r,'curvature_1pm'))<20]; k=[abs(f(r,'curvature_1pm')) for r in q]; v=[abs(f(r,'cmd_vx_mps')) for r in q]; lat=[abs(f(r,'lateral_residual_m')) for r in q]; t=[f(r,'sim_time_s') for r in q]
            ax[0,0].scatter(k,v,s=2,alpha=.12,color=colors[label],label=label if first else None)
            ax[0,1].scatter(k,lat,s=2,alpha=.12,color=colors[label],label=label if first else None)
            ax[1,0].plot(t,lat,color=colors[label],alpha=.25)
            ax[1,1].plot(t,[f(r,'translation_error_m') for r in q],color=colors[label],alpha=.25)
            first=False
    ax[0,0].set(xlabel='|command curvature| (1/m)',ylabel='|v| (m/s)',title='Speed versus curvature')
    ax[0,1].set(xlabel='|command curvature| (1/m)',ylabel='|lateral residual| (m)',title='Per-step lateral residual')
    ax[1,0].set(xlabel='simulation time (s)',ylabel='|lateral residual| (m)',title='Residual over time')
    ax[1,1].set(xlabel='simulation time (s)',ylabel='GT→odom translation error (m)',title='Secondary odometry error')
    for axy in ax.flat: axy.grid(True,alpha=.25)
    ax[0,0].legend(); ax[0,1].legend(); ax[1,0].plot([], [], color=colors['DWB'], label='DWB'); ax[1,0].plot([], [], color=colors['RPP'], label='RPP'); ax[1,0].legend(); ax[1,1].plot([], [], color=colors['DWB'], label='DWB'); ax[1,1].plot([], [], color=colors['RPP'], label='RPP'); ax[1,1].legend()
    fig.savefig(a.output,dpi=160)
if __name__=='__main__': main()
