#!/usr/bin/env python3
"""Plot signed-curvature and turn-direction comparison."""
from __future__ import annotations
import argparse, csv
from pathlib import Path
import matplotlib.pyplot as plt

def read(d):
    with open(Path(d)/'per_step_residuals.csv', newline='', encoding='utf-8') as f: return list(csv.DictReader(f))
def n(r,k): return float(r[k])
def aggregate(paths):
    q=[]
    for d in paths: q += [r for r in read(d) if abs(n(r,'cmd_vx_mps'))>.005 and abs(n(r,'curvature_1pm'))<20]
    return q
def high(paths):
    q=[r for r in aggregate(paths) if 2<=abs(n(r,'curvature_1pm'))<5]
    d=sum(abs(n(r,'gt_forward_m')) for r in q)
    return sum(abs(n(r,'lateral_residual_m')) for r in q)/d if d else None
def main():
    ap=argparse.ArgumentParser(); ap.add_argument('--output',required=True); ap.add_argument('--ccw-dwb',nargs=3,required=True); ap.add_argument('--cw-dwb',nargs=3,required=True); ap.add_argument('--ccw-rpp',nargs=3,required=True); ap.add_argument('--cw-rpp',nargs=3,required=True); a=ap.parse_args()
    groups={'CCW DWB':a.ccw_dwb,'CW DWB':a.cw_dwb,'CCW RPP':a.ccw_rpp,'CW RPP':a.cw_rpp}; colors={'CCW DWB':'tab:blue','CW DWB':'steelblue','CCW RPP':'tab:orange','CW RPP':'darkorange'}
    fig,ax=plt.subplots(2,2,figsize=(12,8),constrained_layout=True)
    for label,paths in groups.items():
        q=aggregate(paths); k=[n(r,'signed_curvature_1pm') for r in q]; v=[abs(n(r,'cmd_vx_mps')) for r in q]; lat=[abs(n(r,'lateral_residual_m')) for r in q]
        ax[0,0].scatter(k,v,s=2,alpha=.14,color=colors[label],label=label)
        ax[0,1].scatter(k,lat,s=2,alpha=.14,color=colors[label],label=label)
    labels=['CCW DWB','CW DWB','CCW RPP','CW RPP']; vals=[high(groups[x]) for x in labels]
    ax[1,0].bar(labels,vals,color=[colors[x] for x in labels]); ax[1,0].set_ylabel('high-k lateral residual/m'); ax[1,0].set_title('High-curvature residual by direction/controller'); ax[1,0].tick_params(axis='x',rotation=25)
    ccw=[high(a.ccw_dwb),high(a.ccw_rpp)]; cw=[high(a.cw_dwb),high(a.cw_rpp)]; x=[0,1]; w=.35
    ax[1,1].bar([i-w/2 for i in x],ccw,w,label='CCW',color=['tab:blue','tab:orange']); ax[1,1].bar([i+w/2 for i in x],cw,w,label='CW',color=['steelblue','darkorange']); ax[1,1].set_xticks(x,['DWB','RPP']); ax[1,1].set_ylabel('high-k lateral residual/m'); ax[1,1].set_title('Turn-direction symmetry'); ax[1,1].legend()
    ax[0,0].set(xlabel='signed curvature (1/m)',ylabel='|v| (m/s)',title='Speed vs signed curvature'); ax[0,1].set(xlabel='signed curvature (1/m)',ylabel='|lateral residual| (m)',title='Residual vs signed curvature')
    for axy in ax.flat: axy.grid(True,alpha=.25)
    ax[0,0].legend(markerscale=4); ax[0,1].legend(markerscale=4)
    fig.savefig(a.output,dpi=160)
if __name__=='__main__': main()
