"""Score an observed closed-loop lap using the independent rear-axle pose."""
import argparse
import json
from pathlib import Path

import numpy as np


def metric(x):
    return {'rmse_m': float(np.sqrt(np.mean(x*x))), 'abs_p95_m': float(np.quantile(abs(x), .95)),
            'abs_max_m': float(abs(x).max())} if len(x) else {}


def score(data):
    rows=data['rows']
    result={'finish': data['reason'], 'gt_samples': len(rows), 'route_length_m': data['length']}
    if len(rows)<3:
        return result
    idx={name:i for i,name in enumerate(data['columns'])}
    def field(name):
        return np.array([r[idx[name]] for r in rows], dtype=float)
    t=field('t')
    # Per-pose velocity uses a centered 100 ms secant; clip at endpoints.
    x, y=field('x'), field('y')
    ta=np.maximum(t-.05, t[0]); tb=np.minimum(t+.05,t[-1])
    speed=np.hypot(np.interp(tb,t,x)-np.interp(ta,t,x),np.interp(tb,t,y)-np.interp(ta,t,y))/(tb-ta)
    dt=np.diff(t,append=t[-1])
    moving=speed>=.35
    err=field('csv_cte')
    selected=field('selected_cte')
    curv=field('curvature')
    progress=field('progress')
    result.update(duration_sim_s=float(t[-1]-t[0]), wall_reason=data['reason'],
                  distance_m=float(np.hypot(np.diff(x),np.diff(y)).sum()),
                  net_laps=float(progress[-1]/data['length']), gt_speed_peak_mps=float(speed.max()),
                  gt_speed_p95_mps=float(np.quantile(speed[moving],.95)) if moving.any() else None,
                  commanded_speed_max_mps=float(field('cmd_v').max()),
                  z_min_m=float(min(field('z'))), z_max_m=float(max(field('z'))))
    pf=np.array(data['pf'],dtype=float)
    pf_error=np.full(len(t),np.nan)
    if len(pf)>=2:
        pt,idx_pf=np.unique(pf[:,0],return_index=True)
        if len(pt)>=2:
            hi=np.clip(np.searchsorted(pt,t,side='right'),1,len(pt)-1)
            ok=(pt[hi]-pt[hi-1]<.16)&(t>=pt[0])&(t<=pt[-1])
            pf_error[ok]=np.hypot(x[ok]-np.interp(t[ok],pt,pf[idx_pf,1]), y[ok]-np.interp(t[ok],pt,pf[idx_pf,2]))
    sources=np.array([r[idx['source']] for r in rows])
    result['speed_bins']={}
    for name,mask in [('moving',moving),('low_0.35_to_2.5',moving&(speed<2.5)),
                      ('near_3', (speed>=2.5)&(speed<=3.5)),('over_4',speed>=4.),
                      ('over_5',speed>=5.),('over_6',speed>=6.)]:
        duration=float(dt[mask].sum())
        curved=mask&(abs(curv)>=.12)
        inside=(err*np.sign(curv))[curved].clip(min=0)
        result['speed_bins'][name]={
            'samples':int(mask.sum()),'sim_s':duration,'csv_cte':metric(err[mask]),
            'selected_cte':metric(selected[mask&np.isfinite(selected)]),
            'pf_to_gt':metric(pf_error[mask&np.isfinite(pf_error)]),
            'corner_s':float(dt[curved].sum()),'inside_corner_p95_m':float(np.percentile(inside,95)) if len(inside) else None,
            'inside_corner_max_m':float(inside.max()) if len(inside) else None,
            'pp_main_fraction':float(dt[mask&(sources=='pp_main')].sum()/duration) if duration>0 else None}
    return result


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('observation',type=Path)
    args=parser.parse_args()
    result=score(json.loads(args.observation.read_text()))
    dest=args.observation.with_name('score.json')
    dest.write_text(json.dumps(result,indent=2,allow_nan=False)+'\n')
    print(json.dumps(result,indent=2))
