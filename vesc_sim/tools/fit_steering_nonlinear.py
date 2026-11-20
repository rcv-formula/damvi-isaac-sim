"""Monotone nonlinear steering calibration with independent time-block validation.

The fitting samples and physical filters match fit_steering.py. A smooth,
monotone PCHIP map can represent linkage curvature, asymmetric trim, a central
flat region and end saturation without extrapolating beyond observed input.
"""
import argparse,csv,json
from pathlib import Path
import numpy as np
import yaml
from scipy.interpolate import PchipInterpolator
from scipy.optimize import least_squares


def collect(folder,lag,wheelbase=.26):
    datasets=[];pieces=[]
    for bi,p in enumerate(sorted(folder.glob('*.npz'))):
        d=np.load(p);datasets.append((p.stem,d))
        t=d['t'];q=t-lag
        u=np.interp(q,t,np.nan_to_num(d['u']))
        us=np.interp(q,t,np.nan_to_num((d['servo']-.5)/.52))
        valid=np.interp(q,t,(np.isfinite(d['u'])&np.isfinite(d['servo'])).astype(float))>.999
        y=np.arctan(np.divide(wheelbase*d['r'],d['v'],out=np.full_like(t,np.nan),where=d['v']>.1))
        m=d['strict']&valid&(abs(u-us)<.04)&np.isfinite(y)&(abs(u)<=.9)
        block=(bi*10000+np.floor(t[m]/2)).astype(int)
        pieces.append(np.column_stack([u[m],y[m],np.full(m.sum(),bi),block,t[m],d['v'][m]]))
    return datasets,np.concatenate(pieces)


KNOTS=np.array([-.872664626,-.70,-.50,-.30,-.15,0.,.15,.30,.50,.70,.872664626])
LIMIT=.301


def nonlinear_fit(x,y,penalty,weights=None):
    if weights is None:weights=np.ones_like(x)
    weights=weights/weights.sum()
    initial=np.clip(.40*KNOTS+.01447,-.28,.28)+np.arange(len(KNOTS))*1e-7
    gaps=np.diff(np.r_[-LIMIT,initial,LIMIT])
    initial_parameters=np.log(gaps[:-1]/gaps[-1])
    def angles(parameters):
        logs=np.r_[parameters,0.];gaps=np.exp(logs-np.max(logs));gaps/=gaps.sum()
        return -LIMIT+2*LIMIT*np.cumsum(gaps[:-1])
    # Second divided differences penalize unnecessary linkage curvature.
    def residuals(parameters):
        values=angles(parameters)
        pred=PchipInterpolator(KNOTS,values,extrapolate=False)(np.clip(x,KNOTS[0],KNOTS[-1]))
        residual=(pred-y)/.02
        data=np.sign(residual)*np.sqrt(weights*2*.02**2*(np.sqrt(1+residual**2)-1))
        slope=np.diff(values)/np.diff(KNOTS)
        return np.r_[data,np.sqrt(penalty/(len(slope)-1))*np.diff(slope)]
    result=least_squares(residuals,initial_parameters,bounds=(-20,20),
                         ftol=1e-9,xtol=1e-9,gtol=1e-9,max_nfev=1500)
    if not result.success:raise RuntimeError(result.message)
    values=angles(result.x)
    # Remove sub-microradian solver noise that can violate strict monotonicity.
    values=np.maximum.accumulate(values)
    return values


def predict(values,x):
    return PchipInterpolator(KNOTS,values)(np.clip(x,KNOTS[0],KNOTS[-1]))


def linear_fit(x,y):
    return least_squares(lambda b:np.clip(b[0]+b[1]*x,-LIMIT,LIMIT)-y,[.01,.4],
                         loss='soft_l1',f_scale=.02).x


def run(aligned,output):
    lag=.07
    datasets,samples=collect(aligned,lag)
    x,y,bag,blocks,t,speed=samples.T;blocks=blocks.astype(int);bag=bag.astype(int)
    train=blocks%5!=0;test=~train
    baseline=linear_fit(x[train],y[train])
    baseline_rmse=float(np.sqrt(np.mean((np.clip(baseline[0]+baseline[1]*x[test],-LIMIT,LIMIT)-y[test])**2)))
    candidates=[]
    for penalty in [0.,.00001,.00003,.0001,.0003,.001,.003,.01,.03]:
        values=nonlinear_fit(x[train],y[train],penalty)
        error=predict(values,x[test])-y[test]
        rmse=float(np.sqrt(np.mean(error**2)))
        candidates.append({'penalty':penalty,'holdout_rmse_deg':float(np.rad2deg(rmse))})
        print(candidates[-1],flush=True)
    best=min(candidates,key=lambda r:r['holdout_rmse_deg'])
    # Prefer a smoother map if validation differs by <= 0.03 degrees.
    chosen=max((c for c in candidates if c['holdout_rmse_deg']<=best['holdout_rmse_deg']+.03),key=lambda r:r['penalty'])
    values=nonlinear_fit(x,y,chosen['penalty']);pred=predict(values,x)
    per_bag=[]
    for bi,(name,d) in enumerate(datasets):
        m=bag==bi
        item={'bag':name,'selected_seconds':float(m.sum()*.02),'samples':int(m.sum())}
        if m.any():
            item['rmse_deg']=float(np.rad2deg(np.sqrt(np.mean((pred[m]-y[m])**2))))
            if (~m).sum()>100:
                other=nonlinear_fit(x[~m],y[~m],chosen['penalty'])
                item['leave_one_bag_out_rmse_deg']=float(np.rad2deg(np.sqrt(np.mean((predict(other,x[m])-y[m])**2))))
        per_bag.append(item)
    rng=np.random.default_rng(1538);unique=np.unique(blocks);boots=[]
    for _ in range(100):
        draws=rng.choice(unique,len(unique),replace=True)
        counts={b:int((draws==b).sum()) for b in unique}
        weights=np.array([counts[b] for b in blocks],dtype=float)
        boots.append(nonlinear_fit(x,y,chosen['penalty'],weights))
    ci=np.percentile(boots,[2.5,97.5],axis=0)
    config=yaml.safe_load((output/'steering.yaml').read_text())
    config.pop('offset_rad',None);config.pop('positive_gain',None);config.pop('negative_gain',None)
    config.update(model='monotone_pchip',input_knots_rad=KNOTS.tolist(),wheel_angle_knots_rad=values.tolist())
    config['calibration'].update(blocked_holdout_rmse_deg=chosen['holdout_rmse_deg'],nonlinear_regularization=chosen['penalty'])
    config['calibration']['report']='../calibration/steering/nonlinear_report.json'
    report={'model':'monotone_pchip','command_to_imu_lag_s':lag,'selected_seconds':len(x)*.02,
            'selected_samples':len(x),'knot_input_rad':KNOTS.tolist(),'knot_angle_rad':values.tolist(),
            'knot_block_bootstrap_95pct_rad':ci.tolist(),'block_size_s':2.,'bootstrap_samples':100,
            'linear_baseline_holdout_rmse_deg':float(np.rad2deg(baseline_rmse)),
            'nonlinear_holdout_rmse_deg':chosen['holdout_rmse_deg'],
            'regularization_candidates':candidates,'chosen_penalty':chosen['penalty'],
            'overall_rmse_deg':float(np.rad2deg(np.sqrt(np.mean((pred-y)**2)))),
            'per_bag':per_bag,
            'limitations':['This is a kinematic equivalent angle inferred from IMU yaw and wheel speed, not a direct mechanical wheel-angle measurement.',
                'No cmd_vel is recorded; /drive steering_angle is the proxy, matched to executed servo commands.',
                'Data are sparse near the input limits. The curve clamps there and the endpoint uncertainty is reported.',
                'Record-to-record slip/trim variations remain. A wheel-angle sensor or static servo sweep is needed to isolate linkage geometry.',
                'The selected nonlinear form is compared against the earlier affine map using the same time-block holdout.']}
    (output/'nonlinear_report.json').write_text(json.dumps(report,indent=2))
    (output/'steering_nonlinear.yaml').write_text(yaml.safe_dump(config,sort_keys=False))
    with (output/'nonlinear_knots.csv').open('w',newline='') as f:
        w=csv.writer(f);w.writerow(['input_rad','angle_rad','ci_low_rad','ci_high_rad'])
        w.writerows(zip(KNOTS,values,ci[0],ci[1]))
    with (output/'pchip_reference.csv').open('w',newline='') as f:
        w=csv.writer(f);w.writerow(['input_rad','angle_rad'])
        grid=np.linspace(-1.,1.,401)
        w.writerows(zip(grid,predict(values,grid)))
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig,ax=plt.subplots(figsize=(9,5),constrained_layout=True)
    for bi,(name,d) in enumerate(datasets):
        m=bag==bi
        if m.any():ax.scatter(np.rad2deg(x[m]),np.rad2deg(y[m]),s=7,alpha=.25,label=name)
    grid=np.linspace(KNOTS[0],KNOTS[-1],400)
    ax.plot(np.rad2deg(grid),np.rad2deg(predict(values,grid)),'k',lw=2,label='Monotone nonlinear map')
    ax.plot(np.rad2deg(grid),np.rad2deg(np.clip(baseline[0]+baseline[1]*grid,-LIMIT,LIMIT)),'k--',lw=1,label='Earlier affine baseline')
    ax.plot(np.rad2deg(KNOTS),np.rad2deg(values),'ko',ms=4)
    ax.fill_between(np.rad2deg(KNOTS),np.rad2deg(ci[0]),np.rad2deg(ci[1]),color='black',alpha=.1,label='2 s block bootstrap 95% interval')
    ax.set(xlabel='Nominal steering command (deg)',ylabel='IMU equivalent wheel angle (deg)',
           title=f'Nonlinear map: holdout {chosen["holdout_rmse_deg"]:.2f} deg; affine {np.rad2deg(baseline_rmse):.2f} deg')
    ax.grid(alpha=.25);ax.legend(fontsize=8);fig.savefig(output/'nonlinear_fit.png',dpi=180);plt.close(fig)
    print(json.dumps({k:report[k] for k in ['linear_baseline_holdout_rmse_deg','nonlinear_holdout_rmse_deg','chosen_penalty','knot_input_rad','knot_angle_rad','per_bag']},indent=2),flush=True)


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('aligned',type=Path);p.add_argument('-o','--output',type=Path,required=True)
    args=p.parse_args();run(args.aligned,args.output)
