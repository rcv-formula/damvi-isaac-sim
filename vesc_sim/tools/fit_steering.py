"""Low-speed, low-acceleration equivalent-steering identification from MCAP signals.

This estimates a kinematic equivalent angle, not a direct wheel encoder angle.
Use extract_steering_bags.py first. SciPy/matplotlib are analysis dependencies only.
"""
import argparse,configparser,json
from pathlib import Path
import numpy as np
from scipy.signal import savgol_filter
from scipy.optimize import least_squares

DT=.02

def clean(a,column=0):
    order=np.argsort(a[:,column],kind='stable');a=a[order]
    return a[np.r_[np.diff(a[:,column])>1e-7,True]]

def interp(a,t,col,timecol=0,hold=False):
    if not a.size:return np.full_like(t,np.nan)
    a=clean(a,timecol);st=a[:,timecol]
    if hold:
        i=np.searchsorted(st,t,side='right')-1
        out=a[np.clip(i,0,len(a)-1),col].copy()
        out[(i<0)|(t-st[np.clip(i,0,len(a)-1)]>.15)]=np.nan
        return out
    out=np.interp(t,st,a[:,col],left=np.nan,right=np.nan)
    i=np.searchsorted(st,t);nearest=np.minimum(abs(t-st[np.clip(i,0,len(a)-1)]),abs(t-st[np.clip(i-1,0,len(a)-1)]))
    out[nearest>.1]=np.nan
    return out

def smooth(v,window=.3,deriv=0):
    n=int(round(window/DT))|1
    good=np.isfinite(v)
    if good.sum()<n:return np.full_like(v,np.nan)
    filled=np.interp(np.arange(len(v)),np.flatnonzero(good),v[good])
    out=savgol_filter(filled,n,3,deriv=deriv,delta=DT)
    out[~good]=np.nan
    return out

def yaw(q):
    x,y,z,w=q.T
    return np.unwrap(np.arctan2(2*(w*z+x*y),1-2*(y*y+z*z)))

def prepare(path):
    d=np.load(path)
    im=clean(d['imu'],1);core=clean(d['core'])
    start=max(im[0,1],core[0,0])+1.;end=min(im[-1,1],core[-1,0])-1.
    t=np.arange(start,end,DT)
    q=im[:,2:6]
    yaw_samples=np.column_stack([im[:,1],yaw(q)])
    heading=interp(yaw_samples,t,1)
    r=smooth(heading,.3,deriv=1)
    gyro=smooth(interp(im,t,8,timecol=1),.15)
    v=smooth(interp(core,t,2)/1538.,.3)
    acc=smooth(v,.3,deriv=1)
    command=interp(d['drive'],t,2,hold=True)
    servo=interp(d['servo'],t,1,hold=True)
    # PF map positions provide a separate speed consistency check. Their local
    # smoothing reduces scan-correction jumps; these positions are not ground truth.
    pf=d['pf'];px=interp(pf,t,2);py=interp(pf,t,3)
    vpf=np.hypot(smooth(px,.5,1),smooth(py,.5,1))
    apf=smooth(vpf,.5,1)
    vwheel=np.hypot(interp(d['wheel'],t,8),interp(d['wheel'],t,9))
    # Reject the full smoothing support at acquisition gaps and bag edges.
    base=(v>=.6)&(v<=2.5)&(np.abs(acc)<=.5)&(np.abs(r)*v<=1.5)
    base&=np.isfinite(r)&np.isfinite(gyro)&(np.abs(r-gyro)<.35)
    speed_agree=np.abs(vpf-v)<=np.maximum(.20,.20*v)
    strict=base&speed_agree&(vpf<=2.5)&(np.abs(apf)<=.7)
    ratio=np.divide(.26*r,v,out=np.full_like(v,np.nan),where=np.abs(v)>1e-6)
    result={'t':t,'u':command,'servo':servo,'v':v,'a':acc,'vpf':vpf,'apf':apf,'r':r,'gyro':gyro,
            'base':base,'strict':strict,'delta':np.arctan(ratio),'old_wheel_speed':vwheel}
    return result

def robust_linear(x,y):
    m=np.isfinite(x)&np.isfinite(y)
    fit=least_squares(lambda b:(b[0]*x[m]+b[1]-y[m]),[.35,0],loss='soft_l1',f_scale=.02)
    return fit.x,np.sqrt(np.mean((fit.x[0]*x[m]+fit.x[1]-y[m])**2))

def identify(aligned,output,ini):
    import yaml
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    cfg=configparser.ConfigParser(inline_comment_prefixes=('#',));cfg.read(ini)
    vehicle=json.loads(cfg['OPTIMIZATION_OPTIONS']['vehicle_params_mintime'])
    wheelbase=vehicle['wheelbase_front']+vehicle['wheelbase_rear']
    limit=vehicle['delta_max']
    datasets=[(p.stem,np.load(p)) for p in sorted(aligned.glob('*.npz'))]

    def collect(lag):
        xs=[];ys=[];bags=[];blocks=[];speeds=[];times=[]
        for bi,(name,d) in enumerate(datasets):
            t=d['t'];query=t-lag
            u=np.interp(query,t,np.nan_to_num(d['u']))
            servo_u=np.interp(query,t,np.nan_to_num((d['servo']-.5)/.52))
            fresh=np.interp(query,t,(np.isfinite(d['u'])&np.isfinite(d['servo'])).astype(float))>.999
            m=d['strict']&fresh&(np.abs(u-servo_u)<.04)
            delta=np.arctan(np.divide(wheelbase*d['r'],d['v'],out=np.full_like(t,np.nan),where=d['v']>.1))
            m&=np.isfinite(delta)&(np.abs(u)<=.9)
            xs.extend(u[m]);ys.extend(delta[m]);bags.extend([bi]*m.sum())
            blocks.extend((bi*10000+np.floor(t[m]/2)).astype(int));speeds.extend(d['v'][m]);times.extend(t[m])
        return tuple(np.asarray(v) for v in (xs,ys,bags,blocks,speeds,times))

    def predict(b,x,model):
        angle=b[0]+b[1]*x if model=='linear' else b[0]+b[1]*np.maximum(x,0)+b[2]*np.minimum(x,0)
        return np.clip(angle,-limit,limit)

    def fit(x,y,model,weights=None):
        init=[.01,.4] if model=='linear' else [.01,.4,.4]
        low=[-.08,.05] if model=='linear' else [-.08,.05,.05]
        high=[.08,1.2] if model=='linear' else [.08,1.2,1.2]
        w=np.ones_like(x) if weights is None else weights
        return least_squares(lambda b:(predict(b,x,model)-y)*np.sqrt(w),init,
                             bounds=(low,high),loss='soft_l1',f_scale=.02).x

    candidates=[]
    for lag in np.arange(0,.301,.01):
        x,y,bag,block,v,t=collect(lag)
        train=(block%5)!=0;test=~train
        if train.sum()<100 or test.sum()<30:raise ValueError('Insufficient calibration samples')
        b=fit(x[train],y[train],'linear')
        train_loss=np.mean(np.minimum((predict(b,x[train],'linear')-y[train])**2,.08**2))
        candidates.append((float(train_loss),float(lag)))
    lag=min(candidates)[1]
    x,y,bag,block,v,t=collect(lag)
    train=(block%5)!=0;test=~train
    model_scores={}
    for model in ('linear','asymmetric_linear'):
        b=fit(x[train],y[train],model)
        model_scores[model]=float(np.sqrt(np.mean((predict(b,x[test],model)-y[test])**2)))
    model='asymmetric_linear' if model_scores['asymmetric_linear']<.95*model_scores['linear'] else 'linear'
    coefficients=fit(x,y,model)
    pred=predict(coefficients,x,model)
    validation=[]
    for bi,(name,d) in enumerate(datasets):
        mask=bag==bi
        item={'bag':name,'total_seconds':len(d['t'])*DT,'speed_accel_selected_s':float(d['base'].sum()*DT),
              'slip_consistency_selected_s':float(d['strict'].sum()*DT),'command_matched_s':float(mask.sum()*DT),
              'selected_samples':int(mask.sum())}
        if mask.sum():
            item.update(rmse_deg=float(np.rad2deg(np.sqrt(np.mean((pred[mask]-y[mask])**2)))),
                        command_range_rad=np.percentile(x[mask],[1,99]).tolist())
            if (~mask).sum()>100:
                other=fit(x[~mask],y[~mask],model)
                item['leave_one_bag_out_rmse_deg']=float(np.rad2deg(np.sqrt(np.mean((predict(other,x[mask],model)-y[mask])**2))))
        validation.append(item)
    # Resample entire 2-second blocks, not correlated 50 Hz rows.
    rng=np.random.default_rng(1538);unique=np.unique(block);boots=[]
    for _ in range(200):
        sampled=rng.choice(unique,len(unique),replace=True)
        counts={b:int((sampled==b).sum()) for b in unique}
        weights=np.asarray([counts[b] for b in block],dtype=float)
        boots.append(fit(x,y,model,weights))
    ci=np.percentile(boots,[2.5,97.5],axis=0)
    config={'enabled':True,'input_topic':'/cmd_vel','input_field':'angular.z',
            'input_semantics':'steering_angle_rad; not yaw_rate_rad_s',
            'model':model,'offset_rad':float(coefficients[0]),
            'positive_gain':float(coefficients[1]),
            'negative_gain':float(coefficients[1] if model=='linear' else coefficients[2]),
            'input_limit_rad':float(np.deg2rad(50)),
            'center_angle_limit_rad':float(limit),
            'actuator_time_constant_s':float(vehicle['t_delta']),
            'actuator_rate_limit_rad_s':3.2,
            'identified_command_to_imu_lag_s':lag,
            'lag_applied_as_extra_delay':False,
            'calibration_wheelbase_m':float(wheelbase),
            'simulation_wheelbase_m':.25,'simulation_front_track_m':.20,
            'geometry_policy':'Preserve existing USD dimensions; angle mapping is calibrated using the real INI wheelbase.',
            'joint_stiffness':10.0,'joint_damping':.1,'joint_max_torque_nm':1.0,
            'calibration':{'method':'low-speed IMU-yaw/physical-ERPM kinematic equivalent, not direct wheel encoder',
                'command_proxy':'/drive.drive.steering_angle, only when actual servo agrees with 0.5+0.52*command',
                'speed_max_mps':2.5,'speed_min_mps':.6,'longitudinal_accel_max_mps2':.5,
                'lateral_accel_max_mps2':1.5,'pf_speed_agreement_relative':.2,
                'pf_speed_agreement_absolute_mps':.2,'source_ini':str(ini),
                'selected_seconds':float(len(x)*DT),'blocked_holdout_rmse_deg':float(np.rad2deg(model_scores[model])),
                'report':'../calibration/steering/report.json'}}
    output.mkdir(parents=True,exist_ok=True)
    (output/'steering.yaml').write_text(yaml.safe_dump(config,sort_keys=False))
    report={'model':model,'coefficients_offset_positive_negative':
                [config['offset_rad'],config['positive_gain'],config['negative_gain']],
            'coefficient_block_bootstrap_95pct':ci.tolist(),'block_size_s':2.0,'bootstrap_samples':200,
            'selected_samples':len(x),'selected_seconds':len(x)*DT,'source_ini':str(ini),
            'wheelbase_m':wheelbase,'angle_limit_rad':limit,'delay_grid':candidates,'selected_lag_s':lag,
            'blocked_holdout_rmse_deg':{k:float(np.rad2deg(z)) for k,z in model_scores.items()},
            'overall_rmse_deg':float(np.rad2deg(np.sqrt(np.mean((pred-y)**2)))),
            'identity_mapping_rmse_deg':float(np.rad2deg(np.sqrt(np.mean((x-y)**2)))),
            'per_bag':validation,
            'limitations':['No cmd_vel or wheel angle encoder is recorded. Existing adapter copies cmd_vel.angular.z into Ackermann steering_angle.',
                'This is a low-slip equivalent angle; residual tire slip and localization/IMU errors cannot be separated from linkage calibration.',
                'PF position-derived speed is a consistency filter, not independent ground truth.',
                'Lag includes sensor/vehicle response; it is not applied as an additional servo delay.',
                'Actual firmware/servo settings may differ across bags; leave-one-bag-out errors quantify transfer.',
                'The simulation wheelbase remains 0.25 m while the calibration vehicle wheelbase is 0.26 m.']}
    (output/'report.json').write_text(json.dumps(report,indent=2))
    import csv
    with (output/'selected_samples.csv').open('w',newline='') as f:
        writer=csv.writer(f);writer.writerow(['bag','time_s','command_rad','equivalent_angle_rad','predicted_angle_rad','speed_mps','block'])
        writer.writerows((datasets[int(b)][0],tt,xx,yy,pp,vv,int(bb)) for b,tt,xx,yy,pp,vv,bb in zip(bag,t,x,y,pred,v,block))
    fig,axes=plt.subplots(1,2,figsize=(11,4.3),constrained_layout=True)
    for bi,(name,d) in enumerate(datasets):
        m=bag==bi
        if m.any():axes[0].scatter(np.rad2deg(x[m]),np.rad2deg(y[m]),s=7,alpha=.35,label=name)
    grid=np.linspace(-.9,.9,400)
    axes[0].plot(np.rad2deg(grid),np.rad2deg(predict(coefficients,grid,model)),'k',lw=2,label='fitted + physical limit')
    axes[0].set(xlabel='Nominal steering command (deg)',ylabel='IMU equivalent wheel angle (deg)',title='Selected low-speed, gentle-acceleration data')
    axes[0].grid(alpha=.25);axes[0].legend(fontsize=7)
    axes[1].scatter(np.rad2deg(pred),np.rad2deg(y),c=v,cmap='viridis',s=6,alpha=.4)
    axes[1].plot([-20,20],[-20,20],'k--',lw=1)
    axes[1].set(xlabel='Mapped angle (deg)',ylabel='IMU equivalent angle (deg)',title=f'Blocked holdout RMSE {np.rad2deg(model_scores[model]):.2f} deg')
    axes[1].grid(alpha=.25);fig.savefig(output/'fit.png',dpi=180);plt.close(fig)
    print(json.dumps({k:report[k] for k in ['model','coefficients_offset_positive_negative','selected_seconds','selected_lag_s','blocked_holdout_rmse_deg','overall_rmse_deg','per_bag']},indent=2),flush=True)


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('cache',type=Path);p.add_argument('-o','--output',type=Path,required=True)
    a=p.parse_args();a.output.mkdir(parents=True,exist_ok=True)
    for path in sorted(a.cache.glob('*.npz')):
        d=prepare(path);mask=d['strict']&np.isfinite(d['u'])
        both=np.isfinite(d['u'])&np.isfinite(d['servo'])
        print(path.stem,'seconds',len(d['t'])*DT,'selected base/strict',d['base'].sum()*DT,d['strict'].sum()*DT,
              'cmd fit',robust_linear(d['u'][mask],d['delta'][mask]) if mask.sum()>30 else None,
              'servo_cmd',robust_linear(d['u'][both],d['servo'][both]) if both.sum()>30 else None,
              'vpf/wheel physical ratio',np.nanmedian(d['vpf'][d['base']]/d['v'][d['base']]),flush=True)
        np.savez_compressed(a.output/(path.stem+'.npz'),**d)
    identify(a.output,a.output.parent,Path('/home/shin/Desktop/raceline/Raceline-Optimization/params/f110_rgb.ini'))
