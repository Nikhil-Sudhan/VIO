#!/usr/bin/env python3
"""Provisional overlapping Allan deviation from a checked stationary recording.

No estimator YAML is generated: bias random walk needs a longer, reviewed run.
Gravity and constant bias remain in the saved input and time-series plots.
"""
import argparse
import hashlib
import json
from pathlib import Path
import numpy as np


def overlapping_adev(samples, sizes):
    values=np.asarray(samples,dtype=float)
    if values.ndim==1:values=values[:,None]
    if not np.isfinite(values).all():raise ValueError('Nonfinite input')
    c=np.vstack([np.zeros(values.shape[1]),np.cumsum(values,axis=0)])
    result=[]
    for m in sizes:
        if m<1 or 2*m>=len(values):raise ValueError('Invalid averaging size')
        d=(c[2*m:]-2*c[m:-m]+c[:-2*m])/m
        result.append(np.sqrt(np.mean(d*d,axis=0)/2))
    return np.array(result)


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('session',type=Path);p.add_argument('--output',type=Path,required=True)
    p.add_argument('--start-s',type=float,default=5,help='Explicit warmup/window trim; raw data untouched')
    p.add_argument('--end-s',type=float)
    a=p.parse_args()
    report_path=a.session/'analysis/report.json'
    report=json.loads(report_path.read_text())
    if not report['raw_capture_checks_passed']:raise ValueError('Acquisition checks failed')
    source=a.session/'analysis/imu_derived.csv'
    raw=np.genfromtxt(source,delimiter=',',names=True)
    t=(raw['mapped_boot_ns']-raw['mapped_boot_ns'][0])*1e-9
    keep=t>=a.start_s
    if a.end_s is not None:keep &= t<=a.end_s
    t=t[keep];raw=raw[keep]
    if len(t)<1000:raise ValueError('Need at least 1000 contiguous samples')
    ticks=np.diff(raw['sensor_ticks'])
    if not np.all(ticks==ticks[0]):raise ValueError('Hardware sample gaps')
    dt=float(np.mean(np.diff(t)));variation=float(np.max(np.abs(np.diff(t)/dt-1)))
    if variation>.005:raise ValueError('Clock variation too large for uniform-rate Allan analysis')
    names=['gx_rad_s','gy_rad_s','gz_rad_s','ax_m_s2','ay_m_s2','az_m_s2']
    values=np.column_stack([raw[k] for k in names])
    sizes=np.unique(np.logspace(0,np.log10(len(t)//20),60).astype(int))
    tau=sizes*dt;adev=overlapping_adev(values,sizes)
    mask=(tau>=.03)&(tau<=.3)
    fits=[]
    for k in range(6):
        slope,intercept=np.polyfit(np.log(tau[mask]),np.log(adev[mask,k]),1)
        candidate=float(np.exp(np.mean(np.log(adev[mask,k])+.5*np.log(tau[mask]))))
        fits.append({'axis':names[k],'log_log_slope':float(slope),
            'white_slope_compatible':bool(-.65<slope<-.35),
            'provisional_density_si_per_sqrt_hz':candidate if -.65<slope<-.35 else None})
    # A predeclared long-time window tests the +1/2 slope rather than taking an
    # arbitrary Allan minimum as a random-walk estimate. All candidates still
    # require stationarity, residual and duration review before estimator use.
    walk_mask=(tau>=10)&(tau<=100)
    walk_fits=[]
    for k in range(6):
        fit={'axis':names[k],'random_walk_slope_compatible':False,'provisional_bias_diffusion_si':None}
        if walk_mask.sum()>=6 and tau[walk_mask][-1]/tau[walk_mask][0]>=3:
            slope,intercept=np.polyfit(np.log(tau[walk_mask]),np.log(adev[walk_mask,k]),1)
            fit['log_log_slope']=float(slope)
            compatible=.35<slope<.65
            fit['random_walk_slope_compatible']=bool(compatible)
            if compatible:
                fit['provisional_bias_diffusion_si']=float(np.sqrt(3)*np.exp(np.mean(np.log(adev[walk_mask,k])-.5*np.log(tau[walk_mask]))))
        else:fit['reason']='Insufficient long-time averaging range with at least ten independent pairs'
        walk_fits.append(fit)
    block_statistics=[]
    gyro_center=np.median(values[:,:3],axis=0)
    for start in np.arange(t[0],t[-1],60):
        block=values[(t>=start)&(t<start+60)]
        if len(block)<20:continue
        block_statistics.append({'start_s':float(start),'samples':len(block),
            'mean_si':block.mean(axis=0).tolist(),'std_si':block.std(axis=0).tolist(),
            'gyro_max_deviation_from_full_window_median_rad_s':float(np.max(np.linalg.norm(block[:,:3]-gyro_center,axis=1)))})
    temperature=[]
    temp_path=a.session/'temperature.jsonl'
    if temp_path.exists():
        origin=int(np.genfromtxt(source,delimiter=',',names=True,max_rows=1)['mapped_boot_ns'])
        for line in temp_path.read_text().splitlines():
            r=json.loads(line)
            when=((r['host_before_boot_ns']+r['host_after_boot_ns'])/2-origin)*1e-9
            if t[0]<=when<=t[-1]:temperature.append((when,r['temperature_c']))
    a.output.mkdir()
    result={'accepted_for_estimator':False,'scope':'Short-run noise characterization; physical stationarity must be reviewed',
        'source':str(a.session.resolve()),'input_sha256':hashlib.sha256(source.read_bytes()).hexdigest(),
        'analysis_sha256':hashlib.sha256(report_path.read_bytes()).hexdigest(),
        'window_start_s':float(t[0]),'window_end_s':float(t[-1]),'duration_s':float(t[-1]-t[0]),
        'samples':len(t),'mean_dt_s':dt,'relative_dt_max_deviation':variation,
        'mean_native_axes_si':dict(zip(names,values.mean(axis=0).tolist())),
        'white_fit_tau_s':[.03,.3],'white_noise_candidates':fits,'bias_random_walk':None,
        'random_walk_fit_tau_s':[10,100],'bias_random_walk_candidates':walk_fits,
        'si_axis_order':names,'minute_blocks':block_statistics,
        'temperature':{'samples':len(temperature),'min_max_c':[float(min(v[1] for v in temperature)),float(max(v[1] for v in temperature))],
            'first_last_c':[temperature[0][1],temperature[-1][1]]} if temperature else None,
        'notes':['Averaging uses actual consecutive hardware samples and mean mapped sample interval; no interpolated samples.',
                 'Allan estimates overlap; plotted point count is not an independent sample count.',
                 'No accepted random-walk parameter is generated automatically; any slope-compatible candidate still needs stationarity and duration review.',
                 'Kalibr recommends 15–24 hours stationary data for full noise identification; temperature and motion errors need separate assessment.'],
        'source_documentation':'https://github.com/ethz-asl/kalibr/wiki/IMU-Noise-Model'}
    (a.output/'report.json').write_text(json.dumps(result,indent=2)+'\n')
    if temperature:
        np.savetxt(a.output/'temperature.csv',temperature,delimiter=',',header='receipt_t_rel_s,die_temperature_c',comments='')
    np.savetxt(a.output/'allan.csv',np.column_stack([tau,len(t)//(2*sizes),adev]),delimiter=',',
        header='tau_s,nonoverlapping_pair_count,'+','.join(names),comments='')
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig,axes=plt.subplots(2,2,figsize=(12,8))
    for group,unit in [(0,'rad/s'),(1,'m/s²')]:
        for k,axis in enumerate('xyz'):
            j=group*3+k
            axes[group,0].loglog(tau,adev[:,j],label=axis)
            axes[group,1].plot(t[::10],values[::10,j],linewidth=.5,label=axis)
        axes[group,0].set(xlabel='Averaging time (s)',ylabel=f'Allan deviation ({unit})')
        axes[group,1].set(xlabel='Time from recording start (s)',ylabel=f'Native measurement ({unit})')
        axes[group,0].legend();axes[group,1].legend()
    fig.suptitle('Provisional IMU noise — no accepted bias random-walk estimate')
    fig.tight_layout();fig.savefig(a.output/'allan.png',dpi=150);fig.savefig(a.output/'allan.pdf')
    if temperature:
        fig2,axes2=plt.subplots(3,1,figsize=(11,8),sharex=True)
        block_t=[v['start_s']+30 for v in block_statistics]
        means=np.array([v['mean_si'] for v in block_statistics])
        for k,axis in enumerate('xyz'):
            axes2[0].plot(block_t,means[:,k],'.-',label=axis)
            axes2[1].plot(block_t,means[:,k+3],'.-',label=axis)
        axes2[0].set_ylabel('Minute mean gyro (rad/s)');axes2[1].set_ylabel('Minute mean accel (m/s²)')
        axes2[0].legend();axes2[1].legend()
        axes2[2].plot(*np.array(temperature).T);axes2[2].set(xlabel='Time from recording start (s)',ylabel='IMU die temperature (°C)')
        fig2.suptitle('Stationarity and temperature review; mean acceleration includes gravity')
        fig2.tight_layout();fig2.savefig(a.output/'stationarity-temperature.png',dpi=150);fig2.savefig(a.output/'stationarity-temperature.pdf')
    print(json.dumps(result,indent=2))


if __name__=='__main__':main()
