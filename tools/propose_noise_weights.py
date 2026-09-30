#!/usr/bin/env python3
"""Create explicit experimental fitting weights from measured Allan envelopes.

These are conservative surrogate weights, NOT identified white/random-walk
coefficients. Hardware use requires separate review and sensitivity validation.
"""
import argparse
import hashlib
import json
from pathlib import Path
import numpy as np
import yaml


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('reports',type=Path,nargs='+');p.add_argument('--output',type=Path,required=True)
    p.add_argument('--rate-source',type=Path,required=True)
    p.add_argument('--inflation',type=float,default=10)
    a=p.parse_args()
    if not np.isfinite(a.inflation) or a.inflation<=0:p.error('Inflation must be finite and positive')
    whites=[];walks=[];sources=[]
    for folder in a.reports:
        report=json.loads((folder/'report.json').read_text())
        data=np.genfromtxt(folder/'allan.csv',delimiter=',',names=True);tau=data['tau_s']
        names=['gx_rad_s','gy_rad_s','gz_rad_s','ax_m_s2','ay_m_s2','az_m_s2']
        A=np.column_stack([data[n] for n in names])
        short=(tau>=.005)&(tau<=.3);long=(tau>=10)&(tau<=30)
        if not short.any():raise ValueError('Short-time observations missing')
        whites.append(np.max(A[short]*np.sqrt(tau[short,None]),axis=0))
        if long.any():walks.append(np.max(A[long]*np.sqrt(3/tau[long,None]),axis=0))
        sources.append({'report':str(folder/'report.json'),'report_sha256':hashlib.sha256((folder/'report.json').read_bytes()).hexdigest(),
            'allan_sha256':hashlib.sha256((folder/'allan.csv').read_bytes()).hexdigest(),
            'window_s':[report['window_start_s'],report['window_end_s']]})
    if not walks:raise ValueError('No long-time envelope data')
    white=np.max(whites,axis=0);walk=np.max(walks,axis=0)
    rate=json.loads(a.rate_source.read_text())['imu_observed_hz']
    values={'rostopic':'/imu0','update_rate':rate,
        'gyroscope_noise_density':float(max(white[:3])*a.inflation),
        'accelerometer_noise_density':float(max(white[3:])*a.inflation),
        'gyroscope_random_walk':float(max(walk[:3])*a.inflation),
        'accelerometer_random_walk':float(max(walk[3:])*a.inflation)}
    a.output.mkdir()
    r={'accepted_as_identified_sensor_noise':False,'accepted_for_hardware_estimation':False,
        'purpose':'Provisional weights for exploratory joint fitting and sensitivity tests only',
        'method':'Across explicitly selected quiet windows and axes, take max(AD*sqrt(tau)) for 0.005–0.3 s and max(AD*sqrt(3/tau)) for 10–30 s; multiply all scalar model weights by recorded inflation.',
        'interpretation':'The long-time quantity includes white/colored noise, temperature/tilt drift and is an empirical surrogate, not a detected +1/2 random-walk slope or statistical confidence bound.',
        'short_envelope_axes_si':white.tolist(),'long_envelope_axes_si':walk.tolist(),
        'inflation':a.inflation,'yaml_values':values,'sources':sources,'sample_rate_source':str(a.rate_source),
        'limits':['Complete 30-minute dataset is rejected as stationary because of disturbances.',
            'Quiet windows are not concatenated and raw data remains unchanged.',
            'No accepted long-term random-walk identification exists.',
            'Compare 0.5x and 2x weights, residuals, covariance and independent motion data before any calibration acceptance.'],
        'official_guidance':['https://github.com/ethz-asl/kalibr/wiki/IMU-Noise-Model','https://docs.openvins.com/gs-calibration.html']}
    (a.output/'review.json').write_text(json.dumps(r,indent=2)+'\n')
    (a.output/'noise.yaml').write_text('# EXPERIMENTAL FITTING WEIGHTS, NOT IDENTIFIED SENSOR NOISE. See review.json.\n'+yaml.safe_dump(values,sort_keys=False))
    print(json.dumps(r,indent=2))


if __name__=='__main__':main()
