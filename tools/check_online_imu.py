#!/usr/bin/env python3
"""Exercise causal clock/FIFO logic in actual recorded arrival order."""
import argparse
import csv
import json
from pathlib import Path
import numpy as np
from online_imu import CausalClock,FifoDecoder


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('session',type=Path);p.add_argument('--output',type=Path,required=True)
    a=p.parse_args();report=json.loads((a.session/'analysis/report.json').read_text())
    if not report['raw_capture_checks_passed']:raise ValueError('Source acquisition failed')
    with (a.session/'analysis/imu_derived.csv').open() as f:offline={int(r['sensor_ticks']):r for r in csv.DictReader(f)}
    decoder=FifoDecoder();clock=CausalClock();differences=[];latencies=[];warmup=0;compared=0
    def samples(groups):
        nonlocal warmup,compared
        for g in groups:
            other=offline[g['ticks']]
            if int(other['fifo_slot_counter'])!=g['count'] or int(other['host_received_boot_ns'])!=g['receipt_ns']:
                raise ValueError('Incremental decoder disagrees with offline data')
            actual=[v*np.pi/180*.0175 for v in g['gyro']]+[v*9.80665*.000122 for v in g['accel']]
            expected=[float(other[f'g{x}_rad_s']) for x in 'xyz']+[float(other[f'a{x}_m_s2']) for x in 'xyz']
            if not np.allclose(actual,expected,rtol=1e-14,atol=1e-15):raise ValueError('Decoded SI payload differs')
            compared+=1;ns=clock.map_sample(g['ticks'],g['receipt_ns'])
            if ns is None:warmup+=1;continue
            differences.append(ns-int(other['mapped_boot_ns']));latencies.append(g['receipt_ns']-ns)
    with (a.session/'raw.jsonl').open() as f:
        for line in f:
            record=json.loads(line)
            if record['kind']=='clock_anchor':clock.anchor(record)
            else:samples(decoder.push(record))
    tail,trimmed=decoder.finish();samples(tail)
    if compared!=len(offline):raise ValueError('Incremental decoder omitted samples')
    if len(differences)<100 or not clock.validation_residuals:raise ValueError('Insufficient causal validation')
    result={'scope':'Recorded arrival-order software test; not live physical synchronization',
        'session':str(a.session.resolve()),'compared_samples':compared,'startup_samples_explicitly_excluded':warmup,
        'trailing_incomplete_slot_trimmed':trimmed,
        'causal_vs_offline_abs_difference_us_p50_p95_max':(np.percentile(np.abs(differences),[50,95,100])/1000).tolist(),
        'prior_anchor_prediction_residual_us_p50_p95_max':(np.percentile(np.abs(clock.validation_residuals),[50,95,100])/1000).tolist(),
        'sample_receipt_latency_ms_min_p50_p95_max':(np.percentile(latencies,[0,50,95,100])/1e6).tolist(),
        'all_fifo_payloads_match_offline':True,'software_checks_passed':True,
        'limitations':'Different causal/offline clock maps retain uncertainty. No camera/IMU offset or filter delay is inferred.'}
    a.output.write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result,indent=2))


if __name__=='__main__':main()
