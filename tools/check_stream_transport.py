#!/usr/bin/env python3
"""Inject transport faults into the actual C++ adapter, using reference config only."""
import argparse
import json
import subprocess
import tempfile
from pathlib import Path
import numpy as np
from stream_protocol import MAGIC,HEADER,IMU,camera_packet,imu_packet,end_packet

ROOT=Path(__file__).resolve().parents[1]


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--output',type=Path,required=True);a=p.parse_args()
    good=imu_packet(1_000_000_000,0,0,[0,0,0,0,0,9.8])
    cases={
        'bad_magic':(b'XXXXXXXX','Wrong protocol'),
        'truncated_header':(MAGIC+b'\x01','Truncated'),
        'oversized_length':(MAGIC+HEADER.pack(1,1_000_000_000,0,0,4_000_001),'oversized'),
        'nonfinite_imu':(MAGIC+HEADER.pack(1,1_000_000_000,0,0,48)+IMU.pack(float('nan'),0,0,0,0,0),'Nonfinite IMU'),
        'imu_time_reversal':(MAGIC+good+imu_packet(999_000_000,0,1,[0]*6),'Invalid IMU'),
        'imu_sequence_gap':(MAGIC+good+imu_packet(1_005_000_000,0,2,[0]*6),'Invalid IMU'),
        'wrong_geometry':(MAGIC+good+camera_packet(1_002_000_000,0,0,np.zeros((4,4),np.uint8)),'geometry/calibration'),
        'missing_tail':(MAGIC+good+camera_packet(1_002_000_000,0,0,np.zeros((512,512),np.uint8))+end_packet(),'Missing IMU tail'),
        'clean_end_without_states':(MAGIC+good+end_packet(),'No initialized states'),
    }
    results=[]
    with tempfile.TemporaryDirectory(prefix='stream-faults-',dir=ROOT/'build') as temp:
        for name,(payload,message) in cases.items():
            run=subprocess.run([str(ROOT/'build/adapter/vio_stream'),str(ROOT/'config/reference/tumvi-mono/estimator_config.yaml'),
                str(Path(temp)/name),'reference'],input=payload,capture_output=True,timeout=15)
            log=run.stderr.decode(errors='replace');passed=run.returncode!=0 and message in log
            results.append({'case':name,'return_code':run.returncode,'expected_diagnostic':message,'passed':passed,'stderr':log})
    report={'scope':'Actual C++ transport rejection tests; reference configuration, no physical hardware',
        'passed':all(r['passed'] for r in results),'cases':results}
    a.output.write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report,indent=2))
    return int(not report['passed'])


if __name__=='__main__':raise SystemExit(main())
