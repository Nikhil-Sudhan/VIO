#!/usr/bin/env python3
"""Bounded LSM6DSOX electrical self-test; saves and restores sensor registers.

Follows ST's lsm6dsox_self_test.c: XL 52 Hz / 4g, negative stimulus;
gyro 208 Hz / 2000 dps, positive stimulus. Not a VIO calibration.
Requires a stationary rig and exclusive IMU access (enforced by FifoDevice).
"""
import argparse
import json
import struct
import time
from pathlib import Path
from datetime import datetime, timezone
from imu_fifo_probe import FifoDevice


def average(dev, register, ready_bit, scale):
    samples = []
    for _ in range(21):
        deadline = time.monotonic() + 1
        while not (dev.read(0x1e, 1)[0] & ready_bit):
            if time.monotonic() > deadline:
                raise TimeoutError('No new sensor sample during self-test')
            time.sleep(.001)
        samples.append([v * scale for v in struct.unpack('<hhh', dev.read(register, 6))])
    samples = samples[1:]  # discard first sample following mode change
    return {'mean': [sum(r[i] for r in samples)/len(samples) for i in range(3)],
            'samples': samples}


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('output', type=Path)
    args = p.parse_args()
    args.output.mkdir()
    report = {'created_utc': datetime.now(timezone.utc).isoformat(),
              'source': 'https://github.com/STMicroelectronics/STMems_Standard_C_drivers/blob/master/lsm6dsox_STdC/examples/lsm6dsox_self_test.c',
              'scope': 'Electrical self-test only; not calibrated accuracy or camera/IMU rigidity',
              'status': 'started'}
    dev = None
    try:
        dev = FifoDevice(args.output)
        dev.start()
        dev.write(0x0a, 0)
        dev.write(0x11, 0)
        dev.write(0x10, 0x38)
        dev.write(0x14, 0)
        time.sleep(.2)
        xl_off = average(dev, 0x28, 1, .122)
        dev.write(0x14, 2)
        time.sleep(.2)
        xl_on = average(dev, 0x28, 1, .122)
        dev.write(0x14, 0)
        dev.write(0x10, 0)
        dev.write(0x11, 0x5c)
        time.sleep(.2)
        gy_off = average(dev, 0x22, 2, .070)
        dev.write(0x14, 4)
        time.sleep(.2)
        gy_on = average(dev, 0x22, 2, .070)
        for name, off, on, limits, unit in [
                ('accelerometer', xl_off, xl_on, [50,1700], 'mg'),
                ('gyroscope', gy_off, gy_on, [150,700], 'deg/s')]:
            delta = [abs(b-a) for a,b in zip(off['mean'],on['mean'])]
            report[name] = {'off': off, 'on': on, 'absolute_change': delta,
                            'limits': limits, 'units': unit,
                            'axis_pass': [limits[0] <= v <= limits[1] for v in delta]}
        report['passed'] = all(all(report[n]['axis_pass']) for n in ['accelerometer','gyroscope'])
        report['status'] = 'completed'
    except Exception as error:
        report['status'] = 'failed'
        report['error'] = repr(error)
        raise
    finally:
        if dev is not None:
            saved = dict(dev.saved)
            try:
                dev.write(0x14, 0)
            finally:
                dev.close()
            from Adafruit_PureIO.smbus import SMBus
            bus = SMBus(1)
            try:
                actual = {r: bytes(bus.read_i2c_block_data(0x6a,r,1))[0] for r in saved}
                report['registers_restored'] = actual == saved
            finally:
                bus.close()
        (args.output/'report.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps({k:v for k,v in report.items() if k not in ['accelerometer','gyroscope']},indent=2))
    for name in ['accelerometer','gyroscope']:
        print(name,report[name]['absolute_change'],report[name]['units'],report[name]['axis_pass'])


if __name__ == '__main__':
    main()
