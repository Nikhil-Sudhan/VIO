#!/usr/bin/env python3
"""Capture original LSM6DSOX FIFO bytes, statuses and clock read brackets.

No synthesized sample timestamps. This diagnostic does not assert calibration
or camera synchronization. Run only one IMU-access process at a time.
"""
import argparse
import fcntl
import json
import tempfile
import time
import traceback
from datetime import datetime, timezone
from pathlib import Path

from Adafruit_PureIO.smbus import SMBus

ROOT = Path(__file__).resolve().parents[1]


def boot_ns():
    return time.clock_gettime_ns(time.CLOCK_BOOTTIME)


class FifoDevice:
    # All changed persistent registers are saved before modification.
    registers = tuple(range(0x07, 0x0f)) + tuple(range(0x10, 0x1a))

    def __init__(self, folder, address=0x6a):
        self.address = address
        self.lock = (ROOT / 'data/imu.lock').open('a')
        fcntl.flock(self.lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        self.bus = SMBus(1)
        self.saved = {}
        if self.read(0x0f, 1) != b'\x6c' or self.read(0x01, 1) != b'\x00':
            self.bus.close()
            raise RuntimeError('Expected LSM6DSOX WHO_AM_I=0x6c and main register bank')
        self.saved = {r: self.read(r, 1)[0] for r in self.registers}
        (folder / 'imu-registers-before.json').write_text(json.dumps(self.saved, indent=2) + '\n')

    def read(self, register, count):
        return bytes(self.bus.read_i2c_block_data(self.address, register, count))

    def write(self, register, value):
        self.bus.write_bytes(self.address, bytes((register, value)))

    def start(self):
        self.write(0x0a, 0)  # bypass/flush FIFO before configuring
        self.write(0x10, 0)  # stop XL and gyro during configuration
        self.write(0x11, 0)
        for register, value in {0x07: 0, 0x08: 0, 0x09: 0x55, 0x0b: 0,
                                0x0c: 0, 0x0d: 0, 0x0e: 0, 0x12: 0x44,
                                0x13: 0, 0x14: 0, 0x15: 0, 0x16: 0, 0x17: 0,
                                0x18: 0, 0x19: 0x20, 0x10: 0x58, 0x11: 0x54}.items():
            self.write(register, value)
        time.sleep(0.25)
        self.write(0x42, 0xaa)  # documented timestamp counter reset
        self.write(0x0a, 0x46)  # timestamp each BDR slot, continuous FIFO
        return {r: self.read(r, 1)[0] for r in self.registers}

    def anchor(self):
        before = boot_ns()
        payload = self.read(0x40, 4)
        after = boot_ns()
        return {'kind': 'clock_anchor', 'host_before_boot_ns': before, 'host_after_boot_ns': after,
                'sensor_ticks_u32': int.from_bytes(payload, 'little'), 'raw_hex': payload.hex()}

    def batch(self):
        before = boot_ns()
        status = self.read(0x3a, 2)
        status_after = boot_ns()
        level = status[0] | ((status[1] & 3) << 8)
        count = min(level, 128)
        data_before = boot_ns()
        payload = self.read(0x78, count * 7) if count else b''
        after = boot_ns()
        return {'kind': 'fifo', 'host_before_boot_ns': before, 'host_status_after_boot_ns': status_after,
                'host_data_before_boot_ns': data_before, 'host_after_boot_ns': after,
                'status_hex': status.hex(), 'level_words': level, 'read_words': count,
                'overflow_or_full': bool(status[1] & 0x68), 'raw_hex': payload.hex()}

    def close(self):
        try:
            if self.saved:
                self.write(0x0a, 0)
                self.write(0x10, 0)
                self.write(0x11, 0)
                for register, value in self.saved.items():
                    if register not in (0x0a, 0x10, 0x11):
                        self.write(register, value)
                for register in (0x10, 0x11, 0x0a):
                    self.write(register, self.saved[register])
        finally:
            self.bus.close()
            self.lock.close()


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--seconds', type=float, default=10)
    args = p.parse_args()
    if not 1 <= args.seconds <= 7200:
        p.error('seconds must be 1..7200')
    folder = Path(tempfile.mkdtemp(prefix='fifo-', dir=ROOT / 'evidence'))
    report = {'created_utc': datetime.now(timezone.utc).isoformat(), 'status': 'starting',
              'calibrated': False, 'hardware_synchronized': False, 'sensor_tick_nominal_ns': 25000,
              'odr_requested_hz': 208, 'ranges': {'accel_g': 4, 'gyro_dps': 500},
              'axes': 'native LSM6DSOX XYZ, no mounting rotation or gravity subtraction',
              'clock_start': {'boottime_ns': boot_ns(), 'monotonic_ns': time.monotonic_ns()},
              'overflow_or_full_batches': 0, 'words': 0, 'max_level_words': 0}
    device = None
    try:
        device = FifoDevice(folder)
        report['configured_registers'] = device.start()
        print(folder, flush=True)
        deadline, next_anchor = time.monotonic() + args.seconds, 0
        with (folder / 'raw.jsonl').open('w', buffering=65536) as output:
            while time.monotonic() < deadline:
                if time.monotonic() >= next_anchor:
                    output.write(json.dumps(device.anchor()) + '\n')
                    next_anchor = time.monotonic() + 0.1
                batch = device.batch()
                output.write(json.dumps(batch) + '\n')
                report['overflow_or_full_batches'] += batch['overflow_or_full']
                report['words'] += batch['read_words']
                report['max_level_words'] = max(report['max_level_words'], batch['level_words'])
                time.sleep(0.01 if batch['level_words'] < 128 else 0)
        report['status'] = 'completed'
    except KeyboardInterrupt:
        report['status'] = 'interrupted'
    except Exception:
        report['status'] = 'failed'
        report['error'] = traceback.format_exc()
    finally:
        if device:
            try:
                device.close()
            except Exception:
                report['status'] = 'failed'
                report['restore_error'] = traceback.format_exc()
        report['clock_end'] = {'boottime_ns': boot_ns(), 'monotonic_ns': time.monotonic_ns()}
        (folder / 'session.json').write_text(json.dumps(report, indent=2) + '\n')
    print(json.dumps(report, indent=2))
    return int(report['status'] != 'completed' or report['overflow_or_full_batches'] > 0)


if __name__ == '__main__':
    raise SystemExit(main())
