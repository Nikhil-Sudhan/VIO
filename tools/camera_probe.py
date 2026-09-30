#!/usr/bin/env python3
"""Short geometry/exposure diagnostic. Originals are never overwritten."""
import argparse
import json
import tempfile
import time
from datetime import datetime, timezone
from pathlib import Path

import numpy as np
from PIL import Image
from picamera2 import Picamera2


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--mode', choices=('original', 'wide'), default='wide')
    p.add_argument('--exposure-us', type=int)
    p.add_argument('--gain', type=float, default=1.0)
    p.add_argument('--output', type=Path, default=Path(__file__).resolve().parents[1] / 'evidence')
    args = p.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    folder = Path(tempfile.mkdtemp(prefix=f'camera-{args.mode}-', dir=args.output))
    width, height = (640, 480) if args.mode == 'original' else (820, 616)
    controls = {'FrameRate': 20}
    if args.exposure_us is not None:
        controls.update(AeEnable=False, ExposureTime=args.exposure_us, AnalogueGain=args.gain)
    cam = Picamera2()
    try:
        # Picamera2 mode enumeration temporarily reconfigures the camera.
        # Complete it BEFORE applying the configuration under test.
        sensor_modes = cam.sensor_modes
        mode = {} if args.mode == 'original' else {'sensor': {'output_size': (1640, 1232), 'bit_depth': 10}}
        config = cam.create_video_configuration(main={'size': (width, height), 'format': 'YUV420'},
                                                controls=controls, buffer_count=6, queue=False, **mode)
        cam.configure(config)
        report = {'created_utc': datetime.now(timezone.utc).isoformat(),
                  'configuration': cam.camera_configuration(), 'properties': cam.camera_properties,
                  'controls': cam.camera_controls, 'sensor_modes': sensor_modes, 'frames': []}
        cam.start()
        time.sleep(2)
        for i in range(5):
            request = cam.capture_request()
            received = time.clock_gettime_ns(time.CLOCK_BOOTTIME)
            try:
                metadata = request.get_metadata()
                pixels = request.make_array('main')[:height, :width].copy()
                camera_sequence = request.request.sequence
            finally:
                request.release()
            if pixels.shape != (height, width) or pixels.dtype != np.uint8:
                raise RuntimeError(f'Unexpected luma layout: {pixels.shape} {pixels.dtype}')
            Image.fromarray(pixels).save(folder / f'{i:03d}.png')
            report['frames'].append({'host_received_boot_ns': received, 'camera_sequence': camera_sequence,
                                     'metadata': metadata, 'mean': float(pixels.mean()),
                                     'std': float(pixels.std()), 'percentiles': np.percentile(pixels, [1, 50, 99]).tolist()})
        (folder / 'report.json').write_text(json.dumps(report, indent=2, default=str) + '\n')
        print(folder)
        print(json.dumps(report['frames'][0], default=str))
    finally:
        cam.stop()
        cam.close()


if __name__ == '__main__':
    main()
