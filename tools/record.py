#!/usr/bin/env python3
"""Buffered camera + hardware FIFO acquisition; calibration is still required."""
import argparse
import csv
import hashlib
import json
import queue
import shutil
import signal
import tempfile
import threading
import time
import traceback
from datetime import datetime, timezone
from pathlib import Path

from picamera2 import Picamera2
from libcamera import controls
from imu_fifo_probe import FifoDevice, boot_ns, ROOT


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--seconds', type=float, default=30)
    p.add_argument('--config', type=Path, default=ROOT / 'config/camera-candidate.json')
    p.add_argument('--output', type=Path, default=ROOT / 'data/recordings')
    p.add_argument('--imu-only', action='store_true', help='Record buffered raw IMU for noise characterization without opening the camera')
    p.add_argument('--temperature', action='store_true', help='Also save 1 Hz IMU die temperature and its host read brackets separately')
    p.add_argument('--image-format', choices=['pgm','png'], default='pgm', help='Lossless stored image format; PNG reduces storage while preserving every pixel')
    p.add_argument('--estimator-config', type=Path, help='Experimental live OpenVINS; requires a reviewed joint-calibration bundle')
    p.add_argument('--ram-estimator-output', action='store_true', help='Keep derived estimator output in RAM during the live run, then copy it to the session; raw sensor data stays on disk')
    p.add_argument('--experimental-calibration', action='store_true', help='Explicit bounded manual demo using reviewed provisional calibration; NOT navigation validation')
    p.add_argument('--finish-on-usr1', action='store_true', help='SIGUSR1 ends a guided capture normally, with IMU tail')
    p.add_argument('--save-camera-every', type=int, default=1, help='Demo archive stride only; every camera frame still feeds live VIO')
    p.add_argument('--no-camera-archive', action='store_true', help='Live demo only: feed every camera frame to VIO without saving raw camera images')
    args = p.parse_args()
    if args.no_camera_archive and (not args.experimental_calibration or not args.estimator_config or args.imu_only):
        p.error('no-camera-archive requires an experimental camera-and-IMU live estimator run')
    if args.save_camera_every < 1 or (args.save_camera_every != 1 and not args.experimental_calibration):
        p.error('Camera archive stride must be positive; sampling is only allowed for experimental demos')
    # Import the encoder before hardware threads start: its shared-library
    # loading can hold the GIL long enough to starve FIFO/clock acquisition.
    if args.image_format == 'png':
        import cv2
    if not 1 <= args.seconds <= 7200:
        p.error('seconds must be 1..7200')
    if args.experimental_calibration and (not args.estimator_config or args.seconds > 300):
        p.error('Experimental calibration requires estimator-config and a bounded run of at most 300 seconds')
    if args.ram_estimator_output and not args.estimator_config:
        p.error('ram-estimator-output requires estimator-config')
    cfg = json.loads(args.config.read_text())
    calibration_review = None
    if args.estimator_config:
        if args.imu_only:
            p.error('The estimator requires camera and IMU capture')
        from calibration_guard import checked_bundle
        calibration_review = checked_bundle(args.estimator_config, args.config,experimental=args.experimental_calibration)
    width, height = cfg['main']['size']
    args.output.mkdir(parents=True, exist_ok=True)
    image_bytes = 0 if args.imu_only or args.no_camera_archive else width * height * (cfg['controls']['FrameRate'] * args.seconds / args.save_camera_every + 1)
    required = int(image_bytes * 1.2 + 100_000 * args.seconds + 200_000_000)
    if shutil.disk_usage(args.output).free < required:
        raise RuntimeError(f'Insufficient storage: require {required} bytes')
    folder = Path(tempfile.mkdtemp(prefix=datetime.now().strftime('%Y%m%d_%H%M%S_'), dir=args.output))
    if not args.imu_only:
        (folder / 'images').mkdir()
    stop, imu_ready, capture_enabled = threading.Event(), threading.Event(), threading.Event()
    finish_requested = threading.Event()
    images = queue.Queue(maxsize=32)
    raw_records = queue.Queue(maxsize=1024)
    errors, workers = [], []
    status_stop = threading.Event()
    status_worker = None
    cam = None
    stream = None
    report = {'schema': 2, 'created_utc': datetime.now(timezone.utc).isoformat(), 'status': 'starting',
              'purpose': 'live demo without camera archive, NOT calibration input' if args.no_camera_archive else 'raw calibration/acquisition input, NOT calibrated VIO', 'calibrated': False,
              'requested_seconds': args.seconds, 'camera_requested': None if args.imu_only else cfg,
              'capture_mode': 'imu-only' if args.imu_only else 'camera-and-imu',
              'camera_archive_stride': None if args.no_camera_archive else args.save_camera_every,
              'camera_archive_enabled': not args.no_camera_archive,
              'complete_camera_archive': not args.no_camera_archive and args.save_camera_every == 1,
              'temperature_recorded': args.temperature,
              'image_format': args.image_format,
              'experimental_calibration': args.experimental_calibration,
              'pid': __import__('os').getpid(), 'finish_on_usr1': args.finish_on_usr1,
              'boot_id': Path('/proc/sys/kernel/random/boot_id').read_text().strip(),
              'camera_config_sha256': None if args.imu_only else hashlib.sha256(args.config.read_bytes()).hexdigest(),
              'acquisition_source_sha256': {name: hashlib.sha256((ROOT / 'tools' / name).read_bytes()).hexdigest()
                                           for name in ('record.py', 'imu_fifo_probe.py', 'live_bridge.py', 'online_imu.py', 'stream_protocol.py', 'calibration_guard.py')},
              'camera_timestamp': 'unmodified SensorTimestamp; not an exposure midpoint',
              'imu_timestamp': 'FIFO hardware ticks and host clock read brackets; mapping is derived separately',
              'clock_start': {'boottime_ns': boot_ns(), 'monotonic_ns': time.monotonic_ns()},
              'frames_enqueued': 0, 'frames_written': 0, 'image_queue_high_water': 0,
              'raw_queue_high_water': 0, 'raw_records_written': 0,
              'fifo_words': 0, 'fifo_max_level': 0, 'fifo_overflow_or_full_batches': 0}
    # Preserve the actual implementation/configuration used by this run, even
    # if the worktree changes later. No existing recording is modified.
    source_folder = folder / 'acquisition_source'
    source_folder.mkdir()
    for name in report['acquisition_source_sha256']:
        shutil.copyfile(ROOT / 'tools' / name, source_folder / name)
    shutil.copyfile(args.config, folder / 'camera-config.json')
    if args.estimator_config:
        shutil.copytree(args.estimator_config.resolve().parent, folder / 'calibration')
        binary = ROOT / 'build/adapter/vio_stream'
        report['estimator_binary_sha256'] = hashlib.sha256(binary.read_bytes()).hexdigest()
        shutil.copyfile(ROOT / 'adapter/stream.cpp', source_folder / 'stream.cpp')

    def save_session():
        report['errors'] = errors.copy()
        temp = folder / f'session.json.{threading.get_ident()}.tmp'
        temp.write_text(json.dumps(report, indent=2, default=str) + '\n')
        temp.replace(folder / 'session.json')

    def status_writer():
        try:
            while not status_stop.wait(5):
                if stream:
                    report['live_transport'] = stream.report.copy()
                print(f"{report['frames_enqueued']} images, {report['fifo_words']} FIFO words, queue {images.qsize()}", flush=True)
                save_session()
        except Exception:
            errors.append(traceback.format_exc())
            stop.set()

    def imu_loop():
        dev = None
        try:
            dev = FifoDevice(folder)
            report['imu_configured_registers'] = dev.start()
            if calibration_review and {str(k): v for k, v in report['imu_configured_registers'].items()} != calibration_review['imu_configured_registers']:
                raise RuntimeError('IMU register configuration differs from reviewed calibration')
            report['imu_internal_freq_fine_raw'] = dev.read(0x63, 1)[0]
            next_anchor = 0
            next_temperature = 0
            imu_ready.set()
            while not stop.is_set():
                if time.monotonic() >= next_anchor:
                    anchor = dev.anchor()
                    raw_records.put_nowait(anchor)
                    if stream:
                        stream.submit(anchor)
                    # More host/sensor clock observations provide redundancy
                    # when CPU scheduling widens a read bracket beyond 2 ms.
                    # The clock fitter still rejects those wide reads and keeps
                    # its original minimum-data, prediction and stale checks.
                    next_anchor = time.monotonic() + 0.05
                batch = dev.batch()
                raw_records.put_nowait(batch)
                if stream:
                    stream.submit(batch)
                if args.temperature and time.monotonic() >= next_temperature:
                    before = boot_ns()
                    payload = dev.read(0x20, 2)
                    raw_records.put_nowait({'kind': 'temperature', 'host_before_boot_ns': before,
                        'host_after_boot_ns': boot_ns(), 'raw_hex': payload.hex(),
                        'temperature_c': 25.0 + int.from_bytes(payload, 'little', signed=True) / 256.0,
                        'timestamp_scope': 'host receipt bracket, not sensor measurement time'})
                    next_temperature = time.monotonic() + 1.0
                report['raw_queue_high_water'] = max(report['raw_queue_high_water'], raw_records.qsize())
                report['fifo_words'] += batch['read_words']
                report['fifo_max_level'] = max(report['fifo_max_level'], batch['level_words'])
                if batch['overflow_or_full']:
                    report['fifo_overflow_or_full_batches'] += 1
                    raise RuntimeError('IMU FIFO overflow/full: raw status preserved; acquisition stopped')
                stop.wait(0.01 if batch['level_words'] < 128 else 0)
        except Exception:
            errors.append(traceback.format_exc())
            stop.set()
        finally:
            if dev:
                try:
                    dev.close()
                except Exception:
                    errors.append(traceback.format_exc())
                    stop.set()
            imu_ready.set()

    def raw_writer():
        # SD-card writeback can block for seconds. Never perform filesystem writes
        # in the thread draining the ~0.8-second hardware FIFO.
        try:
            with (folder / 'raw.jsonl').open('x', buffering=65536) as output, \
                 (folder / 'temperature.jsonl').open('x', buffering=65536) as temp_output:
                while True:
                    record = raw_records.get()
                    if record is None:
                        break
                    destination = temp_output if record['kind'] == 'temperature' else output
                    destination.write(json.dumps(record) + '\n')
                    report['raw_records_written'] += 1
        except Exception:
            errors.append(traceback.format_exc())
            stop.set()

    def image_writer():
        if args.imu_only:
            return
        try:
            with (folder / 'frames.csv').open('w', newline='') as f, (folder / 'frame_metadata.jsonl').open('w') as mf:
                writer = csv.writer(f)
                writer.writerow(['sequence', 'request_sequence', 'sensor_timestamp_ns', 'host_received_boot_ns',
                                 'host_received_monotonic_ns', 'host_written_boot_ns', 'exposure_us', 'filename'])
                while True:
                    item = images.get()
                    if item is None:
                        break
                    i, seq, received, mono, pixels, metadata = item
                    filename = f'images/{i:06d}.{args.image_format}'
                    with (folder / filename).open('xb') as out:
                        if args.image_format == 'png':
                            ok, encoded = cv2.imencode('.png', pixels, [cv2.IMWRITE_PNG_COMPRESSION, 1])
                            if not ok:
                                raise RuntimeError('Lossless image encoding failed')
                            out.write(encoded.tobytes())
                        else:
                            out.write(f'P5\n{width} {height}\n255\n'.encode())
                            out.write(pixels.tobytes())
                    writer.writerow([i, seq, metadata['SensorTimestamp'], received, mono,
                                     boot_ns(), metadata.get('ExposureTime'), filename])
                    mf.write(json.dumps({'sequence': i, 'metadata': metadata}, default=str) + '\n')
                    report['frames_written'] += 1
        except Exception:
            errors.append(traceback.format_exc())
            stop.set()

    def camera_callback(request):
        # Picamera2 invokes this for every successful request, including batches
        # completed while the Python application was briefly descheduled.
        # Repeated capture_request jobs with queue=False can skip those frames.
        if not capture_enabled.is_set() or stop.is_set():
            return
        try:
            received, mono = boot_ns(), time.monotonic_ns()
            metadata = request.get_metadata()
            seq = request.request.sequence
            pixels = request.make_array('main')[:height, :width].copy()
            if pixels.shape != (height, width) or pixels.dtype.name != 'uint8':
                raise RuntimeError(f'Incorrect image layout: {pixels.shape}')
            if 'SensorTimestamp' not in metadata or list(metadata.get('ScalerCrop', [])) != cfg['expected_scaler_crop']:
                raise RuntimeError('Missing timestamp or unexpected camera crop')
            if not args.no_camera_archive and report['frames_enqueued'] % args.save_camera_every == 0:
                images.put_nowait((report['frames_enqueued'], seq, received, mono, pixels, metadata))
            if stream:
                stream.submit({'kind': 'camera', 'timestamp_ns': metadata['SensorTimestamp'],
                               'receipt_ns': received, 'sequence': seq, 'pixels': pixels})
            report['frames_enqueued'] += 1
            report['image_queue_high_water'] = max(report['image_queue_high_water'], images.qsize())
        except Exception:
            errors.append(traceback.format_exc())
            stop.set()

    interrupted = [False]
    def interrupt(_sig, _frame):
        interrupted[0] = True
        stop.set()
    signal.signal(signal.SIGINT, interrupt)
    signal.signal(signal.SIGTERM, interrupt)
    if args.finish_on_usr1:
        signal.signal(signal.SIGUSR1, lambda _sig, _frame: finish_requested.set())
    print(folder, flush=True)
    save_session()
    try:
        if args.estimator_config:
            clocks = report['clock_start']
            if abs(clocks['boottime_ns'] - clocks['monotonic_ns']) > 1_000_000:
                raise RuntimeError('Camera/IMU host clock relation is not established after suspend')
            from live_bridge import LiveBridge
            stream = LiveBridge(args.estimator_config, folder, stop, errors,ram_output=args.ram_estimator_output)
            report['estimator_config'] = str(args.estimator_config.resolve())
            report['estimator_config_sha256'] = hashlib.sha256(args.estimator_config.read_bytes()).hexdigest()
            report['calibration_review'] = calibration_review
        for target, name in ((imu_loop, 'imu-fifo'), (image_writer, 'image-writer'), (raw_writer, 'raw-writer')):
            worker = threading.Thread(target=target, name=name, daemon=True)
            worker.start()
            workers.append(worker)
        if not imu_ready.wait(5) or errors:
            raise RuntimeError('IMU failed to start')
        if not args.imu_only:
            cam = Picamera2()
            capture_controls = dict(cfg['controls'])
            capture_controls['NoiseReductionMode'] = controls.draft.NoiseReductionModeEnum.Off
            camera_config = cam.create_video_configuration(main=cfg['main'], sensor=cfg['sensor'],
                                                           controls=capture_controls, buffer_count=cfg['buffer_count'], queue=False)
            cam.configure(camera_config)
            report['camera_configuration'] = cam.camera_configuration()
            report['camera_properties'] = cam.camera_properties
            report['camera_delivery'] = 'Picamera2 post_callback per successful request'
            cam.post_callback = camera_callback
            cam.start()
            stop.wait(2)
        report['status'] = 'recording'
        save_session()
        status_worker = threading.Thread(target=status_writer, name='status-writer', daemon=True)
        status_worker.start()
        capture_enabled.set()
        deadline = time.monotonic() + args.seconds
        progress_key = 'fifo_words' if args.imu_only else 'frames_enqueued'
        while time.monotonic() < deadline and not stop.is_set() and not finish_requested.is_set():
            count = report[progress_key]
            stop.wait(min(1, max(0, deadline-time.monotonic())))
            if time.monotonic() < deadline and count == report[progress_key] and not stop.is_set():
                raise RuntimeError(f'No {progress_key} progress for one second')
        capture_enabled.clear()
        stop.wait(1.0 if stream else 0.1)
        report['completion_reason'] = ('signal_interruption' if interrupted[0] else
                                       'error' if errors or stop.is_set() else
                                       'guided_finish_signal' if finish_requested.is_set() else 'duration')
        report['status'] = 'interrupted' if interrupted[0] else 'completed'
    except Exception:
        errors.append(traceback.format_exc())
        report['status'] = 'failed'
    finally:
        capture_enabled.clear()
        stop.set()
        status_stop.set()
        if status_worker:
            status_worker.join(timeout=10)
            if status_worker.is_alive():
                errors.append('Status writer still running after 10 seconds')
        if cam:
            try:
                cam.stop()
                cam.close()
            except Exception:
                errors.append(traceback.format_exc())
        # Join the producer before queueing writer sentinels, preserving its tail.
        if workers:
            workers[0].join(timeout=5)
        for index, q in ((1, images), (2, raw_records)):
            if len(workers) > index and workers[index].is_alive():
                try:
                    q.put(None, timeout=5)
                except queue.Full:
                    errors.append(f'{workers[index].name} did not drain within 5 seconds')
        for worker in workers:
            worker.join(timeout=10)
            if worker.is_alive():
                errors.append(f'Worker still running: {worker.name}')
        if stream:
            try:
                report['live_transport'] = stream.close()
            except Exception:
                errors.append(traceback.format_exc())
        if errors:
            report['status'] = 'failed'
        report['clock_end'] = {'boottime_ns': boot_ns(), 'monotonic_ns': time.monotonic_ns()}
        save_session()
    print(json.dumps(report, indent=2, default=str), flush=True)
    return int(report['status'] != 'completed')


if __name__ == '__main__':
    raise SystemExit(main())
