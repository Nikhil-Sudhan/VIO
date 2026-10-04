#!/usr/bin/python3
"""Native desktop controls for a bounded physical OpenVINS/RViz session."""
import argparse
import fcntl
import json
import os
import signal
import subprocess
import sys
import time
import xmlrpc.client
from datetime import datetime
from pathlib import Path
from PyQt5 import QtCore, QtGui, QtWidgets
from recording_storage import capture_folder

ROOT = Path(__file__).resolve().parents[1]


class Demo(QtWidgets.QWidget):
    def __init__(self, calibration=None, seconds=120, camera_config=None, archive_camera=False):
        super().__init__()
        self.seconds = seconds
        self.archive_camera = archive_camera
        self.calibration = calibration or ROOT/'calibration/new-case-20261004/supervised-natural-v3/estimator_config.yaml'
        self.camera_config = camera_config or ROOT/'config/camera-candidate.json'
        self.setWindowTitle('OpenVINS Live — camera + IMU')
        self.resize(420, 230)
        self.record = self.bridge = self.rviz = self.master = None
        self.video = None
        self.video_path = None
        self.folder = self.logpath = None
        self.start_time = 0
        self.raw_mode = False
        layout = QtWidgets.QVBoxLayout(self)
        from calibration_guard import read_yaml
        grid_features = read_yaml(self.calibration).get('use_aprilgrid_features', False)
        scene = ('Keep one AprilGrid sheet visible in a well-lit scene.' if grid_features
                 else 'Point at a well-lit scene with texture, such as books or shelves.')
        instructions = QtWidgets.QLabel(f'{scene}\nKeep still until the status says ready to move.\nMove sideways slowly until tracking starts, then explore slowly.\n\nExperimental local mapping · each run lasts {seconds} seconds.')
        layout.addWidget(instructions)
        self.status = QtWidgets.QLabel('Ready')
        self.status.setWordWrap(True)
        layout.addWidget(self.status)
        self.start_button = QtWidgets.QPushButton(f'Start live ({seconds} sec)')
        self.start_button.clicked.connect(self.start)
        layout.addWidget(self.start_button)
        self.raw_button = QtWidgets.QPushButton(f'Record sensors ({seconds} sec maximum)')
        self.raw_button.clicked.connect(lambda: self.start(raw=True))
        layout.addWidget(self.raw_button)
        from calibration_guard import checked_bundle
        try:
            checked_bundle(self.calibration, self.camera_config, experimental=True)
            self.tracking_available = True
        except (OSError, ValueError, KeyError):
            self.tracking_available = False
        self.start_button.setEnabled(self.tracking_available)
        self.stop_button = QtWidgets.QPushButton('Stop and save')
        self.stop_button.clicked.connect(self.stop)
        layout.addWidget(self.stop_button)
        self.preview = QtWidgets.QLabel()
        self.preview.hide()
        layout.addWidget(self.preview)
        self.timer = QtCore.QTimer(self)
        self.timer.timeout.connect(self.poll)
        self.timer.start(500)

    def view_last(self):
        saved = ROOT/'data/native-control/current-session.txt'
        if not saved.exists():
            return
        self.folder = Path(saved.read_text().strip())
        if not (capture_folder(self.folder)/'vio').exists():
            self.raw_mode = True
            self.status.setText('Sensor recording stopped. Use Record sensors for a new take; no live tracking.')
            return
        self.rviz = self.spawn(['bash', 'tools/run_rviz.sh'], 'last-session-rviz.log')
        self.bridge = self.spawn(['/usr/bin/python3', 'tools/rviz_bridge.py', str(self.folder)], 'bridge.log')
        self.status.setText('Capture is stopped. RViz shows the last saved image and estimator result.')
        review = self.folder/'vio/display-review.json'
        if review.exists() and json.loads(review.read_text()).get('usable_vio') is False:
            self.status.setText('Last run failed: '+json.loads(review.read_text())['reason'])

    def spawn(self, command, name):
        log = (ROOT/'data/native-control'/name).open('ab')
        proc = subprocess.Popen(command, cwd=ROOT, stdout=log, stderr=subprocess.STDOUT)
        log.close()
        return proc

    def start(self, raw=False):
        if self.record is not None and self.record.poll() is None:
            return
        if self.video is not None and self.video.poll() is None:
            self.status.setText('Finishing the previous video — please wait.')
            return
        if not raw:
            try:
                from calibration_guard import checked_bundle
                checked_bundle(self.calibration, self.camera_config, experimental=True)
            except (OSError, ValueError, KeyError) as error:
                self.status.setText('Cannot start: '+str(error))
                return
            try:
                xmlrpc.client.ServerProxy(os.environ['ROS_MASTER_URI']).getPid('/native_demo')
            except OSError:
                self.master = self.spawn(['roscore', '-p', '11319'], 'roscore.log')
        self.raw_mode = bool(raw)
        for proc in [self.bridge, self.rviz]:
            if proc is not None and proc.poll() is None:
                proc.terminate()
                try:
                    proc.wait(timeout=3)
                except subprocess.TimeoutExpired:
                    proc.kill()
        name = datetime.now().strftime('%Y%m%d_%H%M%S_%f')
        self.logpath = ROOT/'data/native-control'/f'{name}-capture.log'
        log = self.logpath.open('xb')
        self.record = subprocess.Popen([
            '/home/pluto/vio-env/bin/python', '-u', str(ROOT/'tools/record.py'),
            '--seconds', str(self.seconds), '--finish-on-usr1',
            *(['--ram-recording', '--temperature', '--image-format', 'pgm'] if raw else [
                '--experimental-calibration', '--image-format', 'pgm', '--ram-recording', '--temperature',
                '--estimator-config', str(self.calibration)]),
            '--config', str(self.camera_config),
            '--output', str(ROOT/'data/native-live')], cwd=ROOT, stdout=log, stderr=subprocess.STDOUT)
        log.close()
        self.folder = None
        self.bridge = None
        self.rviz = self.video = None
        if not raw:
            self.rviz = self.spawn(['bash', 'tools/run_rviz.sh'], f'{name}-rviz.log')
            # Keep live acquisition free of desktop-video encoding and SD writes.
        self.preview.setVisible(bool(raw))
        self.preview.clear()
        if raw:
            self.resize(640, 760)
            self.move(40, 60)
        self.start_time = time.monotonic()
        self.start_button.setEnabled(False)
        self.raw_button.setEnabled(False)
        self.status.setText('Starting sensors — keep the rig still.')

    def stop(self):
        if self.record is not None and self.record.poll() is None:
            if self.folder is not None:
                try:
                    storage = json.loads((self.folder/'recording-storage.json').read_text())
                    if storage['state'] != 'capturing':
                        self.status.setText('Saving sensor data — please wait.')
                        return
                except (OSError, ValueError, KeyError):
                    pass
            # The recorder installs this handler after startup.
            if self.folder is not None or time.monotonic()-self.start_time > 5:
                self.record.send_signal(signal.SIGUSR1)
            else:
                self.record.terminate()
            self.status.setText('Stopping and saving…')

    def poll(self):
        # Publish controller state even when acquisition fails before a session
        # directory exists, so the web controls can always offer another take.
        capturing = self.record is not None and self.record.poll() is None
        saving = not capturing and self.video is not None and self.video.poll() is None
        archive_saving = False
        if self.folder is not None:
            try:
                archive_saving = json.loads((self.folder/'recording-storage.json').read_text())['state'] == 'saving'
            except (OSError, ValueError, KeyError):
                pass
        snapshot = {'running': capturing or saving, 'message': self.status.text(),
                    'phase': 'saving' if archive_saving or saving else 'recording' if capturing else 'idle',
                    'raw_mode': self.raw_mode, 'tracking_available': self.tracking_available,
                    'boot_id': Path('/proc/sys/kernel/random/boot_id').read_text().strip(),
                    'emitted_boot_ns': time.clock_gettime_ns(time.CLOCK_BOOTTIME)}
        target = ROOT/'data/native-control/controller-state.json'
        temp = target.with_suffix('.tmp')
        temp.write_text(json.dumps(snapshot))
        temp.replace(target)
        request = ROOT/'data/native-control/request.json'
        if request.exists():
            try:
                command = json.loads(request.read_text()).get('action')
                request.unlink()
                if command == 'start':
                    self.start()
                elif command == 'start_raw':
                    self.start(raw=True)
                elif command == 'stop':
                    self.stop()
            except (OSError, ValueError):
                pass
        if self.record is None:
            return
        if self.folder is None and self.logpath.exists():
            with self.logpath.open() as log:
                first = log.readline().strip()
            candidate = Path(first)
            if candidate.is_absolute() and candidate.parent == ROOT/'data/native-live' and candidate.is_dir():
                self.folder = candidate
                (ROOT/'data/native-control/current-session.txt').write_text(str(candidate)+'\n')
                if not self.raw_mode:
                    self.bridge = self.spawn(['/usr/bin/python3', 'tools/rviz_bridge.py', str(candidate)], 'bridge.log')
        code = self.record.poll()
        if code is not None:
            if self.video is not None and self.video.poll() is None:
                self.video.terminate()
            self.start_button.setEnabled(self.tracking_available)
            self.raw_button.setEnabled(True)
            message = f'Run ended (exit {code}). Start live creates a new run.'
            if self.folder is None and code != 0 and self.logpath is not None:
                lines = self.logpath.read_text(errors='replace').strip().splitlines()
                message = 'Could not start: '+(lines[-1] if lines else f'exit {code}')
            if self.folder is not None:
                if self.raw_mode and code != 0:
                    message = 'Sensor capture or saving failed. Data retained for diagnosis; do not repeat yet.'
                try:
                    session = json.loads((self.folder/'session.json').read_text())
                    if session.get('errors'):
                        message = 'Run stopped with an error; measurements and estimator.log saved.'
                    elif self.raw_mode and code == 0 and session['status'] == 'completed':
                        message = f"Saved {session['frames_written']} camera frames and IMU data. Ready for analysis."
                    review_path = self.folder/'vio/display-review.json'
                    if review_path.exists() and json.loads(review_path.read_text()).get('usable_vio') is False:
                        message = 'Tracking failed: '+json.loads(review_path.read_text())['reason']
                except (OSError, ValueError):
                    pass
            self.status.setText(message)
            return
        remaining = max(0, self.seconds-int(time.monotonic()-self.start_time))
        if self.raw_mode:
            if self.folder is None:
                return
            try:
                storage = json.loads((self.folder/'recording-storage.json').read_text())
                if storage['state'] == 'saving':
                    self.status.setText('Saving and checking sensor data — please wait.')
                    return
                current = capture_folder(self.folder)
                session = json.loads((current/'session.json').read_text())
                elapsed = (time.clock_gettime_ns(time.CLOCK_BOOTTIME)-session.get('recording_started_boot_ns', time.clock_gettime_ns(time.CLOCK_BOOTTIME)))/1e9
                count = session['frames_written']
                if count:
                    pixmap = QtGui.QPixmap(str(current/'images'/f'{count-1:06d}.pgm'))
                    if not pixmap.isNull():
                        self.preview.setPixmap(pixmap.scaledToWidth(600, QtCore.Qt.SmoothTransformation))
                if session['status'] != 'recording':
                    message = 'Starting sensors — keep the case still.'
                elif elapsed < 5:
                    message = f'Keep still — {max(1, int(5-elapsed)+1)} s remaining.'
                else:
                    message = 'Recording sensors: move sideways slowly, then back. Press Stop and save when resting.'
                self.status.setText(f'{message}\n{remaining} seconds remaining. No live tracking.')
            except (OSError, ValueError, KeyError):
                pass
            return
        try:
            if self.folder is None:
                return
            if archive_saving:
                self.status.setText('Saving and checking sensor data and local maps — please wait.')
                return
            snap = json.loads((capture_folder(self.folder)/'vio/display.json').read_text())
            fresh = time.clock_gettime_ns(time.CLOCK_BOOTTIME)-snap['emitted_boot_ns'] < 2e9
            if not fresh:
                text = 'Output delayed — check the session log.'
            elif snap.get('image_mean_luma', 255) < 24 and snap.get('image_std_luma', 255) < 4:
                text = 'Scene too dark — turn on the room lights.'
            else:
                labels = {
                    'initializing': ('Tracking lost — keep still to initialize a separate local map.'
                                     if snap.get('reset_epoch', 0) else 'Initializing — keep the rig completely still.'),
                    'ready': 'Ready — move sideways slowly to establish depth.',
                    'initializing_depth': 'Establishing depth — position is not available yet.',
                    'tracking': 'Tracking natural features in the current local map.',
                    'stationary': 'Stationary — local map retained.',
                    'lost': 'Tracking lost — hold still to start a separate local map.',
                }
                text = labels.get(snap.get('tracking_state'), 'Output unverified — tracking health evidence missing.')
            self.status.setText(f'{text}\n{remaining} seconds remaining.')
        except (OSError, ValueError, TypeError):
            pass

    def closeEvent(self, event):
        if self.record is not None and self.record.poll() is None:
            self.stop()
            self.status.setText('Stopping and saving. Close this window after saving finishes.')
            event.ignore()
            return
        self.stop()
        if self.record is not None and self.record.poll() is None:
            try:
                self.record.wait(timeout=15)
            except subprocess.TimeoutExpired:
                self.record.terminate()
        for proc in [self.bridge, self.rviz, self.master, self.video]:
            if proc is not None and proc.poll() is None:
                proc.terminate()
        event.accept()


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--calibration', type=Path, help='Reviewed development configuration for the current mounting')
    parser.add_argument('--camera-config', type=Path, help='Camera acquisition configuration matching the reviewed bundle')
    parser.add_argument('--start', action='store_true')
    parser.add_argument('--view-last', action='store_true')
    parser.add_argument('--archive-camera', action='store_true', help='Save every raw image for diagnostic replay; about 1.2 GB per 120-second run')
    parser.add_argument('--seconds', type=int, default=120, help='Bounded recording duration, 15 to 300 seconds (default: 120)')
    args = parser.parse_args()
    if not 15 <= args.seconds <= 300:
        parser.error('--seconds must be between 15 and 300')
    (ROOT/'data/native-control').mkdir(parents=True, exist_ok=True)
    app = QtWidgets.QApplication(sys.argv)
    lock = (ROOT/'data/native-control/controller.lock').open('w')
    try:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError:
        QtWidgets.QMessageBox.information(None, 'OpenVINS Live', 'The live controller is already open. Select it in the taskbar.')
        sys.exit(0)
    demo = Demo(args.calibration, args.seconds, args.camera_config, args.archive_camera)
    demo.show()
    if args.start:
        QtCore.QTimer.singleShot(500, demo.start)
    elif args.view_last:
        QtCore.QTimer.singleShot(500, demo.view_last)
    sys.exit(app.exec_())
