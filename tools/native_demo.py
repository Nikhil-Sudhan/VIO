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
from PyQt5 import QtCore, QtWidgets

ROOT = Path(__file__).resolve().parents[1]


class Demo(QtWidgets.QWidget):
    def __init__(self, calibration=None, seconds=60, camera_config=None):
        super().__init__()
        self.seconds = seconds
        self.calibration = calibration or ROOT/'calibration/demo-recovery-20260930/manual-quiet-veto/estimator_config.yaml'
        self.camera_config = camera_config or ROOT/'config/camera-room-demo.json'
        self.setWindowTitle('OpenVINS Live — camera + IMU')
        self.resize(420, 230)
        self.record = self.bridge = self.rviz = self.master = None
        self.video = None
        self.video_path = None
        self.folder = self.logpath = None
        self.start_time = 0
        layout = QtWidgets.QVBoxLayout(self)
        instructions = QtWidgets.QLabel(f'Keep one AprilGrid sheet visible in a well-lit scene.\nKeep the whole rig still until initialized.\nThen move it slowly, keeping both sensors fixed.\n\nProvisional calibration · each run lasts {seconds} seconds.')
        layout.addWidget(instructions)
        self.status = QtWidgets.QLabel('Ready')
        self.status.setWordWrap(True)
        layout.addWidget(self.status)
        self.start_button = QtWidgets.QPushButton(f'Start live ({seconds} sec)')
        self.start_button.clicked.connect(self.start)
        layout.addWidget(self.start_button)
        self.stop_button = QtWidgets.QPushButton('Stop and save')
        self.stop_button.clicked.connect(self.stop)
        layout.addWidget(self.stop_button)
        self.timer = QtCore.QTimer(self)
        self.timer.timeout.connect(self.poll)
        self.timer.start(500)

    def view_last(self):
        saved = ROOT/'data/native-control/current-session.txt'
        if not saved.exists():
            return
        self.folder = Path(saved.read_text().strip())
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

    def start(self):
        if self.record is not None and self.record.poll() is None:
            return
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
        for proc in [self.bridge, self.rviz]:
            if proc is not None and proc.poll() is None:
                proc.terminate()
                try:
                    proc.wait(timeout=3)
                except subprocess.TimeoutExpired:
                    proc.kill()
        name = datetime.now().strftime('%Y%m%d_%H%M%S')
        self.logpath = ROOT/'data/native-control'/f'{name}-capture.log'
        log = self.logpath.open('xb')
        self.record = subprocess.Popen([
            '/home/pluto/vio-env/bin/python', '-u', str(ROOT/'tools/record.py'),
            '--seconds', str(self.seconds), '--finish-on-usr1', '--experimental-calibration', '--image-format', 'png',
            '--ram-estimator-output',
            '--config', str(self.camera_config),
            '--estimator-config', str(self.calibration),
            '--output', str(ROOT/'data/native-live')], cwd=ROOT, stdout=log, stderr=subprocess.STDOUT)
        log.close()
        self.folder = None
        self.bridge = None
        self.rviz = self.spawn(['bash', 'tools/run_rviz.sh'], f'{name}-rviz.log')
        self.video_path = Path.home()/'Videos'/f'VIO-demo-{name}.mp4'
        self.video = self.spawn(['/usr/bin/python3', 'tools/record_desktop.py', str(self.video_path),
                                '--seconds', str(min(300, self.seconds+3))], f'{name}-video.log')
        self.start_time = time.monotonic()
        self.start_button.setEnabled(False)
        self.status.setText('Starting sensors — keep the rig still.')

    def stop(self):
        if self.record is not None and self.record.poll() is None:
            # The recorder installs this handler after startup.
            if time.monotonic()-self.start_time > 5:
                self.record.send_signal(signal.SIGUSR1)
            else:
                self.record.terminate()
            self.status.setText('Stopping and saving…')

    def poll(self):
        request = ROOT/'data/native-control/request.json'
        if request.exists():
            try:
                command = json.loads(request.read_text()).get('action')
                request.unlink()
                if command == 'start':
                    self.start()
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
                self.bridge = self.spawn(['/usr/bin/python3', 'tools/rviz_bridge.py', str(candidate)], 'bridge.log')
        code = self.record.poll()
        if code is not None:
            if self.video is not None and self.video.poll() is None:
                self.video.terminate()
            self.start_button.setEnabled(True)
            message = f'Run ended (exit {code}). Start live creates a new run.'
            if self.folder is not None:
                try:
                    session = json.loads((self.folder/'session.json').read_text())
                    if session.get('errors'):
                        message = 'Run stopped with an error; measurements and estimator.log saved.'
                    review_path = self.folder/'vio/display-review.json'
                    if review_path.exists() and json.loads(review_path.read_text()).get('usable_vio') is False:
                        message = 'Tracking failed: '+json.loads(review_path.read_text())['reason']
                except (OSError, ValueError):
                    pass
            self.status.setText(message)
            return
        remaining = max(0, self.seconds-int(time.monotonic()-self.start_time))
        try:
            snap = json.loads((self.folder/'vio/display.json').read_text())
            fresh = time.clock_gettime_ns(time.CLOCK_BOOTTIME)-snap['emitted_boot_ns'] < 2e9
            if not fresh:
                text = 'Output delayed — check the session log.'
            elif snap.get('image_mean_luma', 255) < 24 and snap.get('image_std_luma', 255) < 4:
                text = 'Scene too dark — turn on the room lights.'
            elif snap.get('zero_velocity_update'):
                text = 'Stationary update active. Moving accuracy remains unverified.'
            elif snap['initialized']:
                count = len(snap.get('msckf', []))+len(snap.get('slam', []))
                age = snap.get('last_msckf_update_age_s')
                text = ('No recent visual corrections — tracking unreliable.' if not snap.get('slam') and (age is None or age > 1)
                        else f'Live estimator. Current 3D features: {count}.')
            elif snap.get('stationary_ready'):
                text = 'Stationary initialization ready — move the whole rig slowly.'
            else:
                text = 'Initializing — keep the rig completely still.'
            self.status.setText(f'{text}\n{remaining} seconds remaining.')
        except (OSError, ValueError, TypeError):
            pass

    def closeEvent(self, event):
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
    parser.add_argument('--seconds', type=int, default=60, help='Bounded recording duration, 15 to 300 seconds (default: 60)')
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
    demo = Demo(args.calibration, args.seconds, args.camera_config)
    demo.show()
    if args.start:
        QtCore.QTimer.singleShot(500, demo.start)
    elif args.view_last:
        QtCore.QTimer.singleShot(500, demo.view_last)
    sys.exit(app.exec_())
