#!/usr/bin/env python3
"""Full-window live preview of the VIO camera geometry for physical positioning."""
import json
import argparse
import signal
import sys
import time
from pathlib import Path

from picamera2 import Picamera2
from PyQt5 import QtCore, QtGui, QtWidgets

ROOT = Path(__file__).resolve().parents[1]


class Preview(QtWidgets.QWidget):
    def __init__(self, recording=None):
        super().__init__()
        self.recording = recording
        self.last_saved = -1
        self.cam = None
        self.setWindowTitle('Live camera — position the assembly')
        self.setStyleSheet('background: #111; color: white; font-size: 18px;')
        layout = QtWidgets.QVBoxLayout(self)
        self.status = QtWidgets.QLabel('Starting live camera…')
        layout.addWidget(self.status)
        self.picture = QtWidgets.QLabel()
        self.picture.setAlignment(QtCore.Qt.AlignCenter)
        self.picture.setMinimumSize(410, 308)
        self.picture.setSizePolicy(QtWidgets.QSizePolicy.Ignored, QtWidgets.QSizePolicy.Ignored)
        layout.addWidget(self.picture, 1)
        layout.addWidget(QtWidgets.QLabel('Fit the complete AprilGrid sheet inside the picture. Keep both sensors fixed together.'))
        close = QtWidgets.QPushButton('Close preview')
        close.clicked.connect(self.close)
        layout.addWidget(close)
        cfg = json.loads((ROOT/'config/camera-room-demo.json').read_text())
        self.width, self.height = cfg['main']['size']
        if recording is None:
            self.cam = Picamera2()
            self.cam.configure(self.cam.create_video_configuration(
                main={'size': (self.width, self.height), 'format': 'YUV420'},
                sensor={'output_size': tuple(cfg['sensor']['output_size']), 'bit_depth': cfg['sensor']['bit_depth']},
                controls=cfg['controls'], buffer_count=8, queue=False))
            self.cam.start()
        else:
            self.setWindowTitle('Calibration recording — keep the grid visible')
        self.evidence = ROOT/'data/position-preview'
        self.evidence.mkdir(exist_ok=True)
        self.count = 0
        self.timer = QtCore.QTimer(self)
        self.timer.timeout.connect(self.update_frame)
        self.timer.start(50)

    def update_frame(self):
        if self.recording is not None:
            self.update_saved_frame()
            return
        request = self.cam.capture_request()
        try:
            pixels = request.make_array('main')[:self.height, :self.width].copy()
            metadata = request.get_metadata()
            sequence = request.request.sequence
        finally:
            request.release()
        now = time.clock_gettime_ns(time.CLOCK_BOOTTIME)
        age = (now-metadata['SensorTimestamp'])/1e9
        frame = QtGui.QImage(pixels.data, self.width, self.height, self.width, QtGui.QImage.Format_Grayscale8).copy()
        self.picture.setPixmap(QtGui.QPixmap.fromImage(frame).scaled(
            self.picture.size(), QtCore.Qt.KeepAspectRatio, QtCore.Qt.SmoothTransformation))
        self.status.setText(f'LIVE CAMERA · positioning preview · frame {sequence} · {age*1000:.0f} ms old'
                            if 0 <= age < 1 else 'Camera delayed — preview is not current')
        self.count += 1
        if self.count % 10 == 0:
            tmp = self.evidence/'status.tmp.json'
            tmp.write_text(json.dumps({'mode': 'live camera positioning; no pose estimation',
                                      'sequence': sequence, 'sensor_timestamp_ns': metadata['SensorTimestamp'],
                                      'emitted_boot_ns': now, 'age_s': age, 'live': 0 <= age < 1}))
            tmp.replace(self.evidence/'status.json')

    def update_saved_frame(self):
        index = self.last_saved
        while (self.recording/'images'/f'{index+1:06d}.png').exists():
            index += 1
        if index < 0:
            return
        path = self.recording/'images'/f'{index:06d}.png'
        # The writer may have opened but not finished the newest PNG yet.
        frame = QtGui.QImage(str(path))
        if frame.isNull():
            return
        self.last_saved = index
        self.picture.setPixmap(QtGui.QPixmap.fromImage(frame).scaled(
            self.picture.size(), QtCore.Qt.KeepAspectRatio, QtCore.Qt.SmoothTransformation))
        age = time.time()-path.stat().st_mtime
        live = 0 <= age < 1
        self.status.setText(f'CALIBRATION RECORDING · live saved frame {index}' if live
                            else 'Recording ended or delayed · last saved image')
        self.count += 1
        if self.count % 10 == 0:
            tmp = self.evidence/'status.tmp.json'
            tmp.write_text(json.dumps({'mode': 'calibration recording image preview; no pose estimation',
                                      'session': str(self.recording), 'sequence': index,
                                      'emitted_boot_ns': time.clock_gettime_ns(time.CLOCK_BOOTTIME),
                                      'file_write_age_s': age, 'live': live}))
            tmp.replace(self.evidence/'status.json')

    def closeEvent(self, event):
        self.timer.stop()
        if self.cam is not None:
            self.cam.stop()
            self.cam.close()
        event.accept()


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--recording', type=Path, help='Display saved frames from an active recorder without opening the camera')
    args = parser.parse_args()
    app = QtWidgets.QApplication(sys.argv)
    window = Preview(args.recording)
    # Close after an in-progress capture callback returns. Calling cam.stop()
    # directly from a signal interrupting capture_request can deadlock.
    signal.signal(signal.SIGTERM, lambda *_: QtCore.QTimer.singleShot(0, window.close))
    signal.signal(signal.SIGINT, lambda *_: QtCore.QTimer.singleShot(0, window.close))
    window.showMaximized()
    window.raise_()
    window.activateWindow()
    sys.exit(app.exec_())
