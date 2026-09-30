#!/usr/bin/python3
"""Native, square-pixel Kalibr target; physical screen dimensions must be measured."""
import json
from pathlib import Path
from PyQt5 import QtCore, QtGui, QtWidgets

ROOT = Path(__file__).resolve().parents[1]


def target_image():
    codes = json.loads((ROOT/'config/april36h11-first36.json').read_text())['codes']
    image = QtGui.QImage(89, 89, QtGui.QImage.Format_RGB32)
    image.fill(QtCore.Qt.white)
    painter = QtGui.QPainter(image)
    for tag, code in enumerate(codes):
        # Tag zero occupies bottom left, matching tools/make_target.py.
        x, y = 7 + (tag % 6)*13, 7 + (5-tag//6)*13
        painter.fillRect(x, y, 10, 10, QtCore.Qt.black)
        for row in range(6):
            for col in range(6):
                if (code >> (6*(5-row)+(5-col))) & 1:
                    painter.fillRect(x+2+col, y+2+row, 1, 1, QtCore.Qt.white)
        for dx, dy in [(-3,-3), (10,-3), (10,10), (-3,10)]:
            painter.fillRect(x+dx, y+dy, 3, 3, QtCore.Qt.black)
    painter.end()
    return image


class Target(QtWidgets.QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle('AprilGrid calibration target — measure displayed size')
        self.grid = target_image()

    def paintEvent(self, event):
        p = QtGui.QPainter(self)
        p.fillRect(self.rect(), QtCore.Qt.white)
        p.setPen(QtCore.Qt.black)
        p.setFont(QtGui.QFont('Sans', 12))
        p.drawText(QtCore.QRect(0, 5, self.width(), 28), QtCore.Qt.AlignCenter,
                   'Calibration target — keep the screen still')
        cell = max(1, min((self.width()-70)//89, (self.height()-88)//89))
        side = 89*cell
        x, y = (self.width()-side)//2, 39
        p.setRenderHint(QtGui.QPainter.SmoothPixmapTransform, False)
        p.drawImage(QtCore.QRect(x, y, side, side), self.grid)
        p.setPen(QtGui.QPen(QtGui.QColor('#d00000'), 2))
        lo, hi = 7*cell, 82*cell
        # Red endpoints coincide with the outer edges of the six actual tags.
        p.drawLine(x+lo, y+cell, x+hi, y+cell)
        p.drawLine(x+lo, y, x+lo, y+2*cell)
        p.drawLine(x+hi, y, x+hi, y+2*cell)
        p.drawLine(x+cell, y+lo, x+cell, y+hi)
        p.drawLine(x, y+lo, x+2*cell, y+lo)
        p.drawLine(x, y+hi, x+2*cell, y+hi)
        p.setPen(QtCore.Qt.black)
        p.setFont(QtGui.QFont('Sans', 10))
        p.drawText(QtCore.QRect(0, self.height()-40, self.width(), 32),
                   QtCore.Qt.AlignCenter,
                   'Measure red line lengths: width and height. Keep this zoom unchanged.  Esc closes.')

    def keyPressEvent(self, event):
        if event.key() == QtCore.Qt.Key_Escape:
            self.close()


if __name__ == '__main__':
    app = QtWidgets.QApplication([])
    target = Target()
    target.showFullScreen()
    app.exec_()
