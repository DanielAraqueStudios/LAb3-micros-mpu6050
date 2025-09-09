#!/usr/bin/env python3
"""
Polished PyQt6 frontend for MPU6050 live data.

Features:
- Modern dark UI with numeric cards and polished plots
- Device selection and connect/disconnect controls
- Live converted values (g, °/s, °C)

Run: python read_mpu_gui.py
"""

from collections import deque
import sys, time

import numpy as np
import serial
import serial.tools.list_ports

from PyQt6.QtCore import Qt, QTimer, pyqtSignal, QThread
from PyQt6.QtWidgets import (
    QApplication, QWidget, QLabel, QPushButton, QHBoxLayout,
    QVBoxLayout, QComboBox, QFrame, QGridLayout, QSizePolicy
)
import pyqtgraph as pg

ACCEL_SENS = 16384.0
GYRO_SENS = 131.0
SAMPLE_HZ = 30
BUFFER_SECONDS = 6
BUFFER_LEN = SAMPLE_HZ * BUFFER_SECONDS


class SerialReader(QThread):
    data_ready = pyqtSignal(object)
    error = pyqtSignal(str)

    def __init__(self, port, baud=115200, parent=None):
        super().__init__(parent)
        self.port = port
        self.baud = baud
        self._running = False
        self.ser = None

    def run(self):
        try:
            self.ser = serial.Serial(self.port, self.baud, timeout=1)
        except Exception as e:
            self.error.emit(str(e))
            return
        self._running = True
        while self._running:
            try:
                line = self.ser.readline().decode(errors='ignore').strip()
                if not line:
                    continue
                parts = line.split(',')
                if len(parts) != 7:
                    continue
                ax, ay, az, gx, gy, gz, temp = map(int, parts)
                self.data_ready.emit({'t': time.time(), 'ax': ax, 'ay': ay, 'az': az,
                                      'gx': gx, 'gy': gy, 'gz': gz, 'temp': temp})
            except Exception as e:
                self.error.emit(str(e))
                break
        try:
            if self.ser and self.ser.is_open:
                self.ser.close()
        except Exception:
            pass

    def stop(self):
        self._running = False
        self.wait(1000)


class StatCard(QFrame):
    def __init__(self, title, subtitle='', parent=None):
        super().__init__(parent)
        self.setObjectName('card')
        self.setStyleSheet('''
        QFrame#card{background: #1b1b1f; border-radius:10px; padding:10px}
        QLabel#value{color:#ffffff; font-size:20px; font-weight:700}
        QLabel#label{color:#bfbfbf; font-size:11px}
        ''')
        v = QVBoxLayout(self)
        self.label = QLabel(title)
        self.label.setObjectName('label')
        self.value = QLabel(subtitle)
        self.value.setObjectName('value')
        v.addWidget(self.label)
        v.addStretch()
        v.addWidget(self.value)

    def setValue(self, text):
        self.value.setText(text)


class MainWindow(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle('MPU6050 — Modern Live Monitor')
        self.resize(1100, 720)
        self._build_ui()
        self._init_state()

    def _build_ui(self):
        # Header
        title = QLabel('MPU6050 Live')
        title.setStyleSheet('color:#ffffff; font-size:20px; font-weight:700')
        subtitle = QLabel('Real-time sensor dashboard')
        subtitle.setStyleSheet('color:#9aa0a6')

        # Controls
        self.port_cb = QComboBox(); self._refresh_ports()
        self.refresh_btn = QPushButton('Refresh')
        self.connect_btn = QPushButton('Connect')
        self.disconnect_btn = QPushButton('Disconnect'); self.disconnect_btn.setEnabled(False)

        hdr_left = QVBoxLayout(); hdr_left.addWidget(title); hdr_left.addWidget(subtitle)
        hdr_right = QHBoxLayout(); hdr_right.addWidget(self.port_cb); hdr_right.addWidget(self.refresh_btn);
        hdr_right.addWidget(self.connect_btn); hdr_right.addWidget(self.disconnect_btn)

        header = QHBoxLayout()
        header.addLayout(hdr_left)
        header.addStretch()
        header.addLayout(hdr_right)

        # Cards area
        self.cards = {}
        grid = QGridLayout()
        keys = [('ax','Accel X (g)'), ('ay','Accel Y (g)'), ('az','Accel Z (g)'),
                ('acc_mag','|a| (g)'), ('gx','Gyro X (°/s)'), ('gy','Gyro Y (°/s)'), ('gz','Gyro Z (°/s)'),
                ('temp','Temp (°C)')]
        r = c = 0
        for k, label in keys:
            card = StatCard(label, '--')
            card.setMinimumHeight(80)
            card.setMaximumWidth(220)
            self.cards[k] = card
            grid.addWidget(card, r, c)
            c += 1
            if c >= 4:
                c = 0; r += 1

        left_v = QVBoxLayout(); left_v.addLayout(grid); left_v.addStretch()

        # Plots
        pg.setConfigOptions(antialias=True)
        self.plot_acc = pg.PlotWidget(background='#0f0f10', title='Accelerometer (g)')
        self.plot_gyro = pg.PlotWidget(background='#0f0f10', title='Gyroscope (°/s)')
        for p in (self.plot_acc, self.plot_gyro):
            p.getAxis('left').setPen('#9aa0a6'); p.getAxis('bottom').setPen('#9aa0a6')
            p.showGrid(x=True, y=True, alpha=0.2)
        self.acc_curves = [self.plot_acc.plot(pen=pg.mkPen(col, width=2)) for col in ('#ff6b6b','#50fa7b','#51a0ff')]
        self.gyro_curves = [self.plot_gyro.plot(pen=pg.mkPen(col, width=2)) for col in ('#ff6b6b','#50fa7b','#51a0ff')]

        plots_v = QVBoxLayout(); plots_v.addWidget(self.plot_acc); plots_v.addWidget(self.plot_gyro)

        main_h = QHBoxLayout();
        left_frame = QFrame(); left_frame.setLayout(left_v); left_frame.setStyleSheet('background:#151516; padding:8px; border-radius:8px'); left_frame.setMaximumWidth(930)
        main_h.addWidget(left_frame)
        main_h.addLayout(plots_v)

        root = QVBoxLayout(self)
        root.addLayout(header)
        root.addLayout(main_h)

        # Connect signals
        self.refresh_btn.clicked.connect(self._refresh_ports)
        self.connect_btn.clicked.connect(self._connect)
        self.disconnect_btn.clicked.connect(self._disconnect)

    def _init_state(self):
        self.reader = None
        self.tbuf = deque(maxlen=BUFFER_LEN)
        self.ax = deque(maxlen=BUFFER_LEN); self.ay = deque(maxlen=BUFFER_LEN); self.az = deque(maxlen=BUFFER_LEN)
        self.gx = deque(maxlen=BUFFER_LEN); self.gy = deque(maxlen=BUFFER_LEN); self.gz = deque(maxlen=BUFFER_LEN)

        self.timer = QTimer(); self.timer.setInterval(int(1000 / SAMPLE_HZ)); self.timer.timeout.connect(self._update_plots)

    def _refresh_ports(self):
        self.port_cb.clear()
        for p in serial.tools.list_ports.comports():
            self.port_cb.addItem(f"{p.device} — {p.description}", p.device)

    def _connect(self):
        port = self.port_cb.currentData()
        if not port:
            return
        self.reader = SerialReader(port)
        self.reader.data_ready.connect(self._on_data)
        self.reader.error.connect(self._on_error)
        self.reader.start()
        self.timer.start()
        self.connect_btn.setEnabled(False); self.disconnect_btn.setEnabled(True)

    def _disconnect(self):
        if self.reader:
            self.reader.stop(); self.reader = None
        self.timer.stop(); self.connect_btn.setEnabled(True); self.disconnect_btn.setEnabled(False)

    def _on_data(self, d):
        t = d['t']
        self.tbuf.append(t)
        axg = d['ax'] / ACCEL_SENS; ayg = d['ay'] / ACCEL_SENS; azg = d['az'] / ACCEL_SENS
        gx = d['gx'] / GYRO_SENS; gy = d['gy'] / GYRO_SENS; gz = d['gz'] / GYRO_SENS
        self.ax.append(axg); self.ay.append(ayg); self.az.append(azg)
        self.gx.append(gx); self.gy.append(gy); self.gz.append(gz)
        acc_mag = (axg**2 + ayg**2 + azg**2) ** 0.5
        temp_c = d['temp'] / 340.0 + 36.53
        # Update cards
        self.cards['ax'].setValue(f"{axg:.3f}")
        self.cards['ay'].setValue(f"{ayg:.3f}")
        self.cards['az'].setValue(f"{azg:.3f}")
        self.cards['acc_mag'].setValue(f"{acc_mag:.3f}")
        self.cards['gx'].setValue(f"{gx:.1f}")
        self.cards['gy'].setValue(f"{gy:.1f}")
        self.cards['gz'].setValue(f"{gz:.1f}")
        self.cards['temp'].setValue(f"{temp_c:.2f}")

    def _on_error(self, msg):
        print('Serial error:', msg)
        self._disconnect()

    def _update_plots(self):
        if len(self.tbuf) < 2:
            return
        t0 = self.tbuf[0]
        xs = np.array(self.tbuf) - t0
        self.acc_curves[0].setData(xs, np.array(self.ax))
        self.acc_curves[1].setData(xs, np.array(self.ay))
        self.acc_curves[2].setData(xs, np.array(self.az))
        self.gyro_curves[0].setData(xs, np.array(self.gx))
        self.gyro_curves[1].setData(xs, np.array(self.gy))
        self.gyro_curves[2].setData(xs, np.array(self.gz))
        self.plot_acc.setYRange(-4, 4)
        self.plot_gyro.setYRange(-500, 500)


def main():
    app = QApplication(sys.argv)
    app.setStyle('Fusion')
    # Minimal dark stylesheet
    app.setStyleSheet('''
        QWidget { background: #0f0f11; color: #e6eef3; font-family: Inter, Arial; }
        QPushButton { background:#242426; color:#eaf2f5; padding:6px 10px; border-radius:6px }
        QComboBox { background:#1b1b1d; color:#eaf2f5; padding:4px }
    ''')
    w = MainWindow()
    w.show()
    sys.exit(app.exec())


if __name__ == '__main__':
    main()
