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

from PyQt6.QtCore import Qt, QTimer, pyqtSignal, QThread, QEasingCurve, QPropertyAnimation, QPointF
from PyQt6.QtWidgets import (
    QApplication, QWidget, QLabel, QPushButton, QHBoxLayout,
    QVBoxLayout, QComboBox, QFrame, QGridLayout, QSizePolicy, QGraphicsDropShadowEffect
)
from PyQt6.QtGui import QColor, QPainter, QPaintEvent, QPen, QBrush, QFont, QLinearGradient, QPolygonF
import pyqtgraph as pg
from pyqtgraph.opengl import GLViewWidget, MeshData, GLMeshItem, GLLinePlotItem

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
    def __init__(self, title, subtitle='', bg='#6A4C93', label_color='#FFCA3A', parent=None):
        """Stat card with configurable background so we can use the full palette.

        bg: background hex color for the card
        label_color: color for the small label text
        """
        super().__init__(parent)
        self.setObjectName('card')
        # dynamic stylesheet per-card using provided palette color
        self.setStyleSheet(f'''
        QFrame#card{{background: {bg}; border-radius:10px; padding:12px}}
        QLabel#value{{color:#ffffff; font-size:20px; font-weight:800}}
        QLabel#label{{color:{label_color}; font-size:11px; font-weight:600}}
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


class HexPatternWidget(QWidget):
    """Paints a subtle hexagonal pattern used as a futuristic HUD background."""
    def __init__(self, parent=None, color='#0f1112'):
        super().__init__(parent)
        self._color = QColor(color)
        self.setAttribute(Qt.WidgetAttribute.WA_TransparentForMouseEvents)

    def paintEvent(self, ev: QPaintEvent):
        p = QPainter(self)
        p.setRenderHint(QPainter.RenderHint.Antialiasing)
        w = self.width(); h = self.height()
        pen = QPen(QColor(255,255,255,12))
        pen.setWidth(1)
        p.setPen(pen)
        p.setBrush(Qt.BrushStyle.NoBrush)
        # draw repeating hex tiles
        size = 36
        dx = size * 3**0.5 / 2
        dy = size * 0.75
        cols = int(w / dx) + 3
        rows = int(h / dy) + 3
        for r in range(rows):
            for c in range(cols):
                cx = c * dx + (r % 2) * (dx/2) - dx
                cy = r * dy - dy
                self._draw_hex(p, cx, cy, size*0.45)
        p.end()

    def _draw_hex(self, p, cx, cy, rad):
        pts = []
        for i in range(6):
            a = i * (2 * np.pi / 6)
            x = cx + rad * np.cos(a)
            y = cy + rad * np.sin(a)
            pts.append((x, y))
        # build QPolygonF and draw it
        qpts = QPolygonF([QPointF(x, y) for x, y in pts])
        p.drawPolygon(qpts)


class StatusIndicator(QWidget):
    """Pulsing circular status indicator with neon glow."""
    def __init__(self, color='#8AC926', parent=None):
        super().__init__(parent)
        self._color = QColor(color)
        self.setFixedSize(18,18)
        self._glow = QGraphicsDropShadowEffect(blurRadius=18, xOffset=0, yOffset=0)
        self._glow.setColor(self._color)
        self.setGraphicsEffect(self._glow)
        self._anim = QPropertyAnimation(self._glow, b'blurRadius', self)
        self._anim.setStartValue(6)
        self._anim.setEndValue(26)
        self._anim.setDuration(1000)
        self._anim.setLoopCount(-1)
        self._anim.setEasingCurve(QEasingCurve.Type.InOutQuad)
        self._anim.start()

    def paintEvent(self, ev: QPaintEvent):
        p = QPainter(self)
        p.setRenderHint(QPainter.RenderHint.Antialiasing)
        r = min(self.width(), self.height()) - 4
        rect = self.rect().adjusted(2,2,-2,-2)
        grad = QLinearGradient(rect.topLeft(), rect.bottomRight())
        grad.setColorAt(0.0, self._color.lighter(150))
        grad.setColorAt(1.0, self._color.darker(110))
        p.setBrush(QBrush(grad))
        p.setPen(Qt.PenStyle.NoPen)
        p.drawEllipse(rect)
        p.end()


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
        title.setStyleSheet('color:#FFCA3A; font-size:20px; font-weight:800')
        subtitle = QLabel('Real-time sensor dashboard')
        subtitle.setStyleSheet('color:#8AC926')

        # Controls
        self.port_cb = QComboBox()
        self._refresh_ports()
        self.refresh_btn = QPushButton('Refresh')
        self.connect_btn = QPushButton('Connect')
        self.disconnect_btn = QPushButton('Disconnect')
        self.disconnect_btn.setEnabled(False)

        hdr_left = QVBoxLayout()
        hdr_left.addWidget(title)
        hdr_left.addWidget(subtitle)
        hdr_right = QHBoxLayout()
        hdr_right.addWidget(self.port_cb)
        hdr_right.addWidget(self.refresh_btn)
        hdr_right.addWidget(self.connect_btn)
        hdr_right.addWidget(self.disconnect_btn)

        header = QHBoxLayout()
        header.addLayout(hdr_left)
        header.addStretch()
        header.addLayout(hdr_right)

        # Cards area
        self.cards = {}
        grid = QGridLayout()
        keys = [
            ('ax','Accel X (g)'), ('ay','Accel Y (g)'), ('az','Accel Z (g)'),
            ('acc_mag','|a| (g)'), ('gx','Gyro X (°/s)'), ('gy','Gyro Y (°/s)'), ('gz','Gyro Z (°/s)'),
            ('temp','Temp (°C)'), ('roll','Roll (°)'), ('pitch','Pitch (°)'), ('yaw','Yaw (°)')
        ]
        palette = ['#6A4C93', '#1982C4', '#8AC926', '#FFCA3A', '#FF595E']
        r = c = 0
        for i, (k, label) in enumerate(keys):
            card_bg = palette[i % len(palette)]
            label_accent = '#FFCA3A' if card_bg != '#FFCA3A' else '#6A4C93'
            card = StatCard(label, '--', bg=card_bg, label_color=label_accent)
            card.setMinimumHeight(80)
            card.setMaximumWidth(220)
            self.cards[k] = card
            grid.addWidget(card, r, c)
            c += 1
            if c >= 4:
                c = 0
                r += 1

        left_v = QVBoxLayout()
        left_v.addLayout(grid)

        # Plots & 3D view: create GL view first, then place it under the cards
        pg.setConfigOptions(antialias=True)
        self.gl_view = GLViewWidget()
        self.gl_view.setBackgroundColor((15,15,16))
        self.gl_view.opts['distance'] = 6

        # cuboid geometry
        lx, ly, lz = 2.0, 1.2, 0.25
        x = lx / 2.0; y = ly / 2.0; z = lz / 2.0
        base_verts = np.array([
            [-x, -y, -z], [ x, -y, -z], [ x,  y, -z], [-x,  y, -z],
            [-x, -y,  z], [ x, -y,  z], [ x,  y,  z], [-x,  y,  z]
        ], dtype=float)
        base_faces = np.array([
            [0,1,2],[0,2,3],
            [4,5,6],[4,6,7],
            [0,1,5],[0,5,4],
            [1,2,6],[1,6,5],
            [2,3,7],[2,7,6],
            [3,0,4],[3,4,7],
        ], dtype=int)
        self._base_verts = base_verts
        self._base_faces = base_faces
        base_mesh = MeshData(vertexes=base_verts, faces=base_faces)
        self.plane_mesh_item = GLMeshItem(meshdata=base_mesh, smooth=False, color=(0.4157,0.2980,0.5765,1.0), shader='shaded', drawEdges=True)
        self.gl_view.addItem(self.plane_mesh_item)
        ax_line = GLLinePlotItem(pos=np.array([[0,0,0],[1.5,0,0]]), color=(1.0,0.349,0.369,1), width=2, antialias=True)
        ay_line = GLLinePlotItem(pos=np.array([[0,0,0],[0,1.5,0]]), color=(0.541,0.788,0.149,1), width=2, antialias=True)
        az_line = GLLinePlotItem(pos=np.array([[0,0,0],[0,0,1.5]]), color=(0.098,0.510,0.769,1), width=2, antialias=True)
        self.gl_view.addItem(ax_line)
        self.gl_view.addItem(ay_line)
        self.gl_view.addItem(az_line)

        self.gl_view.setMinimumHeight(220)
        left_v.addWidget(self.gl_view)
        left_v.addStretch()

        # plots
        self.plot_acc = pg.PlotWidget(background='#111214', title='Accelerometer (g)')
        self.plot_gyro = pg.PlotWidget(background='#111214', title='Gyroscope (°/s)')
        for p in (self.plot_acc, self.plot_gyro):
            p.getAxis('left').setPen('#dfeaf1')
            p.getAxis('bottom').setPen('#dfeaf1')
            p.showGrid(x=True, y=True, alpha=0.12)
        self.acc_curves = [self.plot_acc.plot(pen=pg.mkPen(col, width=2)) for col in ('#6A4C93','#1982C4','#8AC926')]
        self.gyro_curves = [self.plot_gyro.plot(pen=pg.mkPen(col, width=2)) for col in ('#FFCA3A','#FF595E','#6A4C93')]

        # combine plots (3D already placed under cards)
        plots_v = QVBoxLayout()
        plots_v.addWidget(self.plot_acc)
        plots_v.addWidget(self.plot_gyro)

        main_h = QHBoxLayout()
        left_frame = QFrame()
        left_frame.setLayout(left_v)
        # left panel uses dark background to contrast cards
        left_frame.setStyleSheet('background:#0f1112; padding:8px; border-radius:8px; color:#e6eef3')
        left_frame.setMaximumWidth(930)
        main_h.addWidget(left_frame)
        main_h.addLayout(plots_v)

        root = QVBoxLayout(self)
        root.addLayout(header)
        root.addLayout(main_h)

        # style buttons using the palette
        self.refresh_btn.setStyleSheet('background:#6A4C93; color:#ffffff; padding:6px; border-radius:6px')
        self.connect_btn.setStyleSheet('background:#8AC926; color:#0f1112; padding:6px; border-radius:6px')
        self.disconnect_btn.setStyleSheet('background:#FF595E; color:#ffffff; padding:6px; border-radius:6px')

        # Connect signals
        self.refresh_btn.clicked.connect(self._refresh_ports)
        self.connect_btn.clicked.connect(self._connect)
        self.disconnect_btn.clicked.connect(self._disconnect)

    def _init_state(self):
        self.reader = None
        self.tbuf = deque(maxlen=BUFFER_LEN)
        self.ax = deque(maxlen=BUFFER_LEN)
        self.ay = deque(maxlen=BUFFER_LEN)
        self.az = deque(maxlen=BUFFER_LEN)
        self.gx = deque(maxlen=BUFFER_LEN)
        self.gy = deque(maxlen=BUFFER_LEN)
        self.gz = deque(maxlen=BUFFER_LEN)
        # orientation state (degrees)
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0
        self._last_ang_t = None

        self.timer = QTimer()
        self.timer.setInterval(int(1000 / SAMPLE_HZ))
        self.timer.timeout.connect(self._update_plots)

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
        # orientation estimation (complementary filter)
        # accel-based angles (radians)
        try:
            roll_acc = np.arctan2(ayg, azg)
            pitch_acc = np.arctan2(-axg, np.sqrt(ayg*ayg + azg*azg))
        except Exception:
            roll_acc = 0.0; pitch_acc = 0.0
        # delta t
        if self._last_ang_t is None:
            dt = 1.0 / SAMPLE_HZ
        else:
            dt = max(1e-3, t - self._last_ang_t)
        self._last_ang_t = t
        # integrate gyro rates (deg/s) -> degrees
        # gx,gy,gz are in deg/s already from conversion earlier
        alpha = 0.96
        self.roll = alpha * (self.roll + gx * dt) + (1 - alpha) * (np.degrees(roll_acc))
        self.pitch = alpha * (self.pitch + gy * dt) + (1 - alpha) * (np.degrees(pitch_acc))
        self.yaw = self.yaw + gz * dt
        # update angle cards
        self.cards['roll'].setValue(f"{self.roll:.1f}")
        self.cards['pitch'].setValue(f"{self.pitch:.1f}")
        self.cards['yaw'].setValue(f"{self.yaw:.1f}")

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
        # update 3D plane orientation
        self._update_3d_plane()

    def _rotation_matrix(self, roll_deg, pitch_deg, yaw_deg):
        # create rotation matrix R = Rz(yaw) * Ry(pitch) * Rx(roll)
        r = np.radians(roll_deg); p = np.radians(pitch_deg); y = np.radians(yaw_deg)
        Rx = np.array([[1,0,0],[0,np.cos(r), -np.sin(r)],[0,np.sin(r), np.cos(r)]])
        Ry = np.array([[np.cos(p),0,np.sin(p)],[0,1,0],[-np.sin(p),0,np.cos(p)]])
        Rz = np.array([[np.cos(y), -np.sin(y),0],[np.sin(y), np.cos(y),0],[0,0,1]])
        return Rz @ Ry @ Rx

    def _update_3d_plane(self):
        R = self._rotation_matrix(self.roll, self.pitch, self.yaw)
        # apply rotation to all 8 vertices
        verts = (R @ self._base_verts.T).T
        mesh = MeshData(vertexes=verts, faces=self._base_faces)
        # update mesh item (replace geometry)
        self.plane_mesh_item.setMeshData(meshdata=mesh)


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
