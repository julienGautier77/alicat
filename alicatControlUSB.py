# -*- coding: utf-8 -*-
"""
Created on 26/09/25
pip install pyzmq pyserial pyqtgraph
@author: SALLEJAUNE

Serveur ZMQ REQ/REP sur port 5560.
Commandes :
  {"cmd": "ping"} / {"cmd": "get_pressure"} / {"cmd": "get_setpoint"}
  {"cmd": "get_all"} / {"cmd": "set_setpoint", "value": 5.0}
  {"cmd": "get_pid"} / {"cmd": "set_pid", "P":..., "D":..., "I":..., "loop_type":...}
"""

from PyQt6 import QtCore
from PyQt6.QtWidgets import (QApplication, QWidget, QMainWindow, QDialog,
                             QVBoxLayout, QHBoxLayout, QDoubleSpinBox, QSpinBox,
                             QLabel, QPushButton, QFormLayout, QGroupBox,
                             QComboBox, QFrame, QSizePolicy)
from PyQt6.QtGui import QIcon, QFont, QColor, QPainter, QPen, QBrush, QCursor
from PyQt6.QtCore import Qt, pyqtSignal
import sys
import time
import math
import pathlib
import os
import zmq
import threading
import socket as _socket
from collections import deque
import pyqtgraph as pg
from alicatLibUSB import AlicatController


# ---------------------------------------------------------------------------
# Polices — standard, disponibles sur Windows et Linux sans installation
# ---------------------------------------------------------------------------
MONO_FONT = 'Courier New'   # présent partout
SANS_FONT = 'Arial'         # présent partout


# ---------------------------------------------------------------------------
# Palette & Style  (généré dynamiquement après détection des polices)
# ---------------------------------------------------------------------------
def build_style(mono: str, sans: str) -> str:
    return f"""
QMainWindow, QDialog, QWidget {{
    background-color: #0d1117;
    color: #c9d1d9;
    font-family: '{sans}', sans-serif;
    font-size: 12px;
}}
QGroupBox {{
    border: 1px solid #21262d;
    border-radius: 8px;
    margin-top: 14px;
    padding: 12px 10px 8px 10px;
    font-size: 10px;
    font-weight: bold;
    letter-spacing: 1.5px;
    color: #484f58;
}}
QGroupBox::title {{
    subcontrol-origin: margin;
    subcontrol-position: top left;
    left: 12px; top: -1px;
    padding: 0 4px;
    background-color: #0d1117;
}}
QDoubleSpinBox, QSpinBox {{
    background-color: #161b22;
    border: 1px solid #30363d;
    border-radius: 6px;
    padding: 5px 8px;
    color: #e6edf3;
    font-family: '{mono}', monospace;
    font-size: 13px;
    min-width: 110px;
}}
QDoubleSpinBox:focus, QSpinBox:focus {{ border: 1px solid #1f6feb; }}
QDoubleSpinBox::up-button, QSpinBox::up-button,
QDoubleSpinBox::down-button, QSpinBox::down-button {{
    background-color: #21262d; border: none; width: 20px;
}}
QDoubleSpinBox::up-button:hover, QSpinBox::up-button:hover,
QDoubleSpinBox::down-button:hover, QSpinBox::down-button:hover {{
    background-color: #30363d;
}}
QComboBox {{
    background-color: #161b22; border: 1px solid #30363d;
    border-radius: 6px; padding: 5px 10px; color: #e6edf3; min-width: 120px;
}}
QComboBox:focus {{ border: 1px solid #1f6feb; }}
QComboBox::drop-down {{ border: none; width: 24px; }}
QComboBox QAbstractItemView {{
    background-color: #161b22; border: 1px solid #30363d;
    selection-background-color: #1f6feb; color: #e6edf3;
}}
QPushButton {{
    background-color: #21262d; border: 1px solid #30363d;
    border-radius: 6px; padding: 6px 14px; color: #c9d1d9; font-weight: 500;
}}
QPushButton:hover {{ background-color: #30363d; border-color: #484f58; color: #e6edf3; }}
QPushButton:pressed {{ background-color: #161b22; }}
QPushButton#btnPrimary {{
    background-color: #0d419d; border-color: #1f6feb; color: #fff; font-weight: bold;
}}
QPushButton#btnPrimary:hover {{ background-color: #1f6feb; }}
QPushButton#btnSuccess {{
    background-color: #1a4a1a; border-color: #3fb950; color: #3fb950; font-weight: bold;
}}
QPushButton#btnSuccess:hover {{ background-color: #238636; color: #fff; }}
QPushButton#btnPid {{
    background-color: #1a2f45; border: 1px solid #1f6feb;
    border-radius: 6px; color: #58a6ff; font-weight: bold;
    padding: 7px 20px; font-size: 11px; letter-spacing: 0.5px;
}}
QPushButton#btnPid:hover {{ background-color: #1f6feb; color: #fff; }}
QFrame#separator {{ background-color: #21262d; max-height: 1px; }}
QStatusBar {{
    background-color: #161b22; border-top: 1px solid #21262d;
    color: #484f58; font-size: 10px;
}}
QLabel {{ background: transparent; }}
"""

STYLE: str = ''   # rempli dans main() après détection des polices


# Constantes géométrie gauge
GAUGE_START  = 225   # angle de départ en degrés (bas-gauche)
GAUGE_SPAN   = 270   # amplitude totale en degrés (sens anti-horaire)


# ---------------------------------------------------------------------------
# Widget gauge circulaire interactive
# ---------------------------------------------------------------------------
class PressureGauge(QWidget):
    """
    Gauge circulaire dessinée avec QPainter.
    - Arc amber  = setpoint  (anneau extérieur)
    - Arc coloré = mesure    (anneau intérieur)
    - Drag circulaire → émet setpoint_changed(float)
    """
    setpoint_changed = pyqtSignal(float)

    def __init__(self, max_val: float = 80, parent=None):
        super().__init__(parent)
        self.max_val    = max_val
        self._value     = 0.0
        self._setpoint  = 0.0
        self._dragging  = False
        self.setMinimumSize(230, 230)
        self.setMaximumSize(250, 250)
        self.setSizePolicy(QSizePolicy.Policy.Fixed, QSizePolicy.Policy.Fixed)
        self.setCursor(QCursor(Qt.CursorShape.OpenHandCursor))

    # ---- Setters -----------------------------------------------------------

    def setValue(self, v: float):
        self._value = max(0.0, min(float(v), self.max_val))
        self.update()

    def setSetpoint(self, v: float):
        self._setpoint = max(0.0, min(float(v), self.max_val))
        self.update()

    # ---- Géométrie interne -------------------------------------------------

    def _geometry(self):
        w, h = self.width(), self.height()
        cx, cy = w / 2, h / 2
        r = min(w, h) / 2 - 14
        return cx, cy, r

    def _angle_from_value(self, v: float) -> float:
        """Valeur → angle en degrés (repère trigonométrique standard)."""
        frac  = v / self.max_val
        angle = GAUGE_START - frac * GAUGE_SPAN   # sens anti-horaire
        return angle

    def _value_from_pos(self, x: float, y: float) -> float:
        """Position souris → valeur setpoint."""
        cx, cy, _ = self._geometry()
        dx, dy = x - cx, cy - y          # dy inversé (axe Qt vers le bas)
        angle  = math.degrees(math.atan2(dy, dx))  # -180..180
        # Normaliser dans le repère de la gauge
        # START=225° (bas-gauche), tourne anti-horaire sur SPAN=270°
        rel = GAUGE_START - angle
        if rel < 0:
            rel += 360
        if rel > 360:
            rel -= 360
        # Clips hors de la plage utile
        if rel > GAUGE_SPAN:
            # Côté "avant" le 0 → forcer à 0
            if rel < GAUGE_SPAN + (360 - GAUGE_SPAN) / 2:
                rel = GAUGE_SPAN
            else:
                rel = 0
        v = (rel / GAUGE_SPAN) * self.max_val
        return round(max(0.0, min(v, self.max_val)), 1)

    # ---- Interactions souris -----------------------------------------------

    def mousePressEvent(self, event):
        if event.button() == Qt.MouseButton.LeftButton:
            self._dragging = True
            self.setCursor(QCursor(Qt.CursorShape.ClosedHandCursor))
            self._update_setpoint_from_event(event)

    def mouseMoveEvent(self, event):
        if self._dragging:
            self._update_setpoint_from_event(event)

    def mouseReleaseEvent(self, event):
        if event.button() == Qt.MouseButton.LeftButton:
            self._dragging = False
            self.setCursor(QCursor(Qt.CursorShape.OpenHandCursor))

    def _update_setpoint_from_event(self, event):
        v = self._value_from_pos(event.position().x(), event.position().y())
        self._setpoint = v
        self.setpoint_changed.emit(v)
        self.update()

    # ---- Dessin ------------------------------------------------------------

    def paintEvent(self, event):
        p = QPainter(self)
        p.setRenderHint(QPainter.RenderHint.Antialiasing)

        cx, cy, r = self._geometry()
        cx, cy = int(cx), int(cy)
        ri = int(r)

        # ── Fond disque ──
        p.setPen(Qt.PenStyle.NoPen)
        p.setBrush(QBrush(QColor("#0d1117")))
        p.drawEllipse(cx - ri, cy - ri, 2*ri, 2*ri)

        # ── Arc fond (gris) ──
        pen_bg = QPen(QColor("#21262d"), 12, Qt.PenStyle.SolidLine,
                      Qt.PenCapStyle.RoundCap)
        p.setPen(pen_bg)
        p.drawArc(cx - ri + 6, cy - ri + 6, 2*(ri-6), 2*(ri-6),
                  GAUGE_START * 16, -GAUGE_SPAN * 16)

        # ── Arc setpoint (amber, anneau extérieur) ──
        if self._setpoint > 0:
            sp_span = int(GAUGE_SPAN * self._setpoint / self.max_val)
            pen_sp = QPen(QColor("#d29922"), 12, Qt.PenStyle.SolidLine,
                          Qt.PenCapStyle.RoundCap)
            p.setPen(pen_sp)
            p.drawArc(cx - ri + 6, cy - ri + 6, 2*(ri-6), 2*(ri-6),
                      GAUGE_START * 16, -sp_span * 16)

        # ── Arc valeur (anneau intérieur, plus fin) ──
        if self._value > 0:
            val_span = int(GAUGE_SPAN * self._value / self.max_val)
            if self._value > self.max_val * 0.85:
                col = "#f85149"
            elif self._value > self.max_val * 0.70:
                col = "#d29922"
            else:
                col = "#39d353"
            pen_v = QPen(QColor(col), 7, Qt.PenStyle.SolidLine,
                         Qt.PenCapStyle.RoundCap)
            p.setPen(pen_v)
            inner = int(ri - 10)
            p.drawArc(cx - inner + 6, cy - inner + 6, 2*(inner-6), 2*(inner-6),
                      GAUGE_START * 16, -val_span * 16)

        # ── Disque intérieur ──
        p.setPen(QPen(QColor("#21262d"), 1))
        p.setBrush(QBrush(QColor("#161b22")))
        half = int(ri * 0.55)
        p.drawEllipse(cx - half, cy - half, 2*half, 2*half)

        # ── Valeur mesurée ──
        p.setPen(QPen(QColor("#e6edf3")))
        p.setFont(QFont(MONO_FONT, 17, QFont.Weight.Bold))
        p.drawText(cx - 48, cy - 18, 96, 28,
                   Qt.AlignmentFlag.AlignCenter, f"{self._value:.2f}")

        # ── Unité ──
        p.setPen(QPen(QColor("#484f58")))
        p.setFont(QFont(SANS_FONT, 8))
        p.drawText(cx - 24, cy + 10, 48, 16,
                   Qt.AlignmentFlag.AlignCenter, "Bar")

        # ── Setpoint (petit texte en haut) ──
        p.setPen(QPen(QColor("#d29922")))
        p.setFont(QFont(MONO_FONT, 8))
        p.drawText(cx - 40, cy - 42, 80, 16,
                   Qt.AlignmentFlag.AlignCenter, f"SP {self._setpoint:.1f}")

        # ── Curseur setpoint (petit triangle sur l'arc) ──
        if self._setpoint >= 0:
            sp_angle = math.radians(self._angle_from_value(self._setpoint))
            tick_r = int(ri - 6)
            tx = int(cx + tick_r * math.cos(sp_angle))
            ty = int(cy - tick_r * math.sin(sp_angle))
            p.setPen(Qt.PenStyle.NoPen)
            p.setBrush(QBrush(QColor("#d29922")))
            p.drawEllipse(tx - 5, ty - 5, 10, 10)

        # ── Labels 0 / max ──
        p.setPen(QPen(QColor("#8b949e")))
        p.setFont(QFont(MONO_FONT, 9, QFont.Weight.Bold))
        lbl_r = int(ri + 10)
        # 0  = début de l'arc  → GAUGE_START = 225° (bas-gauche)
        # max = fin de l'arc   → GAUGE_START - GAUGE_SPAN = -45° (bas-droite)
        a0 = math.radians(GAUGE_START)
        ae = math.radians(GAUGE_START - GAUGE_SPAN)
        x0 = int(cx + lbl_r * math.cos(a0))
        y0 = int(cy - lbl_r * math.sin(a0))
        xe = int(cx + lbl_r * math.cos(ae))
        ye = int(cy - lbl_r * math.sin(ae))
        p.drawText(x0 - 16, y0 - 8, 32, 16, Qt.AlignmentFlag.AlignCenter, "0")
        p.drawText(xe - 20, ye - 8, 40, 16, Qt.AlignmentFlag.AlignCenter,
                   str(int(self.max_val)))

        p.end()


# ---------------------------------------------------------------------------
# Utilitaire réseau
# ---------------------------------------------------------------------------
def get_local_ip() -> str:
    try:
        s = _socket.socket(_socket.AF_INET, _socket.SOCK_DGRAM)
        s.connect(("8.8.8.8", 80))
        ip = s.getsockname()[0]
        s.close()
        return ip
    except Exception:
        return "127.0.0.1"


# ---------------------------------------------------------------------------
# Serveur ZMQ
# ---------------------------------------------------------------------------
class ZmqServer(threading.Thread):

    def __init__(self, alicat: AlicatController, port: int = 5560,
                 ui_setpoint_cb=None):
        super().__init__(daemon=True)
        self.alicat = alicat
        self.zmq_port = port
        self._ui_setpoint_cb = ui_setpoint_cb   # appelé quand set_setpoint reçu
        self._stop_event = threading.Event()
        self.context = zmq.Context()
        self.socket = self.context.socket(zmq.REP)
        self.socket.bind(f"tcp://*:{self.zmq_port}")
        self.socket.setsockopt(zmq.RCVTIMEO, 200)

    def run(self):
        while not self._stop_event.is_set():
            try:
                msg = self.socket.recv_json()
            except zmq.Again:
                continue
            except Exception as e:
                print(f"[ZmqServer] recv error: {e}")
                continue
            try:
                self.socket.send_json(self._handle(msg))
            except Exception as e:
                print(f"[ZmqServer] send error: {e}")

    def _handle(self, msg: dict) -> dict:
        cmd = msg.get("cmd", "")
        try:
            if cmd == "ping":
                return {"status": "pong"}
            elif cmd == "get_pressure":
                return {"pressure": round(float(self.alicat.get_pressure()), 3)}
            elif cmd == "get_all":
                r = self.alicat.get_readings()
                if r:
                    return {"pressure": round(r['pressure_primary'], 3),
                            "setpoint": round(r['setpoint'], 3),
                            "state": r.get('state', '')}
                return {"error": "Pas de reponse"}
            elif cmd == "get_setpoint":
                r = self.alicat.get_readings()
                return {"setpoint": round(float(r['setpoint'] if r else 0), 3)}
            elif cmd == "set_setpoint":
                value = float(msg.get("value", 0.0))
                if not (0.0 <= value <= 80.0):
                    return {"error": f"Hors limites: {value}"}
                self.alicat.set_setpoint(value)
                # Notifier l'UI Qt (thread-safe via signal)
                if self._ui_setpoint_cb is not None:
                    self._ui_setpoint_cb(value)
                return {"status": "ok", "setpoint": value}
            elif cmd == "get_pid":
                pid = self.alicat.get_pid()
                return pid if pid else {"error": "Lecture PID echouee"}
            elif cmd == "set_pid":
                ok = self.alicat.set_pid(
                    P=msg.get("P"), I=msg.get("I"),
                    D=msg.get("D"), loop_type=msg.get("loop_type"))
                return {"status": "ok"} if ok else {"error": "Ecriture PID echouee"}
            else:
                return {"error": f"Commande inconnue: {cmd}"}
        except Exception as e:
            return {"error": str(e)}

    def stop(self):
        self._stop_event.set()
        self.join(timeout=2.0)
        self.socket.close()
        self.context.term()


# ---------------------------------------------------------------------------
# Thread pression
# ---------------------------------------------------------------------------
class THREADPRESSURE(QtCore.QThread):
    MEAS = pyqtSignal(float)

    def __init__(self, parent):
        super().__init__(parent)
        self.parent = parent
        self.stop   = False

    def run(self):
        while not self.stop:
            try:
                self.MEAS.emit(float(self.parent.alicat.get_pressure()))
            except Exception as e:
                print(f"[THREADPRESSURE] {e}")
            time.sleep(0.5)

    def stopThread(self):
        self.stop = True


# ---------------------------------------------------------------------------
# Fenêtre PID
# ---------------------------------------------------------------------------
class PidDialog(QDialog):
    LOOP_TYPES = ['PD/PDF', 'PD2I']

    def __init__(self, alicat: AlicatController, parent=None):
        super().__init__(parent)
        self.alicat = alicat
        self.setWindowTitle("PID Configuration")
        self.setModal(False)
        self.setMinimumWidth(400)
        self.setStyleSheet(STYLE)
        self._build_ui()
        self._load_pid()

    def _build_ui(self):
        layout = QVBoxLayout(self)
        layout.setSpacing(14)
        layout.setContentsMargins(20, 20, 20, 20)

        title = QLabel("PID  CONFIGURATION")
        title.setStyleSheet(
            f"font-family:{MONO_FONT},monospace;font-size:11px;font-weight:bold;"
            "letter-spacing:3px;color:#58a6ff;")
        layout.addWidget(title)

        sep = QFrame(); sep.setObjectName("separator")
        sep.setFrameShape(QFrame.Shape.HLine)
        layout.addWidget(sep)

        grpLoop = QGroupBox("ALGORITHM")
        fl = QFormLayout(grpLoop); fl.setSpacing(10)
        self.comboLoop = QComboBox()
        self.comboLoop.addItems(self.LOOP_TYPES)
        self.comboLoop.currentIndexChanged.connect(self._on_loop_change)
        fl.addRow("Loop type :", self.comboLoop)
        layout.addWidget(grpLoop)

        grpPid = QGroupBox("GAINS")
        fp = QFormLayout(grpPid); fp.setSpacing(10)
        self.spinP = self._spin(); self.spinD = self._spin(); self.spinI = self._spin()
        self.labelI = QLabel("I  —  Integral :")
        fp.addRow("P  —  Proportional :", self.spinP)
        fp.addRow("D  —  Derivative :",   self.spinD)
        fp.addRow(self.labelI,             self.spinI)
        layout.addWidget(grpPid)

        self.statusLabel = QLabel("")
        self.statusLabel.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.statusLabel.setStyleSheet(
            f"font-family:{MONO_FONT},monospace;font-size:10px;color:#484f58;"
            "padding:4px;border-radius:4px;background:#161b22;")
        layout.addWidget(self.statusLabel)

        row = QHBoxLayout(); row.setSpacing(8)
        self.btnRead  = QPushButton("⟳  Read")
        self.btnWrite = QPushButton("✓  Apply"); self.btnWrite.setObjectName("btnSuccess")
        self.btnClose = QPushButton("Close")
        self.btnRead.clicked.connect(self._load_pid)
        self.btnWrite.clicked.connect(self._write_pid)
        self.btnClose.clicked.connect(self.close)
        row.addWidget(self.btnRead); row.addWidget(self.btnWrite); row.addWidget(self.btnClose)
        layout.addLayout(row)

    @staticmethod
    def _spin():
        s = QSpinBox(); s.setRange(0, 9999); s.setSingleStep(1); return s

    def _on_loop_change(self, idx):
        is_pd2i = self.comboLoop.currentText() == 'PD2I'
        self.spinI.setEnabled(is_pd2i)
        self.labelI.setStyleSheet(f"color:{'#c9d1d9' if is_pd2i else '#484f58'};")

    def _load_pid(self):
        self._status("Reading…", "#58a6ff")
        QApplication.processEvents()
        try:
            pid = self.alicat.get_pid()
            if pid is None:
                self._status("Read failed", error=True); return
            lt  = pid.get('loop_type', 'PD/PDF')
            idx = self.LOOP_TYPES.index(lt) if lt in self.LOOP_TYPES else 0
            self.comboLoop.blockSignals(True)
            self.comboLoop.setCurrentIndex(idx)
            self.comboLoop.blockSignals(False)
            self._on_loop_change(idx)
            self.spinP.setValue(int(float(pid.get('P', 0))))
            self.spinD.setValue(int(float(pid.get('D', 0))))
            self.spinI.setValue(int(float(pid.get('I', 0))))
            self._status("Read OK", "#3fb950")
        except Exception as e:
            self._status(f"Error: {e}", error=True)

    def _write_pid(self):
        self._status("Sending…", "#58a6ff")
        QApplication.processEvents()
        try:
            lt = self.comboLoop.currentText()
            ok = self.alicat.set_pid(
                P=self.spinP.value(), D=self.spinD.value(),
                I=self.spinI.value() if lt == 'PD2I' else None,
                loop_type=lt)
            self._status("Applied ✓" if ok else "No confirmation", error=not ok)
        except Exception as e:
            self._status(f"Error: {e}", error=True)

    def _status(self, msg, color="#484f58", error=False):
        c = "#f85149" if error else color
        self.statusLabel.setStyleSheet(
            f"font-family:{MONO_FONT},monospace;font-size:10px;color:{c};"
            "padding:4px;border-radius:4px;background:#161b22;")
        self.statusLabel.setText(msg)


# ---------------------------------------------------------------------------
# Fenêtre Plot — pyqtgraph, deque 5 min
# ---------------------------------------------------------------------------
PLOT_DURATION_S = 300        # 5 minutes
PLOT_INTERVAL_MS = 500       # même cadence que THREADPRESSURE
MAX_POINTS = PLOT_DURATION_S * 1000 // PLOT_INTERVAL_MS   # 600 points

# Configuration pyqtgraph dark
pg.setConfigOption('background', '#0d1117')
pg.setConfigOption('foreground', '#484f58')
pg.setConfigOption('antialias', True)


class PlotWindow(QDialog):
    """
    Fenêtre de tracé pression/setpoint en temps réel.
    Utilise des deque de taille fixe pour limiter à 5 min de données.
    Reçoit les nouvelles valeurs via push_data() appelé depuis AlicatGui.
    """

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setWindowTitle("Pressure Monitor")
        self.setMinimumSize(700, 380)
        self.setStyleSheet(STYLE)

        # ── Buffers circulaires ──────────────────────────────────────────────
        self._t   = deque(maxlen=MAX_POINTS)   # temps relatif (s)
        self._p   = deque(maxlen=MAX_POINTS)   # pression mesurée
        self._sp  = deque(maxlen=MAX_POINTS)   # setpoint
        self._t0  = None                       # timestamp de départ

        self._build_ui()

    def _build_ui(self):
        layout = QVBoxLayout(self)
        layout.setContentsMargins(16, 16, 16, 14)
        layout.setSpacing(10)

        # ── Header ──────────────────────────────────────────────────────────
        hdr = QHBoxLayout()
        title = QLabel("PRESSURE  MONITOR")
        title.setStyleSheet(
            f"font-family:{MONO_FONT},monospace;font-size:11px;font-weight:bold;"
            "letter-spacing:3px;color:#58a6ff;")
        self.lblDuration = QLabel("0 s  /  300 s")
        self.lblDuration.setStyleSheet(
            f"font-family:{MONO_FONT},monospace;font-size:9px;color:#484f58;")
        hdr.addWidget(title)
        hdr.addStretch()
        hdr.addWidget(self.lblDuration)
        layout.addLayout(hdr)

        sep = QFrame(); sep.setObjectName("separator")
        sep.setFrameShape(QFrame.Shape.HLine)
        layout.addWidget(sep)

        # ── PlotWidget ───────────────────────────────────────────────────────
        self.plot_widget = pg.PlotWidget()
        self.plot_widget.setLabel('left',   'Pressure', units='Bar',
                                  color='#484f58', font_size='10pt')
        self.plot_widget.setLabel('bottom', 'Time', units='s',
                                  color='#484f58', font_size='10pt')
        self.plot_widget.showGrid(x=True, y=True, alpha=0.15)
        self.plot_widget.getPlotItem().getAxis('left').setStyle(tickFont=QFont(MONO_FONT, 8))
        self.plot_widget.getPlotItem().getAxis('bottom').setStyle(tickFont=QFont(MONO_FONT, 8))

        # Bordure subtile
        self.plot_widget.setStyleSheet(
            "border: 1px solid #21262d; border-radius: 6px;")

        # Courbes
        self.curve_p = self.plot_widget.plot(
            pen=pg.mkPen(color='#39d353', width=2),
            name='Pressure')
        self.curve_sp = self.plot_widget.plot(
            pen=pg.mkPen(color='#d29922', width=1.5,
                         style=QtCore.Qt.PenStyle.DashLine),
            name='Set Point')

        # Légende
        legend = self.plot_widget.addLegend(
            offset=(10, 10),
            labelTextColor='#c9d1d9')
        legend.setLabelTextSize('9pt')

        layout.addWidget(self.plot_widget)

        # ── Barre infos + bouton clear ───────────────────────────────────────
        bot = QHBoxLayout()
        bot.setSpacing(16)

        self.lblLast = QLabel("P: —  Bar   |   SP: —  Bar")
        self.lblLast.setStyleSheet(
            f"font-family:{MONO_FONT},monospace;font-size:10px;color:#484f58;")

        self.btnClear = QPushButton("⟳  Clear")
        self.btnClear.setFixedWidth(90)
        self.btnClear.clicked.connect(self._clear)

        self.btnClose = QPushButton("Close")
        self.btnClose.setFixedWidth(70)
        self.btnClose.clicked.connect(self.close)

        bot.addWidget(self.lblLast)
        bot.addStretch()
        bot.addWidget(self.btnClear)
        bot.addWidget(self.btnClose)
        layout.addLayout(bot)

    # ── API publique ─────────────────────────────────────────────────────────

    def push_data(self, pressure: float, setpoint: float):
        """Appelé depuis AlicatGui._on_pressure() à chaque mesure."""
        now = time.monotonic()
        if self._t0 is None:
            self._t0 = now
        t_rel = now - self._t0

        self._t.append(t_rel)
        self._p.append(pressure)
        self._sp.append(setpoint)

        # Mise à jour des courbes (numpy-free : conversion liste)
        t_list  = list(self._t)
        p_list  = list(self._p)
        sp_list = list(self._sp)

        self.curve_p.setData(t_list, p_list)
        self.curve_sp.setData(t_list, sp_list)

        # Fenêtre glissante : on fixe le range X sur les 300 dernières secondes
        t_end   = t_list[-1]
        t_start = max(0.0, t_end - PLOT_DURATION_S)
        self.plot_widget.setXRange(t_start, t_end + 1, padding=0)

        # Infos
        elapsed = min(int(t_end), PLOT_DURATION_S)
        self.lblDuration.setText(f"{elapsed} s  /  {PLOT_DURATION_S} s")
        self.lblLast.setText(
            f"P:  <span style='color:#39d353;font-weight:bold;'>{pressure:.2f}</span>  Bar"
            f"   |   SP:  <span style='color:#d29922;font-weight:bold;'>{setpoint:.2f}</span>  Bar")
        self.lblLast.setTextFormat(QtCore.Qt.TextFormat.RichText)

    def _clear(self):
        self._t.clear(); self._p.clear(); self._sp.clear()
        self._t0 = None
        self.curve_p.setData([], [])
        self.curve_sp.setData([], [])
        self.lblDuration.setText(f"0 s  /  {PLOT_DURATION_S} s")
        self.lblLast.setText("P: —  Bar   |   SP: —  Bar")


# ---------------------------------------------------------------------------
# Interface principale
# ---------------------------------------------------------------------------
class AlicatGui(QMainWindow):

    _zmq_setpoint_signal = QtCore.pyqtSignal(float)

    def __init__(self, port='com1', zmq_port=5560,
                 name='Étoile Gas Control', parent=None):
        super().__init__(parent)
        p = pathlib.Path(__file__)
        sepa = os.sep
        self.port      = port
        self.zmq_port  = zmq_port
        self.local_ip  = get_local_ip()
        self.icon      = str(p.parent) + sepa + 'icons' + sepa
        self.setWindowIcon(QIcon(self.icon + 'LOA.png'))
        self._pid_dialog  = None
        self._plot_dialog = None

        self.setStyleSheet(STYLE)
        self._build_ui()
        self._connect_signals()

        # --- Alicat ---
        self.alicat = AlicatController(port=self.port)
        self.alicat.connect()
        device_info = self.alicat.get_device_info()
        sn = device_info['serial_number'] if device_info else 'N/A'

        self.setWindowTitle(f"{name}  —  ZMQ  tcp://{self.local_ip}:{zmq_port}")
        self.labelSN.setText(f"S/N  {sn}")
        self.labelZmq.setText(f"tcp://{self.local_ip}:{self.zmq_port}")

        # --- Lecture du setpoint initial depuis le contrôleur ---
        try:
            readings = self.alicat.get_readings()
            if readings and 'setpoint' in readings:
                sp = float(readings['setpoint'])
                self.setValueSpinBox.setValue(sp)
                self.gauge.setSetpoint(sp)
        except Exception as e:
            print(f"[Init] Lecture setpoint échouée: {e}")

        # --- Thread pression ---
        self.threadPressure = THREADPRESSURE(self)
        self.threadPressure.start()
        self.threadPressure.MEAS.connect(self._on_pressure)

        # --- ZMQ ---
        self.zmq_server = ZmqServer(self.alicat, port=self.zmq_port,
                                    ui_setpoint_cb=self._zmq_setpoint_signal.emit)
        self.zmq_server.start()
        print(f"[ZmqServer]  tcp://{self.local_ip}:{self.zmq_port}")

        self._zmq_setpoint_signal.connect(self._apply_setpoint_from_zmq)
        self.statusBar().showMessage(
            f"ZMQ  ●  tcp://{self.local_ip}:{self.zmq_port}   |   "
            f"Device  {sn}   |   Port  {self.port}")

    # ── Construction UI ──────────────────────────────────────────────────────

    def _build_ui(self):
        central = QWidget()
        root    = QVBoxLayout()
        root.setContentsMargins(20, 18, 20, 14)
        root.setSpacing(14)
        central.setLayout(root)
        self.setCentralWidget(central)

        # ── Header ──────────────────────────────────────────────────────────
        hdr = QHBoxLayout()
        lbl_title = QLabel("ALICAT  PCD")
        lbl_title.setStyleSheet(
            f"font-family:{MONO_FONT},monospace;font-size:15px;font-weight:bold;"
            "letter-spacing:4px;color:#e6edf3;")
        self.ledLabel = QLabel("●  CONNECTED")
        self.ledLabel.setStyleSheet(
            f"font-family:{MONO_FONT},monospace;font-size:9px;"
            "letter-spacing:2px;color:#3fb950;")
        hdr.addWidget(lbl_title)
        hdr.addStretch()
        hdr.addWidget(self.ledLabel)
        root.addLayout(hdr)

        root.addWidget(self._sep())

        # ── Corps : gauge (gauche) + panneau (droite), côte à côte ──────────
        body = QHBoxLayout()
        body.setSpacing(24)
        body.setAlignment(Qt.AlignmentFlag.AlignTop)

        # -- Gauge (gauche, centrée verticalement) --
        gauge_col = QVBoxLayout()
        gauge_col.setAlignment(Qt.AlignmentFlag.AlignVCenter)
        self.gauge = PressureGauge(max_val=80)
        gauge_col.addWidget(self.gauge)

        hint = QLabel("drag to set")
        hint.setAlignment(Qt.AlignmentFlag.AlignCenter)
        hint.setStyleSheet(
            f"font-family:{MONO_FONT},monospace;font-size:8px;"
            "letter-spacing:1px;color:#30363d;")
        gauge_col.addWidget(hint)

        body.addLayout(gauge_col)

        # -- Panneau droite --
        right = QVBoxLayout()
        right.setSpacing(12)
        right.setAlignment(Qt.AlignmentFlag.AlignTop)

        # Pression mesurée
        grpMeas = QGroupBox("MEASURED PRESSURE")
        measLayout = QVBoxLayout(grpMeas)
        measLayout.setSpacing(2)
        measLayout.setContentsMargins(6, 6, 6, 10)

        self.measLabel = QDoubleSpinBox()
        self.measLabel.setDecimals(2)
        self.measLabel.setRange(0, 80)
        self.measLabel.setSuffix('  Bar')
        self.measLabel.setReadOnly(True)
        self.measLabel.setButtonSymbols(QDoubleSpinBox.ButtonSymbols.NoButtons)
        self.measLabel.setMinimumHeight(70)
        self.measLabel.setLocale(QtCore.QLocale(QtCore.QLocale.Language.English))
        self.measLabel.setStyleSheet(
            f"font-size:28px; font-family:'{MONO_FONT}'; color:#39d353;"
            "border: 1px solid #30363d; border-radius:6px; background:#161b22;"
            "padding: 4px 8px;")

        measLayout.addWidget(self.measLabel)
        right.addWidget(grpMeas)

        # Set point
        grpSP = QGroupBox("SET POINT")
        spLayout = QHBoxLayout(grpSP)
        spLayout.setSpacing(8)

        self.setValueSpinBox = QDoubleSpinBox()
        self.setValueSpinBox.setDecimals(1)
        self.setValueSpinBox.setSingleStep(0.5)
        self.setValueSpinBox.setRange(0, 80)
        self.setValueSpinBox.setSuffix('  Bar')
        self.setValueSpinBox.setStyleSheet(
            f"font-size:15px;font-family:'{MONO_FONT}';color:#d29922;")

        self.btnApplySP = QPushButton("Set")
        self.btnApplySP.setObjectName("btnPrimary")
        self.btnApplySP.setFixedWidth(52)

        spLayout.addWidget(self.setValueSpinBox)
        spLayout.addWidget(self.btnApplySP)
        right.addWidget(grpSP)

        # Bouton PID
        self.btnPid = QPushButton("⚙   PID Configuration")
        self.btnPid.setObjectName("btnPid")
        right.addWidget(self.btnPid)

        # Bouton Plot
        self.btnPlot = QPushButton("📈   Plot")
        self.btnPlot.setObjectName("btnPid")   # même style bleu
        right.addWidget(self.btnPlot)

        body.addLayout(right)
        root.addLayout(body)

        root.addWidget(self._sep())

        # ── Footer ──────────────────────────────────────────────────────────
        footer = QHBoxLayout()
        self.labelSN = QLabel("S/N  —")
        self.labelSN.setStyleSheet(
            f"font-family:{MONO_FONT},monospace;font-size:9px;"
            "letter-spacing:1px;color:#30363d;")
        self.labelZmq = QLabel("—")
        self.labelZmq.setStyleSheet(
            f"font-family:{MONO_FONT},monospace;font-size:9px;"
            "letter-spacing:1px;color:#30363d;")
        self.labelZmq.setTextInteractionFlags(
            Qt.TextInteractionFlag.TextSelectableByMouse)
        footer.addWidget(self.labelSN)
        footer.addStretch()
        footer.addWidget(self.labelZmq)
        root.addLayout(footer)

    def _sep(self) -> QFrame:
        f = QFrame(); f.setObjectName("separator")
        f.setFrameShape(QFrame.Shape.HLine)
        return f

    def _connect_signals(self):
        # Spinbox → gauge (preview visuel)
        self.setValueSpinBox.valueChanged.connect(
            lambda v: self.gauge.setSetpoint(v))
        # Bouton Set → envoyer à l'Alicat
        self.btnApplySP.clicked.connect(self._send_setpoint)
        # Drag sur la gauge → spinbox (sans envoyer)
        self.gauge.setpoint_changed.connect(self._on_gauge_drag)
        # Bouton PID
        self.btnPid.clicked.connect(self._open_pid)
        # Bouton Plot
        self.btnPlot.clicked.connect(self._open_plot)

    # ── Slots ────────────────────────────────────────────────────────────────

    def _on_gauge_drag(self, v: float):
        """Drag gauge → met à jour le spinbox (sans envoyer à l'Alicat)."""
        self.setValueSpinBox.blockSignals(True)
        self.setValueSpinBox.setValue(v)
        self.setValueSpinBox.blockSignals(False)
        self.gauge.setSetpoint(v)   # déjà fait dans gauge mais on force la cohérence

    def _send_setpoint(self):
        v = self.setValueSpinBox.value()
        self.alicat.set_setpoint(v)
        self.gauge.setSetpoint(v)

    def _apply_setpoint_from_zmq(self, value: float):
        self.setValueSpinBox.setValue(value)
        self.gauge.setSetpoint(value)
        self.alicat.set_setpoint(value)

    def _on_pressure(self, pressure: float):
        self.gauge.setValue(pressure)
        if pressure > 68:
            col = "#f85149"
        elif pressure > 56:
            col = "#d29922"
        else:
            col = "#39d353"
        self.measLabel.setStyleSheet(
            f"font-size:28px; font-family:'{MONO_FONT}'; color:{col};"
            "border: 1px solid #30363d; border-radius:6px; background:#161b22;"
            "padding: 4px 8px;")
        self.measLabel.setValue(pressure)

        # Alimenter le plot si ouvert
        if self._plot_dialog is not None and self._plot_dialog.isVisible():
            self._plot_dialog.push_data(pressure, self.setValueSpinBox.value())

    def _open_plot(self):
        if self._plot_dialog is None or not self._plot_dialog.isVisible():
            self._plot_dialog = PlotWindow(parent=self)
            self._plot_dialog.show()
        else:
            self._plot_dialog.raise_()
            self._plot_dialog.activateWindow()

    def _open_pid(self):
        if self._pid_dialog is None or not self._pid_dialog.isVisible():
            self._pid_dialog = PidDialog(self.alicat, parent=self)
            self._pid_dialog.show()
        else:
            self._pid_dialog.raise_()
            self._pid_dialog.activateWindow()

    def closeEvent(self, event):
        if self._pid_dialog:
            self._pid_dialog.close()
        if self._plot_dialog:
            self._plot_dialog.close()
        self.threadPressure.stopThread()
        self.threadPressure.wait(1000)
        self.zmq_server.stop()
        self.alicat.disconnect()
        time.sleep(0.5)
        event.accept()


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------
if __name__ == '__main__':
    appli = QApplication(sys.argv)

    STYLE = build_style(MONO_FONT, SANS_FONT)

    e = AlicatGui(port='/dev/ttyUSB0', zmq_port=5560)
    e.setMinimumSize(560, 360)
    e.show()
    appli.exec()