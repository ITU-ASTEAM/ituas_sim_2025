"""
Pixhawk / RFD - Mission Planner UDP mirror uzerinden anlik telemetri gorsellestirme.
Kartlar: Hiz, Heading/Yaw, Thruster kuvvet istegi, Pusula (yaw gostergesi),
Gemi Egimi (pitch/roll attitude indicator), GPS Harita. Sagda acilir/kapanir
gorunum ayar paneli - hangi kartin gosterilecegi buradan secilir.

Kurulum:
    pip install pymavlink pyqtgraph PyQt5 PyQtWebEngine

Mission Planner tarafinda (Ctrl+F > Mavlink script):
    Type: UDP
    Direction: Outbound
    Host/Baud: 127.0.0.1
    Port: 14550
    Write: isaretli, Go: basili

Kullanim:
    python mavlink_live_plot_v2.py
"""

import json
import math
import os
import subprocess
import sys
from collections import deque

from pymavlink import mavutil
from pyqtgraph.Qt import QtWidgets, QtCore, QtGui
import pyqtgraph as pg

try:
    from PyQt5.QtWebEngineWidgets import QWebEngineView
    HAS_WEBENGINE = True
except ImportError:
    HAS_WEBENGINE = False

# ---- Ayarlar ----
CONNECTION_STRING = "udpin:127.0.0.1:14550"
CONNECTION_FALLBACKS = ["udp:127.0.0.1:14550", "udpin:127.0.0.1:14551", "udp:127.0.0.1:14551"]
HISTORY_LEN = 300
UPDATE_MS = 50
# Aktif thruster kanallari. Sadece burada listelenenler icin veri toplanir,
# grafik cizilir ve kenar cubugunda kutucuk olusturulur.
THRUSTER_CHANNELS = [1, 3]
THRUSTER_HUES = 8  # thruster egri renkleri icin sabit renk cemberi
PWM_NEUTRAL = 1500
PWM_RANGE = 400
MIN_SPEED_SPAN = 1.0

# Dashboard yerlesimi (surukle-birak ile olusturulan kartlar) buraya kaydedilir
# ve program acilisinda otomatik geri yuklenir.
try:
    _LAYOUT_DIR = os.path.dirname(os.path.abspath(__file__))
except Exception:
    _LAYOUT_DIR = os.getcwd()
LAYOUT_FILE = os.path.join(_LAYOUT_DIR, "dashboard_layout.json")

# ---- Tema renkleri ----
BG_WINDOW = "#0b0d12"
BG_CARD = "#14161c"
BORDER_CARD = "#262a33"
ACCENT_SPEED = "#22d3ee"
ACCENT_YAW = "#4ade80"
ACCENT_THRUSTER = "#f87171"
ACCENT_GAUGE = "#60a5fa"
ACCENT_ATTITUDE = "#a78bfa"
ACCENT_MAP = "#fbbf24"
ACCENT_SETTINGS = "#9ca3af"
TEXT_MAIN = "#c9d1d9"
TEXT_DIM = "#7d8590"

pg.setConfigOption("background", BG_CARD)
pg.setConfigOption("foreground", TEXT_MAIN)
pg.setConfigOptions(antialias=True)


class DragListWidget(QtWidgets.QListWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setDragEnabled(True)
        self.setSelectionMode(QtWidgets.QAbstractItemView.SingleSelection)
        # Alternatif satir renkleri KAPALI: acik/beyaz zemin uzerine acik gri
        # metin gorunmez oluyordu. Zemin/metin renklerini stylesheet ile netlestiriyoruz.
        self.setAlternatingRowColors(False)
        # only allow dragging items out to canvas; prevent internal reordering
        self.setDragDropMode(QtWidgets.QAbstractItemView.DragOnly)
        self.setMovement(QtWidgets.QListView.Static)
        self.setMouseTracking(True)
        # Tum kutucuklar alt alta gorunsun diye ic scrollbar kapali (yukseklik
        # fit_to_contents ile icerige gore ayarlanir).
        self.setVerticalScrollBarPolicy(QtCore.Qt.ScrollBarAlwaysOff)
        self.setHorizontalScrollBarPolicy(QtCore.Qt.ScrollBarAlwaysOff)
        self.setSizePolicy(QtWidgets.QSizePolicy.Preferred, QtWidgets.QSizePolicy.Fixed)
        self._press_pos = None

        self.setStyleSheet(f"""
            QListWidget {{
                background: {BG_CARD};
                color: {TEXT_MAIN};
                border: 1px solid {BORDER_CARD};
                border-radius: 10px;
                padding: 4px;
                outline: none;
            }}
            QListWidget::item {{
                padding: 8px;
                margin: 2px;
                color: {TEXT_MAIN};
                background: #1b1e26;
                border: 1px solid {BORDER_CARD};
                border-radius: 6px;
            }}
            QListWidget::item:selected {{
                background-color: {ACCENT_SPEED};
                color: #071223;
                font-weight: 700;
                border: 1px solid {ACCENT_SPEED};
            }}
            QListWidget::item:disabled {{
                color: {TEXT_DIM};
                background: transparent;
                border: none;
                font-weight: 700;
            }}
        """)

    def fit_to_contents(self):
        """Listeyi icindeki tum satirlari kaydirmadan gosterecek yukseklige ayarla."""
        n = self.count()
        if n == 0:
            self.setFixedHeight(40)
            return
        row_h = self.sizeHintForRow(0)
        if row_h <= 0:
            row_h = 38
        # her satir: row_h + item margin (2+2) + border (1+1) ~= row_h + 6
        total = n * (row_h + 6) + 2 * self.frameWidth() + 10
        self.setFixedHeight(total)

    def startDrag(self, supportedActions):
        item = self.currentItem()
        if item is None:
            return
        try:
            print(f"[DragListWidget] startDrag called, item='{item.text()}'")
        except Exception:
            pass
        payload = item.data(QtCore.Qt.UserRole)
        if payload is None:
            payload = item.text()
        mime = QtCore.QMimeData()
        mime.setText(payload)
        drag = QtGui.QDrag(self)
        drag.setMimeData(mime)
        pixmap = QtGui.QPixmap(220, 48)
        pixmap.fill(QtCore.Qt.transparent)
        painter = QtGui.QPainter(pixmap)
        painter.setRenderHint(QtGui.QPainter.Antialiasing)
        rect = QtCore.QRectF(2, 2, 216, 44)
        painter.setBrush(QtGui.QColor(ACCENT_GAUGE))
        painter.setPen(QtCore.Qt.NoPen)
        painter.drawRoundedRect(rect, 10, 10)
        painter.setPen(QtGui.QColor("#ffffff"))
        painter.setFont(QtGui.QFont("Segoe UI", 10, QtGui.QFont.Bold))
        painter.drawText(rect, QtCore.Qt.AlignCenter, item.text())
        painter.end()
        drag.setPixmap(pixmap)
        # Force move action and report
        try:
            res = drag.exec_(QtCore.Qt.MoveAction)
            print(f"[DragListWidget] drag.exec_ result: {res}")
        except Exception as exc:
            print(f"[DragListWidget] drag.exec_ exception: {exc}")

    def mousePressEvent(self, event):
        if event.button() == QtCore.Qt.LeftButton:
            self._press_pos = event.pos()
            try:
                print(f"[DragListWidget] mousePressEvent at {self._press_pos}")
            except Exception:
                pass
        return super().mousePressEvent(event)

    def mouseMoveEvent(self, event):
        if self._press_pos is None:
            return super().mouseMoveEvent(event)
        dist = (event.pos() - self._press_pos).manhattanLength()
        try:
            print(f"[DragListWidget] mouseMoveEvent dist={dist} threshold={QtWidgets.QApplication.startDragDistance()}")
        except Exception:
            pass
        if dist >= QtWidgets.QApplication.startDragDistance():
            # start drag manually
            try:
                print("[DragListWidget] initiating startDrag due to distance")
            except Exception:
                pass
            self.startDrag(QtCore.Qt.MoveAction)
            self._press_pos = None
            return
        return super().mouseMoveEvent(event)

    def mouseReleaseEvent(self, event):
        self._press_pos = None
        return super().mouseReleaseEvent(event)


class DashboardCanvas(QtWidgets.QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setAcceptDrops(True)
        self.setMinimumHeight(320)
        self.setStyleSheet(f"""
            QWidget {{ background: qlineargradient(x1:0 y1:0, x2:1 y2:1, stop:0 {BG_WINDOW}, stop:1 {BG_CARD}); }}
        """)
        self.dashboard_items = []
        self.do_action_button = None

    def paintEvent(self, event):
        super().paintEvent(event)
        painter = QtGui.QPainter(self)
        painter.setPen(QtGui.QPen(QtGui.QColor("#2b3140"), 1))
        step = 24
        for x in range(0, self.width(), step):
            painter.drawLine(x, 0, x, self.height())
        for y in range(0, self.height(), step):
            painter.drawLine(0, y, self.width(), y)
        painter.end()

    def dragEnterEvent(self, event):
        if event.mimeData().hasText():
            event.acceptProposedAction()

    def dropEvent(self, event):
        payload = event.mimeData().text()
        pos = event.pos()
        if payload.startswith("telemetry:"):
            key = payload.split(":", 1)[1]
            self.window().create_dashboard_widget("telemetry", key, pos)
        elif payload.startswith("control:"):
            control_type = payload.split(":", 1)[1]
            self.window().create_dashboard_widget("control", control_type, pos)
        event.acceptProposedAction()

    def mousePressEvent(self, event):
        # Right-click on empty canvas no longer shows DoAction button.
        # Keep default behaviour so users can use canvas context menu in future.
        super().mousePressEvent(event)

    # Removed inline DoAction button that appeared on right-click. DoAction
    # is now available from the sidebar as a draggable control.


class DashboardItemBase(QtWidgets.QFrame):
    def __init__(self, parent, title):
        super().__init__(parent)
        self.setFrameShape(QtWidgets.QFrame.StyledPanel)
        self.setLineWidth(1)
        self.setStyleSheet(f"""
            QFrame {{
                background: rgba(20, 22, 28, 220);
                border: 1px solid {BORDER_CARD};
                border-radius: 12px;
            }}
        """)
        self._drag_mode = None
        self._start_geometry = None
        self._start_mouse = None
        self.title = title

        layout = QtWidgets.QVBoxLayout(self)
        layout.setContentsMargins(10, 8, 10, 10)
        layout.setSpacing(6)

        header = QtWidgets.QHBoxLayout()
        header.setContentsMargins(0, 0, 0, 0)
        title_label = QtWidgets.QLabel(title)
        title_label.setStyleSheet(f"color: {ACCENT_GAUGE}; font-weight: 700;")
        header.addWidget(title_label)
        header.addStretch(1)
        header.setSpacing(4)
        layout.addLayout(header)
        self.body_layout = QtWidgets.QVBoxLayout()
        self.body_layout.setContentsMargins(0, 0, 0, 0)
        self.body_layout.setSpacing(6)
        layout.addLayout(self.body_layout)
        # add a size grip so users can resize the widget from bottom-right reliably
        self._size_grip = QtWidgets.QSizeGrip(self)
        self._size_grip.setFixedSize(14, 14)
        self._size_grip.setStyleSheet("QSizeGrip{width:14px;height:14px;background:transparent}")
        self._size_grip.raise_()
        self.setMouseTracking(True)

    def mousePressEvent(self, event):
        if event.button() == QtCore.Qt.LeftButton:
            pos = event.pos()
            # allow resizing from right/bottom edges and bottom-right corner
            margin = 12
            near_right = pos.x() >= self.width() - margin
            near_bottom = pos.y() >= self.height() - margin
            if near_right and near_bottom:
                self._drag_mode = "resize_both"
            elif near_right:
                self._drag_mode = "resize_h"
            elif near_bottom:
                self._drag_mode = "resize_v"
            else:
                self._drag_mode = "move"
            self._start_geometry = self.geometry()
            self._start_mouse = event.globalPos()
            self.raise_()
            event.accept()
            return
        super().mousePressEvent(event)

    def mouseMoveEvent(self, event):
        # Guard: bir tasima/boyutlandirma islemi baslamis ama sol tus artik
        # basili degilse (release event'i cocuk PlotWidget'a gidip kaybolmus
        # olabilir), islemi iptal et. Aksi halde mouseTracking acik oldugu icin
        # widget mouse'u takip etmeye devam eder ve "takili kalma" hatasi olusur.
        if self._drag_mode is not None and not (event.buttons() & QtCore.Qt.LeftButton):
            self._drag_mode = None
            self._start_geometry = None
            self._start_mouse = None
            self.setCursor(QtCore.Qt.ArrowCursor)
            event.accept()
            return
        # if not dragging, update cursor if hovering near edges
        if self._drag_mode is None or self._start_geometry is None or self._start_mouse is None:
            pos = event.pos()
            margin = 12
            if pos.x() >= self.width() - margin and pos.y() >= self.height() - margin:
                self.setCursor(QtCore.Qt.SizeFDiagCursor)
            elif pos.x() >= self.width() - margin:
                self.setCursor(QtCore.Qt.SizeHorCursor)
            elif pos.y() >= self.height() - margin:
                self.setCursor(QtCore.Qt.SizeVerCursor)
            else:
                self.setCursor(QtCore.Qt.ArrowCursor)
            return
        delta = event.globalPos() - self._start_mouse
        if self._drag_mode == "move":
            new_x = self._start_geometry.x() + delta.x()
            new_y = self._start_geometry.y() + delta.y()
            new_rect = QtCore.QRect(new_x, new_y, self._start_geometry.width(), self._start_geometry.height())
        else:
            new_w = self._start_geometry.width()
            new_h = self._start_geometry.height()
            new_x = self._start_geometry.x()
            new_y = self._start_geometry.y()
            if self._drag_mode in ("resize_both", "resize_h"):
                new_w = max(180, self._start_geometry.width() + delta.x())
            if self._drag_mode in ("resize_both", "resize_v"):
                new_h = max(120, self._start_geometry.height() + delta.y())
            new_rect = QtCore.QRect(new_x, new_y, new_w, new_h)
        parent_rect = self.parent().rect()
        if new_rect.right() > parent_rect.right():
            new_rect.setRight(parent_rect.right())
        if new_rect.bottom() > parent_rect.bottom():
            new_rect.setBottom(parent_rect.bottom())
        if new_rect.x() < 0:
            new_rect.setX(0)
        if new_rect.y() < 0:
            new_rect.setY(0)
        self.setGeometry(new_rect)
        event.accept()

    def mouseReleaseEvent(self, event):
        self._drag_mode = None
        self._start_geometry = None
        self._start_mouse = None
        super().mouseReleaseEvent(event)

    def resizeEvent(self, event):
        super().resizeEvent(event)
        try:
            self._size_grip.move(self.width() - self._size_grip.width() - 2, self.height() - self._size_grip.height() - 2)
        except Exception:
            pass

    def contextMenuEvent(self, event):
        menu = QtWidgets.QMenu(self)
        delete_action = menu.addAction("Sil")
        # telemetry widgets: allow toggling graphical/text mode
        toggle_graph_action = None
        if isinstance(self, DashboardTelemetryWidget):
            if self.graphical_view:
                toggle_graph_action = menu.addAction("Metin göster")
            else:
                toggle_graph_action = menu.addAction("Grafiksel göster")

        action = menu.exec_(self.mapToGlobal(event.pos()))
        if action == delete_action:
            self.deleteLater()
            if hasattr(self.parent(), "dashboard_items"):
                self.parent().dashboard_items = [item for item in self.parent().dashboard_items if item is not self]
        elif action == toggle_graph_action and toggle_graph_action is not None:
            # toggle graphical view for telemetry widgets immediately
            if isinstance(self, DashboardTelemetryWidget):
                try:
                    self.toggle_graphical()
                except Exception as exc:
                    try:
                        print(f"[DashboardItemBase] toggle_graphical exception: {exc}")
                    except Exception:
                        pass


class DashboardTelemetryWidget(DashboardItemBase):
    def __init__(self, parent, title, data_key, value_getter):
        super().__init__(parent, title)
        self.data_key = data_key
        self.value_getter = value_getter
        self.value_label = QtWidgets.QLabel("--")
        self.value_label.setStyleSheet(f"color: {TEXT_MAIN}; font-size: 22px; font-weight: 600;")
        self.body_layout.addWidget(self.value_label)
        self.graphical_view = False
        self._graph = None
        self.resize(220, 120)

    def set_value(self, value):
        if value is None:
            text = "--"
        else:
            if isinstance(value, float):
                text = f"{value:.2f}"
            else:
                text = str(value)
        self.value_label.setText(text)

    def update_from_latest(self, latest):
        if not self.graphical_view:
            self.set_value(self.value_getter(latest))
        else:
            # update small graph from global buffer available on window
            win = self.window()
            if not hasattr(win, "buf"):
                return
            buf = win.buf
            key = self.data_key
            t = list(buf.t)
            if key == "actual_speed":
                y = list(buf.actual_speed)
            elif key == "setpoint_speed":
                y = list(buf.setpoint_speed)
            elif key == "actual_yaw":
                y = list(buf.actual_yaw)
            elif key == "setpoint_yaw":
                y = list(buf.setpoint_yaw)
            elif key.startswith("thruster_pwm"):
                ch = int(key.split("_")[-1])
                y = list(buf.thruster.get(ch, []))
            else:
                # fallback to value_getter time series not available
                val = self.value_getter(latest)
                self.set_value(val)
                return

            if self._graph is None:
                self._graph = pg.PlotWidget(parent=self)
                self._graph.setBackground(None)
                self._graph.setMinimumHeight(80)
                self._graph.setMouseEnabled(x=False, y=False)
                # clear current body and add graph
                for i in reversed(range(self.body_layout.count())):
                    w = self.body_layout.itemAt(i).widget()
                    if w:
                        w.setParent(None)
                self.body_layout.addWidget(self._graph)

            try:
                self._graph.plot(list(t), list(y), clear=True, pen=pg.mkPen(ACCENT_SPEED, width=1))
            except Exception:
                pass

    def toggle_graphical(self):
        """Toggle graphical view immediately and (re)create or remove the internal graph widget."""
        self.graphical_view = not self.graphical_view
        try:
            print(f"[DashboardTelemetryWidget] toggle_graphical -> {self.graphical_view} for {self.title}")
        except Exception:
            pass
        if self.graphical_view:
            # remove existing body children and create graph immediately
            for i in reversed(range(self.body_layout.count())):
                w = self.body_layout.itemAt(i).widget()
                if w:
                    w.setParent(None)
            self._graph = pg.PlotWidget(parent=self)
            self._graph.setBackground(None)
            self._graph.setMinimumHeight(80)
            self._graph.setMouseEnabled(x=False, y=False)
            self.body_layout.addWidget(self._graph)
            # populate immediately from available buffer if possible
            try:
                win = self.window()
                if hasattr(win, 'buf'):
                    # force an immediate update to show data
                    self.update_from_latest(win.latest)
            except Exception:
                pass
        else:
            # switching back to text mode
            if self._graph is not None:
                self._graph.setParent(None)
                self._graph = None
            # restore label
            for i in reversed(range(self.body_layout.count())):
                w = self.body_layout.itemAt(i).widget()
                if w:
                    w.setParent(None)
            self.body_layout.addWidget(self.value_label)


class DashboardControlWidget(DashboardItemBase):
    def __init__(self, parent, master):
        super().__init__(parent, "Kontrol")
        self.master = master
        self.arm_button = QtWidgets.QPushButton("ARM")
        self.arm_button.setStyleSheet(f"""
            QPushButton {{
                background-color: #16a34a; color: white; border-radius: 8px; padding: 8px 10px;
            }}
        """)
        self.arm_button.clicked.connect(lambda: self._send_arm(True))
        self.body_layout.addWidget(self.arm_button)

        self.disarm_button = QtWidgets.QPushButton("DISARM")
        self.disarm_button.setStyleSheet(f"""
            QPushButton {{
                background-color: #dc2626; color: white; border-radius: 8px; padding: 8px 10px;
            }}
        """)
        self.disarm_button.clicked.connect(lambda: self._send_arm(False))
        self.body_layout.addWidget(self.disarm_button)

        self.mode_combo = QtWidgets.QComboBox()
        self.mode_combo.addItems(["MANUAL", "AUTO"])
        # activated: kullanici bir secim yaptiginda (ayni deger olsa bile) tetiklenir.
        # currentTextChanged sadece deger degisince tetiklendigi icin, arac zaten
        # o moddayken tekrar secmek ise yaramiyordu.
        self.mode_combo.activated.connect(self._on_mode_selected)
        self.body_layout.addWidget(self.mode_combo)
        self.resize(220, 160)

    def _send_arm(self, arm):
        if self.master is None:
            return
        try:
            self.master.mav.command_long_send(
                self.master.target_system,
                self.master.target_component,
                mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                0,
                1 if arm else 0,
                0, 0, 0, 0, 0, 0,
            )
        except Exception as exc:
            QtWidgets.QMessageBox.warning(self, "ARM/DISARM Hatası", str(exc))

    def _on_mode_selected(self, _index):
        self._send_mode(self.mode_combo.currentText())

    def _send_mode(self, mode_name):
        if self.master is None:
            return
        mode_name = (mode_name or "").upper()
        # ArduRover mod numaralari (mode_mapping alinamazsa yedek olarak)
        ROVER_MODES = {
            "MANUAL": 0, "ACRO": 1, "STEERING": 3, "HOLD": 4, "LOITER": 5,
            "FOLLOW": 6, "SIMPLE": 7, "AUTO": 10, "RTL": 11, "SMART_RTL": 12,
            "GUIDED": 15,
        }
        mode_id = None
        # Once araca ozel mod haritasini dene (arac tipine gore dogru numarayi verir)
        try:
            mode_map = self.master.mode_mapping()
        except Exception:
            mode_map = None
        if mode_map and mode_name in mode_map:
            mode_id = mode_map[mode_name]
        else:
            mode_id = ROVER_MODES.get(mode_name)

        if mode_id is None:
            QtWidgets.QMessageBox.warning(self, "Mode", f"Bilinmeyen mod: {mode_name}")
            return

        try:
            base_mode = mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED
            # Iki yontemi de gonderiyoruz; ArduPilot her ikisini de kabul eder,
            # boylece farkli surumlerde mod degisimi garanti calisir.
            self.master.mav.set_mode_send(
                self.master.target_system,
                base_mode,
                mode_id,
            )
            self.master.mav.command_long_send(
                self.master.target_system,
                self.master.target_component,
                mavutil.mavlink.MAV_CMD_DO_SET_MODE,
                0,
                base_mode,
                mode_id,
                0, 0, 0, 0, 0,
            )
        except Exception as exc:
            QtWidgets.QMessageBox.warning(self, "Mode Hatası", str(exc))


class DashboardActionWidget(DashboardItemBase):
    """A draggable action widget which lets the user choose a python file and run it."""
    def __init__(self, parent, title="DoAction"):
        super().__init__(parent, title)
        self.file_label = QtWidgets.QLabel("Dosya: --")
        self.file_label.setStyleSheet(f"color: {TEXT_MAIN};")
        self.body_layout.addWidget(self.file_label)

        btn_row = QtWidgets.QHBoxLayout()
        self.select_btn = QtWidgets.QPushButton("Seç")
        self.select_btn.clicked.connect(self._select_file)
        btn_row.addWidget(self.select_btn)
        self.run_btn = QtWidgets.QPushButton("Çalıştır")
        self.run_btn.clicked.connect(self._run_file)
        btn_row.addWidget(self.run_btn)
        self.body_layout.addLayout(btn_row)
        self._path = None
        self.resize(260, 120)

    def _select_file(self):
        file_path, _ = QtWidgets.QFileDialog.getOpenFileName(self, "Python dosyası seç", os.getcwd(), "Python dosyaları (*.py)")
        if file_path:
            self._path = file_path
            self.file_label.setText(f"Dosya: {os.path.basename(file_path)}")

    def _run_file(self):
        if not self._path:
            QtWidgets.QMessageBox.information(self, "Seçim", "Önce bir dosya seçin.")
            return
        try:
            subprocess.Popen([sys.executable, self._path], shell=False)
        except Exception as exc:
            QtWidgets.QMessageBox.critical(self, "Hata", f"Dosya çalıştırılamadı: {exc}")


class DashboardVisualWidget(DashboardItemBase):
    """Canli grafik/gosterge barindiran kart: pusula (yaw gauge), gemi egimi
    (attitude) veya GPS harita. Bu widgetlar bilendigi gibi dogrudan grafik
    olarak acilir; metin/grafik gecisi (sag tik) YOKTUR, hep gorseldir."""

    def __init__(self, parent, title, kind):
        super().__init__(parent, title)
        self.kind = kind  # "gauge" | "attitude" | "gps"
        if kind == "gauge":
            self.visual = YawGaugeWidget()
            self.resize(280, 300)
        elif kind == "attitude":
            self.visual = AttitudeIndicatorWidget()
            self.resize(280, 300)
        elif kind == "gps":
            self.visual = GpsMapWidget()
            self.resize(380, 340)
        else:
            self.visual = QtWidgets.QLabel("--")
            self.resize(260, 160)
        self.body_layout.addWidget(self.visual)

    def update_from_latest(self, latest):
        try:
            if self.kind == "gauge":
                self.visual.set_values(latest.actual_yaw, latest.setpoint_yaw)
            elif self.kind == "attitude":
                self.visual.set_values(latest.actual_pitch, latest.actual_roll, latest.actual_yaw)
            elif self.kind == "gps":
                self.visual.update_position(latest.lat, latest.lon, latest.actual_yaw)
        except Exception:
            pass


class CollapsibleSidebar(QtWidgets.QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setMinimumWidth(72)
        self.setMaximumWidth(320)
        self.setStyleSheet(f"""
            QWidget {{ background: {BG_CARD}; border: 1px solid {BORDER_CARD}; border-radius: 12px; }}
        """)
        self._collapsed = False
        layout = QtWidgets.QVBoxLayout(self)
        layout.setContentsMargins(8, 8, 8, 8)
        layout.setSpacing(8)

        header = QtWidgets.QHBoxLayout()
        header.setContentsMargins(0, 0, 0, 0)
        self.toggle_button = QtWidgets.QToolButton()
        self.toggle_button.setText("<")
        self.toggle_button.setToolTip("Aç/Kapat")
        self.toggle_button.clicked.connect(self.toggle)
        header.addWidget(self.toggle_button)
        title = QtWidgets.QLabel("Telemetry / Kontrol")
        title.setStyleSheet(f"color: {ACCENT_GAUGE}; font-weight: 700;")
        header.addWidget(title)
        header.addStretch(1)
        layout.addLayout(header)

        self.content = QtWidgets.QWidget(self)
        self.content_layout = QtWidgets.QVBoxLayout(self.content)
        self.content_layout.setContentsMargins(0, 0, 0, 0)
        self.content_layout.setSpacing(10)
        layout.addWidget(self.content)

        # Consolidated list: telemetry items and controls in one column.
        # Tum kutucuklar alt alta ve tamamen gorunur olsun diye ic scrollbar
        # kapatilir; liste icerige gore boyutlanir (bkz. populate -> fit_to_contents).
        self.item_list = DragListWidget(self)
        self.content_layout.addWidget(QtWidgets.QLabel("Veriler ve Kontroller"))
        self.content_layout.addWidget(self.item_list)

        self.content_layout.addStretch(1)

    def toggle(self):
        self._collapsed = not self._collapsed
        self.content.setVisible(not self._collapsed)
        self.toggle_button.setText(">" if self._collapsed else "<")
        if self._collapsed:
            self.setMaximumWidth(54)
            self.setMinimumWidth(54)
        else:
            self.setMaximumWidth(320)
            self.setMinimumWidth(72)

    def populate(self, telemetry_items, control_items):
        self.item_list.clear()
        for name, key in telemetry_items:
            item = QtWidgets.QListWidgetItem(name)
            item.setData(QtCore.Qt.UserRole, f"telemetry:{key}")
            # ensure the item is enabled and draggable
            flags = item.flags() | QtCore.Qt.ItemIsEnabled | QtCore.Qt.ItemIsSelectable | QtCore.Qt.ItemIsDragEnabled
            item.setFlags(flags)
            self.item_list.addItem(item)
        # separator
        sep = QtWidgets.QListWidgetItem("--- Kontroller ---")
        sep.setFlags(QtCore.Qt.NoItemFlags)
        self.item_list.addItem(sep)
        for name, key in control_items:
            item = QtWidgets.QListWidgetItem(name)
            item.setData(QtCore.Qt.UserRole, f"control:{key}")
            flags = item.flags() | QtCore.Qt.ItemIsEnabled | QtCore.Qt.ItemIsSelectable | QtCore.Qt.ItemIsDragEnabled
            item.setFlags(flags)
            self.item_list.addItem(item)
        # Tum kutucuklari kaydirmadan alt alta gorunur kilmak icin listeyi
        # icerige gore boyutlandir.
        self.item_list.fit_to_contents()

    def update_values(self, latest):
        for idx in range(self.item_list.count()):
            item = self.item_list.item(idx)
            payload = item.data(QtCore.Qt.UserRole)
            if payload is None:
                continue
            kind_key = payload.split(":", 1)
            if len(kind_key) != 2:
                continue
            kind, key = kind_key
            if kind != "telemetry":
                continue
            value = self._get_value(latest, key)
            base = item.text().split(':', 1)[0]
            item.setText(f"{base}: {self._fmt(value)}")

    def _get_value(self, latest, key):
        getter_map = {
            "actual_yaw": lambda s: s.actual_yaw,
            "setpoint_yaw": lambda s: s.setpoint_yaw,
            "actual_pitch": lambda s: s.actual_pitch,
            "actual_roll": lambda s: s.actual_roll,
            "actual_speed": lambda s: s.actual_speed,
            "setpoint_speed": lambda s: s.setpoint_speed,
            "lat": lambda s: s.lat,
            "lon": lambda s: s.lon,
            "gps_sats": lambda s: s.sats,
            "gps_hdop": lambda s: s.hdop,
        }
        getter = getter_map.get(key)
        if getter is not None:
            return getter(latest)
        if key.startswith("thruster_pwm"):
            ch = int(key.split("_")[-1])
            return latest.thruster_pwm.get(ch)
        return None

    @staticmethod
    def _fmt(val):
        if val is None:
            return "--"
        if isinstance(val, float):
            return f"{val:.2f}"
        return str(val)


def make_card(title, widget, accent):
    card = QtWidgets.QFrame()
    card.setObjectName("card")
    card.setStyleSheet(f"""
        QFrame#card {{
            background-color: {BG_CARD};
            border: 1px solid {BORDER_CARD};
            border-radius: 14px;
        }}
    """)
    v = QtWidgets.QVBoxLayout(card)
    v.setContentsMargins(14, 10, 14, 14)
    v.setSpacing(6)

    title_label = QtWidgets.QLabel(title)
    title_label.setStyleSheet(
        f"color: {accent}; font-weight: 800; font-size: 18px; "
        f"letter-spacing: 1px; background: transparent; border: none; padding-left: 2px; padding-bottom: 2px;"
    )
    v.addWidget(title_label)
    v.addWidget(widget)
    return card


def apply_min_yrange(plot, *value_lists, min_span, pad_ratio=0.15):
    vals = [v for values in value_lists for v in values if not math.isnan(v)]
    if not vals:
        return
    vmin, vmax = min(vals), max(vals)
    span = vmax - vmin
    if span < min_span:
        center = (vmax + vmin) / 2.0
        vmin = center - min_span / 2.0
        vmax = center + min_span / 2.0
    pad = (vmax - vmin) * pad_ratio
    plot.setYRange(vmin - pad, vmax + pad, padding=0)


def quaternion_to_yaw_deg(q):
    w, x, y, z = q
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.degrees(math.atan2(siny_cosp, cosy_cosp))


def quaternion_to_euler_deg(q):
    w, x, y, z = q
    ysqr = y * y

    t0 = 2.0 * (w * x + y * z)
    t1 = 1.0 - 2.0 * (x * x + ysqr)
    roll = math.degrees(math.atan2(t0, t1))

    t2 = 2.0 * (w * y - z * x)
    t2 = max(-1.0, min(1.0, t2))
    pitch = math.degrees(math.asin(t2))

    t3 = 2.0 * (w * z + x * y)
    t4 = 1.0 - 2.0 * (ysqr + z * z)
    yaw = math.degrees(math.atan2(t3, t4))
    return roll, pitch, yaw


def norm360(deg):
    if deg is None:
        return None
    d = deg % 360.0
    if d < 0:
        d += 360.0
    return d


class YawGaugeWidget(QtWidgets.QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setMinimumSize(220, 220)
        self.actual_deg = None
        self.setpoint_deg = None

    def set_values(self, actual_deg, setpoint_deg):
        self.actual_deg = actual_deg
        self.setpoint_deg = setpoint_deg
        self.update()

    def paintEvent(self, event):
        painter = QtGui.QPainter(self)
        painter.setRenderHint(QtGui.QPainter.Antialiasing)
        side = min(self.width(), self.height())
        painter.translate(self.width() / 2.0, self.height() / 2.0)
        painter.scale(side / 260.0, side / 260.0)

        painter.setPen(QtGui.QPen(QtGui.QColor("#3a3f4b"), 2))
        painter.setBrush(QtGui.QColor("#0f1117"))
        painter.drawEllipse(QtCore.QPointF(0, 0), 100, 100)

        font = painter.font()
        font.setPointSize(10)
        painter.setFont(font)
        for deg in range(0, 360, 30):
            painter.save()
            painter.rotate(deg)
            painter.setPen(QtGui.QPen(QtGui.QColor("#5b6270"), 2 if deg % 90 == 0 else 1))
            painter.drawLine(QtCore.QPointF(0, -100), QtCore.QPointF(0, -88 if deg % 90 == 0 else -92))
            painter.restore()

        painter.setPen(QtGui.QColor(TEXT_MAIN))
        labels = {0: "0/360", 90: "90", 180: "180", 270: "270"}
        for deg, text in labels.items():
            painter.save()
            painter.rotate(deg)
            painter.translate(0, -72)
            painter.rotate(-deg)
            painter.drawText(QtCore.QRectF(-40, -10, 80, 20), QtCore.Qt.AlignCenter, text)
            painter.restore()

        if self.setpoint_deg is not None:
            painter.save()
            painter.rotate(self.setpoint_deg)
            pen = QtGui.QPen(QtGui.QColor(ACCENT_THRUSTER), 3, QtCore.Qt.DashLine)
            pen.setCapStyle(QtCore.Qt.RoundCap)
            painter.setPen(pen)
            painter.drawLine(QtCore.QPointF(0, 0), QtCore.QPointF(0, -80))
            painter.restore()

        if self.actual_deg is not None:
            painter.save()
            painter.rotate(self.actual_deg)
            pen = QtGui.QPen(QtGui.QColor(ACCENT_GAUGE), 5)
            pen.setCapStyle(QtCore.Qt.RoundCap)
            painter.setPen(pen)
            painter.drawLine(QtCore.QPointF(0, 12), QtCore.QPointF(0, -85))
            painter.restore()

        painter.setPen(QtCore.Qt.NoPen)
        painter.setBrush(QtGui.QColor(ACCENT_GAUGE))
        painter.drawEllipse(QtCore.QPointF(0, 0), 6, 6)

        painter.setPen(QtGui.QColor("#ffffff"))
        big_font = painter.font()
        big_font.setPointSize(13)
        big_font.setBold(True)
        painter.setFont(big_font)
        val_text = f"{self.actual_deg:.0f} deg" if self.actual_deg is not None else "--"
        painter.drawText(QtCore.QRectF(-70, 40, 140, 25), QtCore.Qt.AlignCenter, val_text)

        painter.end()


class AttitudeIndicatorWidget(QtWidgets.QWidget):
    """Gemi/IDA egim gostergesi: klasik ucak 'artificial horizon' mantiginda
    pitch (yukari/asagi egim) ve roll (yan yatis) gosterir, ortada sabit
    tekne sembolu. Yaw ayri pusula kartinda gosteriliyor."""

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setMinimumSize(220, 220)
        self.pitch_deg = None
        self.roll_deg = None
        self.yaw_deg = None

    def set_values(self, pitch_deg, roll_deg, yaw_deg=None):
        self.pitch_deg = pitch_deg
        self.roll_deg = roll_deg
        self.yaw_deg = yaw_deg
        self.update()

    def paintEvent(self, event):
        painter = QtGui.QPainter(self)
        painter.setRenderHint(QtGui.QPainter.Antialiasing)
        side = min(self.width(), self.height())
        cx, cy = self.width() / 2.0, self.height() / 2.0
        r = side / 2.0 - 6

        clip_path = QtGui.QPainterPath()
        clip_path.addEllipse(QtCore.QPointF(cx, cy), r, r)
        painter.setClipPath(clip_path)

        pitch = self.pitch_deg or 0.0
        roll = self.roll_deg or 0.0
        px_per_deg = r / 45.0

        painter.save()
        painter.translate(cx, cy)
        painter.rotate(-roll)
        painter.translate(0, pitch * px_per_deg)

        big = r * 3
        sky = QtGui.QLinearGradient(0, -big, 0, 0)
        sky.setColorAt(0, QtGui.QColor("#3b82c4"))
        sky.setColorAt(1, QtGui.QColor("#7fb8e8"))
        painter.setBrush(sky)
        painter.setPen(QtCore.Qt.NoPen)
        painter.drawRect(QtCore.QRectF(-big, -big, 2 * big, big))

        sea = QtGui.QLinearGradient(0, 0, 0, big)
        sea.setColorAt(0, QtGui.QColor("#0e4a4f"))
        sea.setColorAt(1, QtGui.QColor("#062226"))
        painter.setBrush(sea)
        painter.drawRect(QtCore.QRectF(-big, 0, 2 * big, big))

        painter.setPen(QtGui.QPen(QtGui.QColor("#ffffff"), 2))
        painter.drawLine(QtCore.QPointF(-big, 0), QtCore.QPointF(big, 0))

        font = painter.font()
        font.setPointSize(8)
        painter.setFont(font)
        for d in range(-40, 41, 10):
            if d == 0:
                continue
            y = -d * px_per_deg
            half_w = 28 if d % 20 == 0 else 16
            painter.setPen(QtGui.QPen(QtGui.QColor("#ffffff"), 1.5))
            painter.drawLine(QtCore.QPointF(-half_w, y), QtCore.QPointF(half_w, y))
            if d % 20 == 0:
                painter.drawText(QtCore.QRectF(half_w + 4, y - 8, 30, 16), QtCore.Qt.AlignVCenter, str(d))
        painter.restore()

        # disk kenari
        painter.setClipping(False)
        painter.setPen(QtGui.QPen(QtGui.QColor("#3a3f4b"), 3))
        painter.setBrush(QtCore.Qt.NoBrush)
        painter.drawEllipse(QtCore.QPointF(cx, cy), r, r)

        # sabit tekne/arac sembolu (merkezde, donmez)
        painter.save()
        painter.translate(cx, cy)
        pen = QtGui.QPen(QtGui.QColor("#ffd400"), 4)
        pen.setCapStyle(QtCore.Qt.RoundCap)
        painter.setPen(pen)
        painter.drawLine(QtCore.QPointF(-r * 0.35, 0), QtCore.QPointF(-8, 0))
        painter.drawLine(QtCore.QPointF(8, 0), QtCore.QPointF(r * 0.35, 0))
        painter.setBrush(QtGui.QColor("#ffd400"))
        painter.setPen(QtCore.Qt.NoPen)
        painter.drawEllipse(QtCore.QPointF(0, 0), 5, 5)
        painter.restore()

        # alt bilgi seridi
        painter.setPen(QtGui.QColor("#ffffff"))
        info_font = painter.font()
        info_font.setPointSize(11)
        info_font.setBold(True)
        painter.setFont(info_font)
        pitch_txt = f"{pitch:.0f}" if self.pitch_deg is not None else "--"
        roll_txt = f"{roll:.0f}" if self.roll_deg is not None else "--"
        painter.drawText(
            QtCore.QRectF(cx - r, cy + r * 0.55, 2 * r, 24),
            QtCore.Qt.AlignCenter,
            f"PITCH {pitch_txt}°   ROLL {roll_txt}°",
        )
        painter.end()


# ---- GPS harita (Leaflet + OpenStreetMap) ----
MAP_HTML = """
<!DOCTYPE html>
<html>
<head>
<meta charset="utf-8" />
<link rel="stylesheet" href="https://unpkg.com/leaflet@1.9.4/dist/leaflet.css" />
<style>
  html, body, #map { height: 100%; margin: 0; background: #0f1117; }
  .leaflet-control-attribution { display: none; }
</style>
</head>
<body>
<div id="map"></div>
<script src="https://unpkg.com/leaflet@1.9.4/dist/leaflet.js"></script>
<script>
  var map = L.map('map', {zoomControl: true, attributionControl: false}).setView([0, 0], 2);
  L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', {maxZoom: 19}).addTo(map);

  var track = [];
  var polyline = L.polyline([], {color: '#22d3ee', weight: 4, opacity: 0.9}).addTo(map);
  var marker = null;
  var firstFix = true;

  function makeIcon(heading) {
    var size = 54;
    var html =
      '<div style="width:' + size + 'px;height:' + size + 'px;position:relative;">' +
        '<div style="position:absolute;top:50%;left:50%;width:44px;height:44px;' +
             'margin:-22px 0 0 -22px;border-radius:50%;' +
             'background:radial-gradient(circle, rgba(255,82,82,0.35) 0%, rgba(255,82,82,0) 70%);"></div>' +
        '<div style="width:100%;height:100%;transform: rotate(' + heading + 'deg);">' +
          '<svg viewBox="0 0 24 24" width="' + size + '" height="' + size + '">' +
            '<polygon points="12,1.5 21,22.5 12,17.5 3,22.5" fill="#ff3b3b" stroke="#ffffff" stroke-width="1.4"/>' +
          '</svg>' +
        '</div>' +
      '</div>';
    return L.divIcon({className: '', html: html, iconSize: [size, size], iconAnchor: [size / 2, size / 2]});
  }

  function setPosition(lat, lon, heading) {
    var latlng = [lat, lon];
    if (marker === null) {
      marker = L.marker(latlng, {icon: makeIcon(heading), zIndexOffset: 1000}).addTo(map);
    } else {
      marker.setLatLng(latlng);
      marker.setIcon(makeIcon(heading));
    }
    track.push(latlng);
    if (track.length > 1000) { track.shift(); }
    polyline.setLatLngs(track);
    if (firstFix) {
      map.setView(latlng, 19);
      firstFix = false;
    } else {
      map.panTo(latlng);
    }
  }
</script>
</body>
</html>
"""


class GpsMapWidget(QtWidgets.QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setMinimumHeight(280)
        self._ready = False
        layout = QtWidgets.QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)

        if HAS_WEBENGINE:
            self.view = QWebEngineView()
            self.view.loadFinished.connect(self._on_load_finished)
            self.view.setHtml(MAP_HTML)
            layout.addWidget(self.view)
            self._fallback_plot = None
        else:
            self.view = None
            self._fallback_plot = pg.PlotWidget(title="GPS izi (harita icin: pip install PyQtWebEngine)")
            self._fallback_plot.showGrid(x=True, y=True, alpha=0.25)
            self._fallback_curve = self._fallback_plot.plot(pen=pg.mkPen(ACCENT_MAP, width=2))
            self._fallback_points_lat = []
            self._fallback_points_lon = []
            layout.addWidget(self._fallback_plot)

    def _on_load_finished(self, ok):
        self._ready = ok
        if ok and self.view is not None:
            # Windows'ta QWebEngineView bazen ilk boyutlandirmada bos kaliyor;
            # kucuk bir resize "nudge" ile yeniden cizdirmeye zorla.
            def _nudge():
                s = self.view.size()
                self.view.resize(s.width() + 1, s.height())
                self.view.resize(s.width(), s.height())
            QtCore.QTimer.singleShot(150, _nudge)
            QtCore.QTimer.singleShot(600, _nudge)

    def update_position(self, lat, lon, heading_deg):
        if lat is None or lon is None:
            return
        hdg = heading_deg if heading_deg is not None else 0
        if HAS_WEBENGINE:
            if not self._ready:
                return
            self.view.page().runJavaScript(f"setPosition({lat}, {lon}, {hdg});")
        else:
            self._fallback_points_lat.append(lat)
            self._fallback_points_lon.append(lon)
            self._fallback_curve.setData(self._fallback_points_lon, self._fallback_points_lat)


class LatestState:
    def __init__(self):
        self.actual_yaw = None
        self.setpoint_yaw = None
        self.actual_pitch = None
        self.actual_roll = None
        self.actual_speed = None
        self.setpoint_speed = None
        self.setpoint_speed_static = None  # CRUISE_SPEED parametresinden (Rover)
        self.lat = None
        self.lon = None
        self.sats = None   # GPS'e bagli uydu sayisi
        self.hdop = None   # yatay konum belirsizligi (HDOP)
        self.thruster_pwm = {}

    def _set_attitude_from_euler(self, roll_rad, pitch_rad, yaw_rad):
        self.actual_roll = math.degrees(roll_rad)
        self.actual_pitch = math.degrees(pitch_rad)
        self.actual_yaw = norm360(math.degrees(yaw_rad))

    def on_attitude(self, msg):
        self._set_attitude_from_euler(msg.roll, msg.pitch, msg.yaw)

    def on_attitude_quaternion(self, msg):
        q = getattr(msg, "q", None)
        if q is None:
            q = [getattr(msg, f"q{i}", None) for i in range(1, 5)]
        if not q or len(q) != 4 or any(v is None for v in q):
            return
        roll_deg, pitch_deg, yaw_deg = quaternion_to_euler_deg(q)
        self.actual_roll = roll_deg
        self.actual_pitch = pitch_deg
        self.actual_yaw = norm360(yaw_deg)

    def on_ahrs2(self, msg):
        self._set_attitude_from_euler(msg.roll, msg.pitch, msg.yaw)

    def on_vfr_hud(self, msg):
        self.actual_speed = msg.groundspeed
        if self.actual_yaw is None:
            self.actual_yaw = norm360(float(msg.heading))

    def on_attitude_target(self, msg):
        self.setpoint_yaw = norm360(quaternion_to_yaw_deg(msg.q))

    def on_position_target_local_ned(self, msg):
        vx, vy = msg.vx, msg.vy
        self.setpoint_speed = math.hypot(vx, vy)

    def on_nav_controller_output(self, msg):
        # ArduRover Auto/waypoint modunda gercek setpoint kaynagi budur:
        # nav_bearing = o an hedeflenen heading (derece)
        self.setpoint_yaw = norm360(float(msg.nav_bearing))

    def on_param_value(self, msg):
        try:
            name = msg.param_id
            if isinstance(name, bytes):
                name = name.decode(errors="ignore")
            name = name.strip("\x00")
        except Exception:
            return
        if name in ("CRUISE_SPEED", "WP_SPEED"):
            self.setpoint_speed_static = float(msg.param_value)

    def on_global_position_int(self, msg):
        if msg.lat != 0 or msg.lon != 0:
            self.lat = msg.lat / 1e7
            self.lon = msg.lon / 1e7

    def on_gps_raw_int(self, msg):
        if self.lat is None and (msg.lat != 0 or msg.lon != 0):
            self.lat = msg.lat / 1e7
            self.lon = msg.lon / 1e7
        # Uydu sayisi (satellites_visible): 255 = bilinmiyor (UINT8_MAX)
        sats = getattr(msg, "satellites_visible", None)
        if sats is not None and sats != 255:
            self.sats = int(sats)
        # HDOP (eph): birimsiz * 100 olarak gelir, 65535 = bilinmiyor (UINT16_MAX)
        eph = getattr(msg, "eph", None)
        if eph is not None and eph != 65535:
            self.hdop = eph / 100.0

    def on_servo_output_raw(self, msg):
        for ch in THRUSTER_CHANNELS:
            val = getattr(msg, f"servo{ch}_raw", None)
            if val:
                self.thruster_pwm[ch] = val


class TelemetryBuffer:
    def __init__(self, maxlen=HISTORY_LEN):
        self.maxlen = maxlen
        self.sample_idx = 0
        self.t = deque(maxlen=maxlen)
        self.actual_speed = deque(maxlen=maxlen)
        self.setpoint_speed = deque(maxlen=maxlen)
        self.actual_yaw = deque(maxlen=maxlen)
        self.setpoint_yaw = deque(maxlen=maxlen)
        self.thruster = {ch: deque(maxlen=maxlen) for ch in THRUSTER_CHANNELS}

    def sample(self, latest: LatestState):
        self.sample_idx += 1
        self.t.append(self.sample_idx)
        speed_sp = latest.setpoint_speed
        if speed_sp is None:
            speed_sp = latest.setpoint_speed_static
        self.actual_speed.append(latest.actual_speed if latest.actual_speed is not None else float("nan"))
        self.setpoint_speed.append(speed_sp if speed_sp is not None else float("nan"))
        self.actual_yaw.append(latest.actual_yaw if latest.actual_yaw is not None else float("nan"))
        self.setpoint_yaw.append(latest.setpoint_yaw if latest.setpoint_yaw is not None else float("nan"))
        for ch in THRUSTER_CHANNELS:
            pwm = latest.thruster_pwm.get(ch)
            pct = ((pwm - PWM_NEUTRAL) / PWM_RANGE * 100.0) if pwm else float("nan")
            self.thruster[ch].append(pct)


class LivePlotWindow(QtWidgets.QMainWindow):
    def __init__(self, master):
        super().__init__()
        self.master = master
        self.latest = LatestState()
        self.buf = TelemetryBuffer()
        self.msg_count = 0
        self.thruster_curves = {}

        self.setWindowTitle("Pixhawk / IDA Telemetri - 2. Ekran (Canli) [v2]")
        self.resize(1750, 980)
        self.setStyleSheet(f"QMainWindow {{ background-color: {BG_WINDOW}; }}")

        central = QtWidgets.QWidget()
        central.setStyleSheet(f"background-color: {BG_WINDOW};")
        self.setCentralWidget(central)
        main_col = QtWidgets.QVBoxLayout(central)
        main_col.setContentsMargins(14, 12, 14, 14)
        main_col.setSpacing(10)

        splitter_qss = f"""
            QSplitter::handle {{
                background-color: {BORDER_CARD};
                border-radius: 3px;
            }}
            QSplitter::handle:hover {{
                background-color: {ACCENT_GAUGE};
            }}
            QSplitter::handle:horizontal {{ width: 7px; margin: 2px 0; }}
            QSplitter::handle:vertical {{ height: 7px; margin: 0 2px; }}
        """

        self.status_label = QtWidgets.QLabel("Veri bekleniyor...")
        self.status_label.setTextFormat(QtCore.Qt.RichText)
        self.status_label.setStyleSheet(f"""
            background-color: {BG_CARD};
            border: 1px solid {BORDER_CARD};
            border-radius: 10px;
            padding: 8px 14px;
            font-size: 13px;
            color: {TEXT_MAIN};
        """)

        # --- En ust bar: durum etiketi + Layout Kaydet / Sifirla butonlari ---
        top_bar = QtWidgets.QHBoxLayout()
        top_bar.setContentsMargins(0, 0, 0, 0)
        top_bar.setSpacing(8)
        top_bar.addWidget(self.status_label, stretch=1)

        self.save_layout_btn = QtWidgets.QPushButton("💾  Layout Kaydet")
        self.save_layout_btn.setCursor(QtCore.Qt.PointingHandCursor)
        self.save_layout_btn.setStyleSheet(f"""
            QPushButton {{
                background-color: {ACCENT_GAUGE}; color: #071223;
                border: none; border-radius: 10px;
                padding: 8px 16px; font-weight: 700; font-size: 13px;
            }}
            QPushButton:hover {{ background-color: #7cb3ff; }}
            QPushButton:pressed {{ background-color: #4a86d6; }}
        """)
        self.save_layout_btn.clicked.connect(self.save_layout)
        top_bar.addWidget(self.save_layout_btn)

        self.reset_layout_btn = QtWidgets.QPushButton("Sıfırla")
        self.reset_layout_btn.setCursor(QtCore.Qt.PointingHandCursor)
        self.reset_layout_btn.setStyleSheet(f"""
            QPushButton {{
                background-color: {BG_CARD}; color: {TEXT_MAIN};
                border: 1px solid {BORDER_CARD}; border-radius: 10px;
                padding: 8px 14px; font-size: 13px;
            }}
            QPushButton:hover {{ border-color: {ACCENT_THRUSTER}; color: {ACCENT_THRUSTER}; }}
        """)
        self.reset_layout_btn.setToolTip("Ekrandaki tüm kartları kaldır ve kayıtlı layout'u sil")
        self.reset_layout_btn.clicked.connect(self.reset_layout)
        top_bar.addWidget(self.reset_layout_btn)

        main_col.addLayout(top_bar)

        content_splitter = QtWidgets.QSplitter(QtCore.Qt.Horizontal)
        content_splitter.setStyleSheet(splitter_qss)
        content_splitter.setChildrenCollapsible(False)
        main_col.addWidget(content_splitter, stretch=1)

        self.sidebar = CollapsibleSidebar(self)
        content_splitter.addWidget(self.sidebar)

        content_area = QtWidgets.QWidget(self)
        content_area.setStyleSheet(f"background-color: {BG_WINDOW};")
        content_layout = QtWidgets.QVBoxLayout(content_area)
        content_layout.setContentsMargins(0, 0, 0, 0)
        content_layout.setSpacing(10)
        content_splitter.addWidget(content_area)

        self.dashboard_canvas = DashboardCanvas(content_area)
        self.dashboard_canvas.setMinimumHeight(320)
        # make canvas occupy the main content area so drops work across the large green area
        content_layout.addWidget(self.dashboard_canvas, stretch=3)

        left_splitter = QtWidgets.QSplitter(QtCore.Qt.Vertical)
        left_splitter.setStyleSheet(splitter_qss)
        left_splitter.setChildrenCollapsible(False)

        self.speed_plot = pg.PlotWidget()
        self.speed_plot.addLegend()
        self.speed_plot.showGrid(x=True, y=True, alpha=0.25)
        self.actual_speed_curve = self.speed_plot.plot(pen=pg.mkPen(ACCENT_SPEED, width=2), name="Gercek hiz")
        self.setpoint_speed_curve = self.speed_plot.plot(
            pen=pg.mkPen("#fbbf24", width=2, style=QtCore.Qt.DashLine), name="Hiz istegi (setpoint)"
        )
        self.speed_plot.enableAutoRange("y", False)
        # Do not add the large default speed card to the UI by default.
        # The speed graph is available as a draggable widget from the sidebar.
        self.card_speed = make_card("HIZ  —  GERCEK / SETPOINT (m/s)", self.speed_plot, ACCENT_SPEED)

        self.yaw_plot = pg.PlotWidget()
        self.yaw_plot.addLegend()
        self.yaw_plot.showGrid(x=True, y=True, alpha=0.25)
        self.actual_yaw_curve = self.yaw_plot.plot(pen=pg.mkPen(ACCENT_YAW, width=2), name="Gercek yaw")
        self.setpoint_yaw_curve = self.yaw_plot.plot(
            pen=pg.mkPen(ACCENT_THRUSTER, width=2, style=QtCore.Qt.DashLine), name="Yaw istegi (setpoint)"
        )
        self.yaw_plot.enableAutoRange("y", False)
        self.card_yaw = make_card("HEADING / YAW  —  0-360 DERECE", self.yaw_plot, ACCENT_YAW)

        self.thruster_plot = pg.PlotWidget()
        self.thruster_plot.addLegend()
        self.thruster_plot.showGrid(x=True, y=True, alpha=0.25)
        self.thruster_plot.setYRange(-100, 100)
        self.card_thruster = make_card("THRUSTERLARDAN KUVVET ISTEGI (%)", self.thruster_plot, ACCENT_THRUSTER)
        left_splitter.setSizes([300, 300, 300])
        # left_splitter not added: graphs are instead available via the sidebar.

        mid_splitter = QtWidgets.QSplitter(QtCore.Qt.Vertical)
        mid_splitter.setStyleSheet(splitter_qss)
        mid_splitter.setChildrenCollapsible(False)

        self.yaw_gauge = YawGaugeWidget()
        self.card_gauge = make_card("PUSULA — YAW", self.yaw_gauge, ACCENT_GAUGE)

        self.attitude_widget = AttitudeIndicatorWidget()
        self.card_attitude = make_card("GEMI EGIMI — PITCH / ROLL", self.attitude_widget, ACCENT_ATTITUDE)

        self.gps_map = GpsMapWidget()
        self.card_map = make_card("GPS HARITA", self.gps_map, ACCENT_MAP)
        mid_splitter.setSizes([280, 280, 400])
        # mid_splitter not added: gauge/attitude/map are available via sidebar.

        self.telemetry_specs = [
            ("Gerçek Yaw", "actual_yaw"),
            ("Hedef Yaw", "setpoint_yaw"),
            ("Gerçek Pitch", "actual_pitch"),
            ("Gerçek Roll", "actual_roll"),
            ("Gerçek Hız", "actual_speed"),
            ("Hedef Hız", "setpoint_speed"),
            ("Enlem", "lat"),
            ("Boylam", "lon"),
            ("GPS Uydu Sayısı", "gps_sats"),
            ("GPS HDOP", "gps_hdop"),
        ]
        self.telemetry_specs.extend((f"Thruster {ch}", f"thruster_pwm_{ch}") for ch in THRUSTER_CHANNELS)
        # Controls: include DoAction and available widget types (graphs/gauges/maps)
        widget_items = [
            ("DoAction", "doaction"),
            ("ARM / DISARM", "arm"),
            ("MODE", "mode"),
            ("Hız Grafiği", "widget_speed_graph"),
            ("Yaw Grafiği", "widget_yaw_graph"),
            ("Thruster Grafiği", "widget_thruster_graph"),
            ("Pusula (Gauge)", "widget_yaw_gauge"),
            ("Gemi Eğimi (Attitude)", "widget_attitude"),
            ("GPS Harita", "widget_gps"),
        ]
        self.sidebar.populate(self.telemetry_specs, widget_items)

        self.dashboard_canvas.setParent(content_area)

        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.update_loop)
        self.timer.start(UPDATE_MS)

        # Pencere gorunur olduktan sonra kayitli layout'u otomatik geri yukle
        # (web engine tabanli GPS harita icin olay dongusunun baslamis olmasi gerekir).
        QtCore.QTimer.singleShot(250, self.load_layout)

    def create_dashboard_widget(self, kind, key, pos):
        if kind == "telemetry":
            title = self._telemetry_title(key)
            widget = DashboardTelemetryWidget(
                self.dashboard_canvas,
                title,
                key,
                lambda latest, k=key: self._get_value_for_key(latest, k),
            )
        elif kind == "control":
            # control key can be arm/mode/doaction/widget_* etc
            if key == "arm":
                widget = DashboardControlWidget(self.dashboard_canvas, self.master)
            elif key == "mode":
                widget = DashboardControlWidget(self.dashboard_canvas, self.master)
            elif key == "doaction":
                widget = DashboardActionWidget(self.dashboard_canvas, "DoAction")
            elif key.startswith("widget_"):
                # Grafik/gosterge widgetlari. Basit zaman serileri telemetry
                # widget'i olur; pusula/attitude/gps ise dogrudan gorsel acilir.
                graph_mapping = {
                    "widget_speed_graph": ("Hız Grafiği", "actual_speed"),
                    "widget_yaw_graph": ("Yaw Grafiği", "actual_yaw"),
                    "widget_thruster_graph": ("Thruster Grafiği", "thruster_pwm_1"),
                }
                visual_mapping = {
                    "widget_yaw_gauge": ("Pusula", "gauge"),
                    "widget_attitude": ("Gemi Eğimi", "attitude"),
                    "widget_gps": ("GPS Harita", "gps"),
                }
                if key in graph_mapping:
                    title, data_key = graph_mapping[key]
                    widget = DashboardTelemetryWidget(
                        self.dashboard_canvas, title, data_key,
                        lambda latest, k=data_key: self._get_value_for_key(latest, k),
                    )
                    # zaman serisi grafikleri dogrudan grafik modunda acilsin
                    widget.toggle_graphical()
                elif key in visual_mapping:
                    title, vkind = visual_mapping[key]
                    widget = DashboardVisualWidget(self.dashboard_canvas, title, vkind)
                else:
                    widget = DashboardControlWidget(self.dashboard_canvas, self.master)
            else:
                widget = DashboardControlWidget(self.dashboard_canvas, self.master)
        else:
            return None

        # Widget'in kendi tercih ettigi boyutu koru (GPS harita/gauge daha buyuk).
        w = max(widget.width(), 220)
        h = max(widget.height(), 140)
        widget.setGeometry(QtCore.QRect(pos.x(), pos.y(), w, h))
        # Layout kaydi icin olusturma bilgisini widget uzerinde sakla.
        widget._spec_kind = kind
        widget._spec_key = key
        widget.show()
        self.dashboard_canvas.dashboard_items.append(widget)
        return widget

    # ---- Layout kaydet / yukle / sifirla ----
    def save_layout(self):
        items = []
        for w in list(self.dashboard_canvas.dashboard_items):
            try:
                if not hasattr(w, "_spec_kind"):
                    continue
                g = w.geometry()
                entry = {
                    "kind": w._spec_kind,
                    "key": w._spec_key,
                    "x": g.x(), "y": g.y(), "w": g.width(), "h": g.height(),
                }
                if isinstance(w, DashboardTelemetryWidget):
                    entry["graphical"] = bool(w.graphical_view)
                if isinstance(w, DashboardActionWidget) and getattr(w, "_path", None):
                    entry["path"] = w._path
                items.append(entry)
            except Exception:
                continue
        try:
            with open(LAYOUT_FILE, "w", encoding="utf-8") as f:
                json.dump({"items": items}, f, ensure_ascii=False, indent=2)
            # kisa gorsel onay
            self.save_layout_btn.setText("✓  Kaydedildi")
            QtCore.QTimer.singleShot(1500, lambda: self.save_layout_btn.setText("💾  Layout Kaydet"))
        except Exception as exc:
            QtWidgets.QMessageBox.warning(self, "Layout Kaydet", f"Kaydedilemedi:\n{exc}")

    def load_layout(self):
        if not os.path.exists(LAYOUT_FILE):
            return
        try:
            with open(LAYOUT_FILE, "r", encoding="utf-8") as f:
                data = json.load(f)
        except Exception as exc:
            print(f"[layout] Yukleme hatasi: {exc}")
            return
        for entry in data.get("items", []):
            try:
                kind = entry.get("kind")
                key = entry.get("key")
                if not kind or not key:
                    continue
                x = int(entry.get("x", 20))
                y = int(entry.get("y", 20))
                w = int(entry.get("w", 220))
                h = int(entry.get("h", 140))
                widget = self.create_dashboard_widget(kind, key, QtCore.QPoint(x, y))
                if widget is None:
                    continue
                widget.setGeometry(QtCore.QRect(x, y, w, h))
                # telemetry: kaydedilen grafik/metin durumunu uygula
                if isinstance(widget, DashboardTelemetryWidget):
                    want_graph = bool(entry.get("graphical", False))
                    if widget.graphical_view != want_graph:
                        widget.toggle_graphical()
                # DoAction: secili dosyayi geri yukle
                if isinstance(widget, DashboardActionWidget) and entry.get("path"):
                    widget._path = entry["path"]
                    widget.file_label.setText(f"Dosya: {os.path.basename(entry['path'])}")
            except Exception as exc:
                print(f"[layout] Oge yuklenemedi: {exc}")
                continue

    def reset_layout(self):
        answer = QtWidgets.QMessageBox.question(
            self, "Layout Sıfırla",
            "Ekrandaki tüm kartlar kaldırılsın ve kayıtlı layout silinsin mi?",
            QtWidgets.QMessageBox.Yes | QtWidgets.QMessageBox.No,
            QtWidgets.QMessageBox.No,
        )
        if answer != QtWidgets.QMessageBox.Yes:
            return
        for w in list(self.dashboard_canvas.dashboard_items):
            try:
                w.deleteLater()
            except Exception:
                pass
        self.dashboard_canvas.dashboard_items = []
        try:
            if os.path.exists(LAYOUT_FILE):
                os.remove(LAYOUT_FILE)
        except Exception as exc:
            QtWidgets.QMessageBox.warning(self, "Layout Sıfırla", f"Dosya silinemedi:\n{exc}")

    def _telemetry_title(self, key):
        for name, data_key in self.telemetry_specs:
            if data_key == key:
                return name
        return key

    def _get_value_for_key(self, latest, key):
        getter_map = {
            "actual_yaw": lambda s: s.actual_yaw,
            "setpoint_yaw": lambda s: s.setpoint_yaw,
            "actual_pitch": lambda s: s.actual_pitch,
            "actual_roll": lambda s: s.actual_roll,
            "actual_speed": lambda s: s.actual_speed,
            "setpoint_speed": lambda s: s.setpoint_speed,
            "lat": lambda s: s.lat,
            "lon": lambda s: s.lon,
            "gps_sats": lambda s: s.sats,
            "gps_hdop": lambda s: s.hdop,
        }
        getter = getter_map.get(key)
        if getter is not None:
            return getter(latest)
        if key.startswith("thruster_pwm"):
            ch = int(key.split("_")[-1])
            return latest.thruster_pwm.get(ch)
        return None

    def _ensure_thruster_curve(self, ch):
        if ch not in self.thruster_curves:
            color = pg.intColor(ch, hues=THRUSTER_HUES)
            self.thruster_curves[ch] = self.thruster_plot.plot(
                pen=pg.mkPen(color, width=2), name=f"Thruster {ch}"
            )

    def update_loop(self):
        while True:
            msg = self.master.recv_match(blocking=False)
            if msg is None:
                break
            mtype = msg.get_type()
            if mtype == "BAD_DATA":
                continue
            self.msg_count += 1

            if mtype == "ATTITUDE":
                self.latest.on_attitude(msg)
            elif mtype == "VFR_HUD":
                self.latest.on_vfr_hud(msg)
            elif mtype == "ATTITUDE_TARGET":
                self.latest.on_attitude_target(msg)
            elif mtype == "POSITION_TARGET_LOCAL_NED":
                self.latest.on_position_target_local_ned(msg)
            elif mtype == "NAV_CONTROLLER_OUTPUT":
                self.latest.on_nav_controller_output(msg)
            elif mtype == "PARAM_VALUE":
                self.latest.on_param_value(msg)
            elif mtype == "GLOBAL_POSITION_INT":
                self.latest.on_global_position_int(msg)
            elif mtype == "GPS_RAW_INT":
                self.latest.on_gps_raw_int(msg)
            elif mtype == "SERVO_OUTPUT_RAW":
                self.latest.on_servo_output_raw(msg)
                for ch in self.latest.thruster_pwm:
                    self._ensure_thruster_curve(ch)
            elif mtype == "ATTITUDE_QUATERNION":
                self.latest.on_attitude_quaternion(msg)
            elif mtype == "AHRS2":
                self.latest.on_ahrs2(msg)

        self.buf.sample(self.latest)
        self.sidebar.update_values(self.latest)
        self.yaw_gauge.set_values(self.latest.actual_yaw, self.latest.setpoint_yaw)
        self.attitude_widget.set_values(self.latest.actual_pitch, self.latest.actual_roll, self.latest.actual_yaw)
        self.gps_map.update_position(self.latest.lat, self.latest.lon, self.latest.actual_yaw)

        for widget in list(self.dashboard_canvas.dashboard_items):
            if isinstance(widget, (DashboardTelemetryWidget, DashboardVisualWidget)):
                widget.update_from_latest(self.latest)

        gps_text = f"{self.latest.lat:.6f}, {self.latest.lon:.6f}" if self.latest.lat is not None else "-"
        self.status_label.setText(
            f"<span style='color:{TEXT_DIM}'>[{self.msg_count} mesaj]</span>&nbsp;&nbsp;"
            f"<span style='color:{ACCENT_SPEED}'>HIZ</span> "
            f"{self._fmt(self.latest.actual_speed)} / {self._fmt(self.latest.setpoint_speed)} m/s"
            f"&nbsp;&nbsp;|&nbsp;&nbsp;"
            f"<span style='color:{ACCENT_YAW}'>YAW</span> "
            f"{self._fmt(self.latest.actual_yaw)} / {self._fmt(self.latest.setpoint_yaw)} deg"
            f"&nbsp;&nbsp;|&nbsp;&nbsp;"
            f"<span style='color:{ACCENT_ATTITUDE}'>PITCH/ROLL</span> "
            f"{self._fmt(self.latest.actual_pitch)} / {self._fmt(self.latest.actual_roll)} deg"
            f"&nbsp;&nbsp;|&nbsp;&nbsp;"
            f"<span style='color:{ACCENT_MAP}'>GPS</span> {gps_text}"
        )

        t = list(self.buf.t)
        if len(t) > 1:
            actual_speed = list(self.buf.actual_speed)
            setpoint_speed = list(self.buf.setpoint_speed)
            actual_yaw = list(self.buf.actual_yaw)
            setpoint_yaw = list(self.buf.setpoint_yaw)

            if hasattr(self, 'actual_speed_curve') and getattr(self, 'actual_speed_curve') is not None:
                try:
                    self.actual_speed_curve.setData(t, actual_speed)
                except Exception:
                    pass
            if hasattr(self, 'setpoint_speed_curve') and getattr(self, 'setpoint_speed_curve') is not None:
                try:
                    self.setpoint_speed_curve.setData(t, setpoint_speed)
                except Exception:
                    pass
            if hasattr(self, 'actual_yaw_curve') and getattr(self, 'actual_yaw_curve') is not None:
                try:
                    self.actual_yaw_curve.setData(t, actual_yaw)
                except Exception:
                    pass
            if hasattr(self, 'setpoint_yaw_curve') and getattr(self, 'setpoint_yaw_curve') is not None:
                try:
                    self.setpoint_yaw_curve.setData(t, setpoint_yaw)
                except Exception:
                    pass
            for ch, curve in list(self.thruster_curves.items()):
                try:
                    curve.setData(t, list(self.buf.thruster[ch]))
                except Exception:
                    pass

            apply_min_yrange(self.speed_plot, actual_speed, setpoint_speed, min_span=MIN_SPEED_SPAN)
            self.yaw_plot.setYRange(0, 360, padding=0)

    @staticmethod
    def _fmt(v):
        return "-" if v is None else f"{v:.1f}"


def request_streams(master):
    sysid = master.target_system or 1
    compid = master.target_component or 1

    master.mav.request_data_stream_send(
        sysid, compid, mavutil.mavlink.MAV_DATA_STREAM_ALL, 10, 1
    )

    msg_ids = {
        "ATTITUDE": mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE,
        "VFR_HUD": mavutil.mavlink.MAVLINK_MSG_ID_VFR_HUD,
        "ATTITUDE_TARGET": mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE_TARGET,
        "POSITION_TARGET_LOCAL_NED": mavutil.mavlink.MAVLINK_MSG_ID_POSITION_TARGET_LOCAL_NED,
        "NAV_CONTROLLER_OUTPUT": mavutil.mavlink.MAVLINK_MSG_ID_NAV_CONTROLLER_OUTPUT,
        "SERVO_OUTPUT_RAW": mavutil.mavlink.MAVLINK_MSG_ID_SERVO_OUTPUT_RAW,
        "GLOBAL_POSITION_INT": mavutil.mavlink.MAVLINK_MSG_ID_GLOBAL_POSITION_INT,
        "GPS_RAW_INT": mavutil.mavlink.MAVLINK_MSG_ID_GPS_RAW_INT,
        "ATTITUDE_QUATERNION": mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE_QUATERNION,
        "AHRS2": mavutil.mavlink.MAVLINK_MSG_ID_AHRS2,
    }
    for name, msg_id in msg_ids.items():
        master.mav.command_long_send(
            sysid, compid,
            mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
            0,
            msg_id,
            100000,
            0, 0, 0, 0, 0,
        )

    # Rover'in hedef hizi (CRUISE_SPEED / WP_SPEED parametresi) - sabit referans olarak
    for pname in (b"CRUISE_SPEED", b"WP_SPEED"):
        master.mav.param_request_read_send(sysid, compid, pname, -1)


def connect_master():
    candidates = [CONNECTION_STRING] + CONNECTION_FALLBACKS
    last_error = None
    for conn in candidates:
        try:
            print(f"[v2] Baglanti deneniyor: {conn}")
            master = mavutil.mavlink_connection(conn)
            hb = master.wait_heartbeat(timeout=3)
            if hb is not None:
                master.target_system = master.target_system or hb.get_srcSystem() or 1
                master.target_component = master.target_component or hb.get_srcComponent() or 1
                print(f"[v2] Heartbeat alindi. sysid={master.target_system} compid={master.target_component} via {conn}")
                return master

            master.target_system = master.target_system or 1
            master.target_component = master.target_component or 1
            print(f"[v2] Heartbeat bekleniyor, baglanti kuruldu: {conn}")
            return master
        except Exception as exc:
            last_error = str(exc)
            print(f"[v2] Baglanti hatasi ({conn}): {exc}")
    raise RuntimeError(f"MAVLink baglantisi kurulamadi: {last_error}")


def main():
    try:
        master = connect_master()
    except RuntimeError as exc:
        print(str(exc))
        sys.exit(1)

    if not HAS_WEBENGINE:
        print("Uyari: PyQtWebEngine bulunamadi, harita basit iz grafigine dusecek.")
        print("Gercek harita icin: pip install PyQtWebEngine")

    request_streams(master)

    app = QtWidgets.QApplication(sys.argv)
    win = LivePlotWindow(master)
    win.show()
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
