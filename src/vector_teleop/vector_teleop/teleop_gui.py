#!/usr/bin/env python3
"""Beautiful Qt-based teleoperation GUI for VECTOR NAV."""

import sys
import math
import signal
import threading

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu

from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QGridLayout, QPushButton, QLabel, QSlider, QFrame, QGroupBox,
    QSizePolicy, QGraphicsDropShadowEffect,
)
from PyQt5.QtCore import Qt, QTimer, pyqtSignal, QObject
from PyQt5.QtGui import QFont, QColor, QPainter, QPen, QBrush, QKeyEvent


# ── Colour palette ──────────────────────────────────────────────────
BG_DARK = "#0f1117"
BG_CARD = "#1a1d28"
BG_CARD_HOVER = "#232736"
ACCENT = "#6c63ff"
ACCENT_LIGHT = "#8b83ff"
ACCENT_DIM = "#3d3799"
TEXT = "#e8e6f0"
TEXT_DIM = "#8a8a9a"
RED = "#ff4d6a"
GREEN = "#2dd4a8"
YELLOW = "#fbbf24"
BORDER = "#2a2d3a"

STYLESHEET = f"""
QMainWindow {{
    background-color: {BG_DARK};
}}
QWidget {{
    color: {TEXT};
    font-family: 'Ubuntu', 'Segoe UI', 'Arial';
}}
QGroupBox {{
    background-color: {BG_CARD};
    border: 1px solid {BORDER};
    border-radius: 12px;
    margin-top: 14px;
    padding: 18px 14px 14px 14px;
    font-size: 13px;
    font-weight: bold;
}}
QGroupBox::title {{
    subcontrol-origin: margin;
    left: 16px;
    padding: 0 6px;
    color: {ACCENT_LIGHT};
    font-size: 12px;
    letter-spacing: 1px;
}}
QLabel {{
    background: transparent;
}}
QSlider::groove:horizontal {{
    border: none;
    height: 6px;
    background: {BORDER};
    border-radius: 3px;
}}
QSlider::handle:horizontal {{
    background: {ACCENT};
    border: 2px solid {ACCENT_LIGHT};
    width: 18px;
    height: 18px;
    margin: -7px 0;
    border-radius: 10px;
}}
QSlider::sub-page:horizontal {{
    background: qlineargradient(x1:0, y1:0, x2:1, y2:0,
        stop:0 {ACCENT_DIM}, stop:1 {ACCENT});
    border-radius: 3px;
}}
"""


def _card_shadow():
    effect = QGraphicsDropShadowEffect()
    effect.setBlurRadius(24)
    effect.setOffset(0, 4)
    effect.setColor(QColor(0, 0, 0, 90))
    return effect


def _make_label(text, size=13, bold=False, color=TEXT, align=Qt.AlignLeft):
    lbl = QLabel(text)
    f = QFont()
    f.setPointSize(size)
    f.setBold(bold)
    lbl.setFont(f)
    lbl.setStyleSheet(f"color: {color};")
    lbl.setAlignment(align)
    return lbl


class DirectionButton(QPushButton):
    """A single direction pad button that scales with window size."""

    def __init__(self, symbol: str, parent=None):
        super().__init__(symbol, parent)
        self.setMinimumSize(48, 48)
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self._pressed = False
        self._update_style(False)

    def _update_style(self, pressed: bool):
        self._pressed = pressed
        bg = ACCENT if pressed else BG_CARD
        border = ACCENT_LIGHT if pressed else BORDER
        text_c = "#fff" if pressed else TEXT
        self.setStyleSheet(f"""
            QPushButton {{
                background-color: {bg};
                color: {text_c};
                border: 2px solid {border};
                border-radius: 14px;
            }}
            QPushButton:hover {{
                background-color: {ACCENT_DIM};
                border-color: {ACCENT};
            }}
        """)

    def set_active(self, active: bool):
        self._update_style(active)

    def resizeEvent(self, event):
        super().resizeEvent(event)
        # scale font to ~30% of button height
        size = max(10, int(self.height() * 0.30))
        self.setFont(QFont("Ubuntu", size, QFont.Bold))


class StopButton(QPushButton):
    """A responsive STOP button that scales with window size."""

    def __init__(self, parent=None):
        super().__init__("STOP", parent)
        self.setMinimumSize(48, 48)
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self.setStyleSheet(f"""
            QPushButton {{
                background-color: {BG_CARD};
                color: {RED};
                border: 2px solid {RED};
                border-radius: 14px;
            }}
            QPushButton:hover {{
                background-color: {RED};
                color: #fff;
            }}
        """)

    def resizeEvent(self, event):
        super().resizeEvent(event)
        size = max(8, int(self.height() * 0.22))
        self.setFont(QFont("Ubuntu", size, QFont.Bold))


class SpeedGauge(QWidget):
    """An arc gauge widget that scales with window size."""

    def __init__(self, label: str, max_val: float, unit: str, color: str, parent=None):
        super().__init__(parent)
        self._label = label
        self._max = max_val
        self._unit = unit
        self._color = color
        self._value = 0.0
        self.setMinimumSize(80, 80)
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)

    def set_value(self, v: float):
        self._value = v
        self.update()

    def paintEvent(self, _event):
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)
        w, h = self.width(), self.height()
        side = min(w, h)
        cx, cy = w // 2, h // 2 + int(side * 0.04)
        r = int(side * 0.36)
        pen_w = max(3, int(side * 0.05))

        # background arc
        pen = QPen(QColor(BORDER), pen_w, Qt.SolidLine, Qt.RoundCap)
        p.setPen(pen)
        p.drawArc(cx - r, cy - r, 2 * r, 2 * r, 225 * 16, -270 * 16)

        # value arc
        frac = min(abs(self._value) / self._max, 1.0) if self._max else 0
        color = QColor(self._color)
        pen.setColor(color)
        pen.setWidth(pen_w)
        p.setPen(pen)
        span = int(-270 * frac * 16)
        p.drawArc(cx - r, cy - r, 2 * r, 2 * r, 225 * 16, span)

        # centre text — value
        val_size = max(8, int(side * 0.13))
        p.setPen(QColor(TEXT))
        f = QFont("Ubuntu", val_size, QFont.Bold)
        p.setFont(f)
        sign = "-" if self._value < 0 else ""
        p.drawText(0, cy - int(val_size * 0.8), w, int(val_size * 1.6),
                    Qt.AlignCenter, f"{sign}{abs(self._value):.2f}")

        # unit
        unit_size = max(6, int(side * 0.07))
        f.setPointSize(unit_size)
        f.setBold(False)
        p.setFont(f)
        p.setPen(QColor(TEXT_DIM))
        p.drawText(0, cy + int(val_size * 0.5), w, int(unit_size * 1.8),
                    Qt.AlignCenter, self._unit)

        # label at top
        lbl_size = max(6, int(side * 0.06))
        f.setPointSize(lbl_size)
        p.setFont(f)
        p.setPen(QColor(ACCENT_LIGHT))
        p.drawText(0, 2, w, int(lbl_size * 2), Qt.AlignCenter, self._label.upper())
        p.end()


class OdomWidget(QWidget):
    """Shows odometry pose in a compact card."""

    def __init__(self, parent=None):
        super().__init__(parent)
        grid = QGridLayout(self)
        grid.setSpacing(6)
        self._labels = {}
        for i, (key, icon) in enumerate([("x", "X"), ("y", "Y"), ("yaw", "θ")]):
            lbl_name = _make_label(icon, size=11, bold=True, color=ACCENT_LIGHT)
            lbl_val = _make_label("0.000", size=13, bold=True, color=TEXT,
                                   align=Qt.AlignRight)
            unit = _make_label("m" if key != "yaw" else "°", size=10, color=TEXT_DIM)
            grid.addWidget(lbl_name, i, 0)
            grid.addWidget(lbl_val, i, 1)
            grid.addWidget(unit, i, 2)
            self._labels[key] = lbl_val

    def update_odom(self, x: float, y: float, yaw_deg: float):
        self._labels["x"].setText(f"{x:.3f}")
        self._labels["y"].setText(f"{y:.3f}")
        self._labels["yaw"].setText(f"{yaw_deg:.1f}")


class RosSignals(QObject):
    """Bridge between ROS callbacks (threads) and Qt (main thread)."""
    odom_received = pyqtSignal(float, float, float, float, float)
    imu_received = pyqtSignal(float, float, float)


class TeleopNode(Node):
    """ROS 2 node for publishing Twist and subscribing to odometry/IMU."""

    def __init__(self, signals: RosSignals):
        super().__init__('vector_teleop_gui')
        self.signals = signals

        self.pub_cmd = self.create_publisher(
            Twist, '/diff_drive_controller/cmd_vel_unstamped', 10)

        self.sub_odom = self.create_subscription(
            Odometry, '/diff_drive_controller/odom', self._odom_cb, 10)

        self.sub_imu = self.create_subscription(
            Imu, '/imu', self._imu_cb, 10)

        self.get_logger().info('Teleop GUI node started')

    def publish_twist(self, linear: float, angular: float):
        msg = Twist()
        msg.linear.x = linear
        msg.angular.z = angular
        self.pub_cmd.publish(msg)

    def _odom_cb(self, msg: Odometry):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        # quaternion → yaw
        q = msg.pose.pose.orientation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny, cosy)
        vx = msg.twist.twist.linear.x
        wz = msg.twist.twist.angular.z
        self.signals.odom_received.emit(x, y, math.degrees(yaw), vx, wz)

    def _imu_cb(self, msg: Imu):
        self.signals.imu_received.emit(
            msg.linear_acceleration.x,
            msg.linear_acceleration.y,
            msg.linear_acceleration.z,
        )


class ToggleSwitch(QWidget):
    """A beautiful animated toggle switch widget."""
    toggled_signal = pyqtSignal(bool)

    def __init__(self, parent=None):
        super().__init__(parent)
        self._on = False
        self._knob_x = 4.0
        self.setFixedSize(52, 28)
        self.setCursor(Qt.PointingHandCursor)

        self._anim_timer = QTimer(self)
        self._anim_timer.setInterval(16)
        self._anim_timer.timeout.connect(self._animate)
        self._target_x = 4.0

    def is_on(self) -> bool:
        return self._on

    def mousePressEvent(self, _event):
        self._on = not self._on
        self._target_x = 26.0 if self._on else 4.0
        self._anim_timer.start()
        self.toggled_signal.emit(self._on)

    def _animate(self):
        diff = self._target_x - self._knob_x
        if abs(diff) < 0.5:
            self._knob_x = self._target_x
            self._anim_timer.stop()
        else:
            self._knob_x += diff * 0.3
        self.update()

    def paintEvent(self, _event):
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)

        # track
        track_color = QColor(ACCENT) if self._on else QColor(BORDER)
        p.setPen(Qt.NoPen)
        p.setBrush(QBrush(track_color))
        p.drawRoundedRect(0, 0, 52, 28, 14, 14)

        # knob
        p.setBrush(QBrush(QColor("#ffffff")))
        p.drawEllipse(int(self._knob_x), 4, 20, 20)
        p.end()


class TeleopWindow(QMainWindow):
    """Main teleop GUI window."""

    def __init__(self, node: TeleopNode, signals: RosSignals):
        super().__init__()
        self.node = node
        self.setWindowTitle("VECTOR NAV  ·  Teleop Control")
        self.setMinimumSize(780, 520)
        self.resize(820, 560)
        self.setStyleSheet(STYLESHEET)

        # ── state ───────────────────────────────────────────────────
        self._keys_pressed: set = set()
        self._max_linear = 0.5   # m/s
        self._max_angular = 1.5  # rad/s
        self._cur_lin = 0.0
        self._cur_ang = 0.0
        self._connected = False
        self._toggle_mode = False          # False = HOLD, True = TOGGLE
        self._toggle_lin = 0.0             # latched direction in toggle mode
        self._toggle_ang = 0.0

        # ── signals ─────────────────────────────────────────────────
        signals.odom_received.connect(self._on_odom)
        signals.imu_received.connect(self._on_imu)

        # ── central widget ──────────────────────────────────────────
        central = QWidget()
        self.setCentralWidget(central)
        root = QVBoxLayout(central)
        root.setContentsMargins(20, 12, 20, 16)
        root.setSpacing(14)

        # header
        header = QHBoxLayout()
        title = _make_label("VECTOR NAV", size=18, bold=True, color=ACCENT_LIGHT)
        subtitle = _make_label("Teleoperation Control", size=12, color=TEXT_DIM)
        self._status_dot = _make_label("●", size=14, color=TEXT_DIM)
        self._status_text = _make_label("Waiting…", size=11, color=TEXT_DIM)
        header.addWidget(title)
        header.addWidget(subtitle)
        header.addStretch()
        header.addWidget(self._status_dot)
        header.addWidget(self._status_text)
        root.addLayout(header)

        # separator
        sep = QFrame()
        sep.setFrameShape(QFrame.HLine)
        sep.setStyleSheet(f"color: {BORDER};")
        root.addWidget(sep)

        # ── body ────────────────────────────────────────────────────
        body = QHBoxLayout()
        body.setSpacing(16)

        # LEFT: D-Pad + speed sliders
        left = QVBoxLayout()
        left.setSpacing(14)

        # D-pad
        dpad_box = QGroupBox("CONTROLS")
        dpad_box.setGraphicsEffect(_card_shadow())
        dpad_layout = QGridLayout()
        dpad_layout.setSpacing(6)
        dpad_layout.setContentsMargins(16, 12, 16, 12)
        for i in range(3):
            dpad_layout.setColumnStretch(i, 1)
            dpad_layout.setRowStretch(i, 1)

        self._btn_fwd_left = DirectionButton("↰")
        self._btn_fwd = DirectionButton("▲")
        self._btn_fwd_right = DirectionButton("↱")
        self._btn_left = DirectionButton("◀")
        self._btn_stop = StopButton()
        self._btn_right = DirectionButton("▶")
        self._btn_back_left = DirectionButton("↲")
        self._btn_back = DirectionButton("▼")
        self._btn_back_right = DirectionButton("↳")
        self._btn_stop.clicked.connect(self._emergency_stop)

        # row 0: forward-left, forward, forward-right
        dpad_layout.addWidget(self._btn_fwd_left, 0, 0)
        dpad_layout.addWidget(self._btn_fwd, 0, 1)
        dpad_layout.addWidget(self._btn_fwd_right, 0, 2)
        # row 1: left, stop, right
        dpad_layout.addWidget(self._btn_left, 1, 0)
        dpad_layout.addWidget(self._btn_stop, 1, 1)
        dpad_layout.addWidget(self._btn_right, 1, 2)
        # row 2: back-left, back, back-right
        dpad_layout.addWidget(self._btn_back_left, 2, 0)
        dpad_layout.addWidget(self._btn_back, 2, 1)
        dpad_layout.addWidget(self._btn_back_right, 2, 2)

        # mouse press/release for all direction buttons
        for btn, lin, ang in [
            (self._btn_fwd_left, 1.0, 1.0),    # U: forward + turn left
            (self._btn_fwd, 1.0, 0.0),          # W: forward
            (self._btn_fwd_right, 1.0, -1.0),   # O: forward + turn right
            (self._btn_left, 0.0, 1.0),          # A: turn left
            (self._btn_right, 0.0, -1.0),        # D: turn right
            (self._btn_back_left, -1.0, -1.0),   # J: back + turn right
            (self._btn_back, -1.0, 0.0),         # S: back
            (self._btn_back_right, -1.0, 1.0),   # L: back + turn left
        ]:
            btn.pressed.connect(lambda l=lin, a=ang: self._on_btn_press(l, a))
            btn.released.connect(self._on_btn_release)

        keys_hint = _make_label("U I O / J K L  or  W A S D", size=10,
                                 color=TEXT_DIM, align=Qt.AlignCenter)
        dpad_layout.addWidget(keys_hint, 3, 0, 1, 3)
        dpad_box.setLayout(dpad_layout)
        left.addWidget(dpad_box)

        # mode switch (HOLD / TOGGLE)
        mode_box = QGroupBox("INPUT MODE")
        mode_box.setGraphicsEffect(_card_shadow())
        mode_layout = QHBoxLayout()
        mode_layout.setContentsMargins(14, 8, 14, 8)
        self._mode_hold_lbl = _make_label("HOLD", size=11, bold=True, color=ACCENT_LIGHT)
        self._mode_toggle_lbl = _make_label("TOGGLE", size=11, bold=False, color=TEXT_DIM)
        self._mode_switch = ToggleSwitch()
        self._mode_switch.toggled_signal.connect(self._on_mode_switch)
        mode_layout.addStretch()
        mode_layout.addWidget(self._mode_hold_lbl)
        mode_layout.addWidget(self._mode_switch)
        mode_layout.addWidget(self._mode_toggle_lbl)
        mode_layout.addStretch()
        mode_box.setLayout(mode_layout)
        left.addWidget(mode_box)

        # sliders
        slider_box = QGroupBox("SPEED LIMITS")
        slider_box.setGraphicsEffect(_card_shadow())
        sl = QVBoxLayout()
        sl.setSpacing(10)

        sl.addWidget(_make_label("Linear (m/s)", size=11, color=TEXT_DIM))
        self._sl_lin = QSlider(Qt.Horizontal)
        self._sl_lin.setRange(5, 100)
        self._sl_lin.setValue(int(self._max_linear * 100))
        self._sl_lin_lbl = _make_label(f"{self._max_linear:.2f}", size=12,
                                        bold=True, color=ACCENT_LIGHT,
                                        align=Qt.AlignRight)
        row_lin = QHBoxLayout()
        row_lin.addWidget(self._sl_lin)
        row_lin.addWidget(self._sl_lin_lbl)
        sl.addLayout(row_lin)

        sl.addWidget(_make_label("Angular (rad/s)", size=11, color=TEXT_DIM))
        self._sl_ang = QSlider(Qt.Horizontal)
        self._sl_ang.setRange(10, 200)
        self._sl_ang.setValue(int(self._max_angular * 100))
        self._sl_ang_lbl = _make_label(f"{self._max_angular:.2f}", size=12,
                                        bold=True, color=ACCENT_LIGHT,
                                        align=Qt.AlignRight)
        row_ang = QHBoxLayout()
        row_ang.addWidget(self._sl_ang)
        row_ang.addWidget(self._sl_ang_lbl)
        sl.addLayout(row_ang)

        self._sl_lin.valueChanged.connect(self._on_linear_slider)
        self._sl_ang.valueChanged.connect(self._on_angular_slider)

        slider_box.setLayout(sl)
        left.addWidget(slider_box)

        # RIGHT: gauges + odom + IMU
        right = QVBoxLayout()
        right.setSpacing(14)

        # gauges
        gauge_box = QGroupBox("VELOCITY")
        gauge_box.setGraphicsEffect(_card_shadow())
        gl = QHBoxLayout()
        gl.setContentsMargins(10, 8, 10, 8)
        self._gauge_lin = SpeedGauge("Linear", 1.0, "m/s", ACCENT)
        self._gauge_ang = SpeedGauge("Angular", 2.0, "rad/s", GREEN)
        gl.addWidget(self._gauge_lin, alignment=Qt.AlignCenter)
        gl.addWidget(self._gauge_ang, alignment=Qt.AlignCenter)
        gauge_box.setLayout(gl)
        right.addWidget(gauge_box)

        # odom
        odom_box = QGroupBox("ODOMETRY")
        odom_box.setGraphicsEffect(_card_shadow())
        ol = QVBoxLayout()
        self._odom_widget = OdomWidget()
        ol.addWidget(self._odom_widget)
        odom_box.setLayout(ol)
        right.addWidget(odom_box)

        # IMU
        imu_box = QGroupBox("IMU  (m/s²)")
        imu_box.setGraphicsEffect(_card_shadow())
        il = QHBoxLayout()
        self._imu_labels = {}
        for axis, col in [("X", ACCENT_LIGHT), ("Y", GREEN), ("Z", YELLOW)]:
            v = QVBoxLayout()
            v.addWidget(_make_label(axis, size=10, bold=True, color=col,
                                     align=Qt.AlignCenter))
            val = _make_label("0.00", size=13, bold=True, align=Qt.AlignCenter)
            v.addWidget(val)
            self._imu_labels[axis] = val
            il.addLayout(v)
        imu_box.setLayout(il)
        right.addWidget(imu_box)

        body.addLayout(left, stretch=5)
        body.addLayout(right, stretch=5)
        root.addLayout(body)

        # ── publish timer (20 Hz) ───────────────────────────────────
        self._pub_timer = QTimer()
        self._pub_timer.setInterval(50)
        self._pub_timer.timeout.connect(self._publish)
        self._pub_timer.start()

    # ── keyboard ────────────────────────────────────────────────────
    def keyPressEvent(self, event: QKeyEvent):
        if event.isAutoRepeat():
            return
        key = event.key()
        if self._toggle_mode:
            self._handle_toggle_key(key)
        else:
            self._keys_pressed.add(key)
            self._update_from_keys()

    def keyReleaseEvent(self, event: QKeyEvent):
        if event.isAutoRepeat():
            return
        if not self._toggle_mode:
            self._keys_pressed.discard(event.key())
            self._update_from_keys()

    def _key_to_direction(self, key):
        """Map a key to (lin_sign, ang_sign) or None."""
        mapping = {
            Qt.Key_U: (1.0, 1.0),
            Qt.Key_I: (1.0, 0.0), Qt.Key_W: (1.0, 0.0), Qt.Key_Up: (1.0, 0.0),
            Qt.Key_O: (1.0, -1.0),
            Qt.Key_A: (0.0, 1.0), Qt.Key_Left: (0.0, 1.0),
            Qt.Key_D: (0.0, -1.0), Qt.Key_Right: (0.0, -1.0),
            Qt.Key_J: (-1.0, -1.0),
            Qt.Key_K: (-1.0, 0.0), Qt.Key_S: (-1.0, 0.0), Qt.Key_Down: (-1.0, 0.0),
            Qt.Key_L: (-1.0, 1.0),
        }
        return mapping.get(key)

    def _handle_toggle_key(self, key):
        """In toggle mode, pressing a key latches that direction; pressing
        the same direction again (or space/stop) cancels it."""
        direction = self._key_to_direction(key)
        if direction is None:
            # Space acts as stop in toggle mode
            if key == Qt.Key_Space:
                self._toggle_lin = 0.0
                self._toggle_ang = 0.0
            return
        lin, ang = direction
        # If same direction is already active, stop
        if lin == self._toggle_lin and ang == self._toggle_ang:
            self._toggle_lin = 0.0
            self._toggle_ang = 0.0
        else:
            self._toggle_lin = lin
            self._toggle_ang = ang
        self._apply_toggle()

    def _apply_toggle(self):
        lin = self._toggle_lin
        ang = self._toggle_ang
        self._cur_lin = lin * self._max_linear
        self._cur_ang = ang * self._max_angular
        self._highlight_buttons(lin, ang)

    def _update_from_keys(self):
        lin = 0.0
        ang = 0.0
        k = self._keys_pressed

        # Diagonal keys (like teleop_twist_keyboard)
        if Qt.Key_U in k:       # forward + turn left
            lin += 1.0
            ang += 1.0
        if Qt.Key_O in k:       # forward + turn right
            lin += 1.0
            ang -= 1.0
        if Qt.Key_J in k:       # back + turn right
            lin -= 1.0
            ang -= 1.0
        if Qt.Key_L in k:       # back + turn left
            lin -= 1.0
            ang += 1.0

        # Cardinal keys
        if Qt.Key_I in k or Qt.Key_W in k or Qt.Key_Up in k:
            lin += 1.0
        if Qt.Key_K in k or Qt.Key_S in k or Qt.Key_Down in k:
            lin -= 1.0
        if Qt.Key_A in k or Qt.Key_Left in k:
            ang += 1.0
        if Qt.Key_D in k or Qt.Key_Right in k:
            ang -= 1.0

        # Clamp to -1..1 before scaling
        lin = max(-1.0, min(1.0, lin))
        ang = max(-1.0, min(1.0, ang))

        self._cur_lin = lin * self._max_linear
        self._cur_ang = ang * self._max_angular
        self._highlight_buttons(lin, ang)

    def _highlight_buttons(self, lin: float, ang: float):
        self._btn_fwd.set_active(lin > 0 and ang == 0)
        self._btn_back.set_active(lin < 0 and ang == 0)
        self._btn_left.set_active(ang > 0 and lin == 0)
        self._btn_right.set_active(ang < 0 and lin == 0)
        self._btn_fwd_left.set_active(lin > 0 and ang > 0)
        self._btn_fwd_right.set_active(lin > 0 and ang < 0)
        self._btn_back_left.set_active(lin < 0 and ang < 0)
        self._btn_back_right.set_active(lin < 0 and ang > 0)

    # ── mouse button helpers ────────────────────────────────────────
    def _on_btn_press(self, lin_sign: float, ang_sign: float):
        if self._toggle_mode:
            # Click acts as toggle: same direction → stop, new → latch
            if lin_sign == self._toggle_lin and ang_sign == self._toggle_ang:
                self._toggle_lin = 0.0
                self._toggle_ang = 0.0
            else:
                self._toggle_lin = lin_sign
                self._toggle_ang = ang_sign
            self._apply_toggle()
        else:
            self._cur_lin = lin_sign * self._max_linear
            self._cur_ang = ang_sign * self._max_angular

    def _on_btn_release(self):
        if self._toggle_mode:
            return  # toggle mode keeps running, don't clear on release
        if not self._keys_pressed:
            self._cur_lin = 0.0
            self._cur_ang = 0.0
            self._clear_all_buttons()

    def _emergency_stop(self):
        self._cur_lin = 0.0
        self._cur_ang = 0.0
        self._toggle_lin = 0.0
        self._toggle_ang = 0.0
        self._keys_pressed.clear()
        self._clear_all_buttons()
        self.node.publish_twist(0.0, 0.0)

    def _clear_all_buttons(self):
        for b in (self._btn_fwd, self._btn_back, self._btn_left, self._btn_right,
                  self._btn_fwd_left, self._btn_fwd_right,
                  self._btn_back_left, self._btn_back_right):
            b.set_active(False)

    # ── mode switch ───────────────────────────────────────────────
    def _on_mode_switch(self, is_toggle: bool):
        self._toggle_mode = is_toggle
        if is_toggle:
            self._mode_hold_lbl.setStyleSheet(f"color: {TEXT_DIM};")
            self._mode_hold_lbl.setFont(QFont("Ubuntu", 11, QFont.Normal))
            self._mode_toggle_lbl.setStyleSheet(f"color: {ACCENT_LIGHT};")
            self._mode_toggle_lbl.setFont(QFont("Ubuntu", 11, QFont.Bold))
        else:
            self._mode_hold_lbl.setStyleSheet(f"color: {ACCENT_LIGHT};")
            self._mode_hold_lbl.setFont(QFont("Ubuntu", 11, QFont.Bold))
            self._mode_toggle_lbl.setStyleSheet(f"color: {TEXT_DIM};")
            self._mode_toggle_lbl.setFont(QFont("Ubuntu", 11, QFont.Normal))
        # Reset motion when switching modes
        self._emergency_stop()

    # ── sliders ─────────────────────────────────────────────────────
    def _on_linear_slider(self, val: int):
        self._max_linear = val / 100.0
        self._sl_lin_lbl.setText(f"{self._max_linear:.2f}")
        self._gauge_lin._max = self._max_linear

    def _on_angular_slider(self, val: int):
        self._max_angular = val / 100.0
        self._sl_ang_lbl.setText(f"{self._max_angular:.2f}")
        self._gauge_ang._max = self._max_angular

    # ── publish loop ────────────────────────────────────────────────
    def _publish(self):
        self.node.publish_twist(self._cur_lin, self._cur_ang)
        self._gauge_lin.set_value(self._cur_lin)
        self._gauge_ang.set_value(self._cur_ang)

    # ── ROS callbacks (via signals) ─────────────────────────────────
    def _on_odom(self, x: float, y: float, yaw: float, vx: float, wz: float):
        self._odom_widget.update_odom(x, y, yaw)
        if not self._connected:
            self._connected = True
            self._status_dot.setStyleSheet(f"color: {GREEN};")
            self._status_text.setText("Connected")
            self._status_text.setStyleSheet(f"color: {GREEN};")

    def _on_imu(self, ax: float, ay: float, az: float):
        self._imu_labels["X"].setText(f"{ax:.2f}")
        self._imu_labels["Y"].setText(f"{ay:.2f}")
        self._imu_labels["Z"].setText(f"{az:.2f}")


def main(args=None):
    rclpy.init(args=args)

    # allow Ctrl-C to kill gracefully
    signal.signal(signal.SIGINT, signal.SIG_DFL)

    app = QApplication(sys.argv)
    app.setStyle("Fusion")

    signals = RosSignals()
    node = TeleopNode(signals)

    # spin ROS in a background thread
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    window = TeleopWindow(node, signals)
    window.show()

    exit_code = app.exec_()

    node.destroy_node()
    rclpy.shutdown()
    sys.exit(exit_code)


if __name__ == '__main__':
    main()
