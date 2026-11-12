#!/usr/bin/env python3
"""PyQt5 teleoperation and live telemetry console for the planar arm."""

from collections import deque
import math
import threading
import time

import rclpy
from geometry_msgs.msg import PointStamped
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray, String

from PyQt5 import QtCore, QtGui, QtWidgets
import pyqtgraph as pg

from planar_arm_control.planar_arm import PlanarArm

LINK_LENGTHS = [3.0, 2.0, 1.5]
JOINT_NAMES = ["joint1", "joint2", "joint3"]
JOINT_COLORS = ["#4da6ff", "#ffb454", "#7ee787"]
STALE_AFTER_S = 1.0
HISTORY_LEN = 400

# ----------------------------------------------------------------- theme --
COLORS = {
    "bg": "#0d1117",
    "panel": "#161b22",
    "panel_alt": "#1c232c",
    "border": "#2d333b",
    "text": "#e6edf3",
    "muted": "#8b949e",
    "accent": "#2f81f7",
    "accent_hover": "#4c97ff",
    "ok": "#3fb950",
    "warn": "#d29922",
    "bad": "#f85149",
}

STYLESHEET = f"""
* {{
    font-family: "Segoe UI", "Inter", "Helvetica Neue", Arial, sans-serif;
    font-size: 13px;
    color: {COLORS['text']};
}}
QMainWindow, QWidget#root {{ background: {COLORS['bg']}; }}
QFrame#header {{
    background: {COLORS['panel']};
    border-bottom: 1px solid {COLORS['border']};
}}
QLabel#title {{ font-size: 17px; font-weight: 600; }}
QLabel#subtitle {{ color: {COLORS['muted']}; font-size: 12px; }}
QFrame#card {{
    background: {COLORS['panel']};
    border: 1px solid {COLORS['border']};
    border-radius: 8px;
}}
QLabel#cardTitle {{
    color: {COLORS['muted']};
    font-size: 11px;
    font-weight: 600;
    letter-spacing: 1px;
}}
QLabel#metricValue {{
    font-family: "Consolas", "JetBrains Mono", "Menlo", monospace;
    font-size: 20px;
    font-weight: 600;
}}
QLabel#metricName {{ color: {COLORS['muted']}; font-size: 11px; }}
QLabel#mono {{
    font-family: "Consolas", "JetBrains Mono", "Menlo", monospace;
    color: {COLORS['muted']};
}}
QLabel#hint {{ color: {COLORS['muted']}; font-size: 12px; }}
QTabWidget::pane {{
    border: 1px solid {COLORS['border']};
    border-radius: 8px;
    background: {COLORS['panel']};
    top: -1px;
}}
QTabBar::tab {{
    background: transparent;
    color: {COLORS['muted']};
    padding: 8px 18px;
    border: none;
    border-bottom: 2px solid transparent;
    font-weight: 600;
}}
QTabBar::tab:selected {{
    color: {COLORS['text']};
    border-bottom: 2px solid {COLORS['accent']};
}}
QTabBar::tab:hover:!selected {{ color: {COLORS['text']}; }}
QDoubleSpinBox, QComboBox {{
    background: {COLORS['panel_alt']};
    border: 1px solid {COLORS['border']};
    border-radius: 5px;
    padding: 4px 8px;
    min-height: 22px;
    selection-background-color: {COLORS['accent']};
}}
QDoubleSpinBox:focus, QComboBox:focus {{ border: 1px solid {COLORS['accent']}; }}
QComboBox QAbstractItemView {{
    background: {COLORS['panel_alt']};
    border: 1px solid {COLORS['border']};
    selection-background-color: {COLORS['accent']};
}}
QPushButton {{
    background: {COLORS['panel_alt']};
    border: 1px solid {COLORS['border']};
    border-radius: 6px;
    padding: 7px 14px;
    font-weight: 600;
}}
QPushButton:hover {{ border-color: {COLORS['muted']}; }}
QPushButton:pressed {{ background: {COLORS['border']}; }}
QPushButton#primary {{
    background: {COLORS['accent']};
    border: 1px solid {COLORS['accent']};
    color: white;
}}
QPushButton#primary:hover {{ background: {COLORS['accent_hover']}; }}
QPushButton:disabled {{ color: {COLORS['muted']}; }}
QPushButton#jog {{
    min-width: 44px; min-height: 36px; padding: 0;
    font-size: 16px;
}}
QSlider::groove:horizontal {{
    height: 4px; background: {COLORS['border']}; border-radius: 2px;
}}
QSlider::sub-page:horizontal {{
    background: {COLORS['accent']}; border-radius: 2px;
}}
QSlider::handle:horizontal {{
    background: {COLORS['text']}; width: 14px; height: 14px;
    margin: -6px 0; border-radius: 7px;
}}
QStatusBar {{
    background: {COLORS['panel']};
    border-top: 1px solid {COLORS['border']};
    color: {COLORS['muted']};
}}
QStatusBar::item {{ border: none; }}
QToolTip {{
    background: {COLORS['panel_alt']};
    color: {COLORS['text']};
    border: 1px solid {COLORS['border']};
}}
"""


def apply_dark_theme(application: QtWidgets.QApplication) -> None:
    """Use the Fusion style with a dark palette plus the app stylesheet."""
    application.setStyle("Fusion")
    palette = QtGui.QPalette()
    qc = QtGui.QColor
    palette.setColor(QtGui.QPalette.Window, qc(COLORS["bg"]))
    palette.setColor(QtGui.QPalette.WindowText, qc(COLORS["text"]))
    palette.setColor(QtGui.QPalette.Base, qc(COLORS["panel_alt"]))
    palette.setColor(QtGui.QPalette.AlternateBase, qc(COLORS["panel"]))
    palette.setColor(QtGui.QPalette.Text, qc(COLORS["text"]))
    palette.setColor(QtGui.QPalette.Button, qc(COLORS["panel_alt"]))
    palette.setColor(QtGui.QPalette.ButtonText, qc(COLORS["text"]))
    palette.setColor(QtGui.QPalette.Highlight, qc(COLORS["accent"]))
    palette.setColor(QtGui.QPalette.HighlightedText, qc("#ffffff"))
    application.setPalette(palette)
    application.setStyleSheet(STYLESHEET)
    pg.setConfigOptions(
        antialias=True,
        background=COLORS["panel"],
        foreground=COLORS["muted"],
    )


# ------------------------------------------------------------------ node --
class GuiNode(Node):
    """Receive arm telemetry and publish operator motion goals."""

    def __init__(self):
        """Create target publishers and thread-safe telemetry subscriptions."""
        super().__init__("gui_node")
        self.arm = PlanarArm(LINK_LENGTHS)
        self._state_lock = threading.Lock()
        self._positions = [0.0, 0.0, 0.0]
        self._status = "Waiting for controller"
        self._last_state_time = None
        self._joint_target_publisher = self.create_publisher(
            Float64MultiArray, "/joint_target", 10
        )
        self._xy_target_publisher = self.create_publisher(
            PointStamped, "/target_pose", 10
        )
        self.create_subscription(
            JointState, "/joint_states", self._on_joint_state, 10
        )
        self.create_subscription(
            String, "/controller_status", self._on_status, 10
        )

    def _on_joint_state(self, message: JointState) -> None:
        by_name = dict(zip(message.name, message.position))
        if not all(name in by_name for name in JOINT_NAMES):
            return
        positions = [float(by_name[name]) for name in JOINT_NAMES]
        if not all(math.isfinite(value) for value in positions):
            return
        with self._state_lock:
            self._positions = positions
            self._last_state_time = time.monotonic()

    def _on_status(self, message: String) -> None:
        with self._state_lock:
            self._status = message.data

    def snapshot(self):
        """Return the latest positions and controller status consistently."""
        with self._state_lock:
            return list(self._positions), self._status

    def telemetry_age(self):
        """Return seconds since the last joint state, or None if never."""
        with self._state_lock:
            if self._last_state_time is None:
                return None
            return time.monotonic() - self._last_state_time

    def publish_xy_target(self, x: float, y: float) -> None:
        """Publish a planar target, mapping planner Y to robot-frame Z."""
        message = PointStamped()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = "base_link"
        message.point.x = float(x)
        message.point.z = float(y)
        self._xy_target_publisher.publish(message)

    def publish_joint_target(self, positions: list[float]) -> None:
        """Publish a three-joint target in radians."""
        self._joint_target_publisher.publish(
            Float64MultiArray(data=[float(value) for value in positions])
        )


# --------------------------------------------------------------- widgets --
def make_card(title: str):
    """Return (frame, body_layout) for a titled dashboard card."""
    frame = QtWidgets.QFrame()
    frame.setObjectName("card")
    outer = QtWidgets.QVBoxLayout(frame)
    outer.setContentsMargins(14, 12, 14, 14)
    outer.setSpacing(10)
    label = QtWidgets.QLabel(title.upper())
    label.setObjectName("cardTitle")
    outer.addWidget(label)
    return frame, outer


class StatusPill(QtWidgets.QLabel):
    """Rounded label whose colour reflects a state."""

    def __init__(self, text=""):
        super().__init__(text)
        self.setAlignment(QtCore.Qt.AlignCenter)
        self.set_state(text, COLORS["muted"])

    def set_state(self, text: str, color: str) -> None:
        """Update the text and accent colour of the pill."""
        self.setText(f"●  {text}")
        self.setStyleSheet(
            f"background: transparent; color: {color};"
            f" border-radius: 12px;"
            "padding: 4px 12px; font-weight: 600;"
        )


class MetricTile(QtWidgets.QWidget):
    """Small name/value readout used for live numbers."""

    def __init__(self, name: str, accent: str = None):
        super().__init__()
        layout = QtWidgets.QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(0)
        self._name = QtWidgets.QLabel(name)
        self._name.setObjectName("metricName")
        self._value = QtWidgets.QLabel("--")
        self._value.setObjectName("metricValue")
        if accent:
            self._value.setStyleSheet(f"color: {accent};")
        layout.addWidget(self._name)
        layout.addWidget(self._value)

    def set_value(self, text: str) -> None:
        """Set the displayed value."""
        self._value.setText(text)


class JointRow(QtWidgets.QWidget):
    """Slider + spin box for one joint with a live-position readout."""

    changed = QtCore.pyqtSignal()

    def __init__(self, index: int, lower_deg: float, upper_deg: float):
        super().__init__()
        self._scale = 10  # slider ticks per degree
        layout = QtWidgets.QGridLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setHorizontalSpacing(10)

        name = QtWidgets.QLabel(f"Joint {index + 1}")
        name.setStyleSheet(f"color: {JOINT_COLORS[index]}; font-weight: 600;")
        self.actual = QtWidgets.QLabel("actual --")
        self.actual.setObjectName("mono")
        self.actual.setAlignment(QtCore.Qt.AlignRight)

        self.slider = QtWidgets.QSlider(QtCore.Qt.Horizontal)
        self.slider.setRange(
            int(round(lower_deg * self._scale)),
            int(round(upper_deg * self._scale)),
        )
        self.spin = QtWidgets.QDoubleSpinBox()
        self.spin.setRange(lower_deg, upper_deg)
        self.spin.setDecimals(1)
        self.spin.setSingleStep(1.0)
        self.spin.setSuffix("°")
        self.spin.setFixedWidth(92)

        layout.addWidget(name, 0, 0)
        layout.addWidget(self.actual, 0, 1, 1, 2)
        layout.addWidget(self.slider, 1, 0, 1, 2)
        layout.addWidget(self.spin, 1, 2)
        layout.setColumnStretch(1, 1)

        self.slider.valueChanged.connect(self._from_slider)
        self.spin.valueChanged.connect(self._from_spin)

    def _from_slider(self, ticks: int) -> None:
        self.spin.blockSignals(True)
        self.spin.setValue(ticks / self._scale)
        self.spin.blockSignals(False)
        self.changed.emit()

    def _from_spin(self, degrees: float) -> None:
        self.slider.blockSignals(True)
        self.slider.setValue(int(round(degrees * self._scale)))
        self.slider.blockSignals(False)
        self.changed.emit()

    def value_deg(self) -> float:
        """Return the commanded angle in degrees."""
        return self.spin.value()

    def set_value_deg(self, degrees: float) -> None:
        """Set the commanded angle without emitting change signals."""
        self.spin.blockSignals(True)
        self.slider.blockSignals(True)
        self.spin.setValue(degrees)
        self.slider.setValue(int(round(self.spin.value() * self._scale)))
        self.spin.blockSignals(False)
        self.slider.blockSignals(False)

    def set_actual_deg(self, degrees: float) -> None:
        """Show the measured angle."""
        self.actual.setText(f"actual {degrees:7.1f}°")


# ---------------------------------------------------------------- window --
class ArmWindow(QtWidgets.QMainWindow):
    """Operator console: workspace view, controls, and live telemetry."""

    def __init__(self, node: GuiNode):
        """Build the console around a ROS node."""
        super().__init__()
        self.node = node
        self.arm = node.arm
        self.setWindowTitle("Planar Arm Console")
        self.resize(1360, 820)
        self.setMinimumSize(1100, 700)

        root = QtWidgets.QWidget()
        root.setObjectName("root")
        self.setCentralWidget(root)
        root_layout = QtWidgets.QVBoxLayout(root)
        root_layout.setContentsMargins(0, 0, 0, 0)
        root_layout.setSpacing(0)

        root_layout.addWidget(self._build_header())

        body = QtWidgets.QWidget()
        body_layout = QtWidgets.QHBoxLayout(body)
        body_layout.setContentsMargins(16, 16, 16, 16)
        body_layout.setSpacing(16)

        content_splitter = QtWidgets.QSplitter(QtCore.Qt.Horizontal)
        content_splitter.setChildrenCollapsible(False)

        left_splitter = QtWidgets.QSplitter(QtCore.Qt.Vertical)
        left_splitter.setChildrenCollapsible(False)
        left_splitter.addWidget(self._build_workspace_card())
        left_splitter.addWidget(self._build_telemetry_card())
        left_splitter.setStretchFactor(0, 3)
        left_splitter.setStretchFactor(1, 2)

        content_splitter.addWidget(left_splitter)
        content_splitter.addWidget(self._build_control_panel())
        content_splitter.setStretchFactor(0, 3)
        content_splitter.setStretchFactor(1, 2)

        content_splitter.setMinimumWidth(0)
        content_splitter.setSizePolicy(
            QtWidgets.QSizePolicy.Expanding,
            QtWidgets.QSizePolicy.Expanding,
        )
        left_splitter.setMinimumWidth(0)
        left_splitter.setSizePolicy(
            QtWidgets.QSizePolicy.Expanding,
            QtWidgets.QSizePolicy.Expanding,
        )
        body.setMinimumWidth(0)
        body_layout.addWidget(content_splitter)
        root_layout.addWidget(body, 1)

        self._build_status_bar()

        self._history_times = deque(maxlen=HISTORY_LEN)
        self._history_positions = [deque(maxlen=HISTORY_LEN) for _ in range(3)]
        self._trail = deque(maxlen=250)
        self._started_at = time.monotonic()
        self._last_plot_update = 0.0
        self._target_xy = None
        self._joint_fields_seeded = False
        self._last_logged_telemetry_state = None
        self._last_logged_controller_error = None

        self._timer = QtCore.QTimer(self)
        self._timer.timeout.connect(self._refresh)
        self._timer.start(50)

    # ---- construction helpers ------------------------------------------
    def _build_header(self) -> QtWidgets.QWidget:
        header = QtWidgets.QFrame()
        header.setObjectName("header")
        header.setFixedHeight(64)
        layout = QtWidgets.QHBoxLayout(header)
        layout.setContentsMargins(20, 0, 20, 0)

        titles = QtWidgets.QVBoxLayout()
        titles.setSpacing(0)
        title = QtWidgets.QLabel("Planar Arm Console")
        title.setObjectName("title")
        subtitle = QtWidgets.QLabel(
            "3-DOF teleoperation  ·  /joint_target  ·  /target_pose"
        )
        subtitle.setObjectName("subtitle")
        titles.addStretch(1)
        titles.addWidget(title)
        titles.addWidget(subtitle)
        titles.addStretch(1)
        layout.addLayout(titles)
        layout.addStretch(1)

        self.link_pill = StatusPill("Telemetry: Offline")
        self.link_pill.setMinimumWidth(170)
        layout.addWidget(self.link_pill)
        return header

    def _build_workspace_card(self) -> QtWidgets.QWidget:
        card, layout = make_card("Workspace  ·  click to place target")
        self.arm_plot = pg.PlotWidget()
        self.arm_plot.setAspectLocked(True)
        self.arm_plot.showGrid(x=True, y=True, alpha=0.18)
        self.arm_plot.setLabel("bottom", "X", units="m")
        self.arm_plot.setLabel("left", "Y", units="m")
        self.arm_plot.setMenuEnabled(False)
        reach = sum(LINK_LENGTHS)
        self.arm_plot.setXRange(-reach - 0.5, reach + 0.5)
        self.arm_plot.setYRange(-reach - 0.5, reach + 0.5)

        angles = [i * math.pi / 90.0 for i in range(181)]
        self.arm_plot.plot(
            [reach * math.cos(a) for a in angles],
            [reach * math.sin(a) for a in angles],
            pen=pg.mkPen(COLORS["border"], width=1.5, style=QtCore.Qt.DashLine),
        )
        self._trail_curve = self.arm_plot.plot(
            [], [], pen=pg.mkPen(COLORS["ok"] + "80", width=2)
        )
        self._target_marker = pg.ScatterPlotItem(
            size=16,
            symbol="crosshair",
            pen=pg.mkPen(COLORS["warn"], width=2),
            brush=pg.mkBrush(None),
        )
        self._target_marker.setZValue(5)
        self.arm_plot.addItem(self._target_marker)
        self.arm_curve = self.arm_plot.plot(
            [],
            [],
            pen=pg.mkPen(COLORS["accent"], width=7),
            symbol="o",
            symbolSize=12,
            symbolBrush=COLORS["text"],
            symbolPen=pg.mkPen(COLORS["accent"], width=2),
        )
        self._base_marker = pg.ScatterPlotItem(
            [0.0], [0.0], size=18, symbol="s",
            pen=pg.mkPen(COLORS["muted"]), brush=pg.mkBrush(COLORS["border"]),
        )
        self.arm_plot.addItem(self._base_marker)
        self._tip_marker = pg.ScatterPlotItem(
            size=14, symbol="d",
            pen=pg.mkPen(COLORS["bg"], width=1.5),
            brush=pg.mkBrush(COLORS["ok"]),
        )
        self._tip_marker.setZValue(6)
        self.arm_plot.addItem(self._tip_marker)

        self.arm_plot.scene().sigMouseClicked.connect(self._on_plot_clicked)
        layout.addWidget(self.arm_plot, 1)
        return card

    def _build_telemetry_card(self) -> QtWidgets.QWidget:
        card, layout = make_card("Joint telemetry")
        self.plot = pg.PlotWidget()
        self.plot.showGrid(x=True, y=True, alpha=0.18)
        self.plot.setLabel("bottom", "Time", units="s")
        self.plot.setLabel("left", "Angle", units="°")
        self.plot.setMenuEnabled(False)
        self.plot.addLegend(offset=(10, 10))
        self.joint_curves = [
            self.plot.plot(
                [], [],
                pen=pg.mkPen(JOINT_COLORS[i], width=2),
                name=f"Joint {i + 1}",
            )
            for i in range(3)
        ]
        layout.addWidget(self.plot, 1)
        return card

    def _build_control_panel(self) -> QtWidgets.QWidget:
        panel = QtWidgets.QWidget()
        panel.setMinimumWidth(0)
        panel.setSizePolicy(
            QtWidgets.QSizePolicy.Ignored,
            QtWidgets.QSizePolicy.Expanding,
        )
        layout = QtWidgets.QVBoxLayout(panel)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(16)

        # Resizable upper section: end-effector readout and control tabs.
        upper_section = QtWidgets.QWidget()
        upper_section.setMinimumWidth(0)
        upper_section.setSizePolicy(
            QtWidgets.QSizePolicy.Ignored,
            QtWidgets.QSizePolicy.Expanding,
        )
        upper_layout = QtWidgets.QVBoxLayout(upper_section)
        upper_layout.setContentsMargins(0, 0, 0, 0)
        upper_layout.setSpacing(16)

        # Live readout card
        readout, readout_layout = make_card("End effector")
        tiles = QtWidgets.QHBoxLayout()
        self.tile_x = MetricTile("X (m)", COLORS["accent"])
        self.tile_y = MetricTile("Y (m)", COLORS["accent"])
        self.tile_err = MetricTile("Target error (m)", COLORS["warn"])
        for tile in (self.tile_x, self.tile_y, self.tile_err):
            tiles.addWidget(tile)
        readout_layout.addLayout(tiles)
        upper_layout.addWidget(readout)

        # Tabs
        tabs = QtWidgets.QTabWidget()
        tabs.addTab(self._build_cartesian_tab(), "Cartesian")
        tabs.addTab(self._build_joint_tab(), "Joints")
        tabs.addTab(self._build_jog_tab(), "Jog")
        upper_layout.addWidget(tabs, 1)

        # Scrollable logs panel below the tabs
        logs_card, logs_layout = make_card("Logs")
        self.log_list = QtWidgets.QListWidget()
        self.log_list.setObjectName("logList")
        self.log_list.setWordWrap(True)
        self.log_list.setVerticalScrollBarPolicy(QtCore.Qt.ScrollBarAsNeeded)
        self.log_list.setHorizontalScrollBarPolicy(QtCore.Qt.ScrollBarAlwaysOff)
        self.log_list.setMinimumHeight(90)
        # self.log_list.setMaximumHeight(125)
        self.log_list.setStyleSheet(
            f"background: {COLORS['panel_alt']};"
            f"border: 1px solid {COLORS['border']};"
            "border-radius: 5px; padding: 4px;"
        )
        self.log_list.setMinimumWidth(0)
        self.log_list.setSizePolicy(
            QtWidgets.QSizePolicy.Expanding,
            QtWidgets.QSizePolicy.Expanding,
        )
        self.log_list.setTextElideMode(QtCore.Qt.ElideNone)
        self.log_list.setResizeMode(QtWidgets.QListView.Adjust)
        logs_layout.addWidget(self.log_list)
        logs_card.setMinimumWidth(0)
        logs_card.setMinimumHeight(130)
        logs_card.setSizePolicy(
            QtWidgets.QSizePolicy.Expanding,
            QtWidgets.QSizePolicy.Preferred,
        )

        # Split between the end-effector/controls area and the logs.
        control_splitter = QtWidgets.QSplitter(QtCore.Qt.Vertical)
        control_splitter.setChildrenCollapsible(False)
        control_splitter.addWidget(upper_section)
        control_splitter.addWidget(logs_card)
        control_splitter.setStretchFactor(0, 4)
        control_splitter.setStretchFactor(1, 1)
        control_splitter.setSizes([600, 165])
        control_splitter.setMinimumWidth(0)
        control_splitter.setSizePolicy(
            QtWidgets.QSizePolicy.Expanding,
            QtWidgets.QSizePolicy.Expanding,
        )

        layout.addWidget(control_splitter, 1)
        return panel

    def _build_cartesian_tab(self) -> QtWidgets.QWidget:
        page = QtWidgets.QWidget()
        layout = QtWidgets.QVBoxLayout(page)
        layout.setContentsMargins(16, 16, 16, 16)
        layout.setSpacing(12)
        hint = QtWidgets.QLabel(
            "Frame: base_link (planner Y is published as Z). "
            "Click the workspace plot to fill the target."
        )
        hint.setObjectName("hint")
        hint.setWordWrap(True)
        layout.addWidget(hint)

        form = QtWidgets.QFormLayout()
        form.setHorizontalSpacing(14)
        form.setVerticalSpacing(10)
        self.x_target = self._coordinate_box()
        self.y_target = self._coordinate_box()
        self.x_target.valueChanged.connect(self._on_xy_edited)
        self.y_target.valueChanged.connect(self._on_xy_edited)
        form.addRow("X", self.x_target)
        form.addRow("Y", self.y_target)
        layout.addLayout(form)

        buttons = QtWidgets.QHBoxLayout()
        use_current = QtWidgets.QPushButton("Use current")
        use_current.clicked.connect(self._set_xy_to_current)
        move_xy = QtWidgets.QPushButton("Move to XY")
        move_xy.setObjectName("primary")
        move_xy.clicked.connect(self._send_xy_target)
        buttons.addWidget(use_current)
        buttons.addWidget(move_xy, 1)
        layout.addLayout(buttons)
        layout.addStretch(1)
        return page

    def _build_joint_tab(self) -> QtWidgets.QWidget:
        page = QtWidgets.QWidget()
        layout = QtWidgets.QVBoxLayout(page)
        layout.setContentsMargins(16, 16, 16, 16)
        layout.setSpacing(16)
        self.joint_rows = []
        for index, (lower, upper) in enumerate(self.arm.joint_limits):
            row = JointRow(
                index, math.degrees(float(lower)), math.degrees(float(upper))
            )
            self.joint_rows.append(row)
            layout.addWidget(row)
        buttons = QtWidgets.QHBoxLayout()
        use_current = QtWidgets.QPushButton("Use current")
        use_current.clicked.connect(self._set_joint_targets_to_current)
        move_joints = QtWidgets.QPushButton("Move joints")
        move_joints.setObjectName("primary")
        move_joints.clicked.connect(self._send_joint_target)
        buttons.addWidget(use_current)
        buttons.addWidget(move_joints, 1)
        layout.addLayout(buttons)
        layout.addStretch(1)
        return page

    def _build_jog_tab(self) -> QtWidgets.QWidget:
        page = QtWidgets.QWidget()
        layout = QtWidgets.QVBoxLayout(page)
        layout.setContentsMargins(16, 16, 16, 16)
        layout.setSpacing(14)

        steps = QtWidgets.QFormLayout()
        self.xy_step = QtWidgets.QComboBox()
        for value in (0.02, 0.05, 0.1, 0.25, 0.5):
            self.xy_step.addItem(f"{value:g} m", value)
        self.xy_step.setCurrentIndex(2)
        self.joint_step = QtWidgets.QComboBox()
        for value in (1, 2, 3, 5, 10):
            self.joint_step.addItem(f"{value}°", math.radians(value))
        self.joint_step.setCurrentIndex(2)
        steps.addRow("XY step", self.xy_step)
        steps.addRow("Joint step", self.joint_step)
        layout.addLayout(steps)

        pad = QtWidgets.QGridLayout()
        pad.setSpacing(6)
        for text, row, col, dx, dy in (
            ("▲", 0, 1, 0, 1),
            ("◀", 1, 0, -1, 0),
            ("▶", 1, 2, 1, 0),
            ("▼", 2, 1, 0, -1),
        ):
            button = QtWidgets.QPushButton(text)
            button.setObjectName("jog")
            button.setFocusPolicy(QtCore.Qt.NoFocus)
            button.clicked.connect(
                lambda _checked=False, a=dx, b=dy: self._send_xy_increment(
                    a * self.xy_step.currentData(),
                    b * self.xy_step.currentData(),
                )
            )
            pad.addWidget(button, row, col)
        pad_wrap = QtWidgets.QHBoxLayout()
        pad_wrap.addStretch(1)
        pad_wrap.addLayout(pad)
        pad_wrap.addStretch(1)
        layout.addLayout(pad_wrap)

        joint_grid = QtWidgets.QGridLayout()
        joint_grid.setSpacing(6)
        for index in range(3):
            label = QtWidgets.QLabel(f"Joint {index + 1}")
            label.setStyleSheet(f"color: {JOINT_COLORS[index]}; font-weight: 600;")
            minus = QtWidgets.QPushButton("−")
            plus = QtWidgets.QPushButton("+")
            for button, sign in ((minus, -1), (plus, 1)):
                button.setObjectName("jog")
                button.setFocusPolicy(QtCore.Qt.NoFocus)
                button.clicked.connect(
                    lambda _checked=False, i=index, s=sign: (
                        self._send_joint_increment(
                            i, s * self.joint_step.currentData()
                        )
                    )
                )
            joint_grid.addWidget(label, index, 0)
            joint_grid.addWidget(minus, index, 1)
            joint_grid.addWidget(plus, index, 2)
        joint_grid.setColumnStretch(0, 1)
        layout.addLayout(joint_grid)

        keys = QtWidgets.QLabel(
            "Keyboard: W/A/S/D or arrows jog XY  ·  1/2, 3/4, 5/6 jog "
            "joints 1/2/3 down/up"
        )
        keys.setObjectName("hint")
        keys.setWordWrap(True)
        layout.addWidget(keys)
        layout.addStretch(1)
        return page

    def _build_status_bar(self) -> None:
        bar = self.statusBar()
        self.sb_message = QtWidgets.QLabel("Ready")
        self.sb_rate = QtWidgets.QLabel("")
        self.sb_rate.setObjectName("mono")
        bar.addWidget(self.sb_message, 1)
        bar.addPermanentWidget(self.sb_rate)

    @staticmethod
    def _coordinate_box():
        box = QtWidgets.QDoubleSpinBox()
        box.setRange(-sum(LINK_LENGTHS), sum(LINK_LENGTHS))
        box.setDecimals(3)
        box.setSingleStep(0.1)
        box.setSuffix(" m")
        return box

    # ---- actions --------------------------------------------------------
    def _add_log(self, text: str, level: str = "INFO") -> None:
        """Append a timestamped entry to the scrollable log panel."""
        if not hasattr(self, "log_list"):
            return

        timestamp = time.strftime("%H:%M:%S")
        item = QtWidgets.QListWidgetItem(f"[{timestamp}] [{level}]  {text}")

        if level == "ERROR":
            item.setForeground(QtGui.QColor(COLORS["bad"]))
        elif level == "WARNING":
            item.setForeground(QtGui.QColor(COLORS["warn"]))
        else:
            item.setForeground(QtGui.QColor(COLORS["text"]))

        self.log_list.addItem(item)
        self.log_list.scrollToBottom()

        while self.log_list.count() > 100:
            self.log_list.takeItem(0)

    def _notify(self, text: str) -> None:
        self.statusBar().showMessage(text, 4000)
        self._add_log(text, "INFO")

    def _on_plot_clicked(self, event) -> None:
        if event.button() != QtCore.Qt.LeftButton:
            return
        view_box = self.arm_plot.getPlotItem().vb
        if not view_box.sceneBoundingRect().contains(event.scenePos()):
            return
        point = view_box.mapSceneToView(event.scenePos())
        for box, value in ((self.x_target, point.x()), (self.y_target, point.y())):
            box.blockSignals(True)
            box.setValue(value)
            box.blockSignals(False)
        self._on_xy_edited()
        self._notify(f"Target set to ({point.x():.3f}, {point.y():.3f})")

    def _on_xy_edited(self) -> None:
        self._target_xy = (self.x_target.value(), self.y_target.value())

    def _set_xy_to_current(self):
        positions, _ = self.node.snapshot()
        x, y = self.arm.end_effector(positions)
        self.x_target.setValue(float(x))
        self.y_target.setValue(float(y))

    def _send_xy_target(self):
        self._on_xy_edited()
        self.node.publish_xy_target(*self._target_xy)
        self._notify(
            f"Sent XY target ({self._target_xy[0]:.3f}, {self._target_xy[1]:.3f})"
        )

    def _send_joint_target(self):
        self.node.publish_joint_target(
            [math.radians(row.value_deg()) for row in self.joint_rows]
        )
        self._notify("Sent joint target")

    def _set_joint_targets_to_current(self):
        positions, _ = self.node.snapshot()
        for row, position in zip(self.joint_rows, positions):
            row.set_value_deg(math.degrees(position))

    def _send_xy_increment(self, dx: float, dy: float):
        positions, _ = self.node.snapshot()
        x, y = self.arm.end_effector(positions)
        self.node.publish_xy_target(x + dx, y + dy)

    def _send_joint_increment(self, joint_index: int, delta: float):
        positions, _ = self.node.snapshot()
        positions[joint_index] += delta
        self.node.publish_joint_target(positions)

    def keyPressEvent(self, event: QtGui.QKeyEvent):
        """Map keyboard jog keys to XY or individual-joint target updates."""
        if isinstance(
            self.focusWidget(),
            (QtWidgets.QLineEdit, QtWidgets.QAbstractSpinBox, QtWidgets.QComboBox),
        ):
            super().keyPressEvent(event)
            return

        step = self.xy_step.currentData()
        jstep = self.joint_step.currentData()
        key = event.key()
        xy_keys = {
            QtCore.Qt.Key_A: (-step, 0.0),
            QtCore.Qt.Key_Left: (-step, 0.0),
            QtCore.Qt.Key_D: (step, 0.0),
            QtCore.Qt.Key_Right: (step, 0.0),
            QtCore.Qt.Key_S: (0.0, -step),
            QtCore.Qt.Key_Down: (0.0, -step),
            QtCore.Qt.Key_W: (0.0, step),
            QtCore.Qt.Key_Up: (0.0, step),
        }
        joint_keys = {
            QtCore.Qt.Key_1: (0, -jstep),
            QtCore.Qt.Key_2: (0, jstep),
            QtCore.Qt.Key_3: (1, -jstep),
            QtCore.Qt.Key_4: (1, jstep),
            QtCore.Qt.Key_5: (2, -jstep),
            QtCore.Qt.Key_6: (2, jstep),
        }
        if key in xy_keys:
            self._send_xy_increment(*xy_keys[key])
            event.accept()
        elif key in joint_keys:
            self._send_joint_increment(*joint_keys[key])
            event.accept()
        else:
            super().keyPressEvent(event)

    # ---- periodic refresh ----------------------------------------------
    def _refresh(self):
        positions, status = self.node.snapshot()
        age = self.node.telemetry_age()

        # Seed the joint target fields once, from the first real state.
        if age is not None and not self._joint_fields_seeded:
            self._set_joint_targets_to_current()
            self._set_xy_to_current()
            self._joint_fields_seeded = True

        points = self.arm.forward_kinematics(positions)
        xs = [float(p[0]) for p in points]
        ys = [float(p[1]) for p in points]
        self.arm_curve.setData(xs, ys)
        end_x, end_y = xs[-1], ys[-1]
        self._tip_marker.setData([end_x], [end_y])

        if age is not None and (
            not self._trail or self._trail[-1] != (end_x, end_y)
        ):
            self._trail.append((end_x, end_y))
        self._trail_curve.setData(
            [p[0] for p in self._trail], [p[1] for p in self._trail]
        )

        if self._target_xy is not None:
            self._target_marker.setData([self._target_xy[0]], [self._target_xy[1]])
            error = math.hypot(
                self._target_xy[0] - end_x, self._target_xy[1] - end_y
            )
            self.tile_err.set_value(f"{error:.3f}")
        self.tile_x.set_value(f"{end_x:+.3f}")
        self.tile_y.set_value(f"{end_y:+.3f}")

        for row, position in zip(self.joint_rows, positions):
            row.set_actual_deg(math.degrees(position))

        self._update_status(status, age)

        now = time.monotonic()
        if now - self._last_plot_update >= 0.1:
            elapsed = now - self._started_at
            self._history_times.append(elapsed)
            for history, position in zip(self._history_positions, positions):
                history.append(math.degrees(position))
            times = list(self._history_times)
            for curve, history in zip(self.joint_curves, self._history_positions):
                curve.setData(times, list(history))
            self.plot.setXRange(max(0.0, elapsed - 40.0), max(elapsed, 5.0))
            self._last_plot_update = now

    def _update_status(self, status: str, age) -> None:
        telemetry_online = age is not None and age <= STALE_AFTER_S

        if telemetry_online:
            self.link_pill.set_state("Telemetry: Online", COLORS["ok"])
            self.sb_rate.setText(f"joint_states: {age * 1000:.0f} ms ago")
            telemetry_state = "Online"
        else:
            self.link_pill.set_state("Telemetry: Offline", COLORS["bad"])
            if age is None:
                self.sb_rate.setText("joint_states: --")
            else:
                self.sb_rate.setText(f"joint_states: stale {age:.1f}s")
            telemetry_state = "Offline"

        # Log only state transitions, not the same message on every refresh.
        if telemetry_state != self._last_logged_telemetry_state:
            if telemetry_state == "Offline":
                self._add_log("Joint-state telemetry is offline or stale.", "ERROR")
            elif self._last_logged_telemetry_state == "Offline":
                self._add_log("Joint-state telemetry is online again.", "INFO")
            self._last_logged_telemetry_state = telemetry_state

        lowered = status.lower()
        is_controller_error = any(
            word in lowered for word in ("error", "fail", "unreach", "limit")
        )

        if is_controller_error:
            if status != self._last_logged_controller_error:
                self._add_log(f"Controller: {status}", "ERROR")
                self._last_logged_controller_error = status
        else:
            self._last_logged_controller_error = None


def main(args=None):
    """Run Qt in the main thread and ROS callbacks in a background executor."""
    rclpy.init(args=args)
    node = GuiNode()
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    ros_thread = threading.Thread(target=executor.spin, daemon=True)
    application = QtWidgets.QApplication([])
    apply_dark_theme(application)
    window = ArmWindow(node)
    ros_thread.start()
    window.show()

    try:
        exit_code = application.exec_()
    finally:
        executor.shutdown()
        ros_thread.join(timeout=2.0)
        node.destroy_node()
        rclpy.shutdown()
    raise SystemExit(exit_code)


if __name__ == "__main__":
    main()