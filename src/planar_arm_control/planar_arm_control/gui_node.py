#!/usr/bin/env python3
"""PyQt5 teleoperation and live telemetry console for the planar arm."""

from collections import deque
import math
import threading
import time

import rclpy
from geometry_msgs.msg import PointStamped
from rclpy.parameter import Parameter
from rclpy.parameter import parameter_value_to_python
from rclpy.parameter_client import AsyncParameterClient
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64, Float64MultiArray, Int32, String
from std_srvs.srv import Trigger
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from PyQt5 import QtCore, QtGui, QtWidgets
import pyqtgraph as pg

from planar_arm_control.planar_arm import PlanarArm

LINK_LENGTHS = [3.0, 2.0, 1.5]
JOINT_NAMES = ["joint1", "joint2", "joint3"]
JOINT_COLORS = ["#4da6ff", "#ffb454", "#7ee787"]
STALE_AFTER_S = 1.0
HISTORY_LEN = 400

# Control-loop timing quality (milliseconds of timer jitter). These drive the
# Stable / Degraded / Unstable badge; tune them for your machine.
JITTER_STABLE_MEAN_MS = 10.0
JITTER_STABLE_PEAK_MS = 30.0
JITTER_DEGRADED_MEAN_MS = 20.0
JITTER_DEGRADED_PEAK_MS = 60.0

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
    "bad_hover": "#ff6b63",
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
    padding: 8px 20px;
    border: none;
    border-bottom: 2px solid transparent;
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
QPushButton#danger {{
    background: {COLORS['bad']};
    border: 1px solid {COLORS['bad']};
    color: white;
    font-weight: 700;
}}
QPushButton#danger:hover {{
    background: {COLORS['bad_hover']};
    border-color: {COLORS['bad_hover']};
}}
QLabel#metricValueSmall {{
    font-family: "Consolas", "JetBrains Mono", "Menlo", monospace;
    font-size: 15px;
    font-weight: 600;
}}
QLabel#speedValue {{
    font-family: "Consolas", "JetBrains Mono", "Menlo", monospace;
    font-weight: 700;
    color: {COLORS['accent']};
    background: {COLORS['panel_alt']};
    border: 1px solid {COLORS['border']};
    border-radius: 10px;
    padding: 2px 10px;
}}
QFrame#subCard {{
    background: {COLORS['panel_alt']};
    border: 1px solid {COLORS['border']};
    border-radius: 6px;
}}
QScrollArea {{ background: transparent; border: none; }}
QScrollArea > QWidget > QWidget {{ background: transparent; }}
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
        self._positions = [
            math.radians(75.0),
            math.radians(99.0),
            0.0,
        ]
        self._status = "Waiting for controller"
        self._pick_place_status = "Pick/place: waiting for node"
        self._controller_error = [0.0, 0.0, 0.0]
        self._controller_timing = [0.0, 0.0, 0.0]
        self._speed_scale = 1.0
        self._last_error_time = None
        self._last_state_time = None
        self._joint_target_publisher = self.create_publisher(
            Float64MultiArray, "/joint_target", 10
        )
        self._xy_target_publisher = self.create_publisher(
            PointStamped, "/target_pose", 10
        )
        self._gripper_publisher = self.create_publisher(
            JointTrajectory,
            "/gripper_controller/joint_trajectory",
            10,
        )
        self._pick_place_parameters = AsyncParameterClient(
            self, "/pick_place_node"
        )
        self._controller_parameters = AsyncParameterClient(
            self, "/controller_node"
        )
        self._pick_client = self.create_client(
            Trigger, "/pick_place/pick"
        )
        self._place_client = self.create_client(
            Trigger, "/pick_place/place"
        )
        self._cancel_client = self.create_client(
            Trigger, "/controller/cancel"
        )
        self._pick_place_cancel_client = self.create_client(
            Trigger, "/pick_place/cancel"
        )
        self._speed_adjust_publisher = self.create_publisher(
            Int32, "/controller/speed_adjust", 10
        )
        self.create_subscription(
            JointState, "/joint_states", self._on_joint_state, 10
        )
        self.create_subscription(
            String, "/controller_status", self._on_status, 10
        )
        self.create_subscription(
            String, "/pick_place/status", self._on_pick_place_status, 10
        )
        self.create_subscription(
            Float64MultiArray, "/controller_error", self._on_controller_error,
            10,
        )
        self.create_subscription(
            Float64MultiArray, "/controller_timing", self._on_controller_timing,
            10,
        )
        self.create_subscription(
            Float64, "/controller_speed_scale", self._on_speed_scale, 10
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

    def _on_pick_place_status(self, message: String) -> None:
        with self._state_lock:
            self._pick_place_status = message.data

    def _on_controller_error(self, message: Float64MultiArray) -> None:
        if len(message.data) != len(JOINT_NAMES) or not all(
            math.isfinite(value) for value in message.data
        ):
            return
        with self._state_lock:
            self._controller_error = list(message.data)
            self._last_error_time = time.monotonic()

    def _on_controller_timing(self, message: Float64MultiArray) -> None:
        if len(message.data) != 3 or not all(
            math.isfinite(value) for value in message.data
        ):
            return
        with self._state_lock:
            self._controller_timing = list(message.data)

    def _on_speed_scale(self, message: Float64) -> None:
        if not math.isfinite(message.data):
            return
        with self._state_lock:
            self._speed_scale = message.data

    def snapshot(self):
        """Return the latest positions and controller status consistently."""
        with self._state_lock:
            return (
                list(self._positions),
                self._status,
                self._pick_place_status,
            )

    def telemetry_age(self):
        """Return seconds since the last joint state, or None if never."""
        with self._state_lock:
            if self._last_state_time is None:
                return None
            return time.monotonic() - self._last_state_time

    def control_telemetry(self):
        """Return tracking error, timer jitter, and error-sample time."""
        with self._state_lock:
            return (
                list(self._controller_error),
                list(self._controller_timing),
                self._last_error_time,
                self._speed_scale,
            )

    def adjust_speed(self, direction: int) -> None:
        """Request one bounded speed-step change from the controller."""
        if direction not in (-1, 1):
            raise ValueError("speed direction must be -1 or +1")
        self._speed_adjust_publisher.publish(Int32(data=direction))

    def get_pid_gains(self, callback) -> None:
        """Read current PID and velocity tracking gains from the controller."""
        if not self._controller_parameters.services_are_ready():
            callback(None, "Controller parameter service is not ready")
            return
        future = self._controller_parameters.get_parameters(
            [
                "effort_pid_kp",
                "effort_pid_ki",
                "effort_pid_kd",
                "velocity_tracking_kp",
            ]
        )

        def on_result(result):
            try:
                values = [
                    float(parameter_value_to_python(value))
                    for value in result.result().values
                ]
            except Exception as error:
                callback(None, f"Could not read controller gains: {error}")
                return
            callback(values, None)

        future.add_done_callback(on_result)

    def set_pid_gains(
        self, kp: float, ki: float, kd: float, velocity_kp: float, callback
    ) -> None:
        """Apply validated effort PID and velocity tracking gains."""
        if not self._controller_parameters.services_are_ready():
            callback("Controller parameter service is not ready")
            return
        parameters = [
            Parameter("effort_pid_kp", value=float(kp)),
            Parameter("effort_pid_ki", value=float(ki)),
            Parameter("effort_pid_kd", value=float(kd)),
            Parameter("velocity_tracking_kp", value=float(velocity_kp)),
        ]
        future = self._controller_parameters.set_parameters(parameters)

        def on_result(result):
            try:
                response = result.result()
            except Exception as error:
                callback(f"Could not update controller gains: {error}")
                return
            rejected = [
                item.reason for item in response.results
                if not item.successful
            ]
            callback(
                f"Gain update rejected: {'; '.join(rejected)}"
                if rejected
                else "Controller gains updated"
            )

        future.add_done_callback(on_result)

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

    def command_gripper(self, opened: bool) -> bool:
        """Command both simulated fingers open or closed."""
        if self._gripper_publisher.get_subscription_count() == 0:
            return False
        position = 0.14 if opened else -0.02
        message = JointTrajectory()
        message.joint_names = ["left_finger_joint", "right_finger_joint"]
        point = JointTrajectoryPoint()
        point.positions = [position, position]
        point.time_from_start.sec = 1
        message.points = [point]
        self._gripper_publisher.publish(message)
        return True

    def start_pick_place(
        self,
        action: str,
        pick_xy: tuple[float, float],
        place_xy: tuple[float, float],
        callback,
    ) -> None:
        """Update task coordinates before requesting pick or place."""
        clients = {
            "pick": self._pick_client,
            "place": self._place_client,
        }
        client = clients.get(action)
        if client is None:
            callback(f"Unknown pick/place action: {action}")
            return
        if not self._pick_place_parameters.services_are_ready():
            callback("Pick/place parameter service is not ready")
            return
        if not client.service_is_ready():
            callback(f"/pick_place/{action} service is not ready")
            return

        parameters = [
            Parameter("pick_x", value=float(pick_xy[0])),
            Parameter("pick_y", value=float(pick_xy[1])),
            Parameter("place_x", value=float(place_xy[0])),
            Parameter("place_y", value=float(place_xy[1])),
        ]
        future = self._pick_place_parameters.set_parameters(parameters)
        future.add_done_callback(
            lambda result: self._on_task_parameters_set(
                result, client, action, callback
            )
        )

    def cancel_motion(self, callback) -> None:
        """Cancel an active task and stop the controller's current motion."""
        primary_client = (
            self._pick_place_cancel_client
            if self._pick_place_cancel_client.service_is_ready()
            else self._cancel_client
        )
        fallback_client = (
            self._cancel_client
            if primary_client is self._pick_place_cancel_client
            else None
        )
        if not primary_client.service_is_ready():
            callback("Controller cancel service is not ready")
            return
        future = primary_client.call_async(Trigger.Request())
        future.add_done_callback(
            lambda result: self._on_cancel_result(
                result, fallback_client, callback
            )
        )

    def _on_cancel_result(self, future, fallback_client, callback) -> None:
        try:
            response = future.result()
        except Exception as error:
            if (
                fallback_client is not None
                and fallback_client.service_is_ready()
            ):
                retry = fallback_client.call_async(Trigger.Request())
                retry.add_done_callback(
                    lambda result: self._on_cancel_result(
                        result, None, callback
                    )
                )
                return
            callback(f"Cancel request failed: {error}")
            return
        if (
            not response.success
            and fallback_client is not None
            and fallback_client.service_is_ready()
        ):
            retry = fallback_client.call_async(Trigger.Request())
            retry.add_done_callback(
                lambda result: self._on_cancel_result(
                    result, None, callback
                )
            )
            return
        callback(
            response.message
            if response.success
            else f"Cancel rejected: {response.message}"
        )

    @staticmethod
    def _on_task_parameters_set(future, client, action, callback) -> None:
        try:
            response = future.result()
        except Exception as error:
            callback(f"Could not configure pick/place task: {error}")
            return
        rejected = [
            result.reason
            for result in response.results
            if not result.successful
        ]
        if rejected:
            callback(f"Pick/place settings rejected: {'; '.join(rejected)}")
            return
        request = Trigger.Request()
        service_future = client.call_async(request)
        service_future.add_done_callback(
            lambda result: GuiNode._on_task_service_result(
                result, action, callback
            )
        )

    @staticmethod
    def _on_task_service_result(future, action, callback) -> None:
        try:
            response = future.result()
        except Exception as error:
            callback(f"{action.capitalize()} request failed: {error}")
            return
        callback(
            response.message
            if response.success
            else f"{action.capitalize()} rejected: {response.message}"
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
    font = label.font()
    font.setLetterSpacing(QtGui.QFont.AbsoluteSpacing, 1.0)
    label.setFont(font)
    outer.addWidget(label)
    return frame, outer


def polish_tabs(tabs: QtWidgets.QTabWidget) -> None:
    """Size tabs from a bold font so labels are never clipped or elided."""
    bar = tabs.tabBar()
    font = bar.font()
    font.setBold(True)
    bar.setFont(font)
    bar.setElideMode(QtCore.Qt.ElideNone)
    bar.setUsesScrollButtons(False)
    bar.setExpanding(False)
    bar.setDrawBase(False)


def scroll_page(page: QtWidgets.QWidget) -> QtWidgets.QScrollArea:
    """Wrap a tab page so tall content scrolls instead of squeezing."""
    area = QtWidgets.QScrollArea()
    area.setWidgetResizable(True)
    area.setFrameShape(QtWidgets.QFrame.NoFrame)
    area.setHorizontalScrollBarPolicy(QtCore.Qt.ScrollBarAlwaysOff)
    area.setWidget(page)
    return area


def classify_jitter(mean_ms: float, peak_ms: float):
    """Return (label, colour) describing control-loop timing quality."""
    if (
        mean_ms <= JITTER_STABLE_MEAN_MS
        and peak_ms <= JITTER_STABLE_PEAK_MS
    ):
        return "Stable", COLORS["ok"]
    if (
        mean_ms <= JITTER_DEGRADED_MEAN_MS
        and peak_ms <= JITTER_DEGRADED_PEAK_MS
    ):
        return "Degraded", COLORS["warn"]
    return "Unstable", COLORS["bad"]


class JitterGauge(QtWidgets.QWidget):
    """Bar of the worst delay over green / amber / red zones; dot = average."""

    def __init__(self):
        super().__init__()
        self.setMinimumSize(140, 28)
        self._mean_ms = 0.0
        self._peak_ms = 0.0
        self._scale_ms = JITTER_DEGRADED_PEAK_MS * 1.5

    def set_values(self, mean_ms: float, peak_ms: float) -> None:
        """Update the displayed average and worst delay in milliseconds."""
        self._mean_ms = max(0.0, mean_ms)
        self._peak_ms = max(0.0, peak_ms)
        self.update()

    def _x(self, milliseconds: float) -> float:
        fraction = min(1.0, max(0.0, milliseconds / self._scale_ms))
        return fraction * self.width()

    def paintEvent(self, _event):
        painter = QtGui.QPainter(self)
        painter.setRenderHint(QtGui.QPainter.Antialiasing)
        top, height = 2.0, 8.0
        painter.setPen(QtCore.Qt.NoPen)
        zones = (
            (0.0, JITTER_STABLE_PEAK_MS, COLORS["ok"]),
            (JITTER_STABLE_PEAK_MS, JITTER_DEGRADED_PEAK_MS, COLORS["warn"]),
            (JITTER_DEGRADED_PEAK_MS, self._scale_ms, COLORS["bad"]),
        )
        for start, end, colour in zones:
            zone = QtGui.QColor(colour)
            zone.setAlpha(60)
            painter.setBrush(zone)
            painter.drawRect(
                QtCore.QRectF(
                    self._x(start), top,
                    self._x(end) - self._x(start), height,
                )
            )
        if self._peak_ms > 0.0:
            _label, colour = classify_jitter(self._mean_ms, self._peak_ms)
            painter.setBrush(QtGui.QColor(colour))
            painter.drawRect(
                QtCore.QRectF(0.0, top, max(2.0, self._x(self._peak_ms)), height)
            )
            painter.setBrush(QtGui.QColor(COLORS["text"]))
            painter.drawEllipse(
                QtCore.QPointF(self._x(self._mean_ms), top + height / 2.0),
                4.0, 4.0,
            )
        font = QtGui.QFont(self.font())
        font.setPixelSize(10)
        painter.setFont(font)
        painter.setPen(QtGui.QColor(COLORS["muted"]))
        label_top = top + height + 3.0
        painter.drawText(
            QtCore.QRectF(0.0, label_top, 44.0, 14.0),
            QtCore.Qt.AlignLeft, "0",
        )
        for milliseconds in (JITTER_STABLE_PEAK_MS, JITTER_DEGRADED_PEAK_MS):
            painter.drawText(
                QtCore.QRectF(self._x(milliseconds) - 22.0, label_top, 44.0, 14.0),
                QtCore.Qt.AlignHCenter, f"{milliseconds:.0f}",
            )
        painter.drawText(
            QtCore.QRectF(self.width() - 44.0, label_top, 44.0, 14.0),
            QtCore.Qt.AlignRight, "ms",
        )


class ControlHealthStrip(QtWidgets.QFrame):
    """Plain-language summary of control-loop timing instead of raw numbers."""

    def __init__(self):
        super().__init__()
        self.setObjectName("subCard")
        self.setToolTip(
            "Control-loop timing: how far the controller's timer drifts from "
            "its schedule. Smaller delays mean smoother, more repeatable "
            "motion.\n"
            f"Stable: average up to {JITTER_STABLE_MEAN_MS:.0f} ms and worst "
            f"up to {JITTER_STABLE_PEAK_MS:.0f} ms.\n"
            f"Degraded: average up to {JITTER_DEGRADED_MEAN_MS:.0f} ms and "
            f"worst up to {JITTER_DEGRADED_PEAK_MS:.0f} ms."
        )
        layout = QtWidgets.QHBoxLayout(self)
        layout.setContentsMargins(12, 8, 12, 8)
        layout.setSpacing(18)

        heading = QtWidgets.QVBoxLayout()
        heading.setSpacing(2)
        name = QtWidgets.QLabel("Control loop timing")
        name.setObjectName("metricName")
        self.badge = StatusPill("Waiting for data")
        heading.addWidget(name)
        heading.addWidget(self.badge)
        layout.addLayout(heading)

        self.latest = MetricTile("Latest delay", compact=True)
        self.average = MetricTile("Average delay", compact=True)
        self.worst = MetricTile("Worst delay", compact=True)
        for tile in (self.latest, self.average, self.worst):
            layout.addWidget(tile)

        gauge_box = QtWidgets.QVBoxLayout()
        gauge_box.setSpacing(2)
        self.gauge = JitterGauge()
        caption = QtWidgets.QLabel("Bar: worst delay  ·  Dot: average")
        caption.setObjectName("metricName")
        gauge_box.addWidget(self.gauge)
        gauge_box.addWidget(caption)
        layout.addLayout(gauge_box, 1)

    def update_timing(self, timing_s) -> None:
        """Show last / mean / max timer jitter (seconds) as a health summary."""
        if not any(timing_s):
            self.badge.set_state("Waiting for data", COLORS["muted"])
            for tile in (self.latest, self.average, self.worst):
                tile.set_value("--")
            self.gauge.set_values(0.0, 0.0)
            return
        last_ms = abs(timing_s[0]) * 1000.0
        mean_ms = timing_s[1] * 1000.0
        peak_ms = timing_s[2] * 1000.0
        label, colour = classify_jitter(mean_ms, peak_ms)
        self.badge.set_state(label, colour)
        self.latest.set_value(f"{last_ms:.1f} ms")
        self.average.set_value(f"{mean_ms:.1f} ms")
        self.worst.set_value(f"{peak_ms:.1f} ms")
        self.gauge.set_values(mean_ms, peak_ms)


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

    def __init__(self, name: str, accent: str = None, compact: bool = False):
        super().__init__()
        layout = QtWidgets.QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(0)
        self._name = QtWidgets.QLabel(name)
        self._name.setObjectName("metricName")
        self._value = QtWidgets.QLabel("--")
        self._value.setObjectName(
            "metricValueSmall" if compact else "metricValue"
        )
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
        self.spin.setButtonSymbols(QtWidgets.QAbstractSpinBox.NoButtons)

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

    operation_result = QtCore.pyqtSignal(str)
    pid_values_received = QtCore.pyqtSignal(object, object)

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
        self._error_history_times = deque(maxlen=HISTORY_LEN)
        self._history_errors = [deque(maxlen=HISTORY_LEN) for _ in range(3)]
        self._last_error_sample_time = None
        self.operation_result.connect(self._on_operation_result)
        self.pid_values_received.connect(self._on_pid_values_received)

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
            "3-DOF planar arm  ·  teleoperation and live telemetry"
        )
        subtitle.setObjectName("subtitle")
        title.setToolTip("Publishes /joint_target and /target_pose")
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
        self.telemetry_tabs = QtWidgets.QTabWidget()
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
        self.telemetry_tabs.addTab(self.plot, "Joint positions")

        self.error_plot = pg.PlotWidget()
        self.error_plot.showGrid(x=True, y=True, alpha=0.18)
        self.error_plot.setLabel("bottom", "Time", units="s")
        self.error_plot.setLabel("left", "Tracking error", units="°")
        self.error_plot.setMenuEnabled(False)
        self.error_plot.addLegend(offset=(10, 10))
        self.error_curves = [
            self.error_plot.plot(
                [], [],
                pen=pg.mkPen(JOINT_COLORS[i], width=2),
                name=f"Joint {i + 1}",
            )
            for i in range(3)
        ]
        self.telemetry_tabs.addTab(self.error_plot, "Tracking error")
        polish_tabs(self.telemetry_tabs)
        self.health_strip = ControlHealthStrip()
        layout.addWidget(self.health_strip)
        layout.addWidget(self.telemetry_tabs, 1)
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

        motion, motion_layout = make_card("Motion control")
        speed_row = QtWidgets.QHBoxLayout()
        speed_row.setSpacing(8)
        speed_row.addWidget(QtWidgets.QLabel("Movement speed"))
        self.speed_scale_label = QtWidgets.QLabel("100%")
        self.speed_scale_label.setObjectName("speedValue")
        speed_row.addWidget(self.speed_scale_label)
        speed_row.addStretch(1)
        self.speed_down_button = QtWidgets.QPushButton("− Slower")
        self.speed_up_button = QtWidgets.QPushButton("+ Faster")
        self.speed_down_button.clicked.connect(
            lambda: self._adjust_speed(-1)
        )
        self.speed_up_button.clicked.connect(
            lambda: self._adjust_speed(1)
        )
        speed_row.addWidget(self.speed_down_button)
        speed_row.addWidget(self.speed_up_button)
        motion_layout.addLayout(speed_row)

        self.cancel_button = QtWidgets.QPushButton("Cancel movement")
        self.cancel_button.setObjectName("danger")
        self.cancel_button.clicked.connect(self._cancel_motion)
        motion_layout.addWidget(self.cancel_button)
        upper_layout.addWidget(motion)

        # Tabs
        tabs = QtWidgets.QTabWidget()
        tabs.addTab(scroll_page(self._build_cartesian_tab()), "Cartesian")
        tabs.addTab(scroll_page(self._build_joint_tab()), "Joints")
        tabs.addTab(scroll_page(self._build_jog_tab()), "Jog")
        # "&&" because a single "&" is a keyboard-mnemonic marker in Qt.
        tabs.addTab(scroll_page(self._build_pick_place_tab()), "Pick && Place")
        tabs.addTab(scroll_page(self._build_tuning_tab()), "PID tuning")
        polish_tabs(tabs)
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
            "Click the workspace plot to fill the target. Sending a new "
            "target while moving replans from current feedback."
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

    def _build_tuning_tab(self) -> QtWidgets.QWidget:
        page = QtWidgets.QWidget()
        layout = QtWidgets.QVBoxLayout(page)
        layout.setContentsMargins(16, 16, 16, 16)
        layout.setSpacing(12)

        hint = QtWidgets.QLabel(
            "Tune effort-mode PID and velocity-mode proportional gain. "
            "Apply changes live while monitoring the tracking-error plot; "
            "gain updates reset the integral accumulator."
        )
        hint.setObjectName("hint")
        hint.setWordWrap(True)
        layout.addWidget(hint)

        form = QtWidgets.QFormLayout()
        form.setHorizontalSpacing(14)
        form.setVerticalSpacing(8)
        self.pid_kp = self._gain_box(500.0, 50.0)
        self.pid_ki = self._gain_box(100.0, 0.0)
        self.pid_kd = self._gain_box(100.0, 5.0)
        self.velocity_kp = self._gain_box(100.0, 4.0)
        form.addRow("Effort Kp", self.pid_kp)
        form.addRow("Effort Ki", self.pid_ki)
        form.addRow("Effort Kd", self.pid_kd)
        form.addRow("Velocity Kp", self.velocity_kp)
        layout.addLayout(form)

        feedback = QtWidgets.QFrame()
        feedback.setObjectName("subCard")
        feedback_layout = QtWidgets.QHBoxLayout(feedback)
        feedback_layout.setContentsMargins(12, 8, 12, 8)
        self.pid_feedback_tile = MetricTile(
            "Feedback RMS error", compact=True
        )
        feedback_layout.addWidget(self.pid_feedback_tile)
        feedback_layout.addStretch(1)
        layout.addWidget(feedback)

        gain_buttons = QtWidgets.QHBoxLayout()
        read_button = QtWidgets.QPushButton("Read current gains")
        read_button.clicked.connect(self._read_pid_gains)
        gain_buttons.addWidget(read_button)
        apply_button = QtWidgets.QPushButton("Apply gains")
        apply_button.setObjectName("primary")
        apply_button.clicked.connect(self._apply_pid_gains)
        gain_buttons.addWidget(apply_button)
        layout.addLayout(gain_buttons)
        layout.addStretch(1)
        self._read_pid_gains()
        return page

    @staticmethod
    def _gain_box(maximum: float, default: float):
        box = QtWidgets.QDoubleSpinBox()
        box.setRange(0.0, maximum)
        box.setDecimals(2)
        box.setSingleStep(1.0)
        box.setValue(default)
        box.setButtonSymbols(QtWidgets.QAbstractSpinBox.NoButtons)
        return box

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

    def _build_pick_place_tab(self) -> QtWidgets.QWidget:
        page = QtWidgets.QWidget()
        layout = QtWidgets.QVBoxLayout(page)
        layout.setContentsMargins(16, 16, 16, 16)
        layout.setSpacing(12)

        hint = QtWidgets.QLabel(
            "Set planar X/Y coordinates on the table. Y maps to robot Z. "
            "The arm closes its physical fingers at the pick point."
        )
        hint.setObjectName("hint")
        hint.setWordWrap(True)
        layout.addWidget(hint)

        form = QtWidgets.QFormLayout()
        form.setHorizontalSpacing(14)
        form.setVerticalSpacing(8)
        self.pick_x = self._coordinate_box()
        self.pick_y = self._coordinate_box()
        self.place_x = self._coordinate_box()
        self.place_y = self._coordinate_box()
        for box in (self.pick_y, self.place_y):
            box.setMinimum(0.0)
        self.pick_x.setValue(4.0)
        self.pick_y.setValue(1.95)
        self.place_x.setValue(-3.0)
        self.place_y.setValue(3.0)
        form.addRow("Pick X", self.pick_x)
        form.addRow("Pick Y", self.pick_y)
        form.addRow("Place X", self.place_x)
        form.addRow("Place Y", self.place_y)
        layout.addLayout(form)

        task_buttons = QtWidgets.QHBoxLayout()
        pick_button = QtWidgets.QPushButton("Pick")
        pick_button.setObjectName("primary")
        pick_button.clicked.connect(
            lambda: self._start_pick_place("pick")
        )
        place_button = QtWidgets.QPushButton("Place")
        place_button.clicked.connect(
            lambda: self._start_pick_place("place")
        )
        task_buttons.addWidget(pick_button)
        task_buttons.addWidget(place_button)
        layout.addLayout(task_buttons)

        gripper_buttons = QtWidgets.QHBoxLayout()
        open_button = QtWidgets.QPushButton("Open gripper")
        open_button.clicked.connect(lambda: self._command_gripper(True))
        close_button = QtWidgets.QPushButton("Close gripper")
        close_button.clicked.connect(lambda: self._command_gripper(False))
        gripper_buttons.addWidget(open_button)
        gripper_buttons.addWidget(close_button)
        layout.addLayout(gripper_buttons)

        self.pick_place_status_label = QtWidgets.QLabel(
            "Pick/place: waiting for node"
        )
        self.pick_place_status_label.setObjectName("hint")
        self.pick_place_status_label.setWordWrap(True)
        layout.addWidget(self.pick_place_status_label)
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
        box.setButtonSymbols(QtWidgets.QAbstractSpinBox.NoButtons)
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
        positions, _, _ = self.node.snapshot()
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
        positions, _, _ = self.node.snapshot()
        for row, position in zip(self.joint_rows, positions):
            row.set_value_deg(math.degrees(position))

    def _send_xy_increment(self, dx: float, dy: float):
        positions, _, _ = self.node.snapshot()
        x, y = self.arm.end_effector(positions)
        self.node.publish_xy_target(x + dx, y + dy)

    def _send_joint_increment(self, joint_index: int, delta: float):
        positions, _, _ = self.node.snapshot()
        positions[joint_index] += delta
        self.node.publish_joint_target(positions)

    def _start_pick_place(self, action: str) -> None:
        pick_xy = (self.pick_x.value(), self.pick_y.value())
        place_xy = (self.place_x.value(), self.place_y.value())
        self.node.start_pick_place(
            action,
            pick_xy,
            place_xy,
            lambda message: self.operation_result.emit(message),
        )

    def _command_gripper(self, opened: bool) -> None:
        if self.node.command_gripper(opened):
            self._notify(
                "Gripper opening" if opened else "Gripper closing"
            )
        else:
            self._notify("Gripper controller is not available")

    def _on_operation_result(self, message: str) -> None:
        self._notify(message)

    def _adjust_speed(self, direction: int) -> None:
        self.node.adjust_speed(direction)
        self._notify("Movement speed adjustment requested")

    def _read_pid_gains(self) -> None:
        self.node.get_pid_gains(
            lambda values, error: self.pid_values_received.emit(
                values, error
            )
        )

    def _on_pid_values_received(self, values, error) -> None:
        if error:
            self._notify(error)
            return
        for box, value in zip(
            (self.pid_kp, self.pid_ki, self.pid_kd, self.velocity_kp),
            values,
        ):
            box.setValue(value)
        self._notify("Loaded controller gains")

    def _apply_pid_gains(self) -> None:
        self.node.set_pid_gains(
            self.pid_kp.value(),
            self.pid_ki.value(),
            self.pid_kd.value(),
            self.velocity_kp.value(),
            lambda message: self.operation_result.emit(message),
        )

    def _cancel_motion(self) -> None:
        self.node.cancel_motion(
            lambda message: self.operation_result.emit(message)
        )

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
        positions, status, pick_place_status = self.node.snapshot()
        age = self.node.telemetry_age()
        self.pick_place_status_label.setText(pick_place_status)

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
            errors, timing, error_time, speed_scale = (
                self.node.control_telemetry()
            )
            if (
                error_time is not None
                and error_time != self._last_error_sample_time
            ):
                sample_time = max(0.0, error_time - self._started_at)
                self._error_history_times.append(sample_time)
                for history, error in zip(self._history_errors, errors):
                    history.append(math.degrees(error))
                self._last_error_sample_time = error_time
            error_times = list(self._error_history_times)
            for curve, history in zip(
                self.error_curves, self._history_errors
            ):
                curve.setData(error_times, list(history))
            self.error_plot.setXRange(
                max(0.0, elapsed - 40.0), max(elapsed, 5.0)
            )
            self.health_strip.update_timing(timing)
            self.speed_scale_label.setText(
                f"{speed_scale * 100:.0f}%"
            )
            if error_time is None:
                self.pid_feedback_tile.set_value("--")
            else:
                rms_error = math.sqrt(
                    sum(error * error for error in errors) / len(errors)
                )
                self.pid_feedback_tile.set_value(
                    f"{math.degrees(rms_error):.2f}°"
                )
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