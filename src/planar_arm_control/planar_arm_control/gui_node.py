#!/usr/bin/env python3
"""PyQt5 teleoperation and live telemetry window for the planar arm."""

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


class GuiNode(Node):
    """Receive arm telemetry and publish operator motion goals."""

    def __init__(self):
        """Create target publishers and thread-safe telemetry subscriptions."""
        super().__init__("gui_node")
        self.arm = PlanarArm(LINK_LENGTHS)
        self._state_lock = threading.Lock()
        self._positions = [0.0, 0.0, 0.0]
        self._status = "Waiting for controller"
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

    def _on_status(self, message: String) -> None:
        with self._state_lock:
            self._status = message.data

    def snapshot(self):
        """Return the latest positions and controller status consistently."""
        with self._state_lock:
            return list(self._positions), self._status

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


class ArmWindow(QtWidgets.QMainWindow):
    """Display arm telemetry and provide keyboard, XY, and joint controls."""

    def __init__(self, node: GuiNode):
        """Build the plots and operator controls around a ROS node."""
        super().__init__()
        self.node = node
        self.arm = node.arm
        self.setWindowTitle("Planar Arm Teleoperation")
        self.resize(1000, 720)

        central = QtWidgets.QWidget()
        self.setCentralWidget(central)
        layout = QtWidgets.QGridLayout(central)

        self.arm_plot = pg.PlotWidget(title="Live arm pose")
        self.arm_plot.setAspectLocked(True)
        self.arm_plot.showGrid(x=True, y=True, alpha=0.25)
        self.arm_plot.setLabel("bottom", "X")
        self.arm_plot.setLabel("left", "Y")
        reach = sum(LINK_LENGTHS) + 0.5
        self.arm_plot.setXRange(-reach, reach)
        self.arm_plot.setYRange(-0.5, reach)
        self.arm_curve = self.arm_plot.plot(
            [], [], pen=pg.mkPen("#4da6ff", width=5), symbol="o", symbolSize=9
        )
        layout.addWidget(self.arm_plot, 0, 0, 5, 1)

        target_group = QtWidgets.QGroupBox("Planar target (base_link X/Z)")
        target_layout = QtWidgets.QFormLayout(target_group)
        self.x_target = self._coordinate_box()
        self.y_target = self._coordinate_box()
        target_layout.addRow("X:", self.x_target)
        target_layout.addRow("Y:", self.y_target)
        move_xy = QtWidgets.QPushButton("Move to XY")
        move_xy.clicked.connect(self._send_xy_target)
        target_layout.addRow(move_xy)
        layout.addWidget(target_group, 0, 1)

        joint_group = QtWidgets.QGroupBox("Joint target (degrees)")
        joint_layout = QtWidgets.QFormLayout(joint_group)
        self.joint_targets = []
        for index, (lower, upper) in enumerate(self.arm.joint_limits):
            box = QtWidgets.QDoubleSpinBox()
            box.setRange(
                math.degrees(float(lower)),
                math.degrees(float(upper)),
            )
            box.setDecimals(1)
            box.setSingleStep(1.0)
            self.joint_targets.append(box)
            joint_layout.addRow(f"Joint {index + 1}:", box)
        joint_buttons = QtWidgets.QHBoxLayout()
        move_joints = QtWidgets.QPushButton("Move joints")
        move_joints.clicked.connect(self._send_joint_target)
        set_current = QtWidgets.QPushButton("Use current")
        set_current.clicked.connect(self._set_joint_targets_to_current)
        joint_buttons.addWidget(move_joints)
        joint_buttons.addWidget(set_current)
        joint_layout.addRow(joint_buttons)
        layout.addWidget(joint_group, 1, 1)

        self.status_label = QtWidgets.QLabel("Waiting for controller")
        self.status_label.setWordWrap(True)
        layout.addWidget(self.status_label, 2, 1)
        self.pose_label = QtWidgets.QLabel("Joint state unavailable")
        self.pose_label.setWordWrap(True)
        layout.addWidget(self.pose_label, 3, 1)

        help_label = QtWidgets.QLabel(
            "Keyboard: W/A/S/D or arrow keys jog XY; 1/2, 3/4, 5/6 jog "
            "joints 1/2/3 down/up. Gamepad: left stick jogs XY; buttons "
            "0/1, 2/3, 4/5 jog each joint down/up."
        )
        help_label.setWordWrap(True)
        layout.addWidget(help_label, 4, 1)

        self.plot = pg.PlotWidget(title="Joint telemetry")
        self.plot.showGrid(x=True, y=True, alpha=0.25)
        self.plot.setLabel("bottom", "Time", units="s")
        self.plot.setLabel("left", "Joint angle", units="deg")
        self.plot.addLegend()
        self.joint_curves = [
            self.plot.plot(
                [],
                [],
                pen=pg.intColor(index, hues=3),
                name=f"Joint {index + 1}",
            )
            for index in range(3)
        ]
        layout.addWidget(self.plot, 5, 0, 1, 2)

        self._history_times = deque(maxlen=400)
        self._history_positions = [deque(maxlen=400) for _ in range(3)]
        self._started_at = time.monotonic()
        self._last_plot_update = 0.0
        self._timer = QtCore.QTimer(self)
        self._timer.timeout.connect(self._refresh)
        self._timer.start(50)

    @staticmethod
    def _coordinate_box():
        box = QtWidgets.QDoubleSpinBox()
        box.setRange(-6.5, 6.5)
        box.setDecimals(3)
        box.setSingleStep(0.1)
        return box

    def _send_xy_target(self):
        self.node.publish_xy_target(
            self.x_target.value(),
            self.y_target.value(),
        )

    def _send_joint_target(self):
        self.node.publish_joint_target(
            [math.radians(box.value()) for box in self.joint_targets]
        )

    def _set_joint_targets_to_current(self):
        positions, _ = self.node.snapshot()
        for box, position in zip(self.joint_targets, positions):
            box.setValue(math.degrees(position))

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
            (QtWidgets.QLineEdit, QtWidgets.QAbstractSpinBox),
        ):
            super().keyPressEvent(event)
            return

        key = event.key()
        xy_keys = {
            QtCore.Qt.Key_A: (-0.1, 0.0),
            QtCore.Qt.Key_Left: (-0.1, 0.0),
            QtCore.Qt.Key_D: (0.1, 0.0),
            QtCore.Qt.Key_Right: (0.1, 0.0),
            QtCore.Qt.Key_S: (0.0, -0.1),
            QtCore.Qt.Key_Down: (0.0, -0.1),
            QtCore.Qt.Key_W: (0.0, 0.1),
            QtCore.Qt.Key_Up: (0.0, 0.1),
        }
        joint_keys = {
            QtCore.Qt.Key_1: (0, -0.05),
            QtCore.Qt.Key_2: (0, 0.05),
            QtCore.Qt.Key_3: (1, -0.05),
            QtCore.Qt.Key_4: (1, 0.05),
            QtCore.Qt.Key_5: (2, -0.05),
            QtCore.Qt.Key_6: (2, 0.05),
        }
        if key in xy_keys:
            self._send_xy_increment(*xy_keys[key])
            event.accept()
        elif key in joint_keys:
            self._send_joint_increment(*joint_keys[key])
            event.accept()
        else:
            super().keyPressEvent(event)

    def _refresh(self):
        positions, status = self.node.snapshot()
        points = self.arm.forward_kinematics(positions)
        self.arm_curve.setData(
            [point[0] for point in points],
            [point[1] for point in points],
        )
        end_x, end_y = points[-1]
        self.status_label.setText(f"Controller: {status}")
        self.pose_label.setText(
            f"End effector: X={end_x:.3f}, Y={end_y:.3f}\n"
            + "Joint angles: "
            + ", ".join(f"{math.degrees(value):.1f}°" for value in positions)
        )

        now = time.monotonic()
        if now - self._last_plot_update >= 0.1:
            elapsed = now - self._started_at
            self._history_times.append(elapsed)
            for history, position in zip(self._history_positions, positions):
                history.append(math.degrees(position))
            times = list(self._history_times)
            for curve, history in zip(
                self.joint_curves, self._history_positions
            ):
                curve.setData(times, list(history))
            self._last_plot_update = now


def main(args=None):
    """Run Qt in the main thread and ROS callbacks in a background executor."""
    rclpy.init(args=args)
    node = GuiNode()
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    ros_thread = threading.Thread(target=executor.spin, daemon=True)
    application = QtWidgets.QApplication([])
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
