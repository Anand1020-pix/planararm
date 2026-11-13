#!/usr/bin/env python3
"""ROS 2 target planner and position-command controller for the planar arm."""

import math
import time

import rclpy
from geometry_msgs.msg import PointStamped
from rclpy.node import Node
from sensor_msgs.msg import JointState, Joy
from std_msgs.msg import Float64MultiArray, String
from trajectory_msgs.msg import JointTrajectory

from planar_arm_control.backends import (
    JointTrajectoryBackend,
    SimulatedJointBackend,
)
from planar_arm_control.motion import (
    clamp_joint_limit_roundoff,
    make_trajectory,
    solve_xy_target,
    trajectory_respects_constraints,
)
from planar_arm_control.planar_arm import PlanarArm

LINK_LENGTHS = [3.0, 2.0, 1.5]
HOME_POSITIONS = [math.radians(75.0), math.radians(99.0), 0.0]


class ControllerNode(Node):
    """Plan motion goals safely and drive the configured arm backend."""

    def __init__(self):
        """Create the ROS interfaces and fixed-rate control timer."""
        super().__init__("controller_node")
        self.arm = PlanarArm(LINK_LENGTHS)

        self.declare_parameter("publish_rate_hz", 50.0)
        self.declare_parameter("trajectory_duration", 2.0)
        self.declare_parameter("max_joint_velocity", 1.0)
        self.declare_parameter("home_positions", HOME_POSITIONS)
        self.declare_parameter("backend_mode", "simulated")
        self.declare_parameter("joint_names", ["joint1", "joint2", "joint3"])
        self.declare_parameter("target_topic", "/target_pose")
        self.declare_parameter("joint_target_topic", "/joint_target")
        self.declare_parameter("joint_states_topic", "/joint_states")
        self.declare_parameter("joy_topic", "/joy")
        self.declare_parameter(
            "trajectory_command_topic",
            "/joint_trajectory_controller/joint_trajectory",
        )
        self.declare_parameter("joystick_xy_speed", 0.5)
        self.declare_parameter("joystick_y_axis_sign", -1.0)
        self.declare_parameter("joystick_joint_step_rad", 0.05)
        self.declare_parameter("joystick_trajectory_duration", 0.15)

        self.publish_rate_hz = float(
            self.get_parameter("publish_rate_hz").value
        )
        if (
            not math.isfinite(self.publish_rate_hz)
            or self.publish_rate_hz <= 0.0
        ):
            raise ValueError(
                "publish_rate_hz must be finite and greater than zero"
            )
        self._period = 1.0 / self.publish_rate_hz
        self._trajectory_duration = float(
            self.get_parameter("trajectory_duration").value
        )
        self._max_joint_velocity = float(
            self.get_parameter("max_joint_velocity").value
        )
        self._home_positions = [
            float(value)
            for value in self.get_parameter("home_positions").value
        ]
        self._joystick_xy_speed = float(
            self.get_parameter("joystick_xy_speed").value
        )
        self._joystick_y_axis_sign = float(
            self.get_parameter("joystick_y_axis_sign").value
        )
        self._joystick_joint_step_rad = float(
            self.get_parameter("joystick_joint_step_rad").value
        )
        self._joystick_trajectory_duration = float(
            self.get_parameter("joystick_trajectory_duration").value
        )
        positive_parameters = {
            "trajectory_duration": self._trajectory_duration,
            "max_joint_velocity": self._max_joint_velocity,
            "joystick_xy_speed": self._joystick_xy_speed,
            "joystick_joint_step_rad": self._joystick_joint_step_rad,
            "joystick_trajectory_duration": self._joystick_trajectory_duration,
        }
        for name, value in positive_parameters.items():
            if not math.isfinite(value) or value <= 0.0:
                raise ValueError(
                    f"{name} must be finite and greater than zero"
                )
        if self._joystick_y_axis_sign not in (-1.0, 1.0):
            raise ValueError("joystick_y_axis_sign must be either -1 or 1")

        self._joint_names = list(self.get_parameter("joint_names").value)
        if (
            len(self._joint_names) != self.arm.n_joints
            or len(set(self._joint_names)) != self.arm.n_joints
            or not all(self._joint_names)
        ):
            raise ValueError(
                "joint_names must be distinct and match arm joint count"
            )
        if (
            len(self._home_positions) != self.arm.n_joints
            or not all(math.isfinite(value) for value in self._home_positions)
            or not self.arm.within_joint_limits(self._home_positions)
            or not self.arm.arm_above_base(self._home_positions)
        ):
            raise ValueError(
                "home_positions must be a valid, ground-safe joint pose"
            )

        backend_mode = str(self.get_parameter("backend_mode").value)
        if backend_mode == "simulated":
            self._backend = SimulatedJointBackend(
                self._joint_names, self._home_positions
            )
            self._joint_state_publisher = self.create_publisher(
                JointState,
                str(self.get_parameter("joint_states_topic").value),
                10,
            )
            self._joint_state_subscription = None
            self._command_publisher = None
        elif backend_mode == "ros2_control":
            self._command_publisher = self.create_publisher(
                JointTrajectory,
                str(self.get_parameter("trajectory_command_topic").value),
                10,
            )
            self._backend = JointTrajectoryBackend(
                self._joint_names, self._command_publisher
            )
            self._joint_state_publisher = None
            self._joint_state_subscription = self.create_subscription(
                JointState,
                str(self.get_parameter("joint_states_topic").value),
                self._on_joint_state,
                10,
            )
        else:
            raise ValueError(
                "backend_mode must be either 'simulated' or 'ros2_control'"
            )

        self._status_publisher = self.create_publisher(
            String, "/controller_status", 10
        )
        self._target_subscription = self.create_subscription(
            PointStamped,
            str(self.get_parameter("target_topic").value),
            self._on_xy_target,
            10,
        )
        self._joint_target_subscription = self.create_subscription(
            Float64MultiArray,
            str(self.get_parameter("joint_target_topic").value),
            self._on_joint_target,
            10,
        )
        self._joy_subscription = self.create_subscription(
            Joy,
            str(self.get_parameter("joy_topic").value),
            self._on_joy,
            10,
        )

        self._trajectory = None
        self._trajectory_started_at = 0.0
        self._joy_target_xy = None
        self._joy_last_update = time.monotonic()
        self._previous_joy_buttons: list[int] = []
        self._previous_positions = list(self._home_positions)
        self._startup_pose_pending = backend_mode == "ros2_control"
        self._startup_wait_reported = False
        self._waiting_for_feedback_reported = False
        self._timer = self.create_timer(self._period, self._on_timer)
        self._status("Ready")
        self.get_logger().info(
            f"Controller ready with {backend_mode} backend at "
            f"{self.publish_rate_hz:g} Hz."
        )

    def _status(self, message: str) -> None:
        status = String()
        status.data = message
        self._status_publisher.publish(status)

    def _current_positions(self) -> list[float] | None:
        try:
            positions = self._backend.current_positions()
        except RuntimeError as error:
            if not self._waiting_for_feedback_reported:
                self.get_logger().info(str(error))
                self._status("Waiting for joint-state feedback")
                self._waiting_for_feedback_reported = True
            return None
        positions = clamp_joint_limit_roundoff(self.arm, positions)
        if self._waiting_for_feedback_reported:
            self._status("Ready")
        self._waiting_for_feedback_reported = False
        return positions

    def _on_joint_state(self, message: JointState) -> None:
        self._backend.update_feedback(message.name, message.position)

    def _on_xy_target(
        self, message: PointStamped, from_joystick: bool = False
    ) -> None:
        if not from_joystick:
            self._joy_target_xy = None
        if message.header.frame_id not in ("", "base_link"):
            self._reject(
                f"Expected target coordinates in base_link, got "
                f"'{message.header.frame_id}'"
            )
            return
        start = self._current_positions()
        if start is None:
            self._reject("Cannot plan until joint-state feedback is available")
            return
        try:
            goal = solve_xy_target(
                self.arm,
                (message.point.x, message.point.z),
                start,
            )
        except (ValueError, FloatingPointError) as error:
            self._reject(f"XY target rejected: {error}")
            return
        self._start_trajectory(
            start,
            goal,
            self._joystick_trajectory_duration
            if from_joystick
            else None,
        )

    def _on_joint_target(
        self,
        message: Float64MultiArray,
        from_joystick: bool = False,
    ) -> None:
        self._joy_target_xy = None
        if len(message.data) != self.arm.n_joints:
            self._reject(
                "Joint target must contain exactly "
                f"{self.arm.n_joints} positions"
            )
            return
        start = self._current_positions()
        if start is None:
            self._reject("Cannot plan until joint-state feedback is available")
            return
        goal = [float(position) for position in message.data]
        if not all(math.isfinite(position) for position in goal):
            self._reject("Joint target positions must all be finite")
            return
        if not self.arm.within_joint_limits(goal):
            self._reject(
                "Joint target is outside the configured joint limits"
            )
            return
        if not self.arm.arm_above_base(goal):
            self._reject("Joint target violates the ground constraint")
            return
        self._start_trajectory(
            start,
            goal,
            self._joystick_trajectory_duration
            if from_joystick
            else None,
        )

    def _start_trajectory(
        self,
        start: list[float],
        goal: list[float],
        minimum_duration: float | None = None,
    ) -> None:
        if (
            self._command_publisher is not None
            and self._command_publisher.get_subscription_count() == 0
        ):
            self._reject("Joint trajectory controller is not active")
            return
        if not all(math.isfinite(value) for value in start):
            self._reject("Current joint state contains non-finite positions")
            return
        if (
            not self.arm.within_joint_limits(start)
            or not self.arm.arm_above_base(start)
        ):
            self._reject("Current joint state violates the arm constraints")
            return
        try:
            trajectory = make_trajectory(
                start,
                goal,
                (
                    self._trajectory_duration
                    if minimum_duration is None
                    else minimum_duration
                ),
                self._max_joint_velocity,
            )
            if not trajectory_respects_constraints(
                self.arm, trajectory, self._period / 4.0
            ):
                self._reject(
                    "Planned path violates joint limits or ground constraint"
                )
                return
        except ValueError as error:
            self._reject(f"Trajectory rejected: {error}")
            return

        self._trajectory = trajectory
        self._trajectory_started_at = self._clock_seconds()
        self._status(f"Executing trajectory ({trajectory.duration:.2f} s)")

    def _clock_seconds(self) -> float:
        return self.get_clock().now().nanoseconds / 1_000_000_000.0

    def _on_joy(self, message: Joy) -> None:
        start = self._current_positions()
        if start is None:
            return

        buttons = [int(value) for value in message.buttons]
        rising = [
            index
            for index, value in enumerate(buttons)
            if value
            and (
                index >= len(self._previous_joy_buttons)
                or not self._previous_joy_buttons[index]
            )
        ]
        self._previous_joy_buttons = buttons

        joint_delta = [0.0] * self.arm.n_joints
        step = self._joystick_joint_step_rad
        for joint_index, (positive_button, negative_button) in enumerate(
            ((0, 1), (2, 3), (4, 5))
        ):
            if positive_button in rising:
                joint_delta[joint_index] += step
            if negative_button in rising:
                joint_delta[joint_index] -= step
        if any(joint_delta):
            goal = [
                position + delta
                for position, delta in zip(start, joint_delta)
            ]
            self._on_joint_target(
                Float64MultiArray(data=goal),
                from_joystick=True,
            )
            return

        axes = message.axes
        axis_x = axes[0] if len(axes) > 0 else 0.0
        axis_y = axes[1] if len(axes) > 1 else 0.0
        if not math.isfinite(axis_x) or not math.isfinite(axis_y):
            self._reject("Joystick axes must be finite")
            return
        axis_y *= self._joystick_y_axis_sign
        now = time.monotonic()
        elapsed = min(0.1, max(0.0, now - self._joy_last_update))
        self._joy_last_update = now
        if abs(axis_x) < 0.08 and abs(axis_y) < 0.08:
            if self._trajectory is None:
                self._joy_target_xy = None
            return

        if self._joy_target_xy is None:
            self._joy_target_xy = self.arm.end_effector(start)
        speed = self._joystick_xy_speed
        self._joy_target_xy = (
            self._joy_target_xy[0] + float(axis_x) * speed * elapsed,
            self._joy_target_xy[1] + float(axis_y) * speed * elapsed,
        )
        target = PointStamped()
        target.header.frame_id = "base_link"
        target.point.x, target.point.z = self._joy_target_xy
        self._on_xy_target(target, from_joystick=True)

    def _reject(self, message: str) -> None:
        self.get_logger().warning(message)
        self._status(f"Rejected: {message}")

    def _on_timer(self) -> None:
        positions = self._current_positions()
        if positions is None:
            return

        if self._startup_pose_pending:
            if (
                self._command_publisher is not None
                and self._command_publisher.get_subscription_count() > 0
            ):
                self._startup_pose_pending = False
                self._on_joint_target(
                    Float64MultiArray(data=self._home_positions)
                )
                positions = self._current_positions()
                if positions is None:
                    return
            elif not self._startup_wait_reported:
                self._status("Waiting for joint trajectory controller")
                self._startup_wait_reported = True

        if self._trajectory is not None:
            elapsed = self._clock_seconds() - self._trajectory_started_at
            positions = self._trajectory.sample(elapsed).tolist()
            self._backend.command_positions(
                positions, self.get_clock().now().to_msg(), self._period * 2.0
            )
            if elapsed >= self._trajectory.duration:
                self._trajectory = None
                self._status("Trajectory complete")

        if self._joint_state_publisher is not None:
            message = JointState()
            message.header.stamp = self.get_clock().now().to_msg()
            message.name = self._joint_names
            message.position = positions
            message.velocity = [
                (current - previous) / self._period
                for current, previous in zip(
                    positions, self._previous_positions
                )
            ]
            self._joint_state_publisher.publish(message)
        self._previous_positions = list(positions)


def main(args=None):
    """Initialize ROS and run the controller node."""
    rclpy.init(args=args)
    node = ControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
