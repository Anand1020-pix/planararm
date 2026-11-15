#!/usr/bin/env python3
"""ROS 2 trajectory planner and multi-mode controller for the planar arm."""

import math
import time

import rclpy
from geometry_msgs.msg import PointStamped
from rclpy.lifecycle import (
    LifecycleNode,
    TransitionCallbackReturn,
)
from rcl_interfaces.msg import ParameterDescriptor, SetParametersResult
from rclpy.parameter import Parameter
from sensor_msgs.msg import JointState, Joy
from std_msgs.msg import Float64, Float64MultiArray, Int32, String
from std_srvs.srv import Trigger
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
from planar_arm_control.tracking import (
    PIDTracker,
    TimerJitter,
    velocity_tracking_command,
)

LINK_LENGTHS = [3.0, 2.0, 1.5]
HOME_POSITIONS = [math.radians(75.0), math.radians(99.0), 0.0]
MIN_SPEED_SCALE = 0.25
MAX_SPEED_SCALE = 2.0
MAX_PID_KP = 500.0
MAX_PID_KI = 100.0
MAX_PID_KD = 100.0
MAX_VELOCITY_KP = 100.0


class ControllerNode(LifecycleNode):
    """Plan motion goals safely and drive the configured arm backend."""

    def __init__(self):
        """Create the ROS interfaces and fixed-rate control timer."""
        super().__init__("controller_node")
        self.arm = PlanarArm(LINK_LENGTHS)

        self.declare_parameter("publish_rate_hz", 50.0)
        self.declare_parameter("telemetry_rate_hz", 10.0)
        self.declare_parameter("trajectory_duration", 2.0)
        self.declare_parameter("max_joint_velocity", 1.0)
        self.declare_parameter(
            "command_mode",
            "position",
            descriptor=ParameterDescriptor(
                description=(
                    "Command interface selected at startup; changing it "
                    "requires restarting with a matching controller."
                ),
                read_only=True,
            ),
        )
        self.declare_parameter("velocity_tracking_kp", 4.0)
        self.declare_parameter("effort_pid_kp", 50.0)
        self.declare_parameter("effort_pid_ki", 0.0)
        self.declare_parameter("effort_pid_kd", 5.0)
        self.declare_parameter("speed_scale", 1.0)
        self.declare_parameter("speed_scale_step", 0.25)
        self.declare_parameter("effort_limits", [200.0, 100.0, 50.0])
        self.declare_parameter("home_positions", HOME_POSITIONS)
        self.declare_parameter("backend_mode", "simulated")
        self.declare_parameter("autostart", True)
        self.declare_parameter("joint_names", ["joint1", "joint2", "joint3"])
        self.declare_parameter("target_topic", "/target_pose")
        self.declare_parameter("joint_target_topic", "/joint_target")
        self.declare_parameter("joint_states_topic", "/joint_states")
        self.declare_parameter("joy_topic", "/joy")
        self.declare_parameter(
            "trajectory_command_topic",
            "/joint_trajectory_controller/joint_trajectory",
        )
        self.declare_parameter(
            "velocity_command_topic", "/joint_velocity_controller/commands"
        )
        self.declare_parameter(
            "effort_command_topic", "/joint_effort_controller/commands"
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
        self._telemetry_rate_hz = float(
            self.get_parameter("telemetry_rate_hz").value
        )
        if (
            not math.isfinite(self._telemetry_rate_hz)
            or self._telemetry_rate_hz <= 0.0
        ):
            raise ValueError(
                "telemetry_rate_hz must be finite and greater than zero"
            )
        self._trajectory_duration = float(
            self.get_parameter("trajectory_duration").value
        )
        self._max_joint_velocity = float(
            self.get_parameter("max_joint_velocity").value
        )
        self._command_mode = str(self.get_parameter("command_mode").value)
        if self._command_mode not in ("position", "velocity", "effort"):
            raise ValueError(
                "command_mode must be position, velocity, or effort"
            )
        self._velocity_tracking_kp = float(
            self.get_parameter("velocity_tracking_kp").value
        )
        if (
            not math.isfinite(self._velocity_tracking_kp)
            or not 0.0 <= self._velocity_tracking_kp <= MAX_VELOCITY_KP
        ):
            raise ValueError(
                "velocity_tracking_kp must be within its supported range"
            )
        self._speed_scale = float(self.get_parameter("speed_scale").value)
        self._speed_scale_step = float(
            self.get_parameter("speed_scale_step").value
        )
        if (
            not math.isfinite(self._speed_scale)
            or not MIN_SPEED_SCALE <= self._speed_scale <= MAX_SPEED_SCALE
            or not math.isfinite(self._speed_scale_step)
            or self._speed_scale_step <= 0.0
        ):
            raise ValueError(
                "speed_scale must be within its safe range and "
                "speed_scale_step must be positive"
            )
        effort_limits = [
            float(value)
            for value in self.get_parameter("effort_limits").value
        ]
        if len(effort_limits) != self.arm.n_joints:
            raise ValueError("effort_limits must match the joint count")
        try:
            self._effort_tracker = PIDTracker(
                self.arm.n_joints,
                float(self.get_parameter("effort_pid_kp").value),
                float(self.get_parameter("effort_pid_ki").value),
                float(self.get_parameter("effort_pid_kd").value),
                effort_limits,
            )
        except ValueError as error:
            raise ValueError(f"Invalid effort PID configuration: {error}")
        if (
            self._effort_tracker.kp > MAX_PID_KP
            or self._effort_tracker.ki > MAX_PID_KI
            or self._effort_tracker.kd > MAX_PID_KD
        ):
            raise ValueError("Effort PID gains exceed supported tuning bounds")
        self.add_on_set_parameters_callback(self._on_parameters)
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
            if self._command_mode != "position":
                raise ValueError(
                    "velocity and effort modes require backend_mode "
                    "'ros2_control'"
                )
            self._backend = SimulatedJointBackend(
                self._joint_names, self._home_positions
            )
            self._joint_state_publisher = self.create_lifecycle_publisher(
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
            self._velocity_publisher = self.create_publisher(
                Float64MultiArray,
                str(self.get_parameter("velocity_command_topic").value),
                10,
            )
            self._effort_publisher = self.create_publisher(
                Float64MultiArray,
                str(self.get_parameter("effort_command_topic").value),
                10,
            )
            self._backend = JointTrajectoryBackend(
                self._joint_names, self._command_publisher
            )
            self._mode_publisher = {
                "position": self._command_publisher,
                "velocity": self._velocity_publisher,
                "effort": self._effort_publisher,
            }[self._command_mode]
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

        if backend_mode == "simulated":
            self._velocity_publisher = None
            self._effort_publisher = None
            self._mode_publisher = None
        self._status_publisher = self.create_lifecycle_publisher(
            String, "/controller_status", 10
        )
        self._error_publisher = self.create_lifecycle_publisher(
            Float64MultiArray, "/controller_error", 10
        )
        self._timing_publisher = self.create_lifecycle_publisher(
            Float64MultiArray, "/controller_timing", 10
        )
        self._speed_scale_publisher = self.create_lifecycle_publisher(
            Float64, "/controller_speed_scale", 10
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
        self._speed_adjust_subscription = self.create_subscription(
            Int32, "/controller/speed_adjust", self._on_speed_adjust, 10
        )
        self._cancel_service = self.create_service(
            Trigger, "/controller/cancel", self._on_cancel
        )

        self._trajectory = None
        self._trajectory_elapsed = 0.0
        self._hold_positions: list[float] | None = None
        self._measured_velocities = [0.0] * self.arm.n_joints
        self._tracking_error = [0.0] * self.arm.n_joints
        self._joy_target_xy = None
        self._joy_last_update = time.monotonic()
        self._previous_joy_buttons: list[int] = []
        self._previous_positions = list(self._home_positions)
        self._startup_pose_pending = backend_mode == "ros2_control"
        self._startup_wait_reported = False
        self._waiting_for_feedback_reported = False
        self._active = False
        self._jitter = TimerJitter(self._period)
        self._last_control_time: float | None = None
        self._control_timer = self.create_timer(
            self._period, self._on_timer
        )
        self._telemetry_timer = self.create_timer(
            1.0 / self._telemetry_rate_hz, self._publish_telemetry
        )
        self._control_timer.cancel()
        self._telemetry_timer.cancel()
        self.get_logger().info(
            f"Controller configured with {backend_mode} backend, "
            f"{self._command_mode} commands at {self.publish_rate_hz:g} Hz."
        )

    def on_configure(self, state):
        """Configure control resources without publishing commands."""
        return super().on_configure(state)

    def on_activate(self, state):
        """Enable managed telemetry and start the multi-rate timers."""
        result = super().on_activate(state)
        if result != TransitionCallbackReturn.SUCCESS:
            return result
        self._active = True
        self._jitter = TimerJitter(self._period)
        self._last_control_time = None
        self._control_timer.reset()
        self._telemetry_timer.reset()
        self._status("Ready")
        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state):
        """Stop control timers and disable managed telemetry."""
        positions = self._current_positions()
        if positions is None:
            return TransitionCallbackReturn.FAILURE
        self._trajectory = None
        self._hold_positions = list(positions)
        if self._command_mode == "position":
            self._backend.command_positions(
                positions,
                self.get_clock().now().to_msg(),
                self._period * 2.0,
            )
        elif self._command_mode == "velocity":
            self._velocity_publisher.publish(
                Float64MultiArray(
                    data=[0.0] * self.arm.n_joints
                )
            )
        else:
            self._effort_publisher.publish(
                Float64MultiArray(
                    data=self._effort_tracker.compute(
                        [0.0] * self.arm.n_joints,
                        [0.0] * self.arm.n_joints,
                        self._period,
                    )
                )
            )
        self._active = False
        self._control_timer.cancel()
        self._telemetry_timer.cancel()
        self._effort_tracker.reset()
        return super().on_deactivate(state)

    def on_shutdown(self, state):
        """Stop timers and command a hold before lifecycle shutdown."""
        if self._active:
            result = self.on_deactivate(state)
            if result != TransitionCallbackReturn.SUCCESS:
                return result
        else:
            self._control_timer.cancel()
            self._telemetry_timer.cancel()
        return super().on_shutdown(state)

    def _status(self, message: str) -> None:
        if not self._active:
            return
        status = String()
        status.data = message
        self._status_publisher.publish(status)

    def _trajectory_for_speed(self, speed_scale: float):
        """Replan the active path from feedback for a changed speed scale."""
        if self._trajectory is None:
            return None
        start = self._current_positions()
        if start is None:
            raise ValueError("joint feedback is unavailable")
        goal = list(self._trajectory.goal)
        trajectory = make_trajectory(
            start,
            goal,
            max(self._trajectory.duration, self._trajectory_duration),
            self._max_joint_velocity / speed_scale,
        )
        if not trajectory_respects_constraints(
            self.arm, trajectory, self._period / 4.0
        ):
            raise ValueError("updated-speed path violates arm constraints")
        return trajectory

    def _on_parameters(self, parameters):
        """Validate live controller tuning updates before applying them."""
        proposed = {
            "effort_pid_kp": self._effort_tracker.kp,
            "effort_pid_ki": self._effort_tracker.ki,
            "effort_pid_kd": self._effort_tracker.kd,
            "velocity_tracking_kp": self._velocity_tracking_kp,
            "speed_scale": self._speed_scale,
        }
        for parameter in parameters:
            if parameter.name in proposed:
                try:
                    proposed[parameter.name] = float(parameter.value)
                except (TypeError, ValueError):
                    return SetParametersResult(
                        successful=False,
                        reason=f"{parameter.name} must be a number",
                    )

        if (
            not MIN_SPEED_SCALE
            <= proposed["speed_scale"]
            <= MAX_SPEED_SCALE
        ):
            return SetParametersResult(
                successful=False,
                reason=(
                    f"speed_scale must be between {MIN_SPEED_SCALE} "
                    f"and {MAX_SPEED_SCALE}"
                ),
            )
        if (
            not math.isfinite(proposed["velocity_tracking_kp"])
            or not 0.0
            <= proposed["velocity_tracking_kp"]
            <= MAX_VELOCITY_KP
        ):
            return SetParametersResult(
                successful=False,
                reason=(
                    "velocity_tracking_kp must be between 0 and "
                    f"{MAX_VELOCITY_KP}"
                ),
            )
        gains = (
            proposed["effort_pid_kp"],
            proposed["effort_pid_ki"],
            proposed["effort_pid_kd"],
        )
        if (
            not all(math.isfinite(gain) and gain >= 0.0 for gain in gains)
            or gains[0] > MAX_PID_KP
            or gains[1] > MAX_PID_KI
            or gains[2] > MAX_PID_KD
        ):
            return SetParametersResult(
                successful=False,
                reason=(
                    "PID gains must be non-negative and within supported "
                    "tuning bounds"
                ),
            )

        speed_changed = proposed["speed_scale"] != self._speed_scale
        replacement_trajectory = None
        if speed_changed:
            try:
                replacement_trajectory = self._trajectory_for_speed(
                    proposed["speed_scale"]
                )
            except ValueError as error:
                return SetParametersResult(
                    successful=False,
                    reason=f"Speed update rejected: {error}",
                )

        old_gains = (
            self._effort_tracker.kp,
            self._effort_tracker.ki,
            self._effort_tracker.kd,
        )
        self._effort_tracker.set_gains(*gains)
        if gains != old_gains:
            self._effort_tracker.reset()
        self._velocity_tracking_kp = proposed["velocity_tracking_kp"]
        self._speed_scale = proposed["speed_scale"]
        if replacement_trajectory is not None:
            self._trajectory = replacement_trajectory
            self._trajectory_elapsed = 0.0
            self._hold_positions = list(replacement_trajectory.goal)
            self._effort_tracker.reset()
        if speed_changed:
            self._status(
                f"Movement speed set to {self._speed_scale * 100:.0f}%"
            )
        return SetParametersResult(successful=True)

    def _on_speed_adjust(self, message: Int32) -> None:
        """Change trajectory speed by one bounded step."""
        if not self._active:
            self.get_logger().warning(
                "Ignoring speed adjustment while controller is inactive"
            )
            return
        if message.data not in (-1, 1):
            self._reject("Speed adjustment must be -1 or +1")
            return
        updated = min(
            MAX_SPEED_SCALE,
            max(
                MIN_SPEED_SCALE,
                self._speed_scale
                + message.data * self._speed_scale_step,
            ),
        )
        result = self.set_parameters(
            [Parameter("speed_scale", value=updated)]
        )[0]
        if not result.successful:
            self._reject(f"Speed adjustment rejected: {result.reason}")
            return
        self._status(f"Movement speed set to {self._speed_scale * 100:.0f}%")

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
        by_name = dict(zip(message.name, message.velocity))
        if all(name in by_name for name in self._joint_names):
            velocities = [
                float(by_name[name]) for name in self._joint_names
            ]
            if all(math.isfinite(value) for value in velocities):
                self._measured_velocities = velocities

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
        if not self._active:
            self._reject("Controller lifecycle node is not active")
            return
        if (
            self._mode_publisher is not None
            and self._mode_publisher.get_subscription_count() == 0
        ):
            self._reject(f"{self._command_mode} controller is not active")
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
                self._max_joint_velocity / self._speed_scale,
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

        replacing = self._trajectory is not None
        self._trajectory = trajectory
        self._hold_positions = list(goal)
        self._effort_tracker.reset()
        self._trajectory_elapsed = 0.0
        prefix = (
            "Replacing active trajectory with"
            if replacing
            else "Executing trajectory"
        )
        self._status(f"{prefix} ({trajectory.duration:.2f} s)")

    def _on_cancel(self, request, response):
        """Cancel the active path and hold the latest measured pose."""
        del request
        if not self._active:
            response.success = False
            response.message = "Controller lifecycle node is not active"
            return response
        positions = self._current_positions()
        if positions is None:
            response.success = False
            response.message = "Cannot cancel safely without joint feedback"
            return response
        was_moving = self._trajectory is not None
        self._trajectory = None
        self._hold_positions = list(positions)
        self._effort_tracker.reset()
        self._joy_target_xy = None
        self._status(
            "Trajectory cancelled; holding current position"
            if was_moving
            else "Holding current position"
        )
        response.success = True
        response.message = "Motion cancelled; holding current position"
        return response

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
        if not self._active:
            return
        now_monotonic = time.monotonic()
        self._jitter.observe(now_monotonic)
        now_ros = self._clock_seconds()
        dt = (
            self._period
            if self._last_control_time is None
            else max(0.0, now_ros - self._last_control_time)
        )
        self._last_control_time = now_ros
        positions = self._current_positions()
        if positions is None:
            return

        if self._startup_pose_pending:
            if (
                self._mode_publisher is not None
                and self._mode_publisher.get_subscription_count() > 0
            ):
                self._startup_pose_pending = False
                self._on_joint_target(
                    Float64MultiArray(data=self._home_positions)
                )
                positions = self._current_positions()
                if positions is None:
                    return
            elif not self._startup_wait_reported:
                self._status(
                    f"Waiting for {self._command_mode} controller"
                )
                self._startup_wait_reported = True

        desired_velocity = [0.0] * self.arm.n_joints
        if self._trajectory is not None:
            self._trajectory_elapsed += dt * self._speed_scale
            elapsed = self._trajectory_elapsed
            desired, velocity, _ = self._trajectory.sample_kinematics(
                elapsed
            )
            positions = desired.tolist()
            desired_velocity = (
                velocity * self._speed_scale
            ).tolist()
        elif self._hold_positions is not None:
            positions = list(self._hold_positions)

        measured_positions = self._current_positions()
        if measured_positions is None:
            return
        self._tracking_error = [
            desired - measured
            for desired, measured in zip(positions, measured_positions)
        ]

        if self._command_mode == "position":
            self._backend.command_positions(
                positions,
                self.get_clock().now().to_msg(),
                self._period * 2.0,
            )
        elif self._command_mode == "velocity":
            command = velocity_tracking_command(
                desired_velocity,
                self._tracking_error,
                self._velocity_tracking_kp,
                self._max_joint_velocity,
            )
            self._velocity_publisher.publish(
                Float64MultiArray(data=command)
            )
        else:
            velocity_error = [
                desired - measured
                for desired, measured in zip(
                    desired_velocity, self._measured_velocities
                )
            ]
            command = self._effort_tracker.compute(
                self._tracking_error, velocity_error, max(dt, 1e-6)
            )
            self._effort_publisher.publish(Float64MultiArray(data=command))

        if self._trajectory is not None:
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

    def _publish_telemetry(self) -> None:
        """Publish the latest tracking error and control timer jitter."""
        if not self._active:
            return
        self._error_publisher.publish(
            Float64MultiArray(data=list(self._tracking_error))
        )
        self._timing_publisher.publish(
            Float64MultiArray(data=list(self._jitter.summary()))
        )
        self._speed_scale_publisher.publish(
            Float64(data=self._speed_scale)
        )


def main(args=None):
    """Initialize ROS and run the controller node."""
    rclpy.init(args=args)
    node = ControllerNode()
    try:
        if bool(node.get_parameter("autostart").value):
            if node.trigger_configure() != TransitionCallbackReturn.SUCCESS:
                raise RuntimeError("Could not configure controller lifecycle")
            if node.trigger_activate() != TransitionCallbackReturn.SUCCESS:
                raise RuntimeError("Could not activate controller lifecycle")
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
