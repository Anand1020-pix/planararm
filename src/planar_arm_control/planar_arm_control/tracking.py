"""Small, testable control helpers for trajectory tracking and timing."""

from collections import deque
import math


class PIDTracker:
    """PID effort controller with bounded integral and output."""

    def __init__(
        self,
        joint_count: int,
        kp: float,
        ki: float,
        kd: float,
        output_limits: list[float],
    ):
        values = [kp, ki, kd, *output_limits]
        if (
            joint_count <= 0
            or len(output_limits) != joint_count
            or not all(math.isfinite(value) for value in values)
            or any(limit <= 0.0 for limit in output_limits)
            or min(kp, ki, kd) < 0.0
        ):
            raise ValueError('PID gains and output limits are invalid')
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.output_limits = list(output_limits)
        self.integral = [0.0] * joint_count

    def set_gains(self, kp: float, ki: float, kd: float) -> None:
        """Update PID gains while preserving a bounded integral state."""
        if (
            not all(math.isfinite(value) for value in (kp, ki, kd))
            or min(kp, ki, kd) < 0.0
        ):
            raise ValueError('PID gains must be finite and non-negative')
        self.kp = kp
        self.ki = ki
        self.kd = kd

    def reset(self) -> None:
        """Clear accumulated integral error."""
        self.integral = [0.0] * len(self.integral)

    def compute(
        self,
        position_error: list[float],
        velocity_error: list[float],
        dt: float,
    ) -> list[float]:
        """Return bounded effort commands for the measured tracking error."""
        if (
            not math.isfinite(dt)
            or dt <= 0.0
            or len(position_error) != len(self.integral)
            or len(velocity_error) != len(self.integral)
            or not all(
                math.isfinite(value)
                for value in (*position_error, *velocity_error)
            )
        ):
            raise ValueError('PID inputs must be finite vectors and dt > 0')

        output = []
        for index, (error, velocity_delta, limit) in enumerate(
            zip(position_error, velocity_error, self.output_limits)
        ):
            if self.ki > 0.0:
                max_integral = limit / self.ki
                self.integral[index] = min(
                    max_integral,
                    max(-max_integral, self.integral[index] + error * dt),
                )
            value = (
                self.kp * error
                + self.ki * self.integral[index]
                + self.kd * velocity_delta
            )
            output.append(min(limit, max(-limit, value)))
        return output


def velocity_tracking_command(
    desired_velocity: list[float],
    position_error: list[float],
    kp: float,
    max_velocity: float,
) -> list[float]:
    """Add proportional position correction to trajectory velocity."""
    if (
        not math.isfinite(kp)
        or kp < 0.0
        or not math.isfinite(max_velocity)
        or max_velocity <= 0.0
        or len(desired_velocity) != len(position_error)
        or not all(
            math.isfinite(value)
            for value in (*desired_velocity, *position_error)
        )
    ):
        raise ValueError('velocity tracking inputs are invalid')
    return [
        min(max_velocity, max(-max_velocity, velocity + kp * error))
        for velocity, error in zip(desired_velocity, position_error)
    ]


class TimerJitter:
    """Keep a bounded history of control timer period errors."""

    def __init__(self, expected_period: float, history_size: int = 200):
        if (
            not math.isfinite(expected_period)
            or expected_period <= 0.0
            or history_size <= 0
        ):
            raise ValueError('timer period and history size must be positive')
        self.expected_period = expected_period
        self._samples: deque[float] = deque(maxlen=history_size)
        self._previous_time: float | None = None

    def observe(self, monotonic_time: float) -> float | None:
        """Record a monotonic callback time and return its signed jitter."""
        if not math.isfinite(monotonic_time):
            raise ValueError('timer observation must be finite')
        if self._previous_time is None:
            self._previous_time = monotonic_time
            return None
        interval = monotonic_time - self._previous_time
        if interval < 0.0:
            raise ValueError('timer observations must be monotonic')
        self._previous_time = monotonic_time
        jitter = interval - self.expected_period
        self._samples.append(jitter)
        return jitter

    def summary(self) -> tuple[float, float, float]:
        """Return latest, mean absolute, and maximum absolute jitter."""
        if not self._samples:
            return 0.0, 0.0, 0.0
        magnitudes = [abs(sample) for sample in self._samples]
        return (
            self._samples[-1],
            sum(magnitudes) / len(magnitudes),
            max(magnitudes),
        )
