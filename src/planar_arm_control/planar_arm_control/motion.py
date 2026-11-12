"""Kinematic target validation and time-parameterized joint trajectories."""

from dataclasses import dataclass
import math

import numpy as np


@dataclass(frozen=True)
class QuinticTrajectory:
    """Quintic motion with zero velocity and acceleration at both endpoints."""

    start: tuple[float, ...]
    goal: tuple[float, ...]
    duration: float

    def sample(self, elapsed: float) -> np.ndarray:
        """Return joint positions at elapsed seconds, clamped to the path."""
        progress = min(1.0, max(0.0, elapsed / self.duration))
        blend = 10.0 * progress**3 - 15.0 * progress**4 + 6.0 * progress**5
        start = np.asarray(self.start, dtype=float)
        goal = np.asarray(self.goal, dtype=float)
        return start + blend * (goal - start)


def make_trajectory(
    start: list[float] | np.ndarray,
    goal: list[float] | np.ndarray,
    minimum_duration: float,
    max_joint_velocity: float,
) -> QuinticTrajectory:
    """Create a quintic path long enough to respect the velocity limit."""
    if not math.isfinite(minimum_duration) or minimum_duration <= 0.0:
        raise ValueError(
            "trajectory_duration must be finite and greater than zero"
        )
    if not math.isfinite(max_joint_velocity) or max_joint_velocity <= 0.0:
        raise ValueError(
            "max_joint_velocity must be finite and greater than zero"
        )

    start_array = np.asarray(start, dtype=float)
    goal_array = np.asarray(goal, dtype=float)
    if (
        start_array.ndim != 1
        or start_array.shape != goal_array.shape
        or not np.all(np.isfinite(start_array))
        or not np.all(np.isfinite(goal_array))
    ):
        raise ValueError(
            "joint positions must be finite vectors of equal length"
        )

    peak_velocity_factor = 1.875
    required_duration = (
        peak_velocity_factor * float(np.max(np.abs(goal_array - start_array)))
        / max_joint_velocity
    )
    duration = max(minimum_duration, required_duration)
    return QuinticTrajectory(
        tuple(float(value) for value in start_array),
        tuple(float(value) for value in goal_array),
        duration,
    )


def clamp_joint_limit_roundoff(
    arm,
    joint_positions: list[float],
    tolerance: float = 1e-9,
) -> list[float]:
    """Clamp only floating-point noise just beyond configured joint limits."""
    positions = np.asarray(joint_positions, dtype=float)
    limits = np.asarray(arm.joint_limits, dtype=float)
    if (
        positions.shape != (arm.n_joints,)
        or limits.shape != (arm.n_joints, 2)
        or not np.all(np.isfinite(positions))
    ):
        return list(joint_positions)

    clamped = positions.copy()
    for index, (lower, upper) in enumerate(limits):
        if lower - tolerance <= clamped[index] < lower:
            clamped[index] = lower
        elif upper < clamped[index] <= upper + tolerance:
            clamped[index] = upper
    return clamped.tolist()


def trajectory_respects_constraints(
    arm,
    trajectory: QuinticTrajectory,
    sample_period: float,
) -> bool:
    """Check sampled trajectory points against arm constraints."""
    if not math.isfinite(sample_period) or sample_period <= 0.0:
        raise ValueError(
            "validation sample period must be finite and positive"
        )

    steps = max(1, math.ceil(trajectory.duration / sample_period))
    for step in range(steps + 1):
        joint_positions = trajectory.sample(trajectory.duration * step / steps)
        if not arm.within_joint_limits(joint_positions):
            return False
        if not arm.arm_above_base(joint_positions):
            return False
    return True


def solve_xy_target(
    arm,
    target_xy: tuple[float, float],
    initial_guess,
) -> list[float]:
    """Solve an XY goal and reject unreachable or invalid joint solutions."""
    target = np.asarray(target_xy, dtype=float)
    if target.shape != (2,) or not np.all(np.isfinite(target)):
        raise ValueError("target must contain finite x and y coordinates")
    if float(np.linalg.norm(target)) > sum(arm.link_lengths):
        raise ValueError("target is outside the arm's reachable radius")

    solution = np.asarray(
        arm.inverse_kinematics(target, initial_guess=initial_guess),
        dtype=float,
    )
    if solution.shape != (arm.n_joints,) or not np.all(np.isfinite(solution)):
        raise ValueError(
            "inverse kinematics did not return a finite joint solution"
        )
    if not arm.within_joint_limits(solution):
        raise ValueError(
            "inverse kinematics returned a solution outside joint limits"
        )
    if not arm.arm_above_base(solution):
        raise ValueError(
            "inverse kinematics returned a solution below the "
            "ground constraint"
        )
    return [float(value) for value in solution]
