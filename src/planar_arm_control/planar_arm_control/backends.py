"""Position-command backends shared by the planner/controller node."""

from abc import ABC, abstractmethod

from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


class JointCommandBackend(ABC):
    """Actuator boundary used by the trajectory planner."""

    @abstractmethod
    def current_positions(self) -> list[float]:
        """Return the most recent joint positions in radians."""

    @abstractmethod
    def command_positions(
        self,
        positions: list[float],
        stamp,
        interpolation_period: float,
    ) -> None:
        """Send a position setpoint without changing the planning logic."""

    def update_feedback(
        self,
        names: list[str],
        positions: list[float],
    ) -> None:
        """Update measured positions when the backend has external feedback."""


class SimulatedJointBackend(JointCommandBackend):
    """In-process arm state used when no hardware backend is attached."""

    def __init__(
        self,
        joint_names: list[str],
        initial_positions: list[float],
    ):
        """Create an in-memory position state for the configured joints."""
        self._joint_names = list(joint_names)
        self._positions = list(initial_positions)

    def current_positions(self) -> list[float]:
        """Return the most recently commanded positions."""
        return list(self._positions)

    def command_positions(
        self,
        positions: list[float],
        stamp,
        interpolation_period: float,
    ) -> None:
        """Update the in-memory arm state to the sampled setpoint."""
        self._positions = list(positions)


class JointTrajectoryBackend(JointCommandBackend):
    """Adapter for ros2_control's JointTrajectoryController topic interface."""

    def __init__(self, joint_names: list[str], publisher):
        """Use a ROS publisher for commands and joint feedback."""
        self._joint_names = list(joint_names)
        self._publisher = publisher
        self._positions: list[float] | None = None

    def current_positions(self) -> list[float]:
        """Return the latest measured positions or report missing feedback."""
        if self._positions is None:
            raise RuntimeError(
                "waiting for joint-state feedback from ros2_control"
            )
        return list(self._positions)

    def update_feedback(
        self,
        names: list[str],
        positions: list[float],
    ) -> None:
        """Store complete feedback in the backend's configured joint order."""
        by_name = dict(zip(names, positions))
        if all(name in by_name for name in self._joint_names):
            self._positions = [
                float(by_name[name]) for name in self._joint_names
            ]

    def command_positions(
        self,
        positions: list[float],
        stamp,
        interpolation_period: float,
    ) -> None:
        """Publish the next position setpoint as a short trajectory segment."""
        message = JointTrajectory()
        # A zero stamp starts this rolling setpoint when it is received.
        message.header.stamp.sec = 0
        message.header.stamp.nanosec = 0
        message.joint_names = self._joint_names

        point = JointTrajectoryPoint()
        point.positions = list(positions)
        point.time_from_start.sec = int(interpolation_period)
        point.time_from_start.nanosec = int(
            (interpolation_period - int(interpolation_period)) * 1_000_000_000
        )
        message.points = [point]
        self._publisher.publish(message)
