#!/usr/bin/env python3
"""Orchestrate the Gazebo demo pick-and-place sequence."""

import math
import time

import rclpy
from geometry_msgs.msg import PointStamped
from rclpy.lifecycle import LifecycleNode, TransitionCallbackReturn
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from std_srvs.srv import Trigger
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from planar_arm_control.motion import (
    clamp_joint_limit_roundoff,
    make_trajectory,
    solve_xy_target,
    trajectory_respects_constraints,
)
from planar_arm_control.planar_arm import PlanarArm

LINK_LENGTHS = [3.0, 2.0, 1.5]
PICK_XY = (4.0, 1.95)
PLACE_XY = (-3.0, 3.0)
APPROACH_HEIGHT = 0.3
GRIPPER_OPEN_POSITION = 0.14
GRIPPER_CLOSED_POSITION = -0.02


def build_pick_waypoints(pick_xy=PICK_XY):
    """Return approach, contact, and lift targets for the pick operation."""
    return [
        (
            "Approaching pick",
            (pick_xy[0], pick_xy[1] + APPROACH_HEIGHT),
        ),
        ("Picking item", pick_xy),
        ("Lifting item", (pick_xy[0], pick_xy[1] + APPROACH_HEIGHT)),
    ]


def build_place_waypoints(place_xy=PLACE_XY):
    """Return carry and release targets for the place operation."""
    return [
        ("Carrying item", (place_xy[0], place_xy[1] + APPROACH_HEIGHT)),
        ("Placing item", place_xy),
    ]


def build_pick_place_waypoints(pick_xy=PICK_XY, place_xy=PLACE_XY):
    """Return the full approach, pick, carry, and place route."""
    return (
        build_pick_waypoints(pick_xy)
        + build_place_waypoints(place_xy)
    )


class PickPlaceNode(LifecycleNode):
    """Run safe waypoints and operate the physical parallel-jaw gripper."""

    def __init__(self):
        """Create task interfaces and initialize the sequence state."""
        super().__init__("pick_place_node")
        self.declare_parameter("autostart", True)
        self.arm = PlanarArm(LINK_LENGTHS)
        self.declare_parameter("pick_x", PICK_XY[0])
        self.declare_parameter("pick_y", PICK_XY[1])
        self.declare_parameter("place_x", PLACE_XY[0])
        self.declare_parameter("place_y", PLACE_XY[1])
        self._joint_positions: list[float] | None = None
        self._controller_status = ""
        self._running = False
        self._sequence_ready = False
        self._picked = False
        self._operation: str | None = None
        self._stage_index = 0
        self._waypoints: list[tuple[str, tuple[float, float]]] = []
        self._stage_started = False
        self._stage_complete = False
        self._stage_deadline = 0.0
        self._gripper_settle_until = 0.0
        self._gripper_action_sent = False

        self._target_publisher = self.create_publisher(
            PointStamped, "/target_pose", 10
        )
        self._gripper_publisher = self.create_publisher(
            JointTrajectory,
            "/gripper_controller/joint_trajectory",
            10,
        )
        self._status_publisher = self.create_lifecycle_publisher(
            String, "/pick_place/status", 10
        )
        self._controller_cancel_client = self.create_client(
            Trigger, "/controller/cancel"
        )
        self.create_subscription(
            JointState, "/joint_states", self._on_joint_state, 10
        )
        self.create_subscription(
            String, "/controller_status", self._on_controller_status, 10
        )
        self.create_service(Trigger, "/pick_place/pick", self._on_pick)
        self.create_service(Trigger, "/pick_place/place", self._on_place)
        self.create_service(Trigger, "/pick_place/release", self._on_release)
        self.create_service(Trigger, "/pick_place/cancel", self._on_cancel)
        self._active = False
        self._timer = self.create_timer(0.05, self._on_timer)
        self._timer.cancel()
        self._startup_gripper_pending = True

    def on_configure(self, state):
        """Configure the task node without starting its timer."""
        return super().on_configure(state)

    def on_activate(self, state):
        """Enable status publishing and task progression."""
        result = super().on_activate(state)
        if result != TransitionCallbackReturn.SUCCESS:
            return result
        self._active = True
        self._timer.reset()
        self._publish_status("Ready")
        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state):
        """Stop task progression and cancel any active arm motion."""
        self._timer.cancel()
        if self._running:
            self._cancel_operation("Pick/place operation cancelled")
            self._request_controller_cancel()
        self._active = False
        return super().on_deactivate(state)

    def on_shutdown(self, state):
        """Stop the task timer and cancel an active sequence."""
        if self._active:
            result = self.on_deactivate(state)
            if result != TransitionCallbackReturn.SUCCESS:
                return result
        else:
            self._timer.cancel()
        return super().on_shutdown(state)

    def _publish_status(self, text: str) -> None:
        if not self._active:
            return
        message = String()
        message.data = text
        self._status_publisher.publish(message)

    def _request_controller_cancel(self) -> bool:
        """Request cancellation from the controller if it is available."""
        if not self._controller_cancel_client.service_is_ready():
            return False
        future = self._controller_cancel_client.call_async(
            Trigger.Request()
        )
        future.add_done_callback(self._on_controller_cancel_response)
        return True

    def _on_controller_cancel_response(self, future) -> None:
        try:
            response = future.result()
        except Exception as error:
            self.get_logger().error(
                f"Controller cancellation request failed: {error}"
            )
            return
        if not response.success:
            self.get_logger().warning(response.message)

    def _cancel_operation(self, message: str) -> None:
        self._running = False
        self._sequence_ready = False
        self._stage_started = False
        self._stage_complete = False
        self._publish_status(message)

    def _on_cancel(self, request, response):
        """Stop pick/place progression and cancel the arm controller."""
        del request
        if not self._active:
            response.success = False
            response.message = "Pick/place lifecycle node is not active"
            return response
        controller_ready = self._request_controller_cancel()
        if self._running:
            self._cancel_operation("Pick/place operation cancelled")
        if not controller_ready:
            response.success = False
            response.message = "Controller cancel service is not available"
            return response
        response.success = True
        response.message = "Pick/place and arm motion cancellation requested"
        return response

    def _on_joint_state(self, message: JointState) -> None:
        by_name = dict(zip(message.name, message.position))
        names = ("joint1", "joint2", "joint3")
        if not all(name in by_name for name in names):
            return
        positions = [float(by_name[name]) for name in names]
        if all(math.isfinite(value) for value in positions):
            self._joint_positions = clamp_joint_limit_roundoff(
                self.arm, positions
            )

    def _target_from_parameters(
        self, target_name: str
    ) -> tuple[float, float] | None:
        x = float(self.get_parameter(f"{target_name}_x").value)
        y = float(self.get_parameter(f"{target_name}_y").value)
        if not math.isfinite(x) or not math.isfinite(y):
            return None
        return x, y

    def _command_gripper(self, opened: bool) -> None:
        position = (
            GRIPPER_OPEN_POSITION if opened else GRIPPER_CLOSED_POSITION
        )
        message = JointTrajectory()
        message.joint_names = ["left_finger_joint", "right_finger_joint"]
        point = JointTrajectoryPoint()
        point.positions = [position, position]
        point.time_from_start.sec = 1
        message.points = [point]
        self._gripper_publisher.publish(message)

    def _on_controller_status(self, message: String) -> None:
        self._controller_status = message.data
        if not self._running or not self._sequence_ready:
            return
        if message.data.startswith("Rejected:"):
            self._fail(f"Controller rejected a waypoint: {message.data}")
        elif message.data == "Trajectory cancelled; holding current position":
            self._cancel_operation(
                "Pick/place operation cancelled by controller"
            )
        elif message.data.startswith("Replacing active trajectory"):
            self._cancel_operation(
                "Pick/place operation interrupted by a new motion target"
            )
        elif message.data.startswith("Executing trajectory"):
            self._stage_started = True
        elif message.data == "Trajectory complete" and self._stage_started:
            self._stage_complete = True

    def _validate_waypoints(
        self,
        start: list[float],
        waypoints: list[tuple[str, tuple[float, float]]],
    ) -> None:
        current = list(start)
        for label, point in waypoints:
            goal = solve_xy_target(self.arm, point, current)
            trajectory = make_trajectory(current, goal, 2.0, 1.0)
            if not trajectory_respects_constraints(
                self.arm, trajectory, 0.005
            ):
                raise ValueError(f"{label} path violates arm constraints")
            current = goal

    def _on_pick(self, request, response):
        del request
        if self._picked:
            response.success = False
            response.message = "The item is already picked:call /pick_place/place"
            return response
        pick_xy = self._target_from_parameters("pick")
        if pick_xy is None:
            response.success = False
            response.message = "Pick coordinates must be finite"
            return response
        return self._begin_operation(
            response, "pick", build_pick_waypoints(pick_xy)
        )

    def _on_place(self, request, response):
        del request
        if not self._picked:
            response.success = False
            response.message = "Pick the item first with /pick_place/pick"
            return response
        place_xy = self._target_from_parameters("place")
        if place_xy is None:
            response.success = False
            response.message = "Place coordinates must be finite"
            return response
        return self._begin_operation(
            response, "place", build_place_waypoints(place_xy)
        )

    def _on_release(self, request, response):
        del request
        if not self._active:
            response.success = False
            response.message = "Pick/place lifecycle node is not active"
            return response
        if self._running:
            response.success = False
            response.message = "Wait for the current arm motion to finish"
            return response
        if not self._picked:
            response.success = False
            response.message = "There is no held item to release"
            return response
        if self._gripper_publisher.get_subscription_count() == 0:
            response.success = False
            response.message = "Gripper controller is not available"
            return response
        self._command_gripper(opened=True)
        self._picked = False
        self._publish_status("Gripper opened; item released")
        response.success = True
        response.message = "Gripper opened; item released"
        return response

    def _begin_operation(
        self,
        response,
        operation: str,
        waypoints: list[tuple[str, tuple[float, float]]],
    ):
        if not self._active:
            response.success = False
            response.message = "Pick/place lifecycle node is not active"
            return response
        if self._running:
            response.success = False
            response.message = "A pick/place operation is already running"
            return response
        if self._joint_positions is None:
            response.success = False
            response.message = "Waiting for joint-state feedback"
            return response
        if self._target_publisher.get_subscription_count() == 0:
            response.success = False
            response.message = "Controller is not subscribed to /target_pose"
            return response
        if self._gripper_publisher.get_subscription_count() == 0:
            response.success = False
            response.message = "Gripper controller is not available"
            return response
        if self._controller_status.startswith(
            ("Executing trajectory", "Replacing active trajectory")
        ):
            response.success = False
            response.message = "Controller is already executing a trajectory"
            return response
        try:
            self._validate_waypoints(self._joint_positions, waypoints)
        except (ValueError, FloatingPointError) as error:
            response.success = False
            response.message = f"{operation.capitalize()} path rejected: {error}"
            return response

        self._running = True
        self._operation = operation
        self._sequence_ready = True
        self._stage_index = 0
        self._waypoints = list(waypoints)
        self._send_current_waypoint()
        response.success = True
        response.message = f"{operation.capitalize()} operation started"
        return response

    def _send_current_waypoint(self) -> None:
        label, (x, y) = self._waypoints[self._stage_index]
        target = PointStamped()
        target.header.stamp = self.get_clock().now().to_msg()
        target.header.frame_id = "base_link"
        target.point.x = x
        target.point.z = y
        self._stage_started = False
        self._stage_complete = False
        self._gripper_action_sent = False
        self._stage_deadline = time.monotonic() + 90.0
        self._publish_status(label)
        self._target_publisher.publish(target)

    def _on_timer(self) -> None:
        if not self._active:
            return
        if (
            self._startup_gripper_pending
            and self._gripper_publisher.get_subscription_count() > 0
        ):
            self._command_gripper(opened=True)
            self._startup_gripper_pending = False
        if not self._running:
            return
        if not self._sequence_ready:
            return
        if time.monotonic() > self._stage_deadline:
            self._fail("Timed out waiting for the controller waypoint")
            return
        if not self._stage_complete:
            return

        needs_gripper_action = (
            self._operation == "pick" and self._stage_index == 1
        ) or (
            self._operation == "place"
            and self._stage_index == len(self._waypoints) - 1
        )
        if needs_gripper_action:
            if not self._gripper_action_sent:
                opened = self._operation == "place"
                self._command_gripper(opened=opened)
                self._gripper_action_sent = True
                self._gripper_settle_until = time.monotonic() + 1.0
                self._publish_status(
                    "Opening gripper" if opened else "Closing gripper"
                )
                return
            if time.monotonic() < self._gripper_settle_until:
                return

        if self._stage_index < len(self._waypoints) - 1:
            self._stage_index += 1
            self._send_current_waypoint()
            return

        if self._operation == "pick":
            self._picked = True
            self._running = False
            self._sequence_ready = False
            self._publish_status("Pick complete:ready to place")
        else:
            self._running = False
            self._sequence_ready = False
            self._picked = False
            self._publish_status("Place complete: gripper released item")

    def _fail(self, reason: str) -> None:
        self._running = False
        self._sequence_ready = False
        self._publish_status(f"Failed: {reason}")
        self.get_logger().error(reason)


def main(args=None):
    rclpy.init(args=args)
    node = PickPlaceNode()
    try:
        if bool(node.get_parameter("autostart").value):
            if node.trigger_configure() != TransitionCallbackReturn.SUCCESS:
                raise RuntimeError(
                    "Could not configure pick/place lifecycle"
                )
            if node.trigger_activate() != TransitionCallbackReturn.SUCCESS:
                raise RuntimeError(
                    "Could not activate pick/place lifecycle"
                )
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
