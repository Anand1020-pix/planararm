#!/usr/bin/env python3
"""Orchestrate the Gazebo demo pick-and-place sequence."""

import math
import time

import rclpy
from geometry_msgs.msg import PointStamped
from rclpy.node import Node
from ros_gz_interfaces.msg import Entity
from ros_gz_interfaces.srv import SetEntityPose
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from std_srvs.srv import Trigger

from planar_arm_control.motion import (
    clamp_joint_limit_roundoff,
    make_trajectory,
    solve_xy_target,
    trajectory_respects_constraints,
)
from planar_arm_control.planar_arm import PlanarArm

LINK_LENGTHS = [3.0, 2.0, 1.5]
BASE_HEIGHT = 0.2
PICK_XY = (4.0, 2.0)
PLACE_XY = (-3.0, 3.0)
APPROACH_HEIGHT = 0.3
OBJECT_NAME = "pick_item"
POSE_SERVICE = "/world/planar_arm_world/set_pose"


def build_pick_waypoints():
    """Return approach, contact, and lift targets for the pick operation."""
    return [
        (
            "Approaching pick",
            (PICK_XY[0], PICK_XY[1] + APPROACH_HEIGHT),
        ),
        ("Picking item", PICK_XY),
        ("Lifting item", (PICK_XY[0], PICK_XY[1] + APPROACH_HEIGHT)),
    ]


def build_place_waypoints():
    """Return carry and release targets for the place operation."""
    return [
        ("Carrying item", (PLACE_XY[0], PLACE_XY[1] + APPROACH_HEIGHT)),
        ("Placing item", PLACE_XY),
    ]


def build_pick_place_waypoints():
    """Return the full approach, pick, carry, and place route."""
    return build_pick_waypoints() + build_place_waypoints()


def tool_planar_position(
    arm: PlanarArm, joint_positions: list[float]
) -> tuple[float, float]:
    """Return measured tool XY for attaching or releasing the item."""
    x, y = arm.end_effector(joint_positions)
    return float(x), float(y)


class PickPlaceNode(Node):
    """Run safe waypoints and visually attach the Gazebo item."""

    def __init__(self):
        """Create task interfaces and initialize the sequence state."""
        super().__init__("pick_place_node")
        self.arm = PlanarArm(LINK_LENGTHS)
        self._joint_positions: list[float] | None = None
        self._controller_status = ""
        self._running = False
        self._sequence_ready = False
        self._attached = False
        self._picked = False
        self._operation: str | None = None
        self._stage_index = 0
        self._stage_started = False
        self._stage_complete = False
        self._stage_deadline = 0.0
        self._last_object_update = 0.0
        self._pose_request_pending = False
        self._queued_item_pose: tuple[float, float] | None = None

        self._target_publisher = self.create_publisher(
            PointStamped, "/target_pose", 10
        )
        self._status_publisher = self.create_publisher(
            String, "/pick_place/status", 10
        )
        self.create_subscription(
            JointState, "/joint_states", self._on_joint_state, 10
        )
        self.create_subscription(
            String, "/controller_status", self._on_controller_status, 10
        )
        self._pose_client = self.create_client(SetEntityPose, POSE_SERVICE)
        self.create_service(Trigger, "/pick_place/pick", self._on_pick)
        self.create_service(Trigger, "/pick_place/place", self._on_place)
        self.create_service(Trigger, "/pick_place/release", self._on_release)
        self._timer = self.create_timer(0.05, self._on_timer)
        self._publish_status("Ready")

    def _publish_status(self, text: str) -> None:
        message = String()
        message.data = text
        self._status_publisher.publish(message)

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

    def _on_controller_status(self, message: String) -> None:
        self._controller_status = message.data
        if not self._running or not self._sequence_ready:
            return
        if message.data.startswith("Rejected:"):
            self._fail(f"Controller rejected a waypoint: {message.data}")
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
            response.message = "The item is already picked; call /pick_place/place"
            return response
        return self._begin_operation(
            response, "pick", build_pick_waypoints(), reset_item=True
        )

    def _on_place(self, request, response):
        del request
        if not self._picked:
            response.success = False
            response.message = "Pick the item first with /pick_place/pick"
            return response
        return self._begin_operation(
            response, "place", build_place_waypoints(), reset_item=False
        )

    def _on_release(self, request, response):
        del request
        if self._running:
            response.success = False
            response.message = "Wait for the current arm motion to finish"
            return response
        if not self._picked or not self._attached:
            response.success = False
            response.message = "There is no held item to release"
            return response
        if self._joint_positions is None:
            response.success = False
            response.message = "Waiting for joint-state feedback"
            return response
        x, y = tool_planar_position(self.arm, self._joint_positions)
        self._release_item(x, y, "Item released")
        response.success = True
        response.message = f"Item released at ({x:.2f}, {y:.2f})"
        return response

    def _release_item(self, x: float, y: float, status: str) -> None:
        self._attached = False
        self._picked = False
        self._set_item_pose(x, y)
        self._publish_status(f"{status} at ({x:.2f}, {y:.2f})")

    def _begin_operation(
        self,
        response,
        operation: str,
        waypoints: list[tuple[str, tuple[float, float]]],
        reset_item: bool,
    ):
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
        if not self._pose_client.service_is_ready():
            response.success = False
            response.message = (
                f"Gazebo pose service is unavailable: {POSE_SERVICE}"
            )
            return response
        if self._controller_status.startswith("Executing trajectory"):
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
        self._sequence_ready = not reset_item
        self._attached = operation == "place"
        self._stage_index = 0
        if reset_item:
            self._publish_status("Resetting item at pick position")
            self._set_item_pose(*PICK_XY)
        else:
            self._send_current_waypoint()
        response.success = True
        response.message = f"{operation.capitalize()} operation started"
        return response

    def _set_item_pose(self, x: float, planar_y: float) -> None:
        if self._pose_request_pending:
            self._queued_item_pose = (float(x), float(planar_y))
            return
        request = SetEntityPose.Request()
        request.entity.name = OBJECT_NAME
        request.entity.type = Entity.MODEL
        request.pose.position.x = float(x)
        request.pose.position.y = 0.0
        request.pose.position.z = BASE_HEIGHT + float(planar_y)
        request.pose.orientation.w = 1.0
        self._pose_request_pending = True
        future = self._pose_client.call_async(request)
        future.add_done_callback(self._on_item_pose_response)

    def _on_item_pose_response(self, future) -> None:
        self._pose_request_pending = False
        try:
            result = future.result()
        except Exception as error:
            self._fail(f"Gazebo could not set the item pose: {error}")
            return
        if not result.success:
            self._fail("Gazebo rejected the item pose update")
            return
        if self._queued_item_pose is not None:
            x, y = self._queued_item_pose
            self._queued_item_pose = None
            self._set_item_pose(x, y)
            return
        if self._running and not self._sequence_ready:
            self._sequence_ready = True
            self._send_current_waypoint()

    def _send_current_waypoint(self) -> None:
        waypoints = (
            build_pick_waypoints()
            if self._operation == "pick"
            else build_place_waypoints()
        )
        label, (x, y) = waypoints[self._stage_index]
        target = PointStamped()
        target.header.stamp = self.get_clock().now().to_msg()
        target.header.frame_id = "base_link"
        target.point.x = x
        target.point.z = y
        self._stage_started = False
        self._stage_complete = False
        self._stage_deadline = time.monotonic() + 90.0
        self._publish_status(label)
        self._target_publisher.publish(target)

    def _update_attached_item(self) -> None:
        if (
            self._joint_positions is None
            or self._pose_request_pending
            or time.monotonic() - self._last_object_update < 0.02
        ):
            return
        x, y = tool_planar_position(self.arm, self._joint_positions)
        self._set_item_pose(x, y)
        self._last_object_update = time.monotonic()

    def _on_timer(self) -> None:
        if self._attached:
            self._update_attached_item()
        if not self._running:
            return
        if not self._sequence_ready:
            return
        if time.monotonic() > self._stage_deadline:
            self._fail("Timed out waiting for the controller waypoint")
            return
        if not self._stage_complete:
            return

        waypoints = (
            build_pick_waypoints()
            if self._operation == "pick"
            else build_place_waypoints()
        )
        if self._stage_index < len(waypoints) - 1:
            if self._operation == "pick" and self._stage_index == 1:
                self._attached = True
                self._last_object_update = 0.0
                self._update_attached_item()
            self._stage_index += 1
            self._send_current_waypoint()
            return

        if self._operation == "pick":
            self._picked = True
            self._running = False
            self._sequence_ready = False
            self._publish_status("Item lifted and picked; ready for place")
        else:
            self._running = False
            self._sequence_ready = False
            x, y = tool_planar_position(self.arm, self._joint_positions)
            self._release_item(x, y, "Place complete")

    def _fail(self, reason: str) -> None:
        self._running = False
        self._sequence_ready = False
        self._attached = self._picked
        self._publish_status(f"Failed: {reason}")
        self.get_logger().error(reason)


def main(args=None):
    rclpy.init(args=args)
    node = PickPlaceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
