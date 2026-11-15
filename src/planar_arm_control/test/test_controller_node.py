"""ROS lifecycle and cancellation tests for the arm controller."""

import math
import unittest

from planar_arm_control.controller_node import ControllerNode, HOME_POSITIONS

import rclpy
from rclpy.lifecycle import TransitionCallbackReturn
from rclpy.parameter import Parameter
from std_srvs.srv import Trigger


class TestControllerNode(unittest.TestCase):
    """Exercise controller lifecycle state and cancel behavior."""

    @classmethod
    def setUpClass(cls):
        if not rclpy.ok():
            rclpy.init()

    @classmethod
    def tearDownClass(cls):
        if rclpy.ok():
            rclpy.shutdown()

    def setUp(self):
        self.node = ControllerNode()
        self.assertEqual(
            self.node.trigger_configure(),
            TransitionCallbackReturn.SUCCESS,
        )
        self.assertEqual(
            self.node.trigger_activate(),
            TransitionCallbackReturn.SUCCESS,
        )

    def tearDown(self):
        self.node.destroy_node()

    def test_cancel_clears_trajectory_and_holds_measured_pose(self):
        goal = list(HOME_POSITIONS)
        goal[0] += math.radians(1.0)
        self.node._start_trajectory(list(HOME_POSITIONS), goal)
        self.assertIsNotNone(self.node._trajectory)

        response = self.node._on_cancel(
            Trigger.Request(), Trigger.Response()
        )

        self.assertTrue(response.success)
        self.assertIsNone(self.node._trajectory)
        self.assertEqual(self.node._hold_positions, list(HOME_POSITIONS))

    def test_lifecycle_deactivation_stops_control_timer(self):
        self.assertTrue(self.node._active)

        result = self.node.trigger_deactivate()

        self.assertEqual(result, TransitionCallbackReturn.SUCCESS)
        self.assertFalse(self.node._active)
        self.assertTrue(self.node._control_timer.is_canceled())

    def test_speed_scale_and_pid_gains_update_live(self):
        speed_result = self.node.set_parameters(
            [Parameter('speed_scale', value=1.5)]
        )[0]
        gain_result = self.node.set_parameters(
            [
                Parameter('effort_pid_kp', value=60.0),
                Parameter('effort_pid_ki', value=0.25),
                Parameter('effort_pid_kd', value=6.0),
            ]
        )

        self.assertTrue(speed_result.successful)
        self.assertEqual(self.node._speed_scale, 1.5)
        self.assertTrue(all(result.successful for result in gain_result))
        self.assertEqual(
            (
                self.node._effort_tracker.kp,
                self.node._effort_tracker.ki,
                self.node._effort_tracker.kd,
            ),
            (60.0, 0.25, 6.0),
        )

    def test_speed_and_pid_parameter_limits_are_enforced(self):
        speed_result = self.node.set_parameters(
            [Parameter('speed_scale', value=3.0)]
        )[0]
        gain_result = self.node.set_parameters(
            [Parameter('effort_pid_kp', value=501.0)]
        )[0]

        self.assertFalse(speed_result.successful)
        self.assertFalse(gain_result.successful)
        self.assertEqual(self.node._speed_scale, 1.0)
        self.assertEqual(self.node._effort_tracker.kp, 50.0)

    def test_speed_change_replans_active_path_from_feedback(self):
        goal = list(HOME_POSITIONS)
        goal[0] += math.radians(1.0)
        self.node._start_trajectory(list(HOME_POSITIONS), goal)
        original_trajectory = self.node._trajectory

        result = self.node.set_parameters(
            [Parameter('speed_scale', value=1.5)]
        )[0]

        self.assertTrue(result.successful)
        self.assertIsNot(self.node._trajectory, original_trajectory)
        self.assertEqual(self.node._trajectory_elapsed, 0.0)
        self.assertEqual(self.node._trajectory.goal, tuple(goal))


if __name__ == '__main__':
    unittest.main()
